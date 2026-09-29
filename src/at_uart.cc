#include "at_uart.h"
#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <chrono>

#define TAG "AtUart"
// 通道等待超过该阈值视为明显排队/竞争，打印告警辅助定位 GPS/数据通信互相卡顿的问题
#define CHANNEL_CONTENTION_WARN_US 150000


// AtUart 构造函数实现
AtUart::AtUart(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dtr_pin, uart_port_t uart_num)
    : tx_pin_(tx_pin), rx_pin_(rx_pin), dtr_pin_(dtr_pin), uart_num_(uart_num),
      baud_rate_(115200), initialized_(false),
      event_task_handle_(nullptr), event_queue_handle_(nullptr), event_group_handle_(nullptr) {
}

AtUart::~AtUart() {
    if (event_task_handle_) {
        vTaskDelete(event_task_handle_);
    }
    if (event_group_handle_) {
        vEventGroupDelete(event_group_handle_);
    }
    if (initialized_) {
        uart_driver_delete(uart_num_);
    }
}

void AtUart::Initialize(size_t rx_buf_size, size_t task_stack) {
    if (initialized_) {
        return;
    }

    event_group_handle_ = xEventGroupCreate();
    if (!event_group_handle_) {
        ESP_LOGE(TAG, "创建事件组失败");
        return;
    }

    uart_config_t uart_config = {};
    uart_config.baud_rate = baud_rate_;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_LOGI(TAG, "init uart %d, rx_buf=%u, task_stack=%u", uart_num_,
             static_cast<unsigned>(rx_buf_size), static_cast<unsigned>(task_stack));

    ESP_ERROR_CHECK(uart_driver_install(uart_num_, rx_buf_size, 0, 20, &event_queue_handle_, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (dtr_pin_ != GPIO_NUM_NC) {
        gpio_config_t config = {};
        config.pin_bit_mask = (1ULL << dtr_pin_);
        config.mode = GPIO_MODE_OUTPUT;
        config.pull_up_en = GPIO_PULLUP_DISABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&config);
        gpio_set_level(dtr_pin_, 0);
    }

    xTaskCreate([](void* arg) {
        auto ml307_at_modem = (AtUart*)arg;
        ml307_at_modem->EventTask();
        vTaskDelete(NULL);
    }, "modem_event", task_stack, this, 15, &event_task_handle_);

    xTaskCreate([](void* arg) {
        auto ml307_at_modem = (AtUart*)arg;
        ml307_at_modem->ReceiveTask();
        vTaskDelete(NULL);
    }, "modem_receive", task_stack, this, 10, &receive_task_handle_);
    initialized_ = true;
}

void AtUart::EventTaskWrapper(void* arg) {
    auto uart = static_cast<AtUart*>(arg);
    uart->EventTask();
    vTaskDelete(nullptr);
}

void AtUart::EventTask() {
    uart_event_t event;
    uint32_t interval_read_bytes = 0;
    uint32_t interval_read_count = 0;
    uint32_t interval_data_events = 0;
    uint32_t interval_overflows = 0;
    size_t interval_max_read = 0;
    size_t interval_max_hw_buffered = 0;
    size_t interval_max_sw_buffered = 0;
    UBaseType_t interval_max_event_queue_depth = 0;
    int64_t interval_max_data_event_gap_us = 0;
    int64_t last_data_event_us = 0;
    int64_t last_log_us = esp_timer_get_time();
    while (true) {
        if (xQueueReceive(event_queue_handle_, &event, portMAX_DELAY) == pdTRUE) {
            switch (event.type)
            {
            case UART_DATA: {
                int64_t data_event_us = esp_timer_get_time();
                if (last_data_event_us != 0) {
                    interval_max_data_event_gap_us = std::max(
                        interval_max_data_event_gap_us, data_event_us - last_data_event_us);
                }
                last_data_event_us = data_event_us;
                ++interval_data_events;
                // 立即读取数据，避免FIFO溢出
                // 循环读取直到没有数据，确保快速清空硬件FIFO
                bool has_data = false;
                
                while (true) {
                    size_t available;
                    uart_get_buffered_data_len(uart_num_, &available);
                    if (available == 0) {
                        break;
                    }
                    interval_max_hw_buffered = std::max(interval_max_hw_buffered, available);
                    
                    // 限制单次读取大小，避免阻塞
                    size_t read_size = std::min(available, size_t(2048));
                    
                    // 直接读取到rx_buffer_，减少中间步骤，提高速度
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        size_t old_size = rx_buffer_.size();
                        rx_buffer_.resize(old_size + read_size);
                        uart_read_bytes(uart_num_, &rx_buffer_[old_size], read_size, 0);
                        interval_max_sw_buffered = std::max(interval_max_sw_buffered, rx_buffer_.size());
                    }
                    interval_read_bytes += read_size;
                    ++interval_read_count;
                    interval_max_read = std::max(interval_max_read, read_size);
                    has_data = true;
                }
                
                // 如果有数据被读取，通知接收任务处理
                if (has_data) {
                    xEventGroupSetBits(event_group_handle_, AT_EVENT_DATA_AVAILABLE);
                }
                break;
            }
            case UART_BREAK:
                ESP_LOGI(TAG, "break");
                break;
            case UART_BUFFER_FULL: {
                ++interval_overflows;
                size_t available;
                uart_get_buffered_data_len(uart_num_, &available);
                ESP_LOGE(TAG, "[溢出] UART buffer full! rx_buffer: %zu bytes, UART剩余: %zu bytes", 
                         rx_buffer_.size(), available);
                break;
            }
            case UART_FIFO_OVF: {
                ++interval_overflows;
                // FIFO溢出是瞬时事件，可能发生在数据到达的瞬间
                // 即使循环读取，如果数据到达太快，在两次UART_DATA事件之间也可能溢出
                // 在溢出事件中也尝试读取剩余数据，减少数据丢失
                size_t available;
                uart_get_buffered_data_len(uart_num_, &available);
                ESP_LOGW(TAG, "[溢出] FIFO overflow detected! rx_buffer: %zu bytes, UART剩余: %zu bytes", 
                         rx_buffer_.size(), available);
                
                // 尝试读取剩余数据（如果有的话）
                if (available > 0) {
                    uint8_t temp_buf[2048];
                    size_t read_size = std::min(available, size_t(sizeof(temp_buf)));
                    uart_read_bytes(uart_num_, temp_buf, read_size, 0);
                    
                    std::lock_guard<std::mutex> lock(mutex_);
                    rx_buffer_.resize(rx_buffer_.size() + read_size);
                    memcpy(&rx_buffer_[rx_buffer_.size() - read_size], temp_buf, read_size);
                    
                    // 通知接收任务处理数据
                    xEventGroupSetBits(event_group_handle_, AT_EVENT_DATA_AVAILABLE);
                    ESP_LOGW(TAG, "[溢出] 已读取 %zu bytes 剩余数据", read_size);
                }
                
                HandleUrc("FIFO_OVERFLOW", {});
                break;
            }
            default:
                ESP_LOGE(TAG, "unknown event type: %d", event.type);
                break;
            }
            interval_max_event_queue_depth = std::max(
                interval_max_event_queue_depth, uxQueueMessagesWaiting(event_queue_handle_));

            int64_t now_us = esp_timer_get_time();
            int64_t elapsed_us = now_us - last_log_us;
            if (elapsed_us >= 1000000) {
                size_t hw_buffered = 0;
                size_t sw_buffered = 0;
                uint32_t actual_baud = 0;
                uart_get_baudrate(uart_num_, &actual_baud);
                uart_get_buffered_data_len(uart_num_, &hw_buffered);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    sw_buffered = rx_buffer_.size();
                }
                double bytes_per_second = interval_read_bytes * 1000000.0 / elapsed_us;
                ESP_LOGD(TAG,
                         "[RX] uart=%d baud=%u %.0f B/s (%u B/%u ms), events=%u reads=%u, "
                         "event_gap_max=%u ms queue_max=%u max_read=%u "
                         "hw_now/max=%u/%u sw_now/max=%u/%u overflow=%u",
                         uart_num_, static_cast<unsigned>(actual_baud), bytes_per_second,
                         interval_read_bytes,
                         static_cast<unsigned>(elapsed_us / 1000), interval_data_events,
                         interval_read_count,
                         static_cast<unsigned>(interval_max_data_event_gap_us / 1000),
                         static_cast<unsigned>(interval_max_event_queue_depth),
                         static_cast<unsigned>(interval_max_read),
                         static_cast<unsigned>(hw_buffered),
                         static_cast<unsigned>(interval_max_hw_buffered),
                         static_cast<unsigned>(sw_buffered),
                         static_cast<unsigned>(interval_max_sw_buffered),
                         interval_overflows);
                interval_read_bytes = 0;
                interval_read_count = 0;
                interval_data_events = 0;
                interval_overflows = 0;
                interval_max_read = 0;
                interval_max_hw_buffered = 0;
                interval_max_sw_buffered = sw_buffered;
                interval_max_event_queue_depth = 0;
                interval_max_data_event_gap_us = 0;
                last_log_us = now_us;
            }
        }
    }
}

void AtUart::ReceiveTask() {
    uint32_t interval_parse_count = 0;
    uint32_t interval_process_count = 0;
    uint32_t interval_parse_limit_hits = 0;
    size_t interval_max_sw_before = 0;
    size_t interval_max_sw_after = 0;
    size_t interval_max_hw_after = 0;
    int64_t interval_max_process_us = 0;
    int64_t last_log_us = esp_timer_get_time();
    
    while (true) {
        auto bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_DATA_AVAILABLE, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & AT_EVENT_DATA_AVAILABLE) {
            int64_t process_start_us = esp_timer_get_time();
            size_t rx_buffer_before = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                rx_buffer_before = rx_buffer_.size();
            }
            
            // 限制解析次数，避免长时间阻塞
            int parse_count = 0;
            const int MAX_PARSE_PER_LOOP = 50;  // 每次最多解析50条响应
            while (ParseResponse() && ++parse_count < MAX_PARSE_PER_LOOP) {}
            
            size_t sw_buffer_after = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                sw_buffer_after = rx_buffer_.size();
            }
            size_t hw_buffer_after = 0;
            uart_get_buffered_data_len(uart_num_, &hw_buffer_after);

            ++interval_process_count;
            interval_parse_count += parse_count;
            interval_max_sw_before = std::max(interval_max_sw_before, rx_buffer_before);
            interval_max_sw_after = std::max(interval_max_sw_after, sw_buffer_after);
            interval_max_hw_after = std::max(interval_max_hw_after, hw_buffer_after);
            interval_max_process_us = std::max(interval_max_process_us,
                                                esp_timer_get_time() - process_start_us);
            if (parse_count >= MAX_PARSE_PER_LOOP) {
                ++interval_parse_limit_hits;
            }

            // 保持原有调度策略：仅在硬件 UART 仍有数据时再次唤醒。
            if (hw_buffer_after > 0) {
                ESP_LOGD(TAG, "[消费] UART还有 %zu bytes 未读，继续处理", hw_buffer_after);
                xEventGroupSetBits(event_group_handle_, AT_EVENT_DATA_AVAILABLE);
            }

            int64_t now_us = esp_timer_get_time();
            int64_t elapsed_us = now_us - last_log_us;
            if (elapsed_us >= 1000000) {
                ESP_LOGD(TAG,
                         "[Parse] uart=%d parsed=%u wakes=%u limit_hits=%u, "
                         "sw_before_max=%u sw_after_now/max=%u/%u hw_after_max=%u "
                         "loop_max=%u us",
                         uart_num_, interval_parse_count, interval_process_count,
                         interval_parse_limit_hits,
                         static_cast<unsigned>(interval_max_sw_before),
                         static_cast<unsigned>(sw_buffer_after),
                         static_cast<unsigned>(interval_max_sw_after),
                         static_cast<unsigned>(interval_max_hw_after),
                         static_cast<unsigned>(interval_max_process_us));
                interval_parse_count = 0;
                interval_process_count = 0;
                interval_parse_limit_hits = 0;
                interval_max_sw_before = sw_buffer_after;
                interval_max_sw_after = sw_buffer_after;
                interval_max_hw_after = 0;
                interval_max_process_us = 0;
                last_log_us = now_us;
            }
        }
    }
}

static bool is_number(const std::string& s) {
    return !s.empty() && std::all_of(s.begin(), s.end(), ::isdigit) && s.length() < 10;
}

bool AtUart::ParseResponse() {
    std::string command, values;
    std::vector<AtArgumentValue> parsed_arguments;

    // 加锁保护rx_buffer_，避免与EventTask冲突
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (rx_buffer_.empty()) {
        return false;
    }
    
    if (wait_for_response_ && rx_buffer_[0] == '>') {
        rx_buffer_.erase(0, 1);
        lock.unlock();  // 释放锁后再设置事件
        xEventGroupSetBits(event_group_handle_, AT_EVENT_COMMAND_DONE);
        return true;
    }

    // ========== 特殊处理：+QMTRECV 基于 payload_length 精确解析 ==========
    // 格式：+QMTRECV: <client_idx>,<msgid>,"<topic>",<payload_length>,<payload>\r\n
    // payload 可能包含 \r\n、逗号、引号等任意字符，必须用 payload_length 定位结尾
    if (rx_buffer_.size() >= 10 && rx_buffer_.compare(0, 10, "+QMTRECV: ") == 0) {
        auto topic_open = rx_buffer_.find('"', 10);
        if (topic_open != std::string::npos) {
            auto topic_close = rx_buffer_.find('"', topic_open + 1);
            if (topic_close == std::string::npos) return false; // topic 不完整

            if (topic_close + 2 < rx_buffer_.size() && rx_buffer_[topic_close + 1] == ',') {
                size_t len_start = topic_close + 2;
                auto len_comma = rx_buffer_.find(',', len_start);
                if (len_comma != std::string::npos) {
                    std::string len_str = rx_buffer_.substr(len_start, len_comma - len_start);
                    if (is_number(len_str)) {
                        int payload_length = std::stoi(len_str);
                        size_t payload_start = len_comma + 1;
                        bool quoted = (payload_start < rx_buffer_.size() && rx_buffer_[payload_start] == '"');
                        if (quoted) payload_start++;
                        size_t payload_end = payload_start + payload_length;
                        size_t line_end = payload_end + (quoted ? 1 : 0) + 2; // 闭合引号 + \r\n
                        if (rx_buffer_.size() < line_end) return false; // 数据不完整，等

                        // 解析各字段
                        std::string header = rx_buffer_.substr(10, topic_open - 10);
                        std::string topic = rx_buffer_.substr(topic_open + 1, topic_close - topic_open - 1);
                        std::string payload = rx_buffer_.substr(payload_start, payload_length);

                        std::istringstream hss(header);
                        std::string h_item;
                        while (std::getline(hss, h_item, ',')) {
                            if (!h_item.empty() && is_number(h_item)) {
                                AtArgumentValue arg;
                                arg.type = AtArgumentValue::Type::Int;
                                arg.int_value = std::stoi(h_item);
                                parsed_arguments.push_back(arg);
                            }
                        }
                        AtArgumentValue topic_arg;
                        topic_arg.type = AtArgumentValue::Type::String;
                        topic_arg.string_value = std::move(topic);
                        parsed_arguments.push_back(topic_arg);
                        AtArgumentValue len_arg;
                        len_arg.type = AtArgumentValue::Type::Int;
                        len_arg.int_value = payload_length;
                        parsed_arguments.push_back(len_arg);
                        AtArgumentValue payload_arg;
                        payload_arg.type = AtArgumentValue::Type::String;
                        payload_arg.string_value = std::move(payload);
                        parsed_arguments.push_back(payload_arg);

                        ESP_LOGI(TAG, "<< +QMTRECV payload_length=%d", payload_length);
                        rx_buffer_.erase(0, line_end);
                        command = "QMTRECV";
                        lock.unlock();
                        HandleUrc(command, parsed_arguments);
                        return true;
                    }
                }
            }
        }
        // 没有引号或没有 payload_length → 走通用解析
    }

    // ========== 特殊处理：缓存模式 AT+QIRD 返回的原始数据 ==========
    // viewmode=0 格式：+QIRD: <read_actual_length>\r\n<raw_data>\r\n
    // raw_data 可能包含 \r\n，必须按 read_actual_length 精确定位。
    if (rx_buffer_.size() >= 7 && rx_buffer_.compare(0, 7, "+QIRD: ") == 0) {
        auto header_end = rx_buffer_.find("\r\n", 7);
        if (header_end == std::string::npos) return false;

        std::string len_str = rx_buffer_.substr(7, header_end - 7);
        if (is_number(len_str)) {
            int data_length = std::stoi(len_str);
            size_t data_start = header_end + 2;
            size_t response_end = data_start;
            if (data_length > 0) {
                response_end = data_start + static_cast<size_t>(data_length) + 2;
                if (rx_buffer_.size() < response_end) return false;
            }

            std::string raw_data;
            if (data_length > 0) {
                raw_data = rx_buffer_.substr(data_start, data_length);
            }
            rx_buffer_.erase(0, response_end);

            AtArgumentValue arg_len;
            arg_len.type = AtArgumentValue::Type::Int;
            arg_len.int_value = data_length;
            parsed_arguments.push_back(arg_len);
            if (data_length > 0) {
                AtArgumentValue arg_data;
                arg_data.type = AtArgumentValue::Type::String;
                arg_data.string_value = std::move(raw_data);
                parsed_arguments.push_back(std::move(arg_data));
            }

            static uint32_t qird_count = 0;
            static uint32_t qird_bytes = 0;
            static uint32_t qird_zero_count = 0;
            static size_t qird_max_chunk = 0;
            static int64_t qird_max_gap_us = 0;
            static int64_t qird_max_callback_us = 0;
            static int64_t qird_last_data_us = 0;
            static int64_t qird_last_log_us = esp_timer_get_time();

            int64_t data_now_us = esp_timer_get_time();
            ++qird_count;
            if (data_length > 0) {
                qird_bytes += static_cast<uint32_t>(data_length);
                qird_max_chunk = std::max(qird_max_chunk, static_cast<size_t>(data_length));
                if (qird_last_data_us != 0) {
                    qird_max_gap_us = std::max(qird_max_gap_us,
                                               data_now_us - qird_last_data_us);
                }
                qird_last_data_us = data_now_us;
            } else {
                ++qird_zero_count;
            }

            lock.unlock();
            int64_t callback_start_us = esp_timer_get_time();
            HandleUrc("QIRD", parsed_arguments);
            qird_max_callback_us = std::max(qird_max_callback_us,
                                             esp_timer_get_time() - callback_start_us);

            int64_t log_now_us = esp_timer_get_time();
            int64_t log_elapsed_us = log_now_us - qird_last_log_us;
            if (log_elapsed_us >= 1000000) {
                double bytes_per_second = qird_bytes * 1000000.0 / log_elapsed_us;
                ESP_LOGD(TAG,
                         "[QIRD RX] uart=%d %.0f B/s (%u B/%u ms), reads=%u empty=%u "
                         "chunk_max=%u gap_max=%u ms callback_max=%u us",
                         uart_num_, bytes_per_second, qird_bytes,
                         static_cast<unsigned>(log_elapsed_us / 1000), qird_count,
                         qird_zero_count, static_cast<unsigned>(qird_max_chunk),
                         static_cast<unsigned>(qird_max_gap_us / 1000),
                         static_cast<unsigned>(qird_max_callback_us));
                qird_count = 0;
                qird_bytes = 0;
                qird_zero_count = 0;
                qird_max_chunk = 0;
                qird_max_gap_us = 0;
                qird_max_callback_us = 0;
                qird_last_log_us = log_now_us;
            }
            return true;
        }
        // AT+QIRD=<id>,0 的长度查询会返回多个数字，交给通用解析。
    }

    // ========== 特殊处理：+QIURC: "recv" 基于 data_length 精确解析（文本模式）==========
    // 格式：+QIURC: "recv",<connect_id>,<data_length>,<raw_data>\r\n
    // raw_data 可能包含 \r\n、逗号、引号等任意字符，必须用 data_length 定位结尾
    bool is_qiurc_recv = rx_buffer_.size() >= 15 &&
                         rx_buffer_.compare(0, 15, "+QIURC: \"recv\",") == 0;
    auto qiurc_id_comma = is_qiurc_recv ? rx_buffer_.find(',', 15) : std::string::npos;
    auto qiurc_header_end = is_qiurc_recv ? rx_buffer_.find("\r\n", 15) : std::string::npos;
    // 缓存模式只有 +QIURC: "recv",<id>\r\n，应交给下方通用行解析。
    // 只有 id 后面在本行内确实还有逗号时，才是带 length/data 的直吐模式。
    bool is_qiurc_direct_push = is_qiurc_recv && qiurc_id_comma != std::string::npos &&
                                (qiurc_header_end == std::string::npos ||
                                 qiurc_id_comma < qiurc_header_end);
    if (is_qiurc_direct_push) {
        static uint32_t qiurc_count = 0;
        static uint32_t qiurc_bytes = 0;
        static size_t qiurc_min_chunk = SIZE_MAX;
        static size_t qiurc_max_chunk = 0;
        static int64_t qiurc_max_gap_us = 0;
        static int64_t qiurc_max_callback_us = 0;
        static int64_t qiurc_last_rx_us = 0;
        static int64_t qiurc_last_log_us = esp_timer_get_time();

        auto comma1 = qiurc_id_comma;
        std::string id_str = rx_buffer_.substr(15, comma1 - 15);
        if (!is_number(id_str)) return false;

        auto comma2 = rx_buffer_.find(',', comma1 + 1);
        if (comma2 == std::string::npos) return false;
        std::string len_str = rx_buffer_.substr(comma1 + 1, comma2 - comma1 - 1);
        if (!is_number(len_str)) return false;

        int data_length = std::stoi(len_str);
        size_t data_start = comma2 + 1;
        size_t line_end = data_start + data_length + 2; // raw_data + \r\n
        if (rx_buffer_.size() < line_end) return false;

        int connect_id = std::stoi(id_str);
        std::string raw_data = rx_buffer_.substr(data_start, data_length);

        ESP_LOGD(TAG, "<< +QIURC: recv, id=%d, len=%d", connect_id, data_length);
        rx_buffer_.erase(0, line_end);

        AtArgumentValue arg_recv;
        arg_recv.type = AtArgumentValue::Type::String;
        arg_recv.string_value = "recv";
        parsed_arguments.push_back(arg_recv);
        AtArgumentValue arg_id;
        arg_id.type = AtArgumentValue::Type::Int;
        arg_id.int_value = connect_id;
        parsed_arguments.push_back(arg_id);
        AtArgumentValue arg_len;
        arg_len.type = AtArgumentValue::Type::Int;
        arg_len.int_value = data_length;
        parsed_arguments.push_back(arg_len);
        AtArgumentValue arg_data;
        arg_data.type = AtArgumentValue::Type::String;
        arg_data.string_value = std::move(raw_data);
        parsed_arguments.push_back(arg_data);

        command = "QIURC";
        lock.unlock();
        int64_t rx_now_us = esp_timer_get_time();
        if (qiurc_last_rx_us != 0) {
            qiurc_max_gap_us = std::max(qiurc_max_gap_us, rx_now_us - qiurc_last_rx_us);
        }
        qiurc_last_rx_us = rx_now_us;
        ++qiurc_count;
        qiurc_bytes += data_length;
        qiurc_min_chunk = std::min(qiurc_min_chunk, static_cast<size_t>(data_length));
        qiurc_max_chunk = std::max(qiurc_max_chunk, static_cast<size_t>(data_length));

        int64_t callback_start_us = esp_timer_get_time();
        HandleUrc(command, parsed_arguments);
        qiurc_max_callback_us = std::max(qiurc_max_callback_us,
                                         esp_timer_get_time() - callback_start_us);

        int64_t log_now_us = esp_timer_get_time();
        int64_t log_elapsed_us = log_now_us - qiurc_last_log_us;
        if (log_elapsed_us >= 1000000) {
            double bytes_per_second = qiurc_bytes * 1000000.0 / log_elapsed_us;
            ESP_LOGD(TAG,
                     "[QIURC RX] uart=%d %.0f B/s (%u B/%u ms), urc=%u "
                     "chunk_min/max=%u/%u gap_max=%u ms callback_max=%u us",
                     uart_num_, bytes_per_second, qiurc_bytes,
                     static_cast<unsigned>(log_elapsed_us / 1000), qiurc_count,
                     static_cast<unsigned>(qiurc_min_chunk == SIZE_MAX ? 0 : qiurc_min_chunk),
                     static_cast<unsigned>(qiurc_max_chunk),
                     static_cast<unsigned>(qiurc_max_gap_us / 1000),
                     static_cast<unsigned>(qiurc_max_callback_us));
            qiurc_count = 0;
            qiurc_bytes = 0;
            qiurc_min_chunk = SIZE_MAX;
            qiurc_max_chunk = 0;
            qiurc_max_gap_us = 0;
            qiurc_max_callback_us = 0;
            qiurc_last_log_us = log_now_us;
        }
        return true;
    }

    // ========== 特殊处理：+QSSLURC: "recv" 基于 data_length 精确解析（文本模式）==========
    // 格式：+QSSLURC: "recv",<ssl_id>,<data_length>,<raw_data>\r\n
    if (rx_buffer_.size() >= 17 && rx_buffer_.compare(0, 17, "+QSSLURC: \"recv\",") == 0) {
        auto comma1 = rx_buffer_.find(',', 17);
        if (comma1 == std::string::npos) return false;
        std::string id_str = rx_buffer_.substr(17, comma1 - 17);
        if (!is_number(id_str)) return false;

        auto comma2 = rx_buffer_.find(',', comma1 + 1);
        if (comma2 == std::string::npos) return false;
        std::string len_str = rx_buffer_.substr(comma1 + 1, comma2 - comma1 - 1);
        if (!is_number(len_str)) return false;

        int data_length = std::stoi(len_str);
        size_t data_start = comma2 + 1;
        size_t line_end = data_start + data_length + 2; // raw_data + \r\n
        if (rx_buffer_.size() < line_end) return false;

        int ssl_id = std::stoi(id_str);
        std::string raw_data = rx_buffer_.substr(data_start, data_length);

        ESP_LOGD(TAG, "<< +QSSLURC: recv, id=%d, len=%d", ssl_id, data_length);
        rx_buffer_.erase(0, line_end);

        AtArgumentValue arg_recv;
        arg_recv.type = AtArgumentValue::Type::String;
        arg_recv.string_value = "recv";
        parsed_arguments.push_back(arg_recv);
        AtArgumentValue arg_id;
        arg_id.type = AtArgumentValue::Type::Int;
        arg_id.int_value = ssl_id;
        parsed_arguments.push_back(arg_id);
        AtArgumentValue arg_len;
        arg_len.type = AtArgumentValue::Type::Int;
        arg_len.int_value = data_length;
        parsed_arguments.push_back(arg_len);
        AtArgumentValue arg_data;
        arg_data.type = AtArgumentValue::Type::String;
        arg_data.string_value = std::move(raw_data);
        parsed_arguments.push_back(arg_data);

        command = "QSSLURC";
        lock.unlock();
        HandleUrc(command, parsed_arguments);
        return true;
    }

    // ========== 通用解析：基于 \r\n 行分割 ==========
    auto end_pos = rx_buffer_.find("\r\n");
    if (end_pos == std::string::npos) {
        // FIXME: for +MHTTPURC: "ind", missing newline
        if (rx_buffer_.size() >= 16 && memcmp(rx_buffer_.c_str(), "+MHTTPURC: \"ind\"", 16) == 0) {
            // Find the end of this line and add \r\n if missing
            auto next_plus = rx_buffer_.find("+", 1);
            if (next_plus != std::string::npos) {
                // Insert \r\n before the next + command
                rx_buffer_.insert(next_plus, "\r\n");
            } else {
                // Append \r\n at the end
                rx_buffer_.append("\r\n");
            }
            end_pos = rx_buffer_.find("\r\n");
        } else {
            return false;
        }
    }

    // Ignore empty lines
    if (end_pos == 0) {
        rx_buffer_.erase(0, 2);
        return true;
    }

    ESP_LOGD(TAG, "<< %.64s (%u bytes)", rx_buffer_.substr(0, end_pos).c_str(), end_pos);
    // print last 64 bytes before end_pos if available
    // if (end_pos > 64) {
    //     ESP_LOGI(TAG, "<< LAST: %.64s", rx_buffer_.c_str() + end_pos - 64);
    // }

    // Parse "+CME ERROR: 123,456,789"
    if (rx_buffer_[0] == '+') {
        std::string command, values;
        auto pos = rx_buffer_.find(": ");
        if (pos == std::string::npos || pos > end_pos) {
            command = rx_buffer_.substr(1, end_pos - 1);
        } else {
            command = rx_buffer_.substr(1, pos - 1);
            values = rx_buffer_.substr(pos + 2, end_pos - pos - 2);
        }
        rx_buffer_.erase(0, end_pos + 2);

        // QLTS contains commas inside its quoted date/time. Preserve the raw
        // value for the modem's strict parser instead of splitting it as CSV.
        if (command == "QLTS") {
            AtArgumentValue value{};
            value.type = AtArgumentValue::Type::String;
            value.string_value = std::move(values);
            std::vector<AtArgumentValue> arguments;
            arguments.push_back(std::move(value));
            lock.unlock();
            HandleUrc(command, arguments);
            return true;
        }

        // Parse "string", int, int, ... into AtArgumentValue
        std::vector<AtArgumentValue> arguments;
        std::istringstream iss(values);
        std::string item;
        while (std::getline(iss, item, ',')) {
            AtArgumentValue argument;
            if (item.front() == '"') {
                argument.type = AtArgumentValue::Type::String;
                argument.string_value = item.substr(1, item.size() - 2);
            } else if (item.find(".") != std::string::npos) {
                argument.type = AtArgumentValue::Type::Double;
                argument.double_value = std::stod(item);
            } else if (is_number(item)) {
                argument.type = AtArgumentValue::Type::Int;
                argument.int_value = std::stoi(item);
                argument.string_value = std::move(item);
            } else {
                argument.type = AtArgumentValue::Type::String;
                argument.string_value = std::move(item);
            }
            arguments.push_back(argument);
        }

        // 释放锁后再调用HandleUrc，避免死锁（HandleUrc内部也会获取mutex_）
        lock.unlock();
        HandleUrc(command, arguments);
        return true;
    } else if (rx_buffer_.size() >= 4 && rx_buffer_[0] == 'O' && rx_buffer_[1] == 'K' && rx_buffer_[2] == '\r' && rx_buffer_[3] == '\n') {
        rx_buffer_.erase(0, 4);
        lock.unlock();  // 释放锁后再设置事件
        xEventGroupSetBits(event_group_handle_, AT_EVENT_COMMAND_DONE);
        return true;
    } else if (rx_buffer_.size() >= 7 && rx_buffer_[0] == 'E' && rx_buffer_[1] == 'R' && rx_buffer_[2] == 'R' && rx_buffer_[3] == 'O' && rx_buffer_[4] == 'R' && rx_buffer_[5] == '\r' && rx_buffer_[6] == '\n') {
        rx_buffer_.erase(0, 7);
        lock.unlock();  // 释放锁后再设置事件
        xEventGroupSetBits(event_group_handle_, AT_EVENT_COMMAND_ERROR);
        return true;
    } else {
        // mutex_ already locked at function start
        response_ = rx_buffer_.substr(0, end_pos);
        rx_buffer_.erase(0, end_pos + 2);
        return true;
    }
    return false;
}

void AtUart::HandleCommand(const char* command) {
    // 这个函数现在主要用于向后兼容，大部分处理逻辑已经移到 ParseLine 中
    if (wait_for_response_) {
        response_.append(command);
        response_.append("\r\n");
    }
}

void AtUart::HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments) {
    if (command == "CME ERROR") {
        cme_error_code_ = arguments[0].int_value;
        xEventGroupSetBits(event_group_handle_, AT_EVENT_COMMAND_ERROR);
        return;
    }

    // Never invoke external callbacks while holding the RX-buffer/callback-list
    // mutex. Snapshot the callbacks so UART event handling can continue buffering
    // response bytes while application callbacks run.
    std::list<UrcCallback> callbacks;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        callbacks = urc_callbacks_;
    }
    for (auto& callback : callbacks) {
        callback(command, arguments);
    }
}

bool AtUart::DetectBaudRate() {
    int baud_rates[] = {115200, 921600, 460800, 230400, 57600, 38400, 19200, 9600};
    while (true) {
        ESP_LOGI(TAG, "Detecting baud rate...");
        for (size_t i = 0; i < sizeof(baud_rates) / sizeof(baud_rates[0]); i++) {
            int rate = baud_rates[i];
            uart_set_baudrate(uart_num_, rate);
            if (SendCommand("AT", 20)) {
                ESP_LOGI(TAG, "Detected baud rate: %d", rate);
                baud_rate_ = rate;
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

bool AtUart::SetBaudRate(int new_baud_rate) {
    if (!DetectBaudRate()) {
        ESP_LOGE(TAG, "Failed to detect baud rate");
        return false;
    }
    if (new_baud_rate == baud_rate_) {
        return true;
    }
    // Set new baud rate
    if (!SendCommand(std::string("AT+IPR=") + std::to_string(new_baud_rate))) {
        ESP_LOGI(TAG, "Failed to set baud rate to %d", new_baud_rate);
        return false;
    }
    uart_set_baudrate(uart_num_, new_baud_rate);
    baud_rate_ = new_baud_rate;
    ESP_LOGI(TAG, "Set baud rate to %d", new_baud_rate);
    return true;
}

bool AtUart::SendData(const char* data, size_t length) {
    // ESP_LOGW(TAG, ">> %.*s (%zu bytes)", (int)length, data, length);

    if (!initialized_) {
        ESP_LOGE(TAG, "UART未初始化");
        return false;
    }
    
    int ret = uart_write_bytes(uart_num_, data, length);
    if (ret < 0) {
        ESP_LOGE(TAG, "uart_write_bytes failed: %d", ret);
        return false;
    }
    return true;
}

bool AtUart::SendCommand(const std::string& command, size_t timeout_ms, bool add_crlf) {
    int64_t t_request = esp_timer_get_time();
    std::lock_guard<std::recursive_timed_mutex> lock(command_mutex_);
    int64_t wait_us = esp_timer_get_time() - t_request;
    if (wait_us > CHANNEL_CONTENTION_WARN_US) {
        // 通道被别的命令占用了较长时间才轮到本条，常见于 GPS 轮询与 MQTT/HTTP 数据通信抢占同一 AT 通道
        ESP_LOGW(TAG, "[通道] \"%.32s\" 排队等待通道 %lld ms", command.c_str(), (long long)(wait_us / 1000));
    }

    xEventGroupClearBits(event_group_handle_, AT_EVENT_COMMAND_DONE | AT_EVENT_COMMAND_ERROR);
    wait_for_response_ = true;
    cme_error_code_ = 0;
    response_.clear();
    // ESP_LOGW(TAG, ">> %.64s (%u bytes)", command.data(), command.length());

    if (add_crlf) {
        if (!SendData((command + "\r\n").data(), command.length() + 2)) {
            return false;
        }
    } else {
        if (!SendData(command.data(), command.length())) {
            return false;
        }
    }
    if (timeout_ms > 0) {
        int64_t t_sent = esp_timer_get_time();
        auto bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_COMMAND_DONE | AT_EVENT_COMMAND_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
        wait_for_response_ = false;
        bool ok = bits & AT_EVENT_COMMAND_DONE;
        if (!ok) {
            int64_t elapsed_ms = (esp_timer_get_time() - t_sent) / 1000;
            ESP_LOGW(TAG, "[无响应] \"%.32s\" 等待 %lld ms 未收到OK (limit=%zums, CME=%d)",
                     command.c_str(), (long long)elapsed_ms, timeout_ms, cme_error_code_);
        }
        return ok;
    } else {
        wait_for_response_ = false;
    }
    return true;
}

bool AtUart::SendCommandWithData(const std::string& command, const char* data, size_t data_length, size_t timeout_ms) {
    int64_t t_request = esp_timer_get_time();
    std::lock_guard<std::recursive_timed_mutex> lock(command_mutex_);
    int64_t wait_us = esp_timer_get_time() - t_request;
    if (wait_us > CHANNEL_CONTENTION_WARN_US) {
        ESP_LOGW(TAG, "[通道] \"%.32s\"(带数据) 排队等待通道 %lld ms", command.c_str(), (long long)(wait_us / 1000));
    }

    xEventGroupClearBits(event_group_handle_, AT_EVENT_COMMAND_DONE | AT_EVENT_COMMAND_ERROR);
    wait_for_response_ = true;
    cme_error_code_ = 0;
    response_.clear();

    // 原子性地发送命令和数据
    if (!SendData((command + "\r\n").data(), command.length() + 2)) {
        wait_for_response_ = false;
        return false;
    }

    // 等待命令响应（通常是 ">" 提示符）
    int64_t t_sent = esp_timer_get_time();
    auto bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_COMMAND_DONE | AT_EVENT_COMMAND_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(5000));
    if (!(bits & AT_EVENT_COMMAND_DONE)) {
        wait_for_response_ = false;
        ESP_LOGW(TAG, "[无响应] \"%.32s\" 等待 %lld ms 未收到\">\"提示符 (CME=%d)",
                 command.c_str(), (long long)((esp_timer_get_time() - t_sent) / 1000), cme_error_code_);
        return false;
    }

    // 发送数据
    if (!SendData(data, data_length)) {
        wait_for_response_ = false;
        return false;
    }

    // 等待最终响应
    if (timeout_ms > 0) {
        int64_t t_data_sent = esp_timer_get_time();
        bits = xEventGroupWaitBits(event_group_handle_, AT_EVENT_COMMAND_DONE | AT_EVENT_COMMAND_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
        wait_for_response_ = false;
        bool ok = bits & AT_EVENT_COMMAND_DONE;
        if (!ok) {
            ESP_LOGW(TAG, "[无响应] \"%.32s\" 发送数据(%zuB)后等待 %lld ms 未收到最终OK (limit=%zums, CME=%d)",
                     command.c_str(), data_length, (long long)((esp_timer_get_time() - t_data_sent) / 1000),
                     timeout_ms, cme_error_code_);
        }
        return ok;
    } else {
        wait_for_response_ = false;
    }
    return true;
}

bool AtUart::TryLockChannel(uint32_t timeout_ms) {
    return command_mutex_.try_lock_for(std::chrono::milliseconds(timeout_ms));
}

void AtUart::UnlockChannel() {
    command_mutex_.unlock();
}

std::list<UrcCallback>::iterator AtUart::RegisterUrcCallback(UrcCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    return urc_callbacks_.insert(urc_callbacks_.end(), callback);
}

void AtUart::UnregisterUrcCallback(std::list<UrcCallback>::iterator iterator) {
    std::lock_guard<std::mutex> lock(mutex_);
    urc_callbacks_.erase(iterator);
}

void AtUart::SetDtrPin(bool high) {
    if (dtr_pin_ != GPIO_NUM_NC) {
        ESP_LOGD(TAG, "Set DTR pin %d to %d", dtr_pin_, high ? 1 : 0);
        gpio_set_level(dtr_pin_, high ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static const char hex_chars[] = "0123456789ABCDEF";
// 辅助函数，将单个十六进制字符转换为对应的数值
inline uint8_t CharToHex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;  // 对于无效输入，返回0
}

void AtUart::EncodeHexAppend(std::string& dest, const char* data, size_t length) {
    dest.reserve(dest.size() + length * 2 + 4);  // 预分配空间，多分配4个字节用于\r\n\0
    for (size_t i = 0; i < length; i++) {
        dest.push_back(hex_chars[(data[i] & 0xF0) >> 4]);
        dest.push_back(hex_chars[data[i] & 0x0F]);
    }
}

void AtUart::DecodeHexAppend(std::string& dest, const char* data, size_t length) {
    dest.reserve(dest.size() + length / 2 + 4);  // 预分配空间，多分配4个字节用于\r\n\0
    for (size_t i = 0; i < length; i += 2) {
        char byte = (CharToHex(data[i]) << 4) | CharToHex(data[i + 1]);
        dest.push_back(byte);
    }
}

std::string AtUart::EncodeHex(const std::string& data) {
    std::string encoded;
    EncodeHexAppend(encoded, data.c_str(), data.size());
    return encoded;
}

std::string AtUart::DecodeHex(const std::string& data) {
    std::string decoded;
    DecodeHexAppend(decoded, data.c_str(), data.size());
    return decoded;
}
