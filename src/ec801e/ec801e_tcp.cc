#include "ec801e_tcp.h"

#include <esp_log.h>

#define TAG "Ec801ETcp"


Ec801ETcp::Ec801ETcp(std::shared_ptr<AtUart> at_uart, int tcp_id,
                     TcpAccessMode access_mode)
    : at_uart_(at_uart), tcp_id_(tcp_id), access_mode_(access_mode) {
    event_group_handle_ = xEventGroupCreate();

    urc_callback_it_ = at_uart_->RegisterUrcCallback([this](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
        if (command == "QIOPEN" && arguments.size() == 2) {
            if (arguments[0].int_value == tcp_id_) {
                if (arguments[1].int_value == 0) {
                    connected_ = true;
                    instance_active_ = true;
                    xEventGroupClearBits(event_group_handle_, EC801E_TCP_DISCONNECTED | EC801E_TCP_ERROR);
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_CONNECTED);
                } else {
                    ESP_LOGE(TAG, "QIOPEN error code: %d (id=%d)", arguments[1].int_value, tcp_id_);
                    connected_ = false;
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_ERROR);
                    if (disconnect_callback_) {
                        disconnect_callback_();
                    }
                }
            }
        } else if (command == "QISEND" && arguments.size() == 3) {
            if (arguments[0].int_value == tcp_id_) {
                if (arguments[1].int_value == 0) {
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_SEND_COMPLETE);
                } else {
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_SEND_FAILED);
                }
            }
        } else if (command == "QIURC" && arguments.size() >= 2) {
            if (arguments[1].int_value == tcp_id_) {
                if (arguments[0].string_value == "recv") {
                    if (arguments.size() >= 4) {
                        // 兼容直吐模式的原始数据 URC。
                        if (connected_ && stream_callback_) {
                            stream_callback_(arguments[3].string_value);
                        }
                    } else if (connected_ && access_mode_ == TcpAccessMode::Buffer) {
                        // 缓存模式只上报数据可读通知，由独立任务执行 QIRD。
                        xEventGroupSetBits(event_group_handle_, EC801E_TCP_DATA_AVAILABLE);
                    }
                } else if (arguments[0].string_value == "closed") {
                    if (connected_) {
                        connected_ = false;
                        if (disconnect_callback_) {
                            disconnect_callback_();
                        }
                    }
                    xEventGroupSetBits(event_group_handle_, EC801E_TCP_DISCONNECTED);
                } else {
                    ESP_LOGE(TAG, "Unknown QIURC command: %s", arguments[0].string_value.c_str());
                }
            }
        } else if (command == "QIRD" && qird_in_progress_.load()) {
            if (!arguments.empty()) {
                int data_length = arguments[0].int_value;
                last_qird_length_.store(data_length);
                if (data_length > 0 && arguments.size() >= 2 && connected_ && stream_callback_) {
                    stream_callback_(arguments[1].string_value);
                }
            }
        } else if (command == "QISTATE" && arguments.size() > 5) {
            if (arguments[0].int_value == tcp_id_) {
                connected_ = arguments[5].int_value == 2;
                instance_active_ = true;
                xEventGroupSetBits(event_group_handle_, EC801E_TCP_INITIALIZED);
            }
        } else if (command == "FIFO_OVERFLOW") {
            xEventGroupSetBits(event_group_handle_, EC801E_TCP_ERROR);
            Disconnect();
        }
    });

    if (access_mode_ == TcpAccessMode::Buffer) {
        if (xTaskCreate([](void* arg) {
                auto tcp = static_cast<Ec801ETcp*>(arg);
                tcp->ReceiveTask();
                vTaskDelete(nullptr);
            }, "ec801e_qird", 6144, this, 5, &receive_task_handle_) != pdPASS) {
            receive_task_handle_ = nullptr;
            ESP_LOGE(TAG, "Failed to create QIRD receive task");
        }
    }
}

Ec801ETcp::~Ec801ETcp() {
    Disconnect();
    StopReceiveTask();
    at_uart_->UnregisterUrcCallback(urc_callback_it_);
    if (event_group_handle_) {
        vEventGroupDelete(event_group_handle_);
    }
}

bool Ec801ETcp::Connect(const std::string& host, int port) {
    // Clear bits
    xEventGroupClearBits(event_group_handle_, EC801E_TCP_CONNECTED | EC801E_TCP_DISCONNECTED | EC801E_TCP_ERROR);

    bool use_buffer_mode = access_mode_ == TcpAccessMode::Buffer;
    int view_mode = use_buffer_mode ? 0 : 1;
    int access_mode = static_cast<int>(access_mode_);
    ESP_LOGI(TAG, "TCP id=%d access mode: %s", tcp_id_,
             use_buffer_mode ? "buffer/QIRD" : "direct push/QIURC");

    // 缓存模式使用文档的分行 QIRD 格式，直吐模式使用单行 QIURC 格式。
    at_uart_->SendCommand("AT+QICFG=\"close/mode\",1;+QICFG=\"viewmode\"," +
                          std::to_string(view_mode) +
                          ";+QICFG=\"sendinfo\",1;+QICFG=\"dataformat\",0,0");

    // 无条件关闭，确保模块侧 ID 空闲（忽略返回值）
    at_uart_->SendCommand("AT+QICLOSE=" + std::to_string(tcp_id_));
    xEventGroupWaitBits(event_group_handle_, EC801E_TCP_DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(2000));
    instance_active_ = false;

    std::string command = "AT+QIOPEN=1," + std::to_string(tcp_id_) +
                          ",\"TCP\",\"" + host + "\"," + std::to_string(port) +
                          ",0," + std::to_string(access_mode);
    ESP_LOGI(TAG, "Sending: %s", command.c_str());
    if (!at_uart_->SendCommand(command)) {
        ESP_LOGE(TAG, "Failed to open TCP connection");
        return false;
    }

    // 等待连接完成
    auto bits = xEventGroupWaitBits(event_group_handle_, EC801E_TCP_CONNECTED | EC801E_TCP_ERROR, pdTRUE, pdFALSE, TCP_CONNECT_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (bits & EC801E_TCP_ERROR) {
        ESP_LOGE(TAG, "Failed to connect to %s:%d", host.c_str(), port);
        return false;
    }
    if (!(bits & EC801E_TCP_CONNECTED)) {
        ESP_LOGE(TAG, "Connect timeout to %s:%d", host.c_str(), port);
        return false;
    }
    return true;
}

void Ec801ETcp::ReceiveTask() {
    constexpr int kReadLength = 1500;
    constexpr int kMaxReadsPerBatch = 16;

    while (true) {
        auto bits = xEventGroupWaitBits(
            event_group_handle_,
            EC801E_TCP_DATA_AVAILABLE | EC801E_TCP_RECEIVE_TASK_STOP,
            pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & EC801E_TCP_RECEIVE_TASK_STOP) {
            break;
        }
        if (!(bits & EC801E_TCP_DATA_AVAILABLE) || !connected_) {
            continue;
        }

        bool needs_more_reads = false;
        int reads = 0;
        for (; reads < kMaxReadsPerBatch && connected_; ++reads) {
            // 先拿到 AT 命令通道，确保只有当前 Tcp 实例处理没有 connectID
            // 字段的 +QIRD 返回。SendCommand 内部使用可递归锁，可安全重入。
            if (!at_uart_->TryLockChannel(200)) {
                needs_more_reads = true;
                break;
            }

            last_qird_length_.store(-1);
            qird_in_progress_.store(true);
            bool ok = at_uart_->SendCommand(
                "AT+QIRD=" + std::to_string(tcp_id_) + "," + std::to_string(kReadLength),
                2000);
            qird_in_progress_.store(false);
            at_uart_->UnlockChannel();

            if (!ok) {
                ESP_LOGW(TAG, "QIRD failed for id=%d", tcp_id_);
                needs_more_reads = connected_;
                break;
            }

            int actual_length = last_qird_length_.load();
            if (actual_length < 0) {
                ESP_LOGW(TAG, "QIRD returned OK without a parsed length (id=%d)", tcp_id_);
                needs_more_reads = connected_;
                break;
            }
            if (actual_length == 0) {
                // 文档 3.2.3：QIRD: 0 表示模块接收缓存已读空。
                break;
            }
        }

        if (connected_ && (needs_more_reads || reads >= kMaxReadsPerBatch)) {
            // 缓存未读空时模块不会再报新 URC，因此需要主动续读。
            xEventGroupSetBits(event_group_handle_, EC801E_TCP_DATA_AVAILABLE);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    xEventGroupSetBits(event_group_handle_, EC801E_TCP_RECEIVE_TASK_STOPPED);
}

void Ec801ETcp::StopReceiveTask() {
    if (!receive_task_handle_) {
        return;
    }

    xEventGroupSetBits(event_group_handle_, EC801E_TCP_RECEIVE_TASK_STOP);
    auto bits = xEventGroupWaitBits(event_group_handle_, EC801E_TCP_RECEIVE_TASK_STOPPED,
                                    pdTRUE, pdFALSE, pdMS_TO_TICKS(3000));
    if (!(bits & EC801E_TCP_RECEIVE_TASK_STOPPED)) {
        ESP_LOGW(TAG, "QIRD receive task did not stop in time");
        vTaskDelete(receive_task_handle_);
    }
    receive_task_handle_ = nullptr;
}

void Ec801ETcp::SetReceiveTaskPriority(unsigned int priority) {
    if (receive_task_handle_) {
        vTaskPrioritySet(receive_task_handle_, priority);
    }
}

void Ec801ETcp::Disconnect() {
    if (!instance_active_) {
        return;
    }
    
    if (at_uart_->SendCommand("AT+QICLOSE=" + std::to_string(tcp_id_))) {
        instance_active_ = false;
    }

    if (connected_) {
        connected_ = false;
        if (disconnect_callback_) {
            disconnect_callback_();
        }
    }
}

int Ec801ETcp::Send(const std::string& data) {
    const size_t MAX_PACKET_SIZE = 1460;
    const int MAX_RETRY_COUNT = 3;
    const int RETRY_DELAY_MS = 10;
    size_t total_sent = 0;

    if (!connected_) {
        ESP_LOGE(TAG, "Not connected");
        return -1;
    }

    while (total_sent < data.size()) {
        size_t chunk_size = std::min(data.size() - total_sent, MAX_PACKET_SIZE);
        
        std::string command = "AT+QISEND=" + std::to_string(tcp_id_) + "," + std::to_string(chunk_size);
        
        // 使用原子方法发送命令和数据，避免并发问题
        // 添加重试机制
        bool send_success = false;
        int retry_count = 0;
        
        while (retry_count < MAX_RETRY_COUNT) {
            if (at_uart_->SendCommandWithData(command, data.data() + total_sent, chunk_size)) {
                send_success = true;
                break;
            }
            
            retry_count++;
            if (retry_count < MAX_RETRY_COUNT) {
                ESP_LOGW(TAG, "Send command and data failed, retrying (%d/%d)...", retry_count, MAX_RETRY_COUNT);
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            } else {
                ESP_LOGE(TAG, "Send command and data failed after %d retries", MAX_RETRY_COUNT);
            }
        }
        
        if (!send_success) {
            ESP_LOGE(TAG, "Send command and data failed, disconnecting");
            Disconnect();
            return -1;
        }
        
        // 等待发送完成
        auto bits = xEventGroupWaitBits(event_group_handle_, EC801E_TCP_SEND_COMPLETE | EC801E_TCP_SEND_FAILED, pdTRUE, pdFALSE, pdMS_TO_TICKS(TCP_CONNECT_TIMEOUT_MS));
        if (bits & EC801E_TCP_SEND_FAILED) {
            ESP_LOGW(TAG, "Send failed, retrying chunk...");
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
            continue;  // 重试当前chunk
        } else if (!(bits & EC801E_TCP_SEND_COMPLETE)) {
            ESP_LOGE(TAG, "Send timeout");
            return -1;
        }
        
        total_sent += chunk_size;
    }
    return data.size();
}
