#include "ec801e_gnss.h"
#include <esp_log.h>
#include <cstdlib>
#include <cstring>

#define TAG "Ec801EGnss"

// ── 共用坐标解析（ddmm.mmmm[N/S/E/W] → 十进制度）────────────────────────────

static double ParseQgpsCoordRaw(const char* s, size_t len) {
    if (len < 2) return 0.0;
    char dir = s[len - 1];
    char num[20] = {};
    size_t num_len = len - 1;
    if (num_len >= sizeof(num)) return 0.0;
    memcpy(num, s, num_len);
    double raw = atof(num);
    int degrees = (int)(raw / 100);
    double decimal = degrees + (raw - degrees * 100.0) / 60.0;
    if (dir == 'S' || dir == 'W') decimal = -decimal;
    return decimal;
}

// Quectel NMEA ddmm.mmmm[N/S/E/W] → 十进制度（供 Ec801EAtModem 的 AtUart 路径使用）
static double ParseQgpsCoord(const std::string& s) {
    return ParseQgpsCoordRaw(s.c_str(), s.size());
}

// ── 供 Ec801EAtModem（4G 路径）复用的任务 ────────────────────────────────────

void Ec801ERunGnssTask(std::shared_ptr<AtUart> at_uart, GnssCallback callback, int timeout_seconds) {
    struct TaskArgs {
        std::shared_ptr<AtUart> at_uart;
        GnssCallback callback;
        int timeout_seconds;
    };
    auto* args = new TaskArgs{at_uart, std::move(callback), timeout_seconds};

    xTaskCreate([](void* arg) {
        auto* args = static_cast<TaskArgs*>(arg);
        auto at_uart = args->at_uart;
        auto callback = std::move(args->callback);
        int timeout_seconds = args->timeout_seconds;
        delete args;

        GnssLocation location;
        volatile bool got_fix = false;

        auto urc_cb = at_uart->RegisterUrcCallback(
            [&location, &got_fix](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
                if (command == "QGPSLOC" && arguments.size() >= 5) {
                    location.latitude  = ParseQgpsCoord(arguments[1].string_value);
                    location.longitude = ParseQgpsCoord(arguments[2].string_value);
                    location.altitude  = arguments[4].double_value;
                    location.valid     = true;
                    got_fix = true;
                }
            });

        if (at_uart->SendCommand("AT+QGPS=1", 3000)) {
            ESP_LOGI(TAG, "[GNSS] AT+QGPS=1 成功，开始搜星");
        } else {
            ESP_LOGW(TAG, "[GNSS] AT+QGPS=1 失败，模组可能不支持或已开启");
        }

        int elapsed = 0;
        while (elapsed < timeout_seconds && !got_fix) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            elapsed += 10;
            got_fix = false;
            at_uart->SendCommand("AT+QGPSLOC?", 3000);
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP_LOGI(TAG, "[GNSS] 搜星中... %d/%d 秒", elapsed, timeout_seconds);
            if (got_fix) break;
        }

        at_uart->SendCommand("AT+QGPS=0", 2000);
        at_uart->UnregisterUrcCallback(urc_cb);

        if (got_fix) {
            ESP_LOGI(TAG, "[GNSS] 定位成功: lat=%.6f, lon=%.6f, alt=%.1f",
                     location.latitude, location.longitude, location.altitude);
        } else {
            ESP_LOGW(TAG, "[GNSS] 搜星超时，未获取到定位");
        }
        callback(got_fix, location);
        vTaskDelete(NULL);
    }, "gnss_task", 4096, args, 5, NULL);
}

// ── Ec801EGnss：Wi-Fi 模式直接 UART，无后台任务 ──────────────────────────────

// 波特率自动检测（尝试常见波特率，发送 AT 直到收到 OK）
static int detect_baud_rate(uart_port_t port) {
    const int baud_rates[] = {921600, 115200, 460800, 230400, 57600};
    char rx_buf[32];

    for (size_t i = 0; i < sizeof(baud_rates) / sizeof(baud_rates[0]); i++) {
        int rate = baud_rates[i];
        uart_set_baudrate(port, rate);

        // 清空接收缓冲
        uart_flush_input(port);

        // 发送 AT\r\n
        const char* at_cmd = "AT\r\n";
        uart_write_bytes(port, at_cmd, strlen(at_cmd));

        // 等待响应（最多 300ms）
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(300);
        int pos = 0;
        bool got_ok = false;

        while (xTaskGetTickCount() < deadline && pos < (int)sizeof(rx_buf) - 1) {
            uint8_t c;
            if (uart_read_bytes(port, &c, 1, pdMS_TO_TICKS(50)) > 0) {
                if (c != '\r' && c != '\n') {
                    rx_buf[pos++] = (char)c;
                } else if (pos > 0) {
                    rx_buf[pos] = '\0';
                    if (strncmp(rx_buf, "OK", 2) == 0 || strncmp(rx_buf, "AT", 2) == 0) {
                        got_ok = true;
                        break;
                    }
                    pos = 0;
                }
            }
        }

        if (got_ok) {
            ESP_LOGI(TAG, "检测到模组波特率: %d", rate);
            return rate;
        }
    }

    ESP_LOGW(TAG, "未检测到有效波特率，使用默认 921600");
    return 921600;
}

// 读取一行（到 \n），返回字节数，超时返回 -1
static int gnss_read_line(uart_port_t port, char* buf, int size, uint32_t timeout_ms) {
    int pos = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (pos < size - 1) {
        if (xTaskGetTickCount() > deadline) return -1;
        uint8_t c;
        if (uart_read_bytes(port, &c, 1, pdMS_TO_TICKS(50)) <= 0) continue;
        if (c == '\n') { buf[pos] = '\0'; return pos; }
        if (c != '\r') buf[pos++] = (char)c;
    }
    buf[pos] = '\0';
    return pos;
}

// 获取 CSV 第 idx 字段（从 0 开始）
static const char* csv_field(const char* s, int idx, size_t* out_len) {
    for (int i = 0; i < idx; i++) {
        s = strchr(s, ',');
        if (!s) return nullptr;
        s++;
    }
    const char* end = strchr(s, ',');
    *out_len = end ? (size_t)(end - s) : strlen(s);
    return s;
}

Ec801EGnss::Ec801EGnss(gpio_num_t tx_pin, gpio_num_t rx_pin, uart_port_t uart_num)
    : tx_pin_(tx_pin), rx_pin_(rx_pin), uart_num_(uart_num) {}

void Ec801EGnss::GetGnssLocation(GnssCallback callback, int timeout_seconds) {
    struct TaskArgs {
        gpio_num_t tx_pin, rx_pin;
        uart_port_t uart_num;
        GnssCallback callback;
        int timeout_seconds;
    };
    auto* args = new TaskArgs{tx_pin_, rx_pin_, uart_num_, std::move(callback), timeout_seconds};

    xTaskCreate([](void* arg) {
        auto* a = static_cast<TaskArgs*>(arg);
        const gpio_num_t tx   = a->tx_pin;
        const gpio_num_t rx   = a->rx_pin;
        const uart_port_t port = a->uart_num;
        auto callback          = std::move(a->callback);
        const int timeout_sec  = a->timeout_seconds;
        delete a;

        // 安装 UART driver（最小 buffer，无 event queue）
        if (uart_is_driver_installed(port)) {
            uart_driver_delete(port);
        }
        uart_config_t cfg = {};
        cfg.baud_rate  = 115200;  // 初始波特率，后续自动检测
        cfg.data_bits  = UART_DATA_8_BITS;
        cfg.parity     = UART_PARITY_DISABLE;
        cfg.stop_bits  = UART_STOP_BITS_1;
        cfg.source_clk = UART_SCLK_DEFAULT;
        uart_driver_install(port, 512, 0, 0, nullptr, 0);
        uart_param_config(port, &cfg);
        uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        // 自动检测并对齐波特率
        int detected_baud = detect_baud_rate(port);
        ESP_LOGI(TAG, "使用波特率: %d", detected_baud);

        // 初始化模组
        static const char* const init_cmds[] = {
            "ATE0\r\n",
            "AT+QURCCFG=\"urcport\",\"uart1\"\r\n",
        };
        char flush[64];
        for (auto cmd : init_cmds) {
            uart_write_bytes(port, cmd, strlen(cmd));
            vTaskDelay(pdMS_TO_TICKS(300));
            while (uart_read_bytes(port, (uint8_t*)flush, sizeof(flush), pdMS_TO_TICKS(100)) > 0) {}
        }

        // 检查模组通信
        const char* ate = "ATE0\r\n";
        uart_write_bytes(port, ate, strlen(ate));
        char line[128];
        bool modem_ok = false;
        TickType_t check_end = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
        while (xTaskGetTickCount() < check_end) {
            if (gnss_read_line(port, line, sizeof(line), 200) > 0 &&
                (strncmp(line, "OK", 2) == 0 || strncmp(line, "ATE0", 4) == 0)) {
                modem_ok = true;
                break;
            }
        }
        if (modem_ok) {
            ESP_LOGI(TAG, "模组通信正常 (ATE0 OK)");
        } else {
            ESP_LOGE(TAG, "ATE0 无响应，请检查 UART 引脚或模组供电");
        }

        // 开启 GNSS
        const char* qgps_on = "AT+QGPS=1\r\n";
        uart_write_bytes(port, qgps_on, strlen(qgps_on));
        bool gps_started = false;
        TickType_t gps_end = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
        while (xTaskGetTickCount() < gps_end) {
            if (gnss_read_line(port, line, sizeof(line), 300) > 0) {
                if (strncmp(line, "OK", 2) == 0) { gps_started = true; break; }
                if (strncmp(line, "ERROR", 5) == 0) break;
            }
        }
        ESP_LOGI(TAG, "[GNSS] AT+QGPS=1 %s", gps_started ? "成功，开始搜星" : "失败，模组可能已开启");

        GnssLocation location;
        bool got_fix = false;
        int elapsed = 0;

        while (elapsed < timeout_sec && !got_fix) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            elapsed += 10;

            const char* query = "AT+QGPSLOC?\r\n";
            uart_write_bytes(port, query, strlen(query));

            TickType_t resp_end = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
            while (xTaskGetTickCount() < resp_end && !got_fix) {
                int n = gnss_read_line(port, line, sizeof(line), 300);
                if (n <= 0) continue;

                // +QGPSLOC: utc,lat,lon,hdop,alt,...
                if (strncmp(line, "+QGPSLOC:", 9) != 0) continue;

                const char* data = line + 9;
                while (*data == ' ') data++;

                size_t lat_len = 0, lon_len = 0, alt_len = 0;
                const char* lat_s = csv_field(data, 1, &lat_len);
                const char* lon_s = csv_field(data, 2, &lon_len);
                const char* alt_s = csv_field(data, 4, &alt_len);

                if (!lat_s || !lon_s) continue;

                location.latitude  = ParseQgpsCoordRaw(lat_s, lat_len);
                location.longitude = ParseQgpsCoordRaw(lon_s, lon_len);
                if (alt_s && alt_len > 0) {
                    char alt_buf[16] = {};
                    memcpy(alt_buf, alt_s, alt_len < 15 ? alt_len : 15);
                    location.altitude = atof(alt_buf);
                }
                location.valid = true;
                got_fix = true;
            }

            ESP_LOGI(TAG, "[GNSS] 搜星中... %d/%d 秒", elapsed, timeout_sec);
        }

        // 关闭 GNSS 并释放 UART
        const char* qgps_off = "AT+QGPS=0\r\n";
        uart_write_bytes(port, qgps_off, strlen(qgps_off));
        vTaskDelay(pdMS_TO_TICKS(200));
        uart_driver_delete(port);

        if (got_fix) {
            ESP_LOGI(TAG, "[GNSS] 定位成功: lat=%.6f, lon=%.6f, alt=%.1f",
                     location.latitude, location.longitude, location.altitude);
        } else {
            ESP_LOGW(TAG, "[GNSS] 搜星超时，未获取到定位");
        }
        callback(got_fix, location);
        vTaskDelete(NULL);
    }, "gnss_task", 4096, args, 5, nullptr);
}
