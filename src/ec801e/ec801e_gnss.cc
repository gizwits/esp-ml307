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

        // 使用 mode=2（十进制度）让 AtUart 直接解析 double_value，避免 N/S/E/W 字母丢失的问题
        auto urc_cb = at_uart->RegisterUrcCallback(
            [&location, &got_fix](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
                if (command == "QGPSLOC" && arguments.size() >= 5) {
                    double lat = arguments[1].double_value;
                    double lon = arguments[2].double_value;
                    if (lat == 0.0 && lon == 0.0) return;  // 无效坐标，丢弃
                    location.latitude  = lat;
                    location.longitude = lon;
                    location.altitude  = arguments[4].double_value;
                    location.valid     = true;
                    got_fix = true;
                }
            });

        // 与 Wi-Fi 模式对齐：配置 NMEA 类型、星系、nmeasrc、AP-Flash 热启动
        at_uart->SendCommand("AT+QGPSCFG=\"gpsnmeatype\",31", 1000);  // GGA+RMC+GSV+GSA+VTG
        at_uart->SendCommand("AT+QGPSCFG=\"gnssconfig\",1", 1000);    // GPS+BDS
        at_uart->SendCommand("AT+QGPSCFG=\"nmeasrc\",1", 1000);
        at_uart->SendCommand("AT+QGPSCFG=\"apflash\",1", 1000);       // 关闭时保存星历，下次热启动

        // 检查 GNSS 是否已开启，避免重复开启报错
        volatile bool gnss_already_on = false;
        {
            auto status_urc = at_uart->RegisterUrcCallback(
                [&gnss_already_on](const std::string& cmd, const std::vector<AtArgumentValue>& args) {
                    if (cmd == "QGPS" && !args.empty() && args[0].int_value == 1) {
                        gnss_already_on = true;
                    }
                });
            at_uart->SendCommand("AT+QGPS?", 1000);
            at_uart->UnregisterUrcCallback(status_urc);
        }

        if (gnss_already_on) {
            ESP_LOGI(TAG, "[GNSS] GNSS已开启，继续搜星");
        } else if (at_uart->SendCommand("AT+QGPS=1", 3000)) {
            ESP_LOGI(TAG, "[GNSS] AT+QGPS=1 成功，开始搜星");
        } else {
            ESP_LOGW(TAG, "[GNSS] AT+QGPS=1 失败 (CME:%d)", at_uart->GetCmeErrorCode());
        }

        // NMEA URC 解析辅助：打印并解析 GSV/GSA 诊断字段
        auto parse_diag_nmea = [&location](const std::string& cmd, const std::vector<AtArgumentValue>& args) {
            if (cmd != "QGPSGNMEA" || args.empty()) return;
            std::string nmea;
            for (size_t i = 0; i < args.size(); i++) {
                if (i > 0) nmea += ',';
                nmea += args[i].string_value;
            }
            ESP_LOGI(TAG, "[GNSS][NMEA] +QGPSGNMEA: %s", nmea.c_str());
            const std::string& type = args[0].string_value;
            if (type.size() < 6) return;
            // GSV: 可见卫星总数（field 3 = numSV，整数）
            if (type[3] == 'G' && type[4] == 'S' && type[5] == 'V' && args.size() > 3) {
                int nsv = args[3].int_value;
                if (nsv > location.satellites_visible) location.satellites_visible = nsv;
            }
            // GSA: 参与定位卫星数（fields 3-14）+ PDOP(15) + HDOP(16)
            // 多条 GSA（每个星系一条），取最大卫星数，取最小非零 DOP（质量最好）
            if (type[3] == 'G' && type[4] == 'S' && type[5] == 'A') {
                int used = 0;
                for (size_t f = 3; f <= 14 && f < args.size(); f++)
                    if (!args[f].string_value.empty()) used++;
                if (used > location.satellites_used) location.satellites_used = used;
                if (args.size() > 15) {
                    float pdop = (float)args[15].double_value;
                    if (pdop > 0.0f && (location.pdop == 0.0f || pdop < location.pdop))
                        location.pdop = pdop;
                }
                if (args.size() > 16) {
                    float hdop = (float)args[16].double_value;
                    if (hdop > 0.0f && (location.hdop == 0.0f || hdop < location.hdop))
                        location.hdop = hdop;
                }
            }
        };

        int elapsed = 0;
        while (elapsed < timeout_seconds && !got_fix) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            elapsed += 10;

            // 先查 GSV/GSA 收集诊断数据，必须在 QGPSLOC 之前，否则 QGPSLOC 一旦成功
            // 设置 got_fix=true 后就不会再查 NMEA，导致卫星数永远为 0
            {
                auto diag_urc = at_uart->RegisterUrcCallback(parse_diag_nmea);
                for (const char* t : {"GSV", "GSA"}) {
                    if (!at_uart->SendCommand(std::string("AT+QGPSGNMEA=\"") + t + "\"", 1500)) {
                        ESP_LOGW(TAG, "[GNSS][NMEA] %s 查询失败: +CME ERROR: %d", t, at_uart->GetCmeErrorCode());
                    }
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                at_uart->UnregisterUrcCallback(diag_urc);
            }

            bool loc_ok = at_uart->SendCommand("AT+QGPSLOC=2", 3000);
            int loc_cme = at_uart->GetCmeErrorCode();

            if (!got_fix) {
                // RMC 降级：QGPSLOC 未给出坐标时尝试 RMC
                auto rmc_urc = at_uart->RegisterUrcCallback(
                    [&location, &got_fix](const std::string& cmd, const std::vector<AtArgumentValue>& args) {
                        if (cmd != "QGPSGNMEA" || args.empty()) return;
                        std::string nmea;
                        for (size_t i = 0; i < args.size(); i++) {
                            if (i > 0) nmea += ',';
                            nmea += args[i].string_value;
                        }
                        ESP_LOGI(TAG, "[GNSS][NMEA] +QGPSGNMEA: %s", nmea.c_str());
                        const std::string& type = args[0].string_value;
                        if (type.size() < 6 || type[3] != 'R' || type[4] != 'M' || type[5] != 'C') return;
                        if (args.size() < 7 || args[1].string_value.empty() ||
                            args[3].string_value.empty() || args[4].string_value.empty() ||
                            args[5].string_value.empty() || args[6].string_value.empty()) return;
                        std::string lat_s = args[3].string_value + args[4].string_value;
                        std::string lon_s = args[5].string_value + args[6].string_value;
                        double lat = ParseQgpsCoord(lat_s);
                        double lon = ParseQgpsCoord(lon_s);
                        if (lat == 0.0 && lon == 0.0) return;
                        location.latitude  = lat;
                        location.longitude = lon;
                        location.altitude  = 0.0;
                        location.valid     = false;
                        got_fix = true;
                        ESP_LOGW(TAG, "[GNSS] RMC 降级定位: lat=%.6f, lon=%.6f", lat, lon);
                    });
                if (!at_uart->SendCommand("AT+QGPSGNMEA=\"RMC\"", 1500)) {
                    ESP_LOGW(TAG, "[GNSS][NMEA] RMC 查询失败: +CME ERROR: %d", at_uart->GetCmeErrorCode());
                }
                vTaskDelay(pdMS_TO_TICKS(200));
                at_uart->UnregisterUrcCallback(rmc_urc);

                if (loc_ok) {
                    ESP_LOGI(TAG, "[GNSS] 搜星中... %d/%d 秒（已收到QGPSLOC响应但未固定）", elapsed, timeout_seconds);
                } else {
                    ESP_LOGW(TAG, "[GNSS] 搜星中... %d/%d 秒（QGPSLOC CME错误码: %d）", elapsed, timeout_seconds, loc_cme);
                }
            }
        }

        // 与 Wi-Fi 模式对齐：只在本次开启时才关闭 GNSS
        if (!gnss_already_on) {
            at_uart->SendCommand("AT+QGPS=0", 2000);
        }
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

        // 初始化模组并配置GNSS
        static const char* const init_cmds[] = {
            "ATE0\r\n",
            "AT+QURCCFG=\"urcport\",\"uart1\"\r\n",
            "AT+QGPSCFG=\"gpsnmeatype\",31\r\n",  // GGA+RMC+GSV+GSA+VTG
            "AT+QGPSCFG=\"gnssconfig\",1\r\n",   // GPS+BDS
            "AT+QGPSCFG=\"nmeasrc\",1\r\n",      // 允许通过AT+QGPSGNMEA获取NMEA语句
            // AP-Flash: GNSS关闭时自动保存星历到Flash（有效期1小时），下次启动为热启动(TTFF<5s)
            "AT+QGPSCFG=\"apflash\",1\r\n",
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
        char line[256];
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

        // 查询GNSS配置状态
        ESP_LOGI(TAG, "[诊断] 查询GNSS配置...");
        const char* query_cfg = "AT+QGPSCFG?\r\n";
        uart_write_bytes(port, query_cfg, strlen(query_cfg));
        vTaskDelay(pdMS_TO_TICKS(500));
        while (gnss_read_line(port, line, sizeof(line), 200) > 0) {
            if (strncmp(line, "+QGPSCFG:", 9) == 0) {
                ESP_LOGI(TAG, "[诊断] 配置: %s", line);
            }
        }

        // 查询GNSS是否已开启
        const char* query_status = "AT+QGPS?\r\n";
        uart_write_bytes(port, query_status, strlen(query_status));
        vTaskDelay(pdMS_TO_TICKS(300));
        bool gnss_already_on = false;
        while (gnss_read_line(port, line, sizeof(line), 200) > 0) {
            if (strncmp(line, "+QGPS:", 6) == 0) {
                ESP_LOGI(TAG, "[诊断] GNSS状态: %s", line);
                if (strstr(line, ",1") != nullptr) {
                    gnss_already_on = true;
                }
            }
        }

        // 开启 GNSS（如果尚未开启）
        bool gps_started = gnss_already_on;
        if (!gnss_already_on) {
            const char* qgps_on = "AT+QGPS=1\r\n";
            uart_write_bytes(port, qgps_on, strlen(qgps_on));
            TickType_t gps_end = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
            while (xTaskGetTickCount() < gps_end) {
                if (gnss_read_line(port, line, sizeof(line), 300) > 0) {
                    if (strncmp(line, "OK", 2) == 0) { gps_started = true; break; }
                    if (strncmp(line, "ERROR", 5) == 0) break;
                }
            }
            ESP_LOGI(TAG, "[GNSS] AT+QGPS=1 %s", gps_started ? "成功，开始搜星" : "失败");
        } else {
            ESP_LOGI(TAG, "[GNSS] GNSS已开启，继续搜星");
        }

        GnssLocation location;
        bool got_fix = false;
        int elapsed = 0;

        // 解析一条 +QGPSGNMEA: 行，更新 GSV/GSA 诊断字段
        auto parse_diag_nmea_line = [&location](const char* line) {
            const char* nmea = line + 11;
            while (*nmea == ' ') nmea++;
            if (nmea[0] != '$' || strlen(nmea) < 6) return;

            if (nmea[3] == 'G' && nmea[4] == 'S' && nmea[5] == 'V') {
                size_t nsv_len;
                const char* nsv_f = csv_field(nmea, 3, &nsv_len);
                if (nsv_f && nsv_len > 0) {
                    char buf[8] = {};
                    memcpy(buf, nsv_f, nsv_len < 7 ? nsv_len : 7);
                    int nsv = atoi(buf);
                    if (nsv > location.satellites_visible) location.satellites_visible = nsv;
                }
            }
            if (nmea[3] == 'G' && nmea[4] == 'S' && nmea[5] == 'A') {
                int used = 0;
                for (int f = 3; f <= 14; f++) {
                    size_t flen;
                    const char* fp = csv_field(nmea, f, &flen);
                    if (fp && flen > 0) used++;
                }
                if (used > location.satellites_used) location.satellites_used = used;
                size_t pdop_len, hdop_len;
                const char* pdop_f = csv_field(nmea, 15, &pdop_len);
                const char* hdop_f = csv_field(nmea, 16, &hdop_len);
                if (pdop_f && pdop_len > 0) {
                    char buf[16] = {};
                    memcpy(buf, pdop_f, pdop_len < 15 ? pdop_len : 15);
                    float pdop = (float)atof(buf);
                    if (pdop > 0.0f && (location.pdop == 0.0f || pdop < location.pdop))
                        location.pdop = pdop;
                }
                if (hdop_f && hdop_len > 0) {
                    char buf[16] = {};
                    memcpy(buf, hdop_f, hdop_len < 15 ? hdop_len : 15);
                    float hdop = (float)atof(buf);
                    if (hdop > 0.0f && (location.hdop == 0.0f || hdop < location.hdop))
                        location.hdop = hdop;
                }
            }
        };

        while (elapsed < timeout_sec && !got_fix) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            elapsed += 10;

            // 先查 GSV/GSA 收集诊断数据，必须在 QGPSLOC 之前，否则 QGPSLOC 一旦成功
            // 设置 got_fix=true 后就不会再查 NMEA，导致卫星数永远为 0
            // 注意：Wi-Fi 路径是同步 UART，模组响应有延迟，两条命令连发后统一读取
            uart_write_bytes(port, "AT+QGPSGNMEA=\"GSV\"\r\n", 20);
            vTaskDelay(pdMS_TO_TICKS(100));
            uart_write_bytes(port, "AT+QGPSGNMEA=\"GSA\"\r\n", 20);
            {
                int nmea_count = 0;
                TickType_t nmea_end = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
                while (xTaskGetTickCount() < nmea_end) {
                    int n = gnss_read_line(port, line, sizeof(line), 250);
                    if (n <= 0) continue;
                    if (strncmp(line, "+QGPSGNMEA:", 11) == 0) {
                        ESP_LOGI(TAG, "[GNSS][NMEA] %s", line);
                        parse_diag_nmea_line(line);
                        nmea_count++;
                    } else if (strncmp(line, "+CME ERROR:", 11) == 0) {
                        ESP_LOGW(TAG, "[GNSS][NMEA] 查询失败: %s", line);
                    }
                }
                ESP_LOGI(TAG, "[GNSS] GSV/GSA 收到 %d 条消息, sats_vis=%d sats_used=%d hdop=%.1f",
                         nmea_count, location.satellites_visible, location.satellites_used, location.hdop);
            }

            // QGPSLOC（mode=0: ddmm.mmmmm格式，与ParseQgpsCoordRaw匹配）
            bool loc_has_response = false;
            bool loc_cme_error = false;
            int cme_code = -1;
            uart_write_bytes(port, "AT+QGPSLOC=0\r\n", 14);
            TickType_t resp_end = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
            while (xTaskGetTickCount() < resp_end && !got_fix) {
                int n = gnss_read_line(port, line, sizeof(line), 300);
                if (n <= 0) continue;
                if (strncmp(line, "+QGPSLOC:", 9) == 0) {
                    loc_has_response = true;
                    const char* data = line + 9;
                    while (*data == ' ') data++;
                    size_t lat_len = 0, lon_len = 0, alt_len = 0;
                    const char* lat_s = csv_field(data, 1, &lat_len);
                    const char* lon_s = csv_field(data, 2, &lon_len);
                    const char* alt_s = csv_field(data, 4, &alt_len);
                    if (!lat_s || !lon_s) continue;
                    double lat = ParseQgpsCoordRaw(lat_s, lat_len);
                    double lon = ParseQgpsCoordRaw(lon_s, lon_len);
                    if (lat == 0.0 && lon == 0.0) continue;
                    location.latitude  = lat;
                    location.longitude = lon;
                    if (alt_s && alt_len > 0) {
                        char alt_buf[16] = {};
                        memcpy(alt_buf, alt_s, alt_len < 15 ? alt_len : 15);
                        location.altitude = atof(alt_buf);
                    }
                    location.valid = true;
                    got_fix = true;
                    break;
                }
                if (strncmp(line, "+CME ERROR:", 11) == 0) {
                    loc_cme_error = true;
                    cme_code = atoi(line + 11);
                    break;
                }
            }

            // RMC 降级（仅在 QGPSLOC 未给出坐标时尝试）
            if (!got_fix) {
                uart_write_bytes(port, "AT+QGPSGNMEA=\"RMC\"\r\n", 20);
                TickType_t rmc_end = xTaskGetTickCount() + pdMS_TO_TICKS(1200);
                while (xTaskGetTickCount() < rmc_end && !got_fix) {
                    int n = gnss_read_line(port, line, sizeof(line), 250);
                    if (n <= 0) continue;
                    if (strncmp(line, "+QGPSGNMEA:", 11) == 0) {
                        ESP_LOGI(TAG, "[GNSS][NMEA] %s", line);
                        const char* nmea = line + 11;
                        while (*nmea == ' ') nmea++;
                        if (nmea[0] == '$' && nmea[3] == 'R' && nmea[4] == 'M' && nmea[5] == 'C') {
                            size_t t_len, lat_len, ns_len, lon_len, ew_len;
                            const char* t_f   = csv_field(nmea, 1, &t_len);
                            const char* lat_f = csv_field(nmea, 3, &lat_len);
                            const char* ns_f  = csv_field(nmea, 4, &ns_len);
                            const char* lon_f = csv_field(nmea, 5, &lon_len);
                            const char* ew_f  = csv_field(nmea, 6, &ew_len);
                            if (t_f && t_len > 0 && lat_f && lat_len > 0 &&
                                ns_f && ns_len > 0 && lon_f && lon_len > 0 && ew_f && ew_len > 0) {
                                char lat_buf[20] = {}, lon_buf[20] = {};
                                size_t ll = lat_len < 18 ? lat_len : 18;
                                size_t el = lon_len < 18 ? lon_len : 18;
                                memcpy(lat_buf, lat_f, ll); lat_buf[ll] = *ns_f;
                                memcpy(lon_buf, lon_f, el); lon_buf[el] = *ew_f;
                                double lat = ParseQgpsCoordRaw(lat_buf, ll + 1);
                                double lon = ParseQgpsCoordRaw(lon_buf, el + 1);
                                if (!(lat == 0.0 && lon == 0.0)) {
                                    location.latitude  = lat;
                                    location.longitude = lon;
                                    location.altitude  = 0.0;
                                    location.valid     = false;
                                    got_fix = true;
                                    ESP_LOGW(TAG, "[GNSS] RMC 降级定位: lat=%.6f, lon=%.6f", lat, lon);
                                }
                            }
                        }
                    } else if (strncmp(line, "OK", 2) == 0 || strncmp(line, "ERROR", 5) == 0 ||
                               strncmp(line, "+CME ERROR:", 11) == 0) {
                        break;
                    }
                }
            }

            if (loc_has_response) {
                ESP_LOGI(TAG, "[GNSS] 搜星中... %d/%d 秒（已收到QGPSLOC响应但未固定）", elapsed, timeout_sec);
            } else if (loc_cme_error) {
                ESP_LOGW(TAG, "[GNSS] 搜星中... %d/%d 秒（QGPSLOC CME错误码: %d）", elapsed, timeout_sec, cme_code);
            } else if (!got_fix) {
                ESP_LOGW(TAG, "[GNSS] 搜星中... %d/%d 秒（QGPSLOC无有效响应）", elapsed, timeout_sec);
            }
        }

        // 仅在本函数开启GNSS时才关闭，避免影响外部状态
        if (!gnss_already_on) {
            const char* qgps_off = "AT+QGPS=0\r\n";
            uart_write_bytes(port, qgps_off, strlen(qgps_off));
            vTaskDelay(pdMS_TO_TICKS(200));
        }
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
