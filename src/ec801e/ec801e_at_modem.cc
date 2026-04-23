#include "ec801e_at_modem.h"
#include <esp_log.h>
#include <esp_err.h>
#include <cassert>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include "ec801e_ssl.h"
#include "ec801e_tcp.h"
#include "ec801e_udp.h"
#include "ec801e_mqtt.h"
#include "http_client.h"
#include "web_socket.h"

#define TAG "Ec801EAtModem"

// 将 Quectel NMEA ddmm.mmmm[N/S/E/W] 格式转为十进制度
static double ParseQgpsCoord(const std::string& s) {
    if (s.size() < 2) return 0.0;
    char dir = s.back();
    std::string num = s.substr(0, s.size() - 1);
    double raw = std::atof(num.c_str());
    int degrees = static_cast<int>(raw / 100);
    double minutes = raw - degrees * 100;
    double decimal = degrees + minutes / 60.0;
    if (dir == 'S' || dir == 'W') decimal = -decimal;
    return decimal;
}

// 当 AT 解析器把 "ddmm.mmmmN" 误识别成 double 时，退化按正半球解析（国内场景通常可用）
static double ParseQgpsCoordFromDdmm(double raw) {
    int degrees = static_cast<int>(raw / 100);
    double minutes = raw - degrees * 100;
    return degrees + minutes / 60.0;
}


Ec801EAtModem::Ec801EAtModem(std::shared_ptr<AtUart> at_uart) : AtModem(at_uart) {
    // 子类特定的初始化在这里
    // ATE0 关闭 echo
    at_uart_->SendCommand("ATE0");
    // 设置 URC 端口为 UART1
    at_uart_->SendCommand("AT+QURCCFG=\"urcport\",\"uart1\"");
}

void Ec801EAtModem::HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments) {
    AtModem::HandleUrc(command, arguments);
}

bool Ec801EAtModem::SetSleepMode(bool enable, int delay_seconds) {
    if (enable) {
        if (delay_seconds > 0) {
            at_uart_->SendCommand("AT+QSCLKEX=1," + std::to_string(delay_seconds) + ",30");
        }
        return at_uart_->SendCommand("AT+QSCLK=1");
    } else {
        return at_uart_->SendCommand("AT+QSCLK=0");
    }
}

std::unique_ptr<Http> Ec801EAtModem::CreateHttp(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<HttpClient>(this, connect_id);
}

std::unique_ptr<Tcp> Ec801EAtModem::CreateTcp(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ec801ETcp>(at_uart_, connect_id);
}

std::unique_ptr<Tcp> Ec801EAtModem::CreateSsl(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ec801ESsl>(at_uart_, connect_id);
}

std::unique_ptr<Udp> Ec801EAtModem::CreateUdp(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ec801EUdp>(at_uart_, connect_id);
}

std::unique_ptr<Mqtt> Ec801EAtModem::CreateMqtt(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<Ec801EMqtt>(at_uart_, connect_id);
}

std::unique_ptr<WebSocket> Ec801EAtModem::CreateWebSocket(int connect_id) {
    assert(connect_id >= 0);
    return std::make_unique<WebSocket>(this, connect_id);
}

void Ec801EAtModem::GetGnssLocation(GnssCallback callback, int timeout_seconds) {
    struct TaskArgs {
        std::shared_ptr<AtUart> at_uart;
        GnssCallback callback;
        int timeout_seconds;
    };
    auto* args = new TaskArgs{at_uart_, std::move(callback), timeout_seconds};

    xTaskCreate([](void* arg) {
        auto* args = static_cast<TaskArgs*>(arg);
        auto at_uart = args->at_uart;
        auto callback = std::move(args->callback);
        int timeout_seconds = args->timeout_seconds;
        delete args;

        GnssLocation location;
        volatile bool got_fix = false;

        // 注册 URC 回调捕获 +QGPSLOC 响应
        // AT+QGPSLOC? 格式: +QGPSLOC: <utc>,<lat>,<lon>,<hdop>,<alt>,...
        // lat/lon 形如 "2237.4656N" 含 '.' 会被解析为 Double 类型（string_value 为空）
        // 所以需要兼容：若 string_value 非空用 ParseQgpsCoord，否则直接用 double_value
        auto urc_cb = at_uart->RegisterUrcCallback(
            [&location, &got_fix](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
                if (command == "QGPSLOC" && arguments.size() >= 5) {
                    // 兼容两种格式：
                    // 1. string_value 非空: "2237.4656N" 格式，用 ParseQgpsCoord 解析
                    // 2. string_value 为空: 被解析为 double，直接用 double_value
                    auto parseCoord = [](const AtArgumentValue& arg) -> double {
                        if (!arg.string_value.empty() && arg.string_value.size() > 1) {
                            // ddmm.mmmmN/S/E/W 格式
                            char dir = arg.string_value.back();
                            if (dir == 'N' || dir == 'S' || dir == 'E' || dir == 'W') {
                                std::string num = arg.string_value.substr(0, arg.string_value.size() - 1);
                                double raw = std::atof(num.c_str());
                                int degrees = static_cast<int>(raw / 100);
                                double minutes = raw - degrees * 100;
                                double decimal = degrees + minutes / 60.0;
                                if (dir == 'S' || dir == 'W') decimal = -decimal;
                                return decimal;
                            }
                            // 纯数字字符串
                            return std::atof(arg.string_value.c_str());
                        }
                        // double_value 直接可用
                        return arg.double_value;
                    };
                    location.latitude  = parseCoord(arguments[1]);
                    location.longitude = parseCoord(arguments[2]);
                    location.altitude  = arguments[4].double_value;
                    location.valid = true;
                    got_fix = true;
                    ESP_LOGI("Ec801EAtModem", "[GNSS] URC解析: lat=%.6f, lon=%.6f, alt=%.1f (arg1_str='%s' arg2_str='%s')",
                             location.latitude, location.longitude, location.altitude,
                             arguments[1].string_value.c_str(), arguments[2].string_value.c_str());
                }
            });

        // 开启 GNSS
        bool gps_started = at_uart->SendCommand("AT+QGPS=1");
        if (!gps_started) {
            ESP_LOGW(TAG, "[GNSS] AT+QGPS=1 失败，模组可能不支持或已开启");
            // 尝试查询GPS状态，看看是否已经开启
            at_uart->SendCommand("AT+QGPS?");
        } else {
            ESP_LOGI(TAG, "[GNSS] GPS已成功开启");
        }

        // 轮询定位，每 10 秒查询一次直到超时
        int elapsed = 0;
        while (elapsed < timeout_seconds && !got_fix) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            elapsed += 10;
            // 删除了 got_fix = false; 这行会导致定位成功标志被重置！
            at_uart->SendCommand("AT+QGPSLOC?");
            vTaskDelay(pdMS_TO_TICKS(500)); // 增加等待时间到500ms，确保URC到达
            ESP_LOGI(TAG, "[GNSS] 搜星中... %d/%d 秒 (got_fix=%d)", elapsed, timeout_seconds, got_fix);
            if (got_fix) {
                ESP_LOGI(TAG, "[GNSS] 检测到定位成功，退出搜星循环");
                break;
            }
        }

        // 注意：不立即关闭GPS，让它保持开启状态以便下次快速定位
        // 如果需要省电，可以在更高层逻辑中决定何时关闭
        // at_uart->SendCommand("AT+QGPS=0", 2000);
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