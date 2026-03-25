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
        // 格式: +QGPSLOC: <utc>,<lat>,<lon>,<hdop>,<alt>,...
        auto urc_cb = at_uart->RegisterUrcCallback(
            [&location, &got_fix](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
                if (command == "QGPSLOC" && arguments.size() >= 5) {
                    location.latitude  = ParseQgpsCoord(arguments[1].string_value);
                    location.longitude = ParseQgpsCoord(arguments[2].string_value);
                    location.altitude  = arguments[4].double_value;
                    location.valid = true;
                    got_fix = true;
                }
            });

        // 开启 GNSS
        if (!at_uart->SendCommand("AT+QGPS=1", 3000)) {
            ESP_LOGW(TAG, "[GNSS] AT+QGPS=1 失败，模组可能不支持或已开启");
        }

        // 轮询定位，每 10 秒查询一次直到超时
        int elapsed = 0;
        while (elapsed < timeout_seconds && !got_fix) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            elapsed += 10;
            got_fix = false;
            at_uart->SendCommand("AT+QGPSLOC?", 3000);
            vTaskDelay(pdMS_TO_TICKS(200)); // 等待 URC 到达
            ESP_LOGI(TAG, "[GNSS] 搜星中... %d/%d 秒", elapsed, timeout_seconds);
            if (got_fix) break;
        }

        // 关闭 GNSS
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