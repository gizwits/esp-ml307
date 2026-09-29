#include "ec801e_at_modem.h"
#include "ec801e_gnss.h"
#include "ec801e_network_time.h"
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


Ec801EAtModem::Ec801EAtModem(std::shared_ptr<AtUart> at_uart,
                             TcpAccessMode tcp_access_mode)
    : AtModem(at_uart), tcp_access_mode_(tcp_access_mode) {
    // 子类特定的初始化在这里
    // ATE0 关闭 echo
    at_uart_->SendCommand("ATE0");
    // 设置 URC 端口为 UART1
    at_uart_->SendCommand("AT+QURCCFG=\"urcport\",\"uart1\"");
}

void Ec801EAtModem::HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments) {
    if (command == "QLTS") {
        std::lock_guard<std::mutex> lock(mutex_);
        if (network_time_query_active_) {
            network_time_valid_ = arguments.size() == 1 &&
                Ec801EParseNetworkTime(arguments[0].string_value, network_time_);
        }
        return;
    }
    AtModem::HandleUrc(command, arguments);
}

bool Ec801EAtModem::GetNetworkTime(time_t& timestamp) {
    // A UART callback cannot wait for a reply handled by the same UART worker.
    // Never queue behind a busy SSE/MQTT command: the caller can retry later.
    if (at_uart_->IsInUartTask() || !at_uart_->TryLockChannel(0)) return false;
    struct ChannelGuard {
        AtUart* uart;
        ~ChannelGuard() { uart->UnlockChannel(); }
    } guard{at_uart_.get()};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        network_time_valid_ = false;
        network_time_query_active_ = true;
    }
    // The manual specifies 300ms; allow 500ms for UART transport/parsing.
    const bool ok = at_uart_->SendCommand("AT+QLTS=1", 500);
    std::lock_guard<std::mutex> lock(mutex_);
    network_time_query_active_ = false;
    if (!ok || !network_time_valid_) return false;
    timestamp = network_time_;
    return true;
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
    return std::make_unique<Ec801ETcp>(at_uart_, connect_id, tcp_access_mode_);
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
    Ec801ERunGnssTask(at_uart_, std::move(callback), timeout_seconds);
}
