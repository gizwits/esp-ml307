#pragma once
#include "at_uart.h"
#include "at_modem.h"
#include <driver/gpio.h>
#include <driver/uart.h>

// Wi-Fi 模式下的轻量 GNSS 接口
// 直接操作 UART driver，无后台任务，查询结束自动释放 UART
// 用法：
//   auto gnss = std::make_unique<Ec801EGnss>(ML307_TX_PIN, ML307_RX_PIN, UART_NUM_2);
//   gnss->GetGnssLocation([](bool ok, GnssLocation loc) { ... });
class Ec801EGnss {
public:
    Ec801EGnss(gpio_num_t tx_pin, gpio_num_t rx_pin, uart_port_t uart_num = UART_NUM_1);
    void GetGnssLocation(GnssCallback callback, int timeout_seconds = 300);

private:
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    uart_port_t uart_num_;
};

// 4G 模式（Ec801EAtModem）复用的底层任务，基于已初始化的 AtUart
void Ec801ERunGnssTask(std::shared_ptr<AtUart> at_uart, GnssCallback callback, int timeout_seconds);
