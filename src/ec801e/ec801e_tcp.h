#ifndef EC801E_TCP_H
#define EC801E_TCP_H

#include "tcp.h"
#include "at_uart.h"

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <atomic>
#include <string>

#define EC801E_TCP_CONNECTED BIT0
#define EC801E_TCP_DISCONNECTED BIT1
#define EC801E_TCP_ERROR BIT2
#define EC801E_TCP_SEND_COMPLETE BIT3
#define EC801E_TCP_SEND_FAILED BIT4
#define EC801E_TCP_INITIALIZED BIT5
#define EC801E_TCP_DATA_AVAILABLE BIT6
#define EC801E_TCP_RECEIVE_TASK_STOP BIT7
#define EC801E_TCP_RECEIVE_TASK_STOPPED BIT8

#define TCP_CONNECT_TIMEOUT_MS 10000

class Ec801ETcp : public Tcp {
public:
    Ec801ETcp(std::shared_ptr<AtUart> at_uart, int tcp_id, TcpAccessMode access_mode);
    ~Ec801ETcp();

    bool Connect(const std::string& host, int port) override;
    void Disconnect() override;
    int Send(const std::string& data) override;
    void SetReceiveTaskPriority(unsigned int priority) override;

private:
    std::shared_ptr<AtUart> at_uart_;
    int tcp_id_;
    TcpAccessMode access_mode_;
    bool instance_active_ = false;
    EventGroupHandle_t event_group_handle_;
    std::list<UrcCallback>::iterator urc_callback_it_;
    TaskHandle_t receive_task_handle_ = nullptr;
    std::atomic<bool> qird_in_progress_{false};
    std::atomic<int> last_qird_length_{-1};

    void ReceiveTask();
    void StopReceiveTask();
};

#endif // EC801E_TCP_H
