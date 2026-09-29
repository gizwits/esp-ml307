#ifndef _EC801E_AT_MODEM_H_
#define _EC801E_AT_MODEM_H_

#include "at_modem.h"

class Ec801EAtModem : public AtModem {
public:
    Ec801EAtModem(std::shared_ptr<AtUart> at_uart, TcpAccessMode tcp_access_mode);
    ~Ec801EAtModem() override = default;

    bool SetSleepMode(bool enable, int delay_seconds=0) override;
    bool GetNetworkTime(time_t& timestamp) override;

    // GNSS 定位（使用 Quectel AT+QGPS* 指令）
    void GetGnssLocation(GnssCallback callback, int timeout_seconds = 300) override;

    // 实现基类的纯虚函数
    std::unique_ptr<Http> CreateHttp(int connect_id) override;
    std::unique_ptr<Tcp> CreateTcp(int connect_id) override;
    std::unique_ptr<Tcp> CreateSsl(int connect_id) override;
    std::unique_ptr<Udp> CreateUdp(int connect_id) override;
    std::unique_ptr<Mqtt> CreateMqtt(int connect_id) override;
    std::unique_ptr<WebSocket> CreateWebSocket(int connect_id) override;

protected:
    void HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments) override;

private:
    TcpAccessMode tcp_access_mode_;
    bool network_time_query_active_ = false;  // Protected by mutex_.
    bool network_time_valid_ = false;
    time_t network_time_ = 0;
};


#endif // _EC801E_AT_MODEM_H_
