#ifndef TCP_H
#define TCP_H


#include <string>
#include <functional>

class Tcp {
public:
    virtual ~Tcp() = default;
    virtual bool Connect(const std::string& host, int port) = 0;
    virtual void Disconnect() = 0;
    virtual int Send(const std::string& data) = 0;

    virtual void OnStream(std::function<void(const std::string& data)> callback) {
        stream_callback_ = callback;
    }
    
    virtual void OnDisconnected(std::function<void()> callback) {
        disconnect_callback_ = callback;
    }
    
    // 设置接收限流回调：返回 true 表示可以继续接收，false 表示应该暂停接收
    virtual void OnCanReceive(std::function<bool()> callback) {
        can_receive_callback_ = callback;
    }
    
    // 连接状态查询
    bool connected() const { return connected_; }

protected:
    std::function<void(const std::string& data)> stream_callback_;
    std::function<void()> disconnect_callback_;
    std::function<bool()> can_receive_callback_;  // 接收限流回调
    
    // 连接状态管理
    bool connected_ = false;         // 是否可以正常读写数据
};

#endif // TCP_H
