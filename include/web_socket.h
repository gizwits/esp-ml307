#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <functional>
#include <string>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>

#include "tcp.h"

class NetworkInterface;

class WebSocket {
public:
    WebSocket(NetworkInterface* network, int connect_id);
    ~WebSocket();

    void SetHeader(const char* key, const char* value);
    void SetReceiveBufferSize(size_t size);
    bool IsConnected() const;
    bool Connect(const char* uri);
    bool Send(const std::string& data);
    bool Send(const void* data, size_t len, bool binary = false, bool fin = true);
    void Ping();
    void Close();

    void OnConnected(std::function<void()> callback);
    void OnDisconnected(std::function<void(bool is_clean)> callback);
    void OnData(std::function<void(const char*, size_t, bool binary)> callback);
    void OnError(std::function<void(int)> callback);

private:
    NetworkInterface* network_;
    int connect_id_;
    // 同一 TCP 连接的数据帧、pong 和 close 帧必须串行。
    // 析构也使用这把锁，确保 Tcp::Send() 返回前不会释放 tcp_。
    mutable std::recursive_mutex send_mutex_;
    std::unique_ptr<Tcp> tcp_;
    bool continuation_ = false;
    size_t receive_buffer_size_ = 2048;
    std::string receive_buffer_;
    bool handshake_completed_ = false;
    std::atomic<bool> connected_{false};
    
    // FreeRTOS 事件组用于同步握手
    EventGroupHandle_t handshake_event_group_;
    static const EventBits_t HANDSHAKE_SUCCESS_BIT = BIT0;
    static const EventBits_t HANDSHAKE_FAILED_BIT = BIT1;

    std::map<std::string, std::string> headers_;
    std::function<void(const char*, size_t, bool binary)> on_data_;
    std::function<void(int)> on_error_;
    std::function<void()> on_connected_;
    std::function<void(bool is_clean)> on_disconnected_;
    std::atomic<bool> is_closing_{false};  // 标记是否主动关闭
    size_t pong_payload_length_ = 0;
    uint8_t pong_payload_[125];
    esp_timer_handle_t pong_timer_ = nullptr;  // 用于管理pong响应的定时器
    
    // WebSocket 帧分片状态（用于处理分片消息）
    std::vector<char> current_message_;
    bool is_fragmented_ = false;
    bool is_binary_ = false;

    void OnTcpData(const std::string& data);
    bool SendControlFrame(uint8_t opcode, const void* data, size_t len);
    void ResetFragmentState();  // 重置分片状态
};

#endif // WEBSOCKET_H
