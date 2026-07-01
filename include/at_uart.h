#ifndef _AT_UART_H_
#define _AT_UART_H_

#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <list>
#include <cstdlib>
#include <memory>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>
#include <driver/uart.h>

// UART事件定义
#define AT_EVENT_DATA_AVAILABLE BIT1
#define AT_EVENT_COMMAND_DONE   BIT2
#define AT_EVENT_COMMAND_ERROR  BIT3

// 默认配置
#define UART_NUM                UART_NUM_1

// AT命令参数值结构
struct AtArgumentValue {
    enum class Type { String, Int, Double };
    Type type;
    std::string string_value;
    int int_value;
    double double_value;
    
    std::string ToString() const {
        switch (type) {
            case Type::String:
                return "\"" + string_value + "\"";
            case Type::Int:
                return std::to_string(int_value);
            case Type::Double:
                return std::to_string(double_value);
            default:
                return "";
        }
    }
};

// 数据接收回调函数类型
typedef std::function<void(const std::string& command, const std::vector<AtArgumentValue>& arguments)> UrcCallback;

class AtUart {
public:
    // 构造函数
    AtUart(gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dtr_pin = GPIO_NUM_NC, uart_port_t uart_num = UART_NUM);
    ~AtUart();

    // 初始化和配置
    // rx_buf_size: UART RX ring buffer 大小（4G 用 8192，GNSS-only 用 2048）
    // task_stack:  收发任务 stack 大小（4G 用 8192，GNSS-only 用 3072）
    void Initialize(size_t rx_buf_size = 8192, size_t task_stack = 8192);
    
    // 波特率管理
    bool SetBaudRate(int new_baud_rate);
    int GetBaudRate() const { return baud_rate_; }
    
    // 数据发送
    bool SendData(const char* data, size_t length);
    bool SendCommand(const std::string& command, size_t timeout_ms = 1000, bool add_crlf = true);
    // 新增：原子性地发送命令和数据，避免并发问题
    bool SendCommandWithData(const std::string& command, const char* data, size_t data_length, size_t timeout_ms = 1000);
    const std::string& GetResponse() const { return response_; }
    int GetCmeErrorCode() const { return cme_error_code_; }

    // 非阻塞地预占命令通道：低优先级的轮询任务（如 GNSS）可用它在通道被占用
    // 时直接跳过本轮，而不是阻塞等待，避免和 MQTT/HTTP 等数据通信互相卡顿。
    // 成功后需配对调用 UnlockChannel()；期间该线程内的 SendCommand 调用可正常重入。
    bool TryLockChannel(uint32_t timeout_ms);
    void UnlockChannel();
    
    // 回调管理
    std::list<UrcCallback>::iterator RegisterUrcCallback(UrcCallback callback);
    void UnregisterUrcCallback(std::list<UrcCallback>::iterator iterator);
    
    // 控制接口
    void SetDtrPin(bool high);
    bool IsInitialized() const { return initialized_; }

    std::string EncodeHex(const std::string& data);
    std::string DecodeHex(const std::string& data);
    void EncodeHexAppend(std::string& dest, const char* data, size_t length);
    void DecodeHexAppend(std::string& dest, const char* data, size_t length);

private:
    // 配置参数
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    gpio_num_t dtr_pin_;
    uart_port_t uart_num_;
    int baud_rate_;
    bool initialized_;
    int cme_error_code_ = 0;
    std::string response_;
    bool wait_for_response_ = false;
    // recursive: TryLockChannel()/UnlockChannel() 可跨多条 SendCommand 预占通道，
    // timed: 支持 GNSS 等低优先级轮询做非阻塞的“忙则跳过”判断
    std::recursive_timed_mutex command_mutex_;
    std::mutex mutex_;
    
    // FreeRTOS 对象
    TaskHandle_t event_task_handle_ = nullptr;
    TaskHandle_t receive_task_handle_ = nullptr;
    QueueHandle_t event_queue_handle_;
    EventGroupHandle_t event_group_handle_;
    
    std::string rx_buffer_;
    
    // 回调函数
    std::list<UrcCallback> urc_callbacks_;
    
    // 内部方法
    void EventTask();
    void ReceiveTask();
    bool ParseResponse();
    bool DetectBaudRate();
    // 处理 AT 命令
    void HandleCommand(const char* command);
    // 处理 URC
    void HandleUrc(const std::string& command, const std::vector<AtArgumentValue>& arguments);

    // 静态任务函数
    static void EventTaskWrapper(void* arg);
};

#endif // _AT_UART_H_
