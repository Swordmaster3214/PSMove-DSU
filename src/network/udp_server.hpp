#pragma once
#include "utils/platform.hpp"
#include <vector>
#include <functional>
#include <atomic>
#include <thread>
#include <string>

class UDPServer {
public:
    using MessageHandler = std::function<void(const uint8_t* data, size_t len, const sockaddr_in& from)>;
    
    UDPServer(int port);
    ~UDPServer();
    
    bool start();
    void stop();
    void send_to(const std::vector<uint8_t>& data, const sockaddr_in& addr);
    
    // NEW: Batch send for better performance
    void send_to_multiple(const std::vector<uint8_t>& data, const std::vector<sockaddr_in>& addresses);
    
    void set_message_handler(MessageHandler handler);
    
    bool is_running() const { return running_.load(); }
    int port() const { return port_; }
    // Reason the last start() failed (empty if it succeeded).
    const std::string& last_error() const { return last_error_; }
    
private:
    void receive_loop();
    
    int port_;
    int sock_;
    std::atomic<bool> running_;
    MessageHandler message_handler_;
    std::thread receive_thread_;
    std::string last_error_;
};
