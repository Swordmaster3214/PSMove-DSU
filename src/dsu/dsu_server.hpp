#pragma once
#include "config/config.hpp"
#include "network/udp_server.hpp"
#include "psmove/psmove_manager.hpp"
#include "dsu/dsu_protocol.hpp"
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

struct Subscription {
    sockaddr_in addr{};
};

class DSUServer {
public:
    DSUServer(const Config& config);
    ~DSUServer();
    
    bool start();
    void run();
    void stop();
    
private:
    void handle_message(const uint8_t* data, size_t len, const sockaddr_in& from);
    void send_thread_main();
    void status_thread_main();
    
    Config config_;
    uint32_t server_id_;
    uint64_t base_mac_;
    
    UDPServer udp_server_;
    PSMoveManager psmove_manager_;
    
    std::vector<Subscription> subscriptions_;
    std::mutex subscription_mutex_;
    
    std::thread send_thread_;
    std::thread status_thread_;
    std::atomic<bool> running_;
    
    uint32_t packet_counters_[Constants::MAX_CONTROLLERS];
};
