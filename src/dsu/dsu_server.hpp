#pragma once
#include "config/config.hpp"
#include "config/constants.hpp"
#include "network/udp_server.hpp"
#include "psmove/psmove_manager.hpp"
#include "dsu/dsu_protocol.hpp"
#include <array>
#include <chrono>
#include <vector>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>

// A DSU client (usually an emulator) as seen by the GUI.
struct ClientSnapshot {
    std::string address;               // "127.0.0.1:54321"
    double seconds_since_seen = 0;
    double seconds_since_data_request = -1;  // -1 = never asked for data
    uint64_t version_requests = 0;
    uint64_t info_requests = 0;
    uint64_t data_requests = 0;
    uint64_t packets_sent = 0;
    bool subscribed_all = false;       // Active "all controllers" subscription
    std::array<bool, Constants::MAX_CONTROLLERS> subscribed_slots{};  // Slots it currently receives
    std::string last_request;          // Human-readable last DATA registration
};

struct ServerStatus {
    bool listening = false;
    int port = Constants::DSU_PORT;
    std::string error;                 // Why the UDP port couldn't be opened
    uint64_t packets_sent = 0;
    double uptime_s = 0;
};

class DSUServer {
public:
    DSUServer(const Config& config);
    ~DSUServer();

    // If require_network is false, a failure to open the UDP port is reported
    // through get_status() instead of failing start(), so the GUI can show it
    // and offer a retry.
    bool start(bool require_network = true);
    void run();
    void stop();
    bool retry_network();

    // Mapping settings can be changed while running.
    Config get_config();
    void set_config(const Config& config);

    PSMoveManager& controllers() { return psmove_manager_; }
    ServerStatus get_status();
    std::vector<ClientSnapshot> get_clients();
    uint64_t slot_mac(int slot) const { return base_mac_ + (static_cast<uint64_t>(slot) << 8); }

private:
    using Clock = std::chrono::steady_clock;

    struct Client {
        sockaddr_in addr{};
        std::string addr_str;
        Clock::time_point first_seen;
        Clock::time_point last_seen;
        Clock::time_point last_data_request;
        bool has_data_request = false;
        Clock::time_point all_until;                                  // Subscription to every slot
        std::array<Clock::time_point, Constants::MAX_CONTROLLERS> slot_until{};  // Per-slot subscriptions
        bool was_receiving = false;
        uint64_t version_requests = 0;
        uint64_t info_requests = 0;
        uint64_t data_requests = 0;
        uint64_t packets_sent = 0;
        std::string last_request;
    };

    void handle_message(const uint8_t* data, size_t len, const sockaddr_in& from);
    void handle_data_request(Client& client, const uint8_t* data, size_t len, Clock::time_point now);
    // Slots named by the 8-byte controller header (flags, slot, MAC) that DATA
    // and the rumble messages share. Flags 0 = every slot.
    std::vector<int> slots_in_header(const uint8_t* data, size_t len) const;
    Client& find_or_add_client(const sockaddr_in& from, Clock::time_point now);
    void send_thread_main();
    void status_thread_main();

    Config config_;
    std::mutex config_mutex_;
    std::atomic<uint64_t> config_version_{0};

    uint32_t server_id_;
    uint64_t base_mac_;

    UDPServer udp_server_;
    PSMoveManager psmove_manager_;

    std::vector<Client> clients_;
    std::mutex subscription_mutex_;  // Guards clients_

    std::thread send_thread_;
    std::thread status_thread_;
    std::atomic<bool> running_;
    std::atomic<uint64_t> packets_sent_{0};
    Clock::time_point start_time_;

    uint32_t packet_counters_[Constants::MAX_CONTROLLERS];
};
