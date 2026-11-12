#include "dsu/dsu_server.hpp"
#include "utils/logging.hpp"
#include "utils/signal_handler.hpp"
#include "config/constants.hpp"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#include <processthreadsapi.h>
#endif

namespace {
std::string format_addr(const sockaddr_in& a) {
    char ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, const_cast<in_addr*>(&a.sin_addr), ip, sizeof(ip));
    return std::string(ip) + ":" + std::to_string(ntohs(a.sin_port));
}

bool same_addr(const sockaddr_in& a, const sockaddr_in& b) {
    return a.sin_addr.s_addr == b.sin_addr.s_addr && a.sin_port == b.sin_port;
}

std::string format_mac(const uint8_t* m) {
    char buf[18];
    std::snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
    return buf;
}
} // namespace

DSUServer::DSUServer(const Config& config)
    : config_(config)
    , server_id_(0x12345678u)
    , base_mac_(0x665544332211ULL)
    , udp_server_(Constants::DSU_PORT)
    , running_(false) {

    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        packet_counters_[i] = 0;
    }
    psmove_manager_.set_simulated_controllers(config.simulate);
}

DSUServer::~DSUServer() {
    stop();
}

bool DSUServer::start(bool require_network) {
    if (running_.load()) return true;
    start_time_ = Clock::now();

    // Start PSMove manager
    if (!psmove_manager_.start()) {
        Logger::error("Failed to start PSMove manager");
        return false;
    }

    // Start UDP server
    udp_server_.set_message_handler([this](const uint8_t* data, size_t len, const sockaddr_in& from) {
        handle_message(data, len, from);
    });

    if (!udp_server_.start()) {
        Logger::error("Failed to start UDP server");
        if (require_network) {
            psmove_manager_.stop();
            return false;
        }
    }

    running_ = true;

    // Start worker threads
    send_thread_ = std::thread(&DSUServer::send_thread_main, this);

    if (!config_.verbose) {
        status_thread_ = std::thread(&DSUServer::status_thread_main, this);
    }

    Logger::info("DSU server started successfully (optimized for low latency)");
    return true;
}

bool DSUServer::retry_network() {
    if (udp_server_.is_running()) return true;
    Logger::info("Retrying to open UDP port " + std::to_string(udp_server_.port()) + "...");
    return udp_server_.start();
}

void DSUServer::run() {
    Logger::info("Multi-controller DSU server running (optimized timing)...");
    if (!config_.verbose) {
        Logger::info("Use --verbose for detailed logs and latency monitoring.");
    }

    // Wait for shutdown signal
    while (SignalHandler::should_exit() == false && running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    stop();
}

void DSUServer::stop() {
    running_ = false;

    if (send_thread_.joinable()) send_thread_.join();
    if (status_thread_.joinable()) status_thread_.join();

    udp_server_.stop();
    psmove_manager_.stop();
}

Config DSUServer::get_config() {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void DSUServer::set_config(const Config& config) {
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_.copy_runtime_settings(config);
    }
    config_version_.fetch_add(1);
}

ServerStatus DSUServer::get_status() {
    ServerStatus st;
    st.listening = udp_server_.is_running();
    st.port = udp_server_.port();
    if (!st.listening) st.error = udp_server_.last_error();
    st.packets_sent = packets_sent_.load();
    st.uptime_s = running_.load() ? std::chrono::duration<double>(Clock::now() - start_time_).count() : 0;
    return st;
}

std::vector<ClientSnapshot> DSUServer::get_clients() {
    auto now = Clock::now();
    auto secs = [&](Clock::time_point t) { return std::chrono::duration<double>(now - t).count(); };
    std::vector<ClientSnapshot> out;
    std::lock_guard<std::mutex> lock(subscription_mutex_);
    for (const auto& c : clients_) {
        ClientSnapshot s;
        s.address = c.addr_str;
        s.seconds_since_seen = secs(c.last_seen);
        s.seconds_since_data_request = c.has_data_request ? secs(c.last_data_request) : -1;
        s.version_requests = c.version_requests;
        s.info_requests = c.info_requests;
        s.data_requests = c.data_requests;
        s.packets_sent = c.packets_sent;
        s.subscribed_all = now < c.all_until;
        for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
            s.subscribed_slots[i] = s.subscribed_all || now < c.slot_until[i];
        }
        s.last_request = c.last_request;
        out.push_back(s);
    }
    return out;
}

// Caller holds subscription_mutex_.
DSUServer::Client& DSUServer::find_or_add_client(const sockaddr_in& from, Clock::time_point now) {
    for (auto& c : clients_) {
        if (same_addr(c.addr, from)) {
            c.last_seen = now;
            return c;
        }
    }
    Client c;
    c.addr = from;
    c.addr_str = format_addr(from);
    c.first_seen = now;
    c.last_seen = now;
    clients_.push_back(c);
    Logger::info("New DSU client: " + c.addr_str);
    return clients_.back();
}

void DSUServer::handle_message(const uint8_t* data, size_t len, const sockaddr_in& from) {
    if (len < 20) return;
    if (!(data[0] == 'D' && data[1] == 'S' && data[2] == 'U' && data[3] == 'C')) return;

    uint32_t msg_type = uint32_t(data[16]) | (uint32_t(data[17]) << 8) |
                        (uint32_t(data[18]) << 16) | (uint32_t(data[19]) << 24);
    auto now = Clock::now();

    if (msg_type == Constants::MSG_PROTOCOL_VER) {
        {
            std::lock_guard<std::mutex> lock(subscription_mutex_);
            find_or_add_client(from, now).version_requests++;
        }
        auto response = DSUProtocol::make_protocol_response(server_id_);
        udp_server_.send_to(response, from);

    } else if (msg_type == Constants::MSG_INFO) {
        Logger::debug("Client requesting controller info");
        {
            std::lock_guard<std::mutex> lock(subscription_mutex_);
            find_or_add_client(from, now).info_requests++;
        }

        // Payload: int32 port count, then that many slot numbers.
        std::vector<int> slots;
        if (len >= 24) {
            int32_t count = int32_t(uint32_t(data[20]) | (uint32_t(data[21]) << 8) |
                                    (uint32_t(data[22]) << 16) | (uint32_t(data[23]) << 24));
            if (count < 0) count = 0;
            if (count > Constants::MAX_CONTROLLERS) count = Constants::MAX_CONTROLLERS;
            for (int i = 0; i < count && size_t(24 + i) < len; i++) {
                if (data[24 + i] < Constants::MAX_CONTROLLERS) slots.push_back(data[24 + i]);
            }
        } else {
            // Malformed / legacy request: report every slot.
            for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) slots.push_back(slot);
        }

        for (int slot : slots) {
            int battery = psmove_manager_.get_battery_level_for_slot(slot);
            bool connected = psmove_manager_.has_controller_for_slot(slot);

            auto response = DSUProtocol::make_info_response(server_id_, slot, connected, battery, slot_mac(slot));
            udp_server_.send_to(response, from);

            Logger::debug("Replied INFO for slot " + std::to_string(slot) +
                         " (connected: " + (connected ? "yes" : "no") + ")");

            std::this_thread::sleep_for(std::chrono::microseconds(500)); // Reduced delay
        }

    } else if (msg_type == Constants::MSG_DATA) {
        std::lock_guard<std::mutex> lock(subscription_mutex_);
        Client& client = find_or_add_client(from, now);
        handle_data_request(client, data, len, now);

    } else if (msg_type == Constants::MSG_MOTOR_INFO) {
        {
            std::lock_guard<std::mutex> lock(subscription_mutex_);
            find_or_add_client(from, now);
        }
        // The PS Move has one motor.
        for (int slot : slots_in_header(data, len)) {
            bool connected = psmove_manager_.has_controller_for_slot(slot);
            auto response = DSUProtocol::make_motor_info_response(
                server_id_, uint8_t(slot), connected, psmove_manager_.get_battery_level_for_slot(slot),
                slot_mac(slot), connected ? 1 : 0);
            udp_server_.send_to(response, from);
        }

    } else if (msg_type == Constants::MSG_RUMBLE) {
        // Header, then motor id and strength (0-255).
        if (len < 30) return;
        {
            std::lock_guard<std::mutex> lock(subscription_mutex_);
            find_or_add_client(from, now);
        }
        uint8_t motor = data[28];
        uint8_t strength = data[29];
        if (motor != 0) return;
        for (int slot : slots_in_header(data, len)) {
            if (psmove_manager_.has_controller_for_slot(slot)) psmove_manager_.set_rumble(slot, strength);
        }
    }
}

std::vector<int> DSUServer::slots_in_header(const uint8_t* data, size_t len) const {
    std::vector<int> out;
    uint8_t flags = len > 20 ? data[20] : 0;
    bool want[Constants::MAX_CONTROLLERS] = {};
    if (flags == 0) {
        for (bool& w : want) w = true;
    }
    if ((flags & 0x01) && len > 21 && data[21] < Constants::MAX_CONTROLLERS) {
        want[data[21]] = true;
    }
    if ((flags & 0x02) && len >= 28) {
        for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
            uint8_t ours[6];
            DSUProtocol::mac_bytes(slot_mac(slot), ours);
            if (std::memcmp(ours, data + 22, 6) == 0) want[slot] = true;
        }
    }
    for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
        if (want[slot]) out.push_back(slot);
    }
    return out;
}

// Payload: flags (1 = by slot, 2 = by MAC, 0 = everything), slot, 6-byte MAC.
// Clients like Dolphin open one socket per controller and register only that
// slot, and they don't check the slot of the packets they receive, so sending
// every slot to every subscriber would mix the controllers' data together.
void DSUServer::handle_data_request(Client& client, const uint8_t* data, size_t len, Clock::time_point now) {
    const auto until = now + std::chrono::milliseconds(Constants::SUBSCRIPTION_TIMEOUT_MS);
    uint8_t flags = len > 20 ? data[20] : 0;
    std::string desc;

    if (flags == 0) {
        client.all_until = until;
        desc = "all controllers";
    }
    if ((flags & 0x01) && len > 21) {
        uint8_t slot = data[21];
        if (slot < Constants::MAX_CONTROLLERS) {
            client.slot_until[slot] = until;
        }
        desc = "slot " + std::to_string(slot);
    }
    if ((flags & 0x02) && len >= 28) {
        const uint8_t* mac = data + 22;
        bool matched = false;
        for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
            uint8_t ours[6];
            DSUProtocol::mac_bytes(slot_mac(slot), ours);
            if (std::memcmp(ours, mac, 6) == 0) {
                client.slot_until[slot] = until;
                matched = true;
            }
        }
        if (!desc.empty()) desc += " + ";
        desc += "MAC " + format_mac(mac) + (matched ? "" : " (unknown)");
    }

    client.data_requests++;
    client.has_data_request = true;
    client.last_data_request = now;
    if (desc != client.last_request) {
        Logger::info("Client " + client.addr_str + " subscribed to " + desc);
        client.last_request = desc;
    }
}

void DSUServer::send_thread_main() {
    // OPTIMIZATION: Set high thread priority
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#else
    struct sched_param param;
    param.sched_priority = 40; // Lower than PSMove thread but still high
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
#endif

    int diag_counter = 0;
    // Per slot: the sample last sent (by its read time), to spot the first send
    // of each new one, and when we last warned about a slow first send.
    uint64_t last_sent_poll_us[Constants::MAX_CONTROLLERS] = {};
    uint64_t last_latency_warn_us[Constants::MAX_CONTROLLERS] = {};

    // Local copy of the mapping settings. Re-copied (under the mutex) only when
    // the GUI changes something, so the steady state takes no lock for config.
    Config config;
    uint64_t config_version = ~0ull;

    struct Target {
        sockaddr_in addr;
        uint32_t slot_mask;
        uint64_t sent;
    };
    std::vector<Target> targets;
    std::vector<sockaddr_in> addrs;

    while (running_.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        auto now = Clock::now();

        uint64_t v = config_version_.load();
        if (v != config_version) {
            std::lock_guard<std::mutex> lock(config_mutex_);
            config = config_;
            config_version = v;
        }

        // Work out who wants which slots (minimize lock time)
        targets.clear();
        {
            std::lock_guard<std::mutex> lock(subscription_mutex_);
            for (auto it = clients_.begin(); it != clients_.end();) {
                uint32_t mask = 0;
                for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
                    if (now < it->all_until || now < it->slot_until[slot]) mask |= (1u << slot);
                }
                if (it->was_receiving && mask == 0) {
                    Logger::info("Client " + it->addr_str + " stopped renewing its subscription (timed out)");
                }
                it->was_receiving = mask != 0;

                if (mask == 0 && now - it->last_seen > std::chrono::milliseconds(Constants::CLIENT_FORGET_MS)) {
                    it = clients_.erase(it);
                    continue;
                }
                if (mask) targets.push_back({it->addr, mask, 0});
                ++it;
            }
        }

        if (!targets.empty()) {
            int sent_controllers = 0;

            for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
                addrs.clear();
                for (auto& t : targets) {
                    if (t.slot_mask & (1u << slot)) addrs.push_back(t.addr);
                }
                if (addrs.empty()) continue;

                std::optional<MoveSample> sample_opt = psmove_manager_.get_latest_for_slot(slot);
                if (!sample_opt.has_value()) continue;
                MoveSample sample = sample_opt.value();

                // OPTIMIZATION: Set send timestamp here for latency measurement
                sample.send_timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();

                // Latency the server adds: from reading a report to sending it
                // the first time. (Later sends repeat the same sample until the
                // controller's next report, so their age says nothing about us.)
                if (sample.poll_timestamp_us != last_sent_poll_us[slot]) {
                    last_sent_poll_us[slot] = sample.poll_timestamp_us;
                    uint64_t latency = sample.get_latency_us();
                    if (Logger::is_verbose() && latency > uint64_t(Constants::LATENCY_WARNING_US) &&
                        sample.send_timestamp_us - last_latency_warn_us[slot] > 1000000) {
                        last_latency_warn_us[slot] = sample.send_timestamp_us;
                        Logger::warn("Slot " + std::to_string(slot) + ": report sent " +
                                     std::to_string(latency / 1000) + " ms after it was read (send loop delayed)");
                    }
                }

                int battery = psmove_manager_.get_battery_level_for_slot(slot);
                auto data_packet = DSUProtocol::make_data_packet(
                    server_id_, slot, sample, packet_counters_[slot]++, battery, slot_mac(slot), config);

                udp_server_.send_to_multiple(data_packet, addrs);
                packets_sent_.fetch_add(addrs.size(), std::memory_order_relaxed);
                for (auto& t : targets) {
                    if (t.slot_mask & (1u << slot)) t.sent++;
                }
                sent_controllers++;
            }

            // Per-client counters for the GUI
            {
                std::lock_guard<std::mutex> lock(subscription_mutex_);
                for (const auto& t : targets) {
                    if (!t.sent) continue;
                    for (auto& c : clients_) {
                        if (same_addr(c.addr, t.addr)) {
                            c.packets_sent += t.sent;
                            break;
                        }
                    }
                }
            }

            // Debug output with performance metrics
            if (Logger::is_verbose() && sent_controllers > 0 && (++diag_counter % 250) == 0) {
                auto loop_end = std::chrono::high_resolution_clock::now();
                auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                    loop_end - loop_start).count();

                Logger::debug("Sent DATA for " + std::to_string(sent_controllers) +
                             " controllers to " + std::to_string(targets.size()) +
                             " clients (loop: " + std::to_string(loop_duration) + "us)");
            }
        }

        // OPTIMIZATION: High-precision timing
        auto next_time = loop_start + std::chrono::microseconds(Constants::DSU_SEND_MS * 1000);
        std::this_thread::sleep_until(next_time);
    }
}

void DSUServer::status_thread_main() {
    int status_counter = 0;

    while (running_.load()) {
        // Sleep in small steps so stop() doesn't wait up to 5 s.
        for (int i = 0; i < 50 && running_.load(); i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!running_.load()) break;

        int connected_count = psmove_manager_.get_connected_count();
        int subscriber_count = 0;
        {
            auto now = Clock::now();
            std::lock_guard<std::mutex> lock(subscription_mutex_);
            for (const auto& c : clients_) {
                bool active = now < c.all_until;
                for (const auto& t : c.slot_until) active = active || now < t;
                if (active) subscriber_count++;
            }
        }

        if ((++status_counter % 6) == 0 || connected_count > 0 || subscriber_count > 0) {
            Logger::info("Status: " + std::to_string(connected_count) + " controllers, " +
                        std::to_string(subscriber_count) + " clients (optimized mode)");
        }
    }
}
