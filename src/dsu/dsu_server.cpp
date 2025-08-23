#include "dsu/dsu_server.hpp"
#include "utils/logging.hpp"
#include "utils/signal_handler.hpp"
#include "config/constants.hpp"
#include <chrono>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#include <processthreadsapi.h>
#endif

DSUServer::DSUServer(const Config& config) 
    : config_(config)
    , server_id_(0x12345678u)
    , base_mac_(0x665544332211ULL)
    , udp_server_(Constants::DSU_PORT)
    , running_(false) {
    
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        packet_counters_[i] = 0;
    }
}

DSUServer::~DSUServer() {
    stop();
}

bool DSUServer::start() {
    if (running_.load()) return true;
    
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
        psmove_manager_.stop();
        return false;
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

void DSUServer::handle_message(const uint8_t* data, size_t len, const sockaddr_in& from) {
    if (len < 20) return;
    if (!(data[0] == 'D' && data[1] == 'S' && data[2] == 'U' && data[3] == 'C')) return;
    
    uint32_t msg_type = uint32_t(data[16]) | (uint32_t(data[17]) << 8) | 
                        (uint32_t(data[18]) << 16) | (uint32_t(data[19]) << 24);
    
    if (msg_type == Constants::MSG_PROTOCOL_VER) {
        auto response = DSUProtocol::make_protocol_response(server_id_);
        udp_server_.send_to(response, from);
        
    } else if (msg_type == Constants::MSG_INFO) {
        Logger::debug("Client requesting controller info");
        
        // Send info for all controller slots
        for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
            int battery = psmove_manager_.get_battery_level_for_slot(slot);
            bool connected = psmove_manager_.has_controller_for_slot(slot);
            uint64_t slot_mac = base_mac_ + (static_cast<uint64_t>(slot) << 8);
            
            auto response = DSUProtocol::make_info_response(server_id_, slot, connected, battery, slot_mac);
            udp_server_.send_to(response, from);
            
            Logger::debug("Replied INFO for slot " + std::to_string(slot) + 
                         " (connected: " + (connected ? "yes" : "no") + ")");
            
            std::this_thread::sleep_for(std::chrono::microseconds(500)); // Reduced delay
        }
        
    } else if (msg_type == Constants::MSG_DATA) {
        std::lock_guard<std::mutex> lock(subscription_mutex_);
        
        // Check if client is already subscribed
        bool found = false;
        for (const auto& sub : subscriptions_) {
            if (sub.addr.sin_addr.s_addr == from.sin_addr.s_addr && 
                sub.addr.sin_port == from.sin_port) {
                found = true;
                break;
            }
        }
        
        if (!found) {
            Subscription new_sub;
            new_sub.addr = from;
            subscriptions_.push_back(new_sub);
            Logger::info("Client subscribed to DATA. Total: " + std::to_string(subscriptions_.size()));
        }
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
    
    uint64_t t_us = 0;
    int diag_counter = 0;
    int latency_warning_counter = 0;
    
    while (running_.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        
        // Get copy of subscriptions (minimize lock time)
        std::vector<Subscription> subs_copy;
        {
            std::lock_guard<std::mutex> lock(subscription_mutex_);
            subs_copy = subscriptions_;
        }
        
        if (!subs_copy.empty()) {
            t_us += Constants::DSU_SEND_MS * 1000ull;
            
            // OPTIMIZATION: Batch processing of all active controllers
            std::vector<std::pair<int, MoveSample>> active_controllers;
            std::vector<std::pair<int, int>> battery_levels; // slot, battery
            
            // Collect all data in one pass
            for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
                std::optional<MoveSample> sample_opt = psmove_manager_.get_latest_for_slot(slot);
                if (sample_opt.has_value()) {
                    MoveSample sample = sample_opt.value();
                    
                    // OPTIMIZATION: Set send timestamp here for latency measurement
                    sample.send_timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                    
                    active_controllers.emplace_back(slot, sample);
                    battery_levels.emplace_back(slot, psmove_manager_.get_battery_level_for_slot(slot));
                    
                    // Monitor latency
                    if (config_.verbose && sample.poll_timestamp_us > 0) {
                        uint64_t total_latency = sample.get_latency_us();
                        if (total_latency > Constants::LATENCY_WARNING_US && 
                            (++latency_warning_counter % 250) == 0) {
                            Logger::warn("High end-to-end latency for slot " + std::to_string(slot) + 
                                       ": " + std::to_string(total_latency) + "µs");
                        }
                    }
                }
            }
            
            // OPTIMIZATION: Convert subscribers to address vector for batch sending
            std::vector<sockaddr_in> subscriber_addresses;
            for (const auto& sub : subs_copy) {
                subscriber_addresses.push_back(sub.addr);
            }
            
            // Send data for all active controllers
            for (size_t i = 0; i < active_controllers.size(); i++) {
                int slot = active_controllers[i].first;
                const MoveSample& sample = active_controllers[i].second;
                int battery = battery_levels[i].second;
                
                uint64_t slot_mac = base_mac_ + (static_cast<uint64_t>(slot) << 8);
                
                auto data_packet = DSUProtocol::make_data_packet(
                    server_id_, slot, sample, packet_counters_[slot]++, battery, slot_mac, config_);
                
                // OPTIMIZATION: Use batch send
                udp_server_.send_to_multiple(data_packet, subscriber_addresses);
            }
            
            // Debug output with performance metrics
            if (config_.verbose && !active_controllers.empty() && (++diag_counter % 250) == 0) {
                auto loop_end = std::chrono::high_resolution_clock::now();
                auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                    loop_end - loop_start).count();
                
                Logger::debug("Sent DATA for " + std::to_string(active_controllers.size()) + 
                             " controllers to " + std::to_string(subs_copy.size()) + 
                             " clients (loop: " + std::to_string(loop_duration) + "µs)");
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
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        int connected_count = psmove_manager_.get_connected_count();
        int subscriber_count;
        {
            std::lock_guard<std::mutex> lock(subscription_mutex_);
            subscriber_count = static_cast<int>(subscriptions_.size());
        }
        
        if ((++status_counter % 6) == 0 || connected_count > 0 || subscriber_count > 0) {
            Logger::info("Status: " + std::to_string(connected_count) + " controllers, " + 
                        std::to_string(subscriber_count) + " clients (optimized mode)");
        }
    }
}
