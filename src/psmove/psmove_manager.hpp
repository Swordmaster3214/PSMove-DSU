#pragma once
#include "dsu/dsu_protocol.hpp"
#include "config/constants.hpp"
#include <psmove.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>
#include <string>

#ifndef _WIN32
#include <pthread.h>
#include <sched.h>
#endif

class PSMoveManager {
public:
    PSMoveManager();
    ~PSMoveManager();
    
    bool start();
    void stop();
    
    std::optional<MoveSample> get_latest_for_slot(int slot);
    int get_battery_level_for_slot(int slot);
    bool has_controller_for_slot(int slot);
    std::string get_controller_serial_for_slot(int slot);
    int get_connected_count();
    
private:
    void thread_main();
    void scan_for_controllers();
    std::string get_serial_safe(PSMove* controller);
    std::string get_connection_type_string(PSMove* controller);
    int find_serial_slot(const std::string& serial);
    
    PSMove* controllers_[Constants::MAX_CONTROLLERS];
    bool has_data_[Constants::MAX_CONTROLLERS];
    
    // OPTIMIZATION: Lock-free double buffering for sample data
    MoveSample samples_[Constants::MAX_CONTROLLERS][2];  // Double buffer per controller
    std::atomic<int> current_buffer_[Constants::MAX_CONTROLLERS];  // Which buffer is current
    std::atomic<MoveSample*> latest_atomic_[Constants::MAX_CONTROLLERS];  // Atomic pointer to current data
    
    std::thread worker_;
    std::atomic<bool> running_;
    std::mutex mtx_;  // Only for controller management, not data access
};
