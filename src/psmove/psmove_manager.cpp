#include "psmove/psmove_manager.hpp"
#include "utils/logging.hpp"
#include <chrono>
#include <cstdlib>

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>  // For timeBeginPeriod
#pragma comment(lib, "winmm.lib")
#endif

PSMoveManager::PSMoveManager() : running_(false) {
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        controllers_[i] = nullptr;
        has_data_[i] = false;
        current_buffer_[i].store(0);
        latest_atomic_[i].store(nullptr);
    }
}

PSMoveManager::~PSMoveManager() {
    stop();
}

bool PSMoveManager::start() {
    if (running_.load()) return true;
    
#ifdef _WIN32
    // CRITICAL FIX: Set Windows to 1ms timer resolution
    if (timeBeginPeriod(1) != TIMERR_NOERROR) {
        Logger::warn("Failed to set 1ms timer resolution - may have higher latency");
    } else {
        Logger::info("Windows 1ms timer resolution enabled");
    }
#endif
    
    running_ = true;
    worker_ = std::thread(&PSMoveManager::thread_main, this);
    Logger::debug("PSMove manager started with Windows latency fixes");
    return true;
}

void PSMoveManager::stop() {
    running_ = false;
    if (worker_.joinable()) {
        worker_.join();
    }
    
#ifdef _WIN32
    // Restore Windows timer resolution
    timeEndPeriod(1);
#endif
    
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        if (controllers_[i]) {
            psmove_disconnect(controllers_[i]);
            controllers_[i] = nullptr;
        }
        latest_atomic_[i].store(nullptr);
    }
}

std::optional<MoveSample> PSMoveManager::get_latest_for_slot(int slot) {
    if (slot < 0 || slot >= Constants::MAX_CONTROLLERS) {
        Logger::debug("Invalid slot requested: " + std::to_string(slot));
        return {};
    }
    
    // OPTIMIZATION: Use atomic access instead of mutex
    MoveSample* current = latest_atomic_[slot].load(std::memory_order_acquire);
    if (!current) {
        return {};
    }
    
    // Check data freshness
    auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    if (now - current->timestamp_us > 100000) { // 100ms threshold for stale data warning
        Logger::debug("Potentially stale data for slot " + std::to_string(slot) + 
                     " (age: " + std::to_string((now - current->timestamp_us) / 1000) + "ms)");
    }
    
    return *current;
}

int PSMoveManager::get_battery_level_for_slot(int slot) {
    if (slot < 0 || slot >= Constants::MAX_CONTROLLERS) return -1;
    std::lock_guard<std::mutex> lk(mtx_);
    if (!controllers_[slot]) return -1;
    return psmove_get_battery(controllers_[slot]);
}

bool PSMoveManager::has_controller_for_slot(int slot) {
    if (slot < 0 || slot >= Constants::MAX_CONTROLLERS) return false;
    std::lock_guard<std::mutex> lk(mtx_);
    return controllers_[slot] != nullptr;
}

std::string PSMoveManager::get_controller_serial_for_slot(int slot) {
    if (slot < 0 || slot >= Constants::MAX_CONTROLLERS) return "INVALID_SLOT";
    std::lock_guard<std::mutex> lk(mtx_);
    if (!controllers_[slot]) return "NO_CONTROLLER";
    return get_serial_safe(controllers_[slot]);
}

int PSMoveManager::get_connected_count() {
    std::lock_guard<std::mutex> lk(mtx_);
    int count = 0;
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        if (controllers_[i]) count++;
    }
    return count;
}

void PSMoveManager::thread_main() {
    // OPTIMIZATION: Set high thread priority for consistent timing
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    
    // CRITICAL Windows fix: Set thread affinity to prevent CPU core switching
    DWORD_PTR mask = 1; // Pin to CPU core 0
    if (SetThreadAffinityMask(GetCurrentThread(), mask) == 0) {
        Logger::debug("Could not set thread affinity (non-critical)");
    } else {
        Logger::debug("Polling thread pinned to CPU core 0");
    }
#else
    struct sched_param param;
    param.sched_priority = 50;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        Logger::debug("Could not set real-time priority (try running as root)");
    }
#endif
    
    auto last_scan = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    const auto scan_interval = std::chrono::seconds(5);
    int debug_counter = 0;
    int latency_warning_counter = 0;
    int flush_counter = 0;
    
    while (running_) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        
        // Periodically scan for new controllers
        if (loop_start - last_scan >= scan_interval) {
            last_scan = loop_start;
            scan_for_controllers();
        }
        
        // Poll all connected controllers
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
                if (!controllers_[slot]) {
                    has_data_[slot] = false;
                    latest_atomic_[slot].store(nullptr, std::memory_order_release);
                    continue;
                }
                
                // Verify controller is still valid
                if (psmove_connection_type(controllers_[slot]) == Conn_Unknown) {
                    Logger::error("Controller in slot " + std::to_string(slot) + " became invalid, removing");
                    psmove_disconnect(controllers_[slot]);
                    controllers_[slot] = nullptr;
                    has_data_[slot] = false;
                    latest_atomic_[slot].store(nullptr, std::memory_order_release);
                    continue;
                }
                
                uint64_t poll_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                
                // WINDOWS CRITICAL FIX: Multiple polling attempts with immediate retry
                bool got_data = false;
                int poll_attempts = 0;
                
#ifdef _WIN32
                // Windows: Try multiple times immediately to break through buffering
                const int max_attempts = 5;
                for (int attempt = 0; attempt < max_attempts && !got_data; attempt++) {
                    got_data = psmove_poll(controllers_[slot]);
                    poll_attempts++;
                    if (!got_data && attempt < max_attempts - 1) {
                        // Tiny delay between attempts
                        std::this_thread::sleep_for(std::chrono::microseconds(200));
                    }
                }
                
                // WINDOWS CRITICAL FIX: Periodic buffer flush to prevent 1-second delays
                if ((++flush_counter % 200) == 0) { // Every 200 polls (about 1 second at 250Hz)
                    int flushed = 0;
                    while (psmove_poll(controllers_[slot]) && flushed < 50) {
                        flushed++; // Drain up to 50 buffered packets
                    }
                    if (flushed > 5) { // Only log if significant buffering occurred
                        Logger::warn("Windows: Flushed " + std::to_string(flushed) + 
                                   " buffered packets from slot " + std::to_string(slot) + 
                                   " - this indicates Windows HID buffering issues");
                    }
                }
#else
                // Linux: Single poll attempt
                got_data = psmove_poll(controllers_[slot]);
                poll_attempts = 1;
#endif
                
                if (!got_data) {
                    continue;
                }
                
                // Read sensor data
                float ax = 0, ay = 0, az = 0;
                psmove_get_accelerometer_frame(controllers_[slot], Frame_SecondHalf, &ax, &ay, &az);
                
                float gx = 0, gy = 0, gz = 0;
                psmove_get_gyroscope_frame(controllers_[slot], Frame_SecondHalf, &gx, &gy, &gz);
                
                unsigned int btns = psmove_get_buttons(controllers_[slot]);
                unsigned char trig = psmove_get_trigger(controllers_[slot]);
                
                uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                // Use double buffering
                int current_buf = current_buffer_[slot].load();
                int next_buf = 1 - current_buf;
                
                MoveSample& sample = samples_[slot][next_buf];
                sample.accel_x = ax; sample.accel_y = ay; sample.accel_z = az;
                sample.gyro_x = gx; sample.gyro_y = gy; sample.gyro_z = gz;
                sample.buttons = btns;
                sample.trigger = trig;
                sample.timestamp_us = ts;
                sample.poll_timestamp_us = poll_timestamp;
                sample.send_timestamp_us = 0;
                
                current_buffer_[slot].store(next_buf);
                latest_atomic_[slot].store(&sample, std::memory_order_release);
                has_data_[slot] = true;
                
                // Enhanced debug output for Windows
                if (slot == 0 && (++debug_counter % 1000) == 0) { // Every 4 seconds at 250Hz
                    uint64_t processing_latency = ts - poll_timestamp;
                    Logger::info("Slot " + std::to_string(slot) + " - " + 
                               get_serial_safe(controllers_[slot]) +
                               " | Processing: " + std::to_string(processing_latency) + "µs" +
                               " | Poll attempts: " + std::to_string(poll_attempts));
                    
                    if (processing_latency > 50000) { // > 50ms indicates serious issue
                        Logger::error("CRITICAL: Very high processing latency: " + 
                                    std::to_string(processing_latency) + "µs - Windows HID issue likely");
                    }
                }
            }
        }
        
        // High-precision timing
        auto next_time = loop_start + std::chrono::microseconds(Constants::PSMOVE_POLL_MS * 1000);
        
#ifdef _WIN32
        // Windows: More aggressive timing to combat scheduling issues
        while (std::chrono::high_resolution_clock::now() < next_time) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
#else
        std::this_thread::sleep_until(next_time);
#endif
    }
}

void PSMoveManager::scan_for_controllers() {
    std::lock_guard<std::mutex> lk(mtx_);
    Logger::debug("=== CONTROLLER SCAN START ===");
    
    std::vector<PSMove*> available_controllers;
    std::vector<std::string> available_serials;
    
    for (int id = 0; id < 8; id++) {
        PSMove* controller = psmove_connect_by_id(id);
        if (!controller) continue;
        
        std::string serial = get_serial_safe(controller);
        std::string conn_type = get_connection_type_string(controller);
        
        if (conn_type != "BT") {
            Logger::debug("Skipping " + serial + " (" + conn_type + ") - not Bluetooth");
            psmove_disconnect(controller);
            continue;
        }
        
        if (find_serial_slot(serial) >= 0) {
            Logger::debug("Skipping " + serial + " - already connected");
            psmove_disconnect(controller);
            continue;
        }
        
        available_controllers.push_back(controller);
        available_serials.push_back(serial);
        Logger::debug("Found " + serial + " (" + conn_type + ")");
    }
    
    for (size_t i = 0; i < available_controllers.size(); i++) {
        PSMove* controller = available_controllers[i];
        std::string serial = available_serials[i];
        
        int free_slot = -1;
        for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
            if (!controllers_[slot]) {
                free_slot = slot;
                break;
            }
        }
        
        if (free_slot == -1) {
            Logger::warn("No free slots for " + serial);
            psmove_disconnect(controller);
            continue;
        }
        
        controllers_[free_slot] = controller;
        has_data_[free_slot] = false;
        latest_atomic_[free_slot].store(nullptr, std::memory_order_release);
        
#ifdef _WIN32
        // Windows: Initialize controller to prevent buffering issues
        psmove_set_leds(controller, 0, 0, 0);
        psmove_update_leds(controller);
        
        // Drain any initial buffered data
        int drained = 0;
        while (psmove_poll(controller) && drained < 100) {
            drained++;
        }
        if (drained > 0) {
            Logger::info("Windows: Drained " + std::to_string(drained) + 
                        " initial buffered packets from " + serial);
        }
#endif
        
        Logger::info("Assigned " + serial + " -> Slot " + std::to_string(free_slot));
    }
    
    Logger::debug("=== SCAN COMPLETE ===");
}

std::string PSMoveManager::get_serial_safe(PSMove* controller) {
    if (!controller) return "";
    char* serial = psmove_get_serial(controller);
    return serial ? std::string(serial) : "";
}

std::string PSMoveManager::get_connection_type_string(PSMove* controller) {
    if (!controller) return "NONE";
    auto conn = psmove_connection_type(controller);
    return (conn == Conn_Bluetooth) ? "BT" : (conn == Conn_USB) ? "USB" : "UNKNOWN";
}

int PSMoveManager::find_serial_slot(const std::string& serial) {
    if (serial.empty()) return -1;
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        if (controllers_[i] && get_serial_safe(controllers_[i]) == serial) {
            return i;
        }
    }
    return -1;
}
