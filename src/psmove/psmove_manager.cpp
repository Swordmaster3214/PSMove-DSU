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
    
    // Start background scanner thread FIRST
    scanner_thread_ = std::thread(&PSMoveManager::scanner_thread_main, this);
    
    // Start main polling thread
    worker_ = std::thread(&PSMoveManager::thread_main, this);
    
    Logger::debug("PSMove manager started with background scanning (zero-latency)");
    return true;
}

void PSMoveManager::stop() {
    running_ = false;
    
    // Join threads
    if (scanner_thread_.joinable()) {
        scanner_thread_.join();
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    
#ifdef _WIN32
    // Restore Windows timer resolution
    timeEndPeriod(1);
#endif
    
    // Cleanup controllers
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        if (controllers_[i]) {
            psmove_disconnect(controllers_[i]);
            controllers_[i] = nullptr;
        }
        latest_atomic_[i].store(nullptr);
    }
    
    // Cleanup pending controllers
    {
        std::lock_guard<std::mutex> pending_lock(pending_controllers_mutex_);
        for (auto& pending : pending_controllers_) {
            if (pending.controller) {
                psmove_disconnect(pending.controller);
            }
        }
        pending_controllers_.clear();
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
    DWORD_PTR mask = 1;
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
    
    // REMOVED: auto last_scan and scan_interval - no longer needed!
    int debug_counter = 0;
    int latency_warning_counter = 0;
    int flush_counter = 0;
    
    // Request initial scan
    scan_requested_.store(true);
    
    while (running_) {
        auto loop_start = std::chrono::steady_clock::now();
        
        // OPTIMIZATION: Integrate any newly discovered controllers (fast operation)
        integrate_pending_controllers();
        
        // Poll all connected controllers (unchanged)
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
                    
                    // Request rescan after controller loss
                    scan_requested_.store(true);
                    continue;
                }
                
                uint64_t poll_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                
                // Windows polling logic (unchanged)
                bool got_data = false;
                int poll_attempts = 0;
                
#ifdef _WIN32
                const int max_attempts = 5;
                for (int attempt = 0; attempt < max_attempts && !got_data; attempt++) {
                    got_data = psmove_poll(controllers_[slot]);
                    poll_attempts++;
                    if (!got_data && attempt < max_attempts - 1) {
                        std::this_thread::sleep_for(std::chrono::microseconds(200));
                    }
                }
                
                if ((++flush_counter % 200) == 0) {
                    int flushed = 0;
                    while (psmove_poll(controllers_[slot]) && flushed < 50) {
                        flushed++;
                    }
                    if (flushed > 5) {
                        Logger::warn("Windows: Flushed " + std::to_string(flushed) + 
                                   " buffered packets from slot " + std::to_string(slot));
                    }
                }
#else
                got_data = psmove_poll(controllers_[slot]);
                poll_attempts = 1;
#endif
                
                if (!got_data) {
                    continue;
                }
                
                // Read sensor data (unchanged)
                float ax = 0, ay = 0, az = 0;
                psmove_get_accelerometer_frame(controllers_[slot], Frame_SecondHalf, &ax, &ay, &az);
                
                float gx = 0, gy = 0, gz = 0;
                psmove_get_gyroscope_frame(controllers_[slot], Frame_SecondHalf, &gx, &gy, &gz);
                
                unsigned int btns = psmove_get_buttons(controllers_[slot]);
                unsigned char trig = psmove_get_trigger(controllers_[slot]);
                
                uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                // Double buffering (unchanged)
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
                
                // Debug output (unchanged)
                if (slot == 0 && (++debug_counter % 1000) == 0) {
                    uint64_t processing_latency = ts - poll_timestamp;
                    Logger::info("Slot " + std::to_string(slot) + " - " + 
                               get_serial_safe(controllers_[slot]) +
                               " | Processing: " + std::to_string(processing_latency) + "µs" +
                               " | Poll attempts: " + std::to_string(poll_attempts));
                    
                    if (processing_latency > 50000) {
                        Logger::error("CRITICAL: Very high processing latency: " + 
                                    std::to_string(processing_latency) + "µs");
                    }
                }
            }
        }
        
        // High-precision timing (unchanged)
        auto next_time = loop_start + std::chrono::microseconds(Constants::PSMOVE_POLL_MS * 1000);
        
#ifdef _WIN32
        while (std::chrono::steady_clock::now() < next_time) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
#else
        std::this_thread::sleep_until(next_time);
#endif
    }
}

// NEW: Background scanner thread
void PSMoveManager::scanner_thread_main() {
    // Lower priority than main polling thread
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#else
    struct sched_param param;
    param.sched_priority = 30; // Lower than polling thread
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
#endif
    
    while (running_) {
        // Check if scan is requested
        if (scan_requested_.exchange(false)) {
            Logger::debug("=== BACKGROUND CONTROLLER SCAN START ===");
            scan_for_controllers_async();
            Logger::debug("=== BACKGROUND SCAN COMPLETE ===");
        }
        
        // Check every 100ms, but also periodically request scans
        static int scan_timer = 0;
        if (++scan_timer >= 50) { // Every 5 seconds (50 * 100ms)
            scan_timer = 0;
            scan_requested_.store(true);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// NEW: Non-blocking controller scan
void PSMoveManager::scan_for_controllers_async() {
    std::vector<PSMove*> available_controllers;
    std::vector<std::string> available_serials;
    
    // Scan for controllers (this is the slow part, but now in background)
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
        
        // Check if already connected (need mutex for this check)
        bool already_connected = false;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (find_serial_slot(serial) >= 0) {
                already_connected = true;
            }
        }
        
        if (already_connected) {
            Logger::debug("Skipping " + serial + " - already connected");
            psmove_disconnect(controller);
            continue;
        }
        
        available_controllers.push_back(controller);
        available_serials.push_back(serial);
        Logger::debug("Found " + serial + " (" + conn_type + ")");
    }
    
    // Queue new controllers for integration
    {
        std::lock_guard<std::mutex> pending_lock(pending_controllers_mutex_);
        
        for (size_t i = 0; i < available_controllers.size(); i++) {
            PSMove* controller = available_controllers[i];
            std::string serial = available_serials[i];
            
            // Find target slot
            int target_slot = -1;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
                    if (!controllers_[slot]) {
                        target_slot = slot;
                        break;
                    }
                }
            }
            
            if (target_slot == -1) {
                Logger::warn("No free slots for " + serial);
                psmove_disconnect(controller);
                continue;
            }
            
            // Initialize controller in background
#ifdef _WIN32
            psmove_set_leds(controller, 0, 0, 0);
            psmove_update_leds(controller);
            
            int drained = 0;
            while (psmove_poll(controller) && drained < 100) {
                drained++;
            }
            if (drained > 0) {
                Logger::info("Windows: Drained " + std::to_string(drained) + 
                            " initial buffered packets from " + serial);
            }
#endif
            
            PendingController pending = {controller, serial, target_slot};
            pending_controllers_.push_back(pending);
            
            Logger::info("Queued " + serial + " for assignment to Slot " + std::to_string(target_slot));
        }
    }
}

// NEW: Fast integration of discovered controllers
void PSMoveManager::integrate_pending_controllers() {
    std::lock_guard<std::mutex> pending_lock(pending_controllers_mutex_);
    
    if (pending_controllers_.empty()) {
        return;
    }
    
    // Brief lock to integrate controllers
    {
        std::lock_guard<std::mutex> lk(mtx_);
        
        for (auto& pending : pending_controllers_) {
            if (pending.target_slot >= 0 && pending.target_slot < Constants::MAX_CONTROLLERS &&
                !controllers_[pending.target_slot]) {
                
                controllers_[pending.target_slot] = pending.controller;
                has_data_[pending.target_slot] = false;
                latest_atomic_[pending.target_slot].store(nullptr, std::memory_order_release);
                
                Logger::info("Integrated " + pending.serial + " -> Slot " + std::to_string(pending.target_slot));
            } else {
                // Slot taken, disconnect
                Logger::warn("Slot " + std::to_string(pending.target_slot) + " taken, disconnecting " + pending.serial);
                psmove_disconnect(pending.controller);
            }
        }
    }
    
    pending_controllers_.clear();
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
