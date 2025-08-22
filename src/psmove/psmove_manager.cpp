#include "psmove/psmove_manager.hpp"
#include "utils/logging.hpp"
#include <chrono>
#include <cstdlib>

PSMoveManager::PSMoveManager() : running_(false) {
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        controllers_[i] = nullptr;
        has_data_[i] = false;
    }
}

PSMoveManager::~PSMoveManager() {
    stop();
}

bool PSMoveManager::start() {
    if (running_.load()) return true;
    
    running_ = true;
    worker_ = std::thread(&PSMoveManager::thread_main, this);
    Logger::debug("PSMove manager started");
    return true;
}

void PSMoveManager::stop() {
    running_ = false;
    if (worker_.joinable()) {
        worker_.join();
    }
    
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        if (controllers_[i]) {
            psmove_disconnect(controllers_[i]);
            controllers_[i] = nullptr;
        }
    }
}

std::optional<MoveSample> PSMoveManager::get_latest_for_slot(int slot) {
    if (slot < 0 || slot >= Constants::MAX_CONTROLLERS) {
        Logger::debug("Invalid slot requested: " + std::to_string(slot));
        return {};
    }
    
    std::lock_guard<std::mutex> lk(mtx_);
    if (!controllers_[slot] || !has_data_[slot]) {
        return {};
    }
    
    // Check data freshness
    auto now = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    
    if (now - latest_[slot].timestamp_us > 1000000) { // 1 second old
        Logger::warn("Stale data for slot " + std::to_string(slot));
    }
    
    return latest_[slot];
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
    auto last_scan = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    const auto scan_interval = std::chrono::seconds(5);
    int debug_counter = 0;
    
    while (running_) {
        auto now = std::chrono::steady_clock::now();
        
        // Periodically scan for new controllers
        if (now - last_scan >= scan_interval) {
            last_scan = now;
            scan_for_controllers();
        }
        
        // Poll all connected controllers
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
                if (!controllers_[slot]) {
                    has_data_[slot] = false;
                    continue;
                }
                
                // Verify controller is still valid
                if (psmove_connection_type(controllers_[slot]) == Conn_Unknown) {
                    Logger::error("Controller in slot " + std::to_string(slot) + " became invalid, removing");
                    psmove_disconnect(controllers_[slot]);
                    controllers_[slot] = nullptr;
                    has_data_[slot] = false;
                    continue;
                }
                
                // Poll this controller
                if (!psmove_poll(controllers_[slot])) {
                    continue; // No new data
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
                
                MoveSample sample;
                sample.accel_x = ax; sample.accel_y = ay; sample.accel_z = az;
                sample.gyro_x = gx; sample.gyro_y = gy; sample.gyro_z = gz;
                sample.buttons = btns;
                sample.trigger = trig;
                sample.timestamp_us = ts;
                
                latest_[slot] = sample;
                has_data_[slot] = true;
                
                // Debug output
                if (slot <= 1 && (++debug_counter % 250) == 0) {
                    Logger::debug("Poll slot " + std::to_string(slot) + " serial=" + get_serial_safe(controllers_[slot]));
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::PSMOVE_POLL_MS));
    }
}

void PSMoveManager::scan_for_controllers() {
    std::lock_guard<std::mutex> lk(mtx_);
    Logger::debug("=== CONTROLLER SCAN START ===");
    
    // Try to connect to available Bluetooth controllers
    std::vector<PSMove*> available_controllers;
    std::vector<std::string> available_serials;
    
    for (int id = 0; id < 8; id++) {
        PSMove* controller = psmove_connect_by_id(id);
        if (!controller) continue;
        
        std::string serial = get_serial_safe(controller);
        std::string conn_type = get_connection_type_string(controller);
        
        // Only accept Bluetooth controllers
        if (conn_type != "BT") {
            Logger::debug("Skipping " + serial + " (" + conn_type + ") - not Bluetooth");
            psmove_disconnect(controller);
            continue;
        }
        
        // Skip if already connected
        if (find_serial_slot(serial) >= 0) {
            Logger::debug("Skipping " + serial + " (" + conn_type + ") - already connected");
            psmove_disconnect(controller);
            continue;
        }
        
        available_controllers.push_back(controller);
        available_serials.push_back(serial);
        Logger::debug("Found " + serial + " (" + conn_type + ")");
    }
    
    // Assign controllers to slots
    for (size_t i = 0; i < available_controllers.size(); i++) {
        PSMove* controller = available_controllers[i];
        std::string serial = available_serials[i];
        
        // Find empty slot
        int free_slot = -1;
        for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
            if (!controllers_[slot]) {
                free_slot = slot;
                break;
            }
        }
        
        if (free_slot == -1) {
            Logger::warn("No free slots for " + serial + " (BT)");
            psmove_disconnect(controller);
            continue;
        }
        
        controllers_[free_slot] = controller;
        has_data_[free_slot] = false;
        Logger::info("Assigned " + serial + " (BT) -> Slot " + std::to_string(free_slot));
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
