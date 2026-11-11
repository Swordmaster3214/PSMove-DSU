#pragma once
#include "dsu/dsu_protocol.hpp"
#include "config/constants.hpp"
#include <psmove.h>
#include <array>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>
#include <string>
#include <vector>

#ifndef _WIN32
#include <pthread.h>
#include <sched.h>
#endif

// Snapshot of one controller slot, for display. Cheap to build: reads atomics
// and a small info struct, never the polling mutex.
struct ControllerStatus {
    int slot = 0;
    bool connected = false;
    bool simulated = false;
    std::string serial;
    std::string connection;      // "Bluetooth", "USB", "Simulated"
    std::string model;           // "CECH-ZCM1", "CECH-ZCM2"
    int battery = -1;            // PSMove_Battery_Level, -1 if unknown
    bool has_sample = false;
    MoveSample sample;
    uint64_t sample_age_us = 0;
    uint64_t reports_total = 0;      // HID reports read
    uint64_t polls_total = 0;        // Poll loop iterations for this slot
    uint64_t empty_polls_total = 0;  // Iterations where no report was waiting
    uint32_t backlog_peak = 0;       // Most reports read in one iteration since last reset
    uint64_t connected_since_us = 0;
    bool identifying = false;
};

struct ManagerStatus {
    bool realtime_priority = false;  // Polling thread got elevated scheduling
    uint64_t loop_overruns = 0;      // Poll loops that took longer than PSMOVE_POLL_MS
    uint32_t loop_peak_us = 0;       // Longest poll loop since last reset
    bool scanning = false;
    std::vector<std::string> usb_serials;  // Controllers seen on USB in the last scan
    int simulated_slots = 0;
};

struct PairStatus {
    enum class State { Idle, Running, Succeeded, Failed };
    State state = State::Idle;
    std::string serial;
    std::string message;
};

class PSMoveManager {
public:
    PSMoveManager();
    ~PSMoveManager();

    bool start();
    void stop();

    // Fills the highest `count` slots with virtual controllers (for testing
    // without hardware). Call before start().
    void set_simulated_controllers(int count);

    std::optional<MoveSample> get_latest_for_slot(int slot);
    int get_battery_level_for_slot(int slot);
    bool has_controller_for_slot(int slot);
    std::string get_controller_serial_for_slot(int slot);
    int get_connected_count();

    // --- GUI / diagnostics ---
    // reset_peaks clears backlog_peak / loop_peak_us after reading them.
    std::array<ControllerStatus, Constants::MAX_CONTROLLERS> get_status(bool reset_peaks);
    ManagerStatus get_manager_status(bool reset_peaks);
    // Lights the controller's LED and rumbles it briefly so you can tell which
    // physical controller is in which slot.
    void identify(int slot, int duration_ms = 1500);
    // Pairs the USB-connected controller with this serial ("" = first found)
    // to this computer's Bluetooth adapter. Runs in the background.
    void request_pair(const std::string& serial);
    PairStatus get_pair_status();
    void request_scan() { scan_requested_.store(true); }

private:
    void thread_main();
    std::string get_serial_safe(PSMove* controller);
    std::string get_connection_type_string(PSMove* controller);
    int find_serial_slot(const std::string& serial);
    bool is_simulated_slot(int slot) const;
    void fill_simulated_sample(int slot, uint64_t now_us);
    void set_slot_info(int slot, PSMove* controller);
    void clear_slot_info(int slot);
    void update_identify(int slot, uint64_t now_us);
    void pair_thread_main(std::string serial);

    // NEW: Background scanning methods
    void scanner_thread_main();
    void scan_for_controllers_async();
    void integrate_scanned_controller(PSMove* controller, const std::string& serial, int target_slot);
    void integrate_pending_controllers();
    void publish_sample(int slot, const MoveSample& s);

    PSMove* controllers_[Constants::MAX_CONTROLLERS];
    bool has_data_[Constants::MAX_CONTROLLERS];

    // OPTIMIZATION: Lock-free double buffering for sample data
    MoveSample samples_[Constants::MAX_CONTROLLERS][2];  // Double buffer per controller
    std::atomic<int> current_buffer_[Constants::MAX_CONTROLLERS];  // Which buffer is current
    std::atomic<MoveSample*> latest_atomic_[Constants::MAX_CONTROLLERS];  // Atomic pointer to current data

    std::thread worker_;
    std::atomic<bool> running_;
    std::mutex mtx_;  // Only for controller management, not data access

    // NEW: Scanner thread members
    std::thread scanner_thread_;
    std::atomic<bool> scan_requested_{false};
    std::atomic<bool> scanning_{false};
    std::atomic<bool> pause_scan_{false};
    std::mutex pending_controllers_mutex_;

    struct PendingController {
        PSMove* controller;
        std::string serial;
        int target_slot;
    };
    std::vector<PendingController> pending_controllers_;

    // Diagnostics, written by the polling thread and read by anyone.
    struct SlotStats {
        std::atomic<bool> connected{false};
        std::atomic<int> battery{-1};
        std::atomic<uint64_t> reports_total{0};
        std::atomic<uint64_t> polls_total{0};
        std::atomic<uint64_t> empty_polls{0};
        std::atomic<uint32_t> backlog_peak{0};
        std::atomic<uint64_t> connected_since_us{0};
        std::atomic<uint64_t> identify_until_us{0};
        bool identify_applied = false;  // Polling thread only
    };
    SlotStats stats_[Constants::MAX_CONTROLLERS];

    // Rarely-changing descriptive info, guarded by info_mutex_ (never held while polling).
    struct SlotInfo {
        std::string serial;
        std::string connection;
        std::string model;
    };
    SlotInfo info_[Constants::MAX_CONTROLLERS];
    std::vector<std::string> usb_serials_;
    std::mutex info_mutex_;

    std::atomic<bool> realtime_priority_{false};
    std::atomic<uint64_t> loop_overruns_{0};
    std::atomic<uint32_t> loop_peak_us_{0};
    int simulated_count_ = 0;

    std::thread pair_thread_;
    std::mutex pair_mutex_;
    PairStatus pair_status_;
};
