#include "psmove/psmove_manager.hpp"
#include "utils/logging.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>  // For timeBeginPeriod
#include <processthreadsapi.h> // For thread priority
#pragma comment(lib, "winmm.lib")
#endif

namespace {
uint64_t steady_now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Raise an atomic to at least `v`.
template <typename T>
void store_max(std::atomic<T>& a, T v) {
    T cur = a.load(std::memory_order_relaxed);
    while (v > cur && !a.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {}
}

const char* model_string(PSMove* controller) {
    switch (psmove_get_model(controller)) {
        case Model_ZCM1: return "CECH-ZCM1";
        case Model_ZCM2: return "CECH-ZCM2";
        default: return "Unknown";
    }
}
} // namespace

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

void PSMoveManager::set_simulated_controllers(int count) {
    if (running_.load()) return;
    if (count < 0) count = 0;
    if (count > Constants::MAX_CONTROLLERS) count = Constants::MAX_CONTROLLERS;
    simulated_count_ = count;
}

bool PSMoveManager::is_simulated_slot(int slot) const {
    return slot >= Constants::MAX_CONTROLLERS - simulated_count_;
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

    for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
        if (is_simulated_slot(slot)) {
            std::lock_guard<std::mutex> lk(info_mutex_);
            info_[slot] = {"SIM-" + std::to_string(slot), "Simulated", "Virtual"};
            stats_[slot].connected = true;
            stats_[slot].connected_since_us = steady_now_us();
        }
    }
    if (simulated_count_ > 0) {
        Logger::warn("Simulating " + std::to_string(simulated_count_) +
                     " virtual controller(s) for testing (--simulate)");
    }

    // Start background scanner thread FIRST
    scanner_thread_ = std::thread(&PSMoveManager::scanner_thread_main, this);

    // Start main polling thread
    worker_ = std::thread(&PSMoveManager::thread_main, this);

    Logger::debug("PSMove manager started with background scanning (zero-latency)");
    return true;
}

void PSMoveManager::stop() {
    // stop() runs from both DSUServer::stop() and the destructor; only undo start() once.
    if (!running_.exchange(false)) return;

    // Join threads
    if (pair_thread_.joinable()) {
        pair_thread_.join();
    }
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
        stats_[i].connected = false;
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
    auto now = steady_now_us();

    if (now - current->timestamp_us > 100000) { // 100ms threshold for stale data warning
        Logger::debug("Potentially stale data for slot " + std::to_string(slot) +
                     " (age: " + std::to_string((now - current->timestamp_us) / 1000) + "ms)");
    }

    return *current;
}

int PSMoveManager::get_battery_level_for_slot(int slot) {
    if (slot < 0 || slot >= Constants::MAX_CONTROLLERS) return -1;
    // Cached by the polling thread so the send loop never waits on mtx_.
    return stats_[slot].connected.load() ? stats_[slot].battery.load() : -1;
}

bool PSMoveManager::has_controller_for_slot(int slot) {
    if (slot < 0 || slot >= Constants::MAX_CONTROLLERS) return false;
    return stats_[slot].connected.load();
}

std::string PSMoveManager::get_controller_serial_for_slot(int slot) {
    if (slot < 0 || slot >= Constants::MAX_CONTROLLERS) return "INVALID_SLOT";
    std::lock_guard<std::mutex> lk(info_mutex_);
    if (!stats_[slot].connected.load()) return "NO_CONTROLLER";
    return info_[slot].serial;
}

int PSMoveManager::get_connected_count() {
    int count = 0;
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        if (stats_[i].connected.load()) count++;
    }
    return count;
}

std::array<ControllerStatus, Constants::MAX_CONTROLLERS> PSMoveManager::get_status(bool reset_peaks) {
    std::array<ControllerStatus, Constants::MAX_CONTROLLERS> out;
    uint64_t now = steady_now_us();
    for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
        ControllerStatus& st = out[slot];
        SlotStats& s = stats_[slot];
        st.slot = slot;
        st.connected = s.connected.load();
        st.simulated = is_simulated_slot(slot);
        st.battery = s.battery.load();
        st.reports_total = s.reports_total.load();
        st.polls_total = s.polls_total.load();
        st.empty_polls_total = s.empty_polls.load();
        st.backlog_peak = reset_peaks ? s.backlog_peak.exchange(0) : s.backlog_peak.load();
        st.connected_since_us = s.connected_since_us.load();
        st.identifying = now < s.identify_until_us.load();

        MoveSample* current = latest_atomic_[slot].load(std::memory_order_acquire);
        if (current && st.connected) {
            st.sample = *current;
            st.has_sample = true;
            st.sample_age_us = now > st.sample.timestamp_us ? now - st.sample.timestamp_us : 0;
        }
    }
    {
        std::lock_guard<std::mutex> lk(info_mutex_);
        for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
            if (!out[slot].connected) continue;
            out[slot].serial = info_[slot].serial;
            out[slot].connection = info_[slot].connection;
            out[slot].model = info_[slot].model;
        }
    }
    return out;
}

ManagerStatus PSMoveManager::get_manager_status(bool reset_peaks) {
    ManagerStatus st;
    st.realtime_priority = realtime_priority_.load();
    st.loop_overruns = loop_overruns_.load();
    st.loop_peak_us = reset_peaks ? loop_peak_us_.exchange(0) : loop_peak_us_.load();
    st.scanning = scanning_.load();
    st.simulated_slots = simulated_count_;
    std::lock_guard<std::mutex> lk(info_mutex_);
    st.usb_serials = usb_serials_;
    return st;
}

void PSMoveManager::identify(int slot, int duration_ms) {
    if (slot < 0 || slot >= Constants::MAX_CONTROLLERS) return;
    stats_[slot].identify_until_us = steady_now_us() + uint64_t(duration_ms) * 1000;
}

void PSMoveManager::set_slot_info(int slot, PSMove* controller) {
    std::lock_guard<std::mutex> lk(info_mutex_);
    info_[slot].serial = get_serial_safe(controller);
    info_[slot].connection = get_connection_type_string(controller) == "BT" ? "Bluetooth" : "USB";
    info_[slot].model = model_string(controller);
}

void PSMoveManager::clear_slot_info(int slot) {
    std::lock_guard<std::mutex> lk(info_mutex_);
    info_[slot] = SlotInfo{};
}

// Called from the polling thread with mtx_ held. Only writes to the device when
// the wanted state changes, so it costs nothing in the normal case.
void PSMoveManager::update_identify(int slot, uint64_t now_us) {
    SlotStats& s = stats_[slot];
    bool want = now_us < s.identify_until_us.load(std::memory_order_relaxed);
    if (want == s.identify_applied) return;

    PSMove* c = controllers_[slot];
    if (want) {
        psmove_set_leds(c, 255, 255, 255);
        psmove_set_rumble(c, 160);
    } else {
        psmove_set_leds(c, 0, 0, 0);
        psmove_set_rumble(c, 0);
    }
    // Update_Ignored means rate-limited; try again on the next loop.
    if (psmove_update_leds(c) == Update_Success) {
        s.identify_applied = want;
    }
}

void PSMoveManager::publish_sample(int slot, const MoveSample& s) {
    int current_buf = current_buffer_[slot].load();
    int next_buf = 1 - current_buf;
    samples_[slot][next_buf] = s;
    current_buffer_[slot].store(next_buf);
    latest_atomic_[slot].store(&samples_[slot][next_buf], std::memory_order_release);
    has_data_[slot] = true;
}

// Synthetic motion and button presses so emulators and the GUI can be tested
// without a physical controller.
void PSMoveManager::fill_simulated_sample(int slot, uint64_t now_us) {
    double t = now_us / 1e6;
    double phase = slot * 1.3;
    MoveSample s;
    s.gyro_x = float(1.5 * std::sin(t * 1.7 + phase));
    s.gyro_y = float(1.0 * std::sin(t * 1.1 + phase));
    s.gyro_z = float(0.8 * std::sin(t * 0.7 + phase));
    s.accel_x = float(0.3 * std::sin(t * 0.9 + phase));
    s.accel_y = float(0.3 * std::cos(t * 0.6 + phase));
    s.accel_z = 1.0f;

    // Press each button in turn for half a second, then sweep the trigger.
    static const unsigned int order[] = {
        Constants::Btn_CROSS, Constants::Btn_CIRCLE, Constants::Btn_SQUARE, Constants::Btn_TRIANGLE,
        Constants::Btn_MOVE, Constants::Btn_START, Constants::Btn_SELECT, Constants::Btn_PS};
    int step = int(t * 2.0) % 10;
    s.buttons = step < 8 ? order[step] : 0;
    s.trigger = step >= 8 ? (unsigned char)(255.0 * std::fabs(std::sin(t * 3.14159))) : 0;

    s.timestamp_us = now_us;
    s.poll_timestamp_us = now_us;
    publish_sample(slot, s);
    stats_[slot].reports_total.fetch_add(1, std::memory_order_relaxed);
    stats_[slot].polls_total.fetch_add(1, std::memory_order_relaxed);
    stats_[slot].battery = Batt_MAX;
}

void PSMoveManager::thread_main() {
    // OPTIMIZATION: Set high thread priority for consistent timing
#ifdef _WIN32
    realtime_priority_ = SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) != 0;
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
    } else {
        realtime_priority_ = true;
    }
#endif

    int debug_counter = 0;

    // Request initial scan
    scan_requested_.store(true);

    while (running_) {
        auto loop_start = std::chrono::steady_clock::now();

        // OPTIMIZATION: Integrate any newly discovered controllers (fast operation)
        integrate_pending_controllers();

        // Poll all connected controllers
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
                if (is_simulated_slot(slot)) {
                    fill_simulated_sample(slot, steady_now_us());
                    continue;
                }

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
                    stats_[slot].connected = false;
                    stats_[slot].identify_applied = false;
                    clear_slot_info(slot);

                    // Request rescan after controller loss
                    scan_requested_.store(true);
                    continue;
                }

                // Same clock as every other timestamp in a MoveSample (steady_clock).
                uint64_t poll_timestamp = steady_now_us();

                // Read every report that has queued up since the last loop and
                // keep only the newest. Reading just one per loop lets a backlog
                // build (the controller can send faster than we poll), and that
                // backlog shows up as input lag.
                int reports = 0;
                while (reports < Constants::MAX_REPORTS_PER_POLL && psmove_poll(controllers_[slot])) {
                    reports++;
                }
                int poll_attempts = 1;

#ifdef _WIN32
                // Windows: if nothing was waiting, give the report a moment to arrive.
                const int max_attempts = 5;
                for (int attempt = 1; attempt < max_attempts && reports == 0; attempt++) {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    while (reports < Constants::MAX_REPORTS_PER_POLL && psmove_poll(controllers_[slot])) {
                        reports++;
                    }
                    poll_attempts++;
                }
#endif

                SlotStats& st = stats_[slot];
                st.polls_total.fetch_add(1, std::memory_order_relaxed);
                update_identify(slot, steady_now_us());

                if (reports == 0) {
                    st.empty_polls.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                st.reports_total.fetch_add(reports, std::memory_order_relaxed);
                store_max<uint32_t>(st.backlog_peak, uint32_t(reports));
                if (reports > 8) {
                    Logger::debug("Slot " + std::to_string(slot) + ": drained " + std::to_string(reports) +
                                  " queued reports in one poll");
                }

                // Read sensor data
                float ax = 0, ay = 0, az = 0;
                psmove_get_accelerometer_frame(controllers_[slot], Frame_SecondHalf, &ax, &ay, &az);

                float gx = 0, gy = 0, gz = 0;
                psmove_get_gyroscope_frame(controllers_[slot], Frame_SecondHalf, &gx, &gy, &gz);

                unsigned int btns = psmove_get_buttons(controllers_[slot]);
                unsigned char trig = psmove_get_trigger(controllers_[slot]);
                st.battery = int(psmove_get_battery(controllers_[slot]));

                uint64_t ts = steady_now_us();

                MoveSample sample;
                sample.accel_x = ax; sample.accel_y = ay; sample.accel_z = az;
                sample.gyro_x = gx; sample.gyro_y = gy; sample.gyro_z = gz;
                sample.buttons = btns;
                sample.trigger = trig;
                sample.timestamp_us = ts;
                sample.poll_timestamp_us = poll_timestamp;
                sample.send_timestamp_us = 0;
                publish_sample(slot, sample);

                if (slot == 0 && (++debug_counter % 1000) == 0) {
                    uint64_t processing_latency = ts - poll_timestamp;
                    Logger::info("Slot " + std::to_string(slot) + " - " +
                               get_serial_safe(controllers_[slot]) +
                               " | Processing: " + std::to_string(processing_latency) + "us" +
                               " | Poll attempts: " + std::to_string(poll_attempts));

                    if (processing_latency > 50000) {
                        Logger::error("CRITICAL: Very high processing latency: " +
                                    std::to_string(processing_latency) + "us");
                    }
                }
            }
        }

        auto loop_us = uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - loop_start).count());
        store_max<uint32_t>(loop_peak_us_, loop_us);
        if (loop_us > uint32_t(Constants::PSMOVE_POLL_MS * 1000)) {
            loop_overruns_.fetch_add(1, std::memory_order_relaxed);
        }

        // High-precision timing
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

    int scan_timer = 0;
    while (running_) {
        // Check if scan is requested (pairing pauses scanning so the two
        // don't fight over the USB device)
        if (!pause_scan_.load() && scan_requested_.exchange(false)) {
            scanning_ = true;
            Logger::debug("=== BACKGROUND CONTROLLER SCAN START ===");
            scan_for_controllers_async();
            Logger::debug("=== BACKGROUND SCAN COMPLETE ===");
            scanning_ = false;
        }

        // Check every 100ms, but also periodically request scans
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
    std::vector<std::string> usb_serials;

    // Scan for controllers (this is the slow part, but now in background)
    for (int id = 0; id < 8; id++) {
        PSMove* controller = psmove_connect_by_id(id);
        if (!controller) continue;

        std::string serial = get_serial_safe(controller);
        std::string conn_type = get_connection_type_string(controller);

        if (conn_type != "BT") {
            Logger::debug("Skipping " + serial + " (" + conn_type + ") - not Bluetooth");
            if (conn_type == "USB") usb_serials.push_back(serial);
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

    {
        std::lock_guard<std::mutex> lk(info_mutex_);
        usb_serials_ = usb_serials;
    }

    // Queue new controllers for integration
    {
        std::lock_guard<std::mutex> pending_lock(pending_controllers_mutex_);

        for (size_t i = 0; i < available_controllers.size(); i++) {
            PSMove* controller = available_controllers[i];
            std::string serial = available_serials[i];

            // Find target slot (skipping slots already promised to an earlier pending controller)
            int target_slot = -1;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                for (int slot = 0; slot < Constants::MAX_CONTROLLERS; slot++) {
                    if (controllers_[slot] || is_simulated_slot(slot)) continue;
                    bool reserved = false;
                    for (const auto& p : pending_controllers_) {
                        if (p.target_slot == slot) reserved = true;
                    }
                    if (!reserved) {
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

                int slot = pending.target_slot;
                controllers_[slot] = pending.controller;
                has_data_[slot] = false;
                latest_atomic_[slot].store(nullptr, std::memory_order_release);
                set_slot_info(slot, pending.controller);
                stats_[slot].reports_total = 0;
                stats_[slot].polls_total = 0;
                stats_[slot].empty_polls = 0;
                stats_[slot].backlog_peak = 0;
                stats_[slot].identify_applied = false;
                stats_[slot].battery = -1;
                stats_[slot].connected_since_us = steady_now_us();
                stats_[slot].connected = true;

                Logger::info("Integrated " + pending.serial + " -> Slot " + std::to_string(slot));
            } else {
                // Slot taken, disconnect
                Logger::warn("Slot " + std::to_string(pending.target_slot) + " taken, disconnecting " + pending.serial);
                psmove_disconnect(pending.controller);
            }
        }
    }

    pending_controllers_.clear();
}

void PSMoveManager::request_pair(const std::string& serial) {
    {
        std::lock_guard<std::mutex> lk(pair_mutex_);
        if (pair_status_.state == PairStatus::State::Running) return;
        pair_status_.state = PairStatus::State::Running;
        pair_status_.serial = serial;
        pair_status_.message = "Waiting for controller scan to finish...";
    }
    if (pair_thread_.joinable()) pair_thread_.join();
    pair_thread_ = std::thread(&PSMoveManager::pair_thread_main, this, serial);
}

PairStatus PSMoveManager::get_pair_status() {
    std::lock_guard<std::mutex> lk(pair_mutex_);
    return pair_status_;
}

void PSMoveManager::pair_thread_main(std::string serial) {
    auto set_status = [this](PairStatus::State state, const std::string& msg) {
        std::lock_guard<std::mutex> lk(pair_mutex_);
        pair_status_.state = state;
        pair_status_.message = msg;
    };

    // Stop the scanner from opening (and closing) the USB device under us.
    pause_scan_ = true;
    for (int i = 0; i < 200 && scanning_.load(); i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    set_status(PairStatus::State::Running, "Looking for the controller on USB...");
    Logger::info("Pairing: looking for USB controller " + (serial.empty() ? std::string("(any)") : serial));

    PSMove* usb = nullptr;
    for (int id = 0; id < 8 && !usb; id++) {
        PSMove* c = psmove_connect_by_id(id);
        if (!c) continue;
        if (psmove_connection_type(c) == Conn_USB &&
            (serial.empty() || get_serial_safe(c) == serial)) {
            usb = c;
        } else {
            psmove_disconnect(c);
        }
    }

    if (!usb) {
        pause_scan_ = false;
        set_status(PairStatus::State::Failed,
                   "No controller found on USB. Plug it in with a USB cable (data-capable, not charge-only) and try again.");
        Logger::error("Pairing failed: no USB controller found");
        return;
    }

    std::string found_serial = get_serial_safe(usb);
    set_status(PairStatus::State::Running, "Writing this computer's Bluetooth address to " + found_serial + "...");
    int result = psmove_pair(usb);
    psmove_disconnect(usb);
    pause_scan_ = false;
    scan_requested_ = true;

    if (result) {
        Logger::info("Pairing succeeded for " + found_serial);
        set_status(PairStatus::State::Succeeded,
                   "Paired " + found_serial + ". Unplug the USB cable and press the PS button. The red LED will blink, "
                   "then stay lit once connected. If it keeps blinking, charge the controller, turn Bluetooth off and on "
                   "(or restart your computer), and press PS again.");
    } else {
        Logger::error("Pairing failed for " + found_serial);
#if defined(_WIN32)
        const char* hint = "Make sure Bluetooth is turned on, try running PSMove-DSU as administrator, and try again; "
                           "it can take a few attempts on Windows. Third-party Bluetooth drivers (e.g. MotionInJoy) can interfere.";
#elif defined(__APPLE__)
        const char* hint = "Make sure Bluetooth is turned on and that PSMove-DSU is allowed to use Bluetooth. "
                           "Pairing may need administrator rights; try running from a terminal with sudo.";
#else
        const char* hint = "On Linux, pairing must register the controller with BlueZ, which needs root: "
                           "run once with sudo (e.g. sudo ./dsu_server_psmove --pair).";
#endif
        set_status(PairStatus::State::Failed, "Pairing failed for " + found_serial + ". " + hint);
    }
}

std::string PSMoveManager::get_serial_safe(PSMove* controller) {
    if (!controller) return "";
    char* serial = psmove_get_serial(controller);
    std::string s = serial ? std::string(serial) : "";
    // psmove_get_serial() returns a copy the caller must release.
    if (serial) psmove_free_mem(serial);
    return s;
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
