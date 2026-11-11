#pragma once
#include "config/config.hpp"
#include "config/constants.hpp"
#include "dsu/dsu_server.hpp"
#include "psmove/psmove_manager.hpp"
#include "utils/logging.hpp"
#include <array>
#include <chrono>
#include <map>
#include <string>
#include <vector>

// Everything drawn inside the main window. Reads server/controller state once
// per frame through snapshot getters; edits to the mapping go through
// DSUServer::set_config() and are saved to the config file immediately.
class GuiPanels {
public:
    GuiPanels(DSUServer& server, const Config& launch_config);
    void draw();

private:
    using Clock = std::chrono::steady_clock;
    static constexpr int HISTORY = 300;  // ~5 s of plot history at 60 fps

    struct SlotHistory {
        std::array<std::array<float, HISTORY>, 6> v{};  // gyro x,y,z (deg/s), accel x,y,z (g)
        int head = 0;
        int count = 0;
    };

    struct SlotRates {
        uint64_t last_reports = 0;
        uint64_t last_polls = 0;
        uint64_t last_empty = 0;
        double reports_per_s = 0;
        double empty_fraction = 0;
        uint32_t backlog_peak = 0;   // Largest drain in the last second
    };

    void refresh();
    void draw_header();
    void draw_controllers_tab();
    void draw_controller_card(const ControllerStatus& st);
    void draw_buttons_tab();
    void draw_motion_tab();
    void draw_emulators_tab();
    void draw_pairing_tab();
    void draw_log_tab();

    void apply_config();
    int preview_slot();
    std::string diagnostics_report();
    int clients_receiving_slot(int slot) const;

    DSUServer& server_;
    Config config_;
    std::string save_message_;
    bool save_error_ = false;

    // Snapshots, refreshed every frame
    std::array<ControllerStatus, Constants::MAX_CONTROLLERS> controllers_;
    ManagerStatus manager_;
    ServerStatus server_status_;
    std::vector<ClientSnapshot> clients_;
    PairStatus pair_;

    // Derived once per second
    Clock::time_point last_rate_update_;
    std::array<SlotRates, Constants::MAX_CONTROLLERS> rates_;
    std::map<std::string, uint64_t> client_last_packets_;
    std::map<std::string, double> client_packet_rate_;
    uint64_t last_total_packets_ = 0;
    double total_packet_rate_ = 0;
    uint32_t loop_peak_us_ = 0;

    std::array<SlotHistory, Constants::MAX_CONTROLLERS> history_;
    int selected_preview_slot_ = -1;

    // Log view
    std::vector<LogEntry> log_;
    uint64_t log_next_seq_ = 0;
    bool log_show_[4] = {true, true, true, true};  // Debug, Info, Warn, Error
    bool log_autoscroll_ = true;
    char log_filter_[128] = "";

    Clock::time_point toast_until_;
    std::string toast_;
};
