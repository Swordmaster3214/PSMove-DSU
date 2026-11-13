#include "gui/gui_panels.hpp"
#include "config/config_file.hpp"
#include "config/mapping.hpp"
#include "dsu/dsu_protocol.hpp"
#include "utils/licenses.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <sstream>

#include "dsu_version.h"

namespace {

const ImVec4 COL_OK     (0.36f, 0.82f, 0.46f, 1.0f);
const ImVec4 COL_WARN   (0.98f, 0.76f, 0.28f, 1.0f);
const ImVec4 COL_ERR    (0.97f, 0.40f, 0.40f, 1.0f);
const ImVec4 COL_DIM    (0.56f, 0.59f, 0.66f, 1.0f);
const ImVec4 COL_ACCENT (0.48f, 0.67f, 1.00f, 1.0f);
const ImVec4 AXIS_COL[3] = {
    ImVec4(0.96f, 0.47f, 0.47f, 1.0f),
    ImVec4(0.45f, 0.86f, 0.52f, 1.0f),
    ImVec4(0.48f, 0.67f, 1.00f, 1.0f),
};

enum class Level { Ok, Warn, Err, Off };

ImVec4 level_color(Level l) {
    switch (l) {
        case Level::Ok:   return COL_OK;
        case Level::Warn: return COL_WARN;
        case Level::Err:  return COL_ERR;
        default:          return COL_DIM;
    }
}

std::string fmt(const char* f, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof(buf), f, ap);
    va_end(ap);
    return buf;
}

float ui_scale() {
    return ImGui::GetFontSize() / 15.0f;
}

// A coloured dot followed by text, on the current line.
void status_chip(const std::string& text, Level level) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    float h = ImGui::GetTextLineHeight();
    ImVec2 p = ImGui::GetCursorScreenPos();
    float r = h * 0.28f;
    dl->AddCircleFilled(ImVec2(p.x + r, p.y + h * 0.5f + 1), r, ImGui::GetColorU32(level_color(level)));
    ImGui::Dummy(ImVec2(r * 2 + 4, h));
    ImGui::SameLine(0, 0);
    ImGui::TextUnformatted(text.c_str());
}

void help_marker(const char* text) {
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 32.0f);
        ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

// A labelled box that lights up while `on`.
void indicator(const char* label, bool on, float width) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 size(width, ImGui::GetFrameHeight());
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton(label, size);
    ImU32 fill = on ? ImGui::GetColorU32(COL_ACCENT) : IM_COL32(38, 42, 52, 255);
    ImU32 border = on ? ImGui::GetColorU32(COL_ACCENT) : IM_COL32(62, 67, 80, 255);
    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), fill, 4.0f);
    dl->AddRect(p, ImVec2(p.x + size.x, p.y + size.y), border, 4.0f);
    const char* hidden = std::strstr(label, "##");  // "text##id": only draw "text"
    const char* end = hidden ? hidden : label + std::strlen(label);
    ImVec2 ts = ImGui::CalcTextSize(label, end);
    ImU32 tc = on ? IM_COL32(12, 16, 26, 255) : ImGui::GetColorU32(COL_DIM);
    dl->AddText(ImVec2(p.x + (size.x - ts.x) * 0.5f, p.y + (size.y - ts.y) * 0.5f), tc, label, end);
}

// Horizontal bar growing left or right from the centre, for signed values.
void center_bar(float value, float range, float width) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 size(width, ImGui::GetTextLineHeight());
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::Dummy(size);
    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), IM_COL32(32, 35, 44, 255), 3.0f);
    float mid = p.x + size.x * 0.5f;
    float f = std::max(-1.0f, std::min(1.0f, value / range));
    float x = mid + f * size.x * 0.5f;
    dl->AddRectFilled(ImVec2(std::min(mid, x), p.y + 2), ImVec2(std::max(mid, x), p.y + size.y - 2),
                      ImGui::GetColorU32(COL_ACCENT), 2.0f);
    dl->AddLine(ImVec2(mid, p.y), ImVec2(mid, p.y + size.y), IM_COL32(255, 255, 255, 60));
}

std::string battery_text(int b) {
    if (b < 0) return "unknown";
    if (b == Batt_CHARGING) return "charging";
    if (b == Batt_CHARGING_DONE) return "charged";
    if (b == Batt_MIN) return "< 20%";
    if (b <= Batt_MAX) return fmt("%d%%", b * 20);
    return fmt("0x%02X", b);
}

Level battery_level(int b) {
    if (b == Batt_MIN) return Level::Err;
    if (b == Batt_20Percent) return Level::Warn;
    if (b < 0) return Level::Off;
    return Level::Ok;
}

std::string duration_text(double seconds) {
    if (seconds < 1.0) return fmt("%.0f ms", seconds * 1000.0);
    if (seconds < 120.0) return fmt("%.1f s", seconds);
    if (seconds < 7200.0) return fmt("%.0f min", seconds / 60.0);
    return fmt("%.1f h", seconds / 3600.0);
}

const char* os_name() {
#if defined(_WIN32)
#  if defined(_WIN64)
    return "Windows x64";
#  else
    return "Windows x86";
#  endif
#elif defined(__APPLE__)
#  if defined(__aarch64__) || defined(__arm64__)
    return "macOS arm64";
#  else
    return "macOS x64";
#  endif
#elif defined(__linux__)
#  if defined(__aarch64__)
    return "Linux arm64";
#  elif defined(__x86_64__)
    return "Linux x64";
#  else
    return "Linux";
#  endif
#else
    return "unknown OS";
#endif
}

// Index of a DSU button's analog byte in DsuButtonState::analog, or -1.
int analog_index(DsuButton b) {
    switch (b) {
        case DsuButton::DpadLeft:  return 0;
        case DsuButton::DpadDown:  return 1;
        case DsuButton::DpadRight: return 2;
        case DsuButton::DpadUp:    return 3;
        case DsuButton::Triangle:  return 4;
        case DsuButton::Circle:    return 5;
        case DsuButton::Cross:     return 6;
        case DsuButton::Square:    return 7;
        case DsuButton::R1:        return 8;
        case DsuButton::L1:        return 9;
        case DsuButton::R2:        return 10;
        case DsuButton::L2:        return 11;
        default:                   return -1;
    }
}

const char* ANALOG_NAMES[12] = {
    "D-Left", "D-Down", "D-Right", "D-Up", "Triangle", "Circle",
    "Cross", "Square", "R1", "L1", "R2", "L2",
};

bool contains_ci(const std::string& hay, const char* needle) {
    if (!needle || !*needle) return true;
    std::string h = hay, n = needle;
    std::transform(h.begin(), h.end(), h.begin(), ::tolower);
    std::transform(n.begin(), n.end(), n.begin(), ::tolower);
    return h.find(n) != std::string::npos;
}

} // namespace

GuiPanels::GuiPanels(DSUServer& server, const Config& launch_config)
    : server_(server) {
    config_ = server_.get_config();
    config_.config_path = launch_config.config_path;
    last_rate_update_ = Clock::now();
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

void GuiPanels::refresh() {
    auto now = Clock::now();
    double dt = std::chrono::duration<double>(now - last_rate_update_).count();
    bool new_second = dt >= 1.0;

    PSMoveManager& mgr = server_.controllers();
    controllers_ = mgr.get_status(new_second);
    manager_ = mgr.get_manager_status(new_second);
    server_status_ = server_.get_status();
    clients_ = server_.get_clients();
    pair_ = mgr.get_pair_status();
    nav_status_ = nav_pair_.status();

    if (new_second) {
        for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
            const ControllerStatus& c = controllers_[i];
            SlotRates& r = rates_[i];
            // Counters restart when a controller (re)connects
            if (c.reports_total < r.last_reports || c.polls_total < r.last_polls) {
                r.last_reports = r.last_polls = r.last_empty = 0;
            }
            uint64_t polls = c.polls_total - r.last_polls;
            r.reports_per_s = (c.reports_total - r.last_reports) / dt;
            r.empty_fraction = polls ? double(c.empty_polls_total - r.last_empty) / double(polls) : 0.0;
            r.backlog_peak = c.backlog_peak;
            r.last_reports = c.reports_total;
            r.last_polls = c.polls_total;
            r.last_empty = c.empty_polls_total;
        }
        std::map<std::string, double> rates;
        for (const auto& cl : clients_) {
            auto it = client_last_packets_.find(cl.address);
            uint64_t prev = it != client_last_packets_.end() ? it->second : cl.packets_sent;
            rates[cl.address] = cl.packets_sent >= prev ? (cl.packets_sent - prev) / dt : 0.0;
            client_last_packets_[cl.address] = cl.packets_sent;
        }
        client_packet_rate_ = rates;
        total_packet_rate_ = (server_status_.packets_sent - last_total_packets_) / dt;
        last_total_packets_ = server_status_.packets_sent;
        loop_peak_us_ = manager_.loop_peak_us;
        last_rate_update_ = now;
    }

    // Plot history
    const float RAD_TO_DEG = 57.2957795f;
    for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
        const ControllerStatus& c = controllers_[i];
        SlotHistory& h = history_[i];
        if (!c.connected || !c.has_sample) {
            h.count = 0;
            continue;
        }
        const MoveSample& s = c.sample;
        float vals[6] = {s.gyro_x * RAD_TO_DEG, s.gyro_y * RAD_TO_DEG, s.gyro_z * RAD_TO_DEG,
                         s.accel_x, s.accel_y, s.accel_z};
        for (int ch = 0; ch < 6; ch++) h.v[ch][h.head] = vals[ch];
        h.head = (h.head + 1) % HISTORY;
        h.count = std::min(h.count + 1, HISTORY);
    }

    // Log
    log_next_seq_ = Logger::get_entries(log_next_seq_, log_);
    if (log_.size() > 4000) log_.erase(log_.begin(), log_.begin() + (log_.size() - 4000));
}

void GuiPanels::apply_config() {
    server_.set_config(config_);
    std::string err;
    if (ConfigFile::save(config_.config_path, config_, err)) {
        save_message_ = "Saved to " + config_.config_path;
        save_error_ = false;
    } else {
        save_message_ = "Could not save settings: " + err;
        save_error_ = true;
    }
}

int GuiPanels::preview_slot() {
    if (selected_preview_slot_ >= 0 && controllers_[selected_preview_slot_].connected) {
        return selected_preview_slot_;
    }
    for (const auto& c : controllers_) {
        if (c.connected && c.has_sample) return c.slot;
    }
    return -1;
}

int GuiPanels::clients_receiving_slot(int slot) const {
    int n = 0;
    for (const auto& c : clients_) {
        if (c.subscribed_slots[slot]) n++;
    }
    return n;
}

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

void GuiPanels::draw() {
    refresh();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin("PSMove-DSU", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);

    draw_header();

    if (ImGui::BeginTabBar("tabs")) {
        if (ImGui::BeginTabItem("Controllers")) { draw_controllers_tab(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Button mapping")) { draw_buttons_tab(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Motion axes")) { draw_motion_tab(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Emulators")) { draw_emulators_tab(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Pairing")) { draw_pairing_tab(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("Log")) { draw_log_tab(); ImGui::EndTabItem(); }
        if (ImGui::BeginTabItem("About")) { draw_about_tab(); ImGui::EndTabItem(); }
        ImGui::EndTabBar();
    }

    ImGui::End();
}

void GuiPanels::draw_header() {
    ImGui::TextColored(COL_ACCENT, "PSMove-DSU");
    ImGui::SameLine();
    ImGui::TextDisabled("%s", DSU_VERSION);

    const char* copy_label = "Copy diagnostics";
    float copy_w = ImGui::CalcTextSize(copy_label).x + ImGui::GetStyle().FramePadding.x * 2;
    ImGui::SameLine(ImGui::GetContentRegionMax().x - copy_w);
    if (ImGui::Button(copy_label)) {
        ImGui::SetClipboardText(diagnostics_report().c_str());
        toast_ = "Diagnostics copied - paste them into your GitHub issue.";
        toast_until_ = Clock::now() + std::chrono::seconds(4);
    }
    ImGui::SetItemTooltip("Copies controller, emulator and timing status plus recent log lines,\n"
                          "formatted for a GitHub issue.");

    // Status chips
    if (server_status_.listening) {
        status_chip(fmt("Listening on UDP %d", server_status_.port), Level::Ok);
    } else {
        status_chip("Not listening", Level::Err);
    }
    ImGui::SameLine(0, 24 * ui_scale());

    int connected = 0;
    for (const auto& c : controllers_) connected += c.connected ? 1 : 0;
    status_chip(connected == 0 ? std::string("No controllers")
                               : fmt("%d controller%s", connected, connected == 1 ? "" : "s"),
                connected ? Level::Ok : Level::Warn);
    ImGui::SameLine(0, 24 * ui_scale());

    int receiving = 0;
    for (const auto& c : clients_) {
        bool any = false;
        for (bool b : c.subscribed_slots) any = any || b;
        receiving += any ? 1 : 0;
    }
    status_chip(receiving == 0 ? std::string("No emulator receiving data")
                               : fmt("%d emulator connection%s receiving data", receiving, receiving == 1 ? "" : "s"),
                receiving ? Level::Ok : Level::Warn);
    ImGui::SameLine(0, 24 * ui_scale());

    status_chip(manager_.realtime_priority ? "Real-time priority" : "Normal priority",
                manager_.realtime_priority ? Level::Ok : Level::Off);
    ImGui::SetItemTooltip(manager_.realtime_priority
        ? "The polling thread runs at elevated priority."
        : "The polling thread could not get elevated (real-time) priority.\n"
          "Usually fine. If input stutters under load, run with elevated rights\n"
          "(Linux: grant CAP_SYS_NICE, e.g. sudo setcap cap_sys_nice+ep <exe>).");

    if (Clock::now() < toast_until_) {
        ImGui::SameLine(0, 24 * ui_scale());
        ImGui::TextColored(COL_OK, "%s", toast_.c_str());
    }

    if (!server_status_.listening) {
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.30f, 0.12f, 0.13f, 1.0f));
        ImGui::BeginChild("neterr", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
        ImGui::TextColored(COL_ERR, "Emulators can't reach this server.");
        ImGui::TextWrapped("%s", server_status_.error.empty() ? "The UDP port could not be opened."
                                                              : server_status_.error.c_str());
        if (ImGui::Button("Retry")) server_.retry_network();
        ImGui::EndChild();
        ImGui::PopStyleColor();
    }
    ImGui::Spacing();
}

// ---------------------------------------------------------------------------
// Controllers
// ---------------------------------------------------------------------------

void GuiPanels::draw_controllers_tab() {
    ImGui::BeginChild("controllers_scroll");

    int connected = 0;
    for (const auto& c : controllers_) connected += c.connected ? 1 : 0;

    if (connected == 0) {
        ImGui::BeginChild("noctl", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
        ImGui::TextColored(COL_WARN, "No controllers connected.");
        ImGui::Spacing();
        ImGui::TextWrapped("1. If this controller has never been used with this computer, pair it first (Pairing tab).");
        ImGui::TextWrapped("2. Press the PS button on the controller. It should appear here within a few seconds.");
        ImGui::TextWrapped("3. The controller must be connected over Bluetooth. USB is only used for pairing.");
        if (!manager_.usb_serials.empty()) {
            ImGui::Spacing();
            for (const auto& s : manager_.usb_serials) {
                ImGui::TextColored(COL_WARN, "%s is plugged in over USB.", s.c_str());
            }
            ImGui::TextWrapped("Controllers can't send motion data over USB. Unplug the cable and press the PS button, "
                               "or pair it on the Pairing tab first.");
        }
        ImGui::Spacing();
        if (ImGui::Button("Scan now")) server_.controllers().request_scan();
        ImGui::EndChild();
    }

    for (const auto& c : controllers_) {
        if (c.connected) draw_controller_card(c);
    }

    std::string empty;
    for (const auto& c : controllers_) {
        if (!c.connected) empty += (empty.empty() ? "" : ", ") + std::to_string(c.slot);
    }
    if (!empty.empty() && connected > 0) {
        ImGui::TextDisabled("Empty slots: %s", empty.c_str());
    }

    ImGui::EndChild();
}

void GuiPanels::draw_controller_card(const ControllerStatus& st) {
    const float s = ui_scale();
    const SlotRates& rate = rates_[st.slot];
    ImGui::PushID(st.slot);
    ImGui::BeginChild("card", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);

    // Title row
    ImGui::TextColored(COL_ACCENT, "Slot %d", st.slot);
    ImGui::SameLine(0, 16 * s);
    ImGui::TextUnformatted(st.serial.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("%s  |  %s", st.model.c_str(), st.connection.c_str());
    ImGui::SameLine(0, 16 * s);
    status_chip("Battery " + battery_text(st.battery), battery_level(st.battery));
    if (st.rumble > 0) {
        ImGui::SameLine();
        status_chip(fmt("Rumble %d%%", (st.rumble * 100 + 127) / 255), Level::Warn);
        ImGui::SetItemTooltip("An emulator is making this controller vibrate.");
    }

    float id_w = ImGui::CalcTextSize("Identify").x + ImGui::GetStyle().FramePadding.x * 2;
    ImGui::SameLine(ImGui::GetContentRegionMax().x - id_w);
    ImGui::BeginDisabled(st.simulated);
    if (ImGui::Button("Identify")) server_.controllers().identify(st.slot);
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Lights up and rumbles this controller, so you can tell which one is in this slot.");

    // Health metrics
    double age_ms = st.sample_age_us / 1000.0;
    if (ImGui::BeginTable("metrics", 4, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        ImGui::TextDisabled("Reports / second");
        ImGui::Text("%.0f", rate.reports_per_s);
        ImGui::TableNextColumn();
        ImGui::TextDisabled("Data age");
        if (st.has_sample) ImGui::TextColored(age_ms > 100 ? COL_ERR : ImVec4(1, 1, 1, 1), "%.0f ms", age_ms);
        else ImGui::TextColored(COL_ERR, "no data yet");
        ImGui::TableNextColumn();
        ImGui::TextDisabled("Queued reports (peak)");
        help_marker("How many reports had piled up since the previous poll (worst case in the last second). "
                    "1-2 is normal. Higher numbers mean reports arrive in bursts, which is a sign of "
                    "Bluetooth or driver delay.");
        ImGui::TextColored(rate.backlog_peak > 6 ? COL_WARN : ImVec4(1, 1, 1, 1), "%u", rate.backlog_peak);
        ImGui::TableNextColumn();
        ImGui::TextDisabled("Connected for");
        ImGui::Text("%s", st.connected_since_us
            ? duration_text((std::chrono::duration_cast<std::chrono::microseconds>(
                  Clock::now().time_since_epoch()).count() - double(st.connected_since_us)) / 1e6).c_str()
            : "-");
        ImGui::EndTable();
    }

    // Warnings
    if (st.has_sample && age_ms > 250) {
        ImGui::TextColored(COL_ERR, "No new data for %s. Out of range, asleep, or the Bluetooth link dropped?",
                           duration_text(age_ms / 1000.0).c_str());
    } else if (!st.simulated && rate.reports_per_s > 0 && rate.reports_per_s < 40) {
        ImGui::TextColored(COL_WARN, "Low report rate. Bluetooth interference or a weak adapter can cause lag.");
    }
    if (rate.backlog_peak > 6) {
        ImGui::TextColored(COL_WARN, "Reports are arriving in bursts (%u at once). Input may feel delayed.",
                           rate.backlog_peak);
    }

    ImGui::Spacing();

    // Buttons
    const MoveSample& smp = st.sample;
    const float btn_w = 74 * s;
    for (size_t i = 0; i < MOVE_INPUT_COUNT; i++) {
        auto in = static_cast<MoveInput>(i);
        if (in == MoveInput::Trigger) continue;
        static const char* short_names[] = {"Cross", "Circle", "Square", "Triangle", "Move", "Start", "Select", "PS"};
        if (i > 0) ImGui::SameLine();
        indicator(short_names[i], (smp.buttons & Mapping::move_input_mask(in)) != 0, btn_w);
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::ProgressBar(smp.trigger / 255.0f, ImVec2(-FLT_MIN, ImGui::GetFrameHeight()),
                       fmt("T  %d", smp.trigger).c_str());

    // Motion plots
    ImGui::Spacing();
    float avail = ImGui::GetContentRegionAvail().x;
    float plot_w = (avail - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    float plot_h = 110 * s;
    const SlotHistory& h = history_[st.slot];

    auto draw_plot = [&](const char* id, int ch0, float min_range, const char* unit) {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImVec2 p1(p0.x + plot_w, p0.y + plot_h);
        ImGui::InvisibleButton(id, ImVec2(plot_w, plot_h));
        dl->AddRectFilled(p0, p1, IM_COL32(20, 22, 28, 255), 4.0f);

        float range = min_range;
        for (int ch = ch0; ch < ch0 + 3; ch++) {
            for (int i = 0; i < h.count; i++) range = std::max(range, std::fabs(h.v[ch][i]) * 1.1f);
        }
        float mid = (p0.y + p1.y) * 0.5f;
        dl->AddLine(ImVec2(p0.x, mid), ImVec2(p1.x, mid), IM_COL32(255, 255, 255, 28));

        ImVec2 pts[HISTORY];
        for (int ch = 0; ch < 3; ch++) {
            int n = h.count;
            for (int i = 0; i < n; i++) {
                int idx = (h.head - n + i + HISTORY) % HISTORY;
                float x = p0.x + plot_w * float(i + HISTORY - n) / float(HISTORY - 1);
                float y = mid - h.v[ch0 + ch][idx] / range * (plot_h * 0.5f - 3);
                pts[i] = ImVec2(x, y);
            }
            if (n > 1) dl->AddPolyline(pts, n, ImGui::GetColorU32(AXIS_COL[ch]), 0, 1.5f * s);
        }
        std::string lbl = fmt("+/- %.0f %s", range, unit);
        if (range < 10) lbl = fmt("+/- %.1f %s", range, unit);
        dl->AddText(ImVec2(p0.x + 6, p0.y + 4), ImGui::GetColorU32(COL_DIM), lbl.c_str());
    };

    const float RAD_TO_DEG = 57.2957795f;
    ImGui::BeginGroup();
    draw_plot("gyro", 0, 90.0f, "deg/s");
    ImGui::TextDisabled("Gyro (deg/s)");
    float gv[3] = {smp.gyro_x * RAD_TO_DEG, smp.gyro_y * RAD_TO_DEG, smp.gyro_z * RAD_TO_DEG};
    for (int a = 0; a < 3; a++) {
        ImGui::SameLine();
        ImGui::TextColored(AXIS_COL[a], "%s %7.1f", Mapping::axis_name(a), gv[a]);
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    draw_plot("accel", 3, 1.2f, "g");
    ImGui::TextDisabled("Accel (g)");
    float av[3] = {smp.accel_x, smp.accel_y, smp.accel_z};
    for (int a = 0; a < 3; a++) {
        ImGui::SameLine();
        ImGui::TextColored(AXIS_COL[a], "%s %6.2f", Mapping::axis_name(a), av[a]);
    }
    ImGui::EndGroup();

    // What actually goes out
    ImGui::Spacing();
    DsuButtonState out = DSUProtocol::encode_buttons(smp, config_);
    std::string pressed;
    for (size_t b = 1; b < DSU_BUTTON_COUNT; b++) {
        if (DSUProtocol::is_pressed(out, int(b))) {
            pressed += (pressed.empty() ? "" : ", ") + std::string(Mapping::dsu_button_name(static_cast<DsuButton>(b)));
        }
    }
    ImGui::TextDisabled("Sent to emulators:");
    ImGui::SameLine();
    ImGui::TextUnformatted(pressed.empty() ? "no buttons" : pressed.c_str());
    ImGui::SameLine(0, 24 * s);
    int receivers = clients_receiving_slot(st.slot);
    if (receivers > 0) {
        status_chip(fmt("%d emulator connection%s subscribed to this slot", receivers, receivers == 1 ? "" : "s"), Level::Ok);
    } else {
        status_chip("No emulator is subscribed to this slot", Level::Off);
    }

    ImGui::EndChild();
    ImGui::PopID();
    ImGui::Spacing();
}

// ---------------------------------------------------------------------------
// Button mapping
// ---------------------------------------------------------------------------

namespace {
// Combo listing connected controllers for the live preview.
void preview_selector(const std::array<ControllerStatus, Constants::MAX_CONTROLLERS>& ctls,
                      int current, int& selected) {
    ImGui::TextDisabled("Live preview from:");
    ImGui::SameLine();
    std::string label = current >= 0 ? fmt("Slot %d  %s", current, ctls[current].serial.c_str())
                                     : std::string("(no controller connected)");
    ImGui::SetNextItemWidth(260 * ui_scale());
    if (ImGui::BeginCombo("##preview", label.c_str())) {
        for (const auto& c : ctls) {
            if (!c.connected) continue;
            std::string l = fmt("Slot %d  %s", c.slot, c.serial.c_str());
            if (ImGui::Selectable(l.c_str(), c.slot == current)) selected = c.slot;
        }
        ImGui::EndCombo();
    }
}
} // namespace

void GuiPanels::draw_buttons_tab() {
    const float s = ui_scale();
    ImGui::BeginChild("buttons_scroll");

    int slot = preview_slot();
    preview_selector(controllers_, slot, selected_preview_slot_);
    MoveSample smp;
    smp.accel_z = 0;
    if (slot >= 0) smp = controllers_[slot].sample;
    DsuButtonState out = DSUProtocol::encode_buttons(smp, config_);

    ImGui::Spacing();
    if (ImGui::BeginTable("buttonmap", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                                           ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("PS Move button", ImGuiTableColumnFlags_WidthStretch, 1.2f);
        ImGui::TableSetupColumn("Pressed", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Sends to the emulator as", ImGuiTableColumnFlags_WidthStretch, 1.6f);
        ImGui::TableSetupColumn("Emulator sees", ImGuiTableColumnFlags_WidthStretch, 1.4f);
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < MOVE_INPUT_COUNT; i++) {
            auto in = static_cast<MoveInput>(i);
            DsuButton target = config_.button_map[i];
            ImGui::PushID(int(i));
            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(Mapping::move_input_name(in));

            ImGui::TableNextColumn();
            if (in == MoveInput::Trigger) {
                ImGui::ProgressBar(smp.trigger / 255.0f, ImVec2(-FLT_MIN, ImGui::GetFrameHeight()),
                                   fmt("%d", smp.trigger).c_str());
            } else {
                indicator("##pressed", (smp.buttons & Mapping::move_input_mask(in)) != 0, 80 * s);
            }

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::BeginCombo("##target", Mapping::dsu_button_name(target))) {
                for (size_t b = 0; b < DSU_BUTTON_COUNT; b++) {
                    auto db = static_cast<DsuButton>(b);
                    if (ImGui::Selectable(Mapping::dsu_button_name(db), db == target)) {
                        config_.button_map[i] = db;
                        apply_config();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::TableNextColumn();
            if (target != DsuButton::None) {
                bool on = DSUProtocol::is_pressed(out, int(target));
                indicator(Mapping::dsu_button_name(target), on, 110 * s);
                int ai = analog_index(target);
                if (ai >= 0) {
                    ImGui::SameLine();
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextDisabled("analog %3d", out.analog[ai]);
                }
            } else {
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("(not sent)");
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    // Warn about two inputs sharing a target
    for (size_t a = 0; a < MOVE_INPUT_COUNT; a++) {
        for (size_t b = a + 1; b < MOVE_INPUT_COUNT; b++) {
            if (config_.button_map[a] != DsuButton::None && config_.button_map[a] == config_.button_map[b]) {
                ImGui::TextColored(COL_WARN, "%s and %s both send %s.",
                                   Mapping::move_input_name(static_cast<MoveInput>(a)),
                                   Mapping::move_input_name(static_cast<MoveInput>(b)),
                                   Mapping::dsu_button_name(config_.button_map[a]));
            }
        }
    }

    ImGui::Spacing();
    ImGui::SetNextItemWidth(260 * s);
    ImGui::SliderInt("Trigger press threshold", &config_.trigger_threshold, 0, 254);
    if (ImGui::IsItemDeactivatedAfterEdit()) apply_config();
    help_marker("How far T must be pulled before it counts as a digital press. "
                "The analog value is always sent as well.");

    if (ImGui::Button("Reset buttons to defaults")) {
        config_.button_map = Config::default_button_map();
        config_.trigger_threshold = 10;
        apply_config();
    }
    if (!save_message_.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(save_error_ ? COL_ERR : COL_DIM, "%s", save_message_.c_str());
    }

    ImGui::Spacing();
    ImGui::TextWrapped("Tip: in the emulator's controller settings, click a binding and then press the PS Move button. "
                       "Dolphin reads Cross/Circle/Square/Triangle, the D-Pad, L1/R1 and L2/R2 from their analog values, "
                       "and Share, Options and PS from digital bits. PSMove-DSU sends both, so either kind of binding works.");

    if (ImGui::CollapsingHeader("Raw button bytes sent to emulators")) {
        ImGui::Text("Buttons1: 0x%02X   Buttons2: 0x%02X   Home: %d   Touch: %d",
                    out.buttons1, out.buttons2, out.home, out.touch);
        if (ImGui::BeginTable("analog", 12, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchSame)) {
            for (int i = 0; i < 12; i++) ImGui::TableSetupColumn(ANALOG_NAMES[i]);
            ImGui::TableHeadersRow();
            ImGui::TableNextRow();
            for (int i = 0; i < 12; i++) {
                ImGui::TableNextColumn();
                ImGui::Text("%d", out.analog[i]);
            }
            ImGui::EndTable();
        }
    }

    ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// Motion axes
// ---------------------------------------------------------------------------

void GuiPanels::draw_motion_tab() {
    const float s = ui_scale();
    ImGui::BeginChild("motion_scroll");

    int slot = preview_slot();
    preview_selector(controllers_, slot, selected_preview_slot_);
    MoveSample smp;
    smp.accel_z = 0;
    if (slot >= 0) smp = controllers_[slot].sample;
    DsuMotion m = DSUProtocol::map_motion(smp, config_);
    const float RAD_TO_DEG = 57.2957795f;
    float raw_gyro[3] = {smp.gyro_x * RAD_TO_DEG, smp.gyro_y * RAD_TO_DEG, smp.gyro_z * RAD_TO_DEG};
    float raw_accel[3] = {smp.accel_x, smp.accel_y, smp.accel_z};

    struct Row { const char* name; int* map; bool* flip; };

    auto axis_table = [&](const char* id, const char* title, Row* rows, const float* raw, const float* sent,
                          float range, const char* unit) {
        ImGui::SeparatorText(title);
        if (!ImGui::BeginTable(id, 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) return;
        ImGui::TableSetupColumn("Emulator axis", ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableSetupColumn("Comes from PS Move axis", ImGuiTableColumnFlags_WidthStretch, 1.1f);
        ImGui::TableSetupColumn("Invert", ImGuiTableColumnFlags_WidthStretch, 0.5f);
        ImGui::TableSetupColumn("PS Move value", ImGuiTableColumnFlags_WidthStretch, 0.8f);
        ImGui::TableSetupColumn("Sent to emulator", ImGuiTableColumnFlags_WidthStretch, 1.6f);
        ImGui::TableHeadersRow();
        for (int r = 0; r < 3; r++) {
            ImGui::PushID(r);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(rows[r].name);

            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(-FLT_MIN);
            int src = *rows[r].map;
            if (ImGui::BeginCombo("##src", fmt("%s axis", Mapping::axis_name(src)).c_str())) {
                for (int a = 0; a < 3; a++) {
                    if (ImGui::Selectable(fmt("%s axis", Mapping::axis_name(a)).c_str(), a == src)) {
                        *rows[r].map = a;
                        apply_config();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::TableNextColumn();
            if (ImGui::Checkbox("##flip", rows[r].flip)) apply_config();

            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            int a = *rows[r].map;
            ImGui::TextColored(AXIS_COL[a], "%s %7.2f", Mapping::axis_name(a), raw[a]);

            ImGui::TableNextColumn();
            center_bar(sent[r], range, std::max(60.0f * s, ImGui::GetContentRegionAvail().x - 125 * s));
            ImGui::SameLine();
            ImGui::Text("%7.2f %s", sent[r], unit);
            ImGui::PopID();
        }
        ImGui::EndTable();
    };

    Row gyro_rows[3] = {
        {"Pitch", &config_.map_pitch, &config_.flip_pitch},
        {"Yaw",   &config_.map_yaw,   &config_.flip_yaw},
        {"Roll",  &config_.map_roll,  &config_.flip_roll},
    };
    Row accel_rows[3] = {
        {"X", &config_.map_accel_x, &config_.flip_accel_x},
        {"Y", &config_.map_accel_y, &config_.flip_accel_y},
        {"Z", &config_.map_accel_z, &config_.flip_accel_z},
    };

    axis_table("gyro", "Gyroscope (rotation speed)", gyro_rows, raw_gyro, m.gyro, 360.0f, "deg/s");
    ImGui::Spacing();
    axis_table("accel", "Accelerometer (gravity and movement)", accel_rows, raw_accel, m.accel, 2.0f, "g");

    ImGui::Spacing();
    if (ImGui::Button("Reset axes to defaults")) {
        Config d;
        config_.map_pitch = d.map_pitch; config_.map_yaw = d.map_yaw; config_.map_roll = d.map_roll;
        config_.flip_pitch = d.flip_pitch; config_.flip_yaw = d.flip_yaw; config_.flip_roll = d.flip_roll;
        config_.map_accel_x = d.map_accel_x; config_.map_accel_y = d.map_accel_y; config_.map_accel_z = d.map_accel_z;
        config_.flip_accel_x = d.flip_accel_x; config_.flip_accel_y = d.flip_accel_y; config_.flip_accel_z = d.flip_accel_z;
        apply_config();
    }
    if (!save_message_.empty()) {
        ImGui::SameLine();
        ImGui::TextColored(save_error_ ? COL_ERR : COL_DIM, "%s", save_message_.c_str());
    }

    ImGui::Spacing();
    ImGui::TextWrapped("How to check: point the controller at the screen and tilt it up. Only Pitch should move. "
                       "Turn it left and right: only Yaw should move. Twist it: only Roll should move. "
                       "If the wrong bar moves, change where that axis comes from. If the right bar moves the wrong way "
                       "(or the cursor goes the wrong way in game), tick Invert.");
    ImGui::TextWrapped("Held still, the accelerometer reads about 1 g on whichever axis points straight up or down "
                       "(gravity) and close to 0 on the others.");

    ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// Emulators (DSU clients)
// ---------------------------------------------------------------------------

void GuiPanels::draw_emulators_tab() {
    // True if any connection from the same IP as `address` is receiving data.
    auto host_receiving = [this](const std::string& address) {
        std::string host = address.substr(0, address.rfind(':'));
        for (const auto& o : clients_) {
            if (o.address.substr(0, o.address.rfind(':')) != host) continue;
            for (bool b : o.subscribed_slots) {
                if (b) return true;
            }
        }
        return false;
    };

    ImGui::BeginChild("emu_scroll");

    if (server_status_.listening) {
        status_chip(fmt("Listening on UDP port %d on all network interfaces", server_status_.port), Level::Ok);
        ImGui::SameLine(0, 24 * ui_scale());
        ImGui::TextDisabled("%.0f packets/s sent", total_packet_rate_);
    } else {
        status_chip("Not listening: " + server_status_.error, Level::Err);
    }

    ImGui::Spacing();
    ImGui::TextWrapped("In your emulator, add a DSU / Cemuhook server with address 127.0.0.1 and port %d "
                       "(or this computer's network IP if the emulator runs on another device).",
                       server_status_.port);
    ImGui::Spacing();

    if (clients_.empty()) {
        ImGui::BeginChild("noclients", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
        ImGui::TextColored(COL_WARN, "No emulator has contacted this server yet.");
        ImGui::Spacing();
        ImGui::TextWrapped("- Check that the emulator's DSU server address is 127.0.0.1 and the port is %d.",
                           server_status_.port);
        ImGui::TextWrapped("- In Dolphin: Controllers > Alternate Input Sources > enable DSU Client and add the server. "
                           "In Cemu: Input settings > API: DSUController.");
        ImGui::TextWrapped("- Only one DSU server can use the port. Close DS4Windows, BetterJoy or other copies of "
                           "PSMove-DSU (if one of them started first, this program shows a port error above).");
        ImGui::TextWrapped("- If the emulator runs on another device, allow UDP port %d through this computer's firewall.",
                           server_status_.port);
        ImGui::EndChild();
    } else {
        if (ImGui::BeginTable("clients", 7, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                                            ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Client", ImGuiTableColumnFlags_WidthStretch, 1.2f);
            ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch, 1.6f);
            ImGui::TableSetupColumn("Receiving slots", ImGuiTableColumnFlags_WidthStretch, 0.9f);
            ImGui::TableSetupColumn("Last subscription", ImGuiTableColumnFlags_WidthStretch, 1.1f);
            ImGui::TableSetupColumn("Last heard from", ImGuiTableColumnFlags_WidthStretch, 0.8f);
            ImGui::TableSetupColumn("Requests (ver/info/data)", ImGuiTableColumnFlags_WidthStretch, 1.1f);
            ImGui::TableSetupColumn("Packets/s", ImGuiTableColumnFlags_WidthStretch, 0.6f);
            ImGui::TableHeadersRow();

            for (const auto& c : clients_) {
                bool receiving = false;
                std::string slots;
                for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
                    if (!c.subscribed_slots[i]) continue;
                    receiving = true;
                    slots += (slots.empty() ? "" : ", ") + std::to_string(i);
                }
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(c.address.c_str());

                ImGui::TableNextColumn();
                if (receiving) {
                    status_chip("Receiving data", Level::Ok);
                } else if (c.data_requests > 0) {
                    status_chip("Stopped requesting data", Level::Off);
                    ImGui::SetItemTooltip("The client hasn't renewed its subscription in the last %d s. "
                                          "This is normal after closing a game or the controller settings.",
                                          Constants::SUBSCRIPTION_TIMEOUT_MS / 1000);
                } else if (c.info_requests > 0 && host_receiving(c.address)) {
                    // e.g. Dolphin lists controllers on one socket and streams on others
                    status_chip("Controller discovery", Level::Off);
                    ImGui::SetItemTooltip("This connection only asks which controllers exist. Another connection "
                                          "from the same program is receiving the data.");
                } else if (c.info_requests > 0) {
                    status_chip("Sees controllers, not using one", Level::Warn);
                    ImGui::SetItemTooltip("The emulator asked which controllers exist but hasn't asked for data.\n"
                                          "It requests data once a DSU device is selected in a controller profile\n"
                                          "and a game (or the controller settings window) is running.");
                } else {
                    status_chip("Seen", Level::Off);
                }

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(slots.empty() ? "-" : (c.subscribed_all ? "all" : slots.c_str()));

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(c.last_request.empty() ? "-" : c.last_request.c_str());

                ImGui::TableNextColumn();
                ImGui::Text("%s ago", duration_text(c.seconds_since_seen).c_str());

                ImGui::TableNextColumn();
                ImGui::Text("%llu / %llu / %llu", (unsigned long long)c.version_requests,
                            (unsigned long long)c.info_requests, (unsigned long long)c.data_requests);

                ImGui::TableNextColumn();
                auto it = client_packet_rate_.find(c.address);
                ImGui::Text("%.0f", it != client_packet_rate_.end() ? it->second : 0.0);
            }
            ImGui::EndTable();
        }

        // Subscriptions that can't be served
        for (const auto& c : clients_) {
            for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
                if (c.subscribed_slots[i] && !c.subscribed_all && !controllers_[i].connected) {
                    ImGui::TextColored(COL_WARN, "%s is asking for slot %d, but no controller is in that slot.",
                                       c.address.c_str(), i);
                }
            }
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Each emulator controller usually opens its own connection, so one emulator can appear "
                            "several times (e.g. one row per DSU device in Dolphin).");
    }

    ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// Pairing
// ---------------------------------------------------------------------------

void GuiPanels::draw_pairing_tab() {
    ImGui::BeginChild("pair_scroll");
    ImGui::TextWrapped("Pairing stores this computer's Bluetooth address in the controller, so it connects to this "
                       "computer when you press the PS button. You only need to do it once. If you later connect the "
                       "controller to a PS3 or pair it with another PC, you'll need to pair it here again.");
    ImGui::Spacing();
    ImGui::TextWrapped("1. Connect the PS Move to this computer with a USB cable (it doesn't need to be switched on).");
    ImGui::TextWrapped("2. Wait for it to appear below, then click Pair.");
    ImGui::TextWrapped("3. Unplug the cable and press the PS button. The red LED blinks, then stays lit once connected "
                       "over Bluetooth. The controller then shows up on the Controllers tab.");
    ImGui::TextDisabled("Over USB a controller can only be paired, charged, or have its LED and rumble set. "
                        "Motion data needs Bluetooth.");

    ImGui::SeparatorText("Controllers on USB");
    bool running = pair_.state == PairStatus::State::Running;
    if (manager_.usb_serials.empty()) {
        ImGui::TextDisabled(manager_.scanning ? "Scanning..." : "None found. Scans run every 5 seconds.");
    }
    for (const auto& serial : manager_.usb_serials) {
        ImGui::PushID(serial.c_str());
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(serial.c_str());
        ImGui::SameLine();
        ImGui::BeginDisabled(running);
        if (ImGui::Button("Pair")) server_.controllers().request_pair(serial);
        ImGui::EndDisabled();
        ImGui::PopID();
    }
    ImGui::BeginDisabled(running);
    if (ImGui::Button("Scan now")) server_.controllers().request_scan();
    ImGui::EndDisabled();

    if (pair_.state != PairStatus::State::Idle) {
        ImGui::Spacing();
        Level lvl = pair_.state == PairStatus::State::Succeeded ? Level::Ok
                  : pair_.state == PairStatus::State::Failed    ? Level::Err : Level::Warn;
        ImGui::BeginChild("pairstatus", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
        status_chip(pair_.state == PairStatus::State::Running   ? "Pairing..."
                  : pair_.state == PairStatus::State::Succeeded ? "Paired" : "Pairing failed", lvl);
        ImGui::TextWrapped("%s", pair_.message.c_str());
        ImGui::EndChild();
    }

    ImGui::SeparatorText("Navigation controller (Nunchuk)");
    ImGui::TextWrapped("The PS Move Navigation controller is paired the same way (a USB cable, then the PS button). "
                       "It is not read by this program: once connected it appears as a normal gamepad, so add it as "
                       "a second input source in your emulator, e.g. Dolphin's Nunchuk profile.");
    ImGui::TextWrapped("Connect it with a USB cable, then click the button (it waits up to 30 seconds). Afterwards, "
                       "unplug it and press the PS button. If it doesn't connect, turn Bluetooth off and on or "
                       "restart your computer, then press PS again.");
#if defined(__linux__)
    ImGui::TextDisabled("Linux: needs the udev rules from --linux-setup (or root). With them, no root is needed.");
#endif
    {
        bool nav_running = nav_status_.state == NavPairTask::State::Running;
        ImGui::BeginDisabled(nav_running);
        if (ImGui::Button("Pair Navigation controller")) nav_pair_.start();
        ImGui::EndDisabled();
        if (nav_status_.state != NavPairTask::State::Idle) {
            Level lvl = nav_status_.state == NavPairTask::State::Succeeded ? Level::Ok
                      : nav_status_.state == NavPairTask::State::Failed    ? Level::Err : Level::Warn;
            ImGui::BeginChild("navpairstatus", ImVec2(0, 0), ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
            status_chip(nav_running ? "Pairing..." : nav_status_.state == NavPairTask::State::Succeeded
                                                         ? "Paired" : "Pairing failed", lvl);
            ImGui::TextWrapped("%s", nav_status_.message.c_str());
            ImGui::EndChild();
        }
    }

    ImGui::SeparatorText("Connected over Bluetooth");
    bool any = false;
    for (const auto& c : controllers_) {
        if (!c.connected || c.simulated) continue;
        any = true;
        ImGui::Text("Slot %d: %s (%s)", c.slot, c.serial.c_str(), c.model.c_str());
    }
    if (!any) ImGui::TextDisabled("None");

    ImGui::SeparatorText("Troubleshooting");
    ImGui::TextWrapped("If the red LED keeps blinking and never stays lit, the controller isn't paired with this "
                       "computer, or its battery is low (charge it over USB). You can also check your system's Bluetooth "
                       "devices list for an entry called \"Motion Controller\".");
    ImGui::Spacing();
#if defined(_WIN32)
    ImGui::TextWrapped("Windows: pairing can take several attempts before Bluetooth connects. Make sure Bluetooth is on "
                       "and try running PSMove-DSU as administrator. Use the Microsoft Bluetooth stack; third-party "
                       "drivers (e.g. MotionInJoy) interfere with the PS Move. If the controller pairs but won't "
                       "connect, remove it under Settings > Bluetooth & devices and pair again.");
#elif defined(__APPLE__)
    ImGui::TextWrapped("macOS: allow PSMove-DSU to use Bluetooth when asked (System Settings > Privacy & Security > "
                       "Bluetooth). If a PIN dialog appears when you press the PS button, close it and pair again here. "
                       "Pairing may ask for your password.");
    ImGui::TextWrapped("The built-in Bluetooth adapter supports at most 2 Move controllers at once on macOS Sierra and "
                       "later. For more, use an external Bluetooth adapter.");
#else
    ImGui::TextWrapped("Linux: pairing a Move registers it with BlueZ, which needs root even with the udev rules. If "
                       "pairing fails, close this window and run once from a terminal:  sudo ./<this program> --pair  "
                       "(The Navigation controller is different: with the udev rules it pairs without root.)");
    ImGui::TextWrapped("To use controllers without root, or if one pairs but won't connect (BlueZ may reject unbonded "
                       "input devices), run once from a terminal:  sudo ./<this program> --linux-setup  "
                       "It asks before installing udev rules and before changing ClassicBondedOnly in "
                       "/etc/bluetooth/input.conf (which weakens a Bluetooth security fix).");
#endif
    ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// Log
// ---------------------------------------------------------------------------

void GuiPanels::draw_about_tab() {
    ImGui::TextColored(COL_ACCENT, "PSMove-DSU");
    ImGui::SameLine();
    ImGui::TextDisabled("%s", DSU_VERSION);
    ImGui::TextLinkOpenURL("https://github.com/Swordmaster3214/PSMove-DSU");
    ImGui::Spacing();

    ImGui::TextWrapped("%s", Licenses::project_notice());
    ImGui::Spacing();

    // Each text gets its own scrolling area, sharing the rest of the tab.
    if (ImGui::BeginTabBar("license_texts")) {
        const char* texts[2][2] = {
            {"Third-party licenses", Licenses::third_party_text()},
            {"GNU GPL v3", Licenses::gpl_text()},
        };
        for (const auto& t : texts) {
            if (ImGui::BeginTabItem(t[0])) {
                if (ImGui::SmallButton("Copy")) ImGui::SetClipboardText(t[1]);
                ImGui::BeginChild("text", ImVec2(0, 0), ImGuiChildFlags_Borders);
                ImGui::TextUnformatted(t[1]);
                ImGui::EndChild();
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }
}

void GuiPanels::draw_log_tab() {
    bool verbose = Logger::is_verbose();
    if (ImGui::Checkbox("Verbose", &verbose)) Logger::set_verbose(verbose);
    ImGui::SetItemTooltip("Same as --verbose: includes debug messages and latency warnings.");
    ImGui::SameLine(0, 20 * ui_scale());
    ImGui::Checkbox("Debug", &log_show_[0]); ImGui::SameLine();
    ImGui::Checkbox("Info", &log_show_[1]); ImGui::SameLine();
    ImGui::Checkbox("Warnings", &log_show_[2]); ImGui::SameLine();
    ImGui::Checkbox("Errors", &log_show_[3]);
    ImGui::SameLine(0, 20 * ui_scale());
    ImGui::SetNextItemWidth(200 * ui_scale());
    ImGui::InputTextWithHint("##filter", "Filter", log_filter_, sizeof(log_filter_));
    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &log_autoscroll_);

    std::vector<int> visible;
    visible.reserve(log_.size());
    for (int i = 0; i < int(log_.size()); i++) {
        const LogEntry& e = log_[i];
        if (!log_show_[int(e.level)]) continue;
        if (!contains_ci(e.message, log_filter_)) continue;
        visible.push_back(i);
    }

    ImGui::SameLine();
    if (ImGui::Button("Copy")) {
        std::string text;
        for (int i : visible) {
            const LogEntry& e = log_[i];
            text += fmt("%9.3f [%s] ", e.time_s, Logger::level_name(e.level)) + e.message + "\n";
        }
        ImGui::SetClipboardText(text.c_str());
        toast_ = "Log copied.";
        toast_until_ = Clock::now() + std::chrono::seconds(3);
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        log_.clear();
        Logger::clear_history();
        visible.clear();
    }

    ImGui::BeginChild("logview", ImVec2(0, 0), ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
    ImGuiListClipper clipper;
    clipper.Begin(int(visible.size()));
    while (clipper.Step()) {
        for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; row++) {
            const LogEntry& e = log_[visible[row]];
            ImVec4 col = e.level == LogLevel::Error ? COL_ERR
                       : e.level == LogLevel::Warn  ? COL_WARN
                       : e.level == LogLevel::Debug ? COL_DIM : ImVec4(0.88f, 0.90f, 0.94f, 1.0f);
            float x0 = ImGui::GetCursorPosX();
            ImGui::TextDisabled("%9.3f", e.time_s);
            ImGui::SameLine(x0 + 80 * ui_scale());
            ImGui::TextColored(col, "%s", Logger::level_name(e.level));
            ImGui::SameLine(x0 + 135 * ui_scale());
            ImGui::PushStyleColor(ImGuiCol_Text, col);
            ImGui::TextUnformatted(e.message.c_str());
            ImGui::PopStyleColor();
        }
    }
    if (log_autoscroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4) {
        ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

std::string GuiPanels::diagnostics_report() {
    std::ostringstream o;
    o << "<details><summary>PSMove-DSU diagnostics</summary>\n\n```\n";
    o << "Version: " << DSU_VERSION << "   OS: " << os_name() << "\n";
    o << "Uptime: " << duration_text(server_status_.uptime_s) << "\n";
    o << "Server: " << (server_status_.listening ? fmt("listening on UDP %d", server_status_.port)
                                                 : "NOT listening - " + server_status_.error) << "\n";
    o << "Polling: " << (manager_.realtime_priority ? "real-time priority" : "normal priority")
      << ", loop overruns " << manager_.loop_overruns << ", peak loop " << loop_peak_us_ << " us\n";
    o << "Packets sent: " << server_status_.packets_sent << fmt(" (%.0f/s)", total_packet_rate_) << "\n\n";

    o << "Controllers:\n";
    for (const auto& c : controllers_) {
        const SlotRates& r = rates_[c.slot];
        if (!c.connected) {
            o << fmt("  Slot %d: empty\n", c.slot);
            continue;
        }
        o << fmt("  Slot %d: %s %s %s | battery %s | %.0f reports/s | queued peak %u | data age %.0f ms | empty polls %.0f%%\n",
                 c.slot, c.serial.c_str(), c.model.c_str(), c.connection.c_str(), battery_text(c.battery).c_str(),
                 r.reports_per_s, r.backlog_peak, c.sample_age_us / 1000.0, r.empty_fraction * 100.0);
    }
    if (!manager_.usb_serials.empty()) {
        o << "  On USB:";
        for (const auto& s : manager_.usb_serials) o << " " << s;
        o << "\n";
    }

    o << "\nEmulator connections:\n";
    if (clients_.empty()) o << "  none\n";
    for (const auto& c : clients_) {
        std::string slots;
        for (int i = 0; i < Constants::MAX_CONTROLLERS; i++) {
            if (c.subscribed_slots[i]) slots += (slots.empty() ? "" : ",") + std::to_string(i);
        }
        auto it = client_packet_rate_.find(c.address);
        o << fmt("  %s | receiving [%s] | last subscription: %s | requests v/i/d %llu/%llu/%llu | %.0f pkt/s | seen %s ago\n",
                 c.address.c_str(), slots.c_str(), c.last_request.empty() ? "-" : c.last_request.c_str(),
                 (unsigned long long)c.version_requests, (unsigned long long)c.info_requests,
                 (unsigned long long)c.data_requests, it != client_packet_rate_.end() ? it->second : 0.0,
                 duration_text(c.seconds_since_seen).c_str());
    }

    o << "\nButton mapping:";
    for (size_t i = 0; i < MOVE_INPUT_COUNT; i++) {
        o << " " << Mapping::move_input_key(static_cast<MoveInput>(i)) << "->"
          << Mapping::dsu_button_key(config_.button_map[i]);
    }
    o << fmt("\nGyro: pitch=%s%s yaw=%s%s roll=%s%s   Accel: x=%s%s y=%s%s z=%s%s\n",
             config_.flip_pitch ? "-" : "", Mapping::axis_name(config_.map_pitch),
             config_.flip_yaw ? "-" : "", Mapping::axis_name(config_.map_yaw),
             config_.flip_roll ? "-" : "", Mapping::axis_name(config_.map_roll),
             config_.flip_accel_x ? "-" : "", Mapping::axis_name(config_.map_accel_x),
             config_.flip_accel_y ? "-" : "", Mapping::axis_name(config_.map_accel_y),
             config_.flip_accel_z ? "-" : "", Mapping::axis_name(config_.map_accel_z));

    o << "\nRecent log:\n";
    size_t start = log_.size() > 80 ? log_.size() - 80 : 0;
    for (size_t i = start; i < log_.size(); i++) {
        const LogEntry& e = log_[i];
        o << fmt("%9.3f [%s] ", e.time_s, Logger::level_name(e.level)) << e.message << "\n";
    }
    o << "```\n</details>\n";
    return o.str();
}
