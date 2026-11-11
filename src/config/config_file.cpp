#include "config/config_file.hpp"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace {

std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string env(const char* name) {
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
}

bool parse_int(const std::string& s, int& out) {
    try {
        size_t used = 0;
        int v = std::stoi(s, &used);
        if (used != s.size()) return false;
        out = v;
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_bool(const std::string& s, bool& out) {
    if (s == "1" || s == "true" || s == "yes" || s == "on") { out = true; return true; }
    if (s == "0" || s == "false" || s == "no" || s == "off") { out = false; return true; }
    return false;
}

struct AxisSetting {
    const char* map_key;
    const char* flip_key;
    int Config::*map;
    bool Config::*flip;
};

const AxisSetting AXES[] = {
    {"gyro_pitch_axis", "gyro_pitch_invert", &Config::map_pitch,   &Config::flip_pitch},
    {"gyro_yaw_axis",   "gyro_yaw_invert",   &Config::map_yaw,     &Config::flip_yaw},
    {"gyro_roll_axis",  "gyro_roll_invert",  &Config::map_roll,    &Config::flip_roll},
    {"accel_x_axis",    "accel_x_invert",    &Config::map_accel_x, &Config::flip_accel_x},
    {"accel_y_axis",    "accel_y_invert",    &Config::map_accel_y, &Config::flip_accel_y},
    {"accel_z_axis",    "accel_z_invert",    &Config::map_accel_z, &Config::flip_accel_z},
};

} // namespace

namespace ConfigFile {

std::string default_path() {
    try {
        fs::path dir;
#if defined(_WIN32)
        std::string appdata = env("APPDATA");
        dir = appdata.empty() ? fs::path(".") : fs::path(appdata) / "PSMove-DSU";
#elif defined(__APPLE__)
        std::string home = env("HOME");
        dir = home.empty() ? fs::path(".")
                           : fs::path(home) / "Library" / "Application Support" / "PSMove-DSU";
#else
        std::string xdg = env("XDG_CONFIG_HOME");
        std::string home = env("HOME");
        if (!xdg.empty()) dir = fs::path(xdg) / "psmove-dsu";
        else if (!home.empty()) dir = fs::path(home) / ".config" / "psmove-dsu";
        else dir = fs::path(".");
#endif
        return (dir / "config.ini").string();
    } catch (...) {
        // e.g. a Windows profile path that can't be represented in the ANSI code page
        return "config.ini";
    }
}

bool load(const std::string& path, Config& config, std::string& error) {
    std::error_code ec;
    if (!fs::exists(path, ec)) return true;

    std::ifstream in(path);
    if (!in) {
        error = "Could not open " + path;
        return false;
    }

    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';' || line[0] == '[') continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = trim(line.substr(0, eq));
        std::string value = trim(line.substr(eq + 1));

        if (key == "trigger_threshold") {
            int v;
            if (parse_int(value, v) && v >= 0 && v <= 255) config.trigger_threshold = v;
            continue;
        }

        bool handled = false;
        for (const auto& a : AXES) {
            int iv; bool bv;
            if (key == a.map_key && parse_int(value, iv) && iv >= 0 && iv <= 2) {
                config.*(a.map) = iv;
                handled = true;
            } else if (key == a.flip_key && parse_bool(value, bv)) {
                config.*(a.flip) = bv;
                handled = true;
            }
        }
        if (handled) continue;

        const std::string prefix = "button.";
        if (key.rfind(prefix, 0) == 0) {
            std::string input_key = key.substr(prefix.size());
            for (size_t i = 0; i < MOVE_INPUT_COUNT; i++) {
                if (input_key == Mapping::move_input_key(static_cast<MoveInput>(i))) {
                    DsuButton b;
                    if (Mapping::dsu_button_from_key(value, b)) config.button_map[i] = b;
                }
            }
        }
    }
    return true;
}

bool save(const std::string& path, const Config& config, std::string& error) {
    std::error_code ec;
    fs::path p(path);
    if (p.has_parent_path()) fs::create_directories(p.parent_path(), ec);

    std::ostringstream out;
    out << "# PSMove-DSU settings. Edited by the GUI; safe to edit by hand.\n"
        << "# Axis values: 0 = X, 1 = Y, 2 = Z of the PS Move sensor.\n\n";
    for (const auto& a : AXES) {
        out << a.map_key << " = " << config.*(a.map) << "\n";
        out << a.flip_key << " = " << (config.*(a.flip) ? 1 : 0) << "\n";
    }
    out << "\ntrigger_threshold = " << config.trigger_threshold << "\n\n";
    out << "# Button values: none, cross, circle, square, triangle, l1, r1, l2, r2, l3, r3,\n"
        << "# share, options, home, touch, dpad_up, dpad_down, dpad_left, dpad_right\n";
    for (size_t i = 0; i < MOVE_INPUT_COUNT; i++) {
        out << "button." << Mapping::move_input_key(static_cast<MoveInput>(i)) << " = "
            << Mapping::dsu_button_key(config.button_map[i]) << "\n";
    }

    // Write to a temp file then rename, so a crash can't leave a half-written config.
    std::string tmp = path + ".tmp";
    {
        std::ofstream f(tmp, std::ios::trunc);
        if (!f) {
            error = "Could not write " + tmp;
            return false;
        }
        f << out.str();
        if (!f) {
            error = "Could not write " + tmp;
            return false;
        }
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        // rename over an existing file can fail on some Windows setups; retry after removing
        fs::remove(path, ec);
        fs::rename(tmp, path, ec);
        if (ec) {
            error = "Could not replace " + path + ": " + ec.message();
            return false;
        }
    }
    return true;
}

} // namespace ConfigFile
