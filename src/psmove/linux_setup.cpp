#include "psmove/linux_setup.hpp"

#if defined(__linux__)

#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

const char* const kUdevPath = "/etc/udev/rules.d/99-psmove.rules";
const char* const kInputConf = "/etc/bluetooth/input.conf";

// Same content as PSMoveAPI's contrib/99-psmove.rules (without the obsolete libusb line).
const char* const kUdevRules =
    "# hidraw interface: Bluetooth (=0005), PS Move Motion Controller (=054c:03d5)\n"
    "SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:03D5.*\", MODE=\"0666\"\n"
    "\n"
    "# hidraw interface: USB (=0003), PS Move Motion Controller (=054c:03d5)\n"
    "SUBSYSTEM==\"hidraw\", KERNELS==\"0003:054C:03D5.*\", MODE=\"0666\"\n"
    "\n"
    "# hidraw interface: Bluetooth (=0005), PS4 Move Motion Controller, CECH-ZCM2 (=054c:0c5e)\n"
    "SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:0C5E.*\", MODE=\"0666\"\n"
    "\n"
    "# hidraw interface: USB (=0003), PS4 Move Motion Controller, CECH-ZCM2 (=054c:0c5e)\n"
    "SUBSYSTEM==\"hidraw\", KERNELS==\"0003:054C:0C5E.*\", MODE=\"0666\"\n";

bool ask(const std::string& question, bool default_yes) {
    std::cout << question << (default_yes ? " [Y/n] " : " [y/N] ") << std::flush;
    std::string line;
    if (!std::getline(std::cin, line) || line.empty()) return default_yes;
    return line[0] == 'y' || line[0] == 'Y';
}

bool read_file(const std::string& path, std::string& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

bool write_file(const std::string& path, const std::string& data) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) return false;
    f << data;
    return static_cast<bool>(f);
}

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r");
    return s.substr(a, b - a + 1);
}

void install_udev_rules() {
    std::string existing;
    if (read_file(kUdevPath, existing) && existing == kUdevRules) {
        std::cout << "  udev rules are already installed.\n";
        return;
    }
    if (!write_file(kUdevPath, kUdevRules)) {
        std::cout << "  Could not write " << kUdevPath << ".\n";
        return;
    }
    std::cout << "  Wrote " << kUdevPath << ".\n";
    int rc = std::system("udevadm control --reload-rules && udevadm trigger");
    if (rc != 0) std::cout << "  Could not reload udev; run 'sudo udevadm trigger' or reboot.\n";
    else std::cout << "  Reloaded udev rules. Replug the controller if it was already connected.\n";
}

// Sets ClassicBondedOnly=false in the [General] section of BlueZ's input.conf.
void relax_bluez_input() {
    std::string text;
    if (!read_file(kInputConf, text)) {
        std::cout << "  " << kInputConf << " not found; skipping. (Your BlueZ version may not need this.)\n";
        return;
    }

    std::vector<std::string> lines;
    {
        std::stringstream ss(text);
        std::string l;
        while (std::getline(ss, l)) lines.push_back(l);
    }

    int general_end = -1;   // index to insert at if the key is missing
    bool in_general = false, found_general = false, done = false;
    for (size_t i = 0; i < lines.size(); i++) {
        std::string t = trim(lines[i]);
        if (!t.empty() && t[0] == '[') {
            if (in_general && general_end < 0) general_end = static_cast<int>(i);
            in_general = (t == "[General]");
            found_general = found_general || in_general;
            continue;
        }
        if (!in_general) continue;
        std::string key = t;
        if (!key.empty() && key[0] == '#') key = trim(key.substr(1));
        if (key.rfind("ClassicBondedOnly", 0) == 0) {
            if (t[0] != '#' && t.find("false") != std::string::npos) {
                std::cout << "  ClassicBondedOnly is already false.\n";
                return;
            }
            if (!done) { lines[i] = "ClassicBondedOnly=false"; done = true; }
            else if (t[0] != '#') lines[i] = "#" + lines[i];   // avoid a conflicting duplicate
        }
    }
    if (!done) {
        if (!found_general) { lines.push_back("[General]"); lines.push_back("ClassicBondedOnly=false"); }
        else if (general_end >= 0) lines.insert(lines.begin() + general_end, "ClassicBondedOnly=false");
        else lines.push_back("ClassicBondedOnly=false");
    }

    std::string out;
    for (const auto& l : lines) out += l + "\n";

    std::string backup = std::string(kInputConf) + ".psmove-dsu.bak";
    if (!write_file(backup, text)) {
        std::cout << "  Could not write backup " << backup << "; leaving input.conf unchanged.\n";
        return;
    }
    if (!write_file(kInputConf, out)) {
        std::cout << "  Could not write " << kInputConf << ".\n";
        return;
    }
    std::cout << "  Updated " << kInputConf << " (backup: " << backup << ").\n";
    std::cout << "  Restart Bluetooth for it to take effect:  sudo systemctl restart bluetooth\n";
}

} // namespace

int run_linux_setup() {
    std::cout << "\n=== Optional Linux setup ===\n";
    if (geteuid() != 0) {
        std::cout << "Not running as root, so system setup is skipped. To do it, run:\n"
                     "  sudo ./<this program> --linux-setup\n";
        return 0;
    }

    std::cout << "\nudev rules let you use controllers without root (they make PS Move hidraw devices world-accessible).\n";
    if (ask("Install them to " + std::string(kUdevPath) + "?", true)) install_udev_rules();

    std::cout << "\nMany distros' BlueZ rejects unbonded input devices like the PS Move. If your controller pairs but\n"
                 "won't connect, setting ClassicBondedOnly=false allows it. This partially undoes a Bluetooth\n"
                 "security fix (GHSA-qjcj-xg77-6c32), so only do it if you need it.\n";
    if (ask("Set ClassicBondedOnly=false in " + std::string(kInputConf) + "?", false)) relax_bluez_input();
    return 0;
}

#else

int run_linux_setup() { return 0; }

#endif
