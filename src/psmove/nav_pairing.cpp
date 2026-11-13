#include "psmove/nav_pairing.hpp"
#include "utils/logging.hpp"
#include <hidapi.h>
#include <psmove.h>
#include <psmove_port.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace {
constexpr unsigned short kSonyVid = 0x054c;
constexpr unsigned short kNavPid = 0x042f;
constexpr unsigned char kMasterReport = 0xF5;  // feature report holding the Bluetooth master address

bool parse_mac(const char* s, unsigned char out[6]) {
    unsigned v[6];
    if (!s || std::sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
    for (int i = 0; i < 6; i++) {
        if (v[i] > 0xFF) return false;
        out[i] = static_cast<unsigned char>(v[i]);
    }
    return true;
}

std::string mac_string(const unsigned char* m) {
    char buf[18];
    std::snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
    return buf;
}
}  // namespace

int NavPairing::run_pairing_mode() {
    std::cout << "\n=== Navigation Controller Pairing Mode ===\n";

    char* host_str = psmove_port_get_host_bluetooth_address();
    unsigned char host[6];
    if (!host_str || !parse_mac(host_str, host)) {
        Logger::error("Can't determine this computer's Bluetooth address. Make sure Bluetooth is turned on.");
        if (host_str) psmove_free_mem(host_str);
        return 1;
    }
    psmove_free_mem(host_str);

    if (hid_init() != 0) {
        Logger::error("Could not initialise the USB HID layer.");
        return 1;
    }

    std::cout << "Connect the Navigation controller to your computer with a USB cable.\n"
                 "Press its PS button if it isn't detected. Detecting...\n";

    hid_device* dev = nullptr;
    for (int attempt = 0; attempt < 60 && !dev; attempt++) {
        hid_device_info* devs = hid_enumerate(kSonyVid, kNavPid);
        if (devs) {
            dev = hid_open_path(devs->path);
            if (!dev) Logger::debug("Navigation controller found but could not be opened (permissions?)");
            hid_free_enumeration(devs);
        }
        if (!dev) {
            std::cout << "." << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    std::cout << "\n";

    if (!dev) {
        Logger::error("No Navigation controller found on USB. On Linux this may need root or the udev rules "
                      "(see --linux-setup).");
        hid_exit();
        return 1;
    }

    // Report 0xF5 payload is 8 bytes: two header bytes, then the 6-byte master address. hidraw
    // puts the report ID into the data stage as the first byte, so when writing, the ID takes
    // the place of the first header byte (sixpair's 01 00 prefix must not be sent on top of it,
    // or the address ends up shifted by one byte; verified on hardware).
    constexpr int kMacOffset = 2;
    auto read_master = [&](unsigned char out[6]) {
        unsigned char buf[9] = {kMasterReport};
        int got = hid_get_feature_report(dev, buf, sizeof(buf));
        std::string hex;
        for (int i = 0; i < (got > 0 ? got : 0); i++) {
            char b[4];
            std::snprintf(b, sizeof(b), "%02x ", buf[i]);
            hex += b;
        }
        Logger::debug("Report 0xF5 read (" + std::to_string(got) + " bytes): " + hex);
        if (got < kMacOffset + 6) return false;  // 8 bytes are valid; a 9th may be junk
        std::memcpy(out, buf + kMacOffset, 6);
        return true;
    };

    unsigned char cur[6];
    if (read_master(cur)) {
        std::cout << "Current Bluetooth master: " << mac_string(cur) << "\n";
        if (std::memcmp(cur, host, 6) == 0) {
            std::cout << "Already paired with this computer.\n";
            hid_close(dev);
            hid_exit();
            return 0;
        }
    }

    std::cout << "Setting Bluetooth master to " << mac_string(host) << "...\n";
    unsigned char msg[8] = {kMasterReport, 0x00, host[0], host[1], host[2], host[3], host[4], host[5]};
    int res = hid_send_feature_report(dev, msg, sizeof(msg));
    Logger::debug("hid_send_feature_report returned " + std::to_string(res));
    if (res < 0) {
        Logger::error("Writing the pairing data failed.");
        hid_close(dev);
        hid_exit();
        return 1;
    }

    unsigned char after[6];
    bool verified = read_master(after) && std::memcmp(after, host, 6) == 0;
    hid_close(dev);
    hid_exit();
    if (!verified) {
        Logger::error("The controller did not keep the new address (read back differs).");
        return 1;
    }

    std::cout << "Pairing data written and verified.\n\n"
                 "Unplug the cable and press the PS button. Once it connects over Bluetooth it appears as a\n"
                 "gamepad (\"Navigation Controller\") that SDL-based programs such as Dolphin can use.\n"
                 "In Dolphin, add it as an additional input source in the Wii Remote / Nunchuk profile.\n";
#if defined(__linux__)
    std::cout << "\nLinux: BlueZ's sixaxis plugin may already have registered the controller when you plugged it\n"
                 "in. If it still won't connect, try `bluetoothctl trust <address>` or run --linux-setup.\n"
                 "Needs root (or a udev rule) to open the USB device.\n";
#endif
    return 0;
}
