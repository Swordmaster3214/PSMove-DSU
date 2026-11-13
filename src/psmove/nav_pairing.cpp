#include "psmove/nav_pairing.hpp"
#include "utils/logging.hpp"
#include <hidapi.h>
#include <psmove.h>
#include <psmove_port.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>

namespace {
constexpr unsigned short kSonyVid = 0x054c;
constexpr unsigned short kNavPid = 0x042f;
constexpr unsigned char kMasterReport = 0xF5;  // feature report holding the Bluetooth master address

// Report 0xF5 payload is 8 bytes: two header bytes, then the 6-byte master address. hidraw
// puts the report ID into the data stage as the first byte, so when writing, the ID takes
// the place of the first header byte (sixpair's 01 00 prefix must not be sent on top of it,
// or the address ends up shifted by one byte; verified on hardware).
constexpr int kMacOffset = 2;

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

bool read_master(hid_device* dev, unsigned char out[6]) {
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
}

const char* const kNextSteps =
    "Unplug the cable and press the PS button. Once it connects over Bluetooth it appears as a gamepad "
    "(\"Navigation Controller\") that SDL-based programs such as Dolphin can use. If it doesn't connect, "
    "turn Bluetooth off and on (or restart your computer) for the pairing to take effect, then press the "
    "PS button again.";

NavPairResult fail(const std::string& msg) { return {false, msg}; }

bool cancelled(const std::atomic<bool>* c) { return c && c->load(); }
}  // namespace

NavPairResult NavPairing::pair(int timeout_ms, const std::atomic<bool>* cancel) {
    char* host_str = psmove_port_get_host_bluetooth_address();
    unsigned char host[6];
    if (!host_str || !parse_mac(host_str, host)) {
        if (host_str) psmove_free_mem(host_str);
        return fail("Can't determine this computer's Bluetooth address. Make sure Bluetooth is turned on.");
    }
    psmove_free_mem(host_str);

    if (hid_init() != 0) return fail("Could not initialise the USB HID layer.");

    hid_device* dev = nullptr;
    bool found_but_unopenable = false;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!dev && !cancelled(cancel)) {
        hid_device_info* devs = hid_enumerate(kSonyVid, kNavPid);
        if (devs) {
            dev = hid_open_path(devs->path);
            if (!dev) found_but_unopenable = true;
            hid_free_enumeration(devs);
        }
        if (dev || std::chrono::steady_clock::now() >= deadline) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    if (!dev) {
        hid_exit();
        if (cancelled(cancel)) return fail("Cancelled.");
        if (found_but_unopenable) {
#if defined(__linux__)
            return fail("A Navigation controller was found on USB but could not be opened. Run once with "
                        "sudo ./<this program> --linux-setup to install the udev rules (then replug it), or "
                        "run this as root.");
#else
            return fail("A Navigation controller was found on USB but could not be opened.");
#endif
        }
        return fail("No Navigation controller found on USB. Plug it in with a USB cable (data-capable, not "
                    "charge-only); you may need to press its PS button.");
    }

    unsigned char cur[6];
    if (read_master(dev, cur)) {
        Logger::info("Navigation controller: current Bluetooth master " + mac_string(cur));
        if (std::memcmp(cur, host, 6) == 0) {
            hid_close(dev);
            hid_exit();
            return {true, std::string("The controller is already paired with this computer. ") + kNextSteps};
        }
    }

    unsigned char msg[8] = {kMasterReport, 0x00, host[0], host[1], host[2], host[3], host[4], host[5]};
    int res = hid_send_feature_report(dev, msg, sizeof(msg));
    Logger::debug("hid_send_feature_report returned " + std::to_string(res));
    if (res < 0) {
        hid_close(dev);
        hid_exit();
        return fail("Writing the pairing data to the controller failed.");
    }

    unsigned char after[6];
    bool verified = read_master(dev, after) && std::memcmp(after, host, 6) == 0;
    hid_close(dev);
    hid_exit();
    if (!verified) return fail("The controller did not keep the new address (read-back differs). Replug it and try again.");

    std::string msg_ok = "Paired with " + mac_string(host) + ". " + kNextSteps;
#if defined(__linux__)
    msg_ok += " Linux: if it still won't connect, try `bluetoothctl trust <address>` or run --linux-setup.";
#endif
    return {true, msg_ok};
}

int run_nav_pairing_cli() {
    std::cout << "\n=== Navigation Controller Pairing Mode ===\n"
                 "Connect the Navigation controller to your computer with a USB cable.\n"
                 "Press its PS button if it isn't detected. Detecting...\n";
    NavPairResult r = NavPairing::pair(30000);
    if (r.ok) {
        std::cout << "\n" << r.message << "\n";
        return 0;
    }
    Logger::error(r.message);
    return 1;
}

NavPairTask::~NavPairTask() {
    cancel_ = true;
    if (thread_.joinable()) thread_.join();
}

void NavPairTask::start() {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (status_.state == State::Running) return;
        status_.state = State::Running;
        status_.message = "Waiting for a Navigation controller on USB (up to 30 seconds)...";
    }
    if (thread_.joinable()) thread_.join();
    cancel_ = false;
    thread_ = std::thread([this] {
        NavPairResult r = NavPairing::pair(30000, &cancel_);
        std::lock_guard<std::mutex> lk(mutex_);
        status_.state = r.ok ? State::Succeeded : State::Failed;
        status_.message = r.message;
    });
}

NavPairTask::Status NavPairTask::status() {
    std::lock_guard<std::mutex> lk(mutex_);
    return status_;
}
