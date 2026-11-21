#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

// Pairs a PlayStation Move Navigation controller (a DualShock 3 variant, USB
// 054c:042f) with this computer by writing the host's Bluetooth address into it
// over USB. The controller's inputs are not handled here: once paired it shows
// up as an ordinary gamepad that SDL-based programs (e.g. Dolphin) can read.

class PSMoveManager;

struct NavPairResult {
    bool ok = false;
    std::string message;  // Human-readable outcome, including what to do next
};

namespace NavPairing {
// Waits up to timeout_ms for a Navigation controller on USB, then pairs it.
// Blocking. Stops early (with ok=false) once *cancel becomes true.
// Never calls hid_exit(): hidapi's state is shared with PSMoveAPI, which may be
// using it on other threads.
NavPairResult pair(int timeout_ms, const std::atomic<bool>* cancel = nullptr);
}

// Console mode (--pair-nav). Returns the process exit code.
int run_nav_pairing_cli();

// Runs NavPairing::pair() on a background thread, for the GUI. The manager's
// background scanner is paused meanwhile so the two don't use hidapi at once.
class NavPairTask {
public:
    enum class State { Idle, Running, Succeeded, Failed };
    struct Status {
        State state = State::Idle;
        std::string message;
    };

    explicit NavPairTask(PSMoveManager& manager) : manager_(manager) {}
    ~NavPairTask();
    void start();  // No-op while already running
    Status status();

private:
    PSMoveManager& manager_;
    std::mutex mutex_;
    Status status_;
    std::thread thread_;
    std::atomic<bool> cancel_{false};
};
