#pragma once
#include <cstdint>
#include <psmove.h>

namespace Constants {
    // DSU Protocol
    constexpr uint16_t PROTO_VER = 1001;
    constexpr int DSU_PORT = 26760;
    constexpr uint32_t MSG_PROTOCOL_VER = 0x100000;
    constexpr uint32_t MSG_INFO = 0x100001;
    constexpr uint32_t MSG_DATA = 0x100002;
    // Unofficial rumble extension (also implemented by DS4Windows and BetterJoy).
    constexpr uint32_t MSG_MOTOR_INFO = 0x110001;  // How many motors a controller has
    constexpr uint32_t MSG_RUMBLE = 0x110002;      // Set a motor's strength
    // Stop rumbling if the client hasn't re-sent the strength for this long, so a
    // client that disconnects abruptly can't leave a controller vibrating.
    constexpr int RUMBLE_TIMEOUT_MS = 5000;

    // A DATA subscription expires if the client doesn't renew it within this time
    // (the DSU protocol suggests ~5 seconds; clients re-request roughly every second).
    constexpr int SUBSCRIPTION_TIMEOUT_MS = 5000;
    // Clients that have been silent this long are dropped from the client list.
    constexpr int CLIENT_FORGET_MS = 60000;

    // Controller limits
    constexpr int MAX_CONTROLLERS = 4;

    // Timing - OPTIMIZED FOR LOW LATENCY
    constexpr int PSMOVE_POLL_MS = 4;        // ~250 Hz - increased frequency
    constexpr int DSU_SEND_MS = 4;           // ~250 Hz - matched to polling rate
    constexpr int LATENCY_WARNING_US = 8000; // Warn if latency > 8ms

    // Upper bound on HID reports read from one controller per poll loop.
    // Reading until the queue is empty keeps input from lagging behind.
    constexpr int MAX_REPORTS_PER_POLL = 64;

    // A PS Move streams reports continuously (~60-100/s) even when idle. If one
    // stays silent this long it was turned off or went out of range: free its
    // slot so it's picked up again when it reconnects. (PSMoveAPI doesn't report
    // read errors, so silence is the only reliable signal on every platform.)
    constexpr int CONTROLLER_TIMEOUT_MS = 2000;

    // psmove_get_buttons() bitmasks, taken from PSMoveAPI's enum PSMove_Button
    // so they always match what the library reports.
    constexpr unsigned int Btn_TRIANGLE = ::Btn_TRIANGLE;
    constexpr unsigned int Btn_SQUARE = ::Btn_SQUARE;
    constexpr unsigned int Btn_CROSS = ::Btn_CROSS;
    constexpr unsigned int Btn_CIRCLE = ::Btn_CIRCLE;
    constexpr unsigned int Btn_MOVE = ::Btn_MOVE;
    constexpr unsigned int Btn_START = ::Btn_START;
    constexpr unsigned int Btn_SELECT = ::Btn_SELECT;
    constexpr unsigned int Btn_PS = ::Btn_PS;
}
