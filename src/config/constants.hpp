#pragma once
#include <cstdint>

namespace Constants {
    // DSU Protocol
    constexpr uint16_t PROTO_VER = 1001;
    constexpr int DSU_PORT = 26760;
    constexpr uint32_t MSG_PROTOCOL_VER = 0x100000;
    constexpr uint32_t MSG_INFO = 0x100001;
    constexpr uint32_t MSG_DATA = 0x100002;

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

    // PSMove button constants
    constexpr unsigned int Btn_TRIANGLE = 0x10;
    constexpr unsigned int Btn_SQUARE = 0x80;
    constexpr unsigned int Btn_CROSS = 0x40;
    constexpr unsigned int Btn_CIRCLE = 0x20;
    constexpr unsigned int Btn_MOVE = 0x08;
    constexpr unsigned int Btn_START = 0x01;
    constexpr unsigned int Btn_SELECT = 0x02;
    constexpr unsigned int Btn_PS = 0x04;
}
