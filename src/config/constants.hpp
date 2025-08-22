#pragma once
#include <cstdint>

namespace Constants {
    // DSU Protocol
    constexpr uint16_t PROTO_VER = 1001;
    constexpr int DSU_PORT = 26760;
    constexpr uint32_t MSG_PROTOCOL_VER = 0x100000;
    constexpr uint32_t MSG_INFO = 0x100001;
    constexpr uint32_t MSG_DATA = 0x100002;
    
    // Controller limits
    constexpr int MAX_CONTROLLERS = 4;
    
    // Timing
    constexpr int PSMOVE_POLL_MS = 4;    // ~250 Hz
    constexpr int DSU_SEND_MS = 8;       // ~125 Hz
    
    // Trigger settings
    constexpr int ANALOG_TRIGGER_INDEX = 14;
    constexpr unsigned char TRIGGER_DIGITAL_THRESHOLD = 10;
    
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
