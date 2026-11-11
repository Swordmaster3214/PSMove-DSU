#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

// Physical inputs on a PS Move controller that can be remapped.
enum class MoveInput : uint8_t {
    Cross,
    Circle,
    Square,
    Triangle,
    Move,
    Start,
    Select,
    PS,
    Trigger,   // T (analog)
    Count
};

// Buttons a DSU client (emulator) can receive.
enum class DsuButton : uint8_t {
    None,
    Cross,
    Circle,
    Square,
    Triangle,
    L1,
    R1,
    L2,
    R2,
    L3,
    R3,
    Share,
    Options,
    Home,
    Touch,
    DpadUp,
    DpadDown,
    DpadLeft,
    DpadRight,
    Count
};

constexpr size_t MOVE_INPUT_COUNT = static_cast<size_t>(MoveInput::Count);
constexpr size_t DSU_BUTTON_COUNT = static_cast<size_t>(DsuButton::Count);

namespace Mapping {
    // Display name, e.g. "Cross (X)".
    const char* move_input_name(MoveInput input);
    // Stable identifier used in the config file, e.g. "cross".
    const char* move_input_key(MoveInput input);
    // psmove_get_buttons() bitmask for the input (0 for the analog trigger).
    unsigned int move_input_mask(MoveInput input);

    // Display name, e.g. "R1".
    const char* dsu_button_name(DsuButton button);
    // Stable identifier used in the config file, e.g. "r1".
    const char* dsu_button_key(DsuButton button);
    // Parses a config-file identifier; returns false if unknown.
    bool dsu_button_from_key(const std::string& key, DsuButton& out);

    // Sensor axis names (index 0..2).
    const char* axis_name(int axis);
}
