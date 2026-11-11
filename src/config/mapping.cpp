#include "config/mapping.hpp"
#include "config/constants.hpp"

namespace {
struct MoveInputInfo {
    const char* name;
    const char* key;
    unsigned int mask;
};

const MoveInputInfo MOVE_INPUTS[MOVE_INPUT_COUNT] = {
    {"Cross (X)",      "cross",    Constants::Btn_CROSS},
    {"Circle (O)",     "circle",   Constants::Btn_CIRCLE},
    {"Square",         "square",   Constants::Btn_SQUARE},
    {"Triangle",       "triangle", Constants::Btn_TRIANGLE},
    {"Move",           "move",     Constants::Btn_MOVE},
    {"Start",          "start",    Constants::Btn_START},
    {"Select",         "select",   Constants::Btn_SELECT},
    {"PS",             "ps",       Constants::Btn_PS},
    {"Trigger (T)",    "trigger",  0},
};

struct DsuButtonInfo {
    const char* name;
    const char* key;
};

const DsuButtonInfo DSU_BUTTONS[DSU_BUTTON_COUNT] = {
    {"(nothing)",     "none"},
    {"Cross",         "cross"},
    {"Circle",        "circle"},
    {"Square",        "square"},
    {"Triangle",      "triangle"},
    {"L1",            "l1"},
    {"R1",            "r1"},
    {"L2",            "l2"},
    {"R2",            "r2"},
    {"L3",            "l3"},
    {"R3",            "r3"},
    {"Share",         "share"},
    {"Options",       "options"},
    {"PS / Home",     "home"},
    {"Touchpad click","touch"},
    {"D-Pad Up",      "dpad_up"},
    {"D-Pad Down",    "dpad_down"},
    {"D-Pad Left",    "dpad_left"},
    {"D-Pad Right",   "dpad_right"},
};
} // namespace

namespace Mapping {

const char* move_input_name(MoveInput input) {
    auto i = static_cast<size_t>(input);
    return i < MOVE_INPUT_COUNT ? MOVE_INPUTS[i].name : "?";
}

const char* move_input_key(MoveInput input) {
    auto i = static_cast<size_t>(input);
    return i < MOVE_INPUT_COUNT ? MOVE_INPUTS[i].key : "";
}

unsigned int move_input_mask(MoveInput input) {
    auto i = static_cast<size_t>(input);
    return i < MOVE_INPUT_COUNT ? MOVE_INPUTS[i].mask : 0;
}

const char* dsu_button_name(DsuButton button) {
    auto i = static_cast<size_t>(button);
    return i < DSU_BUTTON_COUNT ? DSU_BUTTONS[i].name : "?";
}

const char* dsu_button_key(DsuButton button) {
    auto i = static_cast<size_t>(button);
    return i < DSU_BUTTON_COUNT ? DSU_BUTTONS[i].key : "";
}

bool dsu_button_from_key(const std::string& key, DsuButton& out) {
    for (size_t i = 0; i < DSU_BUTTON_COUNT; i++) {
        if (key == DSU_BUTTONS[i].key) {
            out = static_cast<DsuButton>(i);
            return true;
        }
    }
    return false;
}

const char* axis_name(int axis) {
    static const char* names[3] = {"X", "Y", "Z"};
    return (axis >= 0 && axis < 3) ? names[axis] : "?";
}

} // namespace Mapping
