#pragma once
#include "config/mapping.hpp"
#include <array>
#include <string>

struct Config {
    // --- Launch options (command line only, never saved) ---
    bool verbose = false;
    bool pair_mode = false;
    bool headless = false;       // Run without the GUI
    int simulate = 0;            // Number of virtual test controllers
    std::string config_path;     // Empty = platform default

    // --- Runtime settings (editable in the GUI and saved to the config file) ---

    // Gyro axis mapping: which PS Move gyro axis (0=X, 1=Y, 2=Z) feeds each DSU axis
    int map_pitch = 0;
    int map_yaw = 2;
    int map_roll = 1;

    // Gyro axis flipping
    bool flip_pitch = false;
    bool flip_yaw = true;
    bool flip_roll = false;

    // Accelerometer mapping
    int map_accel_x = 0;
    int map_accel_y = 2;
    int map_accel_z = 1;

    // Accelerometer flipping
    bool flip_accel_x = true;
    bool flip_accel_y = true;
    bool flip_accel_z = false;

    // Button mapping: PS Move input -> DSU button, indexed by MoveInput
    std::array<DsuButton, MOVE_INPUT_COUNT> button_map = default_button_map();

    // Trigger value (0-255) above which T counts as a digital press
    int trigger_threshold = 10;

    static std::array<DsuButton, MOVE_INPUT_COUNT> default_button_map() {
        return {
            DsuButton::Cross,    // Cross
            DsuButton::Circle,   // Circle
            DsuButton::Square,   // Square
            DsuButton::Triangle, // Triangle
            DsuButton::R1,       // Move
            DsuButton::Options,  // Start
            DsuButton::Share,    // Select
            DsuButton::Home,     // PS
            DsuButton::R2,       // Trigger
        };
    }

    DsuButton map_for(MoveInput input) const {
        return button_map[static_cast<size_t>(input)];
    }

    // Copies only the runtime settings from another config.
    void copy_runtime_settings(const Config& o) {
        map_pitch = o.map_pitch; map_yaw = o.map_yaw; map_roll = o.map_roll;
        flip_pitch = o.flip_pitch; flip_yaw = o.flip_yaw; flip_roll = o.flip_roll;
        map_accel_x = o.map_accel_x; map_accel_y = o.map_accel_y; map_accel_z = o.map_accel_z;
        flip_accel_x = o.flip_accel_x; flip_accel_y = o.flip_accel_y; flip_accel_z = o.flip_accel_z;
        button_map = o.button_map;
        trigger_threshold = o.trigger_threshold;
    }
};
