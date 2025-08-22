#pragma once

struct Config {
    // Logging
    bool verbose = false;

    // Pairing
    bool pair_mode = false;
    
    // Axis mapping
    int map_pitch = 0;
    int map_yaw = 2;
    int map_roll = 1;
    
    // Axis flipping
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
};
