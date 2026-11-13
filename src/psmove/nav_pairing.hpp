#pragma once

// Pairs a PlayStation Move Navigation controller (a DualShock 3 variant, USB
// 054c:042f) with this computer by writing the host's Bluetooth address into it
// over USB. The controller's inputs are not handled here: once paired it shows
// up as an ordinary gamepad that SDL-based programs (e.g. Dolphin) can read.
class NavPairing {
public:
    int run_pairing_mode();
};
