#pragma once

// Optional Linux system setup (udev rules for hidraw access, BlueZ input.conf).
// Needs root, so it is only offered from the console (--pair / --linux-setup).
// On other platforms run_linux_setup() does nothing and returns 0.
int run_linux_setup();
