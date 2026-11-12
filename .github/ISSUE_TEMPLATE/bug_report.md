---
name: Bug report
about: Something in PSMove-DSU doesn't work as expected
title: "[BUG] "
labels: bug
assignees: ''

---

## What happened?
<!-- A clear description of the problem. -->

## What did you expect to happen?

## Steps to reproduce
1.
2.
3.

## Setup
- **OS**: <!-- e.g. Windows 11, Ubuntu 24.04, macOS 14 (Apple Silicon) -->
- **PSMove-DSU version**: <!-- shown at the top left of the window, e.g. v3.0.0 -->
- **Emulator and version**: <!-- e.g. Dolphin 2412, Cemu 2.4 -->
- **Emulator on the same computer?**: <!-- yes / no (if no, the server's and emulator's IP addresses) -->
- **Running with**: <!-- GUI (default) / --headless -->

## Diagnostics
<!--
Using the GUI (recommended):
  1. Optional, for more detail: open the Log tab and turn on "Verbose".
  2. Make the problem happen.
  3. Click "Copy diagnostics" at the top right and paste the result below.
     It includes your version, OS, controllers, emulator connections,
     button mapping and the recent log.

Using --headless: run with --verbose, make the problem happen, and paste the
log output below (in a ``` block), including some lines from before the problem.
-->

## What have you tried?
- [ ] The controller's red LED stays lit (not blinking) when it's connected
- [ ] The controller appears on the **Controllers** tab
- [ ] The emulator appears on the **Emulators** tab
- [ ] Only one DSU server is running (no DS4Windows, BetterJoy or second copy of PSMove-DSU using port 26760)
- [ ] Re-paired the controller (**Pairing** tab, or `--pair`)
- [ ] Linux: ran `sudo ./dsu_server_psmove_linux_x64 --linux-setup`
- [ ] Tried another emulator that supports DSU

## Anything else?
<!-- Screenshots, other Bluetooth devices or controller software running, recent system changes... -->
