## ⚠️ Notice:
I am busy with school at the moment so I may not be able to provide updates on a normal cadence. I may pop in from time to time, but do not expect any sort of continuous support of this project for the time being.
However, if you wish to help contribute to this project, feel free to do so! I am looking for help, especially when it comes to Windows programming because I use Linux. Any contributions you make will be greatly appreciated.

# PSMove-DSU

A cross-platform DSU (DualShock UDP) / Cemuhook server for PlayStation Move controllers, enabling motion controls in games and emulators that support the DSU protocol.

## Overview

PSMove-DSU bridges PlayStation Move controllers to applications that support the DSU/Cemuhook protocol, such as:
- **Cemu** (Wii U emulator)
- **Dolphin** (GameCube/Wii emulator)
- **RPCS3** (PS3 emulator)
- **Yuzu** (Nintendo Switch emulator)
- **PCSX2** (PS2 emulator)
- Any application supporting motion controls via DSU

The server translates PS Move controller input (accelerometer, gyroscope, and buttons) into the standardized DSU protocol, allowing these controllers to provide motion control functionality in supported games.

## Features

- **Cross-platform support**: Windows, macOS, and Linux
- **Built-in GUI**: live controller input, button and motion-axis remapping, pairing, emulator connection status, and a one-click diagnostics report for bug reports
- **Real-time motion data**: Accelerometer and gyroscope
- **Button support**: All PS Move controller buttons
- **DSU/Cemuhook protocol**: Compatible with all DSU-enabled applications
- **Automatic controller detection**: Plug and play functionality; controllers that are turned off are released and picked up again, in the same slot, when they reconnect
- **Rumble**: supports the DSU rumble extension, for clients that send it (Dolphin, Cemu, Azahar and Eden currently don't)

## Quick Start

### Using Pre-built Binaries

1. **Download** the latest release for your platform:
   - [Windows](https://github.com/Swordmaster3214/PSMove-DSU/releases/latest/)
   - [macOS](https://github.com/Swordmaster3214/PSMove-DSU/releases/latest/)
   - [Linux](https://github.com/Swordmaster3214/PSMove-DSU/releases/latest/)

2. **Connect your PS Move controller** via Bluetooth

3. **Run the executable** (a window opens; add `--headless` to run in the console only):
   #### Windows
   ```bash
   ./dsu_server_psmove_windows_win64.exe
   ```
   #### macOS/Linux
   ```bash
   ./dsu_server_psmove_macos_x64
   ```
   ```bash
   ./dsu_server_psmove_linux_x64
   ```

5. **Configure your emulator** to connect to `127.0.0.1:26760` (default DSU port)

### The GUI

| Tab | What it's for |
| --- | --- |
| **Controllers** | Every connected controller with live buttons, trigger, gyro and accelerometer graphs, battery, report rate, data age and queued-report count. **Identify** lights up and rumbles a controller so you can tell which slot it's in. |
| **Button mapping** | Choose which emulator button each PS Move button sends, and see live what the emulator receives (digital bits and analog values). |
| **Motion axes** | Choose which PS Move sensor axis feeds the emulator's pitch/yaw/roll and accelerometer X/Y/Z, and invert any of them, with live bars to check the result. |
| **Emulators** | Every DSU client that has contacted the server, which controller slots it is receiving, and how many packets per second it gets. Shows a checklist when no emulator has connected. |
| **Pairing** | Pair a USB-connected controller with one click. |
| **Log** | The server log with filters. Turn on verbose logging without restarting. |

**Copy diagnostics** (top right) copies the current status and recent log in a format ready to paste into a GitHub issue. Please include it when reporting a bug.

Mapping changes take effect immediately and are saved to a settings file:
- Windows: `%APPDATA%\PSMove-DSU\config.ini`
- macOS: `~/Library/Application Support/PSMove-DSU/config.ini`
- Linux: `~/.config/psmove-dsu/config.ini`

The headless mode uses the same file, so you can set things up in the GUI and then run with `--headless`.

### Command-line options

| Option | Description |
| --- | --- |
| `--headless` | Run without the GUI (console only). Also used automatically when no display is available. |
| `--pair` | Pair a USB-connected controller from the console |
| `--verbose` | Detailed logs and latency diagnostics |
| `--config <path>` | Use a different settings file |
| `--map <p> <y> <r>` | Gyro source axis (0=X, 1=Y, 2=Z) for pitch, yaw and roll (overrides the settings file) |
| `--flip <p> <y> <r>` | Invert gyro pitch, yaw, roll (0/1) (overrides the settings file) |
| `--simulate <n>` | Add *n* virtual controllers, for testing an emulator setup without hardware |

### Button Mappings

Default mapping (change it on the GUI's **Button mapping** tab):

Button | Mapping
--- | ---
X | Cross
O | Circle
□ | Square
∆ | Triangle
Move | R1
SELECT | Share
START | Options
T | R2
PS | PS

### Controller Setup

#### Bluetooth Pairing
**Note:** You will not need to do the following if you have already paired your controller with your computer using [PSMoveAPI](https://github.com/thp/psmoveapi)

Connect the controller with a USB cable, open the **Pairing** tab and click **Pair**. Or, from the console:
```bash
./dsu_server_psmove --pair
```
Then unplug the cable and press the PS button. The red LED blinks, then stays lit once the controller is connected over Bluetooth. You only need to pair once per controller and computer; pair again if you connect it to a PS3 or another PC. Over USB, controllers can be paired, charged, and have LED/rumble set, but sensor data is only available over Bluetooth.

**Troubleshooting**
- **LED keeps blinking:** the controller isn't paired with this computer, or the battery is low. Charge it over USB. Your system's Bluetooth device list should show a "Motion Controller" entry.
- **Windows:** pairing may take several attempts. Use the Microsoft Bluetooth stack and remove third-party drivers such as MotionInJoy. May need to run as administrator.
- **macOS:** if a PIN dialog appears when you press PS, close it and pair again. Since macOS Sierra the built-in adapter supports at most 2 controllers at once; use an external adapter for more.
- **Linux:** pairing needs root (it registers the controller with BlueZ), so run `sudo ./dsu_server_psmove --pair` once. To use controllers without root, or if the controller pairs but won't connect (many distros' BlueZ rejects unbonded input devices), run `sudo ./dsu_server_psmove --linux-setup` (also offered at the end of `--pair` when run as root). It asks before installing the udev rules (built into the binary) and, separately, before setting `ClassicBondedOnly=false` in `/etc/bluetooth/input.conf`, which partially undoes a [Bluetooth security fix](https://github.com/advisories/GHSA-qjcj-xg77-6c32); it defaults to no and backs the file up first.

## Building from Source

### File structure
```
dsu_server_psmove/
├── CMakeLists.txt
├── README.md
├── src/
│   ├── main.cpp                    # Entry point and CLI parsing
│   ├── dsu/
│   │   ├── dsu_server.hpp         # DSU server interface
│   │   ├── dsu_server.cpp         # DSU server implementation
│   │   ├── dsu_protocol.hpp       # DSU protocol definitions
│   │   └── dsu_protocol.cpp       # DSU protocol implementation
│   ├── psmove/
│   │   ├── psmove_manager.hpp     # PSMove controller manager
│   │   ├── psmove_manager.cpp     # PSMove controller implementation
│   │   ├── psmove_pairing.hpp     # Pairing functionality
│   │   └── psmove_pairing.cpp     # Pairing implementation
│   ├── network/
│   │   ├── udp_server.hpp         # UDP networking interface
│   │   └── udp_server.cpp         # UDP networking implementation
│   ├── gui/
│   │   ├── gui_app.hpp/.cpp       # Window, fonts and render loop (Dear ImGui + GLFW)
│   │   └── gui_panels.hpp/.cpp    # The GUI tabs
│   ├── utils/
│   │   ├── logging.hpp            # Logging utilities
│   │   ├── logging.cpp            # Logging implementation
│   │   ├── platform.hpp           # Platform-specific definitions
│   │   ├── signal_handler.hpp     # Signal handling utilities
│   │   └── signal_handler.cpp     # Signal handling implementation
│   └── config/
│       ├── constants.hpp          # Global constants
│       ├── config.hpp             # Configuration management
│       ├── config_file.hpp/.cpp   # Settings file load/save
│       └── mapping.hpp/.cpp       # Button and axis mapping definitions
```
### Prerequisites

#### All Platforms
- **CMake** 3.18 or higher
- **C++ compiler** with C++17 support
- **Internet access on the first configure** - CMake downloads the PSMoveAPI 4.0.12, Dear ImGui 1.91.9b and GLFW 3.4 sources (pinned and hash-verified) and compiles them statically into the server. You do *not* need to install any of them yourself.

#### Linux
```bash
sudo apt-get install libbluetooth-dev libdbus-1-dev libudev-dev pkg-config \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev \
    libwayland-dev libxkbcommon-dev
```
The Wayland packages are optional; without them the GUI uses X11 (XWayland). For a console-only build without any of the GUI packages, configure with `-DDSU_BUILD_GUI=OFF`.

#### macOS
- Xcode command line tools (`xcode-select --install`)

#### Windows
- Visual Studio 2019 or higher (or Build Tools for Visual Studio)
- Windows SDK

### Build Instructions

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Swordmaster3214/PSMove-DSU.git && cd PSMove-DSU
   ```

2. **Create build directory**:
   ```bash
   mkdir build && cd build
   ```

3. **Configure with CMake**:
   #### Linux/macOS
   ```bash
   cmake -DCMAKE_BUILD_TYPE=Release ..
   ```
   #### Windows
   ```bash
   cmake ..
   ```

5. **Build the project**:
   ```bash
   cmake --build . --config Release
   ```

6. **Run the executable**:
   The executable will be in the build directory
   ```bash
   ./dsu_server_psmove
   ```

### Building Offline / Custom PSMoveAPI Sources

PSMoveAPI is built from source and linked statically, so end users of a release build do not need `psmoveapi.dll`, `libpsmoveapi.so`, or any other PSMoveAPI install. If you build without internet access, point CMake at local source trees instead of downloading them:

```bash
cmake -DFETCHCONTENT_SOURCE_DIR_PSMOVEAPI=/path/to/psmoveapi-4.0.12 \
      -DFETCHCONTENT_SOURCE_DIR_HIDAPI=/path/to/hidapi \
      -DFETCHCONTENT_SOURCE_DIR_IMGUI=/path/to/imgui-1.91.9b \
      -DFETCHCONTENT_SOURCE_DIR_GLFW=/path/to/glfw-3.4 ..
```

(`hidapi` here is the [thp/hidapi](https://github.com/thp/hidapi) fork at commit `38681c7`, the revision PSMoveAPI 4.0.12 pins as a submodule.)

On Windows with MSVC the C/C++ runtime is also linked statically (so the Visual C++ Redistributable is not required); pass `-DDSU_STATIC_RUNTIME=OFF` to opt out.
## Configuration

### DSU Server Settings
The server runs on `127.0.0.1:26760` by default, which is the standard DSU port. Most applications will automatically detect the server at this address.

### Emulator Setup

#### Cemu
1. Go to **Options** → **GamePad Motion Source** → **DSU1**
2. Set server IP to `127.0.0.1` and port to `26760`

#### Dolphin
1. Go to **Controllers** → **Configure** → **Motion Input**
2. Set source to **DSU** with server `127.0.0.1:26760`

#### Yuzu
1. Go to **Configure** → **Controls** → **Advanced**
2. Enable **Motion** and set to **DSU** with `127.0.0.1:26760`

## Troubleshooting

Start with the GUI: the status line at the top shows whether the server is listening, how many controllers are connected and whether any emulator is receiving data, and each tab explains what to check when something is missing. When opening an issue, click **Copy diagnostics** and paste the result.

### Controller Not Detected
- Ensure the PS Move controller is properly paired
- On Linux, you may need to run with `sudo` for USB access permissions
- Check that no other applications are using the controller

### Connection Issues
- Verify the server is running and listening on port 26760 (if another DSU server such as DS4Windows or BetterJoy already uses the port, PSMove-DSU now reports it instead of starting silently)
- The **Emulators** tab lists every emulator connection and which controller slots it receives. If it is empty, the emulator isn't reaching the server.
- Check firewall settings if connecting from another machine
- Ensure the emulator is configured with the correct IP and port

### Build Issues
- Make sure all dependencies are installed
- Verify CMake version is 3.18 or higher
- On Windows, ensure you're using the correct Visual Studio toolchain

### Linux Permissions
If you get permission errors accessing the controller:
Add udev rules for PS Move controller
```bash
sudo usermod -a -G plugdev $USER
```
Log out and back in for changes to take effect

## Technical Details

### Dependencies
- **PSMoveAPI 4.0.12**: Core PlayStation Move controller interface
- **System libraries**: Platform-specific USB and Bluetooth libraries
- **CMake**: Build system
- **C++17**: Modern C++ features for reliable performance

### Protocol Support
- **DSU Protocol Version 1001**: Full compatibility with Cemuhook standard
- **Motion Data**: 32-bit precision accelerometer and gyroscope
- **Button Data**: All PS Move buttons including trigger
- **Timestamp Synchronization**: Accurate timing for motion prediction

### Performance
- **Low latency**: Direct USB/Bluetooth communication
- **Multi-controller**: Support in the project is implemented for multiple controllers, but only one seems to be working 😅

## Contributing

Contributions are welcome! Please feel free to submit pull requests, report issues, or suggest improvements.

### Development Setup
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test on your target platform(s)
5. Submit a pull request

## Acknowledgments

- **Thomas Perl** and contributors to [PSMoveAPI](https://github.com/thp/psmoveapi)
- **Cemuhook developers** for the DSU protocol specification
- **Emulator developers** for DSU protocol support

## Support

- **Issues**: [GitHub Issues](https://github.com/Swordmaster3214/PSMove-DSU/issues)
- **PSMoveAPI Documentation**: [psmoveapi.readthedocs.io](https://psmoveapi.readthedocs.io/)
- **DSU Protocol**: [Cemuhook Documentation](https://cemuhook.sshnuke.net/)



## Licensing
License details for this project are in the LICENSE file.

The PS Move API library is licensed under the terms of the license below.
However, some optional third party libraries might have a different license.
Be sure to read the README file for details on third party licenses.

====

Copyright (c) 2011, 2012 Thomas Perl <m@thp.io>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
