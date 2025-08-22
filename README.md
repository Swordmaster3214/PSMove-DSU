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
- **Real-time motion data**: Accelerometer and gyroscope
- **Button support**: All PS Move controller buttons
- **DSU/Cemuhook protocol**: Compatible with all DSU-enabled applications
- **Automatic controller detection**: Plug and play functionality

## Quick Start

### Using Pre-built Binaries

1. **Download** the latest release for your platform:
   - [Windows](https://github.com/Swordmaster3214/PSMove-DSU/releases/latest/)
   - [macOS](https://github.com/Swordmaster3214/PSMove-DSU/releases/latest/)
   - [Linux](https://github.com/Swordmaster3214/PSMove-DSU/releases/latest/)

2. **Connect your PS Move controller** via Bluetooth

3. **Run the executable**:
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

### Button Mappings

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
```bash
./dsu_server_psmove --pair
```

## Building from Source

### Prerequisites

#### All Platforms
- **CMake** 3.10 or higher
- **C++ compiler** with C++17 support
- **PSMoveAPI 4.0.12** [downloaded](https://github.com/thp/psmoveapi/releases/latest/)

#### Linux
```bash
sudo apt-get install libusb-1.0-0-dev libbluetooth-dev pkg-config
```

#### macOS
```bash
brew install libusb
```

#### Windows
- Visual Studio 2017 or higher
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
   cmake -DCMAKE_BUILD_TYPE=Release -DPSMOVE_ROOT=/path/to/psmoveapi ..
   ```
   #### Windows
   ```bash
   cmake -DPSMOVE_ROOT=/path/to/psmoveapi ..
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

### Custom PSMoveAPI Location

If you have PSMoveAPI installed in a custom location:

```bash
cmake -DPSMOVE_ROOT=/path/to/psmoveapi ..
```
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

### Controller Not Detected
- Ensure the PS Move controller is properly paired
- On Linux, you may need to run with `sudo` for USB access permissions
- Check that no other applications are using the controller

### Connection Issues
- Verify the server is running and listening on port 26760
- Check firewall settings if connecting from another machine
- Ensure the emulator is configured with the correct IP and port

### Build Issues
- Make sure all dependencies are installed
- Verify CMake version is 3.10 or higher
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
