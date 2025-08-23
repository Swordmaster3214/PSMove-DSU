# Contributing to PSMove-DSU

Thank you for your interest in contributing to PSMove-DSU! This project provides a high-performance DSU (DualShock UDP) server for PlayStation Move controllers, optimized for low-latency motion tracking in VR and gaming applications.

## Quick Start

1. **Fork** the repository
2. **Clone** your fork locally
3. **Create a branch** for your feature/fix
4. **Make your changes**
5. **Test thoroughly**
6. **Submit a pull request**

## Development Setup

### Prerequisites

- **C++17** compatible compiler
- **CMake 3.10+**
- **PSMoveAPI 4.0.12** (automatically downloaded by build system)

### Platform-Specific Dependencies

#### Linux (Ubuntu/Debian)
```bash
sudo apt-get update
sudo apt-get install -y libusb-1.0-0-dev libbluetooth-dev pkg-config build-essential cmake
```

#### macOS
```bash
brew install libusb cmake
```

#### Windows
- **Visual Studio 2017+** or **Build Tools for Visual Studio**
- **CMake** (can be installed via Visual Studio Installer)

### Building from Source

```bash
# Clone the repository
git clone https://github.com/YOUR-USERNAME/PSMove-DSU.git
cd PSMove-DSU

# Build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

The executable will be created as:
- Linux: `dsu_server_psmove_linux_x64`
- macOS: `dsu_server_psmove_macos_x64` or `dsu_server_psmove_macos_arm64`
- Windows: `dsu_server_psmove_win64.exe`

## 📝 Contributing Guidelines

### Code Style

- **Modern C++17** features encouraged
- **RAII** for resource management
- **Consistent naming**: `snake_case` for variables/functions, `PascalCase` for classes
- **Clear comments** for complex algorithms, especially threading and timing code
- **Platform-specific code** should be clearly marked with `#ifdef` guards

### Example Code Style
```cpp
class PSMoveManager {
private:
    // Use descriptive names
    std::atomic<bool> running_{false};
    std::mutex controller_mutex_;
    
    // Clear function naming
    void integrate_pending_controllers();
    std::string get_serial_safe(PSMove* controller);
};
```

### Performance Considerations

This is a **real-time, low-latency** application. When contributing:

- **Avoid blocking operations** in the main polling thread
- **Minimize mutex usage** in hot paths (prefer atomics)
- **Profile timing-critical code** on Windows (most problematic platform)
- **Measure latency impact** of changes
- **Maintain 250Hz polling rate** (4ms intervals)

### Key Architecture Principles

1. **Background Operations**: Slow operations (controller scanning) run in background threads
2. **Lock-Free Data Access**: Main polling uses atomic operations where possible  
3. **Platform Optimization**: Windows, Linux, and macOS each have specific optimizations
4. **Double Buffering**: Sensor data uses double buffering to avoid blocking readers

## Bug Reports

### Before Reporting
- Check existing issues for duplicates
- Test with **verbose mode**: `./executable --verbose`
- Try on **different platforms** if possible

### Include in Bug Reports
- **Platform**: OS version, architecture (x64/ARM)
- **Controller Info**: Model, connection type (Bluetooth/USB)
- **Verbose Logs**: Run with `--verbose` flag
- **Latency Info**: Any high-latency warnings from logs
- **Steps to Reproduce**: Detailed reproduction steps

### Example Bug Report
```
**Platform**: Windows 11 x64
**Controller**: PS Move Navigation Controller via Bluetooth
**Issue**: Controller disconnects after 30 seconds

**Verbose Log Output:**
[ERROR] Controller in slot 0 became invalid, removing
[WARN] Windows: Flushed 47 buffered packets from slot 0

**Steps to Reproduce:**
1. Pair controller using --pair mode
2. Start DSU server
3. Controller connects successfully  
4. After ~30 seconds, controller disconnects
5. Must restart server to reconnect
```

## Feature Requests

### Good Feature Requests
- **Performance improvements** (latency reduction, CPU usage)
- **Platform support** (new OS versions, ARM64)
- **Controller compatibility** (new PlayStation Move variants)
- **Debugging tools** (better diagnostics, monitoring)
- **Configuration options** (new mapping modes, filters)

### Include in Feature Requests
- **Use case**: What problem does this solve?
- **Performance impact**: Will this affect real-time performance?
- **Platform considerations**: Does this work on all platforms?
- **Backward compatibility**: Will this break existing setups?

## Pull Request Process

### Before Submitting
1. **Test on multiple platforms** (at minimum your development platform)
2. **Run with verbose logging** to check for new warnings/errors
3. **Verify performance** hasn't regressed (check for new high-latency warnings)
4. **Update documentation** if changing user-facing behavior

### PR Description Template
```markdown
## Changes
- Brief description of what changed

## Testing
- [ ] Tested on Linux
- [ ] Tested on Windows  
- [ ] Tested on macOS
- [ ] No new latency warnings in verbose mode
- [ ] Backward compatibility maintained

## Performance Impact
- Describe any performance changes (positive or negative)
- Include before/after measurements if applicable

## Platform-Specific Notes
- Any platform-specific considerations or changes
```

### Review Process
1. **Automated builds** must pass on all platforms
2. **Code review** by maintainers
3. **Performance validation** for timing-critical changes
4. **Testing** on real hardware when possible

## 🎯 Priority Areas for Contributions

### High Priority
- **Windows latency optimization**: Windows HID layer improvements
- **Controller compatibility**: Support for new PlayStation Move variants
- **Memory optimization**: Reducing CPU usage and memory footprint
- **Cross-platform testing**: Ensuring consistent behavior across platforms

### Medium Priority  
- **Configuration system**: More flexible axis mapping and filtering
- **Diagnostic tools**: Better debugging and monitoring capabilities
- **Documentation**: Setup guides, troubleshooting, API documentation

### Low Priority
- **UI/GUI**: Currently command-line focused
- **Protocol extensions**: DSU protocol is well-established
- **Alternative controllers**: Focus is on PlayStation Move devices

## 📚 Useful Resources

- **PSMoveAPI Documentation**: https://github.com/thp/psmoveapi
- **DSU Protocol Spec**: https://github.com/Ryochan7/ds4windows/wiki/DSU-Protocol
- **Real-time Programming**: Understanding thread priorities and timing
- **HID Protocol**: USB/Bluetooth Human Interface Device standards

## 🔍 Testing

### Manual Testing Checklist
- [ ] Controller pairing works (`--pair` mode)
- [ ] Controller discovery and connection
- [ ] Data transmission to DSU clients (test with Cemu, Dolphin, etc.)
- [ ] Multiple controller support (up to 4)
- [ ] Controller disconnect/reconnect handling
- [ ] No memory leaks during extended operation

### Performance Testing
```bash
# Run with verbose logging to monitor performance
./dsu_server_psmove_linux_x64 --verbose

# Look for these in logs:
# - Processing latency < 1000µs (normal)
# - No "High end-to-end latency" warnings  
# - "Windows: Flushed" messages should be rare
```

## Questions?

- **General Questions**: Open a GitHub Discussion
- **Bug Reports**: Open a GitHub Issue
- **Feature Ideas**: Open a GitHub Issue with [FEATURE] tag
- **Code Questions**: Comment on relevant pull requests

## Recognition

Contributors will be acknowledged in:
- **Release notes** for significant contributions
- **README.md** contributor section
- **Git commit history** (please use descriptive commit messages)

Thank you for helping make PSMove-DSU better!
