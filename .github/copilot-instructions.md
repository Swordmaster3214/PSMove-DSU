# Copilot instructions for PSMove-DSU

Keep this file short and actionable. Focus on patterns and commands that help an AI agent be productive immediately.

## Big picture
- What this repo is: a low-latency DSU server that exposes PlayStation Move controllers over UDP (DSU protocol). It polls physical controllers using psmoveapi and multicasts DSU-formatted packets to subscribed clients.
- Major components:
  - `src/main.cpp` — CLI entry (flags: `--verbose`, `--pair`, `--map <p> <y> <r>`, `--flip <p> <y> <r>`).
  - `src/dsu/` — DSU implementation and server: `dsu_protocol.*` (packet format, CRC), `dsu_server.*` (subscription, send loop).
  - `src/psmove/` — controller handling: `psmove_manager.*` (double-buffered samples, background scanner, polling thread), `psmove_pairing.*` (pair mode).
  - `src/network/` — `udp_server.*` (non-blocking socket, `send_to_multiple` optimization).
  - `src/utils/` — logging and signal handling helpers.

## Key conventions & patterns (important for code changes)
- Real-time / low-latency focus: timing constants live in `src/config/constants.hpp` (e.g., `PSMOVE_POLL_MS`, `DSU_SEND_MS`, `LATENCY_WARNING_US`). Prefer tuning these constants rather than hardcoding sleeps.
- Lock-free data path for sensor samples: `PSMoveManager` uses double buffering + `std::atomic<MoveSample*> latest_atomic_[]` for readers (`get_latest_for_slot`) to avoid mutexes in the hot path. If you touch data flow, keep atomic semantics intact.
- Background scan for slow operations: controller discovery runs in `scanner_thread_main()` and queues new controllers to `pending_controllers_` for integration on the fast thread. Avoid scanning on the polling thread.
- Thread priorities: code sets thread scheduling (POSIX/Windows) for timing. Be careful when changing or removing these — tests may show latency regressions.
- Use `Logger::{info,debug,warn,error}` consistently. Verbose builds are enabled with `--verbose` and change logging and diagnostics frequency.
- Socket I/O: `UDPServer` uses `MSG_DONTWAIT` / non-blocking operations and a small receive loop with sleeps. Batch sends are via `send_to_multiple()` for all subscribers.
- Protocol details:
  - DSU magic is "DSUS" at packet start; CRC is little-endian and stored at bytes 8..11 (see `DSUProtocol::make_packet`).
  - Message types are defined in `Constants::MSG_*` and parsed in `DSUServer::handle_message`.
  - `DSUProtocol::make_data_packet` constructs payloads (shared begin, connected, packet_no, control bytes, sticks, touch, timestamp, accel, gyro).

## Build & run (exact steps)
- Requirements: `cmake` (>=3.10), compiler with C++17, `psmoveapi` (libusb/bluez on Linux). On Linux you also need libusb/bluez/udev dev packages.
- Configure build (example):
  - mkdir build && cd build
  - cmake -DPSMOVE_ROOT=/path/to/psmoveapi_install ..
  - make
- CMake sets an executable name with a platform suffix (printed on configure); executable is placed in the build directory.
- If psmoveapi isn't detected: set `-DPSMOVE_ROOT` to the directory with `include/psmove.h` and corresponding `lib/`.
- Running locally:
  - Pairing mode: `./<exe> --pair` (run pairing flow implemented in `psmove_pairing.cpp`).
  - Normal mode: `./<exe>` (recommended: run as root if you need realtime priority or raw device access).
  - Debug/verbose: `./<exe> --verbose` to see latency logging and diagnostics.

## Tests / local protocol validation
- A simple DSU test client exists in `test/dummy_dsu_sinewave.cpp` that implements the DSU protocol and prints debug info. Useful to validate server behavior without hardware:
  - compile: `g++ -std=c++17 -O0 -o test/dummy test/dummy_dsu_sinewave.cpp` (requires standard socket headers)
  - run: `./test/dummy` then start server and subscribe from a client.

## Common maintenance tasks and how to approach them
- Adding a new DSU message type:
  - Add constant in `src/config/constants.hpp`.
  - Implement handling in `DSUServer::handle_message`.
  - Add packet constructors in `DSUProtocol` as needed and ensure CRC bytes calculation remains correct.
- Modifying polling/send rates: prefer editing `PSMOVE_POLL_MS` / `DSU_SEND_MS` in `constants.hpp`. Verify latency behavior when running with `--verbose`.
- Changing controller discovery behavior: extend `scan_for_controllers_async()` and maintain the background scanner -> `pending_controllers_` -> `integrate_pending_controllers()` flow to avoid blocking the polling thread.
- When changing thread or socket behavior, check logging for latency warnings (`LATENCY_WARNING_US`) and test with `test/dummy_dsu_sinewave.cpp` to validate timing.

## Platform quirks
- Windows: uses `timeBeginPeriod(1)` to reduce timer resolution; builds set `WIN32` flags in CMake. Be aware of Windows-specific send buffer handling and Winsock error codes.
- Linux: uses `SO_PRIORITY` on UDP socket (non-fatal), sets non-blocking socket flags, and suggests running with elevated privileges for real-time scheduling.

## PR guidance for AI agents
- Keep changes small and testable: prefer incremental changes with a short, focused unit test (or run the dummy DSU client).
- Preserve non-blocking / atomic patterns in hot paths. If you add a lock, justify and add comments explaining why (performance regression risk).
- Use existing logging helpers (Logger) and add a `--verbose` diagnostic if helpful.

---
If anything here is unclear or you want additional examples (e.g., a short guide for adding unit-style tests or a checklist for performance changes), tell me which area to expand. Thanks! 💡🔧