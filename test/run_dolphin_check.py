#!/usr/bin/env python3
"""Checks the DSU server against a real Dolphin Emulator.

    python3 test/run_dolphin_check.py <server-exe> <dolphin-exe> [--seconds N]

Starts `<server-exe> --headless --simulate 2`, then Dolphin in batch mode with
a throwaway user folder: a one-instruction homebrew .dol, the server added as
a DSU server, and Wii Remote 1 and 2 mapped to the two simulated controllers.
After the run time it checks the server's log:

  * Dolphin subscribed to both simulated slots, and
  * no subscription timed out ("stopped renewing").

Dolphin needs a display (on Linux wrap this script in `xvfb-run -a`). It binds
the fixed DSU port 26760, so don't run alongside another server. Works the same
on Windows, Linux and macOS; Dolphin's and the server's logs are printed on
failure. Exit code is 0 on pass, 1 on failure, 2 on usage errors.
"""
import argparse
import os
import re
import signal
import struct
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CONSTANTS = os.path.join(HERE, "..", "src", "config", "constants.hpp")
SERVER_NAME = "PSMove-DSU"  # the name Dolphin shows after DSUClient/<slot>/


def simulated_slots(count):
    """Simulated controllers take the last slots (see PSMoveManager::is_simulated_slot)."""
    with open(CONSTANTS, encoding="utf-8") as f:
        m = re.search(r"MAX_CONTROLLERS\s*=\s*(\d+)", f.read())
    if not m:
        raise SystemExit("could not read MAX_CONTROLLERS from constants.hpp")
    total = int(m.group(1))
    return list(range(total - count, total))


def write_loop_dol(path):
    """A GameCube/Wii executable that is just `b .` (spins forever)."""
    header = bytearray(0x100)
    struct.pack_into(">I", header, 0x00, 0x100)       # text section 0: file offset
    struct.pack_into(">I", header, 0x48, 0x80003100)  # text section 0: load address
    struct.pack_into(">I", header, 0x90, 4)           # text section 0: size
    struct.pack_into(">I", header, 0xE0, 0x80003100)  # entry point
    with open(path, "wb") as f:
        f.write(bytes(header) + struct.pack(">I", 0x48000000))


def write_dolphin_config(user_dir, slots):
    config = os.path.join(user_dir, "Config")
    os.makedirs(config, exist_ok=True)

    def write(name, text):
        with open(os.path.join(config, name), "w", encoding="utf-8") as f:
            f.write(text)

    # No first-run prompts, no confirmation on stop.
    write("Dolphin.ini",
          "[Analytics]\nEnabled = False\nPermissionAsked = True\n"
          "[Interface]\nConfirmStop = False\nUsageStatisticsEnabled = False\n"
          "[Core]\nWiimoteContinuousScanning = False\n")
    write("DSUClient.ini",
          f"[Server]\nEnabled = True\nEntries = {SERVER_NAME}:127.0.0.1:26760;\n")
    # Source = 1 is "Emulated Wii Remote". Buttons/A just needs some binding.
    remotes = "".join(
        f"[Wiimote{i + 1}]\nSource = 1\nDevice = DSUClient/{slot}/{SERVER_NAME}\n"
        "Buttons/A = `Cross`\n"
        for i, slot in enumerate(slots))
    write("WiimoteNew.ini", remotes)
    # Log the controller interface so a failure shows what Dolphin saw.
    write("Logger.ini",
          "[Logs]\nCI = True\n[Options]\nVerbosity = 5\nWriteToFile = True\n"
          "WriteToConsole = False\nWriteToWindow = False\n")


def stop(proc, name, wait=5):
    if proc.poll() is not None:
        return
    if os.name == "nt":
        proc.terminate()
    else:
        proc.send_signal(signal.SIGINT if name == "server" else signal.SIGTERM)
    try:
        proc.wait(timeout=wait)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        if name == "server":
            print("server did not shut down within 5 s", file=sys.stderr)


def read(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("server")
    ap.add_argument("dolphin")
    ap.add_argument("--seconds", type=int, default=120,
                    help="how long Dolphin runs (subscriptions expire after 5 s)")
    ap.add_argument("--controllers", type=int, default=2, help="simulated controllers")
    args = ap.parse_args()

    slots = simulated_slots(args.controllers)
    problems = []

    with tempfile.TemporaryDirectory() as tmp:
        user_dir = os.path.join(tmp, "user")
        dol = os.path.join(tmp, "loop.dol")
        server_log = os.path.join(tmp, "server.log")
        dolphin_out = os.path.join(tmp, "dolphin.out")
        write_loop_dol(dol)
        write_dolphin_config(user_dir, slots)

        with open(server_log, "w") as slog, open(dolphin_out, "w") as dlog:
            server = subprocess.Popen(
                [args.server, "--headless", "--simulate", str(args.controllers),
                 "--config", os.path.join(tmp, "config.ini")],
                stdout=slog, stderr=subprocess.STDOUT)
            dolphin = None
            try:
                time.sleep(1)  # let the UDP port bind
                if server.poll() is not None:
                    print(f"server exited early with code {server.returncode}", file=sys.stderr)
                    problems.append("server did not start")
                else:
                    dolphin = subprocess.Popen(
                        [args.dolphin, "-u", user_dir, "-v", "Null", "-b", "-e", dol],
                        stdout=dlog, stderr=subprocess.STDOUT)
                    deadline = time.time() + args.seconds
                    while time.time() < deadline:
                        time.sleep(0.5)
                        if dolphin.poll() is not None:
                            problems.append(f"Dolphin exited early with code {dolphin.returncode}")
                            break
                        if server.poll() is not None:
                            problems.append("server died during the test")
                            break
            finally:
                if dolphin is not None:
                    stop(dolphin, "dolphin")
                stop(server, "server")

        log = read(server_log)
        subscribed = {int(s) for s in re.findall(r"subscribed to slot (\d+)", log)}
        for slot in slots:
            if slot not in subscribed:
                problems.append(f"Dolphin never subscribed to slot {slot}")
        for line in log.splitlines():
            if "stopped renewing" in line:
                problems.append(f"subscription timed out: {line.strip()}")

        print(f"simulated slots: {slots}; Dolphin subscribed to: {sorted(subscribed)}")
        if problems:
            print("\nFAILED:")
            for p in problems:
                print(f"  - {p}")
            print("\n--- server log ---\n" + log)
            print("--- Dolphin stdout/stderr ---\n" + read(dolphin_out))
            print("--- Dolphin log ---\n" + read(os.path.join(user_dir, "Logs", "dolphin.log")))
            print("--- Dolphin user folder ---")
            for root, _dirs, files in os.walk(user_dir):
                for name in files:
                    path = os.path.join(root, name)
                    print(f"  {os.path.relpath(path, user_dir)} ({os.path.getsize(path)} bytes)")
            return 1
    print(f"OK: Dolphin kept {len(slots)} subscriptions alive for {args.seconds} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
