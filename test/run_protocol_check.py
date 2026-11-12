#!/usr/bin/env python3
"""Runs dsu_protocol_check.py against a server it starts itself.

    python3 test/run_protocol_check.py <server-exe> [--quick]

Used by CTest. Starts `<server-exe> --headless --simulate 2` with a throwaway
config file, runs the protocol check, stops the server and returns the check's
exit code (or 1 if the server died early or would not shut down).
"""
import os
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def stop(proc):
    if proc.poll() is not None:
        return
    if os.name == "nt":
        proc.terminate()
    else:
        proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        print("server did not shut down within 5 s", file=sys.stderr)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    exe, extra = sys.argv[1], sys.argv[2:]
    with tempfile.TemporaryDirectory() as tmp:
        config = os.path.join(tmp, "config.ini")
        proc = subprocess.Popen([exe, "--headless", "--simulate", "2", "--config", config])
        try:
            time.sleep(1)  # let the UDP port bind
            if proc.poll() is not None:
                print(f"server exited early with code {proc.returncode}", file=sys.stderr)
                return 1
            return subprocess.call(
                [sys.executable, os.path.join(HERE, "dsu_protocol_check.py")] + extra)
        finally:
            stop(proc)


if __name__ == "__main__":
    sys.exit(main())
