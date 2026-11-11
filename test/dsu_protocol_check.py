#!/usr/bin/env python3
"""Checks a running PSMove-DSU server speaks the DSU protocol correctly.

Start the server with two virtual controllers (they occupy slots 2 and 3):

    ./dsu_server_psmove_<platform> --headless --simulate 2

then run:

    python3 test/dsu_protocol_check.py [--quick]

--quick skips the ~6 s subscription-timeout check. Exit code 0 = all passed.
Standard library only, so it runs anywhere Python 3 does.
"""
import select
import socket
import struct
import sys
import time
import zlib

SERVER = ("127.0.0.1", 26760)
MSG_INFO = 0x100001
MSG_DATA = 0x100002
MSG_MOTOR_INFO = 0x110001
MSG_RUMBLE = 0x110002

failures = 0


def expect(cond, msg):
    global failures
    print(("PASS " if cond else "FAIL ") + msg, flush=True)
    if not cond:
        failures += 1


def packet(msg_type, payload=b""):
    body = struct.pack("<I", msg_type) + payload
    p = bytearray(b"DSUC" + struct.pack("<HHII", 1001, len(body), 0, 0xC0FFEE) + body)
    p[8:12] = struct.pack("<I", zlib.crc32(p) & 0xFFFFFFFF)
    return bytes(p)


def crc_ok(d):
    p = bytearray(d)
    crc = struct.unpack("<I", p[8:12])[0]
    p[8:12] = b"\0\0\0\0"
    return (zlib.crc32(p) & 0xFFFFFFFF) == crc


def new_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.setblocking(False)
    return s


def receive(s, seconds):
    out = []
    end = time.time() + seconds
    while True:
        remaining = end - time.time()
        if remaining <= 0:
            break
        ready, _, _ = select.select([s], [], [], min(0.05, remaining))
        if ready:
            while True:
                try:
                    out.append(s.recv(2048))
                except BlockingIOError:
                    break
    return out


def slots(packets):
    return sorted(set(d[20] for d in packets))


def subscribe(s, flags, slot=0, mac=b"\0" * 6):
    s.sendto(packet(MSG_DATA, bytes([flags, slot]) + mac), SERVER)


def main():
    quick = "--quick" in sys.argv

    # The server may still be starting; retry INFO for a few seconds.
    s = new_socket()
    replies = []
    for _ in range(20):
        s.sendto(packet(MSG_INFO, struct.pack("<i", 4) + bytes([0, 1, 2, 3])), SERVER)
        replies = [d for d in receive(s, 0.25) if struct.unpack("<I", d[16:20])[0] == MSG_INFO]
        if replies:
            break
    states = {d[20]: d[21] for d in replies}
    expect(len(replies) == 4 and states == {0: 0, 1: 0, 2: 2, 3: 2},
           f"INFO for all slots: {len(replies)} replies, states {states}")

    s.sendto(packet(MSG_INFO, struct.pack("<i", 1) + bytes([3])), SERVER)
    r = receive(s, 0.3)
    expect(len(r) == 1 and r[0][20] == 3, f"INFO for slot 3 only: {len(r)} reply(ies)")

    # Rumble extension: one motor per connected controller, none for empty slots.
    s.sendto(packet(MSG_MOTOR_INFO, bytes([0, 0]) + b"\0" * 6), SERVER)
    r = receive(s, 0.3)
    motors = {d[20]: d[31] for d in r
              if struct.unpack("<I", d[16:20])[0] == MSG_MOTOR_INFO and len(d) >= 32}
    expect(motors == {0: 0, 1: 0, 2: 1, 3: 1}, f"motor info for all slots: {motors}")

    s.sendto(packet(MSG_RUMBLE, bytes([1, 3]) + b"\0" * 6 + bytes([0, 200])), SERVER)  # slot 3, motor 0
    s.sendto(packet(MSG_RUMBLE, bytes([1, 9]) + b"\0" * 6 + bytes([0, 200])), SERVER)  # no such slot
    s.sendto(packet(MSG_RUMBLE, bytes([1, 3])), SERVER)                                # truncated
    s.sendto(packet(MSG_MOTOR_INFO, bytes([1, 3]) + b"\0" * 6), SERVER)
    r = [d for d in receive(s, 0.3) if struct.unpack("<I", d[16:20])[0] == MSG_MOTOR_INFO]
    expect(len(r) == 1 and r[0][20] == 3 and r[0][31] == 1,
           f"still answering after rumble requests ({len(r)} motor info reply(ies) for slot 3)")
    s.sendto(packet(MSG_RUMBLE, bytes([1, 3]) + b"\0" * 6 + bytes([0, 0])), SERVER)  # stop

    # Like Dolphin: one socket per controller, each registered by slot.
    by_slot2, by_slot3, everything, by_mac = new_socket(), new_socket(), new_socket(), new_socket()
    mac3 = struct.pack("<Q", 0x665544332211 + (3 << 8))[:6]
    subscribe(by_slot2, 1, 2)
    subscribe(by_slot3, 1, 3)
    subscribe(everything, 0)
    subscribe(by_mac, 2, 0, mac3)
    registered = time.time()

    r2 = receive(by_slot2, 1.0)
    r3, rall, rmac = receive(by_slot3, 0.01), receive(everything, 0.01), receive(by_mac, 0.01)
    expect(len(r2) > 100 and slots(r2) == [2], f"slot-2 subscriber: {len(r2)} packets, slots {slots(r2)}")
    expect(len(r3) > 100 and slots(r3) == [3], f"slot-3 subscriber: {len(r3)} packets, slots {slots(r3)}")
    expect(slots(rall) == [2, 3], f"'all' subscriber: slots {slots(rall)}")
    expect(len(rmac) > 100 and slots(rmac) == [3], f"MAC subscriber: {len(rmac)} packets, slots {slots(rmac)}")
    expect(bool(r2) and all(len(d) == 100 and crc_ok(d) for d in r2[:50]),
           "data packets are 100 bytes with a valid CRC")

    if not quick:
        print("Waiting for the subscription to expire (~6 s)...", flush=True)
        receive(by_slot2, 4.0 - (time.time() - registered))
        early = receive(by_slot2, 0.5)
        receive(by_slot2, 5.6 - (time.time() - registered))
        late = receive(by_slot2, 0.5)
        expect(len(early) > 50, f"still receiving 4.0-4.5 s after subscribing ({len(early)} packets)")
        expect(len(late) == 0, f"stopped after the 5 s timeout ({len(late)} packets at 5.6-6.1 s)")

    print("All checks passed." if failures == 0 else f"{failures} check(s) failed.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
