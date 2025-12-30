# Cemuhook Protocol Reference (DSU Protocol)

> **Protocol Version:** 1001 (only known version)
>
> Based on the official specification at https://v1993.github.io/cemuhook-protocol/

## Table of Contents

- [Introduction](#introduction)
- [Common Information](#common-information)
- [Packet Structure](#packet-structure)
  - [Header Structure](#header-structure)
  - [Message Types](#message-types)
- [Protocol Messages](#protocol-messages)
  - [Protocol Version Information](#protocol-version-information)
  - [Information About Connected Controllers](#information-about-connected-controllers)
  - [Actual Controllers Data](#actual-controllers-data)
- [Unofficial Extensions](#unofficial-extensions)
  - [Controller Motor Information](#controller-motor-information)
  - [Rumble Controller Motor](#rumble-controller-motor)

---

## Introduction

Cemuhook is a modification for the Cemu WiiU emulator that allows custom button and motion input sources. The Cemuhook protocol (also known as DSU Protocol - DualShock UDP) has become a standard for motion controller input across multiple emulators.

**Supported Emulators:**
- Cemu (WiiU)
- Dolphin (GameCube/Wii)
- RPCS3 (PS3)
- Yuzu (Nintendo Switch)
- PCSX2 (PS2)

---

## Common Information

### Transport Protocol
- **Protocol:** UDP (User Datagram Protocol)
- **Default Port:** `26760`
- **Default Host:** `localhost` (127.0.0.1)
- **Direction:** Bidirectional (client ↔ server)

### Key Concepts

1. **This application is the server** - it listens on port 26760
2. **The emulator is the client** - it connects to your server
3. **One server instance** should serve all available controllers (up to 4)
4. **No persistent connection** - UDP is stateless, use timeouts to detect disconnection

### Packet Structure Overview

Every valid packet contains:

1. **Header** (16 bytes) - Contains magic string, version, length, CRC, IDs
2. **Message Type** (4 bytes) - Identifies what kind of message this is
3. **Payload** (variable) - The actual data

### Number Encoding

**All numbers use little-endian format** (least significant byte first).

This is the native format on x86/x64 processors, but not on ARM or some other architectures.

---

## Packet Structure

### Header Structure

**Total Size:** 20 bytes (16-byte header + 4-byte message type)

| Offset | Length | Type           | Field | Description |
|--------|--------|----------------|-------|-------------|
| 0      | 4      | ASCII String   | Magic | `DSUS` (server/you) or `DSUC` (client/emulator) |
| 4      | 2      | uint16         | Protocol Version | Currently `1001` |
| 6      | 2      | uint16         | Packet Length | Length of packet **excluding** header (bytes after offset 16) |
| 8      | 4      | uint32         | CRC32 | CRC32 checksum of entire packet (with this field set to 0) |
| 12     | 4      | uint32         | Server/Client ID | Unique ID that stays the same during one session |
| 16     | 4      | uint32         | Message Type | See [Message Types](#message-types) below |

**CRC32 Calculation:**
1. Set bytes 8-11 to zero
2. Calculate CRC32 of the entire packet
3. Write the result into bytes 8-11

**Important:** All packet offsets described below are **relative to byte 20** (after the header).

---

### Message Types

| Value      | Direction | Description |
|------------|-----------|-------------|
| `0x100000` | Both      | Protocol version information |
| `0x100001` | Both      | Information about connected controllers |
| `0x100002` | Both      | Actual controller data (buttons, motion, etc.) |
| `0x110001` | Both      | **(Unofficial)** Information about controller motors |
| `0x110002` | Client→Server | **(Unofficial)** Rumble controller motor command |

> **Note:** Message types are the same for both incoming and outgoing packets. If you receive message type `0x100001`, your response should also use `0x100001`.

---

## Protocol Messages

### Protocol Version Information

**Message Type:** `0x100000`

**Purpose:** Exchange supported protocol versions (rarely used in practice)

#### Incoming (from client)

No payload - just the header.

#### Outgoing (to client)

| Offset | Length | Type   | Field | Description |
|--------|--------|--------|-------|-------------|
| 0      | 2      | uint16 | Max Protocol Version | Maximum protocol version your server supports (send `1001`) |

**Example Response:**
```
Bytes 0-19: Standard header with message type 0x100000
Bytes 20-21: 0xE9 0x03 (1001 in little-endian)
```

---

### Shared Response Header

The following two message types (`0x100001` and `0x100002`) both start with the same 11-byte structure:

| Offset | Length | Type   | Field | Description |
|--------|--------|--------|-------|-------------|
| 0      | 1      | uint8  | Slot | Controller slot (0-3) |
| 1      | 1      | uint8  | State | `0` = not connected, `1` = reserved, `2` = connected |
| 2      | 1      | uint8  | Model | `0` = N/A, `1` = no/partial gyro, `2` = full gyro |
| 3      | 1      | uint8  | Connection Type | `0` = N/A, `1` = USB, `2` = Bluetooth |
| 4      | 6      | uint48 | MAC Address | 48-bit MAC address (or zeros if N/A) |
| 10     | 1      | uint8  | Battery | Battery status (see table below) |

**Battery Status Values:**

| Value  | Meaning |
|--------|---------|
| `0x00` | Not applicable |
| `0x01` | Dying |
| `0x02` | Low |
| `0x03` | Medium |
| `0x04` | High |
| `0x05` | Full |
| `0xEE` | Charging |
| `0xEF` | Charged |

---

### Information About Connected Controllers

**Message Type:** `0x100001`

**Purpose:** Client queries which controllers are connected. Server responds with info about each requested slot.

#### Incoming (from client)

| Offset | Length | Type   | Field | Description |
|--------|--------|--------|-------|-------------|
| 0      | 4      | int32  | Port Count | Number of slots to report about (max 4) |
| 4      | 1-4    | uint8[] | Slots | Array of slot numbers to report (each 0-3) |

**Example:** Client might send `[4, 0, 1, 2, 3]` to request info about all 4 slots.

#### Outgoing (to client)

**Send one packet per requested slot.**

| Offset | Length | Type   | Field | Description |
|--------|--------|--------|-------|-------------|
| 0      | 11     | Complex | Shared Header | See [Shared Response Header](#shared-response-header) |
| 11     | 1      | uint8  | Padding | Always `0x00` |

**Total Payload:** 12 bytes

**If controller is not connected:** Send 12 zero bytes (plus packet header).

---

### Actual Controllers Data

**Message Type:** `0x100002`

**Purpose:** Client subscribes to continuous controller data updates. Server sends data packets repeatedly.

**Total Packet Size:** 100 bytes (including 20-byte header)

#### Incoming (from client)

Client subscribes to receive data from specific controller(s).

| Offset | Length | Type   | Field | Description |
|--------|--------|--------|-------|-------------|
| 0      | 1      | uint8  | Flags | `0x01` = slot-based, `0x02` = MAC-based, `0x00` = all controllers |
| 1      | 1      | uint8  | Slot | If slot-based, which slot (0-3) |
| 2      | 6      | uint48 | MAC | If MAC-based, which MAC address |

**Subscription Types:**
- **Slot-based:** Send data for specific slot number
- **MAC-based:** Send data for controller with specific MAC address
- **All controllers:** Send data for all connected controllers (flags = 0)

**Timeout Handling:**
Since UDP has no persistent connection, implement a timeout (~5 seconds). Stop sending data if no subscription renewal is received within the timeout period.

#### Outgoing (to client)

Send this packet repeatedly (recommended: 125-250 Hz) while client is subscribed.

**Important Notes:**
- All analog stick values use full 8-bit range (0-255), neutral = 128
- All analog buttons use full 8-bit range (0-255): 0 = released, 255 = fully pressed
- Accelerometer values are in g's (1g ≈ 9.8 m/s²)
- Gyroscope values are in degrees/second (not radians)
- Touch coordinates have no standard range - clients should implement calibration

##### Packet Layout

| Offset | Length | Type    | Field | Description |
|--------|--------|---------|-------|-------------|
| **Controller Info** ||||
| 0      | 11     | Complex | Shared Header | See [Shared Response Header](#shared-response-header) |
| 11     | 1      | uint8   | Connected | `0` = disconnected, `1` = connected |
| 12     | 4      | uint32  | Packet Number | Incrementing packet counter (per client) |
| **Digital Buttons** ||||
| 16     | 1      | Bitmask | Buttons 1 | D-Pad Left, D-Pad Down, D-Pad Right, D-Pad Up, Options, R3, L3, Share |
| 17     | 1      | Bitmask | Buttons 2 | Y, B, A, X, R1, L1, R2, L2 |
| 18     | 1      | uint8   | HOME | HOME/PS button (0 or 1) |
| 19     | 1      | uint8   | Touch Button | Touch/Click (0 or 1) |
| **Analog Sticks** ||||
| 20     | 1      | uint8   | Left Stick X | Horizontal (0=left, 128=center, 255=right) |
| 21     | 1      | uint8   | Left Stick Y | **Vertical (0=down, 128=center, 255=up)** ⚠️ |
| 22     | 1      | uint8   | Right Stick X | Horizontal (0=left, 128=center, 255=right) |
| 23     | 1      | uint8   | Right Stick Y | **Vertical (0=down, 128=center, 255=up)** ⚠️ |
| **Analog Buttons** ||||
| 24     | 1      | uint8   | Analog D-Pad Left | Pressure (0-255) |
| 25     | 1      | uint8   | Analog D-Pad Down | Pressure (0-255) |
| 26     | 1      | uint8   | Analog D-Pad Right | Pressure (0-255) |
| 27     | 1      | uint8   | Analog D-Pad Up | Pressure (0-255) |
| 28     | 1      | uint8   | Analog Y | Pressure (0-255) |
| 29     | 1      | uint8   | Analog B | Pressure (0-255) |
| 30     | 1      | uint8   | Analog A | Pressure (0-255) |
| 31     | 1      | uint8   | Analog X | Pressure (0-255) |
| 32     | 1      | uint8   | Analog R1 | Pressure (0-255) |
| 33     | 1      | uint8   | Analog L1 | Pressure (0-255) |
| 34     | 1      | uint8   | Analog R2 | Pressure (0-255) |
| 35     | 1      | uint8   | Analog L2 | Pressure (0-255) |
| **Touch Data** ||||
| 36     | 6      | Complex | Touch Point 1 | See [Touch Data Structure](#touch-data-structure) |
| 42     | 6      | Complex | Touch Point 2 | See [Touch Data Structure](#touch-data-structure) |
| **Motion Data** ||||
| 48     | 8      | uint64  | Timestamp | Microseconds since epoch (or arbitrary start) |
| 56     | 4      | float   | Accel X | Acceleration in g's |
| 60     | 4      | float   | Accel Y | Acceleration in g's |
| 64     | 4      | float   | Accel Z | Acceleration in g's |
| 68     | 4      | float   | Gyro Pitch | Rotation in deg/s |
| 72     | 4      | float   | Gyro Yaw | Rotation in deg/s |
| 76     | 4      | float   | Gyro Roll | Rotation in deg/s |

**Total Payload Size:** 80 bytes

##### Button Bitmasks

**Byte 16 (Buttons 1)** - bits in descending order:

| Bit | Button |
|-----|--------|
| 7   | D-Pad Left |
| 6   | D-Pad Down |
| 5   | D-Pad Right |
| 4   | D-Pad Up |
| 3   | Options/Start |
| 2   | R3 (Right Stick Click) |
| 1   | L3 (Left Stick Click) |
| 0   | Share/Select |

**Byte 17 (Buttons 2)** - bits in descending order:

| Bit | Button |
|-----|--------|
| 7   | Y / Triangle |
| 6   | B / Circle |
| 5   | A / Cross |
| 4   | X / Square |
| 3   | R1 / Right Bumper |
| 2   | L1 / Left Bumper |
| 1   | R2 / Right Trigger |
| 0   | L2 / Left Trigger |

##### Touch Data Structure

Each touch point is 6 bytes:

| Offset | Length | Type   | Field | Description |
|--------|--------|--------|-------|-------------|
| 0      | 1      | uint8  | Active | `0` = not touching, `1` = touching |
| 1      | 1      | uint8  | Touch ID | Unique ID for this continuous touch |
| 2      | 2      | uint16 | X Position | Horizontal position (rightward = positive) |
| 4      | 2      | uint16 | Y Position | Vertical position (downward = positive) |

**Touch ID:** Should remain the same for one continuous touch. Increment when finger lifts and touches again.

---

## Unofficial Extensions

These message types are not part of the original Cemuhook specification but are used by some implementations.

### Controller Motor Information

**Message Type:** `0x110001`

**Purpose:** Query how many rumble motors a controller has.

#### Incoming (from client)

| Offset | Length | Type   | Field | Description |
|--------|--------|--------|-------|-------------|
| 0      | 8      | Complex | Controller ID | Same as "Actual controllers data" subscription |

#### Outgoing (to client)

| Offset | Length | Type   | Field | Description |
|--------|--------|--------|-------|-------------|
| 0      | 11     | Complex | Shared Header | See [Shared Response Header](#shared-response-header) |
| 11     | 1      | uint8  | Motor Count | 0 = no rumble, 1 = single motor, 2 = dual motors |

---

### Rumble Controller Motor

**Message Type:** `0x110002`

**Purpose:** Set rumble intensity for controller motor.

**Direction:** Client → Server only (no response required)

#### Incoming (from client)

| Offset | Length | Type   | Field | Description |
|--------|--------|--------|-------|-------------|
| 0      | 8      | Complex | Controller ID | Same as "Actual controllers data" subscription |
| 8      | 1      | uint8  | Motor ID | 0 to `(motor_count - 1)` |
| 9      | 1      | uint8  | Intensity | 0 = off, 255 = maximum vibration |

**Important Implementation Notes:**

1. **Re-send periodically:** Clients should re-send rumble state 2-10 times per second to account for UDP packet loss
2. **Timeout handling:** Servers should reset rumble to zero if no rumble packet received for ~5 seconds (prevents stuck rumble if client crashes)
3. **Send zero values:** Clients should explicitly send intensity=0 when rumble stops (don't just stop sending)

---

## Implementation Tips

### For Contributors

**Dolphin Quirk:** Dolphin requires BOTH digital button bitmasks (bytes 16-18) AND analog button values (bytes 28-35) to be set correctly. Even if your controller has only digital buttons, you must set analog values to either 0 (released) or 255 (pressed).

**Note about controller button naming and mapping:** PlayStation controllers use symbolic face buttons (Cross, Circle, Square, Triangle) which do not directly map to the Nintendo/Xbox-style A/B/X/Y ordering that some DSU clients (e.g., Dolphin or Cemu) expect. A differing bitmask layout is used by the server so that PlayStation controllers are represented consistently to clients; do not change the bitmask ordering without confirming client expectations. Also ensure you set both the digital bitmasks and the corresponding analog values (bytes 28-35) when adding or modifying mappings.

**Subscription Management:**
```
- Store client address + subscription info when you receive 0x100002
- Send data packets continuously while subscribed
- Remove subscription after ~5 seconds of no packets from that client
- One client can subscribe to multiple controllers
```

**Packet Counter:**
- Keep separate counter per client
- Increment for each data packet sent to that client
- Wraps around at 2³² (4,294,967,296)

**CRC32:**
Use the standard CRC32 algorithm with polynomial `0xEDB88320` (reversed `0x04C11DB7`).

**Performance:**
- Target 125-250 Hz update rate (4-8ms between packets)
- Use UDP_NODELAY socket option if available
- Consider batching data collection from multiple controllers

---

## Example Packet Flows

### Initial Connection

```
1. Client → Server: 0x100001 (query controllers)
   [Port Count: 4, Slots: 0,1,2,3]

2. Server → Client: 0x100001 (slot 0 info)
   [Slot: 0, State: 2, Model: 2, ...]
   
3. Server → Client: 0x100001 (slot 1 info)
   [Slot: 1, State: 0, ...]
   
4. Server → Client: 0x100001 (slot 2 info)
   [Slot: 2, State: 0, ...]
   
5. Server → Client: 0x100001 (slot 3 info)
   [Slot: 3, State: 0, ...]
```

### Continuous Data

```
6. Client → Server: 0x100002 (subscribe to slot 0)
   [Flags: 0x01, Slot: 0]

7. Server → Client: 0x100002 (data)
   [Packet #0, buttons, motion, ...]

8. Server → Client: 0x100002 (data)
   [Packet #1, buttons, motion, ...]

9. Server → Client: 0x100002 (data)
   [Packet #2, buttons, motion, ...]
   
   (continues at 125-250 Hz...)
```

### With Rumble

```
10. Client → Server: 0x110002 (rumble)
    [Motor: 0, Intensity: 200]

11. Client → Server: 0x110002 (rumble - resend)
    [Motor: 0, Intensity: 200]
    
    (client resends every 100-500ms while rumbling)

12. Client → Server: 0x110002 (stop rumble)
    [Motor: 0, Intensity: 0]
```

---

## Troubleshooting

### Common Issues

**Buttons not working:**
- Check both bitmask (bytes 16-17) AND analog values (bytes 28-35)
- Verify packet offsets are correct
- Dolphin may be strictest (untested) - test there first

**Motion data erratic:**
- Ensure gyro is in deg/s (not rad/s)
- Check coordinate system handedness
- Verify float endianness

**Controller not discovered:**
- Verify server is listening on correct port
- Check firewall settings
- Ensure proper response to 0x100001 messages

**Packet loss:**
- UDP is unreliable - some packet loss is normal
- Implement rumble re-send mechanism
- Don't rely on every packet arriving

---

## Reference Implementations

**Official:**
- [cemuhook-protocol docs](https://v1993.github.io/cemuhook-protocol/) - Original specification
- [PSMoveService](https://github.com/psmoveservice/PSMoveService) - C++ implementation

**Community:**
- [ds4drv-cemuhook](https://github.com/TheDrHax/ds4drv-cemuhook) - Python
- [PSMove-DSU](https://github.com/Swordmaster3214/PSMove-DSU) - C++ for PS Move controllers
- [JoyShockMapper](https://github.com/JibbSmart/JoyShockMapper) - Multi-controller support

---

## License

This document is based on the [cemuhook-protocol specification](https://v1993.github.io/cemuhook-protocol/) by v1993.

The protocol itself is free to implement without licensing restrictions.