# CANSim Lab 🚗

A fully containerized CAN bus research environment with a realistic ECU simulator
and a live visual vehicle dashboard.

## Highlights

- SocketCAN-based ECU simulator with multiple virtual ECUs and realistic signals
- Live web dashboard with WebSocket updates and control actions
- Ready-to-use CAN ID reference and control frame map
- Dockerized services for quick spin-up and repeatable demos

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Linux HOST KERNEL                                                    │
│  ┌──────────┐         vcan0 (virtual CAN interface)             │
│  │ socketcan│◀══════════════════════════════════════════╗       │
│  └──────────┘                                           ║       │
└─────────────────────────────────────────────────────────║───────┘
                                                          ║
     ┌────────────────────────────┐      ┌────────────────║───────────────────┐
     │  ecu-simulator             │      │  car-dashboard  ║                   │
     │  (Python / python-can)     │      │  (Flask + WS)   ║                   │
     │                            │      │                 ║                   │
     │  ECM  0x0C9  RPM+Speed     │      │  CAN RX loop ───╝                   │
     │  ECM  0x0D0  Throttle      │      │  Decoder                           │
     │  ECM  0x0D1  Temp          │══════│  WebSocket ──► Browser Dashboard   │
     │  ABS  0x0B0  WheelSpeeds   │      │                                    │
     │  BCM  0x3B0  Doors         │      │  REST API ◄── Dashboard actions    │
     │  BCM  0x3B1  Lights        │      │  CAN TX  ──► vcan0 control frames  │
     │  TCU  0x1A1  Gear          │      │                                    │
     │  TPMS 0x2B0-3  Pressures   │      │  http://localhost:5000             │
     │  ...                       │      │                                    │
     │  Listens: 0x7F0-0x7F8      │      └────────────────────────────────────┘
     └────────────────────────────┘
```

---

## Prerequisites

- Linux host with SocketCAN support (vcan) or WSL2 on Windows
- Docker Engine with Compose v2 (Docker Desktop is fine)
- `can-utils` installed on the host for monitoring traffic (optional but recommended)

> Note: The virtual CAN interface `vcan0` must be created on the host OS. On Windows,
> run the host setup from WSL2. The containers will connect to `vcan0` on the host.

## Quick Start

### 1. Host setup (one-time)

```bash
# Install can-utils on host (for monitoring)
sudo apt install can-utils

# Create vcan0 interface
sudo bash scripts/setup-vcan.sh
```

### 2. Build and run

```bash
docker compose up --build
```

If you are using the legacy Compose v1 binary, use `docker-compose` instead.

### 3. Open dashboard

```
http://localhost:5000
```

### 4. Monitor raw traffic

```bash
# Live CAN dump on host
candump vcan0

# Colorized by ID
candump -c vcan0

# Log to file
candump -l vcan0
```

---

## CAN ID Reference

| CAN ID | ECU  | Signal             | Encoding                          |
|--------|------|--------------------|-----------------------------------|
| 0x0C9  | ECM  | RPM + Speed        | Bytes 0-1: RPM×4, Byte 2: km/h   |
| 0x0D0  | ECM  | Throttle + Load    | Byte 0: thr×2.55, Byte 1: load   |
| 0x0D1  | ECM  | Coolant + Intake   | Byte 0: temp+40, Byte 1: temp+40 |
| 0x0B0  | ABS  | Wheel speeds       | 4× (hi,lo) pairs, ×100 km/h      |
| 0x1A1  | TCU  | Gear + Mode        | Byte 0: gear#, Byte 1: P/R/N/D/S |
| 0x3B0  | BCM  | Doors/Windows      | Byte 0: bitmask (see below)       |
| 0x3B1  | BCM  | Lights             | Byte 0: bitmask (see below)       |
| 0x3A1  | IC   | Fuel gauge         | Byte 0: fuel%×2.55               |
| 0x0F0  | SRS  | Airbag+Seatbelts   | Byte 0: bits 0,1=belts, 7=OK     |
| 0x2B0  | TPMS | Front-left tire    | Bytes 0-1: kPa×100, Byte 2: temp |
| 0x2B1  | TPMS | Front-right tire   | same                              |
| 0x2B2  | TPMS | Rear-left tire     | same                              |
| 0x2B3  | TPMS | Rear-right tire    | same                              |
| 0x3C0  | HVAC | Fan/Temp/AC        | Byte 0: fan, 1: temp, 2: AC flag |
| 0x100  | GW   | Heartbeat          | Byte 0: 0xAA, Byte 1: counter    |

### Door bitmask (0x3B0 Byte 0)
```
bit 0 = driver door open
bit 1 = passenger door open
bit 2 = rear-left door open
bit 3 = rear-right door open
bit 4 = hood open
bit 5 = trunk open
```

### Light bitmask (0x3B1 Byte 0)
```
bit 0 = headlights
bit 1 = taillights
bit 2 = turn left
bit 3 = turn right
bit 4 = front fog
bit 5 = rear fog
bit 6 = hazard
```

---

## Control Frames (Dashboard → ECU Simulator)

| CAN ID | Function      | Data                              |
|--------|---------------|-----------------------------------|
| 0x7F0  | Engine START  | `01 00 00 00 00 00 00 00`         |
| 0x7F1  | Engine STOP   | `00 00 00 00 00 00 00 00`         |
| 0x7F2  | Throttle      | `[pct] 00 00 00 00 00 00 00`      |
| 0x7F3  | Door          | `[idx] [0=close/1=open] 00...`    |
| 0x7F4  | Lights        | `[flags] 00 00 00 00 00 00 00`    |
| 0x7F5  | Gear mode     | `[0=P,1=R,2=N,3=D,4=S] 00...`    |
| 0x7F6  | HVAC          | `[fan] [temp_raw] [ac] 00...`     |
| 0x7F7  | Central lock  | `[0=unlock/1=lock] 00...`         |
| 0x7F8  | Seatbelts     | `[driver] [pass] 00 00...`        |

---

## Fuzzing and Attack Examples

### Replay attack — force headlights on
```bash
while true; do cansend vcan0 3B1#01000000000000; sleep 0.1; done
```

### Spoof speed to 200 km/h at idle
```bash
# RPM=800 but speed byte=200
cansend vcan0 0C9#0C800C800000000
```

### Unlock doors via CAN injection
```bash
cansend vcan0 7F7#00000000000000
```

### Trigger fake engine start
```bash
cansend vcan0 7F0#01000000000000
```

### Log and replay a session
```bash
# Record
candump -l vcan0   # creates candump-TIMESTAMP.log

# Replay
canplayer -I candump-*.log
```

### Use can-utils for fuzzing
```bash
# Random fuzzing on a specific ID
cangen vcan0 -g 100 -I 3B0 -L 8
```

---

## File Structure

```
CANSim-Lab/
├── docker-compose.yml
├── scripts/
│   └── setup-vcan.sh          ← Run first on host
├── ecu-simulator/
│   ├── Dockerfile
│   ├── ecu_definitions.py     ← CAN ID map + encode/decode functions
│   └── ecu_simulator.py       ← Multi-ECU traffic generator + vehicle physics
└── car-dashboard/
    ├── Dockerfile
    ├── app.py                 ← Flask/SocketIO backend + CAN decoder
    └── templates/
        └── dashboard.html     ← Visual car interface
```

---

## Extending the Lab

- **Add new ECUs**: Define IDs in `ecu_definitions.py`, add TX thread in `ecu_simulator.py`, decode in `app.py`, display in `dashboard.html`
- **Add OBD-II support**: Respond to `0x7DF` PID requests in the simulator
- **CAN-FD**: Change `python-can` bus type to `socketcan` with `fd=True`
- **Hardware CAN**: Replace `vcan0` with `can0` (real USB-CAN adapter, e.g. Kvaser, PCAN, CANable)
- **IDS testing**: Log baseline traffic, then inject anomalies and detect deviations

## Safety and Ethics

This project is intended for research and education on isolated, virtual networks.
Do not use these techniques on real vehicles or networks without explicit permission.
