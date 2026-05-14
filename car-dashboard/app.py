#!/usr/bin/env python3
"""
Car Dashboard Backend - CAN Bus Lab
=====================================
Flask + SocketIO server that:
  1. Reads all CAN frames from vcan0
  2. Decodes them using the ECU definitions
  3. Pushes decoded state to connected browsers via WebSocket
  4. Accepts REST/WS commands from the dashboard and sends CAN control frames
"""
import eventlet
eventlet.monkey_patch()

import can
import time
import threading
import logging
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import sys
import os


# Add ecu_definitions from the same package (kept alongside the app files)
sys.path.insert(0, '/app')
from ecu_definitions import *

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("DASHBOARD")

INTERFACE = "vcan0"
PING_INTERVAL_SEC = 10
PING_TIMEOUT_SEC = 30

app = Flask(__name__)
app.config['SECRET_KEY'] = 'can-lab-secret'
socketio = SocketIO(
    app,
    async_mode='eventlet',
    cors_allowed_origins='*',
    ping_interval=PING_INTERVAL_SEC,
    ping_timeout=PING_TIMEOUT_SEC,
)

# ─────────────────────────────────────────────
#  Shared vehicle state (decoded from CAN)
# ─────────────────────────────────────────────

vehicle = {
    "engine_on":       False,
    "rpm":             0,
    "speed_kmh":       0,
    "throttle_pct":    0,
    "load_pct":        0,
    "coolant_c":       20,
    "intake_c":        20,
    "gear":            0,
    "gear_mode":       "P",
    "fuel_pct":        0,
    "wheel_speeds":    {"fl": 0, "fr": 0, "rl": 0, "rr": 0},
    "doors": {
        "driver": False, "passenger": False,
        "rl": False,     "rr": False,
        "hood": False,   "trunk": False,
    },
    "lights": {
        "headlights": False, "taillights": False,
        "turn_left":  False, "turn_right": False,
        "fog_front":  False, "fog_rear":   False,
        "hazard":     False,
    },
    "seatbelt_driver": False,
    "seatbelt_pass":   False,
    "airbag_ok":       True,
    "tpms": {
        "fl": {"pressure_kpa": 220, "temp_c": 25},
        "fr": {"pressure_kpa": 220, "temp_c": 25},
        "rl": {"pressure_kpa": 220, "temp_c": 25},
        "rr": {"pressure_kpa": 220, "temp_c": 25},
    },
    "hvac": {
        "fan_speed":  0,
        "temp_set_c": 22,
        "ac_on":      False,
    },
    "central_locked": True,
    "gateway_ok":     False,
    "raw_frames":     [],   # last 50 raw CAN frames for the analyzer
}

vehicle_lock = threading.Lock()
can_bus = None
EMIT_HZ = 15
EMIT_INTERVAL_SEC = 1.0 / EMIT_HZ
last_emit_ts = 0.0

# ─────────────────────────────────────────────
#  Control frame IDs (must match ecu_simulator)
# ─────────────────────────────────────────────

CTRL_ENGINE_START  = 0x7F0
CTRL_ENGINE_STOP   = 0x7F1
CTRL_THROTTLE      = 0x7F2
CTRL_DOOR          = 0x7F3
CTRL_LIGHTS        = 0x7F4
CTRL_GEAR          = 0x7F5
CTRL_HVAC          = 0x7F6
CTRL_LOCKS         = 0x7F7
CTRL_SEATBELT      = 0x7F8

DOOR_MAP = {'driver':0, 'passenger':1, 'rl':2, 'rr':3, 'hood':4, 'trunk':5}
GEAR_MAP = {'P':0, 'R':1, 'N':2, 'D':3, 'S':4}


def send_can(can_id: int, data: bytes):
    global can_bus
    if can_bus is None:
        log.warning("CAN bus not connected, cannot send")
        return
    try:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        can_bus.send(msg)
        log.debug(f"→ TX 0x{can_id:03X}: {data.hex()}")
    except Exception as e:
        log.error(f"CAN TX error: {e}")


# ─────────────────────────────────────────────
#  CAN RX decoder loop
# ─────────────────────────────────────────────

TPMS_MAP = {
    ECU_IDS["TPMS_FL"]: "fl",
    ECU_IDS["TPMS_FR"]: "fr",
    ECU_IDS["TPMS_RL"]: "rl",
    ECU_IDS["TPMS_RR"]: "rr",
}

def can_rx_loop():
    global can_bus, vehicle, last_emit_ts
    retries = 15
    for i in range(retries):
        try:
            can_bus = can.interface.Bus(channel=INTERFACE, interface='socketcan')
            log.info(f"✅  Connected to {INTERFACE}")
            break
        except Exception as e:
            log.warning(f"⏳  Waiting for {INTERFACE}: {e} (attempt {i+1}/{retries})")
            socketio.sleep(2)
    else:
        log.error(f"Failed to connect to {INTERFACE}")
        return

    log.info("📡  CAN RX loop started")
    while True:
        msg = can_bus.recv(timeout=0.0)
        if msg is None:
            socketio.sleep(0.001) 
            continue

        aid = msg.arbitration_id
        d   = msg.data
        changed = False

        with vehicle_lock:
            # Record raw frame
            frame_str = f"0x{aid:03X}  [{len(d)}]  {' '.join(f'{b:02X}' for b in d)}"
            vehicle["raw_frames"].insert(0, frame_str)
            vehicle["raw_frames"] = vehicle["raw_frames"][:60]

            try:
                if aid == ECU_IDS["ECM_ENGINE_RPM_SPEED"]:
                    dec = decode_rpm_speed(d)
                    vehicle["rpm"]       = int(dec["rpm"])
                    vehicle["speed_kmh"] = int(dec["speed_kmh"])
                    vehicle["engine_on"] = vehicle["rpm"] > 200
                    changed = True

                elif aid == ECU_IDS["ECM_THROTTLE_LOAD"]:
                    dec = decode_throttle_load(d)
                    vehicle["throttle_pct"] = dec["throttle_pct"]
                    vehicle["load_pct"]     = dec["load_pct"]
                    changed = True

                elif aid == ECU_IDS["ECM_ENGINE_TEMP"]:
                    dec = decode_engine_temp(d)
                    vehicle["coolant_c"] = dec["coolant_c"]
                    vehicle["intake_c"]  = dec["intake_c"]
                    changed = True

                elif aid == ECU_IDS["ABS_WHEEL_SPEEDS"]:
                    dec = decode_wheel_speeds(d)
                    vehicle["wheel_speeds"] = {
                        "fl": round(dec["fl_kmh"], 1),
                        "fr": round(dec["fr_kmh"], 1),
                        "rl": round(dec["rl_kmh"], 1),
                        "rr": round(dec["rr_kmh"], 1),
                    }
                    changed = True

                elif aid == ECU_IDS["BCM_DOORS_WINDOWS"]:
                    vehicle["doors"] = decode_doors_windows(d)
                    changed = True

                elif aid == ECU_IDS["BCM_LIGHTS"]:
                    vehicle["lights"] = decode_lights(d)
                    changed = True

                elif aid == ECU_IDS["IC_FUEL_GAUGE"]:
                    vehicle["fuel_pct"] = decode_fuel_gauge(d)["fuel_pct"]
                    changed = True

                elif aid == ECU_IDS["TCU_GEAR_STATUS"]:
                    dec = decode_gear(d)
                    vehicle["gear"]      = dec["gear"]
                    vehicle["gear_mode"] = dec["mode"]
                    changed = True

                elif aid == ECU_IDS["SRS_STATUS"]:
                    dec = decode_srs(d)
                    vehicle["seatbelt_driver"] = dec["seatbelt_driver"]
                    vehicle["seatbelt_pass"]   = dec["seatbelt_pass"]
                    vehicle["airbag_ok"]       = dec["airbag_ok"]
                    changed = True

                elif aid in TPMS_MAP:
                    key = TPMS_MAP[aid]
                    vehicle["tpms"][key] = decode_tpms(d)
                    changed = True

                elif aid == ECU_IDS["HVAC_STATUS"]:
                    vehicle["hvac"] = decode_hvac(d)
                    changed = True

                elif aid == ECU_IDS["GATEWAY_HEARTBEAT"]:
                    vehicle["gateway_ok"] = True
                    changed = True

            except Exception as e:
                log.debug(f"Decode error for 0x{aid:03X}: {e}")

        if changed:
            now = time.monotonic()
            if now - last_emit_ts >= EMIT_INTERVAL_SEC:
                last_emit_ts = now
                socketio.emit('vehicle_update', dict(vehicle))


# ─────────────────────────────────────────────
#  REST API - Dashboard Actions
# ─────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/api/state')
def api_state():
    with vehicle_lock:
        return jsonify(dict(vehicle))

@app.route('/api/engine/start', methods=['POST'])
def engine_start():
    send_can(CTRL_ENGINE_START, bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    return jsonify({"ok": True, "action": "engine_start"})

@app.route('/api/engine/stop', methods=['POST'])
def engine_stop():
    send_can(CTRL_ENGINE_STOP, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    return jsonify({"ok": True, "action": "engine_stop"})

@app.route('/api/throttle', methods=['POST'])
def set_throttle():
    pct = int(request.json.get('pct', 0))
    pct = max(0, min(100, pct))
    send_can(CTRL_THROTTLE, bytes([pct, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    return jsonify({"ok": True, "throttle": pct})

@app.route('/api/door', methods=['POST'])
def set_door():
    door = request.json.get('door', 'driver')
    state_val = bool(request.json.get('open', False))
    idx = DOOR_MAP.get(door, 0)
    send_can(CTRL_DOOR, bytes([idx, int(state_val), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    return jsonify({"ok": True, "door": door, "open": state_val})

@app.route('/api/lights', methods=['POST'])
def set_lights():
    data = request.json
    flags = 0
    if data.get('headlights'): flags |= 0x01
    if data.get('taillights'): flags |= 0x02
    if data.get('turn_left'):  flags |= 0x04
    if data.get('turn_right'): flags |= 0x08
    if data.get('fog_front'):  flags |= 0x10
    if data.get('fog_rear'):   flags |= 0x20
    if data.get('hazard'):     flags |= 0x40
    send_can(CTRL_LIGHTS, bytes([flags, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    return jsonify({"ok": True, "flags": hex(flags)})

@app.route('/api/gear', methods=['POST'])
def set_gear():
    mode = request.json.get('mode', 'P')
    m = GEAR_MAP.get(mode, 3)
    send_can(CTRL_GEAR, bytes([m, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    return jsonify({"ok": True, "gear": mode})

@app.route('/api/hvac', methods=['POST'])
def set_hvac():
    data    = request.json
    fan     = int(data.get('fan_speed', 0))
    temp    = float(data.get('temp_set_c', 22))
    ac      = bool(data.get('ac_on', False))
    temp_r  = int((temp + 40) * 2) & 0xFF
    send_can(CTRL_HVAC, bytes([fan, temp_r, int(ac), 0x00, 0x00, 0x00, 0x00, 0x00]))
    return jsonify({"ok": True})

@app.route('/api/locks', methods=['POST'])
def set_locks():
    locked = bool(request.json.get('locked', True))
    send_can(CTRL_LOCKS, bytes([int(locked), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    return jsonify({"ok": True, "locked": locked})

@app.route('/api/seatbelt', methods=['POST'])
def set_seatbelt():
    drv  = bool(request.json.get('driver', False))
    pass_ = bool(request.json.get('passenger', False))
    send_can(CTRL_SEATBELT, bytes([int(drv), int(pass_), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    return jsonify({"ok": True})

@app.route('/api/frames')
def get_frames():
    with vehicle_lock:
        return jsonify(vehicle["raw_frames"])


# ─────────────────────────────────────────────
#  WebSocket Events
# ─────────────────────────────────────────────

@socketio.on('connect')
def on_connect():
    log.info(f"Dashboard client connected: {request.sid}")
    with vehicle_lock:
        emit('vehicle_update', dict(vehicle))

@socketio.on('disconnect')
def on_disconnect():
    log.info(f"Dashboard client disconnected: {request.sid}")


# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────

if __name__ == '__main__':
    log.info("=" * 50)
    log.info("  CAN Bus Lab - Car Dashboard")
    log.info("=" * 50)

    # rx_thread = threading.Thread(target=can_rx_loop, daemon=True, name="can-rx")
    # rx_thread.start()

    socketio.start_background_task(can_rx_loop)

    log.info("🌐  Starting web server on http://0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
