#!/usr/bin/env python3
"""
ECU Simulator - CAN Bus Lab
============================
Simulates a complete vehicle's ECU network on vcan0.
Generates realistic CAN traffic with proper timing and data patterns.

Listens for incoming control frames from the dashboard container
and responds accordingly (engine start/stop, door commands, etc.)
"""

import can
import time
import math
import random
import threading
import logging
import struct
from ecu_definitions import *

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("ECU-SIM")

INTERFACE = "vcan0"

# ─────────────────────────────────────────────
#  Vehicle State
# ─────────────────────────────────────────────

class VehicleState:
    def __init__(self):
        # Engine
        self.engine_running  = False
        self.ignition_on     = False
        self.rpm             = 0
        self.target_speed    = 0.0    # km/h
        self.speed           = 0.0    # km/h
        self.throttle        = 0.0    # 0-100%
        self.coolant_temp    = 20     # °C (ambient)
        self.intake_temp     = 20     # °C
        self.engine_load     = 0.0   # 0-100%

        # Transmission
        self.gear            = 0      # 0=N, 1-8
        self.gear_mode       = 'P'    # P/R/N/D/S

        # Wheels / brakes
        self.wheel_speeds    = [0.0, 0.0, 0.0, 0.0]  # FL,FR,RL,RR
        self.braking         = False
        self.abs_active      = False

        # Body
        self.doors           = {
            'driver': False, 'passenger': False,
            'rl': False,     'rr': False,
            'hood': False,   'trunk': False,
        }
        self.headlights      = False
        self.taillights      = False
        self.turn_left       = False
        self.turn_right      = False
        self.hazard          = False
        self.fog_front       = False
        self.fog_rear        = False
        self.central_locked  = True

        # Safety
        self.seatbelt_driver = False
        self.seatbelt_pass   = False
        self.airbag_ok       = True

        # HVAC
        self.fan_speed       = 0
        self.hvac_temp_set   = 22.0
        self.ac_on           = False
        self.interior_temp   = 22.0
        self.exterior_temp   = 18.0

        # TPMS (pressure in kPa, nominal ~220 kPa = ~32 PSI)
        self.tire_pressure   = [221.0, 220.5, 219.8, 222.1]
        self.tire_temp       = [25, 25, 25, 26]

        # Fuel / battery
        self.fuel_pct        = 78.0
        self.odometer        = 48230   # km
        self.trip_meter      = 0

        # Simulation tick
        self.tick            = 0
        self._lock           = threading.Lock()

    def update_physics(self, dt=0.02):
        """Update vehicle physics simulation every 20ms."""
        with self._lock:
            # ── Engine warmup ──
            if self.engine_running:
                if self.coolant_temp < 90:
                    self.coolant_temp += 0.05 * dt * 50  # warms up over ~90s
                # Idle RPM when engine running
                if self.gear_mode in ('P', 'N') or self.speed < 2:
                    idle = 750 + random.randint(-20, 20)
                    self.rpm = idle if self.throttle < 2 else self._calc_rpm()
                else:
                    self.rpm = self._calc_rpm()
            else:
                # Engine spin-down
                if self.rpm > 0:
                    self.rpm = max(0, self.rpm - 500 * dt)
                if self.coolant_temp > 20:
                    self.coolant_temp -= 0.01 * dt * 50

            # ── Speed simulation ──
            if self.engine_running and self.gear_mode == 'D':
                # Accelerate toward target
                delta = self.target_speed - self.speed
                accel = min(abs(delta), 15.0 * dt * self.throttle / 100.0 + 2.0 * dt)
                if delta > 0:
                    self.speed = min(self.target_speed, self.speed + accel)
                elif delta < 0:
                    self.speed = max(self.target_speed, self.speed - accel * 1.5)
            else:
                # Decelerate if not driving
                if self.speed > 0:
                    self.speed = max(0, self.speed - 8.0 * dt)

            # ── Gear auto-selection ──
            if self.gear_mode == 'D' and self.engine_running:
                spd = self.speed
                if   spd < 15:  self.gear = 1
                elif spd < 30:  self.gear = 2
                elif spd < 50:  self.gear = 3
                elif spd < 80:  self.gear = 4
                elif spd < 110: self.gear = 5
                elif spd < 140: self.gear = 6
                else:           self.gear = 7
            elif self.gear_mode == 'R':
                self.gear = 0xFF  # reverse indicator
            else:
                self.gear = 0

            # ── Wheel speeds (slight variation for realism) ──
            base = self.speed
            for i in range(4):
                drift = random.uniform(-0.3, 0.3)
                self.wheel_speeds[i] = max(0, base + drift)

            # ── Engine load ──
            if self.engine_running:
                self.engine_load = self.throttle * 0.7 + (self.speed / 200.0) * 30
                self.engine_load = min(100, self.engine_load)

            # ── Tire temps increase with speed ──
            for i in range(4):
                target_t = 25 + self.speed * 0.4
                self.tire_temp[i] += (target_t - self.tire_temp[i]) * 0.001

            # ── Fuel consumption ──
            if self.engine_running and self.speed > 0:
                consumption = (0.005 + self.engine_load * 0.0002) * dt
                self.fuel_pct = max(0, self.fuel_pct - consumption)

            # ── Odometer ──
            self.odometer += self.speed * dt / 3600.0
            self.trip_meter += self.speed * dt / 3600.0

            self.tick += 1

    def _calc_rpm(self):
        """Calculate RPM from speed + gear."""
        if self.gear == 0 or self.speed < 1:
            return 850 + random.randint(-15, 15)
        # Approximate gear ratios
        ratios = [0, 3.5, 2.1, 1.4, 1.0, 0.75, 0.62, 0.55]
        ratio = ratios[min(self.gear, 7)]
        # Wheel circumference ~2m, diff ratio ~3.5, 60s/min
        rpm = (self.speed / 3.6) / (2.0 * math.pi * 0.32) * ratio * 3.5 * 60
        return max(700, min(7500, int(rpm) + random.randint(-30, 30)))

    def start_engine(self):
        with self._lock:
            if not self.engine_running:
                log.info("🔑  Engine START")
                self.ignition_on = True
                self.engine_running = True
                self.gear_mode = 'P'
                self.rpm = 1200  # crank

    def stop_engine(self):
        with self._lock:
            log.info("🔑  Engine STOP")
            self.engine_running = False
            self.ignition_on = False
            self.target_speed = 0

    def set_throttle(self, pct):
        with self._lock:
            self.throttle = max(0, min(100, pct))
            if self.engine_running and self.gear_mode == 'D':
                self.target_speed = pct * 2.5  # 100% throttle → ~250 km/h cap


# ─────────────────────────────────────────────
#  CAN Bus Interface
# ─────────────────────────────────────────────

class CANInterface:
    def __init__(self, interface: str):
        self.interface = interface
        self.bus = None
        self._connect()

    def _connect(self):
        retries = 10
        for i in range(retries):
            try:
                self.bus = can.interface.Bus(
                    channel=self.interface,
                    interface='socketcan'
                )
                log.info(f"✅  Connected to {self.interface}")
                return
            except Exception as e:
                log.warning(f"⏳  Waiting for {self.interface}: {e} (attempt {i+1}/{retries})")
                time.sleep(2)
        raise RuntimeError(f"Cannot open {self.interface}")

    def send(self, can_id: int, data: bytes):
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
        except can.CanError as e:
            log.error(f"CAN send error: {e}")

    def recv(self, timeout=0.01):
        return self.bus.recv(timeout=timeout)


# ─────────────────────────────────────────────
#  Control Frame Parser (from dashboard)
# ─────────────────────────────────────────────

# Dashboard sends control frames on reserved IDs
CTRL_ENGINE_START  = 0x7F0
CTRL_ENGINE_STOP   = 0x7F1
CTRL_THROTTLE      = 0x7F2  # data[0] = throttle %
CTRL_DOOR          = 0x7F3  # data[0]=door idx, data[1]=0/1
CTRL_LIGHTS        = 0x7F4  # data[0]=light flags
CTRL_GEAR          = 0x7F5  # data[0]=mode byte
CTRL_HVAC          = 0x7F6  # data[0]=fan, data[1]=temp, data[2]=ac
CTRL_LOCKS         = 0x7F7  # data[0]=0 unlock/1 lock
CTRL_SEATBELT      = 0x7F8  # data[0]=driver, data[1]=pass

DOOR_INDICES = ['driver', 'passenger', 'rl', 'rr', 'hood', 'trunk']
GEAR_MODES   = {0:'P', 1:'R', 2:'N', 3:'D', 4:'S'}

def handle_control_frame(msg: can.Message, state: VehicleState):
    aid = msg.arbitration_id
    d   = msg.data

    if aid == CTRL_ENGINE_START:
        state.start_engine()
    elif aid == CTRL_ENGINE_STOP:
        state.stop_engine()
    elif aid == CTRL_THROTTLE and len(d) >= 1:
        pct = d[0]
        state.set_throttle(pct)
        log.debug(f"Throttle → {pct}%")
    elif aid == CTRL_DOOR and len(d) >= 2:
        idx = d[0]
        val = bool(d[1])
        if 0 <= idx < len(DOOR_INDICES):
            key = DOOR_INDICES[idx]
            with state._lock:
                state.doors[key] = val
            log.info(f"Door {key} → {'OPEN' if val else 'CLOSED'}")
    elif aid == CTRL_LIGHTS and len(d) >= 1:
        f = d[0]
        with state._lock:
            state.headlights  = bool(f & 0x01)
            state.taillights  = bool(f & 0x02)
            state.turn_left   = bool(f & 0x04)
            state.turn_right  = bool(f & 0x08)
            state.fog_front   = bool(f & 0x10)
            state.fog_rear    = bool(f & 0x20)
            state.hazard      = bool(f & 0x40)
    elif aid == CTRL_GEAR and len(d) >= 1:
        mode = GEAR_MODES.get(d[0], 'D')
        with state._lock:
            state.gear_mode = mode
            if mode == 'P': state.target_speed = 0
        log.info(f"Gear → {mode}")
    elif aid == CTRL_HVAC and len(d) >= 3:
        with state._lock:
            state.fan_speed    = d[0]
            state.hvac_temp_set = (d[1] / 2.0) - 40
            state.ac_on        = bool(d[2])
    elif aid == CTRL_LOCKS and len(d) >= 1:
        with state._lock:
            state.central_locked = bool(d[0])
        log.info(f"Central lock → {'LOCKED' if state.central_locked else 'UNLOCKED'}")
    elif aid == CTRL_SEATBELT and len(d) >= 2:
        with state._lock:
            state.seatbelt_driver = bool(d[0])
            state.seatbelt_pass   = bool(d[1])


# ─────────────────────────────────────────────
#  Transmit Threads
# ─────────────────────────────────────────────

def tx_engine_loop(bus: CANInterface, state: VehicleState):
    """10ms cycle - high frequency engine frames"""
    while True:
        s = state
        bus.send(ECU_IDS["ECM_ENGINE_RPM_SPEED"],
                 encode_rpm_speed(int(s.rpm), int(s.speed)))
        bus.send(ECU_IDS["ECM_THROTTLE_LOAD"],
                 encode_throttle_load(s.throttle, s.engine_load))
        time.sleep(0.010)

def tx_engine_temps_loop(bus: CANInterface, state: VehicleState):
    """100ms cycle - temperature frames"""
    while True:
        bus.send(ECU_IDS["ECM_ENGINE_TEMP"],
                 encode_engine_temp(int(state.coolant_temp), int(state.intake_temp)))
        time.sleep(0.100)

def tx_abs_loop(bus: CANInterface, state: VehicleState):
    """10ms cycle - wheel speeds"""
    while True:
        ws = state.wheel_speeds
        bus.send(ECU_IDS["ABS_WHEEL_SPEEDS"],
                 encode_wheel_speeds(*ws))
        time.sleep(0.010)

def tx_body_loop(bus: CANInterface, state: VehicleState):
    """100ms cycle - body status"""
    while True:
        d = state.doors
        bus.send(ECU_IDS["BCM_DOORS_WINDOWS"],
                 encode_doors_windows(d['driver'], d['passenger'],
                                      d['rl'], d['rr'],
                                      d['hood'], d['trunk']))
        bus.send(ECU_IDS["BCM_LIGHTS"],
                 encode_lights(state.headlights, state.taillights,
                               state.turn_left, state.turn_right,
                               state.fog_front, state.fog_rear, state.hazard))
        time.sleep(0.100)

def tx_ic_loop(bus: CANInterface, state: VehicleState):
    """200ms cycle - instrument cluster"""
    while True:
        bus.send(ECU_IDS["IC_FUEL_GAUGE"],
                 encode_fuel_gauge(state.fuel_pct))
        bus.send(ECU_IDS["TCU_GEAR_STATUS"],
                 encode_gear(state.gear, state.gear_mode))
        bus.send(ECU_IDS["SRS_STATUS"],
                 encode_srs(state.seatbelt_driver, state.seatbelt_pass, state.airbag_ok))
        time.sleep(0.200)

def tx_tpms_loop(bus: CANInterface, state: VehicleState):
    """1000ms cycle - tire pressure (slow)"""
    tpms_ids = ["TPMS_FL", "TPMS_FR", "TPMS_RL", "TPMS_RR"]
    while True:
        for i, name in enumerate(tpms_ids):
            bus.send(ECU_IDS[name],
                     encode_tpms(state.tire_pressure[i], int(state.tire_temp[i])))
        time.sleep(1.000)

def tx_hvac_loop(bus: CANInterface, state: VehicleState):
    """500ms cycle - HVAC"""
    while True:
        bus.send(ECU_IDS["HVAC_STATUS"],
                 encode_hvac(state.fan_speed, state.hvac_temp_set, state.ac_on))
        time.sleep(0.500)

def tx_gateway_loop(bus: CANInterface, state: VehicleState):
    """1000ms cycle - heartbeat"""
    counter = 0
    while True:
        payload = bytes([0xAA, counter & 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        bus.send(ECU_IDS["GATEWAY_HEARTBEAT"], payload)
        counter += 1
        time.sleep(1.000)

def rx_control_loop(bus: CANInterface, state: VehicleState):
    """Listen for control frames from dashboard"""
    CTRL_IDS = {
        CTRL_ENGINE_START, CTRL_ENGINE_STOP, CTRL_THROTTLE,
        CTRL_DOOR, CTRL_LIGHTS, CTRL_GEAR, CTRL_HVAC,
        CTRL_LOCKS, CTRL_SEATBELT
    }
    log.info("📡  Listening for control frames...")
    while True:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id in CTRL_IDS:
            handle_control_frame(msg, state)

def physics_loop(state: VehicleState):
    """20ms physics update"""
    while True:
        state.update_physics(dt=0.020)
        time.sleep(0.020)


# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────

def main():
    log.info("=" * 50)
    log.info("  CAN Bus Lab - ECU Simulator")
    log.info("=" * 50)
    log.info(f"Connecting to interface: {INTERFACE}")

    bus   = CANInterface(INTERFACE)
    state = VehicleState()

    threads = [
        threading.Thread(target=physics_loop,       args=(state,),        daemon=True, name="physics"),
        threading.Thread(target=tx_engine_loop,     args=(bus, state),    daemon=True, name="tx-engine"),
        threading.Thread(target=tx_engine_temps_loop, args=(bus, state),  daemon=True, name="tx-temps"),
        threading.Thread(target=tx_abs_loop,        args=(bus, state),    daemon=True, name="tx-abs"),
        threading.Thread(target=tx_body_loop,       args=(bus, state),    daemon=True, name="tx-body"),
        threading.Thread(target=tx_ic_loop,         args=(bus, state),    daemon=True, name="tx-ic"),
        threading.Thread(target=tx_tpms_loop,       args=(bus, state),    daemon=True, name="tx-tpms"),
        threading.Thread(target=tx_hvac_loop,       args=(bus, state),    daemon=True, name="tx-hvac"),
        threading.Thread(target=tx_gateway_loop,    args=(bus, state),    daemon=True, name="tx-gw"),
        threading.Thread(target=rx_control_loop,    args=(bus, state),    daemon=True, name="rx-ctrl"),
    ]

    log.info(f"Starting {len(threads)} threads...")
    for t in threads:
        t.start()
        log.info(f"  ✓ {t.name}")

    log.info("")
    log.info("🚗  ECU Simulator running. CAN IDs being broadcast:")
    for name, can_id in ECU_IDS.items():
        log.info(f"   0x{can_id:03X}  {name}")
    log.info("")
    log.info("📡  Waiting for dashboard control frames on 0x7F0-0x7F8")
    log.info("")

    try:
        while True:
            time.sleep(5)
            log.info(f"  RPM={state.rpm:.0f}  Speed={state.speed:.1f}km/h  "
                     f"Gear={state.gear_mode}  Throttle={state.throttle:.0f}%  "
                     f"Engine={'ON' if state.engine_running else 'OFF'}  "
                     f"Fuel={state.fuel_pct:.1f}%")
    except KeyboardInterrupt:
        log.info("Shutting down ECU simulator...")

if __name__ == "__main__":
    main()
