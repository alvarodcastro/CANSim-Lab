"""
ECU Definitions for CAN Bus Lab
================================
Based on common OBD-II / proprietary CAN IDs used in real vehicles.
IDs are representative of real-world assignments (e.g., Toyota/Honda/Ford patterns).

Frame format: Standard 11-bit CAN IDs, 8-byte data frames.
"""

# ─────────────────────────────────────────────
#  ECU CAN IDs
# ─────────────────────────────────────────────

ECU_IDS = {
    # Engine Control Unit (ECM/PCM)
    "ECM_ENGINE_RPM_SPEED":     0x0C9,  # Engine RPM + Vehicle Speed
    "ECM_THROTTLE_LOAD":        0x0D0,  # Throttle position + Engine load
    "ECM_ENGINE_TEMP":          0x0D1,  # Coolant temp + Intake temp
    "ECM_FUEL_STATUS":          0x0D2,  # Fuel pressure + injection timing
    "ECM_IGNITION":             0x0D3,  # Ignition timing advance
    "ECM_OBD_REQUEST":          0x7DF,  # OBD-II broadcast request (standard)
    "ECM_OBD_RESPONSE":         0x7E8,  # OBD-II ECM response

    # Transmission Control Unit
    "TCU_GEAR_STATUS":          0x1A1,  # Current gear + mode (P/R/N/D)
    "TCU_TORQUE":               0x1A2,  # Torque demand / converter status

    # ABS / ESP / Stability Control
    "ABS_WHEEL_SPEEDS":         0x0B0,  # All 4 wheel speeds (FL,FR,RL,RR)
    "ABS_STATUS":               0x0B1,  # ABS active flags + brake pressure
    "ESP_YAW_LATERAL":          0x0B2,  # Yaw rate + lateral acceleration
    "ESP_STEERING_ANGLE":       0x0B3,  # Steering wheel angle

    # Body Control Module (BCM)
    "BCM_DOORS_WINDOWS":        0x3B0,  # Door open/closed + window positions
    "BCM_LIGHTS":               0x3B1,  # Light status (head, tail, turn, fog)
    "BCM_LOCKS":                0x3B2,  # Central locking status
    "BCM_WIPERS":               0x3B3,  # Wiper speed + washer

    # Instrument Cluster
    "IC_ODOMETER":              0x3A0,  # Odometer + trip meter
    "IC_FUEL_GAUGE":            0x3A1,  # Fuel level percentage
    "IC_WARNING_LAMPS":         0x3A2,  # Check engine, oil, battery etc.

    # HVAC Control
    "HVAC_STATUS":              0x3C0,  # Fan speed + temp setpoints + AC
    "HVAC_SENSORS":             0x3C1,  # Interior/exterior temp sensors

    # TPMS (Tire Pressure Monitoring)
    "TPMS_FL":                  0x2B0,  # Front-left tire pressure + temp
    "TPMS_FR":                  0x2B1,  # Front-right tire pressure + temp
    "TPMS_RL":                  0x2B2,  # Rear-left tire pressure + temp
    "TPMS_RR":                  0x2B3,  # Rear-right tire pressure + temp

    # Airbag / SRS
    "SRS_STATUS":               0x0F0,  # Airbag system status + seatbelts

    # Battery Management (EV/Hybrid style, for modern sim)
    "BMS_STATE":                0x420,  # State of charge + voltage
    "BMS_TEMP":                 0x421,  # Battery pack temperature

    # Gateway / Diagnostic
    "GATEWAY_HEARTBEAT":        0x100,  # Network health heartbeat
}


# ─────────────────────────────────────────────
#  Encoding / Decoding Helpers
# ─────────────────────────────────────────────

def encode_rpm_speed(rpm: int, speed_kmh: int) -> bytes:
    """
    Byte 0-1: RPM  (value * 4 = raw, so raw = rpm // 4 is wrong;
               OBD formula: rpm = (A*256+B)/4 )
    Byte 2-3: Vehicle speed km/h (raw = speed_kmh, 1 byte up to 255)
    Byte 4:   Engine status flags (bit0=running, bit1=warm, bit2=cruise)
    """
    rpm_raw = int(rpm * 4)  # store as rpm*4 for OBD compat
    rpm_hi = (rpm_raw >> 8) & 0xFF
    rpm_lo = rpm_raw & 0xFF
    spd = min(speed_kmh, 255)
    flags = 0x01  # engine running
    if speed_kmh > 0:
        flags |= 0x04
    return bytes([rpm_hi, rpm_lo, spd, 0x00, flags, 0x00, 0x00, 0x00])

def decode_rpm_speed(data: bytes) -> dict:
    rpm = ((data[0] << 8) | data[1]) / 4
    speed = data[2]
    return {"rpm": rpm, "speed_kmh": speed}


def encode_throttle_load(throttle_pct: float, load_pct: float) -> bytes:
    """throttle: 0-100% → 0-255, load: 0-100% → 0-255"""
    thr = int(throttle_pct * 2.55) & 0xFF
    ld  = int(load_pct * 2.55) & 0xFF
    return bytes([thr, ld, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

def decode_throttle_load(data: bytes) -> dict:
    return {
        "throttle_pct": round(data[0] / 2.55, 1),
        "load_pct":     round(data[1] / 2.55, 1),
    }


def encode_engine_temp(coolant_c: int, intake_c: int) -> bytes:
    """OBD formula: temp = A - 40  →  A = temp + 40"""
    cool = max(0, min(255, coolant_c + 40))
    inta = max(0, min(255, intake_c + 40))
    return bytes([cool, inta, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

def decode_engine_temp(data: bytes) -> dict:
    return {
        "coolant_c": data[0] - 40,
        "intake_c":  data[1] - 40,
    }


def encode_wheel_speeds(fl, fr, rl, rr) -> bytes:
    """Each wheel speed in km/h, 2 bytes each (0.01 km/h resolution)"""
    def enc(v):
        raw = int(v * 100) & 0xFFFF
        return (raw >> 8) & 0xFF, raw & 0xFF
    return bytes([*enc(fl), *enc(fr), *enc(rl), *enc(rr)])

def decode_wheel_speeds(data: bytes) -> dict:
    def dec(hi, lo): return ((hi << 8) | lo) / 100.0
    return {
        "fl_kmh": dec(data[0], data[1]),
        "fr_kmh": dec(data[2], data[3]),
        "rl_kmh": dec(data[4], data[5]),
        "rr_kmh": dec(data[6], data[7]),
    }


def encode_doors_windows(driver_open, pass_open, rl_open, rr_open,
                          hood_open=False, trunk_open=False) -> bytes:
    """
    Byte 0: door status bits
      bit0=driver, bit1=passenger, bit2=rear-left, bit3=rear-right
      bit4=hood, bit5=trunk
    Byte 1: window positions (0=closed, 255=fully open)
    """
    doors = 0
    if driver_open: doors |= 0x01
    if pass_open:   doors |= 0x02
    if rl_open:     doors |= 0x04
    if rr_open:     doors |= 0x08
    if hood_open:   doors |= 0x10
    if trunk_open:  doors |= 0x20
    return bytes([doors, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

def decode_doors_windows(data: bytes) -> dict:
    d = data[0]
    return {
        "driver_open":    bool(d & 0x01),
        "passenger_open": bool(d & 0x02),
        "rl_open":        bool(d & 0x04),
        "rr_open":        bool(d & 0x08),
        "hood_open":      bool(d & 0x10),
        "trunk_open":     bool(d & 0x20),
    }


def encode_lights(headlights, taillights, turn_left, turn_right,
                  fog_front=False, fog_rear=False, hazard=False) -> bytes:
    flags = 0
    if headlights:  flags |= 0x01
    if taillights:  flags |= 0x02
    if turn_left:   flags |= 0x04
    if turn_right:  flags |= 0x08
    if fog_front:   flags |= 0x10
    if fog_rear:    flags |= 0x20
    if hazard:      flags |= 0x40
    return bytes([flags, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

def decode_lights(data: bytes) -> dict:
    f = data[0]
    return {
        "headlights":  bool(f & 0x01),
        "taillights":  bool(f & 0x02),
        "turn_left":   bool(f & 0x04),
        "turn_right":  bool(f & 0x08),
        "fog_front":   bool(f & 0x10),
        "fog_rear":    bool(f & 0x20),
        "hazard":      bool(f & 0x40),
    }


def encode_fuel_gauge(fuel_pct: float) -> bytes:
    """0-100% → 0-255"""
    raw = int(fuel_pct * 2.55) & 0xFF
    return bytes([raw, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

def decode_fuel_gauge(data: bytes) -> dict:
    return {"fuel_pct": round(data[0] / 2.55, 1)}


def encode_tpms(pressure_kpa: float, temp_c: int) -> bytes:
    """pressure in kPa (0-655.35 range, 2 bytes), temp offset +40"""
    p = int(pressure_kpa * 100) & 0xFFFF
    t = max(0, min(255, temp_c + 40))
    return bytes([(p >> 8) & 0xFF, p & 0xFF, t, 0x00, 0x00, 0x00, 0x00, 0x00])

def decode_tpms(data: bytes) -> dict:
    return {
        "pressure_kpa": ((data[0] << 8) | data[1]) / 100.0,
        "temp_c":        data[2] - 40,
    }


def encode_gear(gear: int, mode: str) -> bytes:
    """
    gear: 0=neutral, 1-8=gear number, -1=reverse
    mode: 'P','R','N','D','S'
    """
    mode_map = {'P': 0, 'R': 1, 'N': 2, 'D': 3, 'S': 4}
    g = (gear & 0xFF)
    m = mode_map.get(mode, 3)
    return bytes([g, m, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

def decode_gear(data: bytes) -> dict:
    mode_map = {0:'P', 1:'R', 2:'N', 3:'D', 4:'S'}
    return {
        "gear": data[0],
        "mode": mode_map.get(data[1], 'D'),
    }


def encode_srs(seatbelt_driver, seatbelt_pass, airbag_ok=True) -> bytes:
    flags = 0
    if seatbelt_driver: flags |= 0x01
    if seatbelt_pass:   flags |= 0x02
    if airbag_ok:       flags |= 0x80
    return bytes([flags, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

def decode_srs(data: bytes) -> dict:
    f = data[0]
    return {
        "seatbelt_driver": bool(f & 0x01),
        "seatbelt_pass":   bool(f & 0x02),
        "airbag_ok":       bool(f & 0x80),
    }


def encode_hvac(fan_speed: int, temp_set_c: float, ac_on: bool) -> bytes:
    fan = max(0, min(7, fan_speed))
    temp_raw = int((temp_set_c + 40) * 2) & 0xFF
    flags = 0x01 if ac_on else 0x00
    return bytes([fan, temp_raw, flags, 0x00, 0x00, 0x00, 0x00, 0x00])

def decode_hvac(data: bytes) -> dict:
    return {
        "fan_speed": data[0],
        "temp_set_c": (data[1] / 2.0) - 40,
        "ac_on": bool(data[2] & 0x01),
    }
