"""Shared MAVLink helpers for the GNC scripts.

Everything here works the same against ArduSub SITL and the real vehicle. The
one difference, arming, is decided by `is_sitl()` rather than by the caller: the
SIM-only shortcuts (disable ARMING_CHECK, force-arm) are only ever used when
the autopilot itself reports that it is a simulator.
"""
from __future__ import annotations

import math
import time

from pymavlink import mavutil

DEFAULT_CONN = "tcp:127.0.0.1:5780"     # SITL MAVProxy hub, scripts port

# ArduSub flight modes (custom_mode numbers)
MODE_STABILIZE = 0
MODE_ALT_HOLD = 2        # "Depth hold"
MODE_SURFACE = 9
MODE_MANUAL = 19

NEUTRAL = 1500
PWM_MIN, PWM_MAX = 1100, 1900
RC_RELEASE = 0           # RC_CHANNELS_OVERRIDE: 0 hands the channel back to the pilot
FORCE_ARM_MAGIC = 21196  # MAV_CMD_COMPONENT_ARM_DISARM param2 - SIM ONLY

# RC override channel map (ArduSub defaults), 0-based index into the 8-channel list
CH_PITCH, CH_ROLL, CH_THROTTLE, CH_YAW, CH_FORWARD, CH_LATERAL = range(6)


def connect(conn: str = DEFAULT_CONN, timeout: float = 30.0):
    """Open a MAVLink link and wait for the vehicle heartbeat."""
    m = mavutil.mavlink_connection(conn)
    hb = m.wait_heartbeat(timeout=timeout)
    if hb is None:
        raise TimeoutError(f"no heartbeat on {conn} within {timeout:.0f} s")
    return m


def request_streams(m, rates_hz: dict[int, float]) -> None:
    """Ask the autopilot to stream each message id at the given rate."""
    for msg_id, hz in rates_hz.items():
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)


def read_param(m, name: str, timeout: float = 2.0):
    """Return a parameter's value, or None if the autopilot does not have it."""
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and msg.param_id.rstrip("\x00") == name:
            return msg.param_value
    return None


def is_sitl(m) -> bool:
    """True only when talking to a simulator (SIM_* parameters exist only in SITL)."""
    return read_param(m, "SIM_SPEEDUP") is not None


def set_mode(m, mode: int) -> None:
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)


def arm(m, timeout: float = 5.0) -> bool:
    """Arm the vehicle; return True once the heartbeat reports armed.

    On SITL the prearm checks are relaxed and the arm is forced, because the sim
    has no GPS/compass calibration. On the vehicle neither shortcut is used: a
    normal arm request goes out and ArduSub's own prearm checks decide.
    """
    sim = is_sitl(m)
    if sim:
        m.mav.param_set_send(m.target_system, m.target_component, b"ARMING_CHECK", 0,
                             mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        time.sleep(0.4)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, FORCE_ARM_MAGIC if sim else 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb and hb.get_srcSystem() == m.target_system and \
                hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return True
    return False


def prearm_messages(m, window: float = 1.0) -> list[str]:
    """Collect any 'PreArm:' STATUSTEXT seen in the window (why an arm failed)."""
    out, t0 = [], time.time()
    while time.time() - t0 < window:
        s = m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.2)
        if s and "arm" in s.text.lower():
            out.append(s.text)
    return out


def disarm(m) -> None:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            0, 0, 0, 0, 0, 0, 0)


def set_depth_target(m, depth_m: float) -> None:
    """Move ArduSub's depth-hold target (ALT_HOLD only) to depth_m, positive down.

    ArduSub 4.5 accepts a z-only SET_POSITION_TARGET_GLOBAL_INT in ALT_HOLD and
    sets the depth controller's target directly, in the EKF's vertical frame
    (the same frame as LOCAL_POSITION_NED.z). It then moves there under its own
    PILOT_SPEED / PILOT_ACCEL_Z limits.
    """
    mask = 0b0000110111111011   # ignore everything except Z (x, y, vel, acc, yaw, yaw rate ignored)
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT, mask,
        0, 0, -depth_m, 0, 0, 0, 0, 0, 0, 0, 0)


def send_override(m, channels: list[int]) -> None:
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *channels)


def release_overrides(m) -> None:
    """Neutral first (stop any thrust immediately), then hand every channel back.

    ArduSub also drops a stale override after RC_OVERRIDE_TIME (3 s default),
    but that is 3 s of the last thrust command; this makes it immediate.
    """
    send_override(m, [NEUTRAL] * 8)
    time.sleep(0.05)
    send_override(m, [RC_RELEASE] * 8)


def rc_deadzones(m) -> dict[int, float]:
    """RCn_DZ for the forward/lateral/throttle override channels (ArduSub default 30 us)."""
    out = {}
    for ch in (CH_FORWARD, CH_LATERAL, CH_THROTTLE):
        v = read_param(m, f"RC{ch + 1}_DZ")
        out[ch] = 30.0 if v is None else float(v)
    return out


def axis_pwm(u: float, dz: float, limit: float = 400.0) -> int:
    """Controller output (us, +/-limit) -> RC override PWM, skipping ArduSub's dead zone.

    ArduSub ignores stick input within +/-RCn_DZ of neutral. A controller output
    of a few us would otherwise do nothing, and the hold limit-cycles around the
    target; the output is scaled into the live range outside the dead zone.
    """
    if abs(u) < 0.5:
        return NEUTRAL
    mag = dz + abs(u) * (limit - dz) / limit
    return int(round(NEUTRAL + math.copysign(min(mag, limit), u)))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
