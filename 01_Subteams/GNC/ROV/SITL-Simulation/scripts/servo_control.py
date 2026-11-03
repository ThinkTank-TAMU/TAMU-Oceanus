#!/usr/bin/env python3
"""GNC servo / manipulator control — Goal 2 (Servo Control Architecture).

Drives the manipulator servos on PWM channels 14-16 (per GNC-ICD-01) via
MAV_CMD_DO_SET_SERVO. Those channels default to SERVOn_FUNCTION = 0 (Disabled),
which is exactly what lets the GCS command them directly — no reboot needed.

Same MAVLink path works on SITL now and the Navigator later.

Usage:
    python3 servo_control.py <channel> <pwm>            # raw PWM (1100-1900)
    python3 servo_control.py <channel> pos <0..1>       # fraction of travel
    python3 servo_control.py <channel> deg <0..180>     # angle (180° servo)
    python3 servo_control.py gripper open|close         # preset on ch 14
    python3 servo_control.py <channel> sweep            # 1100->1900->1500 demo

Append a connection string as the last arg (default tcp:127.0.0.1:5780).
"""
import sys
import time
from pymavlink import mavutil

PWM_MIN, PWM_MID, PWM_MAX = 1100, 1500, 1900
GRIPPER_CH = 14


def connect(conn):
    m = mavutil.mavlink_connection(conn)
    m.wait_heartbeat()
    return m


def set_pwm(m, channel, pwm):
    """Command a servo channel to a raw PWM via MAV_CMD_DO_SET_SERVO."""
    pwm = int(max(PWM_MIN, min(PWM_MAX, pwm)))
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        channel, pwm, 0, 0, 0, 0, 0)
    return pwm


def set_position(m, channel, frac):
    """frac 0..1 across the servo travel."""
    return set_pwm(m, channel, PWM_MIN + frac * (PWM_MAX - PWM_MIN))


def set_angle(m, channel, deg, full_scale_deg=180.0):
    return set_position(m, channel, max(0.0, min(1.0, deg / full_scale_deg)))


def read_servo(m, channel, timeout=1.0):
    """Latest commanded output for a channel from SERVO_OUTPUT_RAW."""
    v = None
    t = time.time()
    while time.time() - t < timeout:
        s = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=timeout)
        if s:
            v = getattr(s, f"servo{channel}_raw", None)
    return v


def _ack(m):
    a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
    return a.result if a else None


if __name__ == "__main__":
    args = sys.argv[1:]
    conn = "tcp:127.0.0.1:5780"
    if args and args[-1].count(":") >= 1 and not args[-1].isdigit():
        conn = args.pop()

    if not args:
        print(__doc__); sys.exit(1)

    m = connect(conn)

    if args[0] == "gripper":
        pwm = PWM_MAX if args[1] == "open" else PWM_MIN
        set_pwm(m, GRIPPER_CH, pwm)
        print(f"gripper {args[1]} -> ch{GRIPPER_CH} = {pwm} (ack {_ack(m)})")
    elif args[1] == "sweep":
        ch = int(args[0])
        for pwm in (PWM_MIN, PWM_MAX, PWM_MID):
            set_pwm(m, ch, pwm)
            time.sleep(0.8)
            print(f"  ch{ch} -> {pwm}  (output {read_servo(m, ch, 0.3)})")
        print("sweep done")
    elif args[1] == "pos":
        ch = int(args[0]); pwm = set_position(m, ch, float(args[2]))
        print(f"ch{ch} pos {args[2]} -> {pwm} (ack {_ack(m)})")
    elif args[1] == "deg":
        ch = int(args[0]); pwm = set_angle(m, ch, float(args[2]))
        print(f"ch{ch} {args[2]}° -> {pwm} (ack {_ack(m)})")
    else:
        ch = int(args[0]); pwm = set_pwm(m, ch, int(args[1]))
        print(f"ch{ch} -> {pwm} (ack {_ack(m)})")
