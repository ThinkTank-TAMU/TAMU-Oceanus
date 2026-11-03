#!/usr/bin/env python3
"""Inject a water current into the SITL ROV — for ConOps current-compensation /
drift testing.

The SITL submarine model is patched (apply_current_model.sh) so SIM_WIND_SPD /
SIM_WIND_DIR act as a horizontal water current. This sets them live (no rebuild).

Usage:
    python3 inject_current.py <speed_m_s> <dir_deg> [conn]   # set a current
    python3 inject_current.py 0                              # clear the current
    python3 inject_current.py demo [conn]                    # show drift at 0/0.5/1.0 m/s

Examples:
    python3 inject_current.py 0.4 90      # 0.4 m/s toward east (090)
    python3 inject_current.py 0           # still water
"""
import sys
import time
from pymavlink import mavutil


def connect(conn):
    m = mavutil.mavlink_connection(conn)
    m.wait_heartbeat()
    return m


def setp(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)


def set_current(m, speed, direction):
    setp(m, "SIM_WIND_DIR", direction)
    setp(m, "SIM_WIND_SPD", speed)
    print(f"Current set: {speed:.2f} m/s toward {direction:.0f}° "
          f"({'still water' if speed == 0 else 'drift this way when unpowered'})")


def demo(m):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100000, 0, 0, 0, 0, 0)
    t = time.time()
    while time.time() - t < 30 and not m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2):
        pass

    def pos():
        p = None
        t0 = time.time()
        while time.time() - t0 < 0.4:
            x = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
            if x:
                p = x
        return p

    setp(m, "SIM_WIND_DIR", 90.0)
    print("Drift demo (current toward east, vehicle unpowered):")
    for spd in (0.0, 0.5, 1.0):
        setp(m, "SIM_WIND_SPD", spd)
        p0 = pos()
        time.sleep(6)
        p = pos()
        print(f"  {spd:.1f} m/s -> drift east {p.y - p0.y:+5.2f} m in 6s "
              f"(settles to {p.vy:+.2f} m/s)")
    setp(m, "SIM_WIND_SPD", 0.0)
    print("Current cleared.")


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(1)
    if args[0] == "demo":
        conn = args[1] if len(args) > 1 else "tcp:127.0.0.1:5780"
        demo(connect(conn))
    else:
        speed = float(args[0])
        direction = float(args[1]) if len(args) > 1 else 0.0
        conn = args[2] if len(args) > 2 else "tcp:127.0.0.1:5780"
        set_current(connect(conn), speed, direction)
