#!/usr/bin/env python3
"""Practice Project 1 mixing check  —  the 4-thruster TRAINING frame (FRAME_CONFIG 7).

Like thruster_mixing.py, but for the practice frame: drives each degree of freedom
via RC override, reads SERVO_OUTPUT_RAW for outputs 1-4, compares them with the
expected mix, and reports which DoF no thruster responds to (the DoF this frame
cannot control).

SITL only. It refuses to run unless the autopilot is a simulator with
FRAME_CONFIG 7, i.e. the `ardusub-sitl-practice4` compose service. The real ROV
is vectored_6dof; check it with thruster_mixing.py.

Usage:
    python3 practice4_mixing.py [conn]     # default tcp:127.0.0.1:5780
Exit code 0 if every output matches the expected table (±25 µs), 1 otherwise.
"""
import sys
import time

import gnc_link as gl

conn = sys.argv[1] if len(sys.argv) > 1 else gl.DEFAULT_CONN
KICK = 1750       # how hard to push each axis (same as thruster_mixing.py)
TOL = 25          # µs
N = 4             # outputs on this frame

# T1 starboard horizontal, T2 port horizontal, T3 starboard vertical, T4 port vertical.
# Expected PWM for a +250 µs stick: 1500 + 250 x factor (AP_Motors6DOF, case SUB_FRAME_CUSTOM
# as patched by apply_practice_frame.sh).
EXPECTED = {
    "neutral": [1500, 1500, 1500, 1500],
    "forward": [1750, 1750, 1500, 1500],
    "lateral": [1500, 1500, 1500, 1500],   # no lateral thruster
    "heave":   [1500, 1500, 1250, 1250],
    "yaw":     [1250, 1750, 1500, 1500],
    "roll":    [1500, 1500, 1750, 1250],
    "pitch":   [1500, 1500, 1500, 1500],   # verticals sit on the centreline fore-aft
}
STICK = {"neutral": None, "forward": gl.CH_FORWARD, "lateral": gl.CH_LATERAL,
         "heave": gl.CH_THROTTLE, "yaw": gl.CH_YAW, "roll": gl.CH_ROLL, "pitch": gl.CH_PITCH}
DOF = {"forward": "surge", "lateral": "sway", "heave": "heave",
       "yaw": "yaw", "roll": "roll", "pitch": "pitch"}

master = gl.connect(conn)
time.sleep(2)  # let the autopilot finish booting
if not gl.is_sitl(master):
    sys.exit("Refusing: this is a training check for SITL only, and this autopilot is not SITL.")
frame = gl.read_param(master, "FRAME_CONFIG")
if frame is None or int(frame) != 7:
    sys.exit(f"Refusing: FRAME_CONFIG is {frame}, expected 7 (the practice frame).\n"
             "Start it with:  docker compose up --build ardusub-sitl-practice4")
print(f"Connected to {conn} (SITL, FRAME_CONFIG 7 = PRACTICE_4THRUSTER).")

gl.set_mode(master, gl.MODE_MANUAL)
if not gl.arm(master, timeout=5.0):
    sys.exit("Did not arm within 5 s: " + ("; ".join(gl.prearm_messages(master)) or "no reason given"))
print("Armed (MANUAL).\n")


def override(channel):
    """Hold one stick at KICK (or all neutral) for ~0.8 s so the mix settles."""
    chans = [gl.NEUTRAL] * 8
    if channel is not None:
        chans[channel] = KICK
    for _ in range(8):
        master.mav.rc_channels_override_send(master.target_system, master.target_component, *chans)
        time.sleep(0.1)


def read_outputs():
    latest = None
    deadline = time.time() + 1.5
    while time.time() < deadline:
        s = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2)
        if s:
            latest = [getattr(s, f"servo{i}_raw") for i in range(1, N + 1)]
    return latest or [0] * N


ok = True
uncontrolled = []
print(f"{'COMMAND':9s} | {'measured T1-T4':^21s} | {'expected T1-T4':^21s} | result")
print("-" * 68)
try:
    for label, expected in EXPECTED.items():
        override(STICK[label])
        got = read_outputs()
        match = all(abs(g - e) <= TOL for g, e in zip(got, expected))
        ok &= match
        if label != "neutral" and all(abs(g - gl.NEUTRAL) <= TOL for g in got):
            uncontrolled.append(DOF[label])
        print(f"{label:9s} | {' '.join(map(str, got)):^21s} | "
              f"{' '.join(map(str, expected)):^21s} | {'ok' if match else 'MISMATCH'}")
except KeyboardInterrupt:
    print("\ninterrupted")
    ok = False
finally:
    gl.release_overrides(master)
    gl.disarm(master)

print("\nDisarmed.")
print("Controllable DoF:", ", ".join(d for d in DOF.values() if d not in uncontrolled))
print("No thruster responds to:", ", ".join(uncontrolled) or "none",
      "-> this frame cannot control", " or ".join(uncontrolled) or "-")
print("Mix matches the expected table." if ok else "MISMATCH against the expected table.")
sys.exit(0 if ok else 1)
