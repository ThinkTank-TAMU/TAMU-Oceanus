#!/usr/bin/env python3
"""GNC frame-mixing check  —  validate the 8-thruster vectored_6dof allocation.

Arms in MANUAL mode and drives each degree of freedom in turn via RC override,
reading SERVO_OUTPUT_RAW to show which thrusters respond. This exercises the
real ArduSub control-allocation matrix, so it confirms the custom 6-DoF frame
is wired correctly in the firmware (and, later, that ESC order/direction match
on the Navigator).

Why this and not MAV_CMD_DO_MOTOR_TEST: ArduSub's MAVLink motor-test path is
unreliable in SITL (times out); RC override is rock-solid and more informative.

Usage:
    python3 thruster_mixing.py [conn]      # default tcp:127.0.0.1:5780
"""
import sys
import time

import gnc_link as gl

conn = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5780"
SUB_MODE_MANUAL = 19
NEUTRAL = 1500
KICK = 1750  # how hard to push each axis

# ArduSub default RC channel functions
AXES = {"pitch": 0, "roll": 1, "throttle": 2, "yaw": 3, "forward": 4, "lateral": 5}

master = gl.connect(conn)
print(f"Connected to {conn} ({'SITL' if gl.is_sitl(master) else 'VEHICLE - secure it on the stand'}).")
time.sleep(2)  # let the autopilot finish booting

gl.set_mode(master, SUB_MODE_MANUAL)
# gnc_link.arm relaxes prearm checks and force-arms ONLY when the autopilot is SITL;
# on the vehicle it is a normal arm request that ArduSub's prearm checks can refuse.
if not gl.arm(master, timeout=5.0):
    print("Did not arm within 5 s:", "; ".join(gl.prearm_messages(master)) or "no reason given")
    sys.exit(1)
print("Armed (MANUAL).\n")


def override(**axis_values):
    """Hold an RC override for ~0.8s so the mix settles."""
    chans = [NEUTRAL] * 8
    for name, value in axis_values.items():
        chans[AXES[name]] = value
    for _ in range(8):
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component, *chans)
        time.sleep(0.1)


def read_servos():
    latest = None
    deadline = time.time() + 1.5
    while time.time() < deadline:
        s = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2)
        if s:
            latest = [getattr(s, f"servo{i}_raw") for i in range(1, 9)]
    return latest


print(f"{'COMMAND':12s} | {'horizontal 1-4':^23s} | {'vertical 5-8':^23s}")
print("-" * 66)
try:
    for label, axis in [("neutral", None), ("forward", "forward"), ("lateral", "lateral"),
                        ("heave", "throttle"), ("yaw", "yaw"),
                        ("roll", "roll"), ("pitch", "pitch")]:
        override(**({axis: KICK} if axis else {}))
        vals = read_servos() or [0] * 8
        h = " ".join(f"{v}" for v in vals[:4])
        v = " ".join(f"{v}" for v in vals[4:])
        print(f"{label:12s} | {h:^23s} | {v:^23s}")
except KeyboardInterrupt:
    print("\ninterrupted")
finally:
    # always: thrust to neutral, hand the sticks back, disarm
    gl.release_overrides(master)
    gl.disarm(master)

print("\nDisarmed. Horizontal thrusters drive surge/sway/yaw; verticals drive "
      "heave/roll/pitch - independent roll & pitch = true 6-DoF.")
