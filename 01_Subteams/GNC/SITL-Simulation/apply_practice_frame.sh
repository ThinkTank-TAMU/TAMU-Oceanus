#!/usr/bin/env bash
# GNC Practice Project 1: add a 4-thruster motor matrix to ArduSub SITL.
#
# TRAINING ONLY. Applied only when the image is built with PRACTICE_FRAME=1 (the
# `ardusub-sitl-practice4` compose service). The default image is not patched,
# and the real vehicle (FRAME_CONFIG 2, vectored_6dof) is never affected.
#
# A frame has to be defined in two places, and they must agree:
#   1. Firmware  - AP_Motors6DOF::setup_motors(), case SUB_FRAME_CUSTOM
#                  (FRAME_CONFIG 7): how ArduSub mixes pilot/controller demands
#                  into motor outputs.
#   2. Physics   - SIM_Submarine.cpp thruster table, picked by the sim_vehicle.py
#                  frame name: how the simulator turns those outputs into forces.
# If they disagree, the sim flies wrong (ArduSub thinks it is driving one vehicle,
# the physics simulates another).
#
# Why patch, not a built-in frame (checked against branch Sub-4.5):
#   - SimpleROV-4 (FRAME_CONFIG 5) shares the `default:` case and adds 5 motors.
#   - Custom (7) has no motors of its own: its `//break;` is commented out, so it
#     falls through to SimpleROV-3 (3 motors).
#   - SITL physics only has 6- (vectored) and 8- (vectored_6dof) thruster tables;
#     any other frame name silently gets the 6-thruster one.
#
# The layout is SimpleROV's motors 1-4 without its lateral thruster:
#   T1 starboard horizontal, T2 port horizontal  -> surge, yaw (differential)
#   T3 starboard vertical,   T4 port vertical    -> heave, roll (differential)
# Sway and pitch have no thruster: they cannot be controlled.
#
# Usage: apply_practice_frame.sh <path to ardupilot checkout>
set -euo pipefail
ROOT="${1:?path to ardupilot checkout required}"
MOTORS="$ROOT/libraries/AP_Motors/AP_Motors6DOF.cpp"
SIM="$ROOT/libraries/SITL/SIM_Submarine.cpp"

python3 - "$MOTORS" "$SIM" <<'EOF'
import sys

motors_path, sim_path = sys.argv[1], sys.argv[2]

def patch(path, old, new, marker):
    src = open(path).read()
    if marker in src:
        print(f"   already applied: {path}")
        return
    if src.count(old) != 1:
        # Fail the build rather than silently building an unpatched frame.
        sys.exit(f"ERROR: expected exactly one match in {path}; has the Sub-4.5 source changed?")
    open(path, "w").write(src.replace(old, new))
    print(f"   patched: {path}")

# 1. Firmware: give SUB_FRAME_CUSTOM (FRAME_CONFIG 7) its own 4-motor matrix.
patch(motors_path,
"""    case SUB_FRAME_CUSTOM:
        // Put your custom motor setup here
        //break;
""",
"""    case SUB_FRAME_CUSTOM:
        // Team Oceanus GNC Practice Project 1 (training only): 4 thrusters.
        _frame_class_string = "PRACTICE_4THRUSTER";
        //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           0,              0,              -1.0f,              0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     -1.0f,          0,              0,              -1.0f,              0,                  0,              4);
        break;
""",
"PRACTICE_4THRUSTER")

# 2. Physics: the same four thrusters, selected by frame name "vectored_practice4".
patch(sim_path,
"""Submarine::Submarine(const char *frame_str) :""",
"""// Team Oceanus GNC Practice Project 1 (training only). Must match the
// SUB_FRAME_CUSTOM matrix in AP_Motors6DOF.cpp (FRAME_CONFIG 7).
static Thruster practice4_thrusters[] =
{
       //       Motor #     Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor
       Thruster(0,          0,              0,              -1.0f,          0,                  1.0f,               0),
       Thruster(1,          0,              0,              1.0f,           0,                  1.0f,               0),
       Thruster(2,          1.0f,           0,              0,              -1.0f,              0,                  0),
       Thruster(3,          -1.0f,          0,              0,              -1.0f,              0,                  0)
};

Submarine::Submarine(const char *frame_str) :""",
"practice4_thrusters")

patch(sim_path,
"""    if (strstr(frame_str, "vectored_6dof")) {""",
"""    if (strstr(frame_str, "practice4")) {
        thrusters = practice4_thrusters;
        n_thrusters = 4;
    }
    if (strstr(frame_str, "vectored_6dof")) {""",
"thrusters = practice4_thrusters")
EOF

echo ">> Practice 4-thruster frame applied:"
grep -n "PRACTICE_4THRUSTER\|practice4" "$MOTORS" "$SIM" | sed 's/^/   /'
