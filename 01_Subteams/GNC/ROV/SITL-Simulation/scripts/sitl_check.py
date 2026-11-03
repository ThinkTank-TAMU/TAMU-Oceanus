#!/usr/bin/env python3
"""GNC SITL sanity check  —  Practice Project 2 (MAVLink).

Connects to the ArduSub SITL, confirms a heartbeat, then streams the
simulated IMU, attitude, and depth/pressure so you can verify the GNC
software <-> autopilot telemetry link end to end.

Usage:
    python3 sitl_check.py                       # default tcp:127.0.0.1:5780
    python3 sitl_check.py udpin:0.0.0.0:14550   # any pymavlink connection string
"""
import sys
from pymavlink import mavutil

conn = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5780"
print(f"Connecting to {conn} ...")
master = mavutil.mavlink_connection(conn)
master.wait_heartbeat()
print(f"Heartbeat OK — system {master.target_system}, component {master.target_component}\n")

# Ask the autopilot to push the messages we care about (SET_MESSAGE_INTERVAL
# takes an interval in microseconds; 1e6 / hz).
for msg_id, hz in [
    (mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 10),
    (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10),
    (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, 5),
]:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

print("Streaming IMU / attitude / depth — Ctrl-C to stop\n")
try:
    while True:
        msg = master.recv_match(
            type=["RAW_IMU", "ATTITUDE", "SCALED_PRESSURE"],
            blocking=True, timeout=5)
        if msg is None:
            print("  ...no telemetry for 5s (is SITL running on that port?)")
            continue
        kind = msg.get_type()
        if kind == "ATTITUDE":
            print(f"ATT   roll={msg.roll:+.3f}  pitch={msg.pitch:+.3f}  yaw={msg.yaw:+.3f}  [rad]")
        elif kind == "RAW_IMU":
            print(f"IMU   acc=({msg.xacc:6d},{msg.yacc:6d},{msg.zacc:6d})  "
                  f"gyro=({msg.xgyro:6d},{msg.ygyro:6d},{msg.zgyro:6d})")
        elif kind == "SCALED_PRESSURE":
            print(f"PRES  abs={msg.press_abs:8.2f} hPa  temp={msg.temperature/100:.1f} C")
except KeyboardInterrupt:
    print("\nDone.")
