#!/usr/bin/env python3
"""GNC telemetry logger  —  record SITL (or real ROV) data to CSV.

Subscribes to the key flight messages and writes one timestamped row per sample:
depth, attitude, position (NED), velocities, battery, and all 8 thruster outputs.
Use it to capture data for test-report plots and the performance metrics in the
GNC deliverables. Works against SITL now and the Navigator later (same MAVLink).

Usage:
    python3 log_telemetry.py [conn] [out.csv] [rate_hz]
    python3 log_telemetry.py                       # tcp:127.0.0.1:5780, telemetry.csv, 10 Hz
Stop with Ctrl-C — the CSV is flushed continuously, so a kill is safe.
"""
import csv
import sys
import time
from pymavlink import mavutil

conn = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5780"
out_path = sys.argv[2] if len(sys.argv) > 2 else "telemetry.csv"
rate = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

master = mavutil.mavlink_connection(conn)
master.wait_heartbeat()
print(f"Connected to {conn}. Logging to {out_path} at {rate:g} Hz (Ctrl-C to stop).")

# Ask the autopilot to stream what we log.
for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
               mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
               mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
               mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
               mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, int(1e6 / max(rate, 1)), 0, 0, 0, 0, 0)

# Latest-value cache, updated as messages arrive; sampled on a fixed clock.
latest = {}
fields = ["t_s", "depth_m", "roll_deg", "pitch_deg", "yaw_deg",
          "north_m", "east_m", "down_m", "vx", "vy", "vz",
          "batt_v", "batt_a"] + [f"servo{i}" for i in range(1, 9)]

DEG = 57.29578
t0 = time.time()
with open(out_path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(fields)
    next_sample = t0
    try:
        while True:
            # drain everything waiting, keep newest of each type
            while True:
                msg = master.recv_match(blocking=False)
                if not msg:
                    break
                latest[msg.get_type()] = msg
            now = time.time()
            if now >= next_sample:
                a = latest.get("ATTITUDE")
                h = latest.get("VFR_HUD")
                p = latest.get("LOCAL_POSITION_NED")
                s = latest.get("SERVO_OUTPUT_RAW")
                b = latest.get("SYS_STATUS")
                row = [
                    round(now - t0, 3),
                    round(-h.alt, 3) if h else "",
                    round(a.roll * DEG, 2) if a else "",
                    round(a.pitch * DEG, 2) if a else "",
                    round(a.yaw * DEG, 2) if a else "",
                    round(p.x, 3) if p else "", round(p.y, 3) if p else "",
                    round(p.z, 3) if p else "",
                    round(p.vx, 3) if p else "", round(p.vy, 3) if p else "",
                    round(p.vz, 3) if p else "",
                    round(b.voltage_battery / 1000, 2) if b else "",
                    round(b.current_battery / 100, 2) if b and b.current_battery >= 0 else "",
                ] + ([getattr(s, f"servo{i}_raw") for i in range(1, 9)] if s else [""] * 8)
                w.writerow(row)
                f.flush()
                next_sample += 1.0 / rate
    except KeyboardInterrupt:
        print(f"\nStopped. Wrote {out_path}.")
