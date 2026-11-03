#!/usr/bin/env python3
"""GNC servo control-architecture test — Goal 2 deliverable.

Measures the servo command path on a manipulator channel (default 14):
  • response latency  — time from MAV_CMD_DO_SET_SERVO to the output register
    reflecting the commanded PWM (the MAVLink + firmware path)
  • command precision — commanded vs actual PWM across the travel range

Writes a CSV, a chart, and a markdown report. Run against SITL now and the
Navigator later (same MAVLink path).

NOTE: SITL models the command/firmware path, not the servo's mechanical travel
time or physical positional error — those must be measured on the bench. This
test validates the *software/comms architecture*.

Usage:
    python3 servo_test.py [conn] [channel] [iterations] [out_prefix]
"""
import csv
import statistics as stats
import sys
import time
from pymavlink import mavutil

conn = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5780"
CH = int(sys.argv[2]) if len(sys.argv) > 2 else 14
N = int(sys.argv[3]) if len(sys.argv) > 3 else 40
prefix = sys.argv[4] if len(sys.argv) > 4 else "servo_test"

m = mavutil.mavlink_connection(conn)
m.wait_heartbeat()
print(f"Connected. Testing servo channel {CH}, {N} latency iterations.")

# free the channel for direct control and stream the output fast
m.mav.param_set_send(m.target_system, m.target_component, f"SERVO{CH}_FUNCTION".encode(),
                     0, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
time.sleep(0.3)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 5000, 0, 0, 0, 0, 0)  # 200 Hz


def set_servo(pwm):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, CH, pwm, 0, 0, 0, 0, 0)


def flush():
    """Drop any buffered SERVO_OUTPUT_RAW so the next read is fresh."""
    while m.recv_match(type="SERVO_OUTPUT_RAW", blocking=False):
        pass


def read_latest(settle):
    """Return the most recent channel value after `settle` seconds."""
    obs = None
    t = time.time()
    while time.time() - t < settle:
        s = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.1)
        if s:
            obs = getattr(s, f"servo{CH}_raw", None)
    return obs


# ---- latency: command a new PWM, time until the output register shows it ----
import random
random.seed(3)
latencies = []
rows = []
cur = 1500
set_servo(cur); time.sleep(0.3)
for i in range(N):
    target = random.choice([1100, 1250, 1400, 1600, 1750, 1900])
    while target == cur:
        target = random.choice([1100, 1250, 1400, 1600, 1750, 1900])
    flush()                       # only count messages produced after the command
    t0 = time.time()
    set_servo(target)
    dt = float("nan")
    while time.time() - t0 < 1.0:
        s = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=1)
        if s and getattr(s, f"servo{CH}_raw", None) == target:
            dt = (time.time() - t0) * 1000.0  # ms
            break
    latencies.append(dt)
    obs = read_latest(0.05)
    rows.append({"i": i, "commanded": target, "observed": obs, "latency_ms": round(dt, 1)})
    cur = target
    time.sleep(0.08)

# ---- precision: sweep setpoints, compare commanded vs actual ----
prec = []
for pwm in range(1100, 1901, 100):
    set_servo(pwm)
    obs = read_latest(0.25)       # latest value after it settles
    prec.append((pwm, obs))

lat = [x for x in latencies if x == x]  # drop NaN
mean_l, max_l = stats.mean(lat), max(lat)
p95 = sorted(lat)[int(0.95 * len(lat)) - 1]
max_err = max(abs(c - o) for c, o in prec)

# ---- outputs: CSV, chart, markdown report ----
csv_path = f"{prefix}.csv"
with open(csv_path, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["i", "commanded", "observed", "latency_ms"])
    for r in rows:
        w.writerow([r["i"], r["commanded"], r["observed"], r["latency_ms"]])

try:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    plt.style.use("seaborn-v0_8-darkgrid")
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    fig.suptitle(f"Servo control test — channel {CH}", fontweight="bold")
    ax[0].hist(lat, bins=12, color="tab:blue", edgecolor="k")
    ax[0].axvline(mean_l, color="r", ls="--", label=f"mean {mean_l:.1f} ms")
    ax[0].set_title("Response latency (command → output)")
    ax[0].set_xlabel("latency (ms)"); ax[0].set_ylabel("count"); ax[0].legend()
    c = [p[0] for p in prec]; o = [p[1] for p in prec]
    ax[1].plot(c, o, "o-", color="tab:green")
    ax[1].plot([1100, 1900], [1100, 1900], "k--", lw=0.7, label="ideal")
    ax[1].set_title(f"Command precision (max error {max_err} µs)")
    ax[1].set_xlabel("commanded PWM (µs)"); ax[1].set_ylabel("actual PWM (µs)"); ax[1].legend()
    fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig(f"{prefix}.png", dpi=150)
    chart = f"{prefix}.png"
except Exception as e:
    chart = f"(chart skipped: {e})"

report = f"""# Servo Control Architecture — Test Report

**Channel:** {CH} (manipulator, per GNC-ICD-01)  ·  **Iterations:** {N}  ·  **Env:** ArduSub SITL

## Response latency (MAV_CMD_DO_SET_SERVO → output register)
| metric | value |
|---|---|
| mean | {mean_l:.1f} ms |
| p95  | {p95:.1f} ms |
| max  | {max_l:.1f} ms |

Measured at 200 Hz SERVO_OUTPUT_RAW (±5 ms quantisation). This is the
MAVLink + firmware command latency, not mechanical servo travel.

## Command precision (commanded vs actual PWM)
| metric | value |
|---|---|
| max error | {max_err} µs |
| range tested | 1100–1900 µs |

The command path is exact in SITL (pass-through), so precision here validates
PWM resolution, not mechanical positioning.

## On hardware (to be measured on the bench)
- Mechanical travel time per degree (add to the latency above).
- Physical positional accuracy / repeatability of the servo.
- Holding torque under load.

Artifacts: `{csv_path}`, `{chart}`
"""
with open(f"{prefix}_report.md", "w") as f:
    f.write(report)

print(f"\nLatency: mean {mean_l:.1f} ms, p95 {p95:.1f} ms, max {max_l:.1f} ms")
print(f"Precision: max error {max_err} µs over 1100-1900")
print(f"Wrote {csv_path}, {chart}, {prefix}_report.md")
