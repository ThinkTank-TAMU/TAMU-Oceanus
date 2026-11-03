#!/usr/bin/env python3
"""GNC telemetry dashboard  —  turn a logged CSV into one shareable figure.

Reads a CSV from log_telemetry.py and renders a multi-panel dashboard
(trajectory, depth profile, attitude, thruster outputs, power, speed) as a PNG
you can drop straight into a test report or presentation — so testing reads as
a picture, not a wall of numbers.

Usage:
    python3 plot_telemetry.py [in.csv] [out.png] [title]
    python3 plot_telemetry.py telemetry.csv            # -> telemetry.png
"""
import csv
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

in_path = sys.argv[1] if len(sys.argv) > 1 else "telemetry.csv"
out_path = sys.argv[2] if len(sys.argv) > 2 else in_path.rsplit(".", 1)[0] + ".png"
title = sys.argv[3] if len(sys.argv) > 3 else "ROV SITL Test — Telemetry"


def load(path):
    cols = {}
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        data = {h: [] for h in header}
        for row in r:
            for h, v in zip(header, row):
                data[h].append(float(v) if v not in ("", None) else np.nan)
    return {h: np.array(v) for h, v in data.items()}


d = load(in_path)
t = d["t_s"]
dur = np.nanmax(t)

plt.style.use("seaborn-v0_8-darkgrid")
fig, ax = plt.subplots(2, 3, figsize=(16, 9))
fig.suptitle(f"{title}   ({dur:.0f}s run)", fontsize=16, fontweight="bold")

# 1) Top-down trajectory, coloured by time
a = ax[0, 0]
sc = a.scatter(d["east_m"], d["north_m"], c=t, cmap="viridis", s=10)
a.plot(d["east_m"], d["north_m"], color="gray", lw=0.5, alpha=0.5)
a.scatter(d["east_m"][0], d["north_m"][0], c="lime", s=90, marker="o", label="start", zorder=5, edgecolor="k")
a.scatter(d["east_m"][-1], d["north_m"][-1], c="red", s=90, marker="X", label="end", zorder=5, edgecolor="k")
a.set_title("Path (top-down)"); a.set_xlabel("East (m)"); a.set_ylabel("North (m)")
a.set_aspect("equal", adjustable="datalim"); a.legend(loc="best", fontsize=8)
fig.colorbar(sc, ax=a, label="time (s)")

# 2) Depth profile (down is down)
a = ax[0, 1]
a.fill_between(t, d["depth_m"], 0, color="steelblue", alpha=0.3)
a.plot(t, d["depth_m"], color="steelblue", lw=1.8)
a.invert_yaxis()
a.set_title(f"Depth profile (max {np.nanmax(d['depth_m']):.1f} m)")
a.set_xlabel("time (s)"); a.set_ylabel("depth (m)")

# 3) Attitude
a = ax[0, 2]
for k, c in [("roll_deg", "tab:red"), ("pitch_deg", "tab:green"), ("yaw_deg", "tab:blue")]:
    a.plot(t, d[k], label=k.replace("_deg", ""), color=c, lw=1.4)
a.set_title("Attitude"); a.set_xlabel("time (s)"); a.set_ylabel("deg"); a.legend(fontsize=8)

# 4) Thruster outputs (horizontals vs verticals)
a = ax[1, 0]
for i in range(1, 5):
    a.plot(t, d[f"servo{i}"], color="tab:orange", lw=1, alpha=0.8,
           label="horizontal 1-4" if i == 1 else None)
for i in range(5, 9):
    a.plot(t, d[f"servo{i}"], color="tab:purple", lw=1, alpha=0.8,
           label="vertical 5-8" if i == 5 else None)
a.axhline(1500, color="gray", ls="--", lw=0.8)
a.set_title("Thruster outputs (PWM µs)"); a.set_xlabel("time (s)")
a.set_ylabel("µs"); a.legend(fontsize=8)

# 5) Power
a = ax[1, 1]
a.plot(t, d["batt_v"], color="tab:red", lw=1.6, label="voltage (V)")
a.set_ylabel("V", color="tab:red"); a.tick_params(axis="y", labelcolor="tab:red")
a2 = a.twinx()
a2.plot(t, d["batt_a"], color="tab:blue", lw=1.6, label="current (A)")
a2.set_ylabel("A", color="tab:blue"); a2.tick_params(axis="y", labelcolor="tab:blue")
a.set_title("Power draw"); a.set_xlabel("time (s)")

# 6) Speed
a = ax[1, 2]
horiz = np.hypot(d["vx"], d["vy"])
a.plot(t, horiz, color="tab:cyan", lw=1.6, label="horizontal")
a.plot(t, np.abs(d["vz"]), color="tab:olive", lw=1.6, label="vertical")
a.set_title("Speed"); a.set_xlabel("time (s)"); a.set_ylabel("m/s"); a.legend(fontsize=8)

fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(out_path, dpi=150)
print(f"Wrote {out_path}  ({dur:.0f}s, {len(t)} samples)")
