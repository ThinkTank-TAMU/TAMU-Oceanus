"""Plot a Depth vs Time chart from a received_log.txt file.

Each line is "YYYY-MM-DD HH:MM:SS,value". "None" values are treated as 0.
Rows are sorted by time; duplicate timestamps keep the largest value.

Usage:
    python plot_depth.py [path/to/received_log.txt] [output.png]
"""

import sys
from datetime import datetime
from pathlib import Path

import matplotlib.dates as mdates
import matplotlib.pyplot as plt

DEFAULT_LOG = Path.home() / "Downloads" / "received_log.txt"


def load_log(path):
    readings = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            stamp, _, value = line.rpartition(",")
            t = datetime.strptime(stamp, "%Y-%m-%d %H:%M:%S")
            depth = 0.0 if value.strip() in ("None", "") else float(value)
            readings[t] = max(depth, readings.get(t, depth))
    times = sorted(readings)
    return times, [readings[t] for t in times]


def plot_depth(log_path=DEFAULT_LOG, out_path="depth_vs_time.png", show=True):
    times, depths = load_log(log_path)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times, depths, color="tab:blue", linewidth=1.2, marker=".", markersize=3)
    ax.set_title("Depth vs Time")
    ax.set_xlabel("Time")
    ax.set_ylabel("Depth")
    ax.invert_yaxis()  # depth increases downward
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    ax.grid(True, alpha=0.3)
    fig.autofmt_xdate()
    fig.tight_layout()

    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path} ({len(times)} points)")
    if show:
        plt.show()


if __name__ == "__main__":
    log_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_LOG
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("depth_vs_time.png")
    plot_depth(log_path, out_path)
