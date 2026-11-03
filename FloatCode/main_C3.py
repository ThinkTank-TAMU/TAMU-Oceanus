# MAIN C3 - runs on the laptop (not the Pico):  python main_C3.py
# Reads the saved log txt file and shows it as a Depth vs Time chart. No Bluetooth.
# Uses plot_depth.py (must be in the same folder). Needs matplotlib:  pip install matplotlib
# To plot a different file:  python main_C3.py path\to\file.txt

import os
import sys

import plot_depth

LOG_FILE = os.path.join(os.path.expanduser("~"), "Downloads", "received_log.txt")
CHART_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "depth_vs_time.png")

log_file = sys.argv[1] if len(sys.argv) > 1 else LOG_FILE
print("Reading", log_file)
plot_depth.plot_depth(log_file, CHART_FILE)
