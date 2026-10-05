# Sample SITL runs

Example telemetry from the same scripted maneuver (settle → dive → cruise → turn
→ forward → strafe → surface), captured with `scripts/log_telemetry.py` and
rendered with `scripts/plot_telemetry.py`.

| Run | Vehicle model | Dive (same command) |
|---|---|---|
| `bluerov2-baseline_*` | ArduPilot stock BlueROV2 (10.5 kg, T200-class 51.5 N) | ~5.8 m |
| `oceanus-td12_*` | Team Oceanus model: 22 kg + **Diamond Dynamics TD1.2** thrusters (24.5 N) — see [../OCEANUS-VEHICLE-MODEL.md](../OCEANUS-VEHICLE-MODEL.md) | ~3.5 m |
| `current-drift_*` | Oceanus model holding depth while a **0.6 m/s current** is switched on — drifts ~14 m east (ConOps current-compensation test) | n/a |
| `current-compensation_*` | `station_keeping.py` PID vs a **0.4 m/s current**: drifts ~3.9 m during a no-control phase, then recovers to within ~0.2 m and holds | n/a |
| `variable-current_*` | `station_keeping.py` (tuned) vs a **random current** that changes every ~3–4 s (13 changes): holds **mean 0.12 m / max 0.31 m**. `_response.png` overlays the disturbance and the position error | n/a |
| `vision-current-hold.csv` | **Vision** station keeping (`--synthetic --camera down`, features tracker) vs a **0.4 m/s current**: true error ≤ 0.9 cm; camera estimate vs truth 0.44 cm mean. Columns include `true_n/true_e/true_d` from SITL `SIM_STATE` | n/a |
| `vision-task13_*` | **Task 1.3** (`--tracker target`): starts 22 cm too high; the current changes 0.3 → 0.5 → 0.2 m/s in three directions. Height set to 0.62 m (best 0.61 m); judge condition met 48.2 s of 50; 0.3 cm mean / 1.2 cm max error after settling. `.png` is the chart | n/a |
| `servo-control*` | **Goal 2** servo test on ch 14: command-path latency ~10 ms mean, 0 µs precision error. `_report.md` is the test report, `.png` the latency/precision chart, `.csv` the raw data | n/a |

Same command, shallower dive: the Oceanus ROV is heavier **and** its TD1.2 thrusters
push less than half a T200, so it's a slower, gentler vehicle. Top speed in the
Oceanus run is ~0.7 m/s vs ~1.2 m/s on the baseline.

Each run has a `_dashboard.png` (shareable figure) and a `_telemetry.csv` (raw log).
The Oceanus model still uses **design-target/estimated** mass and dimensions
(only the thrusters are a confirmed real value) — regenerate once the ROV is weighed.

To make your own: see "Capturing & presenting results" in the [main README](../README.md).
