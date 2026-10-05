# Team Oceanus — TAMU MATE ROV

This is the Team Oceanus mono-repo (Texas A&M, MATE ROV competition).
Remote: `github.com/ThinkTank-TAMU/TAMU-Oceanus`.

Most AI-assisted work in this repo is **GNC** (Guidance, Navigation & Control)
software. The other subteam folders are mostly documents.

## Repository layout

```
01_Subteams/          ← CANONICAL working tree (use this, by subteam)
  ├─ C&C  EPS  F&P  TMS    mostly docs / meeting notes / 3-week plans
  └─ GNC/                  the active software lives here
       ├─ 3_Week Plan.md           the GNC plan we work against
       ├─ Frame-Config-Decision.md final frame decision (read before touching the sim)
       ├─ SITL-Simulation/         ArduSub SITL kit (Goals 1 & 2) — see its own README
       └─ Photogrammetry-App/      git submodule → akvaithi/MATE-ROV-Photogrammetry (Goal 3)
25-26/                ← LEGACY archive. Do NOT add new work here.
```

**Always work under `01_Subteams/`, never the `25-26/` year folder.** The year
folder is kept only for reference; new GNC code, docs, and runs go in
`01_Subteams/GNC/`.

## The Photogrammetry-App submodule

`01_Subteams/GNC/Photogrammetry-App` is a **git submodule** pointing at the
personal repo `akvaithi/MATE-ROV-Photogrammetry`. It has its own `.git` and its
own [CLAUDE.md](01_Subteams/GNC/Photogrammetry-App/CLAUDE.md). Commit app changes
*inside the submodule* (push to its own remote), then bump the submodule pointer
in this repo separately.

## Canonical ROV facts (don't re-derive these)

- **Frame:** 8 thrusters, ArduSub `vectored_6dof`, `FRAME_CONFIG = 2` (BlueROV2
  Heavy geometry — 4 vectored horizontal + 4 vertical). This is the team's final
  design and is what gives true, independently-controllable 6-DoF.
- **Thrusters:** Diamond Dynamics **TD1.2** (≈24.5 N each), *not* T200s.
- **Stack:** Navigator flight controller + Raspberry Pi 5 + BlueOS, MAVLink/pymavlink.
- **Vehicle physics model** (mass 22 kg, dims ~0.60×0.50×0.35 m) is
  **design-target/estimated** — only the thrusters are a confirmed real value.
  The 25-26 spec sheet is a blank template; update when the ROV is built & weighed.
  See [SITL-Simulation/OCEANUS-VEHICLE-MODEL.md](01_Subteams/GNC/SITL-Simulation/OCEANUS-VEHICLE-MODEL.md).

## GNC dev environment (macOS, Apple Silicon)

The SITL kit runs the **real ArduSub firmware** in Docker against a simulated
vehicle, so control logic tested here transfers to the Navigator unchanged.

- Build & launch: `cd 01_Subteams/GNC/SITL-Simulation && ./run_sitl.sh`
  (first run builds the image, ~20–40 min).
- Connect: scripts/pymavlink → `tcp:127.0.0.1:5780`; QGroundControl → `tcp:localhost:5781`.
  (SITL itself uses 5760–576x internally — don't reuse those for the MAVProxy hub.)
- Python scripts run from `SITL-Simulation/.venv` (`pip install -r scripts/requirements.txt`).

### Platform gotchas (these have bitten us before)

- **No `timeout` on macOS.** Use `perl -e 'alarm shift; exec @ARGV' <secs> <cmd>`.
- **Foreground `sleep` is blocked by the harness.** Use a `until <check>; do sleep 2; done`
  loop, `run_in_background`, or Monitor instead.
- **Connecting before MAVProxy binds** spams "EOF on TCP socket" — wait for the
  MAVProxy banner / "Received N parameters" before connecting scripts.
- `MAV_CMD_DO_MOTOR_TEST` is unreliable in ArduSub SITL (times out) — exercise the
  thruster mix via RC override (`thruster_mixing.py`) instead.
- Always `git add` explicitly after a `git mv` — staging the move doesn't stage
  in-file path edits (this caused a follow-up fix PR once).

### Station keeping and SITL parameters (Sept 2026)

- **Station keeping is camera-based** (`scripts/station_keeping.py --source vision`, the
  default). IMU + depth cannot give horizontal position: accelerometer error grows as
  t². The EKF source works only in SITL, which simulates a GPS, or with a DVL.
- **Two MAVProxy ports, one client each.** Run a second script on 5781, not 5780.
- **SITL loads `params/vectored_6dof.parm`, then `params/sitl.parm`** (`sim_entry.sh`).
  Put anything SIM-only in `sitl.parm`, never in the vehicle file.
- **`SIM_BARO_RND` must stay small (0.01).** The stock 0.2 adds ~20 cm of depth
  noise and makes depth hold look broken.
- **Rendered-camera tests use `SIM_STATE` truth** (`lat_int`/`lon_int`, not the float
  `lat`/`lon`, which are ~0.4 m coarse). Never use the EKF as ground truth.
- **ArduSub ignores stick input within ±`RCn_DZ` (30 µs).** Controllers must map past
  it (`gnc_link.axis_pwm`).

## Where work stands

Goals 1 (6-DoF sim + custom frame) and 2 (servo control architecture) are
**done** and documented; the SITL kit, sample runs, and a summary PDF live under
`01_Subteams/GNC/SITL-Simulation/`. Goal 3 (photogrammetry RTSP drop/latency
error logging) is the remaining item and lives in the submodule.
