# GNC SITL Simulation — ArduSub 6-DoF

Software-In-The-Loop (SITL) environment for validating Team Oceanus' control
systems and **custom 6-DoF frame configuration** before they touch hardware.

This runs the **real ArduSub firmware** against a simulated vehicle, so the same
frame mixing, PID loops, depth/heading hold, and MAVLink commands you test here
behave identically on the **Navigator + Raspberry Pi 5 + BlueOS** stack
(per `GNC-ICD-01`). That's why SITL is the right tool — not a hand-rolled
physics sim, which wouldn't match the autopilot.

> Covers **Goal 1 (Configure & Simulate 6-DoF Vehicle)** and **Practice Project 2
> (MAVLink heartbeat + IMU read)**, and gives the servo/thruster work
> (**Goal 2**) a place to run before hardware is ready.

---

## Status: validated end-to-end ✅

Built and run on macOS (Apple Silicon) with **ArduSub V4.5.7**. The sim boots
into `Frame: VECTORED_6DOF`, telemetry streams, and all six degrees of freedom
exercise the expected thrusters:

| Command | horizontal 1-4 | vertical 5-8 |
|---|---|---|
| neutral | 1500 1500 1500 1500 | 1500 1500 1500 1500 |
| forward | 1250 1250 1750 1750 | 1500 1500 1500 1500 |
| lateral | 1750 1250 1750 1250 | 1500 1500 1500 1500 |
| heave   | 1500 1500 1500 1500 | 1250 1250 1250 1250 |
| yaw     | 1750 1250 1250 1750 | 1500 1500 1500 1500 |
| roll    | 1500 1500 1500 1500 | 1750 1250 1750 1250 |
| pitch   | 1500 1500 1500 1500 | 1250 1250 1750 1750 |

Independent **roll and pitch** (driven by the 4 vertical thrusters) confirm true
6-DoF — a 2-vertical-thruster frame could not do this.

**Station keeping (Sept 2026):** camera-based, so it works on the real ROV without
a DVL. In SITL it holds within ~1 cm against currents up to 0.5 m/s and passes the
Task 1.3 judging condition for 48 s. See [Station keeping](#station-keeping-conops-current-compensation-tasks-13--21).

---

## The 6-DoF decision (why `vectored_6dof`)

| Frame | Thrusters | Controllable DoF | ArduSub `FRAME_CONFIG` |
|---|---|---|---|
| `vectored` (BlueROV2) | 6 (4 vec + 2 vert) | 4 — surge, sway, heave, yaw | 1 |
| **`vectored_6dof` (BlueROV2 Heavy)** | **8 (4 vec + 4 vert)** | **6 — adds roll & pitch** | **2** |

True, independently-controllable 6-DoF needs the **4 vertical thrusters** — with
only 2 vertical thrusters you cannot actively control roll and pitch. The
**8-thruster `vectored_6dof`** frame is the team's **final design** (confirmed
June 2026; 4 CW + 4 CCW Diamond Dynamics TD1.2, ICD wiring T1–T8). See
[../Frame-Config-Decision.md](../Frame-Config-Decision.md).

---

## Prerequisites

- **Docker Desktop** (macOS/Windows/Linux). This is the clean path on macOS,
  where native ArduPilot builds are painful.
- **QGroundControl** (for visual flight + PID tuning) — optional but recommended.
- **Python 3.10+** on your host for the scripts (`pip install -r scripts/requirements.txt`).

## Quick start

```bash
cd "01_Subteams/GNC/SITL-Simulation"
./run_sitl.sh            # first run builds the image (~20-40 min, ~6 GB), then launches
```

When it prints the MAVProxy banner, the sim is live:

| Connect | Endpoint | For |
|---|---|---|
| GNC scripts / pymavlink | `tcp:127.0.0.1:5780` | telemetry + commands |
| QGroundControl | `tcp:localhost:5781` | add **Application Settings → Comm Links → Add → TCP**, host `localhost`, port `5781` |

Then, in another terminal:

```bash
cd "01_Subteams/GNC/SITL-Simulation"
python3 -m venv .venv && source .venv/bin/activate
pip install -r scripts/requirements.txt

python3 scripts/sitl_check.py        # Project 2: heartbeat + live IMU / attitude / depth
python3 scripts/thruster_mixing.py   # drive each DoF, confirm 8-thruster frame mixing
```

Stop the sim with `Ctrl-C` in the `run_sitl.sh` terminal.

## Capturing & presenting results

No 3D viewer needed — record a run and turn it into one shareable figure:

```bash
# 1. start logging in one terminal (depth, attitude, position, power, thrusters)
python3 scripts/log_telemetry.py tcp:127.0.0.1:5780 run1.csv 10
# 2. fly the ROV (QGC joysticks, or thruster_mixing.py) — Ctrl-C the logger when done
# 3. render a dashboard PNG for the test report / presentation
python3 scripts/plot_telemetry.py run1.csv run1.png "Dive + Forward Test"
```

The dashboard shows the path travelled, depth profile, attitude, which thrusters
fired when, power draw, and speed — so testing reads as a picture, not numbers.

## Water-current testing (ConOps current compensation)

Stock ArduSub SITL is still water, so we patch the model (`apply_current_model.sh`,
baked into the image) so `SIM_WIND_SPD`/`SIM_WIND_DIR` act as a horizontal
**water current**. Set it live — no rebuild:

```bash
python3 scripts/inject_current.py 0.4 90   # 0.4 m/s toward east (090)
python3 scripts/inject_current.py 0        # back to still water
python3 scripts/inject_current.py demo     # drift at 0 / 0.5 / 1.0 m/s
```

Unpowered, the ROV drifts toward the current at the current speed — so you can
test drift detection and station-keeping / current-compensation control. See the
`current-drift_*` example in [sample-runs/](sample-runs/).

## Station keeping (ConOps current compensation, Tasks 1.3 / 2.1)

`station_keeping.py` holds horizontal position against a current: a PID on
position error drives forward/lateral thrust while ArduSub holds depth and heading
(`ALT_HOLD`). It takes its position from one of two sources.

**`--source vision` (default): the one that works on the real ROV.** When the hold
engages, the current camera image becomes the zero point, and every new frame is
compared with it ([`vision_position.py`](scripts/vision_position.py)). The IMU
cannot do this job alone. Its accelerometer error integrates twice into position
and grows with time squared: a 2 mg bias is already ~1 m off after 10 s. A steady
drift at constant velocity produces no acceleration to measure at all. The IMU is
still used: the roll/pitch/yaw change since the zero point is removed from the
image motion, so the vehicle tilting is not read as the vehicle moving (a 4° roll
would otherwise read as ~9 cm).

- `--tracker features`: ORB features matched to the zero-point image (RANSAC).
  Holds wherever the ROV was when the hold engaged. Works on any textured scene.
- `--tracker target`: Task 1.3. Centres on the 50 cm blue coral-garden square and
  reports the judge's condition: the whole blue square in view and no red pipe
  visible. The square's known size gives the height above it. The script sends
  that height to ArduSub's depth hold as a one-time target depth, then re-checks
  with the camera. The best height is computed from the camera's field of view.

**`--source ekf`**: ArduSub's own `LOCAL_POSITION_NED`. It works in SITL because
the sim has a GPS. On the vehicle it works only once a DVL (or other horizontal
position aid) is fitted.

```bash
# on the vehicle: down-looking camera over the coral garden (Task 1.3)
python3 scripts/station_keeping.py --video rtsp://192.168.2.2:8554/<stream> \
    --camera down --tracker target --hfov <camera HFOV deg> --duration 45

# in SITL: SITL has no camera, so --synthetic renders one from the simulated pose
python3 scripts/inject_current.py 0.4 90
python3 scripts/station_keeping.py --synthetic --camera down --tracker target --duration 45
python3 scripts/station_keeping.py --source ekf            # the original simulator-position hold
python3 scripts/test_vision_position.py                    # offline geometry checks, no SITL needed
```

**Safety.**
- If the vehicle is already armed (a normal dive), the script never arms or
  disarms it. On exit it only hands the sticks back.
- Overrides are released on every exit: end of run, Ctrl-C, SIGTERM, an exception,
  or tracking lost for 3 s. Thrust goes neutral as soon as tracking drops.
- An aborted hold exits with code 3.
- The SIM-only arming shortcuts are used only when the autopilot reports that it
  is SITL ([`gnc_link.py`](scripts/gnc_link.py)).
- Controller output is mapped past ArduSub's ±30 µs stick dead zone (`RCn_DZ`).
  Without that mapping, small corrections were ignored and the hold limit-cycled
  ±7 cm in still water.

**Results (SITL, rendered camera, vision source):**

| Run | Result |
|---|---|
| Features, 0.4 m/s current ([`vision-current-hold.csv`](sample-runs/vision-current-hold.csv)) | true error ≤ 0.9 cm the whole run; camera vs truth 0.44 cm mean |
| Task 1.3, starts 22 cm too high, current changes 0.3→0.5→0.2 m/s in three directions ([`vision-task13_hold.csv`](sample-runs/vision-task13_hold.csv), [plot](sample-runs/vision-task13_hold.png)) | height set to 0.62 m (best 0.61); judge condition met **48.2 s** of 50 (30 s needed); true error 0.3 cm mean, 1.2 cm max after 15 s |
| Still water | 0.3 cm mean, 0.6 cm max |

**Task 1.3 tolerance.** The whole blue square must be visible with no red, and
that sets how far the ROV may drift. With a 4:3 image the limit is about **±13 cm**;
with 16:9 it is only **±7 cm**. The camera's field of view changes the best height,
not the margin, so use the task camera in a 4:3 mode.

The earlier EKF-source runs are still in [sample-runs/](sample-runs/)
(`current-compensation_*`, `variable-current_*`).

## Servo control architecture (Goal 2)

The manipulator servos are on PWM channels **14-16** (GNC-ICD-01). They default to
`SERVOn_FUNCTION = 0` (Disabled), which lets the GCS drive them directly with
`MAV_CMD_DO_SET_SERVO` — no reboot or reconfig:

```bash
python3 scripts/servo_control.py gripper open       # ch14 -> 1900
python3 scripts/servo_control.py 14 deg 90          # angle command
python3 scripts/servo_control.py 15 sweep           # exercise a channel
python3 scripts/servo_test.py tcp:127.0.0.1:5780 14 40   # latency + precision -> report
```

`servo_test.py` produces the **Goal 2 test report** (`sample-runs/servo-control_report.md`):
command-path latency **~10 ms** (mean) and **0 µs** command error in SITL.

> SITL models the command/firmware path, not mechanical servo travel or physical
> positional error — measure those on the bench and add to the latency above.

---

## What's in here

| File | Purpose |
|---|---|
| `Dockerfile` | Builds ArduSub SITL from source (firmware = ground truth) |
| `sim_entry.sh` | Container entrypoint: starts SITL + a MAVProxy hub (multi-client) |
| `docker-compose.yml` / `run_sitl.sh` | One-command build & launch |
| `apply_oceanus_model.sh` + [`OCEANUS-VEHICLE-MODEL.md`](OCEANUS-VEHICLE-MODEL.md) | Patches SITL physics to our ROV's mass/size/thrust (edit + rebuild to update) |
| `params/vectored_6dof.parm` | The vehicle's parameter set: load onto the Navigator; SITL loads it too |
| `params/sitl.parm` | SIM-only overrides (prearm checks off, realistic depth-sensor noise). Never load on the vehicle |
| `scripts/sitl_check.py` | Project 2 — heartbeat + IMU/attitude/depth stream |
| `scripts/thruster_mixing.py` | Drive each DoF via RC override, read SERVO_OUTPUT_RAW |
| `scripts/log_telemetry.py` | Record depth/attitude/position/battery/thrusters to CSV for reports |
| `scripts/plot_telemetry.py` | Turn a logged CSV into a shareable dashboard PNG (no 3D sim needed) |
| `scripts/inject_current.py` | Add a water current (ConOps drift / current-compensation testing) |
| `scripts/station_keeping.py` | Position hold against current: vision (camera zero point) or EKF source; Task 1.3 mode |
| `scripts/vision_position.py` | Camera position measurement: feature / blue-square trackers, IMU tilt compensation, synthetic camera |
| `scripts/test_vision_position.py` | Offline checks of the vision geometry against rendered images |
| `scripts/gnc_link.py` | Shared MAVLink helpers: connect, SITL-only arming, override release, depth target, dead zones |
| `scripts/servo_control.py` | Drive the manipulator servos (ch 14-16) via MAV_CMD_DO_SET_SERVO |
| `scripts/servo_test.py` | Servo latency + precision test → CSV, chart, report (Goal 2) |
| `apply_current_model.sh` | Build-time patch making SIM_WIND act as a water current |

---

## Custom ArduSub frame config — step by step (deliverable)

1. **Pick the frame.** 8-thruster Heavy geometry → `FRAME_CONFIG = 2` (`vectored_6dof`).
   In SITL this is selected by `sim_vehicle.py -f vectored_6dof` (already wired in `sim_entry.sh`).
2. **On hardware**, set the same in QGroundControl → **Vehicle Setup → Frame**, or
   load `params/vectored_6dof.parm` via **Parameters → Tools → Load from file**.
3. **Confirm the mixing** with `scripts/thruster_mixing.py`: each DoF should move
   the expected group of thrusters (horizontals for surge/sway/yaw, verticals for
   heave/roll/pitch). Cross-check against `GNC-ICD-01 Figure 4 (Thruster
   Configuration)` and the
   [ArduSub thruster setup guide](https://www.ardusub.com/quick-start/vehicle-frame.html).
   (Note: ArduSub's `MAV_CMD_DO_MOTOR_TEST` is unreliable in SITL — it times out —
   so we exercise the mix via RC override instead.)
4. **Depth / heading hold** use ArduSub's defaults (`PSC_POSZ_P 3.0`,
   `ATC_ANG_YAW_P 6.0`), set explicitly in the parameter file. The file used to hold
   `1.0` / `0.5`, copied from the pseudocode's `kp_Depth` / `kp_Yaw`. Those are
   different quantities: 0.5 is a 12× softer heading hold. SITL now loads the file
   (`sim_entry.sh`), plus `params/sitl.parm` for SIM-only values. SITL used to add
   20 cm of noise to the depth reading (`SIM_BARO_RND 0.2`, the copter air-barometer
   default), and that noise is why depth hold looked like it missed ±5 cm. With
   Bar30-like noise (`0.01`) the default gains hold **±1.75 cm in a 0.5 m/s
   current**. Re-check on the vehicle.
5. **Non-standard geometry?** If the thruster angles/positions differ from
   BlueROV2 Heavy, a fully custom motor matrix means editing
   `AP_Motors6DOF::setup_motors()` and rebuilding (bump `FRAME_CONFIG` to its
   Custom slot). Open an issue and we'll branch the Dockerfile for it.

---

## Troubleshooting

- **Build fails on `install-prereqs`** — usually a transient apt mirror issue;
  re-run `./run_sitl.sh`. The build layer is cached up to that point.
- **`FRAME_CONFIG` not 2 after boot** — connect with QGC or MAVProxy and
  `param set FRAME_CONFIG 2`, then reboot the sim. Report it so we can pin the param file.
- **Scripts can't connect** — make sure `run_sitl.sh` printed the MAVProxy banner
  and ports 5780/5781 aren't taken (`lsof -i :5780`). Each port takes **one**
  client: with a script on 5780, a second script must use 5781 (QGC's port).
- **Changed a `.parm` file** — restart the container (`docker compose restart`);
  `sim_entry.sh` reloads both files on every start (no rebuild needed).
- **Different ArduSub version** — rebuild with another release:
  `docker compose build --build-arg ARDUSUB_REF=Sub-4.1`.

## References

- ArduSub SITL setup — https://www.ardusub.com/developers/sitl.html
- ArduSub frames / thrusters — https://www.ardusub.com/quick-start/vehicle-frame.html
- pymavlink + ArduSub examples — https://www.ardusub.com/developers/pymavlink.html
- MAVProxy — https://ardupilot.org/mavproxy/
