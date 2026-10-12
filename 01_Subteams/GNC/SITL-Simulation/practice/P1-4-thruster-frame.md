# Practice Project 1: a 4-thruster motor matrix in SITL

> **Training only. This is not the Oceanus vehicle.** The ROV is 8 thrusters,
> `vectored_6dof`, `FRAME_CONFIG = 2` ([../params/vectored_6dof.parm](../params/vectored_6dof.parm)).
> Nothing here changes that file, `sitl.parm`, or what `./run_sitl.sh` starts.

From the GNC plan ([3_Week Plan.md](../../3_Week%20Plan.md), Practice Projects):

> **Project 1 — Simulation:** Create a basic alternative motor matrix (e.g., simple
> 4-thruster setup) and load it into SITL to understand the configuration pipeline.

The point is to walk the same path the real frame went through: define the
frame, load it, and prove it mixes and flies as intended. It is also the dry run
for a real custom matrix, which the team would need if TMS's CAD shows the
horizontals are not at 45° ([Frame-Config-Decision.md](../../Frame-Config-Decision.md)).

## Result

Verified 2026-10-12 on ArduSub `Sub-4.5` (branch head `abe1721`), Apple Silicon, Docker.

- Boots as `Frame: PRACTICE_4THRUSTER` with `FRAME_CONFIG = 7`.
- Mixing check passes: every output matches the expected table below.
- The simulated vehicle moves the way the mix says. Surge, heave, yaw and roll
  move it. Sway and pitch have no thruster and do not.
- Regression: `./run_sitl.sh` still boots `Frame: VECTORED_6DOF`, and
  `thruster_mixing.py` gives the same table as the [SITL README](../README.md#status-validated-end-to-end-).

## The frame

Two horizontal and two vertical thrusters, side by side on the vehicle's centreline:

```
        bow
   T2 ->|  |<- T1        T1, T2: horizontal, push forward     -> surge, yaw
   (T4) |  | (T3)        T3, T4: vertical                   -> heave, roll
        stern
   port        starboard
```

| Thruster | Output | Roll | Pitch | Yaw | Throttle | Forward | Lateral |
|---|---|---|---|---|---|---|---|
| T1 starboard horizontal | 1 | 0 | 0 | −1 | 0 | 1 | 0 |
| T2 port horizontal | 2 | 0 | 0 | 1 | 0 | 1 | 0 |
| T3 starboard vertical | 3 | 1 | 0 | 0 | −1 | 0 | 0 |
| T4 port vertical | 4 | −1 | 0 | 0 | −1 | 0 | 0 |

These are ArduSub's factor conventions. The layout is ArduSub's SimpleROV motors
1–4 without SimpleROV's lateral thruster.

**This frame cannot control sway or pitch.** No thruster has a lateral factor,
and both verticals sit on the fore-aft centreline, so neither can tip the nose.
Pitch is held only passively, by the buoyancy-over-weight righting moment.
Compare the real frame: four vectored horizontals give sway, and four verticals
at the corners give pitch as well as roll.

## Why a patch, not a built-in frame

ArduSub ships several frames besides `vectored_6dof`, so the first plan was to
pick a built-in 4-thruster one. The `Sub-4.5` source rules that out:

| `FRAME_CONFIG` | Name | What `setup_motors()` actually builds |
|---|---|---|
| 4 | SimpleROV-3 | 3 motors |
| 5 | **SimpleROV-4** | **5 motors.** Shares the `default:` case with SimpleROV-5 |
| 6 | SimpleROV-5 | 5 motors |
| 7 | **Custom** | **3 motors.** Its `//break;` is commented out, so it falls through to SimpleROV-3 |

The enum (`ArduSub/Parameters.cpp`): 0 BlueROV1, 1 Vectored, 2 Vectored_6DOF,
3 Vectored_6DOF_90, 4 SimpleROV-3, 5 SimpleROV-4, 6 SimpleROV-5, 7 Custom.

The simulator side has the same gap. `libraries/SITL/SIM_Submarine.cpp` has
thruster tables only for `vectored` (6) and `vectored_6dof` (8), and any other
frame name silently gets the 6-thruster table. So "SimpleROV-4" in SITL would
mix for 5 motors and fly as 6.

## How it is wired in

A frame lives in **two places that must agree**:

1. **Firmware mix**: `AP_Motors6DOF::setup_motors()`. How ArduSub turns pilot and
   controller demands into motor outputs. Selected by `FRAME_CONFIG`.
2. **SITL physics**: the thruster table in `SIM_Submarine.cpp`. How the simulator
   turns those outputs into forces and torques. Selected by the
   `sim_vehicle.py -f <frame>` name.

If they disagree, ArduSub drives one vehicle while the simulator models another,
and the sim flies wrong without reporting any error. On the real ROV the physics
is the hull itself, which is why the mixing check comes before any wet test.

| Piece | File | What it does |
|---|---|---|
| Build-time patch | [`apply_practice_frame.sh`](../apply_practice_frame.sh) | Fills the `SUB_FRAME_CUSTOM` case with the 4-motor matrix and adds the matching `practice4_thrusters` table to SITL. Fails the build if the source no longer matches |
| Docker build arg | [`Dockerfile`](../Dockerfile) `PRACTICE_FRAME` | Runs the patch only when `1`. The default image stays stock |
| Compose service | [`docker-compose.yml`](../docker-compose.yml) `ardusub-sitl-practice4` | `PRACTICE_FRAME=1`, image `gnc-ardusub-sitl:practice4`, `FRAME=vectored_practice4`. In the `practice` profile, so `./run_sitl.sh` never starts it |
| Entrypoint | [`sim_entry.sh`](../sim_entry.sh) | `FRAME` picks the vehicle parameter file and `sim_vehicle.py -f`. Default `vectored_6dof`. Refuses the practice frame on an unpatched image |
| Parameters | [`params/practice-4thruster.parm`](../params/practice-4thruster.parm) | `FRAME_CONFIG 7`. Loaded instead of `vectored_6dof.parm`, then `sitl.parm` |
| Check | [`scripts/practice4_mixing.py`](../scripts/practice4_mixing.py) | Drives each axis, compares outputs 1–4 with the expected table, lists the DoF nothing responds to. Runs only on SITL with `FRAME_CONFIG 7` |

The frame name `vectored_practice4` is chosen so that it starts with `vectored`.
SITL matches model names by prefix (`SITL_cmdline.cpp`), so it reaches the
submarine model with no change to SITL's model list. `sim_vehicle.py` prints
`WARNING: no config for frame` for it, which is expected: it has no default
parameter file, and ours is passed explicitly.

## Run it

```bash
cd 01_Subteams/GNC/SITL-Simulation
docker compose down                                   # stop the default sim (same ports)
docker compose up --build ardusub-sitl-practice4      # first build ~2 min on top of the cached image
# wait for "AP: Frame: PRACTICE_4THRUSTER" and the MAVProxy banner, then in another terminal:
.venv/bin/python scripts/practice4_mixing.py
```

Back to the real vehicle: `Ctrl-C`, then `./run_sitl.sh`.

## Expected outputs

Same method as the [vectored_6dof table](../README.md#status-validated-end-to-end-):
MANUAL mode, one RC-override axis at 1750 µs (+250) for 0.8 s, then read
`SERVO_OUTPUT_RAW`. Measured equals expected on every row (tolerance ±25 µs).

| Command | T1 | T2 | T3 | T4 | Moves |
|---|---|---|---|---|---|
| neutral | 1500 | 1500 | 1500 | 1500 | — |
| forward | 1750 | 1750 | 1500 | 1500 | both horizontals |
| lateral | 1500 | 1500 | 1500 | 1500 | **nothing: sway uncontrolled** |
| heave | 1500 | 1500 | 1250 | 1250 | both verticals |
| yaw | 1250 | 1750 | 1500 | 1500 | horizontals, differential |
| roll | 1500 | 1500 | 1750 | 1250 | verticals, differential |
| pitch | 1500 | 1500 | 1500 | 1500 | **nothing: pitch uncontrolled** |

## Does the physics agree?

The mixing check tests only the firmware half. To test the other half, each
axis was held for 3 s (heave at 1300 µs, the rest at 1750 µs) and the motion
measured from `LOCAL_POSITION_NED` and `ATTITUDE`, in the body frame:

| Axis | Surge (m) | Sway (m) | Down (m) | Roll (°) | Pitch (°) | Yaw (°) |
|---|---|---|---|---|---|---|
| surge | **1.47** | −0.04 | −0.16 | −0.1 | 0.0 | 0.3 |
| sway | 0.16 | **−0.03** | −0.31 | 0.1 | −0.1 | 0.2 |
| heave | 0.07 | −0.03 | **0.97** | −0.1 | 0.1 | 0.1 |
| yaw | 0.05 | 0.00 | −0.23 | −0.1 | −0.3 | **130.3** |
| roll | −0.05 | 0.02 | −0.34 | **64.0** | −0.2 | 2.9 |
| pitch | −0.06 | −0.03 | −0.21 | −1.2 | **0.3** | 0.9 |

Each commanded axis moves the vehicle only in that axis, and the sway and pitch
commands produce no meaningful motion. The 0.16 m surge on the sway row is coast
left over from the surge test, and the small negative "down" values are the
simulator's slight positive buoyancy during each 3 s window.

## What was learned

1. **Read the source, not the frame's name.** "SimpleROV-4" builds five motors,
   and "Custom" silently builds three. Both would have booted without an error.
2. **A frame is defined twice in SITL.** The firmware mix and the simulator's
   thruster table are separate. A mismatch flies wrong with no error message.
   That is why this kit's checks compare outputs *and* motion.
3. **Thruster count is not DoF.** Four thrusters here give four DoF. The real ROV
   needs eight to get six. A DoF is controllable only if some thruster has a
   non-zero factor in it, and some independent combination drives it alone.
4. **The custom path works and is cheap.** A build-time patch plus `FRAME_CONFIG 7`
   is about 2 minutes of rebuild on top of the cached image. If TMS's CAD shows
   non-45° horizontals, the real frame takes the same route: new factors in
   `SUB_FRAME_CUSTOM` and in a SITL thruster table, verified with the same checks.
   On the vehicle it would also mean flashing a custom ArduSub build to the
   Navigator instead of BlueOS's stock firmware.
5. **Keep training configuration away from the controlled file.** The practice
   frame has its own `.parm`, image tag and compose service. `run_sitl.sh` and
   `vectored_6dof.parm` are untouched, and the practice check refuses to run on
   any other frame.

## Source references (branch `Sub-4.5`)

- `ArduSub/Parameters.cpp`: `FRAME_CONFIG` values
- `libraries/AP_Motors/AP_Motors6DOF.h` / `.cpp`: `sub_frame_t`, `setup_motors()`
- `libraries/SITL/SIM_Submarine.cpp`: SITL thruster tables, frame selection
- `libraries/AP_HAL_SITL/SITL_cmdline.cpp`: model-name prefix matching
- `Tools/autotest/pysim/vehicleinfo.py`: `sim_vehicle.py` frames for ArduSub
