# Oceanus Vehicle Model (SITL physics)

By default, ArduSub SITL flies ArduPilot's generic **BlueROV2** reference vehicle
(hardcoded in `libraries/SITL/SIM_Submarine.h`). We patch it with Team Oceanus'
numbers so the simulated **dynamics approximate our ROV**, via
[`apply_oceanus_model.sh`](apply_oceanus_model.sh) (run at image-build time).

> ⚠️ The control firmware (frame mixing, PID, MAVLink) is always real. This patch
> only affects the *physics body* — how fast it accelerates, glides, and turns.

## Values applied & where they came from

| Parameter | Value | Source | Confidence |
|---|---|---|---|
| Thrusters | 8 (vectored_6dof) | 4 CW + 4 CCW Diamond Dynamics TD1.2, GNC-ICD-01 wiring | **real** |
| Thrust / thruster | 24.5 N | Diamond Dynamics **TD1.2** max forward ~2.5 kgf (range 1.2–2.5 kgf; Technical Report + manufacturer) | **real** |
| Mass (`weight`) | 22.0 kg | design targets: <35 hard / <25 / <18 (TMS reqs, F&P "25 kg max") | **target — not measured** |
| Length | 0.60 m | "<1 m any dimension" req; 400 mm enclosure/chassis | estimate |
| Width | 0.50 m | same | estimate |
| Height | 0.35 m | same | estimate |
| Thruster mount radius | 0.28 m | frame scale | estimate |
| Equivalent sphere radius | 0.25 m | drag/inertia scale | estimate |
| Buoyancy | ~neutral (`SIM_BUOYANCY≈1`) | req: avg density = water (1 kg/L) | target |
| Drag coefficients | ArduPilot defaults [1.4, 1.8, 2.0] | **no measured hydrodynamics in repo** | unchanged |

## What's NOT in the repo (so these are estimates)
The 25-26 spec sheet is a blank template — there is **no measured assembled-ROV
mass or overall dimension** anywhere in the docs, only requirements/limits and
component data. **Update the values when the ROV is built and weighed.**

## How to update
1. Edit the variables at the top of [`apply_oceanus_model.sh`](apply_oceanus_model.sh).
2. Rebuild: `docker compose build` (recompiles SITL with the new model).
3. Restart: `./run_sitl.sh`.

Buoyancy alone is a live parameter — set `SIM_BUOYANCY` in QGroundControl/MAVProxy
without rebuilding.

## Caveats
- Mass is modelled twice in ArduPilot (`weight` for linear accel, a sphere-derived
  `mass` for rotational inertia) — an upstream approximation, not ours.
- Steady descent/cruise speed is **drag-limited** (≈ thrust vs drag), so it's
  largely mass-independent; mass shows up in acceleration transients and turn rate.
- Drag is a sphere approximation — no tether drag, vortices, or per-axis added mass.
