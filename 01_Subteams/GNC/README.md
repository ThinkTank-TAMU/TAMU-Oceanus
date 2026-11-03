# GNC — Guidance, Navigation & Control

GNC owns the software for both of Team Oceanus's vehicles: the ROV and the float.

> **C&C merged into GNC in October 2026.** C&C (Communication & Computing) built the
> float (Pico W, BLE, depth profiling). Its plan is folded into the
> [3-week plan](3_Week%20Plan.md#merged-cc-scope) and its code now lives in
> [`Float/FloatCode/`](Float/FloatCode/). The `01_Subteams/C&C/` folder was retired.

Shared Drive folder (same layout): <https://drive.google.com/drive/folders/1-ku-eTshWN_Sum3-sCxpTIQi-aLZqLar>

## Layout

| Path | Contents |
|---|---|
| [`3_Week Plan.md`](3_Week%20Plan.md) | The GNC plan, including the merged C&C scope and members (both vehicles) |
| [`GNC-Requirement-Calculations.md`](GNC-Requirement-Calculations.md) | Derivations behind the GNC rows of the RVM (both vehicles) |
| [`ROV/`](ROV/) | The ROV |
| [`ROV/Frame-Config-Decision.md`](ROV/Frame-Config-Decision.md) | Final frame decision: 8 thrusters, `vectored_6dof`. Read before touching the sim |
| [`ROV/SITL-Simulation/`](ROV/SITL-Simulation/) | ArduSub SITL kit: real firmware in Docker, control scripts, sample runs |
| [`ROV/Photogrammetry-App/`](ROV/Photogrammetry-App-README.md) | Git submodule → `akvaithi/MATE-ROV-Photogrammetry` (see [its README](ROV/Photogrammetry-App-README.md)) |
| [`Float/`](Float/) | The float (vertical profiler, Task 4.1) |
| [`Float/Float-Software-Design.md`](Float/Float-Software-Design.md) | Float firmware design document |
| [`Float/FloatCode/`](Float/FloatCode/) | Pico W MicroPython controller, BLE receiver, desk tests |

## Quick start

```bash
# ROV simulator (first run builds the image, ~20–40 min)
cd 01_Subteams/GNC/ROV/SITL-Simulation && ./run_sitl.sh
```

```bash
# Float desk tests (plain CPython, no hardware)
cd 01_Subteams/GNC/Float/FloatCode && python3 -m unittest discover -s tests
```
