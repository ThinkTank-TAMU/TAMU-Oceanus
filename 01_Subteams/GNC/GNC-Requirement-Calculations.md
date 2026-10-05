# GNC Requirement Calculations

Derivations behind the numbers in the GNC rows of the Requirements Verification
Matrix (RVM). Each RVM row that uses a number links to its section here rather than
carrying the math in its rationale column.

Sources: the 2026 PIONEER Manual (`2026 PIONEER Manual_FINAL_Cover.pdf` at the
repo root), the SITL runs in `SITL-Simulation/sample-runs/`, and the published
references listed at the bottom.

---

## C1 — Translational step size < 2.0 cm (GNC-ROV-02.01)

Tasks 3.1 and 3.2 insert one PVC pipe into another (manual, Task 3):

| Insertion | Inserted part | Receiver | Radial clearance |
|---|---|---|---|
| Micropile (3.1) | 1-in PVC, OD 33.4 mm | 2-in PVC, ID 52.5 mm | (52.5 − 33.4) / 2 ≈ **9.5 mm** |
| Power connector (3.2) | ½-in PVC, OD 21.3 mm | 1½-in PVC, ID 40.9 mm | (40.9 − 21.3) / 2 ≈ **9.8 mm** |

These dimensions assume Schedule 40 pipe, since the manual gives only nominal sizes.
Commanding in steps of under 2.0 cm keeps the worst-case placement error at half a
step (under 1.0 cm), which is inside the ~1 cm radial clearance.

**Open:** this is a command-resolution limit. Horizontal *position feedback* at this
resolution needs either a DVL (for example a Water Linked A50: ±1 % long-term, works
from 5 cm altitude) or visual servoing. Without one of them, the pilot closes the
loop by camera.

## C2 — Position-hold tolerance ±13 cm (GNC-ROV-03.01)

Replaces the earlier ±35 cm placeholder, which was too loose.

- **Rule (manual, Task 1.3):** hold for 30 s with the **entire 50 cm blue square** in
  the video and **no part of the red 130 cm square** visible. The red square's clear
  span is taken as 1.28 m (130 cm less one ½-inch pipe width).
- **Geometry:** a camera image W wide and H tall shows a floor footprint w × (H/W)·w.
  With the ROV off-centre by d, along each image axis:
  - blue fully visible: (H/W)·w/2 ≥ 0.25 + d (the short axis binds)
  - red not visible: w/2 + d ≤ 0.64 (the long axis binds)
- **Widest margin:** set the two limits equal, which gives w = (1.28 + 0.50) / (1 + H/W)
  and allowed drift d = (1.28 − w) / 2:

| Image | Footprint width w | Allowed drift d |
|---|---|---|
| 4:3 | 1.02 m | **±13 cm** |
| 16:9 | 1.14 m | **±7 cm** |

  The camera's field of view sets the **height** that gives this footprint,
  h = (w/2) / tan(HFOV/2); for example 0.61 m at 80°. It does not change the margin.
  At zero drift the usable height band at 80° is 0.40–0.76 m.
- **Requirement:** ±13 cm with the task camera in a 4:3 mode, heading aligned with
  the square. With 16:9 video the requirement tightens to ±7 cm.
- **SITL (vision station keeping, `sample-runs/vision-task13_hold.*`):** 0.3 cm mean /
  1.2 cm max true error after settling, with the current changing 0.3 → 0.5 → 0.2 m/s
  in three directions. The judging condition was met for 48.2 s of a 50 s run.
  SITL has no tether drag, turbidity or camera noise, so expect hardware to be worse.

## C3 — Depth hold ±5.0 cm (GNC-ROV-04.01)

- **Rule:** Task 2.2 awards full keel-depth points only within 5 cm of true depth
  (5.01–10 cm earns half).
- **Sensor:** Bar30 (MS5837-30BA) resolution is 0.2 mbar ≈ **2 mm** of water, so it
  can resolve this band. Its absolute accuracy (±200 mbar) is poor, so depth is
  measured *relative to a surface zero* taken at launch.
- **SITL:** the earlier "16.8 cm max, not met" result was a **simulation artefact**.
  Stock SITL adds `SIM_BARO_RND = 0.2`, the copter air-barometer default, which is
  about 20 cm of noise on depth. With Bar30-like noise (`params/sitl.parm`,
  `SIM_BARO_RND 0.01`), ArduSub's default depth gains hold **±1 cm in still water and
  ±1.75 cm in a 0.5 m/s current**. That is met in simulation; confirm on the vehicle.

## C4 — Latency limits (GNC-ROV-04.03 / 04.04)

- **On-vehicle loop < 5 ms (04.03):** ArduSub's main loop runs at 400 Hz by default
  (2.5 ms per cycle). The limit therefore requires one IMU-to-PWM cycle plus 100 % margin.
- **Surface-to-vehicle < 100 ms (04.04):** Kaknjo et al. (2018) report that an
  average control-signal latency of up to 100 ms does not impair ROV control, and
  that degradation shows above 300 ms. Wider teleoperation studies put the onset of
  pilot performance loss at 100–170 ms.
- **SITL:** the Goal 2 servo test measured MAVLink command latency at **10 ms mean /
  18 ms p95 / 28 ms max** (`sample-runs/servo-control_report.md`). SITL runs on
  localhost, so the tether and BlueOS routing will add latency on hardware.

## C5 — Overhead detection > 0.75 m (GNC-FL-05)

- Target hold is 40 cm ± 33 cm (manual, Task 4.1), so the float must stop no
  shallower than 7 cm below the ice.
- If the ice is detected at 0.75 m, the float has 0.75 − 0.40 = **35 cm** to decelerate
  and settle at 40 cm, and a further 33 cm of tolerance before contacting the ice.

## C6 — Logging interval ≤ 5 s (GNC-FL-06.07)

- Task 4.1 requires seven sequential data packets inside each 30 s hold.
- Seven packets span six intervals, and 30 s / 6 = **5 s**. A 5 s interval therefore
  has zero margin: a single late or dropped packet fails the hold. **Recommend
  logging at 1 s or faster.**

## C7 — Specific gravity 1.025 (GNC-ROV-06.04)

- Manual (OPER-002 and Task 4.1): the EGADS ice tank has an SG of about 1.025.
- Depth = P / (ρ·g). Using fresh-water density (1000 kg/m³) over-reads depth by about
  2.5 %, which is ~6 cm at 2.5 m. That is a fifth of the float's ±33 cm band, and
  more than the ROV's ±5 cm keel-depth band.

---

### References
- 2026 MATE ROV Competition PIONEER Class Manual — Tasks 1.3, 2.2, 3.1, 3.2, 4.1; OPER-002, OPER-008, MECH-003P, ELEC-021P, ELEC-022P, ELEC-NRD-001, CCC-004.
- Kaknjo et al., "Real-Time Video Latency Measurement between a Robot and Its Remote Control Station: Causes and Mitigation," *Wireless Communications and Mobile Computing*, 2018. https://onlinelibrary.wiley.com/doi/10.1155/2018/8638019
- Network Latency in Teleoperation of Connected and Autonomous Vehicles: A Review, 2024. https://pmc.ncbi.nlm.nih.gov/articles/PMC11207977/
- Blue Robotics, Bar30 High-Resolution Depth/Pressure Sensor. https://bluerobotics.com/store/sensors-cameras/sensors/bar-depth-pressure-sensor/
- Blue Robotics, Water Linked DVL A50. https://bluerobotics.com/store/the-reef/dvl-a50/
