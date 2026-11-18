# GNC-ROV CoDR: Q&A prep (11/20/26)

Short answers for the likely questions on the GNC-ROV slides. Every number below is
in the repo or the RVM; the source is in brackets.

**Why 8 thrusters and not 6?**
With 2 verticals you get surge, sway, heave and yaw, but no active roll or pitch. We
need roll/pitch to hold attitude under the iceberg and to keep the photogrammetry
camera level. 8 thrusters (4 vectored horizontals + 4 verticals) give independent
6-DoF. [Frame-Config-Decision.md; SITL README mixing table]

**What if the horizontals aren't really at 45°?**
Then `vectored_6dof` is wrong and we add a custom motor matrix in
`AP_Motors6DOF::setup_motors()` (build-time patch). Practice project P1 already ran a
custom 4-thruster matrix through the same pipeline in SITL.
[practice/P1-4-thruster-frame.md]

**Why camera-based station keeping instead of the IMU or ArduSub's EKF?**
Accelerometer error integrates twice: a 2 mg bias is about 1 m off after 10 s, and a
steady drift produces no acceleration at all. The EKF only gets horizontal position
in SITL (simulated GPS) or with a DVL. The camera gives position directly; the IMU is
still used to remove tilt from the image motion (4° roll would otherwise read as ~9 cm).
[SITL README, Station keeping]

**How good is it?**
SITL: 0.3 cm mean / 1.2 cm max error through current changes up to 0.5 m/s; Task 1.3
judging condition held 48.2 s of 50. Calc C2 puts the real limit at ±13 cm for a 4:3
camera. Not yet shown: 1 m/s, real optics and lighting. [RVM GNC-ROV-03.01; Calc C2]

**Isn't SITL just a toy?**
It runs the real ArduSub 4.5.7 firmware, the same binary family the Navigator runs, so
the control code and parameters transfer unchanged. What it does not model well:
real thruster curves, real drag, camera optics. That is why every row is "SITL pass,
pool test pending".

**Depth hold numbers?**
±1 cm still water, ±1.75 cm in a 0.5 m/s current, with Bar30-like noise
(`SIM_BARO_RND 0.01`). Requirement is ±5 cm (Task 2.2). [RVM GNC-ROV-04.01]

**Latency?**
Pilot command to vehicle in SITL: 10 ms mean, 18 ms p95, 28 ms max (localhost).
Requirement < 100 ms. On the tether it gets measured before every dive.
[RVM GNC-ROV-04.04; servo-control_report.md]

**How does the throttle cap work, and can the pilot override it?**
`MOT_PWM_MIN` / `MOT_PWM_MAX` narrow the ESC command window. ArduSub scales full
thrust into that window (`calc_thrust_to_pwm` uses `MOT_PWM_MAX - 1500`), so full stick
still means full *allowed* thrust and no pilot gain can exceed it. Only a parameter
write changes it.

**What number?**
4 × 13 A × k + 3.3 A ≤ 36 A gives k ≤ 0.63. The PWM window comes from a measured
current-vs-PWM curve, because current is not linear in PWM. 13 A per thruster is
EPS's unverified estimate; agree the final value with Christopher Gibbs.

**What if the cap is too tight to hold station at 1 m/s?**
Then it is a power problem, not a control problem: options are moving the lights to
the other converter (frees 3.3 A), splitting thrusters differently across the two
converters, or accepting a lower current rating in the requirement. FNP's drag
estimate decides how much thrust 1 m/s needs.

**The 2027 manual isn't out. Why should we trust these requirements?**
The architecture, frame, depth hold and station keeping are task-independent. Only
sources and numbers change; we re-trace every GNC-ROV row within two weeks of release.
