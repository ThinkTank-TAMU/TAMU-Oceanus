# Servo Control Architecture — Test Report

**Channel:** 14 (manipulator, per GNC-ICD-01)  ·  **Iterations:** 40  ·  **Env:** ArduSub SITL

## Response latency (MAV_CMD_DO_SET_SERVO → output register)
| metric | value |
|---|---|
| mean | 10.0 ms |
| p95  | 18.0 ms |
| max  | 27.8 ms |

Measured at 200 Hz SERVO_OUTPUT_RAW (±5 ms quantisation). This is the
MAVLink + firmware command latency, not mechanical servo travel.

## Command precision (commanded vs actual PWM)
| metric | value |
|---|---|
| max error | 0 µs |
| range tested | 1100–1900 µs |

The command path is exact in SITL (pass-through), so precision here validates
PWM resolution, not mechanical positioning.

## On hardware (to be measured on the bench)
- Mechanical travel time per degree (add to the latency above).
- Physical positional accuracy / repeatability of the servo.
- Holding torque under load.

Artifacts: `servo-control.csv`, `servo-control.png`
