# Team Oceanus — MATE ROV 🌊

Texas A&M underwater robotics team (ThinkTank) building an ROV and float for the
[MATE ROV Competition](https://materovcompetition.org/). This is our team
monorepo — design, code, and documentation, organized by subteam.

## The ROV at a glance

- **8-thruster, fully 6-DoF frame** — vectored (ArduSub `vectored_6dof`), with independent roll & pitch
- **Diamond Dynamics TD1.2** thrusters (4 horizontal + 4 vertical)
- **Navigator** flight controller on a **Raspberry Pi 5**, running **BlueOS + ArduSub**
- Piloted over **MAVLink** in QGroundControl; a companion **float** (vertical profiler) runs its own firmware

## Subteams

| Subteam | Focus |
|---|---|
| **GNC** — Guidance, Navigation & Control | Control software, ArduSub SITL simulation, photogrammetry; communication & computing (ROV & float code, cameras, C3 station) |
| **EPS** — Electrical Power Supply | Power distribution, PCB design |
| **TMS** — Thermal Mechanisms & Structures | Chassis, frame, claw, and float CAD |
| **C&C** — Communication & Computing | Merged into GNC (Oct 2026) |
| **F&P** — Fluids & Propulsion | Propellers, fluid/structural analysis (CFD/FEA) |

Each subteam's plans and notes live under [`01_Subteams/`](01_Subteams/).

> **Note:** C&C merged into GNC in October 2026. GNC now owns the ROV and float code, cameras and the C3 station; [`01_Subteams/C&C/`](01_Subteams/C&C/) is kept as C&C's record.

## Repository layout

| Path | Contents |
|---|---|
| [`01_Subteams/`](01_Subteams/) | Active work, by subteam (**canonical**) |
| [`FloatCode/`](FloatCode/) | Float (vertical profiler) firmware |
| [`ROV CAD files/`](ROV%20CAD%20files/) | Mechanical CAD |
| [`WiringSchematics/`](WiringSchematics/) | Electrical wiring diagrams |
| [`Technical Data/`](Technical%20Data/) | Datasheets and specs |
| [`doc/`](doc/) | Documentation |
| `25-26/` | Previous-season archive (not active) |

## Software highlights

- **GNC SITL simulation** — full ArduSub 6-DoF sim of our ROV, for testing control before hardware: [`01_Subteams/GNC/SITL-Simulation`](01_Subteams/GNC/SITL-Simulation)
- **Photogrammetry Studio** — RTSP-to-3D scanning app (macOS): [latest release](https://github.com/akvaithi/MATE-ROV-Photogrammetry/releases/latest)
