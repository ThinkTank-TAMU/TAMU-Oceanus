# GNC Project Plan

## Objective

Finalize GNC software configurations, validate remaining control systems in simulation, validate servo control architecture, and refine the photogrammetry pipeline for future hardware integration.

---

## Goals Overview

| Goal | Measurables | Deliverables |
|------|-------------|--------------|
| **Configure & simulate 6-DoF vehicle** | Successful loading of custom 6-DoF frame config into ArduSub/BlueOS; successful execution of control algorithms in SITL | Custom frame configuration file; validated simulation environment; updated control architecture documentation |
| **Validate servo control architecture** | Successful actuation of test servos via BlueOS/MAVLink; minimal latency and precise positional control achieved | Code repository with servo control scripts; test report documenting response times and precision metrics |
| **Refine photogrammetry pipeline** | Continuous processing of RTSP video stream into 3D models with minimal error and dropped frames | Updated Python application code; documentation of RTSP-to-3D pipeline and performance metrics |

---

## Software & Tools

| Software/Tool | Purpose | Members | Resources/Links |
|---------------|---------|---------|-----------------|
| BlueOS | Core ROV operating system and communication | All GNC members | [BlueOS Documentation](https://docs.bluerobotics.com/ardusub-zola/software/onboard/BlueOS/) |
| Python | Custom application development (photogrammetry, control scripts) | All GNC members | [Python.org](https://www.python.org) |
| MAVLink / pymavlink | Communication protocol between GNC software and hardware | All GNC members | [MAVLink Developer Guide](https://mavlink.io/en/) |
| ArduSub (SITL) | Software-in-the-loop simulation for testing control algorithms and custom frame configurations | All GNC members | [ArduSub SITL Guide](https://ardusub.com/developers/sitl.html) |
| GitHub | Version control and task management | All GNC members | [TAMU-Oceanus Repo](https://github.com/ThinkTank-TAMU/TAMU-Oceanus) |

---

## Training Plan

**Custom frame configuration in ArduSub**
- Review ArduSub documentation on creating and compiling custom motor matrices for non-standard frame geometries

**BlueOS & MAVLink fundamentals**
- All new/junior members complete a tutorial on sending basic MAVLink commands via Python to a simulated BlueOS instance

**Photogrammetry pipeline review**
- Async or recorded walkthrough covering the custom Python app structure and the RTSP video ingestion process

---

## Practice Projects

- **Project 1 — Simulation:** Create a basic alternative motor matrix (e.g., simple 4-thruster setup) and load it into SITL to understand the configuration pipeline
- **Project 2 — MAVLink:** Write a Python script to send a basic heartbeat message via MAVLink and read simulated IMU data from the SITL environment
- **Project 3 — Photogrammetry:** Modify the existing photogrammetry app to include error logging if the RTSP stream drops or experiences high latency

---

## Documentation

- **Summer progress tracker** — weekly updates on individual task progress: [Summer Progress Tracker](https://docs.google.com/spreadsheets/d/1wniiKqIndSM9qXOthWhWbw_rPwRCrXZwsqm2SjomHbY/edit)
- **Custom ArduSub config guide** — step-by-step instructions for defining and loading the custom motor matrix into ArduSub: [Custom ArduSub Config Guide](https://docs.google.com/document/d/1i9UoEuhculI0tAi5T_Ha28t8Ut6Wo4uNkIs1gtb1Fso/edit) (repo version: [SITL-Simulation/README.md](SITL-Simulation/README.md#custom-ardusub-frame-config--step-by-step-deliverable))
- **Photogrammetry app** — detailed usage instructions and architecture overview: [MATE-ROV-Photogrammetry](https://github.com/akvaithi/MATE-ROV-Photogrammetry)

Later GNC documents (Drive, GNC folder): the Software Design Document (GNC-02-01-01-R0), the GNC ConOps, GNC-ICD-01, and [GNC-Requirement-Calculations.md](GNC-Requirement-Calculations.md).

## Status (October 2026)

| Goal | Status | Remaining |
|------|--------|-----------|
| 1. Configure & simulate 6-DoF vehicle | Done in SITL | Repeat on the assembled ROV; confirm thruster angles with TMS |
| 2. Validate servo control architecture | Done in SITL ([report](SITL-Simulation/sample-runs/servo-control_report.md)) | Bench test of travel time, repeatability, holding torque |
| 3. Refine photogrammetry pipeline | Done | Install and test RealityScan on the control PC; set the vehicle RTSP URL |

---

## Timeline

All tasks are assigned and tracked via [GitHub Issues](https://github.com/ThinkTank-TAMU/TAMU-Oceanus).

| Week | Tasks |
|------|-------|
| **Week 1** | Set up SITL environments; begin 6-DoF frame configuration in ArduSub; define servo testing protocols |
| **Week 2** | Load 6-DoF config into SITL and begin simulated flights; execute servo testing (async) |
| **Week 3** | Finalize photogrammetry app refinements; compile servo test reports; update GNC documentation with 6-DoF simulation results |
