# Handoff: GNC Practice Project 1 (4-thruster motor matrix in SITL)

Written 2026-10-12. Read this, then `CLAUDE.md` at the repo root and
`01_Subteams/GNC/SITL-Simulation/README.md`, before changing anything.

## The task

From the GNC May plan (`01_Subteams/GNC/3_Week Plan.md`, "Practice Projects"):

> **Project 1 — Simulation:** Create a basic alternative motor matrix (e.g., simple
> 4-thruster setup) and load it into SITL to understand the configuration pipeline.

It is the only GNC practice project not done (P2 `scripts/sitl_check.py` and P3 RTSP
drop/latency logging are done). It is listed as "not started" in the RVM workbook's
**Standard Software** tab, which is an SRR deliverable (SRR is **Friday 2026-10-16**).

The point is training: show how a frame other than the team's real one is defined,
loaded and verified, so members understand the pipeline the real frame went through.
**It must not change the real vehicle configuration.**

## Done means

1. A 4-thruster frame runs in the existing SITL kit, selected without touching
   `params/vectored_6dof.parm` (the controlled vehicle file) or the default
   `./run_sitl.sh` behaviour (which must still boot `VECTORED_6DOF`).
2. A mixing check, like `scripts/thruster_mixing.py`, shows which of the 4 outputs move
   for each axis, with a table of expected PWM per axis (same format as the
   vectored_6dof table in the SITL README), and states which DoF the frame cannot control.
3. A short write-up in the repo (e.g. `SITL-Simulation/practice/P1-4-thruster-frame.md`)
   with the steps, the table and what was learned, linked from the SITL README.
4. Docs updated: Standard Software tab row "GNC tutorial projects" (P1 → done),
   Summer Progress Tracker (new Progress Log row), Custom ArduSub Config Guide
   ("If the geometry is not BlueROV2 Heavy" section already mentions the practice
   project; add a pointer to the write-up), and `3_Week Plan.md` status table.

## Two ways to do it (pick after checking the source)

**A. Built-in frame (no firmware change).** ArduSub ships simple frames besides
vectored and vectored_6dof (SimpleROV-3/4/5 and others, selected by `FRAME_CONFIG`).
Check the actual enum and motor layout for the pinned branch in
`ArduSub/Parameters.cpp` and `libraries/AP_Motors/AP_Motors6DOF.cpp`
(`setup_motors()`), branch `Sub-4.5`, before relying on any number. Then check whether
`sim_vehicle.py -v ArduSub -f <frame>` has a matching SITL frame (see
`Tools/autotest/pysim/vehicleinfo.py` and `libraries/SITL/SIM_Submarine.cpp`). If
SITL's physics frame and `FRAME_CONFIG` disagree the sim will fly wrong, so both have
to match.

**B. Custom matrix (the real lesson).** Add the matrix to the `SUB_FRAME_CUSTOM` case
of `AP_Motors6DOF::setup_motors()` and rebuild. In this kit that means a build-time
patch like `apply_current_model.sh` (a `sed`/patch applied in the `Dockerfile`), not a
fork. This is the same path the team would take if TMS's CAD shows the real horizontals
are not at 45° (see `Frame-Config-Decision.md`).

Whichever you pick, select it through an **opt-in** path: e.g. a separate
`params/practice-4thruster.parm` and a `FRAME=` env var read by `sim_entry.sh`
(default stays `vectored_6dof`), or a separate compose service. Never put practice
values in `vectored_6dof.parm` or `sitl.parm`.

## Facts already established (don't re-derive)

- SITL = real ArduSub 4.5 (validated V4.5.7) in Docker, `./run_sitl.sh`; first build
  20-40 min. Scripts on `tcp:127.0.0.1:5780`, QGC on `tcp:localhost:5781`, one client
  per port. SITL itself occupies 5760-576x.
- `sim_entry.sh` loads `params/vectored_6dof.parm` then `params/sitl.parm` on every
  start; `.parm` edits need only `docker compose restart`. Code patches need
  `docker compose build`.
- `MAV_CMD_DO_MOTOR_TEST` times out in SITL; exercise mixing with RC override
  (`thruster_mixing.py` does this; `gnc_link.arm()` force-arms only in SITL).
- RC override channels: 1 pitch, 2 roll, 3 throttle, 4 yaw, 5 forward, 6 lateral;
  ArduSub ignores ±30 µs around 1500 (`RCn_DZ`).
- The SITL vehicle model is patched to Oceanus values (`apply_oceanus_model.sh`:
  24.5 N thrust real; mass/dims are estimates).

## Platform gotchas (macOS host)

- No `timeout`; use `perl -e 'alarm shift; exec @ARGV' <secs> <cmd>`.
- Foreground `sleep` is blocked by the harness; poll with `until …; do sleep 2; done`
  in the background or use Monitor.
- Connecting before the MAVProxy banner spams "EOF on TCP socket".
- After a `git mv`, `git add` the moved file explicitly.

## Repo and workflow

- Repo `ThinkTank-TAMU/TAMU-Oceanus`, work under `01_Subteams/GNC/` only (never `25-26/`).
- Branch, PR, merge (recent GNC work landed as PRs #42, #43). Ask before pushing.

## Drive documents to update (IDs)

- RVM workbook (Standard Software tab): `150cHHkowEpd59xtvnRCmagq3GoED8v-IglVlKp37rkc`
- Summer Progress Tracker: `1wniiKqIndSM9qXOthWhWbw_rPwRCrXZwsqm2SjomHbY`
- Custom ArduSub Config Guide: `1i9UoEuhculI0tAi5T_Ha28t8Ut6Wo4uNkIs1gtb1Fso`
