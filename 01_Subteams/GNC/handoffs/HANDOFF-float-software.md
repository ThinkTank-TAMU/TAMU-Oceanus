# Handoff: float software (code fixes + design documentation)

Written 2026-10-12. Read this and `CLAUDE.md` at the repo root first.

## Why this exists

GNC owns the float requirements **GNC-FL-01 to 05** in the RVM, and the GNC ConOps
(section 5.4.8) describes the float controller. The GNC SDD (GNC-02-01-01-R0) puts the
float out of scope and says it is "documented separately". **No such document exists.**
The code in `FloatCode/` (repo root, not under `01_Subteams/`) does not run.

## Step 0: confirm ownership before editing code

The last commits to `FloatCode/` are by **Andrei Bugayong** (2026-09-06,
BLE files). The 26-27 Roles doc gives "float autonomous operation including depth
profiling and post-recovery data transmission" to **C&C**, while the RVM gives
GNC-FL-01..05 to GNC, and the ConOps says surface-link and memory ownership is "to
be settled with C&C". Ask Arun whether to (a) fix the code directly, (b) open a PR for
the float owners to review, or (c) only document the problems. Default: (b).

## Code problems found (FloatCode/FloatCode.py)

1. **Syntax error, file does not run:** `pwm =  Pin(14), freq=20000` inside
   `actuate_buoyancy_engine` (line 147). Python reports "cannot assign to function call".
2. `format_data_packet(timestamp, depth, temp_raw)` takes 3 args; `main()` calls it with
   4 (`timestamp, pressure_pa, depth_m, temp_c`).
3. `scan_surface` is defined twice (the second returns 0, silently replacing the real
   one) and `actuate_buoyancy_engine` is defined twice (the second only prints).
4. Ice threshold is **30 cm**; GNC-FL-05 requires detection at **> 0.75 m**. The
   ultrasonic range filter also discards anything under 50 cm.
5. Depth uses density 1000 and a fixed 101325 Pa. GNC uses **SG 1.025** in the ice tank
   (GNC Calc C7) and a **surface zero** taken at deployment.
6. `main()` logs every 5 s. GNC Calc C6 says log at **1 s or faster**: seven packets in a
   30 s hold at exactly 5 s leave no margin for one late packet.
7. No profile state machine: nothing commands 2.5 m / 40 cm holds, counts the 30 s,
   restarts the 30 s if the float leaves the ±33 cm band, or runs the second profile.
8. Packet format must match the competition example `[EX01 1:51:42 UTC 9.8 kpa 1.00 meters]`
   (from the Drive doc "Float coding requirements"); current format differs.

Other files: `Depth_Sensor.py`, `sdcard.py` (driver), `BLE_Reciever.py`,
`Testing_Bluetooth.py`, `TestedFloatCode` (no extension). All other `.py` files parse.

## Requirements (source of truth: RVM "26-27 RVM" tab, rows GNC-FL-01..05)

- FL-01 connect to the mission station < 5 s after power-up (brownout reboot)
- FL-02 wireless link at the surface, before descent and after recovery (Bluetooth,
  Pico W, decided 2026-01-05)
- FL-03 > 8 MB non-volatile memory (SD card)
- FL-04 hold bottom of float at 2.5 m ±33 cm for 30 s, then 40 cm ±33 cm (top of float)
  for 30 s, two profiles; no surface break or ice contact
- FL-05 detect overhead ice > 0.75 m
- Competition: 7 sequential packets per hold band, ≥ 20 packets on the graph; sensor
  offset from float bottom must be told to the judge.

## Deliverables

1. Fixed, runnable `FloatCode.py` (per the Step 0 decision), with a profile state
   machine, and a desk test (MicroPython-free unit tests of the pure logic: depth calc,
   packet format, band/hold timing).
2. **Float software design doc.** Either a new section in the SDD or a separate doc in
   the GNC Drive folder. The SDD was a .docx at the time of writing (now a Google Doc; edit it with the Docs connector); editing it meant unzip → edit
   `word/document.xml` → zip → upload as a new version (Drive "Manage versions") so the
   link stays. Cover: hardware (Pico 2W/Pico W, MS5837-30BA on I²C 0x76, ultrasonic on
   UART0, SD on SPI0, peristaltic pump PWM 14/15), state machine, packet format,
   timing, failure handling (brownout, lost link, no ultrasonic reading = assume ice).
3. Update the "Coding For Float" sheet (`11-15uI75wvFyxsbwVj-Y_xtJfL5RWFLM1XK1aYcbczo`)
   statuses, and change the SDD's "documented separately" sentence to point at the doc.
4. Add a `FloatCode/README.md`.

## Drive references (IDs)

- Float coding requirements (doc): `1NMZaV7ZSondo7AtQsaucB4NgG-DRIqeYlpdboZoroQg`
- Coding For Float (sheet): `11-15uI75wvFyxsbwVj-Y_xtJfL5RWFLM1XK1aYcbczo`
- Float Pico W Connections (drawing): `1ZnBWrEuK1_s_ib1kcNhZ-HWW-jrCpgdqmIh_wXlt56M`
- GNC ConOps: `1zurpz-AYavtrARDhuFbjJd1z2Z_sUJVo8RWaeyNOwSg` (section 5.4.8)
- GNC SDD (Google Doc since 2026-10-12; the .docx is trashed): `1E-1e4Evi1M56r1tCHtRKrSp3IqpdSRzJv2H578JTI-Y`
- RVM workbook: `150cHHkowEpd59xtvnRCmagq3GoED8v-IglVlKp37rkc`
- GNC requirement calcs: `01_Subteams/GNC/GNC-Requirement-Calculations.md` (C5, C6, C7)

## Workflow

Branch, PR, merge; ask before pushing. Edit Google Docs with the Docs connector
(read the google-workspace skill first); guard writes with the revision ID.
