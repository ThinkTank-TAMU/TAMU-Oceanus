# Float Software Design Document

**GNC-02-01-02-R0** · Team Oceanus GNC · 10/12/2026 · Prepared by Arun Vaithianathan

This is the document the GNC SDD (GNC-02-01-01-R0, §1.2) refers to when it says the
float firmware is "documented separately". It covers the float controller in
`FloatCode/` of the team repository, for MATE 2026 Task 4.1 (floats under the ice) and
the RVM rows GNC-FL-01 to 05. Values marked **[TBD]** are design targets that have
to be measured on the built float.

## 1. Scope and status

In scope: the MicroPython program on the float's Pico W (`FloatCode.py`, `float_logic.py`),
its drivers (`Depth_Sensor.py`, `sdcard.py`), and the mission-station receiver
(`BLE_Reciever.py`). Out of scope: the float's mechanical design and buoyancy engine
hardware (F&P / TMS), and the C3 display.

Status: the decision logic is desk-tested (35 unit tests, including a closed-loop
simulation of both profiles). Not yet run on the Pico or in water. Ownership of the
surface link and memory is still to be confirmed with C&C (GNC ConOps §1.2).

## 2. Requirements

| ID | Requirement | Design element |
|---|---|---|
| GNC-FL-01 | Connect to the mission station < 5 s after power-up (including after a brownout reboot) | BLE advertising is the first thing `main()` starts, at a 100 ms interval |
| GNC-FL-02 | Wireless link at the surface, before descent and after recovery | BLE (Pico W, decided 2026-01-05): live packets before GO, log download after recovery |
| GNC-FL-03 | > 8 MB non-volatile memory | SD card on SPI0; internal flash fallback (2 MB, enough for one run at about 45 B/s) |
| GNC-FL-04 | Bottom of float at 2.5 m ±33 cm for 30 s, then top of float at 40 cm ±33 cm for 30 s, two profiles, no surface break or ice contact | `Mission` state machine, `HoldTimer`, `pump_command` |
| GNC-FL-05 | Detect overhead ice at > 0.75 m | Ultrasonic, `ICE_DETECT_M = 0.75`; no reading counts as ice |
| Task 4.1 | 7 sequential packets in each band, ≥ 20 packets on the graph; tell the judge the sensor offset | 1 Hz logging (Calc C6); `sensor_targets()` gives the ranges to tell the judge |

## 3. Hardware

From the Drive drawing "Float Pico W Connections".

| Device | Interface | Pins | Notes |
|---|---|---|---|
| Raspberry Pi Pico W (or Pico 2 W) | | | MicroPython; BLE through the CYW43439 |
| MS5837-30BA pressure sensor (Blu-Sub M8 / Bar30) | I²C0, 0x76, 400 kHz | GP8 SDA, GP9 SCL | VIN from the separate 5 V; driver `Depth_Sensor.py` checks the PROM CRC and applies second-order compensation |
| Ultrasonic distance sensor, pointing up | UART0, 115200 | GP0 TX, GP1 RX | Through a 5 V–3.3 V level shifter. 4-byte frame `FF hi lo sum`, triggered with 0x55. **[TBD]** model, blind zone, and whether it is calibrated for water (if for air, `SOUND_SCALE = 1480/343`) |
| SD card reader | SPI0, 1 MHz | GP18 SCK, GP19 MOSI, GP16 MISO, GP17 CS | VCC from VSYS 5 V |
| Peristaltic pump (buoyancy engine) | PWM 20 kHz + direction | GP14 speed (white), GP15 direction (green) | 12 V supply, ground not shared with the Pico per drawing. **[TBD]** which direction floods the reservoir (`PUMP_IN_IS_FORWARD`) |
| Thermistor divider | ADC1 | GP27 | Logged to the diagnostic file only |

## 4. Software structure

| File | Runs on | Role |
|---|---|---|
| `FloatCode.py` | Pico (as `main.py`) | Hardware glue: pins, sensors, pump, SD, BLE, 4 Hz control loop, 1 Hz logging, watchdog |
| `float_logic.py` | Pico and laptop | All decisions: depth from pressure, packet format, ice status, pump law, hold timer, mission state machine, brownout resume. No `machine` import, so it is desk-testable |
| `tests/test_float_logic.py` | Laptop | Desk tests (`python3 -m unittest discover -s tests`) |
| `Depth_Sensor.py`, `sdcard.py` | Pico | Drivers |
| `BLE_Reciever.py` | Mission-station laptop | Connects, syncs the clock, sends GO / DUMP, saves the log and a CSV for the graph |
| `TestedFloatCode`, `Testing_Bluetooth.py` | Pico | Earlier bench scripts (SD, ultrasonic, BLE), kept for bring-up |

## 5. Depth and targets

- **Depth** = (P − P₀) / (ρ g), with ρ = 1025 kg/m³ (SG 1.025, the EGADS ice tank, GNC
  Calc C7) and g = 9.80665. Fresh-water density would over-read by about 6 cm at 2.5 m.
- **Surface zero P₀** is the mean of 20 readings at power-up, taken with the float in air
  on deck. It is saved to internal flash so it survives a reboot (§11).
- **Targets are in sensor depth.** With the pressure port `s` above the float bottom and a
  float height `H` (both **[TBD]**, defaults H = 0.50 m, s = 0):
  deep target = 2.50 − s; shallow target = 0.40 + (H − s).
  Before deployment, tell the judge the adjusted ranges, deep (2.50 − s) ± 0.33 m and
  shallow (0.40 + H − s) ± 0.33 m.

## 6. State machine

```
BOOT ──► PREDIVE ──GO──► DEEP1 ──► SHALLOW1 ──► DEEP2 ──► SHALLOW2 ──► END
  │                        ▲                                           │
  └─ underwater at boot ───┘ (resume saved segment, new 30 s hold)     └─ rise; send log on reconnect
```

| State | Entry | Behaviour | Exit |
|---|---|---|---|
| BOOT | Power-up / reboot | Advertise BLE; start sensors, SD; zero or resume (§11); start the watchdog | → PREDIVE, or the saved segment |
| PREDIVE | | Pump off. Sends a packet every second while the station is connected | `GO` from the station, accepted only after ≥ 1 packet has been sent (Task 4.1) |
| DEEPn / SHALLOWn | Previous segment done | Drive to the target with `pump_command`; `HoldTimer` counts in band | Hold timer reaches 33 s (30 s + 3 s margin) without leaving the band |
| END | SHALLOW2 done | Rise for recovery; the ice backstop still applies. On a new connection the stored log is sent | Power-off |

**Hold timing (`HoldTimer`).** The timer starts only inside ±(0.33 − 0.05) m, so a float
that just grazes the band edge does not start a hold it cannot keep. Once started it keeps
counting anywhere inside ±0.33 m. Leaving ±0.33 m resets it to zero, so the next attempt
is a full new 30 s, as the manual requires. The segment ends at 33 s so the record
clearly spans 30 s.

## 7. Buoyancy engine control

The pump changes the float's buoyancy, and buoyancy sets a drag-limited vertical speed,
so the pump acts on speed, not position. The control law (`pump_command`) has two loops:

1. **Outer loop:** desired speed v* = K_POS · error, capped at
   - V_MAX_FAST = 0.20 m/s in transit,
   - V_MAX_SLOW = 0.05 m/s when rising with ice in view or with no ultrasonic reading,
   - √(2 · A_BRAKE · |error|), the fastest speed the pump can still stop from before the
     target.
2. **Inner loop:** duty = K_VEL · (v* − v), clipped to ±100 %. Below 11 % the pump stalls,
   so the duty is set to 0. Inside 5 cm and 2 cm/s the pump is off.

v is a low-passed derivative of depth (`VelocityEstimator`). `StrokeTracker` dead-reckons
the reservoir fill (seconds of pumping at 100 %) and refuses to drive past either end once
the full-stroke time is known **[TBD]**.

**A_BRAKE must be measured, and set below the measured value.** Desk simulation with
A_BRAKE = 0.008 m/s², a 0.50 m float and ice at the waterline:

| Pump deceleration (m/s²) | 0.005 | 0.0075 | 0.0125 | 0.025 | 0.05 |
|---|---|---|---|---|---|
| Closest the float top came to the surface | broke surface by 17 cm | broke surface by 7 cm | 5.7 cm below | 15 cm below | 27 cm below |
| Both profiles completed (s) | 622 | 478 | 363 | 204 | 190 |

Even at 0.0125 the top briefly went shallower than the 7 cm band edge on the way up,
which restarts that hold. The model is crude (speed proportional to reservoir fill), so
the margins are only indicative. K_POS, K_VEL and the speed caps are **[TBD]**, to be
tuned in the pool.

## 8. Ice detection (GNC-FL-05, Calc C5)

The ultrasonic sensor on top of the float measures the range to whatever is overhead.

| Range | Status | Action |
|---|---|---|
| > 0.75 m | clear | Normal control |
| 0.20–0.75 m | detected | Rising speed capped at 0.05 m/s; depth control continues to the 40 cm target. Calc C5: ice at 0.75 m leaves 35 cm to settle |
| ≤ 0.20 m | too close | Pump at 100 % to sink, overriding everything (including END) |
| no valid reading for 1 s | unknown | **Treated as ice**: rising speed capped as for "detected" |

The old code's 30 cm threshold and its rejection of any reading under 50 cm are removed.
Readings are accepted from 30 mm **[TBD: sensor blind zone]** to 6 m.

## 9. Timing, packets and storage

- **Control loop** 4 Hz (250 ms); the ultrasonic read times out after 150 ms so it cannot
  stall the loop.
- **Logging** every 1 s (GNC Calc C6): a 33 s hold produces about 33 sequential in-band
  packets, well over the 7 required, and one late packet costs nothing.
- **Packet** (Drive "Float coding requirements"): `[EX01 1:51:42 UTC 9.8 kpa 1.00 meters]`.
  The pressure is gauge pressure above the surface zero, and the depth is sensor depth.
  `EX01` is the manual's example. **[TBD]** the company number MATE assigns.
- **Clock.** The Pico has no battery-backed clock. The station sends `T<unix seconds>` on
  every connect, and packets are then UTC. Before a sync, or after a reboot underwater,
  the clock is time since boot and the packet says `BOOT` instead of `UTC`, so an unsynced
  time is never presented as UTC.
- **Files:** `/sd/packets.txt` (competition packets), `/sd/diag.csv` (state, depth, speed,
  ice range, pump duty, hold time, thermistor), and `/float_state.json` on internal flash
  (zero, segment, reservoir estimate). Each line is flushed as it is written.

## 10. Link protocol (BLE)

Service `ABCDABCD-1234-ABCD-BBBB-123412341234`. TX characteristic `…1235` (read/notify,
float → station), RX characteristic `…1236` (write, station → float). Everything the float
sends is text lines split into 20-byte notifications, and the receiver reassembles them.

| Station → float | Effect |
|---|---|
| `T<unix seconds>` | Set the clock to UTC (sent automatically on connect) |
| `GO` | Start the descent; refused with a message until a packet has been sent |
| `DUMP` | Send the packet log: `BEGIN LOG`, the lines, `EOF` |

The log is also sent automatically on the first connection after END. A dump that the
link drops is resent on the next connection.

## 11. Failure handling

| Failure | Detection | Response |
|---|---|---|
| Brownout / reboot | Boot finds a saved mission and the pressure is > 10 cm below the saved zero | Keep the saved zero, resume the saved segment with a new 30 s hold, and re-advertise immediately (FL-01). Packets say `BOOT` until the station re-syncs |
| Program hang | 8 s hardware watchdog | Reboot into the path above |
| Program exception | `finally` in `FloatCode.py` | Pump off |
| Lost link at the surface | No connection | Keeps advertising. Before GO the float stays at the surface; after END the log stays on the SD card for a wired download |
| No ultrasonic reading | No valid frame for 1 s | Assume ice (§8) |
| Ice too close | Range ≤ 0.20 m | Sink at full rate |
| SD card fails | Mount error | Log to internal flash instead |
| Reservoir at either end | `StrokeTracker` (once the stroke time is set) | Pump stops in that direction |

## 12. Verification

| Step | Status |
|---|---|
| Desk tests: depth, SG, sensor offset, packet format and parse, ultrasonic frames, ice thresholds, pump law, hold timer restart rule, mission sequence, brownout resume, closed-loop profile simulation | Done (35 tests) |
| Bench: Pico boots and advertises < 5 s; station connects, syncs, GO accepted only after a packet; DUMP round trip | To do |
| Bench: pump direction, minimum duty, full-stroke time, A_BRAKE | To do |
| Bench: ultrasonic underwater range, blind zone, sound-speed scale | To do |
| Pool: two profiles with the diagnostic log; tune K_POS, K_VEL, caps | To do |

## 13. Open items

1. Float height, sensor offset, company number, pump direction, stroke time, A_BRAKE,
   ultrasonic model and calibration (all **[TBD]** above).
2. Recovery behaviour after SHALLOW2: rising to the surface is assumed. Confirm with the
   Task 4 rules and the recovery plan.
3. Surface-link and memory ownership with C&C (GNC ConOps §1.2; Roles doc gives float
   autonomy to C&C, the RVM gives GNC-FL-01..05 to GNC).
4. The manual's note "seven sequential packets spanning 30 seconds (0, 5, … 30)": the 1 Hz
   log contains those packets. If the judges want exactly 5 s spacing displayed, the
   receiver can show every fifth packet of a hold.
