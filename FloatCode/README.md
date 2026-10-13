# FloatCode: the Oceanus float controller

MicroPython for the float's Raspberry Pi Pico W (MATE 2026 Task 4.1, RVM GNC-FL-01 to 05).
The design is in [Float-Software-Design.md](../01_Subteams/GNC/Float-Software-Design.md)
(also in the GNC Drive folder as GNC-02-01-02-R0).

## Files

| File | Where it runs | What it is |
|---|---|---|
| `FloatCode.py` | Pico, copied as `main.py` | Hardware glue: sensors, pump, SD, Bluetooth, control loop |
| `float_logic.py` | Pico and laptop | All the decisions (depth, packets, ice, pump law, hold timer, state machine). No hardware imports |
| `Depth_Sensor.py` | Pico | MS5837-30BA driver |
| `sdcard.py` | Pico | SD card driver |
| `BLE_Reciever.py` | Mission-station laptop | Connects, syncs the clock, sends GO / DUMP, saves the log |
| `tests/test_float_logic.py` | Laptop | Desk tests |
| `TestedFloatCode`, `Testing_Bluetooth.py` | Pico | Older bench scripts for SD, ultrasonic and BLE bring-up |

## Desk tests (no Pico needed)

```bash
python3 -m unittest discover -s tests -v
```

## Load the Pico

With MicroPython on the Pico W and `mpremote` installed:

```bash
mpremote cp float_logic.py Depth_Sensor.py sdcard.py : && mpremote cp FloatCode.py :main.py
```

It runs on every power-up. To stop it, connect with `mpremote` and press Ctrl-C (the
pump is switched off on exit).

## Run a profile

1. Power the float in air on deck. It takes the surface zero from 20 pressure readings
   and starts advertising as `Oceanus`.
2. On the mission-station laptop (`pip install bleak`):
   ```bash
   python BLE_Reciever.py --go
   ```
   It connects, sets the float's clock to UTC, shows at least one live packet, then
   sends GO. The float refuses GO until a packet has gone out.
3. The float runs DEEP1 → SHALLOW1 → DEEP2 → SHALLOW2 and rises.
4. After recovery:
   ```bash
   python BLE_Reciever.py --dump
   ```
   This saves `received_log.txt` and `received_log.csv` (for the graph). The float also
   sends the log by itself on the first connection after the profiles end.

## Before competition: tell the judge the offset

The packets report the depth at the pressure sensor. Set `FLOAT_HEIGHT_M` and
`SENSOR_ABOVE_BOTTOM_M` in `float_logic.py`, then:

```bash
python3 -c "import float_logic as f; d,s=f.sensor_targets(); print('deep %.2f-%.2f m, shallow %.2f-%.2f m' % (d-.33,d+.33,s-.33,s+.33))"
```

## Values still to measure ([TBD] in the code)

`FLOAT_HEIGHT_M`, `SENSOR_ABOVE_BOTTOM_M`, `COMPANY_ID` (float_logic.py);
`A_BRAKE` and the gains (float_logic.py); `PUMP_IN_IS_FORWARD`, `PUMP_STROKE_S`,
`ULTRASONIC_MIN_MM`, `SOUND_SCALE` (FloatCode.py). **Set `A_BRAKE` below the measured
value.** In simulation, overestimating it lets the float break the surface.

## Files on the float

`/sd/packets.txt` holds the competition packets and `/sd/diag.csv` holds state, depth,
speed, ice range, pump duty and hold time. `/float_state.json` on internal flash holds
the zero and the current segment, which lets a brownout underwater resume instead of
re-zeroing.
