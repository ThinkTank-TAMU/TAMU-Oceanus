# Float items before CoDR (message to Arnav)

Not sent. Copy the block below into Discord / GroupMe.

---

Hey Arnav, I've finished the GNC-ROV slides for CoDR (43–56: requirements, ConOps, architecture, thrusters, 5 trade studies, ICD, station keeping, throttle cap, code, next steps). Here's what's still open on the float side, from the review comments on the deck and the 2027 rules. We want everything done by Wed 11/18.

**1. Profile depths (flagged CRITICAL in review).** The deck has three different versions. The 2027 MATE Floats preview mission (PIONEER) settles it:
- Start at the surface and send one data packet before descending.
- Descend so the *top* of the float is at 2.0 m (±0.33 m) and hold for 30 s.
- Descend to 4.0 m (top of float), then ascend to the surface.
- On the ascent, record depth, temperature and light every 0.5 m from 4.0 m to 0.0 m.
- Do it twice (two profiles). Touching the bottom is a penalty.

GNC-FL-01 already matches this. SR-FL-02/03 (2.5 m then 0.4 m, the 2026 rules) and FNP's slides 50/52 (2 m and 4 m as separate profiles) need to change. Worth a quick sync with Cody.

**2. "No depth-control code" (review comment).** There is one: `Float/FloatCode/float_logic.py` has the pump control law, hold timer and profile state machine, and 35 desk tests pass (`python3 -m unittest discover -s tests`). It's still set to the 2026 depths (2.5 m / 0.4 m), so the targets need updating to the above, but it deserves a slide.

**3. Packet format (GNC-FL-04).** Right now the log is ultrasonic distance only. Each packet needs company number, time, pressure/depth, temperature (°C) and light, with units, e.g. `EX01 1:51:42 UTC 9.8 kpa 1.00 meters 23.6°C 0.546 V`.

**4. Graphs (GNC-FL-06).** `plot_depth.py` makes depth vs time, but scoring is temperature vs depth and light vs depth (5 points each).

**5. Sensor trade study (Emily asked for one).** The temperature-sensor slide needs to be a real trade study in the team template: criteria, weights, scores and a final score per option, like the GNC ones on slides 47–51. The "GNC Trade Studies" sheet has the template (copy a tab).

**6. Slide fixes from the review comments:**
- "Boat controller" → float controller.
- DS18B20 is 1-Wire and TSL2591 is I²C, not ADC.
- Receiver/C3 file names differ between slides (BLE_Receiver.py / C3.py vs Bluetooth_Receiver.py / main_C3.py).
- PT1000 is an RTD, not an NTC.
- DS18B20 accuracy is ±0.4 °C on one slide and ±0.5 °C on another.
- Future Implementations needs owners, dates, a pool test of the 2.0 m hold, and a pass/fail C3 test (e.g. receive ≥ 10 packets and make both graphs within the 15-min demo).
- Say what the sonar is for. The 2027 preview mission doesn't mention ice or overhead obstacles, so it may not be needed.

**7. Repo:** `main.py` imports `UltraSonic_Sensor.py` and `Bluetooth_Sender.py`, which aren't committed. And the placeholder files `Float/test` and `FloatCode/Working end to end/W` can be deleted.

Let me know if you want me to help with any of it.
