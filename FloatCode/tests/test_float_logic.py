"""Desk tests for float_logic (no MicroPython, no hardware).

    cd FloatCode && python3 -m unittest discover -s tests -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import float_logic as fl  # noqa: E402


class DepthTests(unittest.TestCase):
    def test_one_metre_at_sg_1025(self):
        zero = 101325.0
        p = zero + 1025.0 * fl.G * 1.0
        self.assertAlmostEqual(fl.depth_from_pressure(p, zero), 1.0, places=6)

    def test_fresh_water_would_over_read(self):
        # Calc C7: density 1000 over-reads by ~2.5 %, ~6 cm at 2.5 m.
        zero = 100000.0
        p = zero + fl.RHO * fl.G * 2.5
        fresh = (p - zero) / (1000.0 * fl.G)
        self.assertAlmostEqual(fresh - 2.5, 0.0625, places=3)

    def test_above_zero_clamps(self):
        self.assertEqual(fl.depth_from_pressure(100000.0, 100100.0), 0.0)

    def test_surface_zero_is_mean(self):
        self.assertEqual(fl.surface_zero([100000.0, 100010.0]), 100005.0)
        with self.assertRaises(ValueError):
            fl.surface_zero([])

    def test_sensor_targets_with_offset(self):
        # Sensor 25 cm above the bottom of a 50 cm float.
        deep, shallow = fl.sensor_targets(0.50, 0.25)
        self.assertAlmostEqual(deep, 2.25)      # manual's worked example
        self.assertAlmostEqual(shallow, 0.65)   # top at 0.40 -> sensor 0.25 lower

    def test_sensor_at_bottom(self):
        deep, shallow = fl.sensor_targets(0.50, 0.0)
        self.assertAlmostEqual(deep, 2.50)
        self.assertAlmostEqual(shallow, 0.90)


class PacketTests(unittest.TestCase):
    def test_matches_competition_example(self):
        t = 1 * 3600 + 51 * 60 + 42
        pkt = fl.format_packet(t, 9800.0, 1.0)
        self.assertEqual(pkt, "[EX01 1:51:42 UTC 9.8 kpa 1.00 meters]")

    def test_unsynced_clock_is_labelled(self):
        self.assertEqual(fl.format_packet(65, 0.0, 0.0, utc=False),
                         "[EX01 0:01:05 BOOT 0.0 kpa 0.00 meters]")

    def test_time_of_day_wraps(self):
        self.assertEqual(fl.clock_text(86400 + 5), "0:00:05")

    def test_round_trip(self):
        pkt = fl.format_packet(5000, 25130.0, 2.5)
        d = fl.parse_packet(pkt)
        self.assertEqual(d["seconds"], 5000)
        self.assertEqual(d["kpa"], 25.1)
        self.assertEqual(d["depth_m"], 2.5)
        self.assertEqual(d["clock"], "UTC")

    def test_parse_rejects_other_lines(self):
        for line in ("EOF", "BEGIN LOG", "[EX01 1679123456 1.23m 12345ADC]", "[a b c]"):
            self.assertIsNone(fl.parse_packet(line))


class UltrasonicTests(unittest.TestCase):
    def frame(self, mm):
        hi, lo = mm >> 8, mm & 0xFF
        return bytes([0xFF, hi, lo, (0xFF + hi + lo) & 0xFF])

    def test_valid_frame(self):
        self.assertEqual(fl.parse_ultrasonic_frame(self.frame(750)), 750)

    def test_bad_checksum_and_header(self):
        f = bytearray(self.frame(750))
        f[3] ^= 1
        self.assertIsNone(fl.parse_ultrasonic_frame(bytes(f)))
        self.assertIsNone(fl.parse_ultrasonic_frame(b"\x00\x02\xee\xf0"))
        self.assertIsNone(fl.parse_ultrasonic_frame(b"\xff\x02"))
        self.assertIsNone(fl.parse_ultrasonic_frame(None))

    def test_ice_status(self):
        self.assertEqual(fl.ice_status(None), "unknown")
        self.assertEqual(fl.ice_status(0.10), "too_close")
        self.assertEqual(fl.ice_status(0.40), "detected")
        self.assertEqual(fl.ice_status(0.75), "detected")   # GNC-FL-05: at 0.75 m, not 30 cm
        self.assertEqual(fl.ice_status(0.90), "clear")


class PumpTests(unittest.TestCase):
    def test_sinks_when_shallow(self):
        self.assertGreater(fl.pump_command(2.5, 1.0, 0.0), 0)

    def test_rises_when_deep(self):
        self.assertLess(fl.pump_command(0.9, 2.5, 0.0), 0)

    def test_deadband(self):
        self.assertEqual(fl.pump_command(2.5, 2.48, 0.0), 0.0)

    def test_brakes_when_overshooting(self):
        # At target but descending fast: empty water to slow down.
        self.assertLess(fl.pump_command(2.5, 2.5, 0.15), 0)

    def test_ice_slows_the_rise(self):
        free = fl.pump_command(0.9, 2.5, -0.10, "clear")
        iced = fl.pump_command(0.9, 2.5, -0.10, "detected")
        self.assertLess(free, 0)        # still accelerating up
        self.assertGreater(iced, 0)     # already faster than V_MAX_SLOW: brake

    def test_no_reading_treated_as_ice(self):
        self.assertEqual(fl.pump_command(0.9, 2.5, -0.10, "unknown"),
                         fl.pump_command(0.9, 2.5, -0.10, "detected"))

    def test_too_close_sinks_full(self):
        self.assertEqual(fl.pump_command(0.9, 0.9, 0.0, "too_close"), 100.0)

    def test_min_duty(self):
        u = fl.pump_command(2.5, 2.5, 0.0 + fl.DEADBAND_V + 0.001)
        self.assertTrue(u == 0.0 or abs(u) >= fl.MIN_DUTY)

    def test_stroke_limits(self):
        s = fl.StrokeTracker(stroke_s=10.0)
        self.assertEqual(s.limit(-50), 0.0)     # already empty
        s.update(100, 10.0)
        self.assertEqual(s.pos, 10.0)
        self.assertEqual(s.limit(50), 0.0)      # full
        self.assertEqual(s.limit(-50), -50)
        self.assertEqual(fl.StrokeTracker().limit(80), 80)   # no limit configured


class HoldTimerTests(unittest.TestCase):
    def test_needs_full_hold(self):
        h = fl.HoldTimer(2.5, hold_s=30.0)
        for t in range(0, 30):
            h.update(t, 2.5)
            self.assertFalse(h.done(t))
        h.update(30, 2.5)
        self.assertTrue(h.done(30))

    def test_leaving_band_restarts_full_30s(self):
        # Manual: 20 s in band, out, back -> a whole new 30 s.
        h = fl.HoldTimer(2.5, hold_s=30.0)
        for t in range(0, 21):
            h.update(t, 2.5)
        h.update(21, 2.5 + 0.34)
        self.assertEqual(h.elapsed(21), 0.0)
        self.assertEqual(h.restarts, 1)
        for t in range(22, 51):
            h.update(t, 2.5)
            self.assertFalse(h.done(t))
        h.update(52, 2.5)
        self.assertTrue(h.done(52))

    def test_guard_band(self):
        h = fl.HoldTimer(2.5, band=0.33, guard=0.05, hold_s=30.0)
        h.update(0, 2.5 + 0.30)         # inside band, outside guard: does not start
        self.assertEqual(h.elapsed(0), 0.0)
        h.update(1, 2.5 + 0.20)         # starts
        h.update(5, 2.5 + 0.31)         # drifts to the edge but stays in band: keeps counting
        self.assertEqual(h.elapsed(5), 4.0)

    def test_seven_packets_fit(self):
        # Calc C6: at 1 s logging a 33 s hold yields >= 7 sequential in-band packets.
        packets_in_hold = int((fl.HOLD_S + fl.HOLD_MARGIN_S) / fl.LOG_PERIOD_S) + 1
        self.assertGreaterEqual(packets_in_hold, 7)


class MissionTests(unittest.TestCase):
    def test_plan_is_two_profiles(self):
        names = [n for n, _ in fl.segment_plan()]
        self.assertEqual(names, ["DEEP1", "SHALLOW1", "DEEP2", "SHALLOW2"])

    def test_predive_does_nothing(self):
        m = fl.Mission()
        self.assertEqual(m.state, "PREDIVE")
        self.assertEqual(m.update(0, 0, 0, None), (0.0, None))

    def test_full_mission_with_ideal_float(self):
        """Teleport the float to each target: segments must complete in order."""
        m = fl.Mission()
        m.start()
        t, events = 0.0, []
        while m.state != "END" and t < 1000:
            cmd, ev = m.update(t, m.target, 0.0, 1.0)
            if ev:
                events.append((ev, t))
            t += 0.25
        self.assertEqual([e for e, _ in events], ["DEEP1", "SHALLOW1", "DEEP2", "SHALLOW2"])
        for _, at in events:
            self.assertGreaterEqual(at, 0)
        self.assertEqual(m.state, "END")

    def test_end_rises_but_respects_ice(self):
        m = fl.Mission()
        m.start(len(m.plan))
        self.assertEqual(m.state, "END")
        self.assertLess(m.update(0, 1.0, 0.0, 2.0)[0], 0)
        self.assertEqual(m.update(0, 1.0, 0.0, 0.1)[0], 100.0)

    def test_simulated_profile(self):
        """Crude float model: pump changes reservoir volume, net buoyancy
        sets a drag-limited speed. The controller must finish both profiles
        without breaking the surface (sensor depth stays > float height)."""
        m = fl.Mission()
        m.start()
        dt, t = 0.25, 0.0
        depth, vol = 0.9, 0.0           # vol: -1 (max buoyant) .. +1 (max heavy)
        vel_est = fl.VelocityEstimator()
        min_after_start = 99.0
        holds = []
        while m.state != "END" and t < 1200:
            v = vel_est.update(t, depth)
            ice_range = max(depth - fl.FLOAT_HEIGHT_M, 0.0) + 0.02   # ice at the waterline
            cmd, ev = m.update(t, depth, v, ice_range)
            if ev:
                holds.append(ev)
            vol = fl.clamp(vol + cmd / 100.0 * dt * 0.05, -1.0, 1.0)
            speed = 0.25 * vol           # m/s, positive down
            depth = max(depth + speed * dt, 0.0)
            if holds:
                min_after_start = min(min_after_start, depth)
            t += dt
        self.assertEqual(holds, ["DEEP1", "SHALLOW1", "DEEP2", "SHALLOW2"])
        self.assertGreater(min_after_start - fl.FLOAT_HEIGHT_M, 0.0)   # top never surfaced


class ResumeTests(unittest.TestCase):
    def test_brownout_underwater_resumes(self):
        zero = 100000.0
        p = zero + fl.RHO * fl.G * 2.0
        self.assertEqual(fl.resume_decision({"zero_pa": zero, "seg": 2}, p), (zero, 2))

    def test_at_surface_starts_fresh(self):
        self.assertEqual(fl.resume_decision({"zero_pa": 100000.0, "seg": 2}, 100050.0), (None, None))

    def test_predive_or_missing_state(self):
        self.assertEqual(fl.resume_decision(None, 120000.0), (None, None))
        self.assertEqual(fl.resume_decision({"zero_pa": 1e5, "seg": -1}, 120000.0), (None, None))


if __name__ == "__main__":
    unittest.main()
