"""
Pure float logic: no `machine`, no hardware. Runs unchanged on the Pico
(MicroPython) and on a laptop (CPython), so it can be desk-tested with
tests/test_float_logic.py.

Units: depth in metres, positive down, measured at the pressure sensor.
Pressure in pascals. Time in seconds. Pump command in percent duty,
positive = take on water (sink), negative = expel water (rise).

Requirements traced here: GNC-FL-04 (holds), GNC-FL-05 (ice), GNC Calc
C5 (ice range), C6 (log period), C7 (SG 1.025).
"""

G = 9.80665
SG = 1.025                      # EGADS ice tank (GNC Calc C7)
RHO = 1000.0 * SG               # kg/m^3

# --- Float geometry [TBD: measure on the built float] -------------------
FLOAT_HEIGHT_M = 0.50           # bottom of float to top of float
SENSOR_ABOVE_BOTTOM_M = 0.00    # pressure port height above float bottom; tell the judge

# --- Profile (GNC-FL-04, Task 4.1) ---------------------------------------
DEEP_BOTTOM_M = 2.50            # bottom of float at 2.5 m
SHALLOW_TOP_M = 0.40            # top of float at 40 cm
BAND_M = 0.33                   # +/- band; leaving it restarts the hold
GUARD_M = 0.05                  # hold timer only starts inside BAND - GUARD
HOLD_S = 30.0
HOLD_MARGIN_S = 3.0             # stay a little past 30 s so the record clearly spans it
PROFILES = 2

# --- Ice (GNC-FL-05, Calc C5) --------------------------------------------
ICE_DETECT_M = 0.75             # anything this close overhead counts as ice
ICE_MIN_CLEARANCE_M = 0.20      # closer than this: sink at full rate

# --- Logging (Calc C6) -----------------------------------------------------
LOG_PERIOD_S = 1.0
COMPANY_ID = "EX01"             # [TBD: MATE company number]

# --- Buoyancy engine control [TBD: tune in the pool] ----------------------
K_POS = 0.5                     # desired speed (m/s) per metre of depth error
V_MAX_FAST = 0.20               # m/s, transit
V_MAX_SLOW = 0.05               # m/s, rising with ice in view (or unknown)
K_VEL = 400.0                   # % duty per m/s of speed error
MIN_DUTY = 11                   # pump stalls below this; treat as off
DEADBAND_M = 0.05
DEADBAND_V = 0.02
A_BRAKE = 0.008                 # m/s^2 the pump can decelerate the float by; keep
                                # below the bench value [TBD: measure]


def sensor_targets(float_height=FLOAT_HEIGHT_M, sensor_above_bottom=SENSOR_ABOVE_BOTTOM_M):
    """Depths the *sensor* must hold so the float bottom is at 2.5 m and
    the float top at 0.40 m. These are the ranges told to the judge."""
    deep = DEEP_BOTTOM_M - sensor_above_bottom
    shallow = SHALLOW_TOP_M + (float_height - sensor_above_bottom)
    return deep, shallow


def depth_from_pressure(pressure_pa, zero_pa, rho=RHO):
    """Depth below the surface zero. Negative values are clamped to 0."""
    d = (pressure_pa - zero_pa) / (rho * G)
    return d if d > 0 else 0.0


def surface_zero(samples_pa):
    """Mean of the power-up readings, taken in air before deployment."""
    if not samples_pa:
        raise ValueError("no pressure samples for surface zero")
    return sum(samples_pa) / len(samples_pa)


def clock_text(seconds):
    """Seconds -> H:MM:SS (time of day if UTC-synced, else since boot)."""
    s = int(seconds) % 86400
    return "{}:{:02d}:{:02d}".format(s // 3600, (s // 60) % 60, s % 60)


def format_packet(seconds, gauge_pa, depth_m, utc=True, company=COMPANY_ID):
    """Competition packet, e.g. [EX01 1:51:42 UTC 9.8 kpa 1.00 meters].
    Without a UTC sync the clock is time since boot and is labelled BOOT."""
    return "[{} {} {} {:.1f} kpa {:.2f} meters]".format(
        company, clock_text(seconds), "UTC" if utc else "BOOT",
        gauge_pa / 1000.0, depth_m)


def parse_packet(line):
    """Inverse of format_packet, for the receiver and grapher.
    Returns dict or None if the line is not a packet."""
    line = line.strip()
    if not (line.startswith("[") and line.endswith("]")):
        return None
    parts = line[1:-1].split()
    if len(parts) != 7 or parts[4] != "kpa" or parts[6] != "meters":
        return None
    try:
        h, m, s = [int(x) for x in parts[1].split(":")]
        return {"company": parts[0], "seconds": h * 3600 + m * 60 + s,
                "clock": parts[2], "kpa": float(parts[3]), "depth_m": float(parts[5])}
    except ValueError:
        return None


def parse_ultrasonic_frame(frame):
    """4-byte frame 0xFF, high, low, checksum -> distance in mm, or None."""
    if frame is None or len(frame) < 4 or frame[0] != 0xFF:
        return None
    if (frame[0] + frame[1] + frame[2]) & 0xFF != frame[3]:
        return None
    return (frame[1] << 8) | frame[2]


def ice_status(range_m):
    """'unknown' (no reading: assume ice in view), 'too_close', 'detected', 'clear'."""
    if range_m is None:
        return "unknown"
    if range_m <= ICE_MIN_CLEARANCE_M:
        return "too_close"
    if range_m <= ICE_DETECT_M:
        return "detected"
    return "clear"


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def pump_command(target_m, depth_m, vel_mps, ice="clear"):
    """Outer loop: depth error -> desired vertical speed (capped).
    Inner loop: speed error -> pump duty. The pump changes buoyancy, and
    buoyancy sets speed, so the pump acts on speed, not position."""
    if ice == "too_close":
        return 100.0
    err = target_m - depth_m
    if abs(err) < DEADBAND_M and abs(vel_mps) < DEADBAND_V:
        return 0.0
    rise_cap = V_MAX_SLOW if ice in ("detected", "unknown") else V_MAX_FAST
    # Never ask for more speed than the pump can take off before the target.
    v_stop = (2.0 * A_BRAKE * abs(err)) ** 0.5
    v_des = clamp(K_POS * err, -min(rise_cap, v_stop), min(V_MAX_FAST, v_stop))
    u = clamp(K_VEL * (v_des - vel_mps), -100.0, 100.0)
    return 0.0 if abs(u) < MIN_DUTY else u


class VelocityEstimator:
    """Low-pass filtered d(depth)/dt. Positive = descending."""

    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.v = 0.0
        self._last = None

    def update(self, t, depth):
        if self._last is not None:
            dt = t - self._last[0]
            if dt > 0:
                raw = (depth - self._last[1]) / dt
                self.v += self.alpha * (raw - self.v)
        self._last = (t, depth)
        return self.v


class StrokeTracker:
    """Dead-reckons how full the buoyancy reservoir is, in seconds of
    pumping at 100 % duty, so the pump is not driven past either end.
    stroke_s=None disables the limit [TBD: time a full stroke on the bench]."""

    def __init__(self, stroke_s=None, position_s=0.0):
        self.stroke_s = stroke_s
        self.pos = position_s       # 0 = empty (most buoyant)

    def limit(self, cmd):
        if self.stroke_s is None:
            return cmd
        if cmd > 0 and self.pos >= self.stroke_s:
            return 0.0
        if cmd < 0 and self.pos <= 0.0:
            return 0.0
        return cmd

    def update(self, cmd, dt):
        self.pos += cmd / 100.0 * dt
        if self.stroke_s is not None:
            self.pos = clamp(self.pos, 0.0, self.stroke_s)


class HoldTimer:
    """30 s hold with the competition restart rule: leaving the +/- band
    at any point restarts the full 30 s."""

    def __init__(self, target_m, band=BAND_M, guard=GUARD_M,
                 hold_s=HOLD_S + HOLD_MARGIN_S):
        self.target = target_m
        self.band = band
        self.guard = guard
        self.hold_s = hold_s
        self.since = None
        self.restarts = 0

    def update(self, t, depth):
        err = abs(depth - self.target)
        if err > self.band:
            if self.since is not None:
                self.restarts += 1
            self.since = None
        elif self.since is None and err <= self.band - self.guard:
            self.since = t
        return self.elapsed(t)

    def elapsed(self, t):
        return 0.0 if self.since is None else t - self.since

    def done(self, t):
        return self.since is not None and t - self.since >= self.hold_s


def segment_plan(profiles=PROFILES, targets=None):
    """[('DEEP1', 2.5), ('SHALLOW1', x), ('DEEP2', ...), ...] in sensor depth."""
    deep, shallow = targets or sensor_targets()
    plan = []
    for n in range(1, profiles + 1):
        plan.append(("DEEP%d" % n, deep))
        plan.append(("SHALLOW%d" % n, shallow))
    return plan


class Mission:
    """Profile state machine.

    PREDIVE -> DEEP1 -> SHALLOW1 -> DEEP2 -> SHALLOW2 -> END

    Each DEEP/SHALLOW segment drives toward its target and is complete
    once the HoldTimer has run its full hold without leaving the band.
    END rises for recovery, still obeying the ice backstop."""

    def __init__(self, plan=None):
        self.plan = plan or segment_plan()
        self.seg = -1               # -1 = PREDIVE, len(plan) = END
        self.timer = None

    @property
    def state(self):
        if self.seg < 0:
            return "PREDIVE"
        if self.seg >= len(self.plan):
            return "END"
        return self.plan[self.seg][0]

    @property
    def target(self):
        if 0 <= self.seg < len(self.plan):
            return self.plan[self.seg][1]
        return None

    def start(self, seg=0):
        self.seg = seg
        self.timer = HoldTimer(self.target) if self.target is not None else None

    def update(self, t, depth, vel, ice_range_m):
        """Returns (pump_cmd, event). event is a segment name when one
        completes, else None."""
        ice = ice_status(ice_range_m)
        if self.seg < 0:
            return 0.0, None
        if self.seg >= len(self.plan):
            # Rise for recovery; the ice backstop still applies.
            if ice == "too_close":
                return 100.0, None
            return pump_command(0.0, depth, vel, ice), None
        self.timer.update(t, depth)
        if self.timer.done(t):
            done = self.state
            self.start(self.seg + 1)
            return 0.0, done
        return pump_command(self.target, depth, vel, ice), None

    def to_dict(self):
        return {"seg": self.seg}


def resume_decision(saved, pressure_pa, threshold_m=0.10):
    """After a reboot: resume the saved mission only if the float is
    clearly underwater (a brownout mid-profile). At the surface, start
    fresh and take a new zero. Returns (zero_pa or None, seg or None)."""
    if not saved or saved.get("seg", -1) < 0 or "zero_pa" not in saved:
        return None, None
    zero = saved["zero_pa"]
    if depth_from_pressure(pressure_pa, zero) > threshold_m:
        return zero, saved["seg"]
    return None, None
