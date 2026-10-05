#!/usr/bin/env python3
"""GNC station keeping - hold horizontal position against a water current.

A PID controller on horizontal position error drives forward/lateral thrust
while ArduSub holds depth and heading (ALT_HOLD). This is the ConOps "current
compensation" function and the Task 1.3 / 2.1 position hold.

Position sources (--source):
  vision  Camera-based (default). The image when the hold engages is the zero
          point; each new frame is compared with it (see vision_position.py).
          This is the source that works on the real vehicle.
  ekf     ArduSub's LOCAL_POSITION_NED. Works in SITL, which simulates a
          position estimate, and on the vehicle only once a DVL (or other
          horizontal position aid) is fitted. IMU + depth alone cannot provide
          it: accelerometer error integrates twice and grows as t^2.

Trackers for --source vision (--tracker):
  features  hold wherever the vehicle was when the hold engaged (any texture)
  target    centre on the Task 1.3 blue square and report the judge's pass
            condition (whole blue square in view, no red visible)

Safety:
  * If the vehicle is already armed (normal dive) this script never arms or
    disarms it; on exit it only hands the sticks back to the pilot.
  * Overrides are released on every exit path: end of run, Ctrl-C, SIGTERM,
    lost tracking, or an exception.
  * SIM-only arming shortcuts are used only when the autopilot is SITL.

Examples:
  # vehicle, down-looking camera ~0.6 m above the coral garden, Task 1.3
  python3 station_keeping.py --video rtsp://192.168.2.2:8554/video_stream__dev_video2 \\
      --camera down --tracker target --range 0.6 --hfov 80 --duration 45
  # SITL, rendered camera (SITL has none), 0.4 m/s current from inject_current.py
  python3 station_keeping.py --synthetic --camera down --range 0.6 --duration 40
  # SITL, simulator position estimate (the original behaviour)
  python3 station_keeping.py --source ekf
"""
from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time

from pymavlink import mavutil

import gnc_link as gl

LIMIT = 400          # max PWM offset from neutral, us
I_LIMIT = 300        # max integral contribution, us (anti-windup)
LOST_HOLD_S = 0.5    # tracking lost longer than this -> thrust neutral
LOST_ABORT_S = 3.0   # tracking lost longer than this -> stop and hand back control
Z_MIN_ERR = 0.03     # height set: only move if the measured height is off by more than this, m
Z_SETTLED_VZ = 0.03  # height set: settled when vertical speed stays below this, m/s
Z_PASSES = 3         # height set: move / re-measure at most this many times
Z_TIMEOUT = 15.0     # height set: give up on a move after this long, s


class AxisPID:
    """u = Kp*e + Kd*de/dt + Ki*integral(e), clamped, with integral anti-windup.

    e is the reference position relative to the vehicle (metres), so de/dt is
    minus the vehicle's velocity toward it: derivative on measurement, no kick
    when the reference is latched.
    """

    def __init__(self, kp: float, kd: float, ki: float, limit: float = LIMIT, i_limit: float = I_LIMIT):
        self.kp, self.kd, self.ki, self.limit, self.i_limit = kp, kd, ki, limit, i_limit
        self.integral = 0.0

    def step(self, e: float, de: float, dt: float) -> float:
        if self.ki > 0:
            bound = self.i_limit / self.ki
            self.integral = gl.clamp(self.integral + e * dt, -bound, bound)
        return gl.clamp(self.kp * e + self.kd * de + self.ki * self.integral, -self.limit, self.limit)


class HeightSetter:
    """Task 1.3 height: measure it with the camera, let ArduSub's depth hold move there.

    The camera gives the height above the coral garden from the square's known
    size. That reading is too intermittent (square partly out of view while
    moving) to close a loop on, so it is used only to pick a target depth, which
    is sent to ArduSub's depth hold (ALT_HOLD accepts a z-only
    SET_POSITION_TARGET_GLOBAL_INT). ArduSub moves there under its own
    speed/accel limits, then the camera re-measures and corrects, so a depth-scale
    error (e.g. wrong BARO_SPEC_GRAV) is removed rather than trusted.
    """

    def __init__(self, m, tel, desired: float):
        self.m, self.tel, self.desired = m, tel, desired
        self.state, self.samples, self.passes = "measure", [], 0
        self.goal = self.t_state = self.settled_since = None

    @property
    def done(self) -> bool:
        return self.state == "done"

    def _finish(self, msg: str):
        print(f"  {msg}")
        self.state = "done"

    def update(self, meas, now: float) -> None:
        if self.done:
            return
        z = self.tel.pos.z if self.tel.pos is not None else None   # EKF depth, the frame ALT_HOLD targets
        if self.t_state is None:
            self.t_state = now
        if self.state == "measure":
            if meas is not None and meas.ok and meas.range_est:
                self.samples.append(meas.range_est)
            if len(self.samples) >= 8:
                h = sorted(self.samples)[len(self.samples) // 2]
                err = h - self.desired                             # + = too high above the square
                if abs(err) <= Z_MIN_ERR:
                    return self._finish(f"height {h:.2f} m (best {self.desired:.2f} m); depth hold keeps it")
                if z is None:
                    return self._finish("height set needs the autopilot's depth estimate (LOCAL_POSITION_NED); "
                                        "set height by hand")
                if self.passes >= Z_PASSES:
                    return self._finish(f"height {h:.2f} m after {self.passes} moves; leaving it there")
                self.goal = z + err
                gl.set_depth_target(self.m, self.goal)
                self.passes += 1
                print(f"  height {h:.2f} m, best {self.desired:.2f} m: depth target {z:.2f} -> {self.goal:.2f} m")
                self.state, self.t_state, self.settled_since = "moving", now, None
            elif now - self.t_state > 3.0:
                self._finish("square not fully in view long enough to measure height; height left to the pilot")
        elif self.state == "moving":
            # settled = vertical speed small for 1.5 s (not "at the goal number": the
            # camera re-measure decides whether the height is right)
            vz = self.tel.pos.vz if self.tel.pos is not None else 0.0
            if now - self.t_state > 2.0 and abs(vz) < Z_SETTLED_VZ:
                self.settled_since = self.settled_since or now
                if now - self.settled_since >= 1.5:
                    self.state, self.t_state, self.samples = "measure", now, []
            else:
                self.settled_since = None
            if now - self.t_state > Z_TIMEOUT:
                self._finish("vehicle did not settle after the depth change; leaving height to the pilot")


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--conn", default=gl.DEFAULT_CONN, help="MAVLink connection (default %(default)s)")
    p.add_argument("--duration", type=float, default=30.0, help="seconds to hold (default %(default)s)")
    p.add_argument("--source", choices=["vision", "ekf"], default="vision")
    p.add_argument("--video", help="camera stream URL or device index (vision source)")
    p.add_argument("--synthetic", action="store_true",
                   help="SITL only: render the camera image from the simulated pose")
    p.add_argument("--camera", choices=["down", "forward"], default="down")
    p.add_argument("--tracker", choices=["features", "target"], default="features")
    p.add_argument("--range", type=float, default=None,
                   help="m: height above floor (down) or stand-off distance (forward). "
                        "Target tracker: the height to hold (default: best Task 1.3 height for the "
                        "camera's field of view). Otherwise default 0.6")
    p.add_argument("--no-alt-hold", action="store_true",
                   help="target tracker: do not move to the best height when the hold engages")
    p.add_argument("--synthetic-floor", type=float,
                   help="SITL test only: place the rendered floor this far below the vehicle "
                        "instead of at --range (to test the height set)")
    p.add_argument("--hfov", type=float, default=80.0, help="camera horizontal field of view, deg")
    p.add_argument("--kp", type=float, default=440.0, help="us per m of error")
    p.add_argument("--kd", type=float, default=230.0, help="us per m/s")
    p.add_argument("--ki", type=float, default=210.0, help="us per m*s")
    p.add_argument("--rate", type=float, default=10.0, help="control rate, Hz")
    p.add_argument("--log", help="write a per-step CSV here")
    p.add_argument("--save-frames", help="SITL/debug: directory to save a frame every second")
    a = p.parse_args(argv)
    if a.source == "vision" and not (a.video or a.synthetic):
        p.error("--source vision needs --video URL (vehicle) or --synthetic (SITL)")
    return a


class Telemetry:
    """Latest ATTITUDE / LOCAL_POSITION_NED / depth, pumped without blocking."""

    def __init__(self, m):
        self.m = m
        self.att = None
        self.pos = None
        self.depth = None    # m, positive down, from the pressure sensor (VFR_HUD.alt)
        self.truth = None    # SITL only: true (n, e, d, roll, pitch, yaw) from SIM_STATE
        self._origin = None

    def pump(self):
        while True:
            msg = self.m.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD", "SIM_STATE"],
                                    blocking=False)
            if msg is None:
                return
            kind = msg.get_type()
            if kind == "ATTITUDE":
                self.att = msg
            elif kind == "VFR_HUD":
                self.depth = -msg.alt
            elif kind == "SIM_STATE":
                # lat/lon floats are too coarse here (float32 degE7: ~0.4 m steps); use the
                # integer fields (~1.1 cm) and smooth them with the true velocity
                lat, lon = msg.lat_int * 1e-7, msg.lon_int * 1e-7
                if self._origin is None:
                    self._origin = (lat, lon)
                    self._ne, self._t = [0.0, 0.0], time.monotonic()
                n_q = (lat - self._origin[0]) * 111320.0
                e_q = (lon - self._origin[1]) * 111320.0 * math.cos(math.radians(lat))
                now = time.monotonic()
                dt, self._t = now - self._t, now
                self._ne[0] += msg.vn * dt + 0.05 * (n_q - self._ne[0])
                self._ne[1] += msg.ve * dt + 0.05 * (e_q - self._ne[1])
                self.truth = (self._ne[0], self._ne[1], -msg.alt, msg.roll, msg.pitch, msg.yaw)
            else:
                self.pos = msg

    def wait(self, need_pos: bool, timeout: float = 10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump()
            if self.att is not None and (self.pos is not None or not need_pos):
                return True
            time.sleep(0.05)
        return False


def main(argv=None) -> int:
    a = parse_args(argv)
    # Ctrl-C and SIGTERM both end the hold through the same cleanup path. SIGINT is
    # installed explicitly because a launcher can start us with it ignored.
    def _stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    m = gl.connect(a.conn)
    sim = gl.is_sitl(m)
    if a.synthetic and not sim:
        print("--synthetic is for SITL only; on the vehicle use --video")
        return 2
    tel = Telemetry(m)
    need_pos = a.source == "ekf" or a.synthetic
    gl.request_streams(m, {mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 20,
                           mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 20,
                           mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 10})
    if not tel.wait(need_pos):
        print("no attitude" + (" / position estimate" if need_pos else "") + " from the autopilot; aborting.")
        if a.source == "ekf" and not sim:
            print("On the vehicle there is no horizontal position estimate without a DVL - use --source vision.")
        return 1

    # ---- position source -------------------------------------------------------
    vp = cam = tracker = source = synth = None
    height = None
    if a.source == "vision":
        import vision_position as vp
        tracker = vp.TargetTracker() if a.tracker == "target" else vp.FeatureTracker()
        if a.synthetic:
            cam = vp.CameraModel(640, 480, a.hfov)
            if a.range is None:
                a.range = vp.task13_best_range(cam)[0] if a.tracker == "target" else 0.6
            tex = vp.make_texture()
            # floor (or wall) placed so the reference view is at --range
            gl.request_streams(m, {mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE: 20})
            t0 = time.time()
            while tel.truth is None and time.time() - t0 < 5:
                tel.pump()
                time.sleep(0.05)
            if tel.truth is None:
                print("no SIM_STATE from SITL; cannot render the synthetic camera")
                return 1
            n0_, e0_, d0_ = tel.truth[:3]
            floor = a.synthetic_floor or a.range
            # floor/wall placed relative to the true pose; the coral-garden square sits at
            # the texture centre, i.e. at the true position where the sim started
            synth = vp.SyntheticCamera(cam, a.camera, tex, plane_z=d0_ + floor,
                                       plane_n=n0_ + floor)
        else:
            source = vp.FrameSource(a.video)
            t0 = time.time()
            while source.latest()[0] is None and time.time() - t0 < 10:
                time.sleep(0.1)
            if source.latest()[0] is None:
                print(f"no video from {a.video}; aborting.")
                return 1

    def grab():
        """Current frame (resized for processing) and its capture time."""
        if synth is not None:
            tel.pump()
            # render from the simulator's TRUE pose; the controller never sees it
            return synth.render(*tel.truth), time.monotonic()
        frame, t = source.latest()
        return vp.resize_for_processing(frame), t

    dz = gl.rc_deadzones(m)
    was_armed = bool(m.wait_heartbeat(timeout=3).base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    armed_here = False
    log = None
    try:
        gl.set_mode(m, gl.MODE_ALT_HOLD)
        if not was_armed:
            if not gl.arm(m, timeout=5.0):
                print("vehicle did not arm within 5 s:", "; ".join(gl.prearm_messages(m)) or "no reason given")
                return 1
            armed_here = True

        tel.pump()
        att0 = tel.att
        if a.source == "ekf":
            n0, e0 = tel.pos.x, tel.pos.y
            print(f"holding N={n0:.2f} E={e0:.2f} for {a.duration:.0f}s "
                  f"(Kp={a.kp:.0f} Kd={a.kd:.0f} Ki={a.ki:.0f}, source=ekf)")
        else:
            img, _ = grab()
            h, w = img.shape[:2]
            cam = vp.CameraModel(w, h, a.hfov)
            best, margin = vp.task13_best_range(cam)
            if a.range is None:
                a.range = best if a.tracker == "target" else 0.6
            alt_hold = a.tracker == "target" and a.camera == "down" and not a.no_alt_hold
            if a.tracker == "target":
                tracker.f_px = cam.f_px
                print(f"Task 1.3: best height {best:.2f} m above the square, allowed drift +/-{margin*100:.0f} cm "
                      f"({w}x{h}, {a.hfov:.0f} deg); holding height {a.range:.2f} m" if alt_hold else
                      f"Task 1.3: best height {best:.2f} m, allowed drift +/-{margin*100:.0f} cm (height hold off)")
            if not tracker.set_reference(img):
                print("could not lock a reference (too little texture / target not in view); aborting.")
                return 1
            if alt_hold:
                height = HeightSetter(m, tel, a.range)
            print(f"reference locked ({a.tracker}, {a.camera} camera, range {a.range:.2f} m); "
                  f"holding for {a.duration:.0f}s")

        if a.log:
            log = csv.writer(open(a.log, "w", newline=""))
            log.writerow(["t", "valid", "e_fwd", "e_right", "u_fwd", "u_lat", "inliers",
                          "blue_full", "red_visible", "range_est", "depth", "height_state", "true_n", "true_e", "true_d"])
        pid_f, pid_l = AxisPID(a.kp, a.kd, a.ki), AxisPID(a.kp, a.kd, a.ki)
        period = 1.0 / a.rate
        t_start = time.monotonic()
        t_prev = t_start
        prev_e = None
        de_f = de_l = 0.0
        lost_since = None
        last_print = 0.0
        max_err = 0.0
        task_ok_since = None
        best_task_run = 0.0
        next_save = 0.0
        aborted = False

        while time.monotonic() - t_start < a.duration:
            loop_t = time.monotonic()
            tel.pump()
            dt = max(1e-3, loop_t - t_prev)
            t_prev = loop_t
            att = tel.att
            valid, note, meas = True, "", None

            if a.source == "ekf":
                p = tel.pos
                e_n, e_e = n0 - p.x, e0 - p.y
                c, s = math.cos(att.yaw), math.sin(att.yaw)
                e_f, e_r = e_n * c + e_e * s, -e_n * s + e_e * c
                # de/dt = -(vehicle velocity in body frame)
                d_f = -(p.vx * c + p.vy * s)
                d_r = -(-p.vx * s + p.vy * c)
            else:
                img, _ = grab()
                if a.save_frames and loop_t - t_start >= next_save:
                    import os
                    import cv2
                    os.makedirs(a.save_frames, exist_ok=True)
                    cv2.imwrite(f"{a.save_frames}/frame_{int(loop_t - t_start):03d}.jpg", img)
                    next_save += 1.0
                meas = tracker.measure(img)
                valid, note = meas.ok, meas.note
                if valid:
                    rng_now = meas.range_est or None      # target tracker: absolute, from the square's size
                    e_f, e_r = vp.body_error(meas, cam, a.camera, a.range,
                                             att.roll - att0.roll, att.pitch - att0.pitch,
                                             vp.wrap_pi(att.yaw - att0.yaw), range_now=rng_now)
                    if prev_e is not None:
                        alpha = 0.4   # low-pass the finite-difference velocity
                        de_f = alpha * (e_f - prev_e[0]) / dt + (1 - alpha) * de_f
                        de_l = alpha * (e_r - prev_e[1]) / dt + (1 - alpha) * de_l
                    prev_e = (e_f, e_r)
                    d_f, d_r = de_f, de_l

            if height is not None:
                height.update(meas, loop_t)
            ch = [gl.NEUTRAL] * 8
            if valid:
                lost_since = None
                u_f = pid_f.step(e_f, d_f, dt)
                u_l = pid_l.step(e_r, d_r, dt)
                ch[gl.CH_FORWARD] = gl.axis_pwm(u_f, dz[gl.CH_FORWARD])
                ch[gl.CH_LATERAL] = gl.axis_pwm(u_l, dz[gl.CH_LATERAL])
                err = math.hypot(e_f, e_r)
                max_err = max(max_err, err)
            else:
                u_f = u_l = 0.0
                lost_since = lost_since or loop_t
                prev_e = None
                if loop_t - lost_since > LOST_ABORT_S:
                    print(f"tracking lost for {LOST_ABORT_S:.0f} s ({note}); stopping and handing back control.")
                    aborted = True
                    break
            gl.send_override(m, ch)

            # Task 1.3 judge condition (target tracker only)
            if meas is not None and a.tracker == "target":
                task_ok = meas.ok and meas.blue_fully_visible and not meas.red_visible
                if task_ok:
                    task_ok_since = task_ok_since or loop_t
                    best_task_run = max(best_task_run, loop_t - task_ok_since)
                else:
                    task_ok_since = None

            if log:
                tn = te = td = ""
                if tel.truth is not None:
                    tn, te, td = (round(v, 3) for v in tel.truth[:3])
                elif tel.pos is not None and a.source == "ekf":
                    tn, te, td = round(tel.pos.x, 3), round(tel.pos.y, 3), round(tel.pos.z, 3)
                log.writerow([round(loop_t - t_start, 3), int(valid),
                              round(e_f, 4) if valid else "", round(e_r, 4) if valid else "",
                              round(u_f, 1), round(u_l, 1), meas.inliers if meas else "",
                              int(meas.blue_fully_visible) if meas else "", int(meas.red_visible) if meas else "",
                              round(meas.range_est, 3) if meas and meas.range_est else "",
                              round(tel.depth, 3) if tel.depth is not None else "", height.state if height else "",
                              tn, te, td])
            if loop_t - last_print > 2:
                last_print = loop_t
                if valid:
                    extra = ""
                    if a.tracker == "target" and meas is not None:
                        extra = f"  blue-in-view={meas.blue_fully_visible} red-visible={meas.red_visible}"
                    print(f"  t={loop_t - t_start:4.1f}s  error={math.hypot(e_f, e_r):4.2f} m "
                          f"(fwd {e_f:+.2f}, right {e_r:+.2f}){extra}")
                else:
                    print(f"  t={loop_t - t_start:4.1f}s  no valid measurement ({note}); thrust neutral")
            time.sleep(max(0.0, period - (time.monotonic() - loop_t)))

        print(f"\n{'Aborted' if aborted else 'Done'}. Max position error held: {max_err:.2f} m")
        if a.tracker == "target" and a.source == "vision":
            print(f"Longest run meeting the Task 1.3 condition: {best_task_run:.1f} s (need 30 s)")
        return 3 if aborted else 0
    except KeyboardInterrupt:
        print("\ninterrupted - handing back control")
        return 130
    finally:
        gl.release_overrides(m)
        if armed_here:
            gl.disarm(m)
        if source is not None:
            source.close()


if __name__ == "__main__":
    sys.exit(main())
