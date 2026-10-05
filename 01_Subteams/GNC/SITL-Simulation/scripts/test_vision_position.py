#!/usr/bin/env python3
"""Offline checks for vision_position.py - no SITL or camera needed.

Renders the synthetic camera at known poses and checks that the measured
position of the reference (in the body frame) matches the true one, including
with heading change and with roll/pitch (the IMU tilt compensation).

    python3 test_vision_position.py      (or: python3 -m pytest test_vision_position.py)
"""
import math

import numpy as np

import vision_position as vp

CAM = vp.CameraModel(640, 480, 80.0)
TEX = vp.make_texture()
TOL = 0.03   # m


def truth_body(dn, de, yaw):
    """Reference position relative to the vehicle, body frame (fwd, right)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return dn * c + de * s, -dn * s + de * c


def run_case(mount, rng, move, att=(0.0, 0.0, 0.0), tracker=None, compensate=True):
    d0 = 1.0
    sc = vp.SyntheticCamera(CAM, mount, TEX, plane_z=d0 + rng, plane_n=rng)
    tr = tracker or vp.FeatureTracker()
    assert tr.set_reference(sc.render(0.0, 0.0, d0, 0, 0, 0))
    n, e = move
    roll, pitch, yaw = att
    meas = tr.measure(sc.render(n, e, d0, roll, pitch, yaw))
    assert meas.ok, meas.note
    if not compensate:
        roll = pitch = yaw = 0.0
    range_now = rng if mount == "down" else rng
    got = vp.body_error(meas, CAM, mount, range_now, roll, pitch, yaw)
    want = truth_body(-n, -e, att[2])
    return got, want, meas


def check(name, got, want, tol=TOL):
    err = math.hypot(got[0] - want[0], got[1] - want[1])
    status = "ok " if err <= tol else "FAIL"
    print(f"[{status}] {name:48s} got fwd {got[0]:+.3f} right {got[1]:+.3f} | "
          f"true fwd {want[0]:+.3f} right {want[1]:+.3f} | err {err*100:.1f} cm")
    assert err <= tol, name


def test_down_translation():
    for mv in [(0.20, -0.10), (-0.15, 0.25), (0.05, 0.05)]:
        got, want, _ = run_case("down", 0.8, mv)
        check(f"down camera, moved N{mv[0]:+.2f} E{mv[1]:+.2f}", got, want)


def test_down_heading_change():
    got, want, _ = run_case("down", 0.8, (0.15, 0.10), att=(0, 0, math.radians(20)))
    check("down camera, moved + yawed 20 deg", got, want)


def test_down_tilt_compensation():
    att = (math.radians(4), math.radians(-3), 0.0)
    got, want, _ = run_case("down", 0.8, (0.0, 0.0), att=att)
    check("down camera, not moved, rolled 4 / pitched -3 deg", got, want)
    raw, _, _ = run_case("down", 0.8, (0.0, 0.0), att=att, compensate=False)
    raw_err = math.hypot(*raw)
    print(f"       (same case without IMU tilt compensation: {raw_err*100:.1f} cm false error)")
    assert raw_err > 0.05


def test_forward_translation():
    for mv in [(0.25, 0.10), (-0.20, -0.15)]:
        got, want, _ = run_case("forward", 2.0, mv)
        check(f"forward camera 2 m off wall, moved N{mv[0]:+.2f} E{mv[1]:+.2f}", got, want, tol=0.06)


def test_forward_yaw_compensation():
    got, want, _ = run_case("forward", 2.0, (0.0, 0.10), att=(0, 0, math.radians(5)))
    check("forward camera, moved E+0.10 + yawed 5 deg", got, want, tol=0.06)


def test_target_task13():
    """Blue-square tracker: centre error and the judge condition."""
    d0, alt = 1.0, 0.6
    sc = vp.SyntheticCamera(CAM, "down", TEX, plane_z=d0 + alt)
    tr = vp.TargetTracker()
    # vehicle offset 8 cm north, 5 cm east of the square's centre
    img = sc.render(0.08, 0.05, d0, 0, 0, 0)
    assert tr.set_reference(img)
    meas = tr.measure(img)
    got = vp.body_error(meas, CAM, "down", alt, 0, 0, 0)
    check("target tracker, 8 cm N / 5 cm E of blue square", got, truth_body(-0.08, -0.05, 0))
    assert meas.blue_fully_visible and not meas.red_visible, "should meet Task 1.3 condition"
    far = tr.measure(sc.render(0.35, 0.0, d0, 0, 0, 0))
    print(f"[ok ] 35 cm off: blue fully visible={far.blue_fully_visible}, red visible={far.red_visible}")
    assert (not far.blue_fully_visible) or far.red_visible, "35 cm off should fail the condition"


if __name__ == "__main__":
    for fn in [test_down_translation, test_down_heading_change, test_down_tilt_compensation,
               test_forward_translation, test_forward_yaw_compensation, test_target_task13]:
        fn()
    print("\nall vision checks passed")
