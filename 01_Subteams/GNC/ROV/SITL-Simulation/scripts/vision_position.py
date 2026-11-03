"""Camera-based horizontal position measurement for station keeping.

Why a camera: the IMU cannot hold horizontal position on its own. Its
accelerometer bias and tilt error integrate twice into position, so the error
grows with the square of time (a 2 mg bias is ~1 m after 10 s), and a steady
drift at constant velocity produces no acceleration to measure at all. The
camera sees the scene directly, so its error does not grow with time.

How it works: when station keeping engages, the current image is stored as the
zero point (the reference). Every later frame is compared with it and the
result is the reference position's location relative to the vehicle, in
metres, in the body frame (forward, right). The IMU is still used: the change
in roll/pitch/yaw since the reference is removed from the image motion, so the
vehicle tilting is not mistaken for the vehicle moving.

Two trackers:
  * FeatureTracker - ORB features matched against the reference image, robust
    similarity fit (RANSAC). Works on any textured scene: pool floor, coral
    garden, iceberg face.
  * TargetTracker  - Task 1.3: finds the blue coral-garden square and reports
    whether the whole blue square is in view and no red is visible, which is
    exactly the judge's pass condition.

Camera mounts:
  * down    - optical axis along body +z; image right = body right,
              image up = body forward. `range_m` = height above the floor.
  * forward - optical axis along body +x; image right = body right.
              `range_m` = stand-off distance to the surface being viewed.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

PROC_WIDTH = 640          # frames are resized to this width before processing


# --------------------------------------------------------------------------- camera
@dataclass
class CameraModel:
    width: int
    height: int
    hfov_deg: float

    @property
    def f_px(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    @property
    def center(self) -> tuple[float, float]:
        return self.width / 2.0, self.height / 2.0


@dataclass
class Measurement:
    ok: bool
    px: float = 0.0            # reference point (or target centroid) in the current image, px
    py: float = 0.0
    scale: float = 1.0         # current / reference image scale (forward camera range change)
    inliers: int = 0
    note: str = ""
    # TargetTracker extras (Task 1.3 judging condition)
    blue_fully_visible: bool = False
    red_visible: bool = False
    range_est: float = 0.0     # TargetTracker: camera-to-square distance from its known 0.5 m size


def to_gray(frame: np.ndarray) -> np.ndarray:
    return frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def resize_for_processing(frame: np.ndarray) -> np.ndarray:
    h, w = frame.shape[:2]
    if w == PROC_WIDTH:
        return frame
    return cv2.resize(frame, (PROC_WIDTH, int(round(h * PROC_WIDTH / w))), interpolation=cv2.INTER_AREA)


# --------------------------------------------------------------------------- trackers
class FeatureTracker:
    """Reference-frame feature matching (ORB + RANSAC similarity)."""

    def __init__(self, n_features: int = 1000, min_inliers: int = 25):
        self.orb = cv2.ORB_create(nfeatures=n_features, fastThreshold=10)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.min_inliers = min_inliers
        self.ref_kp = None
        self.ref_des = None
        self.center = (0.0, 0.0)

    def set_reference(self, frame: np.ndarray) -> bool:
        gray = to_gray(frame)
        h, w = gray.shape
        self.center = (w / 2.0, h / 2.0)
        self.ref_kp, self.ref_des = self.orb.detectAndCompute(gray, None)
        return self.ref_des is not None and len(self.ref_kp) >= self.min_inliers * 2

    def measure(self, frame: np.ndarray) -> Measurement:
        if self.ref_des is None:
            return Measurement(False, note="no reference")
        kp, des = self.orb.detectAndCompute(to_gray(frame), None)
        if des is None or len(kp) < self.min_inliers:
            return Measurement(False, note="too few features")
        pairs = self.matcher.knnMatch(self.ref_des, des, k=2)
        good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < 0.75 * p[1].distance]
        if len(good) < self.min_inliers:
            return Measurement(False, note=f"{len(good)} matches")
        src = np.float32([self.ref_kp[g.queryIdx].pt for g in good])
        dst = np.float32([kp[g.trainIdx].pt for g in good])
        M, mask = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC,
                                              ransacReprojThreshold=3.0, maxIters=2000)
        inl = int(mask.sum()) if mask is not None else 0
        if M is None or inl < self.min_inliers:
            return Measurement(False, inliers=inl, note=f"{inl} inliers")
        cx, cy = self.center
        px = M[0, 0] * cx + M[0, 1] * cy + M[0, 2]
        py = M[1, 0] * cx + M[1, 1] * cy + M[1, 2]
        scale = math.hypot(M[0, 0], M[1, 0])
        return Measurement(True, px, py, scale, inl)


class TargetTracker:
    """Task 1.3: the blue coral-garden square inside the red 130 cm frame."""

    BLUE = ((95, 90, 40), (130, 255, 255))
    RED_LO = ((0, 110, 60), (8, 255, 255))
    RED_HI = ((172, 110, 60), (180, 255, 255))

    SIDE_M = 0.50            # coral-garden square side (manual, Task 1.3)
    RED_INNER_M = 1.28       # clear span inside the 130 cm red pipe square

    def __init__(self, min_area_frac: float = 0.01, red_frac: float = 0.002, f_px: float | None = None):
        self.min_area_frac = min_area_frac
        self.red_frac = red_frac
        self.f_px = f_px     # set by the caller; enables range_est
        self.ref_area = None

    def set_reference(self, frame: np.ndarray) -> bool:
        self.ref_area = None
        m = self.measure(frame)          # with no reference, scale = raw apparent size
        self.ref_area = m.scale if m.ok else None
        return m.ok

    def measure(self, frame: np.ndarray) -> Measurement:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]
        blue = cv2.inRange(hsv, *self.BLUE)
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        red = cv2.inRange(hsv, *self.RED_LO) | cv2.inRange(hsv, *self.RED_HI)
        red_visible = cv2.countNonZero(red) > self.red_frac * w * h
        n, _, stats, cents = cv2.connectedComponentsWithStats(blue, connectivity=8)
        if n <= 1:
            return Measurement(False, note="blue square not found", red_visible=red_visible)
        i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        x, y, bw, bh, area = stats[i]
        if area < self.min_area_frac * w * h:
            return Measurement(False, note="blue region too small", red_visible=red_visible)
        fully = x > 1 and y > 1 and x + bw < w - 1 and y + bh < h - 1
        # scale is the square's apparent size; relative to the reference for range change
        size = math.sqrt(area)   # side length in px, independent of in-image rotation
        scale = size / self.ref_area if self.ref_area else size
        # distance from the known square size; only trusted when the whole square is visible
        rng = self.f_px * self.SIDE_M / size if (self.f_px and fully) else 0.0
        return Measurement(True, float(cents[i][0]), float(cents[i][1]), scale, int(area),
                           blue_fully_visible=fully, red_visible=red_visible, range_est=rng)


def task13_best_range(cam: CameraModel) -> tuple[float, float]:
    """Height above the coral garden that gives the widest Task 1.3 margin.

    The image footprint (w wide, w*H/W tall) must contain the 0.50 m blue square
    and fit inside the red square's clear span. Equal margins on both limits:
        RED_INNER - w = (H/W)*w - SIDE   ->   w = (RED_INNER + SIDE) / (1 + H/W)
    Returns (range_m, allowed horizontal drift each way, m). The drift margin
    depends only on the aspect ratio, not on the field of view.
    """
    ar = cam.height / cam.width
    w = (TargetTracker.RED_INNER_M + TargetTracker.SIDE_M) / (1.0 + ar)
    rng = (w / 2.0) / math.tan(math.radians(cam.hfov_deg) / 2.0)
    return rng, (TargetTracker.RED_INNER_M - w) / 2.0


# --------------------------------------------------------------------------- geometry
def body_error(meas: Measurement, cam: CameraModel, mount: str, range_m: float,
               d_roll: float, d_pitch: float, d_yaw: float,
               range_now: float | None = None) -> tuple[float, float]:
    """Reference position relative to the vehicle, body frame (forward, right), metres.

    range_m is the range when the reference was taken; range_now the current one
    (default: range_m / scale, i.e. from the image's own scale change, so sinking
    toward the floor does not inflate the error). d_roll/d_pitch/d_yaw are the
    attitude change since the reference, radians (ArduPilot convention: roll
    right, pitch nose-up, yaw right positive). The image shift they cause is
    removed so tilting is not read as translation.
    """
    f = cam.f_px
    cx, cy = cam.center
    if range_now is None:
        range_now = range_m / meas.scale if meas.scale > 0 else range_m
    if mount == "down":
        px = meas.px - f * math.tan(d_roll)
        py = meas.py - f * math.tan(d_pitch)
        e_right = (px - cx) / f * range_now
        e_fwd = -(py - cy) / f * range_now
    elif mount == "forward":
        px = meas.px + f * math.tan(d_yaw)
        e_right = (px - cx) / f * range_now
        # apparent scale s = r0 / r: the reference stand-off is (r - r0) ahead
        e_fwd = range_m * (1.0 / meas.scale - 1.0) if meas.scale > 0 else 0.0
    else:
        raise ValueError(f"unknown camera mount {mount!r}")
    return e_fwd, e_right


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# --------------------------------------------------------------------------- frame sources
class FrameSource:
    """Latest-frame reader for an RTSP/UDP URL or a device index, on its own thread.

    Keeps only the newest frame (like Photogrammetry Studio's StreamWorker) so
    the controller always acts on the current view, never on a queued backlog.
    """

    def __init__(self, url: str):
        src = int(url) if url.isdigit() else url
        self.cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG) if isinstance(src, str) else cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._frame, self._t = None, 0.0
        self._lock = threading.Lock()
        self._stop = False
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self):
        while not self._stop:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._lock:
                self._frame, self._t = frame, time.monotonic()

    def latest(self):
        with self._lock:
            return self._frame, self._t

    def close(self):
        self._stop = True
        self._th.join(timeout=1.0)
        self.cap.release()


def make_texture(size_px: int = 2400, m_per_px: float = 0.004, seed: int = 1,
                 coral_garden: bool = True) -> np.ndarray:
    """Procedural pool-floor texture (~9.6 m square) for the synthetic camera.

    Grey speckle + neutral-toned tiles/stones so features exist everywhere, and, if
    requested, the Task 1.3 layout at the centre: a 50 cm blue square inside a
    130 cm square of red 1/2-inch pipe.
    """
    rng = np.random.default_rng(seed)
    img = rng.normal(150, 18, (size_px, size_px)).astype(np.float32)
    img = cv2.GaussianBlur(img, (0, 0), 2.0)
    img = cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    for _ in range(1800):
        x, y = rng.integers(0, size_px, 2)
        r = int(rng.integers(4, 22))
        g = int(rng.integers(60, 230))          # neutral pool-floor tones: no false blue/red
        c = tuple(int(np.clip(g + t, 0, 255)) for t in rng.integers(-12, 13, 3))
        if rng.random() < 0.5:
            cv2.circle(img, (int(x), int(y)), r, c, -1)
        else:
            cv2.rectangle(img, (int(x), int(y)), (int(x) + r, int(y) + int(r * 1.5)), c, -1)
    if coral_garden:
        c = size_px // 2
        half_red = int(0.65 / m_per_px)
        half_blue = int(0.25 / m_per_px)
        pipe = max(2, int(0.021 / m_per_px))
        cv2.rectangle(img, (c - half_red, c - half_red), (c + half_red, c + half_red), (0, 0, 205), pipe)
        cv2.rectangle(img, (c - half_blue, c - half_blue), (c + half_blue, c + half_blue), (205, 90, 10), -1)
    return img


class SyntheticCamera:
    """Renders what a camera on the vehicle would see of a textured plane.

    SITL has no camera, so for closed-loop testing this draws the image from the
    simulated vehicle pose. The controller only ever sees the rendered image;
    the pose is used to draw it, never passed to the controller.

    NED frame, metres. Down mount looks at a floor at depth `plane_z`; forward
    mount looks at a wall at north = `plane_n`.
    """

    def __init__(self, cam: CameraModel, mount: str, texture: np.ndarray,
                 m_per_px: float = 0.004, plane_z: float = 3.0, plane_n: float = 2.0):
        self.cam, self.mount, self.tex, self.mpp = cam, mount, texture, m_per_px
        self.plane_z, self.plane_n = plane_z, plane_n
        f = cam.f_px
        cx, cy = cam.center
        self.K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float64)
        th, tw = texture.shape[:2]
        # texture pixel (col,row) -> plane coordinates (a,b) in metres, centred
        self.T = np.array([[m_per_px, 0, -tw / 2 * m_per_px],
                           [0, m_per_px, -th / 2 * m_per_px],
                           [0, 0, 1]], dtype=np.float64)
        if mount == "down":       # camera x = body right (+y), camera y = body back (-x), z = body down
            self.R_cb = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float64)
        elif mount == "forward":  # camera x = body right, camera y = body down, z = body forward
            self.R_cb = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=np.float64)
        else:
            raise ValueError(mount)

    @staticmethod
    def R_bn(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Rotation taking NED vectors into the body frame (ZYX Euler)."""
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                  math.sin(pitch), math.cos(yaw), math.sin(yaw))
        R_nb = np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                         [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                         [-sp, sr * cp, cr * cp]])
        return R_nb.T

    def render(self, n: float, e: float, d: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
        R = self.R_cb @ self.R_bn(roll, pitch, yaw)
        C = np.array([n, e, d])
        if self.mount == "down":      # plane axes: a -> east (image right at yaw 0), b -> south
            origin, u, v = np.array([0.0, 0.0, self.plane_z]), np.array([0.0, 1.0, 0.0]), np.array([-1.0, 0.0, 0.0])
        else:                         # wall facing the vehicle: a -> east, b -> down
            origin, u, v = np.array([self.plane_n, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
        H = self.K @ np.column_stack([R @ u, R @ v, R @ (origin - C)])
        return cv2.warpPerspective(self.tex, H @ self.T, (self.cam.width, self.cam.height),
                                   flags=cv2.INTER_LINEAR, borderValue=(40, 40, 40))
