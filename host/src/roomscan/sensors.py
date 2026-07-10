"""LSM6DSV16X sensor state + orientation/heading math (streams 9/10). Thread-safe:
the reader thread calls feed(); the UI thread reads latest_*/history()."""
from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass

import numpy as np

from .magcal import MagCalibration
from .protocol import Frame, FrameType, StreamId, decode_env, decode_imu_quat


@dataclass(frozen=True)
class EnvSample:
    pressure_pa: float
    mag_ut: tuple[float, float, float]
    temp_c: float
    t_us: int


class SensorState:
    def __init__(self, history: int = 256, fusion: "YawFusion | None" = None):
        self._lock = threading.Lock()
        self._quat: tuple[float, float, float, float] | None = None
        self._env: EnvSample | None = None
        self._pressure = deque(maxlen=history)
        self._temp = deque(maxlen=history)
        self._fusion = fusion
        self._raw_mag: tuple[float, float, float] | None = None

    def feed(self, frame: Frame) -> None:
        if frame.header.frame_type != FrameType.DATA:
            return
        sid = frame.header.stream_id
        if sid == StreamId.IMU_QUAT:
            q = decode_imu_quat(frame.payload)
            with self._lock:
                self._quat = q
                if self._fusion is not None and self._raw_mag is not None:
                    self._fusion.update(q, self._raw_mag, frame.header.t_us)
        elif sid == StreamId.ENV:
            pressure, mag, temp = decode_env(frame.payload)
            sample = EnvSample(pressure, mag, temp, frame.header.t_us)
            with self._lock:
                self._env = sample
                self._raw_mag = mag
                self._pressure.append(pressure)
                self._temp.append(temp)

    def latest_quat(self) -> tuple[float, float, float, float] | None:
        with self._lock:
            return self._quat

    def fused_quat(self) -> tuple[float, float, float, float] | None:
        """Yaw-drift-corrected orientation if a fusion filter is attached and has
        produced a result; otherwise the raw SFLP quaternion (today's behavior)."""
        with self._lock:
            if self._fusion is not None:
                fused = self._fusion.fused_quat()
                if fused is not None:
                    return fused
            return self._quat

    def fusion_status(self) -> str:
        with self._lock:
            return self._fusion.status if self._fusion is not None else "off"

    def latest_env(self) -> EnvSample | None:
        with self._lock:
            return self._env

    def pressure_history(self) -> np.ndarray:
        with self._lock:
            return np.array(self._pressure, dtype=np.float64)

    def temp_history(self) -> np.ndarray:
        with self._lock:
            return np.array(self._temp, dtype=np.float64)


def quat_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Unit quaternion [w,x,y,z] -> 3x3 rotation matrix. Normalizes defensively."""
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b) -> tuple[float, float, float, float]:
    """Hamilton product a ⊗ b for [w,x,y,z] quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def wrap180(deg: float) -> float:
    """Wrap an angle in degrees to [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def quat_yaw_deg(quat) -> float:
    """ZYX yaw (heading) of a [w,x,y,z] quaternion, in degrees, [-180, 180)."""
    w, x, y, z = quat
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def quat_pitch_deg(quat) -> float:
    """ZYX pitch of a [w,x,y,z] quaternion, in degrees, clamped to [-90, 90]."""
    w, x, y, z = quat
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(math.asin(s))


def graft_yaw(quat, delta_deg: float) -> tuple[float, float, float, float]:
    """Rotate `quat` about the WORLD +Z axis by `delta_deg` (pre-multiply). This
    changes only heading; roll/pitch (tilt) are preserved. Returns a unit quat."""
    a = math.radians(delta_deg) / 2.0
    qz = (math.cos(a), 0.0, 0.0, math.sin(a))
    w, x, y, z = quat_mul(qz, quat)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def tilt_compensated_heading(
    quat: tuple[float, float, float, float],
    mag_ut: tuple[float, float, float],
) -> float:
    """Heading in degrees [0,360): de-tilt the mag vector into the horizontal plane using
    the orientation, then atan2. Rotates the body-frame mag into world frame and reads the
    horizontal components, so the heading is correct when the device is not level."""
    r = quat_to_matrix(*quat)
    m_world = r @ np.array(mag_ut, dtype=np.float64)
    heading = np.degrees(np.arctan2(m_world[1], m_world[0]))
    return float(heading % 360.0)


def absolute_heading(quat, mag_ut) -> float:
    """Drift-free magnetic heading in degrees [0,360): de-tilt the mag using ONLY
    the orientation's roll/pitch (yaw stripped), so the result depends on the
    device's true heading and tilt but NOT on any yaw drift in `quat`.

    This is the yaw reference the fusion steers toward. Passing the full quat to
    `tilt_compensated_heading` instead would rotate the mag by the drifting yaw
    too, re-injecting exactly the drift the fusion exists to remove."""
    tilt_only = graft_yaw(quat, -quat_yaw_deg(quat))
    return tilt_compensated_heading(tilt_only, mag_ut)


def gizmo_pose(quat: tuple[float, float, float, float], scale: float,
               anchor: tuple[float, float, float]) -> np.ndarray:
    """4x4 pose for the orientation gizmo: rotation from quaternion, uniform scale, placed
    at anchor. Suitable for Open3D geometry.transform()."""
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(*quat) * scale
    m[:3, 3] = np.array(anchor, dtype=np.float64)
    return m


AXIS_CONVENTION = np.eye(3)   # mag-mounting-vs-IMU sign/permutation; resolved on-target
AXIS_CONVENTION.setflags(write=False)   # module constant — guard against in-place mutation


class YawFusion:
    """Stateful yaw-only complementary filter: grafts a gated, low-passed
    tilt-compensated magnetometer heading onto the SFLP quaternion. Tilt is
    taken from SFLP unchanged; only heading is corrected."""

    def __init__(self, tau_s: float = 20.0, calibration: MagCalibration | None = None,
                 anomaly_frac: float = 0.3, motion_rate_dps: float = 40.0,
                 gimbal_margin_deg: float = 15.0):
        self.tau_s = float(tau_s)
        self.cal = calibration
        self.anomaly_frac = float(anomaly_frac)
        self.motion_rate_dps = float(motion_rate_dps)
        self.gimbal_margin_deg = float(gimbal_margin_deg)
        self._delta = 0.0
        self._have_delta = False
        self._last_quat: tuple[float, float, float, float] | None = None
        self._last_t: int | None = None
        self.status = "init"

    def update(self, quat, raw_mag, t_us: int) -> None:
        quat = tuple(float(v) for v in quat)
        prev_quat, prev_t = self._last_quat, self._last_t
        self._last_quat = quat
        if self.cal is None:
            self.status = "gated:no-cal"
            self._last_t = t_us
            return
        if prev_quat is None or prev_t is None:
            self.status = "init"
            self._last_t = t_us
            return
        dt = (t_us - prev_t) / 1e6
        if dt <= 0:
            dt = 1e-3
        # gate: gimbal lock
        if abs(quat_pitch_deg(quat)) > 90.0 - self.gimbal_margin_deg:
            self.status = "gated:gimbal"
            self._last_t = t_us
            return
        # gate: fast motion (SFLP quat angular rate as accel-free motion proxy)
        dot = sum(a * b for a, b in zip(prev_quat, quat))
        ang = 2.0 * math.acos(max(0.0, min(1.0, abs(dot))))   # rad between orientations
        if math.degrees(ang) / dt > self.motion_rate_dps:
            self.status = "gated:motion"
            self._last_t = t_us
            return
        # calibrate + axis-convention the mag, then anomaly gate on magnitude
        cal_mag = AXIS_CONVENTION @ self.cal.apply(raw_mag)
        mag_norm = float(np.linalg.norm(cal_mag))
        if abs(mag_norm - self.cal.field_ut) > self.anomaly_frac * self.cal.field_ut:
            self.status = "gated:anomaly"
            self._last_t = t_us
            return
        heading = absolute_heading(quat, tuple(cal_mag))
        yaw = quat_yaw_deg(quat)
        if not self._have_delta:
            self._delta = wrap180(heading - yaw)   # snap on first valid sample
            self._have_delta = True
        else:
            gain = dt / (self.tau_s + dt)
            # first-order low-pass toward the mag heading; re-wrap so delta stays
            # in [-180, 180) even after many ±180 crossings (diagnostic sanity).
            self._delta = wrap180(self._delta + gain * wrap180(heading - (yaw + self._delta)))
        self.status = "active"
        self._last_t = t_us

    def fused_quat(self):
        if self._last_quat is None:
            return None
        return graft_yaw(self._last_quat, self._delta)
