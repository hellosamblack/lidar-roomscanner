"""LSM6DSV16X sensor state + orientation/heading math (streams 9/10). Thread-safe:
the reader thread calls feed(); the UI thread reads latest_*/history()."""
from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass

import numpy as np

from .protocol import Frame, FrameType, StreamId, decode_env, decode_imu_quat


@dataclass(frozen=True)
class EnvSample:
    pressure_pa: float
    mag_ut: tuple[float, float, float]
    temp_c: float
    t_us: int


class SensorState:
    def __init__(self, history: int = 256):
        self._lock = threading.Lock()
        self._quat: tuple[float, float, float, float] | None = None
        self._env: EnvSample | None = None
        self._pressure = deque(maxlen=history)
        self._temp = deque(maxlen=history)

    def feed(self, frame: Frame) -> None:
        if frame.header.frame_type != FrameType.DATA:
            return
        sid = frame.header.stream_id
        if sid == StreamId.IMU_QUAT:
            q = decode_imu_quat(frame.payload)
            with self._lock:
                self._quat = q
        elif sid == StreamId.ENV:
            pressure, mag, temp = decode_env(frame.payload)
            sample = EnvSample(pressure, mag, temp, frame.header.t_us)
            with self._lock:
                self._env = sample
                self._pressure.append(pressure)
                self._temp.append(temp)

    def latest_quat(self) -> tuple[float, float, float, float] | None:
        with self._lock:
            return self._quat

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


def gizmo_pose(quat: tuple[float, float, float, float], scale: float,
               anchor: tuple[float, float, float]) -> np.ndarray:
    """4x4 pose for the orientation gizmo: rotation from quaternion, uniform scale, placed
    at anchor. Suitable for Open3D geometry.transform()."""
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(*quat) * scale
    m[:3, 3] = np.array(anchor, dtype=np.float64)
    return m
