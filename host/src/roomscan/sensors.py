"""LSM6DSV16X sensor state + orientation/heading math (streams 9/10). Thread-safe:
the reader thread calls feed(); the UI thread reads latest_*/history()."""
from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:   # import-only: imufusion imports the quaternion helpers from here
    from .imufusion import ImuFusion

from .magcal import MagCalibration
from .protocol import (
    IMU_RAW_TICK_US,
    Frame,
    FrameType,
    ImuRawBatch,
    StreamId,
    decode_env,
    decode_imu_cal,
    decode_imu_quat,
    decode_imu_raw,
)


@dataclass(frozen=True)
class EnvSample:
    pressure_pa: float
    mag_ut: tuple[float, float, float]
    temp_c: float
    t_us: int


class SensorState:
    def __init__(self, history: int = 256, fusion: "YawFusion | None" = None,
                 env_spark_interval_s: float = 2.0, env_spark_depth: int = 300,
                 imu_raw_history: int = 64, imu_fusion: "ImuFusion | None" = None):
        self._lock = threading.Lock()
        # stream 11: latest raw-FIFO batch + a short rolling window of them. Consumed by
        # the optional `imu_fusion` stage (roomscan.imufusion) — which is OFF unless a
        # filter is passed in, so the default fused_quat() path is byte-for-byte the
        # stream-9/YawFusion behaviour SLAM already depends on.
        self._imu_raw: ImuRawBatch | None = None
        self._imu_raw_hist: deque[tuple[int, ImuRawBatch]] = deque(maxlen=imu_raw_history)
        self._imu_fusion = imu_fusion
        # stream 12: the LSM's own oscillator trim, which sets what a stream-11 timestamp
        # tick is actually worth. Starts nominal — recordings made before stream 12 existed
        # never carry one, and nominal is exactly the behaviour they were decoded with.
        self._imu_tick_us: float = IMU_RAW_TICK_US
        self._quat: tuple[float, float, float, float] | None = None
        self._env: EnvSample | None = None
        self._pressure = deque(maxlen=history)
        self._temp = deque(maxlen=history)
        self._fusion = fusion
        self._raw_mag: tuple[float, float, float] | None = None
        self._spark_interval_us = int(env_spark_interval_s * 1e6)
        self._pressure_spark = deque(maxlen=env_spark_depth)
        self._temp_spark = deque(maxlen=env_spark_depth)
        self._last_spark_t: int | None = None

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
                if (self._last_spark_t is None
                        or (frame.header.t_us - self._last_spark_t) >= self._spark_interval_us):
                    self._pressure_spark.append(pressure)
                    self._temp_spark.append(temp)
                    self._last_spark_t = frame.header.t_us
        elif sid == StreamId.IMU_RAW:
            with self._lock:
                tick_us = self._imu_tick_us
            batch = decode_imu_raw(frame.payload, tick_us)
            with self._lock:
                self._imu_raw = batch
                self._imu_raw_hist.append((frame.header.t_us, batch))
                if self._imu_fusion is not None:
                    # yaw anchor = whatever the pre-existing path would have returned
                    # (YawFusion output if attached and settled, else the SFLP quat).
                    self._imu_fusion.update(batch, yaw_ref=self._legacy_quat_locked())
        elif sid == StreamId.IMU_CAL:
            cal = decode_imu_cal(frame.payload)
            with self._lock:
                self._imu_tick_us = cal.tick_us

    @property
    def imu_tick_us(self) -> float:
        """LSM timestamp-counter LSB in µs — the trimmed value once a stream-12 frame has
        arrived, the nominal 21.7 µs before that (and forever, on an older recording)."""
        with self._lock:
            return self._imu_tick_us

    def clear_imu_raw(self) -> None:
        """Drop the stream-11 batch + history (owner ask, 2026-07-28: the
        World orientation mode prefers this over the quat-derived gravity
        fallback). Callers that SWAP the frame source (live<->replay, or one
        replay to another) must call this -- otherwise a replay capture with
        no stream 11 silently inherits a stale gravity vector from whatever
        source was active before it, reporting a real-looking but physically
        unrelated tilt/roll. `SensorState` has no other notion of "this data
        belongs to the old source", so this is an explicit reset, not
        automatic."""
        with self._lock:
            self._imu_raw = None
            self._imu_raw_hist.clear()

    def latest_imu_raw(self) -> ImuRawBatch | None:
        """Newest stream-11 raw-FIFO batch, or None if the device isn't sending them."""
        with self._lock:
            return self._imu_raw

    def imu_raw_history(self) -> list[tuple[int, ImuRawBatch]]:
        """Rolling window of (frame t_us, batch), oldest first."""
        with self._lock:
            return list(self._imu_raw_hist)

    def latest_quat(self) -> tuple[float, float, float, float] | None:
        with self._lock:
            return self._quat

    def _legacy_quat_locked(self) -> tuple[float, float, float, float] | None:
        """The pre-stream-11 orientation: YawFusion output if attached and settled,
        else the raw SFLP quaternion. Caller must hold ``self._lock``."""
        if self._fusion is not None:
            fused = self._fusion.fused_quat()
            if fused is not None:
                return fused
        return self._quat

    def fused_quat(self) -> tuple[float, float, float, float] | None:
        """Best available orientation.

        Precedence: the optional stream-11 high-rate ``ImuFusion`` (only when one was
        explicitly attached AND it has converged), then the yaw-drift-corrected
        ``YawFusion`` output, then the raw SFLP quaternion.

        With ``imu_fusion=None`` (the default, and what SLAM gets today) this is
        exactly the previous two-way behaviour — see test_imu_fusion.py's
        ``test_slam_non_regression_*`` guards."""
        with self._lock:
            if self._imu_fusion is not None:
                high_rate = self._imu_fusion.fused_quat()
                if high_rate is not None:
                    return high_rate
            return self._legacy_quat_locked()

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

    def pressure_spark_history(self) -> np.ndarray:
        with self._lock:
            return np.array(self._pressure_spark, dtype=np.float64)

    def temp_spark_history(self) -> np.ndarray:
        with self._lock:
            return np.array(self._temp_spark, dtype=np.float64)

    def reset_fusion(self) -> None:
        """Reset the yaw-fusion filter so it snaps fresh on the next valid mag
        sample.  Safe to call even without a fusion filter attached."""
        with self._lock:
            if self._fusion is not None:
                self._fusion.reset()
            if self._imu_fusion is not None:
                self._imu_fusion.reset()


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


def quat_roll_deg(quat) -> float:
    """ZYX roll (bank about the forward axis) of a [w,x,y,z] quaternion, in
    degrees, [-180, 180). Same Tait-Bryan convention as `quat_yaw_deg`/
    `quat_pitch_deg` (yaw-pitch-roll = Z-Y-X)."""
    w, x, y, z = quat
    return math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))


# --- Alternate orientation decompositions (owner ask, 2026-07-28) -----------
#
# The default roll/pitch/yaw above is ZYX Tait-Bryan (R = Rz(yaw) Ry(pitch)
# Rx(roll)); its gimbal lock is at pitch = +-90 deg, i.e. the world-Z
# projection of body X (Up) -> +-1. A handheld scanner has no fixed "forward",
# so those names mislead depending on grip, AND the singularity bites whenever
# the device is aimed steeply up/down -- exactly the ~86 deg-pitch case that
# motivated this. Two remedies, both presentation-only (never touch
# `display_rotation`/`fused_quat()`): (1) user-relabelable axis names, (2) more
# than one decomposition, so a different grip/aim can pick whichever mode keeps
# its singularity out of the way.


def quat_yaw_alt_deg(quat) -> float:
    """Alternate Tait-Bryan decomposition (R = Rz(a) Rx(b) Ry(c)): the outer
    (Z-ish) component. Paired with `quat_pitch_alt_deg`/`quat_roll_alt_deg`;
    see `quat_pitch_alt_deg` for where this decomposition's gimbal lock sits."""
    r = quat_to_matrix(*quat)
    return math.degrees(math.atan2(-r[0, 1], r[1, 1]))


def quat_pitch_alt_deg(quat) -> float:
    """Alternate Tait-Bryan decomposition's singular component: the world-Z
    projection of body Y (Right), i.e. `asin(R[2,1])`. Unlike the default
    `quat_pitch_deg` (gimbal lock when body X/Up -> vertical, i.e. the device
    aimed steeply up/down), THIS decomposition locks when body Y/Right ->
    vertical -- the device rolled onto its side. The two modes' singularities
    are disjoint attitudes: whichever one degenerates, the other is likely
    still well-conditioned."""
    r = quat_to_matrix(*quat)
    s = max(-1.0, min(1.0, r[2, 1]))
    return math.degrees(math.asin(s))


def quat_roll_alt_deg(quat) -> float:
    """Alternate Tait-Bryan decomposition: the inner (Y-ish) component."""
    r = quat_to_matrix(*quat)
    return math.degrees(math.atan2(-r[2, 0], r[2, 2]))


def boresight_view_deg(quat) -> tuple[float, float, float]:
    """(azimuth_deg [0,360), elevation_deg [-90,90], roll_deg [-180,180)) of
    the ToF optical axis -- the one decomposition that stays meaningful
    however the device is gripped, because it reports where the SENSOR points
    rather than an arbitrary body "forward".

    The boresight is body +Z: `docs/coordinate-frames.md` "The four frames"
    lists the SFLP body frame as X=Up, Y=Right, **Z=Forward**, and separately
    the ToF (CV) frame as Z=Forward with `T_CV_TO_BODY: Z_body = Z_cv` -- the
    CV camera's forward axis maps onto body Z unchanged, so body Z IS the
    optical axis under both frames' own definitions.

    azimuth: compass bearing the sensor points at (0=North, 90=East; SFLP
    world is X=North, Y=West, so East = -Y).
    elevation: angle of the boresight above (+) / below (-) horizontal.
    roll: twist about the boresight, referenced to world "up" projected into
    the plane perpendicular to the boresight -- 0 when the device's structural
    Up axis (body X) points as close to true vertical as the aim allows.

    Singularity: elevation -> +-90 deg (pointing straight at the ceiling/floor)
    -- azimuth and roll both become an ill-defined split of the same rotation
    about a now-vertical boresight. `roll` is returned as 0.0 in that regime;
    callers must consult the singularity margin, not trust the value."""
    r = quat_to_matrix(*quat)
    boresight = r[:, 2]          # body Z column: pointing direction in world
    up_ref = r[:, 0]             # body X column: structural "up" reference
    elevation = math.degrees(math.asin(max(-1.0, min(1.0, float(boresight[2])))))
    azimuth = math.degrees(math.atan2(-float(boresight[1]), float(boresight[0]))) % 360.0
    world_up = np.array([0.0, 0.0, 1.0])
    perp = world_up - float(np.dot(world_up, boresight)) * boresight
    up_perp = up_ref - float(np.dot(up_ref, boresight)) * boresight
    pn, un = float(np.linalg.norm(perp)), float(np.linalg.norm(up_perp))
    if pn < 1e-6 or un < 1e-6:
        roll = 0.0     # near-singular: undefined -- caller must consult the margin
    else:
        perp_n, up_n = perp / pn, up_perp / un
        cross = np.cross(perp_n, up_n)
        roll = math.degrees(math.atan2(float(np.dot(cross, boresight)), float(np.dot(perp_n, up_n))))
    return azimuth, elevation, roll


def gravity_body_from_imu_raw(batch) -> tuple[float, float, float] | None:
    """Body-frame DOWN unit vector from the stream-11 SFLP gravity FIFO tag
    (0x17, mean of the batch's samples) -- fixed +-2g scale at 0.061 mg/LSB,
    ~16x finer in tilt than the fp16-encoded SFLP quaternion step
    (`docs/iks4a1-stacking.md` "Orientation-noise pass"). None if the batch
    carries no gravity samples (stream 11 not enabled, or a batch that only
    had gyro/accel/timestamp words) -- callers must fall back to the
    quat-derived down vector (`quat_to_matrix(*quat).T @ [0,0,-1]`, the same
    computation `ir_gravity_rot` uses) in that case.

    Sign: the SFLP gravity tag reports the sensed reaction (+g on the axis
    pointing "up" when the device sits still, same convention as the raw
    accelerometer) -- negated here so the return value points the direction
    gravity itself pulls, matching `ir_gravity_rot`'s `g_body` convention."""
    if batch is None or batch.gravity_g.shape[0] == 0:
        return None
    v = np.asarray(batch.gravity_g, dtype=np.float64).mean(axis=0)
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return None
    return tuple((-v / n).tolist())


def tilt_from_down_deg(down_body, axis_body=(0.0, 0.0, 1.0)) -> float:
    """Angle in degrees of `axis_body` (default: the boresight, body +Z) from
    horizontal, given only the body-frame DOWN unit vector: 0=horizontal,
    +90=axis points straight up, -90=straight down. Needs just the 2 DoF of
    tilt that gravity alone supplies -- no heading, no full attitude."""
    axis = np.asarray(axis_body, dtype=np.float64)
    down = np.asarray(down_body, dtype=np.float64)
    dot = max(-1.0, min(1.0, float(np.dot(axis, down))))
    return -math.degrees(math.asin(dot))


def triad_roll_deg(down_body, axis_body=(0.0, 0.0, 1.0),
                    up_ref_body=(1.0, 0.0, 0.0)) -> float | None:
    """Roll of `up_ref_body` (default: body X / the structural Up axis) about
    `axis_body` (default: the boresight), referenced to true vertical --
    computed ENTIRELY from the body-frame down vector: the gravity-only half
    of a TRIAD/eCompass construction, no magnetometer and no gyro-integrated
    quaternion involved. None when `axis_body` is within ~0.1 deg of vertical
    (parallel to `down_body`), where the perpendicular reference collapses and
    roll is undefined -- the gravity-tilt singularity."""
    axis = np.asarray(axis_body, dtype=np.float64)
    down = np.asarray(down_body, dtype=np.float64)
    true_up = -down
    perp = true_up - float(np.dot(true_up, axis)) * axis
    ref = np.asarray(up_ref_body, dtype=np.float64)
    ref_perp = ref - float(np.dot(ref, axis)) * axis
    pn, rn = float(np.linalg.norm(perp)), float(np.linalg.norm(ref_perp))
    if pn < 1e-6 or rn < 1e-6:
        return None
    perp_n, ref_n = perp / pn, ref_perp / rn
    cross = np.cross(perp_n, ref_n)
    return math.degrees(math.atan2(float(np.dot(cross, axis)), float(np.dot(perp_n, ref_n))))


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


# True physical mapping derived from board orientation:
# ToF (CV) frame: X=Right, Y=Down, Z=Forward
# SFLP body frame: X=Up, Y=Right, Z=Forward (when board held vertically, USB down)
# Thus: X_body = -Y_cv; Y_body = X_cv; Z_body = Z_cv
T_CV_TO_BODY = np.array([
    [ 0.0, -1.0,  0.0],
    [ 1.0,  0.0,  0.0],
    [ 0.0,  0.0,  1.0]
])

# SFLP World frame: X=North, Y=West, Z=Up
# Open3D CV World: X=Right(East), Y=Down(-Up), Z=Forward(North)
# Thus: X_cv = -Y_world; Y_cv = -Z_world; Z_cv = X_world
T_WORLD_TO_CV = np.array([
    [ 0.0, -1.0,  0.0],
    [ 0.0,  0.0, -1.0],
    [ 1.0,  0.0,  0.0]
])

def gizmo_pose(quat: tuple[float, float, float, float], scale: float,
               anchor: tuple[float, float, float]) -> np.ndarray:
    """4x4 pose for the orientation gizmo: rotation from quaternion, uniform scale, placed
    at anchor. Suitable for Open3D geometry.transform()."""
    r = quat_to_matrix(*quat)
    r_mapped = T_WORLD_TO_CV @ r @ T_CV_TO_BODY
    m = np.eye(4)
    m[:3, :3] = r_mapped * scale
    m[:3, 3] = np.array(anchor, dtype=np.float64)
    return m


def ir_gravity_rot(quat: tuple[float, float, float, float]) -> int:
    """Return the number of CCW 90° turns (0–3) to apply to the raw IR image so
    that its "down" matches the physical gravity direction detected by the SFLP
    accelerometer/gyroscope fusion.

    Method:
      1. Rotate the world-gravity vector [0, 0, -1] (SFLP Z-up world frame)
         into the sensor body frame: g_body = R.T @ [0, 0, -1].
      2. The image plane is the CV XY plane. CV Right = SFLP Y. CV Down = SFLP -X.
      3. The in-plane gravity components are gx = g_body[1] and gy = -g_body[0].
      4. The in-plane roll angle is atan2(gx, gy) — 0° when gravity is along
         +image-down, increasing CCW. Snap to nearest 90° and return rot90 count.
    """
    r = quat_to_matrix(*quat)   # body → world
    gravity_world = np.array([0.0, 0.0, -1.0])
    g_body = r.T @ gravity_world   # sensor body frame gravity vector
    gx, gy = float(g_body[1]), float(-g_body[0])   # in-plane components
    angle_deg = math.degrees(math.atan2(gx, gy))
    # Snap to nearest 90° and convert to rot90 count (CCW turns)
    step = int(round(angle_deg / 90.0)) % 4
    return step


AXIS_CONVENTION = np.diag([1.0, -1.0, -1.0])   # mag-mounting-vs-IMU sign/permutation; resolved on-target
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

    def reset(self) -> None:
        """Clear accumulated yaw correction so the filter snaps fresh on the
        next valid magnetometer sample."""
        self._delta = 0.0
        self._have_delta = False
        self._last_quat = None
        self._last_t = None
        self.status = "init"

    def fused_quat(self):
        if self._last_quat is None:
            return None
        return graft_yaw(self._last_quat, self._delta)
