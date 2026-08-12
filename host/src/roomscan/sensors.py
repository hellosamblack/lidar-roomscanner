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
    is_valid_env,
    is_valid_mag,
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
            if is_valid_env(pressure, mag, temp):
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

    def reset_source(self) -> None:
        """Drop EVERY sample derived from the current source AND the current
        position within it (BUG-091 / issue #102).

        `clear_imu_raw` only protects the stream-11 gravity path. That is enough
        for a source swap's *tilt* readout, but not for replay determinism: the
        latest SFLP quaternion, the latest ENV sample (pressure -> SLAM's baro Z,
        mag -> YawFusion), the pressure/temp histories, the stream-12 tick trim
        and the fusion filters' internal state all survive a seek too. The first
        depth frames after a rewind would then be submitted to SLAM carrying
        orientation and pressure from the timeline the operator just left, so the
        same capture at the same seek position produced a different map depending
        on where playback had been before.

        Every operation that begins a new replay timeline -- load capture, Go
        Live, seek, restart, loop wraparound -- calls this. Pause / resume /
        speed / loop-toggle do NOT: they do not move the read position, so their
        state is still the state of the frames that produced it.

        The post-reset window is deliberately empty rather than seeded: with no
        quat, `SlamRunner.submit` drops frames (it already does this at every
        live start-up), so SLAM simply waits for replay to supply orientation
        again instead of borrowing the old timeline's."""
        with self._lock:
            self._imu_raw = None
            self._imu_raw_hist.clear()
            self._imu_tick_us = IMU_RAW_TICK_US
            self._quat = None
            self._env = None
            self._raw_mag = None
            self._pressure.clear()
            self._temp.clear()
            self._pressure_spark.clear()
            self._temp_spark.clear()
            self._last_spark_t = None
            # Inlined rather than calling `reset_fusion()`: `self._lock` is not
            # reentrant and this whole reset must be one atomic step.
            if self._fusion is not None:
                self._fusion.reset()
            if self._imu_fusion is not None:
                self._imu_fusion.reset()

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


def graft_yaw_error_deg(target, quat) -> float:
    """Degrees about WORLD +Z that carry `quat` as close as possible to `target`.

    The exact inverse of `graft_yaw`: ``graft_yaw(quat, graft_yaw_error_deg(t, quat))``
    is the closest orientation to `t` reachable from `quat` by a pure heading change,
    and ``graft_yaw_error_deg(graft_yaw(q, d), q) == wrap180(d)`` for every `q`, `d`.

    **Use this, not a difference of `quat_yaw_deg`, for any heading-error loop**
    (BUG-039). `quat_yaw_deg` is ZYX yaw — the heading of body **Z** — and this
    device's SFLP body frame has **X = Up**, so at the attitudes it actually flies
    (86 deg ZYX pitch on the stationary capture, 3.8 deg from gimbal lock) the ZYX
    decomposition is ill-conditioned: a small tilt perturbation reads as a large
    apparent yaw, and a loop nulling that difference injects real heading error.
    Measured on `captures/stationary_stream11_20260728_190311.bin`, feeding
    `ImuFusion` the same bytes either way: ZYX 1.689 deg mean / 2.217 deg p95 of
    world-Z heading error, this term 0.017 / 0.053.

    Derivation (why it is the *optimal* pure-yaw correction, not merely a different
    one): `graft_yaw` pre-multiplies by ``qz(d)``, so we want ``qz(d) (x) quat`` to be
    closest to `target`, i.e. ``qz(d)`` closest to the world-frame residual
    ``rel = target (x) quat*``. Maximising ``|cos(d/2) rel_w + sin(d/2) rel_z|``
    gives ``d = 2*atan2(rel_z, rel_w)`` — the swing-twist twist of `rel` about
    world Z. It has no singularity at any attitude; it degenerates only when the
    residual is a 180 deg turn about a horizontal axis, which no tracking loop reaches.
    """
    tw, tx, ty, tz = _unit_quat(target)
    qw, qx, qy, qz_ = _unit_quat(quat)
    rel = quat_mul((tw, tx, ty, tz), (qw, -qx, -qy, -qz_))
    return wrap180(math.degrees(2.0 * math.atan2(rel[3], rel[0])))


def _unit_quat(quat) -> tuple[float, float, float, float]:
    w, x, y, z = (float(c) for c in quat)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def yaw_twist_deg(quat) -> float:
    """How far `quat` is rotated about **world +Z**, in degrees [-180, 180) --
    the swing-twist twist.

    **This is not a heading, and no live path may use it as one (BUG-058).**
    It is a property of the whole rotation: roll the device about a horizontal
    boresight, turning not at all, and this changes degree for degree. It was
    adopted as the heading term in BUG-051 and was wrong for that job in the
    same way `quat_yaw_deg` had been, just on a different axis. Its legitimate
    use is a *residual* between two attitudes -- "how much pure yaw separates
    these two quats" -- which is `graft_yaw_error_deg`. For where something
    points, use `boresight_bearing_deg`; for where north is, use
    `magnetic_north_bearing_deg`.

    Identical to ``-graft_yaw_error_deg((1,0,0,0), quat)`` (pinned by a test);
    written out because it is the primitive and that function is the loop form.

    Two properties `quat_yaw_deg` lacks, both of which the residual form needs:

    * **No singularity.** ZYX yaw is ill-conditioned as its pitch approaches
      +-90 deg, and this device's SFLP body frame has **X = Up**, so ZYX pitch
      is the elevation of the *structural up axis* -- held upright, the normal
      grip, it sits ~87 deg and ZYX yaw is nearly degenerate. That is the whole
      operating envelope, not an edge case.
    * **Exactly additive under `graft_yaw`.** ``yaw_twist_deg(graft_yaw(q, d))
      == wrap180(yaw_twist_deg(q) + d)`` for every `q`, `d`, because pre-
      multiplying by ``qz(d)`` maps ``atan2(z, w) -> atan2(z, w) + d/2``. ZYX
      yaw is only approximately additive, and least so exactly where this
      device lives.
    """
    w, _, _, z = _unit_quat(quat)
    return wrap180(math.degrees(2.0 * math.atan2(z, w)))


def tilt_compensated_heading(
    quat: tuple[float, float, float, float],
    mag_ut: tuple[float, float, float],
) -> float:
    """Azimuth in degrees [0,360) of the magnetic field vector **in the quat's
    own world frame**, measured CCW from world +X (`atan2(m_y, m_x)`).

    Read the name with care: this is where NORTH lies, not where the DEVICE
    points, and it is CCW while every compass bearing in this module
    (`boresight_view_deg`, `magnetic_north_bearing_deg`, `absolute_heading`) is
    CW. Because the field is fixed in the world, the value is independent of
    how the device is held -- rotating the device cannot change it. It is
    therefore *not* a device heading in any attitude, and using it as one cost
    BUG-058; `magnetic_north_bearing_deg` is the same measurement in the
    module's compass convention and is what the fusion consumes.

    Kept because the deprecated desktop panel's compass draws it directly."""
    r = quat_to_matrix(*quat)
    m_world = r @ np.array(mag_ut, dtype=np.float64)
    heading = np.degrees(np.arctan2(m_world[1], m_world[0]))
    return float(heading % 360.0)


# Horizontal field below this is too short to point anywhere: no north, hence no
# bearing. Only reachable at a magnetic pole or inside a shield -- ~19 uT of
# horizontal field is typical at mid-latitudes -- but a silent atan2(0, 0) = 0
# would read as "due north" rather than "unknown".
MIN_HORIZONTAL_FIELD_UT = 0.5

# A compass bearing for the boresight stops existing as it swings to vertical
# (aim at the ceiling and the device has no forward direction to name), and its
# noise grows as 1/cos(elevation) on the way there. Same singularity
# `boresight_view_deg` documents for its roll, and the same margin the web UI
# already warns at (`ORIENTATION_SINGULARITY_MARGIN_DEG`).
BEARING_SINGULARITY_MARGIN_DEG = 10.0


def magnetic_north_bearing_deg(quat, mag_ut) -> float | None:
    """Compass bearing (CW, [-180, 180)) at which magnetic north lies **in the
    frame `quat` is expressed in** -- i.e. how far that frame's +X datum has
    ended up from north. None when the field has no usable horizontal
    component (`MIN_HORIZONTAL_FIELD_UT`).

    This is the whole yaw-drift measurement, and the *only* magnetometer term
    the fusion needs. SFLP is 6-axis, so its world +X is an arbitrary datum
    fixed at boot that then drifts; this says by how much, at any attitude,
    with no singularity -- the field is a world-fixed vector, so rotating the
    device cannot change it, and nothing here reads a body axis.

    Exactly additive under `graft_yaw`, with the sign that makes the fixed
    point trivial: ``magnetic_north_bearing_deg(graft_yaw(q, d), m) ==
    wrap180(<this> - d)``, so grafting by exactly this value lands the fused
    frame's +X on magnetic north and leaves nothing to iterate.

    `mag_ut` must already be calibrated and axis-corrected (`MagCalibration.apply`
    then `AXIS_CONVENTION`) -- this function only reads a direction, it cannot
    tell a hard-iron offset from a real field."""
    r = quat_to_matrix(*quat)
    m_world = r @ np.asarray(mag_ut, dtype=np.float64)
    if float(math.hypot(m_world[0], m_world[1])) < MIN_HORIZONTAL_FIELD_UT:
        return None
    return wrap180(math.degrees(math.atan2(-float(m_world[1]), float(m_world[0]))))


def boresight_bearing_deg(quat) -> float | None:
    """Compass bearing [0,360) of the ToF optical axis in the frame `quat` is
    expressed in, or None within `BEARING_SINGULARITY_MARGIN_DEG` of vertical
    where a bearing does not exist. `boresight_view_deg`'s azimuth, gated.

    Exactly additive under `graft_yaw` (a world-Z rotation of `d` swings the
    boresight's bearing by `-d`) at every attitude it returns a number for --
    the property `quat_yaw_deg` (BUG-051) and then `yaw_twist_deg` (BUG-058)
    were each adopted for and each lacked."""
    azimuth, elevation, _roll = boresight_view_deg(quat)
    if 90.0 - abs(elevation) < BEARING_SINGULARITY_MARGIN_DEG:
        return None
    return azimuth


def absolute_heading(quat, mag_ut) -> float | None:
    """Drift-free magnetic compass bearing of the BORESIGHT, in degrees
    [0,360) -- where the sensor is aimed, referenced to magnetic north instead
    of to the SFLP world frame's drifting +X datum. None when either half is
    undefined: the boresight within `BEARING_SINGULARITY_MARGIN_DEG` of
    vertical, or no horizontal field.

    It is the difference of two bearings read in the same frame -- the
    boresight's, and magnetic north's -- so `quat`'s yaw datum cancels
    identically and the answer is drift-free by construction rather than by
    stripping anything.

    **Do not reintroduce a "strip the yaw, then de-tilt" formulation.** Both
    predecessors were that shape and both failed the same way, on the axis
    their regression test happened not to sweep:

      * `quat_yaw_deg` (ZYX yaw) -- BUG-051, 18.4 deg of systematic error at a
        true bearing of 45 deg, exact at 0/90/180/270 so the cardinal-bearing
        check passed.
      * `yaw_twist_deg` (swing-twist about world Z) -- BUG-058, and its test
        DID sweep off-axis bearings; what it held fixed was ROLL. World-Z twist
        absorbs roll about a horizontal boresight one-for-one, so with the
        device aimed at a fixed bearing and simply rolled in the hand, this
        function tracked the roll at slope -0.978 (`captures/NorthFacingRoll.bin`,
        154 deg of reported heading swing for 0 deg of real turn).

    The lesson under both: a scalar "yaw" extracted from an attitude is a
    property of the whole rotation, not of the direction the sensor points. Ask
    for the bearing of the axis you actually mean."""
    north = magnetic_north_bearing_deg(quat, mag_ut)
    bearing = boresight_bearing_deg(quat)
    if north is None or bearing is None:
        return None
    return (bearing - north) % 360.0


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


def display_rotation(quat) -> np.ndarray | None:
    """Body -> Open3D-CV-world rotation used to gravity-align the live display,
    or None when there is no orientation yet (ToF-only session).

    This is the one composed mapping from `docs/coordinate-frames.md`
    (`T_WORLD_TO_CV @ R @ T_CV_TO_BODY`) — the same matrix the desktop panel
    applied to its orbit-mode cloud (`panel.py:1337`) and the same one shipped to
    the client as the gizmo's `rot`. Never re-derive it locally.

    Lives here rather than in `web.py` (where it was until 2026-07-31) so the
    thumbnail renderer can use it: `thumbs.py` must not import `web`, which
    imports the whole server, the SLAM stack and this module — a cycle. `web.py`
    re-exports it, so every existing caller is unchanged."""
    if quat is None:
        return None
    return T_WORLD_TO_CV @ quat_to_matrix(*quat) @ T_CV_TO_BODY


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


def ir_gravity_angle_deg(quat: tuple[float, float, float, float]) -> float:
    """In-plane rotation to APPLY to the IR image so its content stands upright
    against gravity. **Continuous** (not snapped), in degrees, [-180, 180),
    CCW-positive **as seen on screen** — i.e. `np.rot90`'s sense (row 0 renders at
    the top), so a CSS `transform: rotate()` needs the NEGATED value because CSS
    rotates clockwise.

    Method:
      1. Rotate the world-gravity vector [0, 0, -1] (SFLP Z-up world frame)
         into the sensor body frame: g_body = R.T @ [0, 0, -1].
      2. The image plane is the CV XY plane. CV Right = SFLP Y. CV Down = SFLP -X.
      3. The in-plane gravity components are gx = g_body[1] and gy = -g_body[0].
      4. `atan2(gx, gy)` is **where gravity currently sits**, 0° when it already
         points at +image-down and increasing counter-clockwise on screen.
      5. **Negate it** — that is the correction, not the measurement. Gravity
         sitting φ CCW of down means the content must turn φ CLOCKWISE to bring
         it back down.

    That negation is the fix for a sign inversion inherited from `panel.py`
    (BUG-026 follow-up, 2026-07-29): the pane rotated the wrong way, so instead of
    holding still the content counter-rotated at **twice** the board's rate.

    Beware how easily this hides. Returning `+atan2` instead is invisible in the
    two checks you would naturally reach for: at 180° a sign flip is a no-op
    (−180 ≡ +180), and a 90° turn swaps the image's width/height either way. It
    is pinned now by `test_ir_gravity_angle_matches_the_point_cloud_rotation`,
    which derives the expected value from the *verified* cloud path — note that
    `T_WORLD_TO_CV @ R @ T_CV_TO_BODY` rotates points in the CV frame where **Y
    points down**, so a positive rotation there is CLOCKWISE on screen, the exact
    trap that produced the inversion.
    """
    r = quat_to_matrix(*quat)   # body → world
    gravity_world = np.array([0.0, 0.0, -1.0])
    g_body = r.T @ gravity_world   # sensor body frame gravity vector
    gx, gy = float(g_body[1]), float(-g_body[0])   # in-plane components
    return wrap180(-math.degrees(math.atan2(gx, gy)))


def ir_gravity_rot(quat: tuple[float, float, float, float]) -> int:
    """Number of CCW 90° turns (0–3) to apply to the raw IR image so its "down"
    approaches physical gravity — i.e. `ir_gravity_angle_deg` snapped to the
    nearest quarter turn, as an `np.rot90` count.

    Pixel-exact and free, but it can be up to 45° off. Pair it with
    `ir_gravity_residual_deg` to cover the rest without resampling the image.
    """
    return int(round(ir_gravity_angle_deg(quat) / 90.0)) % 4


def ir_gravity_residual_deg(quat: tuple[float, float, float, float]) -> float:
    """The part of the gravity roll that `ir_gravity_rot`'s quarter-turn snap
    leaves behind: `angle - 90*steps`, wrapped to (-45, 45]. CCW-positive.

    Rotating the IR pane by the snap alone makes it agree with the
    continuously-aligned point cloud only near multiples of 90°; at, say, 40° of
    roll the snap is zero and the pane does not move at all while the cloud tilts
    the full 40°. Applying this residual on top (client-side, as a CSS transform,
    so the 54x42 image is never resampled) closes that gap.
    """
    angle = ir_gravity_angle_deg(quat)
    return wrap180(angle - 90.0 * (int(round(angle / 90.0)) % 4))


# Magnetometer axes -> SFLP body frame. TWO factors, deliberately written out
# rather than folded into one literal, because they are established by different
# evidence and only one of them has ever been wrong (BUG-059):
#
#   MAG_MOUNT_ROTATION  the LIS2MDL's mounting relative to the LSM6DSV16X. A
#       proper rotation (det +1). Corroborated without any ground truth: over a
#       360 deg room sweep, magnetic north's bearing must be CONSTANT, and this
#       is the only assignment that holds it (sd 6.1 deg; every other signed
#       permutation scatters it 75-79 deg).
#
#   -1  the FIELD-DIRECTION sign. As delivered, the calibrated vector points
#       ANTI-PARALLEL to Earth's field: it came out 70-72 deg ABOVE the horizon
#       on every capture, and in the northern hemisphere the field points that
#       far BELOW it. That put magnetic north 180 deg out, so a device aimed
#       north reported south (BUG-059, owner). Same class as
#       `gravity_body_from_imu_raw` negating the accelerometer's sensed
#       reaction: a convention, not a rotation.
#
# The product therefore has det -1, which is correct and is NOT a bug to "fix"
# back -- a sign convention composed with a rotation is not itself a rotation.
# The dip test is the check that matters and it needs no compass: rotate the
# calibrated field into the world frame (world Z is up, verified against the
# accelerometer at (0,0,-1)) and its Z must be NEGATIVE. `host/tools/heading_check.py`
# scores it, and `test_axis_convention_puts_the_field_below_the_horizon` pins it.
MAG_MOUNT_ROTATION = np.diag([1.0, -1.0, -1.0])
MAG_FIELD_SIGN = -1.0
AXIS_CONVENTION = MAG_FIELD_SIGN * MAG_MOUNT_ROTATION
for _m in (MAG_MOUNT_ROTATION, AXIS_CONVENTION):
    _m.setflags(write=False)   # module constants — guard against in-place mutation


class YawFusion:
    """Stateful yaw-only complementary filter: grafts a gated, low-passed
    tilt-compensated magnetometer heading onto the SFLP quaternion. Tilt is
    taken from SFLP unchanged; only heading is corrected.

    What it steers on is `magnetic_north_bearing_deg` -- the angle between this
    quat's world +X datum and magnetic north -- and nothing else. That is the
    drift being corrected, measured directly; the filter never forms, and so
    cannot mis-form, a "device heading" (BUG-058). It reads no body axis, so it
    has no attitude singularity: `gated:no-field` fires only if the horizontal
    field itself vanishes.

    There is no gimbal gate (removed, BUG-051). There used to be one -- freeze
    within `gimbal_margin_deg` of |ZYX pitch| = 90 -- and it was not defending
    the *filter*, it was defending this class's own use of `quat_yaw_deg`. On a
    device whose body X is Up, ZYX pitch is the elevation of the structural up
    axis, so held upright (the normal grip, measured 87.3 deg) the gate tripped
    permanently and yaw fusion could never run: the UI reported "Gimbal lock"
    while the boresight sat 2.1 deg off horizontal."""

    def __init__(self, tau_s: float = 20.0, calibration: MagCalibration | None = None,
                 anomaly_frac: float = 0.3, motion_rate_dps: float = 40.0):
        self.tau_s = float(tau_s)
        self.cal = calibration
        self.anomaly_frac = float(anomaly_frac)
        self.motion_rate_dps = float(motion_rate_dps)
        self._delta = 0.0
        self._have_delta = False
        self._last_quat: tuple[float, float, float, float] | None = None
        self._last_t: int | None = None
        self.status = "init"

    # Stream 9 can arrive in short timestamp bursts (especially over UDP).
    # Dividing the small fp16 quaternion step by a sub-12 ms interval turns
    # quantisation/transport jitter into a false angular-rate spike.  The next
    # sample is still compared with the immediately preceding quaternion, so
    # the normal ~30/60 Hz cadence remains fully covered.
    MOTION_MIN_DT_US = 12_000

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
        # gate: fast motion (SFLP quat angular rate as accel-free motion proxy)
        dot = sum(a * b for a, b in zip(prev_quat, quat))
        ang = 2.0 * math.acos(max(0.0, min(1.0, abs(dot))))   # rad between orientations
        if (t_us - prev_t) >= self.MOTION_MIN_DT_US and math.degrees(ang) / dt > self.motion_rate_dps:
            self.status = "gated:motion"
            self._last_t = t_us
            return
        if not is_valid_mag(raw_mag):
            self.status = "gated:invalid-mag"
            self._last_t = t_us
            return
        # calibrate + axis-convention the mag, then anomaly gate on magnitude
        cal_mag = AXIS_CONVENTION @ self.cal.apply(raw_mag)
        mag_norm = float(np.linalg.norm(cal_mag))
        if abs(mag_norm - self.cal.field_ut) > self.anomaly_frac * self.cal.field_ut:
            self.status = "gated:anomaly"
            self._last_t = t_us
            return
        # The correction IS the measurement: how far this quat's world +X datum
        # has drifted from magnetic north. There is no device-heading term and
        # no yaw extracted from the attitude, so there is nothing to go
        # singular and nothing to double-count. The old `heading - yaw` was a
        # difference of two unlike quantities (BUG-058) whose device-bearing
        # terms did not cancel: driven at a known bearing, the fused quat came
        # out MIRRORED, its boresight reading -bearing.
        target = magnetic_north_bearing_deg(quat, tuple(cal_mag))
        if target is None:
            self.status = "gated:no-field"
            self._last_t = t_us
            return
        if not self._have_delta:
            self._delta = target                   # snap on first valid sample
            self._have_delta = True
        else:
            gain = dt / (self.tau_s + dt)
            # first-order low-pass toward the measured datum error; re-wrap so
            # delta stays in [-180, 180) even after many ±180 crossings.
            self._delta = wrap180(self._delta + gain * wrap180(target - self._delta))
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
