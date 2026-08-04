"""Host-side complementary orientation filter over the stream-11 raw LSM6DSV16X FIFO.

WHY THIS EXISTS
---------------
The orientation the live view is gravity-aligned with is the LSM6DSV16X SFLP
*game-rotation quaternion* (stream 9). That quaternion leaves the sensor as **fp16**
words in the FIFO, so at a general orientation (components O(0.5)) one ulp is
2^-11 in a component ~= 2 * 4.88e-4 rad = **0.056 deg of rotation**. That is a hard
floor no amount of host smoothing can get under, because the information is gone
before we see it.

The 2026-07-28 firmware pass averages the ~16-sample FIFO batch per ToF frame, which
should buy sqrt(16); measurement says it only visits k_eff ~5 distinct levels (14-28%
of consecutive frames are EXACTLY tied, ties running up to 18 frames), i.e. the signal
is *under-dithered* - averaging repeated identical quantizer outputs averages nothing.
Measured floor today: ~0.018 deg/frame mean, ~0.049 deg p95, orientation-dependent.

Stream 11 carries the FIFO words the quaternion was cooked from, and those are 16-bit
fixed point, not fp16:

  * GY_NC       17.5 mdps/LSB          -> high-rate rotation increments at 480 Hz
  * SFLP_GBIAS  4.375 mdps/LSB         -> ST's own live gyro-bias estimate
  * SFLP_GRAVITY 0.061 mg/LSB          -> tilt reference; 0.0035 deg/LSB, ~16x finer
                                          in tilt than the fp16 quaternion step
  * XL_NC       0.122 mg/LSB           -> (carried, not used by this filter yet)
  * TIMESTAMP   ~21.7 us/LSB           -> the only usable dt source (see below)

So: **gyro supplies the high-frequency increments, gravity corrects tilt drift, and
stream 9 (optionally already mag-corrected by `YawFusion`) anchors yaw** - the SFLP
gravity vector carries no heading information at all.

WHY THE FRAME HEADER'S t_us CANNOT BE USED FOR dt
-------------------------------------------------
`FrameHeader.t_us` is `HAL_GetTick() * 1000` - a 1 ms-granular millisecond counter
stamped when the *ToF* frame was assembled. At 480 Hz the sample interval is 2.083 ms,
so a 1 ms quantum is a +-24% dt error per sample, and it says nothing about where
inside the ToF frame each IMU sample actually landed. The LSM's own TIMESTAMP words
(21.7 us/LSB, same clock domain as the samples) are the correct time base and are why
the firmware turns TIMESTAMP_EN + DEC_TS_BATCH on for stream 11.

WIRING / SAFETY
---------------
This filter is **opt-in and OFF by default**. `SensorState(imu_fusion=None)` - the
default, and what SLAM constructs today - leaves `SensorState.fused_quat()` exactly as
it was. Only when an `ImuFusion` is explicitly attached *and* it has converged does
`fused_quat()` return this filter's output. Nothing in the web broadcast path, the
display-only `OrientationSmoother`, or `display_rotation` is touched by this module;
integration and the on-rig A/B are deliberately separate work.

RATE AWARENESS (2026-08-04, Task 8)
------------------------------------
`quat_ref_rate_hz` (constructor kwarg, default `QUAT_REF_RATE_HZ` = 30.0) is the
actual arrival rate of the yaw reference (stream 9 / `YawFusion`'s output) a caller
is feeding through `yaw_ref=`. Task 7 lets that rate diverge from the ToF frame rate
(decoupled IMU/env poll rate), so it is now an explicit input rather than the
derivation-only comment constant it used to be -- see `_rate_scaled_tau_yaw_s`. It
recomputes ONLY the yaw crossover (`tau_yaw_s`); `tau_tilt_s` stays pinned to the
gyro's own fixed 480 Hz ODR (`GYRO_RATE_HZ`, unrelated to the decoupled rate) and
every propagation/gain calculation elsewhere stays in real seconds, unchanged. At
the default rate this is an identity (nothing here changes coupled-30 Hz behavior).
`set_quat_ref_rate_hz()` updates it live, without resetting the filter's state.

Frames and conventions are `docs/coordinate-frames.md`'s, unchanged: the state
quaternion is the same **body -> SFLP-world** rotation stream 9 emits, SFLP world is
Z-up, and the display path stays `T_WORLD_TO_CV @ R @ T_CV_TO_BODY`.
"""
from __future__ import annotations

import math

import numpy as np

from .protocol import IMU_RAW_TICK_US, ImuRawBatch
from .sensors import graft_yaw, graft_yaw_error_deg, quat_mul, quat_to_matrix

# --------------------------------------------------------------------------------------
# Tuning constants.  NOT FINAL - see "How these were derived" below.  Every number here
# is a first estimate from datasheet noise densities plus the measured stream-9 floor;
# the Allan-variance run (deliberately out of scope here) is what turns them into
# measured values.  Retune TAU_TILT_S and TAU_YAW_S first, and only then consider
# switching the tilt loop from proportional-only to PI (see KI_BIAS_HZ).
# --------------------------------------------------------------------------------------

#: Gyro angle random walk, deg/sqrt(s).  DS13510 rev 4 rate noise density
#: 2.8 mdps/sqrt(Hz) == 0.0028 deg/sqrt(s).
GYRO_ARW_DEG_PER_SQRT_S = 0.0028

#: 1-sigma tilt error of one SFLP gravity sample, deg.  Two contributions:
#:   quantisation: 0.061 mg/LSB at 1 g -> 0.0035 deg/LSB, sigma = LSB/sqrt(12) = 0.0010 deg
#:   accel noise:  60 ug/sqrt(Hz) (DS13510 rev 4) over the SFLP gravity estimator's
#:                 effective bandwidth (~10 Hz assumed) -> 190 ug -> 0.011 deg
#: The accel term dominates, so 0.011 deg is the number that sets the crossover.
GRAVITY_TILT_SIGMA_DEG = 0.011

#: 1-sigma of one stream-9 frame's orientation, deg.  This is the *measured* per-frame
#: noise of the current fp16 path (0.018 deg/frame mean), not a datasheet number.
QUAT_REF_SIGMA_DEG = 0.018

#: Nominal rates the crossovers were solved at.
GYRO_RATE_HZ = 480.0        # LSM ODR for GY_NC/gravity/gbias batching
QUAT_REF_RATE_HZ = 30.0     # stream 9 arrives once per ToF frame

# --- How these were derived -----------------------------------------------------------
# A first-order complementary filter with time constant tau passes the reference's noise
# low-passed and the gyro's noise high-passed.  For white reference noise of per-sample
# sigma_ref arriving at f_ref, the filter's output carries
#       sigma_out(ref)  = sigma_ref * sqrt(1 / (2 * tau * f_ref))
# while the gyro's angle random walk accumulates, over the tau the loop lets it run free,
#       sigma_out(gyro) = ARW * sqrt(tau)
# The noise-optimal crossover equates the two:
#       tau* = sigma_ref / (ARW * sqrt(2 * f_ref)) ... solved below per axis pair.
#
# TILT (gravity reference, 480 Hz, sigma 0.011 deg):
#       0.011 / sqrt(2*480*tau) = 0.0028 * sqrt(tau)   ->  tau* ~= 0.13 s
#   We do NOT ship 0.13 s.  That optimum assumes the only gravity error is sensor noise,
#   but the dominant real error on a handheld scanner is *linear acceleration* leaking
#   into the gravity vector, which is neither white nor small (0.1 g of hand motion is
#   ~5.7 deg of apparent tilt).  Trusting the gyro longer is the standard trade, and the
#   cost of doing so is a steady-state tilt error of (residual bias) * tau - with ST's
#   gbias subtracted the residual should be well under 0.05 deg/s, giving <0.025 deg at
#   tau = 0.5 s, i.e. still under the 0.056 deg fp16 step we are trying to beat.
TAU_TILT_S = 0.5
#
# YAW (stream-9 reference, 30 Hz, sigma 0.018 deg):
#       0.018 / sqrt(2*30*tau) = 0.0028 * sqrt(tau)    ->  tau* ~= 0.83 s
#   Rounded up to 1.0 s; yaw has no independent absolute reference in this filter (the
#   magnetometer path stays where it is, inside YawFusion, whose output we take as the
#   anchor), so there is no reason to be aggressive.
TAU_YAW_S = 1.0
#
# WHAT TO RETUNE WHEN ALLAN-VARIANCE NUMBERS LAND:
#   * ARW read off the tau=1 s point of the Allan deviation curve replaces
#     GYRO_ARW_DEG_PER_SQRT_S; both tau* expressions above are then re-solved.
#   * Bias instability (the Allan curve's minimum; ST does NOT specify it) is the number
#     we are currently guessing at.  If it turns out to be large relative to what gbias
#     removes, a proportional-only tilt loop will sit at a visible steady-state offset and
#     the loop should become PI - that is what KI_BIAS_HZ is reserved for.
#   * GRAVITY_TILT_SIGMA_DEG assumes a ~10 Hz effective bandwidth for the SFLP gravity
#     estimator, which is an assumption, not a datasheet figure.  A stationary stream-11
#     capture measures it directly.
KI_BIAS_HZ = 0.0   # reserved: integral (bias-learning) term, disabled -> pure complementary

#: Reject the gravity correction when |gravity| strays this far from 1 g - that means the
#: SFLP gravity estimate is contaminated by linear acceleration and is not a tilt
#: reference.  Same shape as YawFusion's magnitude anomaly gate.
ACCEL_GATE_FRAC = 0.05

#: dt sanity clamp, seconds.  Guards against timestamp wrap, out-of-order words, and the
#: gap left by a dropped ToF frame.  Nominal sample interval is 1/480 = 2.083 ms.
DT_MIN_S = 1.0e-4
DT_MAX_S = 0.05

#: A gap larger than this between batches means we missed frames; reported in `status`.
GAP_WARN_S = 0.1

#: SFLP gravity vector sign convention.  ST's SFLP gravity output follows the
#: accelerometer: at rest, the body axis pointing UP reads +1 g.  SFLP world is Z-up, so
#: the predicted body-frame gravity direction is R.T @ [0, 0, +1].  ASSUMPTION - if real
#: stream-11 data shows the vector pointing down instead, flip this to -1.0 and the
#: filter is correct again with no other change.
GRAVITY_SIGN = 1.0

_WORLD_UP = np.array([0.0, 0.0, 1.0])
_TICK_MASK = 0xFFFFFFFF


def _tick_span_us(ticks: np.ndarray) -> np.ndarray:
    """Cumulative microseconds from ticks[0], unwrapping the 32-bit counter.

    The LSM timestamp is a free-running uint32 at ~21.7 us/LSB - it wraps every ~26 h.
    Modular subtraction makes the wrap a non-event: (b - a) & 0xFFFFFFFF is the true
    forward delta as long as the real gap is under 2^32 ticks."""
    if ticks.size == 0:
        return np.zeros(0)
    d = (ticks[1:].astype(np.int64) - ticks[:-1].astype(np.int64)) & _TICK_MASK
    return np.concatenate(([0.0], np.cumsum(d.astype(np.float64)))) * IMU_RAW_TICK_US


def _forward_ticks(prev: int, cur: int) -> int:
    """Forward tick delta prev -> cur, wrap-safe."""
    return int((int(cur) - int(prev)) & _TICK_MASK)


def _quat_from_rotvec(rv: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation vector (radians, any frame) -> unit quaternion [w, x, y, z]."""
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    half = theta / 2.0
    s = math.sin(half) / theta
    return (math.cos(half), float(rv[0]) * s, float(rv[1]) * s, float(rv[2]) * s)


def _normalize_quat(q) -> tuple[float, float, float, float]:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def quat_from_gravity(up_body: np.ndarray) -> tuple[float, float, float, float]:
    """Smallest rotation whose body -> world map sends `up_body` to world +Z.

    Yaw is arbitrary (gravity carries none); used only to seed the filter when no
    stream-9 quaternion is available yet."""
    n = float(np.linalg.norm(up_body))
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    u = np.asarray(up_body, dtype=np.float64) / n
    axis = np.cross(u, _WORLD_UP)
    dot = float(np.clip(np.dot(u, _WORLD_UP), -1.0, 1.0))
    s = float(np.linalg.norm(axis))
    if s < 1e-9:
        # parallel or antiparallel: identity, or a 180 deg flip about any horizontal axis
        return (1.0, 0.0, 0.0, 0.0) if dot > 0 else (0.0, 1.0, 0.0, 0.0)
    return _normalize_quat(_quat_from_rotvec(axis / s * math.acos(dot)))


class ImuFusion:
    """Complementary orientation filter fed by stream-11 raw FIFO batches.

    One `update()` per ToF frame:

      1. **propagate** - integrate every GY_NC word in the batch at its own dt, taken
         from the LSM TIMESTAMP words, with the SFLP gbias vector subtracted;
      2. **correct tilt** - rotate the estimate a fraction `dt/(tau_tilt+dt)` of the way
         toward the SFLP gravity vector (skipped when |gravity| says the sensor is
         accelerating);
      3. **correct yaw** - graft a fraction `dt/(tau_yaw+dt)` of the heading error
         against `yaw_ref` (the stream-9 / YawFusion quaternion) onto the estimate.

    The output is always a unit quaternion in the same body -> SFLP-world convention as
    stream 9, so it is a drop-in for `SensorState.fused_quat()`.

    Thread-safety: none of its own. `SensorState` calls `update()` under its lock, which
    is the same discipline `YawFusion` follows.
    """

    def __init__(self, tau_tilt_s: float = TAU_TILT_S, tau_yaw_s: float = TAU_YAW_S,
                 accel_gate_frac: float = ACCEL_GATE_FRAC,
                 nominal_rate_hz: float = GYRO_RATE_HZ,
                 gravity_sign: float = GRAVITY_SIGN,
                 quat_ref_rate_hz: float = QUAT_REF_RATE_HZ):
        self.tau_tilt_s = float(tau_tilt_s)
        # `tau_yaw_s` as passed in is the crossover value AT `quat_ref_rate_hz`
        # (the coupled 30 Hz default unless a caller overrides both together).
        # The gain actually used is `self.tau_yaw_s`, recomputed by
        # `_rate_scaled_tau_yaw_s()` below -- see the module docstring's RATE
        # AWARENESS section and that method's own docstring for the
        # 1/sqrt(rate) derivation. At the default rate this is an identity
        # (sqrt(30/30) == 1.0 exactly), so this is a no-op change at the
        # shipped coupled-30 Hz configuration.
        self._tau_yaw_base_s = float(tau_yaw_s)
        self._quat_ref_rate_hz = float(quat_ref_rate_hz)
        self.tau_yaw_s = self._rate_scaled_tau_yaw_s()
        self.accel_gate_frac = float(accel_gate_frac)
        self.nominal_dt_s = 1.0 / float(nominal_rate_hz)
        self.gravity_sign = float(gravity_sign)
        self.status = "init"
        # diagnostics (cheap, read by tests and by whatever HUD picks this up later)
        self.samples = 0            # gyro words integrated since reset
        self.batches = 0            # batches accepted since reset
        self.last_dt_total_s = 0.0  # wall time spanned by the last accepted batch
        self.last_gap_s = 0.0       # raw (unclamped) gap since the previous batch
        self._q: tuple[float, float, float, float] | None = None
        self._bias_dps = np.zeros(3)
        self._prev_tick: int | None = None

    # -- public -------------------------------------------------------------------
    def reset(self) -> None:
        """Drop the estimate so the next batch re-seeds from `yaw_ref`/gravity."""
        self.status = "init"
        self.samples = 0
        self.batches = 0
        self.last_dt_total_s = 0.0
        self.last_gap_s = 0.0
        self._q = None
        self._bias_dps = np.zeros(3)
        self._prev_tick = None

    def fused_quat(self) -> tuple[float, float, float, float] | None:
        """Current estimate, or None before the filter has been seeded."""
        return self._q

    @property
    def quat_ref_rate_hz(self) -> float:
        """The yaw-reference update rate (Hz) `tau_yaw_s` is currently scaled
        for -- Task 7's applied/decoupled IMU/env rate, sourced by the caller
        and never re-derived here (see the module docstring's RATE AWARENESS
        section)."""
        return self._quat_ref_rate_hz

    def _rate_scaled_tau_yaw_s(self) -> float:
        """The ONE rate-derived quantity in this filter (Task 8 step 4): the
        yaw crossover time constant, recomputed from `_quat_ref_rate_hz`.

        Module docstring's "How these were derived", YAW section: the
        noise-optimal tau solves

            sigma_ref / sqrt(2 * tau * f_ref) == ARW * sqrt(tau)

        i.e. tau* is proportional to 1/sqrt(f_ref) -- a reference that arrives
        more often gives the loop more independent samples to average over the
        same real-time window, so it can afford to trust the gyro for a
        SHORTER time and still land at the same steady-state noise floor.
        `_tau_yaw_base_s` is the shipped constant (`TAU_YAW_S`) AT
        `QUAT_REF_RATE_HZ` (30 Hz, the historical coupled default) -- scaling
        by sqrt(QUAT_REF_RATE_HZ / f_ref) reproduces it EXACTLY when
        `f_ref == QUAT_REF_RATE_HZ` (coupled 30 Hz, byte-identical to before
        this method existed) and only changes it when the caller's actual
        reference rate differs.

        Deliberately the ONLY rate-derived term: `TAU_TILT_S` is referenced to
        the fixed 480 Hz gyro ODR (`GYRO_RATE_HZ`), not to the decoupled
        IMU/env poll rate, and every dt-based propagation/gain elsewhere
        already runs in real seconds off the LSM's own TIMESTAMP words."""
        if self._quat_ref_rate_hz <= 0:
            return self._tau_yaw_base_s
        return self._tau_yaw_base_s * math.sqrt(QUAT_REF_RATE_HZ / self._quat_ref_rate_hz)

    def set_quat_ref_rate_hz(self, rate_hz: float) -> None:
        """Live update of the yaw-reference rate (Task 7's decoupled IMU/env
        rate can change mid-stream). Recomputes ONLY `tau_yaw_s` -- `_q`,
        `status`, `samples`, `batches` and every other piece of filter state
        are untouched. A rate change is not a reset."""
        self._quat_ref_rate_hz = float(rate_hz)
        self.tau_yaw_s = self._rate_scaled_tau_yaw_s()

    def update(self, batch: ImuRawBatch, yaw_ref=None) -> None:
        """Fold one stream-11 batch in. Never raises on odd input - a short, empty,
        gyro-less or timestamp-less batch degrades (see `status`) instead of crashing,
        because a dropped ToF frame produces exactly those."""
        gravity = self._batch_gravity(batch)
        self._update_bias(batch)

        if self._q is None:
            # First batch: there is no previous state to propagate FROM, and the
            # reference (stream 9's quaternion / the gravity vector) already describes
            # the END of this batch. Seeding and then also integrating this batch's gyro
            # words would double-count it and leave the estimate one frame ahead of
            # truth for good — so seed, bank the timestamp, and start propagating on the
            # next batch.
            if self._seed(yaw_ref, gravity):
                self.status = "seeded"
                self.batches += 1
            else:
                self.status = "init"
            self._remember_tick(batch)
            return

        dts = self._sample_dts(batch)
        self._remember_tick(batch)
        if dts.size:
            self._propagate(batch.gyro_dps[:dts.size], dts)
            self.samples += int(dts.size)
        dt_total = float(dts.sum()) if dts.size else self.nominal_dt_s
        self.last_dt_total_s = dt_total

        gated = not self._correct_tilt(gravity, dt_total)
        self._correct_yaw(yaw_ref, dt_total)
        self._q = _normalize_quat(self._q)
        self.batches += 1
        if dts.size == 0:
            self.status = "degraded:no-gyro"
        elif self.last_gap_s > GAP_WARN_S:
            self.status = "degraded:gap"
        elif gated:
            self.status = "gated:accel"
        else:
            self.status = "active"

    # -- internals ----------------------------------------------------------------
    def _batch_gravity(self, batch: ImuRawBatch) -> np.ndarray | None:
        """The batch's LAST SFLP gravity vector (body frame, g), or None if absent.

        Deliberately NOT the batch mean. The mean is the gravity direction at the batch
        *midpoint* while the propagated state is at the batch *end*, so it drags a
        half-batch (~17 ms) of staleness into the correction: at 20 deg/s that is a
        ~0.3 deg standing lag, which `test_tracks_known_rotation_one_to_one` catches.
        The sqrt(16) the mean would have bought is not needed either — the complementary
        loop already averages the reference over tau_tilt (~15 batches)."""
        if batch.gravity_g.size == 0:
            return None
        return np.asarray(batch.gravity_g[-1], dtype=np.float64) * self.gravity_sign

    def _update_bias(self, batch: ImuRawBatch) -> None:
        """Latch ST's live gyro-bias estimate. Holding the last known value across a
        gbias-less batch is right: bias moves on a thermal timescale, not a frame one."""
        if batch.gbias_dps.size:
            self._bias_dps = np.asarray(batch.gbias_dps[-1], dtype=np.float64)

    def _seed(self, yaw_ref, gravity: np.ndarray | None) -> bool:
        """First-batch initialisation: prefer the stream-9 quaternion (it has yaw), fall
        back to gravity-only (tilt right, yaw arbitrary). Returns False if neither
        is available, leaving the filter unseeded and `fused_quat()` None."""
        if yaw_ref is not None:
            self._q = _normalize_quat(yaw_ref)
            return True
        if gravity is not None and np.linalg.norm(gravity) > 1e-6:
            self._q = quat_from_gravity(gravity)
            return True
        return False

    def _sample_dts(self, batch: ImuRawBatch) -> np.ndarray:
        """Per-gyro-sample dt in seconds, from the LSM's own TIMESTAMP words.

        TAG_CNT pairs a gyro word with the timestamp word of the same sample time, but
        it is only 2 bits, so it cannot be used to align across a decimated or truncated
        batch. Instead: unwrap the timestamps into a monotone time base and resample it
        onto the gyro indices (exact when there is one timestamp per gyro word, a linear
        interpolation of the sample grid when TS batching is decimated). With fewer than
        two timestamps we fall back to the nominal ODR interval.

        dt[0] is measured from the *previous* batch's last timestamp, so the gap a
        dropped ToF frame leaves is real time, not silently skipped time."""
        n = int(batch.gyro_dps.shape[0])
        self.last_gap_s = 0.0
        if n == 0:
            return np.zeros(0)
        ticks = batch.timestamp_ticks
        if ticks.size and self._prev_tick is not None:
            self.last_gap_s = _forward_ticks(
                self._prev_tick, int(ticks[0])) * IMU_RAW_TICK_US * 1e-6
        if ticks.size >= 2:
            span = _tick_span_us(ticks)
            if ticks.size == n:
                t_us = span
            else:
                t_us = np.interp(np.arange(n, dtype=np.float64),
                                 np.linspace(0.0, n - 1.0, ticks.size), span)
            dts = np.empty(n)
            dts[1:] = np.diff(t_us) * 1e-6
            if self._prev_tick is not None:
                dts[0] = self.last_gap_s
            else:
                dts[0] = dts[1] if n > 1 else self.nominal_dt_s
        else:
            # 0 or 1 timestamp words: no usable time base in this batch.
            dts = np.full(n, self.nominal_dt_s)
            if ticks.size == 1 and self._prev_tick is not None:
                dts[0] = self.last_gap_s
        # NaN/inf can only come from a corrupt payload; clamp everything into a sane band
        # so one bad word cannot blow the estimate up.
        dts = np.nan_to_num(dts, nan=self.nominal_dt_s,
                            posinf=DT_MAX_S, neginf=DT_MIN_S)
        return np.clip(dts, DT_MIN_S, DT_MAX_S)

    def _remember_tick(self, batch: ImuRawBatch) -> None:
        if batch.timestamp_ticks.size:
            self._prev_tick = int(batch.timestamp_ticks[-1])

    def _propagate(self, gyro_dps: np.ndarray, dts: np.ndarray) -> None:
        """Integrate body-frame rates onto the estimate, one exact small rotation per
        sample. q is body -> world, so the increment right-multiplies."""
        omega = np.radians(np.asarray(gyro_dps, dtype=np.float64) - self._bias_dps)
        q = self._q
        for w_body, dt in zip(omega, dts):
            q = quat_mul(q, _quat_from_rotvec(w_body * dt))
        self._q = _normalize_quat(q)

    def _correct_tilt(self, gravity: np.ndarray | None, dt: float) -> bool:
        """Rotate the estimate toward the measured gravity direction. Returns False when
        the correction was skipped (no gravity words, or the magnitude gate tripped).

        Derivation of the sign: with R = body -> world, perturbing by a small body-frame
        rotation delta gives R' = R (I + [delta]x), hence
            up_pred' = R'.T z = up_pred - delta x up_pred.
        Setting up_pred' = up_meas and using (u x v) x u = v - u (u.v) gives
            delta = up_meas x up_pred,
        which is perpendicular to up_pred and therefore injects no yaw - exactly what we
        want, since gravity has no heading information."""
        if gravity is None:
            return False
        norm = float(np.linalg.norm(gravity))
        if norm < 1e-6 or abs(norm - 1.0) > self.accel_gate_frac:
            return False
        up_meas = gravity / norm
        up_pred = quat_to_matrix(*self._q).T @ _WORLD_UP
        err = np.cross(up_meas, up_pred)
        s = float(np.linalg.norm(err))
        if s < 1e-12:
            return True
        angle = math.asin(min(1.0, s))
        if float(np.dot(up_meas, up_pred)) < 0.0:      # >90 deg apart
            angle = math.pi - angle
        gain = dt / (self.tau_tilt_s + dt)
        self._q = quat_mul(self._q, _quat_from_rotvec(err / s * (gain * angle)))
        return True

    def _correct_yaw(self, yaw_ref, dt: float) -> None:
        """First-order pull of our heading toward the stream-9 / YawFusion heading.
        Uses `graft_yaw`, i.e. a pure world-Z rotation, so tilt is untouched.

        The error is measured with `graft_yaw_error_deg` — the world-Z twist of the
        residual, which is `graft_yaw`'s own inverse, so the loop nulls exactly the
        quantity it can correct. It used to be a difference of `quat_yaw_deg` (ZYX
        yaw, i.e. the heading of body **Z**), which is the wrong axis for a body
        frame whose **X is Up**: at this device's real attitudes the ZYX
        decomposition sits within a few degrees of gimbal lock, so tilt noise read
        as huge apparent yaw and the loop grafted that noise on as real heading
        error. Measured on `captures/stationary_stream11_20260728_190311.bin`:
        1.689 deg mean / 2.217 deg p95 world-Z heading error before, 0.017 / 0.053
        after — and the old term was insensitive to `tau_yaw`, the signature of a
        wrong measurement rather than a mistuned gain (BUG-039)."""
        if yaw_ref is None:
            return
        err = graft_yaw_error_deg(yaw_ref, self._q)
        gain = dt / (self.tau_yaw_s + dt)
        self._q = graft_yaw(self._q, gain * err)
