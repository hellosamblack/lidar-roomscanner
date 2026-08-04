"""Unit tests for roomscan.imufusion — the stream-11 complementary orientation filter.

All synthetic: no hardware, no captures. The generators below build *real* 8-byte
stream-11 FIFO records and push them through the real `decode_imu_raw`, so the tests
exercise the same wire path the device will, including the 16-bit quantisation of the
gyro / gravity / gbias words.

The headline test is `test_fused_is_quieter_than_fp16_quat_path`, which reproduces the
under-dithered fp16 quaternion failure the whole module exists to escape.
"""
import math
import struct

import numpy as np
import pytest

from roomscan.imufusion import (
    ACCEL_GATE_FRAC,
    QUAT_REF_RATE_HZ,
    TAU_TILT_S,
    TAU_YAW_S,
    ImuFusion,
    quat_from_gravity,
)
from roomscan.protocol import (
    IMU_RAW_GBIAS_MDPS_PER_LSB,
    IMU_RAW_GRAVITY_MG_PER_LSB,
    IMU_RAW_GY_MDPS_PER_LSB,
    IMU_RAW_TICK_US,
    Frame,
    FrameHeader,
    FrameType,
    ImuFifoTag,
    StreamId,
    decode_imu_raw,
)
from roomscan.sensors import (
    SensorState,
    YawFusion,
    graft_yaw,
    graft_yaw_error_deg,
    quat_mul,
    quat_to_matrix,
    quat_yaw_deg,
    wrap180,
)

GY_LSB_DPS = IMU_RAW_GY_MDPS_PER_LSB / 1000.0        # 0.0175 dps
GRAV_LSB_G = IMU_RAW_GRAVITY_MG_PER_LSB / 1000.0     # 6.1e-5 g
GBIAS_LSB_DPS = IMU_RAW_GBIAS_MDPS_PER_LSB / 1000.0  # 0.004375 dps
RATE_HZ = 480.0
TICKS_PER_SAMPLE = int(round((1e6 / RATE_HZ) / IMU_RAW_TICK_US))   # 96
SAMPLES_PER_FRAME = 16                                # ~480 Hz batched at ~30 fps


# --------------------------------------------------------------------------- builders
def _rec(tag: int, cnt: int, data6: bytes) -> bytes:
    """One verbatim FIFO record: tag byte, 6 data bytes, reserved zero."""
    return bytes([((tag & 0x1F) << 3) | ((cnt & 0x3) << 1)]) + data6 + b"\x00"


def _vec_rec(tag: int, cnt: int, vals: np.ndarray, lsb: float) -> bytes:
    q = np.clip(np.rint(np.asarray(vals, dtype=np.float64) / lsb), -32768, 32767)
    return _rec(tag, cnt, struct.pack("<3h", *(int(v) for v in q)))


def _ts_rec(cnt: int, tick: int) -> bytes:
    return _rec(ImuFifoTag.TIMESTAMP, cnt, struct.pack("<IH", tick & 0xFFFFFFFF, 0))


def make_payload(gyro_dps=None, gravity_g=None, gbias_dps=None, ticks=None,
                 n: int | None = None) -> bytes:
    """Build a stream-11 payload from per-sample physical values.

    Any of the streams may be omitted (None) to simulate a batch that lost words.
    Records are emitted in FIFO order, TAG_CNT cycling 0..3 per sample time, exactly as
    the LSM writes them.
    """
    lengths = [len(a) for a in (gyro_dps, gravity_g, gbias_dps, ticks) if a is not None]
    if n is None:
        n = max(lengths) if lengths else 0
    out = bytearray()
    for i in range(n):
        cnt = i & 0x3
        if ticks is not None and i < len(ticks):
            out += _ts_rec(cnt, int(ticks[i]))
        if gyro_dps is not None and i < len(gyro_dps):
            out += _vec_rec(ImuFifoTag.GY_NC, cnt, gyro_dps[i], GY_LSB_DPS)
        if gravity_g is not None and i < len(gravity_g):
            out += _vec_rec(ImuFifoTag.SFLP_GRAVITY, cnt, gravity_g[i], GRAV_LSB_G)
        if gbias_dps is not None and i < len(gbias_dps):
            out += _vec_rec(ImuFifoTag.SFLP_GBIAS, cnt, gbias_dps[i], GBIAS_LSB_DPS)
    return bytes(out)


def make_batch(**kw):
    return decode_imu_raw(make_payload(**kw))


# ------------------------------------------------------------------------ quat helpers
def q_from_axis_angle(axis, deg):
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    h = math.radians(deg) / 2.0
    s = math.sin(h)
    return (math.cos(h), *(axis * s))


def angle_between(qa, qb) -> float:
    """Rotation angle between two orientations, degrees."""
    d = abs(sum(a * b for a, b in zip(qa, qb)))
    return math.degrees(2.0 * math.acos(min(1.0, d)))


def body_up(q) -> np.ndarray:
    """Where world-up lands in the body frame for orientation q (body -> world)."""
    return quat_to_matrix(*q).T @ np.array([0.0, 0.0, 1.0])


def tilt_error_deg(q_est, q_true) -> float:
    """Angle between the two orientations' gravity directions — yaw-blind."""
    c = float(np.clip(np.dot(body_up(q_est), body_up(q_true)), -1.0, 1.0))
    return math.degrees(math.acos(c))


def fp16_quat(q):
    """Round-trip a quaternion through fp16, i.e. what the SFLP FIFO actually gives us."""
    a = np.asarray(q, dtype=np.float16).astype(np.float64)
    n = np.linalg.norm(a)
    return tuple(a / n)


# A deliberately *general* orientation: all four components are O(0.3-0.8), which is
# where fp16's ulp costs the full ~0.056 deg/step. Near identity three components are
# tiny and fp16 is much finer there — the noise floor really is orientation-dependent,
# and quoting it at identity would flatter the fp16 path.
BASE_Q = q_from_axis_angle((0.4, 0.6, 0.7), 100.0)


def _integrate(q, omega_dps, dt):
    """Reference propagation used to build ground truth (not the code under test)."""
    rv = np.radians(np.asarray(omega_dps, dtype=np.float64)) * dt
    th = float(np.linalg.norm(rv))
    if th < 1e-15:
        return q
    s = math.sin(th / 2.0) / th
    return quat_mul(q, (math.cos(th / 2.0), *(rv * s)))


def run_truth(q0, omega_dps_fn, n_frames, seed=0, gyro_noise=True, gyro_bias=(0, 0, 0),
              gbias_report=None, ticks0=0, fusion=None, yaw_ref_mode="true"):
    """Drive an ImuFusion with synthetic batches; return (fusion, truths, estimates).

    `omega_dps_fn(t)` gives the true body rate. Gyro words get datasheet-grade white
    noise (2.8 mdps/sqrt(Hz) over the 240 Hz Nyquist band) plus `gyro_bias`, and
    `gbias_report` is what the SFLP claims the bias is (defaults to `gyro_bias`, i.e.
    ST sees it perfectly; pass zeros to simulate a residual the filter must reject).
    """
    rng = np.random.default_rng(seed)
    sigma = 0.0028 * math.sqrt(RATE_HZ / 2.0) if gyro_noise else 0.0
    if gbias_report is None:
        gbias_report = gyro_bias
    fusion = fusion if fusion is not None else ImuFusion()
    dt = 1.0 / RATE_HZ
    q = q0
    t = 0.0
    tick = ticks0
    truths, ests = [], []
    for _ in range(n_frames):
        gyro, grav, gb, ticks = [], [], [], []
        for _ in range(SAMPLES_PER_FRAME):
            w = np.asarray(omega_dps_fn(t), dtype=np.float64)
            q = _integrate(q, w, dt)
            t += dt
            tick += TICKS_PER_SAMPLE
            gyro.append(w + np.asarray(gyro_bias) + rng.normal(0.0, sigma, 3))
            grav.append(body_up(q))
            gb.append(np.asarray(gbias_report, dtype=np.float64))
            ticks.append(tick)
        yaw_ref = q if yaw_ref_mode == "true" else (
            fp16_quat(q) if yaw_ref_mode == "fp16" else None)
        fusion.update(make_batch(gyro_dps=gyro, gravity_g=grav, gbias_dps=gb, ticks=ticks),
                      yaw_ref=yaw_ref)
        truths.append(q)
        ests.append(fusion.fused_quat())
    return fusion, truths, ests


# ============================================================================ the tests
def test_unseeded_without_reference_returns_none():
    f = ImuFusion()
    f.update(make_batch(gyro_dps=np.zeros((4, 3)), ticks=np.arange(4) * TICKS_PER_SAMPLE))
    assert f.fused_quat() is None
    assert f.status == "init"


def test_seeds_from_gravity_when_no_quaternion_available():
    f = ImuFusion()
    up = body_up(BASE_Q)
    f.update(make_batch(gyro_dps=np.zeros((4, 3)), gravity_g=np.tile(up, (4, 1)),
                        ticks=np.arange(4) * TICKS_PER_SAMPLE))
    q = f.fused_quat()
    assert q is not None
    assert tilt_error_deg(q, BASE_Q) < 0.05      # tilt right, yaw arbitrary


def test_static_output_holds_still():
    _, truths, ests = run_truth(BASE_Q, lambda t: (0.0, 0.0, 0.0), 300, seed=1)
    assert angle_between(ests[-1], truths[-1]) < 0.05
    steps = [angle_between(ests[i], ests[i - 1]) for i in range(1, len(ests))]
    assert float(np.mean(steps)) < 0.01
    assert float(np.percentile(steps, 95)) < 0.02


def test_fused_is_quieter_than_fp16_quat_path():
    """The reason this module exists.

    Truth is a slow 0.02 deg/s tilt ramp — deliberately UNDER-DITHERED against fp16:
    over one 33 ms frame the orientation moves ~0.0007 deg, ~1.5% of the 0.056 deg fp16
    step, so a whole run of frames quantises to the *identical* level and the firmware's
    16-sample average buys nothing (this is the measured k_eff ~5-of-16 pathology). The
    fp16 path therefore emits a staircase: long ties, then a whole-ulp jump.

    The fused path takes its tilt from the 0.061 mg/LSB SFLP gravity vector instead
    (0.0035 deg/LSB, ~16x finer) and its yaw from the same fp16 quaternion but through a
    1 s low-pass, so it should be quieter on BOTH measures that matter: accuracy against
    truth, and frame-to-frame step (the p95 the project quotes).
    """
    n_frames = 400
    _, truths, ests = run_truth(BASE_Q, lambda t: (0.02, 0.0, 0.0), n_frames, seed=2,
                                yaw_ref_mode="fp16")
    fp16_path = [fp16_quat(q) for q in truths]

    # confirm the premise: the fp16 path really is under-dithered (long exact ties)
    fp16_steps = np.array([angle_between(fp16_path[i], fp16_path[i - 1])
                           for i in range(1, n_frames)])
    tied = float(np.mean(fp16_steps < 1e-9))
    assert tied > 0.5, f"fp16 path is not under-dithered in this fixture (ties={tied:.2f})"

    fused_steps = np.array([angle_between(ests[i], ests[i - 1]) for i in range(1, n_frames)])
    fp16_err = np.array([angle_between(a, b) for a, b in zip(fp16_path, truths)])
    fused_err = np.array([angle_between(a, b) for a, b in zip(ests, truths)])
    fp16_tilt = np.array([tilt_error_deg(a, b) for a, b in zip(fp16_path, truths)])
    fused_tilt = np.array([tilt_error_deg(a, b) for a, b in zip(ests, truths)])

    # 1) tilt accuracy — the axis the SFLP gravity vector actually constrains.
    #    Measured on this fixture: 0.0131 deg (fp16) -> 0.0021 deg (fused), ~6.2x.
    assert fused_tilt.mean() < fp16_tilt.mean() / 4.0
    # 2) full-orientation accuracy improves less, and that is EXPECTED, not a defect:
    #    yaw is still anchored to the same fp16 quaternion (gravity carries no heading),
    #    so roughly half the residual is yaw. Measured ~2.3x.
    assert fused_err.mean() < fp16_err.mean() / 2.0
    # 3) the staircase: the fp16 path sits still and then jumps a whole ulp (~0.056 deg
    #    at this orientation). The fused path never takes a step anywhere near that.
    assert fp16_steps.max() > 0.03
    assert fused_steps.max() < fp16_steps.max() / 5.0


def test_fused_step_noise_far_below_fp16_under_real_motion():
    """The dithered counterpart, sized to reproduce the floor the project measured.

    A gentle 0.5 dps handheld wobble moves the orientation ~0.020 deg per frame — the
    same order as the fp16 step, so the quantiser IS dithered here and the reported
    per-frame step becomes noise around the true one. On this fixture the fp16 path
    reports 0.024 deg/frame mean and 0.060 deg p95, which brackets the 0.018/0.049
    measured on the rig, so the synthetic is in the right regime.

    The metric is step JITTER: how far the reported frame-to-frame step is from the true
    frame-to-frame step. That is the quantity a viewer sees as shimmer.
    """
    n_frames = 400

    def wobble(t):
        return (0.5 * math.sin(2 * math.pi * 1.7 * t),
                0.5 * math.sin(2 * math.pi * 2.3 * t + 1.0),
                0.5 * math.sin(2 * math.pi * 3.1 * t + 2.0))

    _, truths, ests = run_truth(BASE_Q, wobble, n_frames, seed=6, yaw_ref_mode="fp16")
    fp16_path = [fp16_quat(q) for q in truths]

    def steps(path):
        return np.array([angle_between(path[i], path[i - 1]) for i in range(1, n_frames)])

    true_steps, fp16_steps, fused_steps = steps(truths), steps(fp16_path), steps(ests)

    # the fixture is in the measured regime
    assert 0.015 < fp16_steps.mean() < 0.035
    assert 0.04 < np.percentile(fp16_steps, 95) < 0.08

    def jitter(s):
        return float(np.sqrt(((s - true_steps) ** 2).mean()))

    # measured on this fixture: 0.0212 deg (fp16) -> 0.0005 deg (fused), ~40x quieter
    assert jitter(fused_steps) < jitter(fp16_steps) / 20.0
    # and it is not quiet by being wrong: it tracks the true motion, it does not damp it
    assert fused_steps.mean() == pytest.approx(true_steps.mean(), rel=0.05)


# ------------------------------------------------------- the yaw axis (BUG-039)
def _zyx_quat(yaw, pitch, roll):
    return quat_mul(quat_mul(q_from_axis_angle((0, 0, 1), yaw),
                             q_from_axis_angle((0, 1, 0), pitch)),
                    q_from_axis_angle((1, 0, 0), roll))


#: An attitude with body **X** 4 deg off world +Z. SFLP body X is Up, so this is
#: where the device actually sits (86.2 deg ZYX pitch measured on
#: `captures/stationary_stream11_20260728_190311.bin`) -- and it is 4 deg from the
#: ZYX gimbal lock at 90 deg. All four components stay O(0.1-0.7), so the fp16
#: reference quantisation below is realistic rather than flattered.
NEAR_LOCK_Q = _zyx_quat(37.0, 86.0, 23.0)


def _legacy_correct_yaw(self, yaw_ref, dt):
    """The pre-BUG-039 heading error term, verbatim: ZYX yaw, i.e. body Z."""
    from roomscan.sensors import graft_yaw, quat_yaw_deg, wrap180
    if yaw_ref is None:
        return
    err = wrap180(quat_yaw_deg(yaw_ref) - quat_yaw_deg(self._q))
    self._q = graft_yaw(self._q, dt / (self.tau_yaw_s + dt) * err)


def _heading_errors(q0, seed, **kw):
    _, truths, ests = run_truth(q0, lambda t: (0.0, 0.0, 0.0), 300, seed=seed,
                                yaw_ref_mode="fp16", **kw)
    return np.array([abs(graft_yaw_error_deg(t, e)) for t, e in zip(truths, ests)])


def test_yaw_loop_measures_heading_about_world_z_not_body_z(monkeypatch):
    """THE AXIS TEST (BUG-039).

    Held still at the device's real attitude, the only thing wrong with the
    stream-9 reference is its fp16 tilt quantisation -- a *static* error, so it
    biases the loop rather than averaging out (which is why the rig's 1.69 deg was
    insensitive to `tau_yaw`: a wrong measurement, not a mistuned gain).

    4 deg from ZYX gimbal lock that tilt quantisation reads as a large apparent
    body-Z yaw, and a loop nulling it grafts that misreading on as REAL heading
    error. The expected size is derived from the fixture, not hard-coded: it is
    exactly the ZYX-yaw difference between the fp16 reference and the truth.
    """
    misreading = abs(wrap180(quat_yaw_deg(fp16_quat(NEAR_LOCK_Q))
                             - quat_yaw_deg(NEAR_LOCK_Q)))
    assert misreading > 0.1, "fixture no longer exercises the singularity"

    fixed = _heading_errors(NEAR_LOCK_Q, seed=11)
    monkeypatch.setattr(ImuFusion, "_correct_yaw", _legacy_correct_yaw)
    legacy = _heading_errors(NEAR_LOCK_Q, seed=11)

    # the body-Z term converges to its own misreading -- it nulls the wrong thing
    assert legacy[-1] == pytest.approx(misreading, rel=0.05)
    # the world-Z term does not see a heading error that is not there
    assert float(np.percentile(fixed, 95)) < 0.01
    assert float(np.percentile(fixed, 95)) < float(np.percentile(legacy, 95)) / 20.0


def test_yaw_loop_still_follows_a_real_heading_offset(monkeypatch):
    """The other half: the fix must not have made the loop deaf. A genuine 30 deg
    heading offset (not a multiple of 90, where a sign error hides) is still
    tracked out, at the same attitude, and its sign is not inverted."""
    ref = graft_yaw(NEAR_LOCK_Q, 30.0)
    f = ImuFusion()
    for k in range(0, 400, 8):
        f.update(make_batch(gyro_dps=np.zeros((8, 3)),
                            gravity_g=np.tile(body_up(NEAR_LOCK_Q), (8, 1)),
                            ticks=np.arange(k, k + 8) * TICKS_PER_SAMPLE), yaw_ref=ref)
    assert graft_yaw_error_deg(ref, f.fused_quat()) == pytest.approx(0.0, abs=0.05)
    assert tilt_error_deg(f.fused_quat(), NEAR_LOCK_Q) < 0.05      # tilt untouched


def test_yaw_axis_fix_is_a_no_op_far_from_gimbal_lock(monkeypatch):
    """The control. At a level attitude the ZYX misreading scales by tan(pitch) and
    all but vanishes, so the two terms agree -- which is exactly what the
    before/after ensemble showed (the two zero-pitch captures came out
    bit-identical; the 86 deg one moved 100x). This pins the defect as a frame
    error at THIS device's attitudes, not a general retune."""
    level = _zyx_quat(37.0, 5.0, 23.0)
    fixed = _heading_errors(level, seed=12)
    monkeypatch.setattr(ImuFusion, "_correct_yaw", _legacy_correct_yaw)
    legacy = _heading_errors(level, seed=12)
    assert float(np.percentile(fixed, 95)) == pytest.approx(
        float(np.percentile(legacy, 95)), abs=0.005)


# --------------------------------------------------- rate-aware yaw crossover (Task 8)
def test_quat_ref_rate_hz_default_reproduces_shipped_tau_yaw_exactly():
    """The default is the coupled 30 Hz case: `tau_yaw_s` must come out
    EXACTLY equal to the shipped `TAU_YAW_S` constant (sqrt(30/30) == 1.0
    exactly in floating point), i.e. this is a no-op at the shipped
    configuration."""
    f = ImuFusion()
    assert f.quat_ref_rate_hz == QUAT_REF_RATE_HZ
    assert f.tau_yaw_s == TAU_YAW_S


def test_explicit_default_rate_matches_the_implicit_default():
    f_implicit = ImuFusion()
    f_explicit = ImuFusion(quat_ref_rate_hz=QUAT_REF_RATE_HZ)
    assert f_explicit.tau_yaw_s == f_implicit.tau_yaw_s == TAU_YAW_S


def test_higher_reference_rate_shrinks_tau_yaw():
    """A reference that arrives faster gets more independent samples to
    average over the same real-time window, so the noise-optimal crossover
    (tau* ~ 1/sqrt(f_ref)) shrinks -- the loop can afford to trust the gyro
    for less real time and still land at the same noise floor."""
    f = ImuFusion(quat_ref_rate_hz=90.0)
    expected = TAU_YAW_S * math.sqrt(QUAT_REF_RATE_HZ / 90.0)
    assert f.tau_yaw_s == pytest.approx(expected, rel=1e-9)
    assert f.tau_yaw_s < TAU_YAW_S


def test_lower_reference_rate_grows_tau_yaw():
    f = ImuFusion(quat_ref_rate_hz=10.0)
    expected = TAU_YAW_S * math.sqrt(QUAT_REF_RATE_HZ / 10.0)
    assert f.tau_yaw_s == pytest.approx(expected, rel=1e-9)
    assert f.tau_yaw_s > TAU_YAW_S


def test_set_quat_ref_rate_hz_live_update_does_not_reset_filter_state():
    """A live IMU/env rate change (Task 7) must recompute ONLY `tau_yaw_s` --
    the estimate, status and sample/batch counters are untouched. A rate
    change is not a reset."""
    f = _seeded()
    before_q = f.fused_quat()
    before_status = f.status
    before_samples = f.samples
    before_batches = f.batches

    f.set_quat_ref_rate_hz(90.0)

    assert f.quat_ref_rate_hz == 90.0
    assert f.tau_yaw_s == pytest.approx(TAU_YAW_S * math.sqrt(QUAT_REF_RATE_HZ / 90.0),
                                        rel=1e-9)
    assert f.fused_quat() == before_q
    assert f.status == before_status
    assert f.samples == before_samples
    assert f.batches == before_batches


def test_yaw_gain_reproduces_shipped_convergence_at_default_rate_but_differs_at_higher_rate():
    """Step 4's headline claim: the effective yaw gain changes with the
    reference rate (a real behavioral difference, not just a different
    number sitting unused), while the coupled-30 Hz path stays numerically
    unchanged (proven separately by the bit-identical tests above).

    The FIRST batch seeds directly from `yaw_ref` (see `ImuFusion._seed`),
    which would make any single-reference comparison bit-identical
    regardless of `tau_yaw_s` -- so this seeds from GRAVITY ONLY (no
    reference on frame 0, leaving yaw arbitrary) and only starts feeding
    `ref` from the second batch on, which is what actually exercises the
    crossover gain."""
    ref = graft_yaw(NEAR_LOCK_Q, 30.0)

    def converge(quat_ref_rate_hz):
        f = ImuFusion(quat_ref_rate_hz=quat_ref_rate_hz)
        f.update(make_batch(gyro_dps=np.zeros((8, 3)),
                            gravity_g=np.tile(body_up(NEAR_LOCK_Q), (8, 1)),
                            ticks=np.arange(0, 8) * TICKS_PER_SAMPLE))   # gravity-only seed
        err = None
        for k in range(8, 160, 8):
            f.update(make_batch(gyro_dps=np.zeros((8, 3)),
                                gravity_g=np.tile(body_up(NEAR_LOCK_Q), (8, 1)),
                                ticks=np.arange(k, k + 8) * TICKS_PER_SAMPLE), yaw_ref=ref)
            err = abs(graft_yaw_error_deg(ref, f.fused_quat()))
        return err

    err_30 = converge(30.0)
    err_90 = converge(90.0)
    assert err_90 < err_30 * 0.9      # meaningfully faster convergence at the higher rate


def test_tracks_known_rotation_one_to_one():
    """A pure, known 20 dps rotation about body Y for 5 s: the filter must follow it
    1:1, not lag it or scale it."""
    _, truths, ests = run_truth(BASE_Q, lambda t: (0.0, 20.0, 0.0), 150, seed=3,
                                gyro_noise=False)
    travelled = angle_between(ests[-1], ests[0])
    assert travelled == pytest.approx(angle_between(truths[-1], truths[0]), abs=0.1)
    lag = [angle_between(e, t) for e, t in zip(ests, truths)]
    assert max(lag) < 0.2                       # bounded lag, whole run


def test_constant_residual_gyro_bias_does_not_walk_tilt_away():
    """1 dps of bias the SFLP gbias does NOT report. Free integration would reach 20 deg
    over the 20 s run; the gravity correction must hold it at roughly bias * tau_tilt and
    then stop growing."""
    bias = (1.0, 0.0, 0.0)
    _, truths, ests = run_truth(BASE_Q, lambda t: (0.0, 0.0, 0.0), 600, seed=4,
                                gyro_bias=bias, gbias_report=(0.0, 0.0, 0.0))
    err = np.array([tilt_error_deg(e, t) for e, t in zip(ests, truths)])
    predicted = 1.0 * TAU_TILT_S                      # bias * tau, the P-only offset
    assert err[-1] == pytest.approx(predicted, abs=0.25)
    assert err.max() < 1.0                            # vs 20 deg of free integration
    # steady state, not a slow walk: second half no worse than the first half's tail
    assert err[-1] <= err[len(err) // 2] + 0.05


def test_sflp_gbias_is_subtracted():
    """Same bias, but reported by SFLP_GBIAS — the offset should essentially vanish,
    which is the whole reason the firmware batches tag 0x16."""
    _, truths, ests = run_truth(BASE_Q, lambda t: (0.0, 0.0, 0.0), 600, seed=4,
                                gyro_bias=(1.0, 0.0, 0.0))
    assert tilt_error_deg(ests[-1], truths[-1]) < 0.05


def test_gravity_gate_rejects_linear_acceleration():
    f = _seeded()
    assert f.status == "active"
    bad = body_up(BASE_Q) * (1.0 + 3 * ACCEL_GATE_FRAC)     # 15% over 1 g
    f.update(make_batch(gyro_dps=np.zeros((8, 3)), gravity_g=np.tile(bad, (8, 1)),
                        ticks=np.arange(16, 24) * TICKS_PER_SAMPLE), yaw_ref=BASE_Q)
    assert f.status == "gated:accel"
    assert tilt_error_deg(f.fused_quat(), BASE_Q) < 0.05    # estimate unharmed


# --------------------------------------------------------------- degraded / odd inputs
def _seeded():
    """Two batches: the first only seeds (no previous state to propagate from), the
    second is the first real propagate+correct pass."""
    f = ImuFusion()
    for k in (0, 8):
        f.update(make_batch(gyro_dps=np.zeros((8, 3)),
                            gravity_g=np.tile(body_up(BASE_Q), (8, 1)),
                            ticks=np.arange(k, k + 8) * TICKS_PER_SAMPLE), yaw_ref=BASE_Q)
    assert f.status == "active"
    return f


def test_batch_with_no_gyro_words_degrades_without_crashing():
    f = _seeded()
    before = f.fused_quat()
    f.update(make_batch(gravity_g=np.tile(body_up(BASE_Q), (8, 1)),
                        ticks=np.arange(16, 24) * TICKS_PER_SAMPLE), yaw_ref=BASE_Q)
    assert f.status == "degraded:no-gyro"
    assert angle_between(f.fused_quat(), before) < 0.05


def test_short_batch_and_missing_timestamps():
    f = _seeded()
    f.update(make_batch(gyro_dps=np.zeros((3, 3)),
                        gravity_g=np.tile(body_up(BASE_Q), (3, 1))), yaw_ref=BASE_Q)
    assert f.status == "active"
    assert f.last_dt_total_s == pytest.approx(3.0 / RATE_HZ, rel=1e-6)   # nominal fallback
    # a single timestamp word is also not a usable time base on its own
    f.update(make_batch(gyro_dps=np.zeros((5, 3)),
                        gravity_g=np.tile(body_up(BASE_Q), (5, 1)),
                        ticks=[9_999]), yaw_ref=BASE_Q)
    assert f.fused_quat() is not None


def test_dropped_frames_report_a_gap_and_clamp_dt():
    f = _seeded()
    far = np.arange(8) * TICKS_PER_SAMPLE + int(2.0e6 / IMU_RAW_TICK_US)   # +2 s
    f.update(make_batch(gyro_dps=np.zeros((8, 3)),
                        gravity_g=np.tile(body_up(BASE_Q), (8, 1)), ticks=far),
             yaw_ref=BASE_Q)
    assert f.status == "degraded:gap"
    assert f.last_gap_s == pytest.approx(2.0, abs=0.05)
    assert f.last_dt_total_s < 0.2          # per-sample clamp kept the batch bounded


def test_timestamp_wrap_is_a_non_event():
    """The 32-bit tick wraps every ~26 h. A batch straddling the wrap must integrate the
    same as the identical batch far from it."""
    omega = np.tile([0.0, 10.0, 0.0], (SAMPLES_PER_FRAME, 1))
    grav = np.tile(body_up(BASE_Q), (SAMPLES_PER_FRAME, 1))
    results = []
    for base in (1_000_000, 0xFFFFFFFF - 5 * TICKS_PER_SAMPLE):
        f = ImuFusion()
        t0 = (np.arange(SAMPLES_PER_FRAME) * TICKS_PER_SAMPLE + base) & 0xFFFFFFFF
        t1 = (t0 + SAMPLES_PER_FRAME * TICKS_PER_SAMPLE) & 0xFFFFFFFF
        f.update(make_batch(gyro_dps=omega, gravity_g=grav, ticks=t0), yaw_ref=BASE_Q)
        f.update(make_batch(gyro_dps=omega, gravity_g=grav, ticks=t1), yaw_ref=BASE_Q)
        assert f.status == "active"
        results.append(f.fused_quat())
    assert angle_between(results[0], results[1]) < 1e-6


def test_out_of_order_timestamps_do_not_explode():
    f = _seeded()
    jumbled = [500, 100, 900, 300, 1200, 200, 1500, 50]     # nonsense ordering
    f.update(make_batch(gyro_dps=np.full((8, 3), 5.0),
                        gravity_g=np.tile(body_up(BASE_Q), (8, 1)), ticks=jumbled),
             yaw_ref=BASE_Q)
    q = f.fused_quat()
    assert np.isfinite(q).all()
    assert float(np.linalg.norm(q)) == pytest.approx(1.0, abs=1e-9)


def test_output_is_a_unit_quaternion_throughout():
    rng = np.random.default_rng(7)

    def wobble(t):
        return (60.0 * math.sin(3.0 * t), -40.0 * math.cos(2.0 * t), 25.0 * math.sin(t))

    f, _, ests = run_truth(BASE_Q, wobble, 200, seed=5)
    for q in ests:
        assert float(np.linalg.norm(q)) == pytest.approx(1.0, abs=1e-9)
    # and after a reset + garbage-ish input it is still normalised
    f.reset()
    assert f.fused_quat() is None
    for i in range(5):
        f.update(make_batch(gyro_dps=rng.normal(0, 2000, (8, 3)),
                            gravity_g=np.tile(body_up(BASE_Q), (8, 1)),
                            ticks=np.arange(8) * TICKS_PER_SAMPLE + i * 1000),
                 yaw_ref=BASE_Q)
        assert float(np.linalg.norm(f.fused_quat())) == pytest.approx(1.0, abs=1e-9)


def test_quat_from_gravity_handles_degenerate_vectors():
    assert quat_from_gravity(np.zeros(3)) == (1.0, 0.0, 0.0, 0.0)
    assert quat_from_gravity(np.array([0.0, 0.0, 1.0])) == (1.0, 0.0, 0.0, 0.0)
    q = quat_from_gravity(np.array([0.0, 0.0, -1.0]))       # antiparallel
    assert float(np.linalg.norm(q)) == pytest.approx(1.0, abs=1e-9)


# =============================================================== SLAM non-regression
def _frame(stream_id: int, payload: bytes, t_us: int = 123) -> Frame:
    h = FrameHeader(FrameType.DATA, stream_id, 0, 1, t_us, 0, 0, len(payload))
    return Frame(h, payload)


def _quat_frame(q):
    return _frame(StreamId.IMU_QUAT, struct.pack("<4f", *q))


def _raw_frame(n=SAMPLES_PER_FRAME, q=BASE_Q, tick0=0):
    return _frame(StreamId.IMU_RAW,
                  make_payload(gyro_dps=np.zeros((n, 3)), gravity_g=np.tile(body_up(q), (n, 1)),
                               gbias_dps=np.zeros((n, 3)),
                               ticks=np.arange(n) * TICKS_PER_SAMPLE + tick0))


def test_slam_non_regression_default_is_raw_sflp_quat():
    """THE GUARD. SLAM reads SensorState.fused_quat() directly (web.py ~1634 and the
    slam package). With no fusion attached — the default — stream-11 frames must change
    nothing: fused_quat() is bit-for-bit the decoded stream-9 quaternion."""
    st = SensorState()                      # default: fusion=None, imu_fusion=None
    st.feed(_quat_frame(BASE_Q))
    baseline = st.fused_quat()
    assert baseline == tuple(float(v) for v in np.asarray(BASE_Q, dtype=np.float32))
    for i in range(5):
        st.feed(_raw_frame(tick0=i * SAMPLES_PER_FRAME * TICKS_PER_SAMPLE))
    assert st.fused_quat() == baseline       # exact identity, not approximately
    assert st.latest_imu_raw() is not None    # the batches WERE decoded and buffered


def test_slam_non_regression_yawfusion_path_unchanged():
    """Same guard with a YawFusion attached: stream-11 traffic must not perturb the
    existing yaw-corrected output either."""
    kw = dict(tau_s=5.0, calibration=None)
    a, b = SensorState(fusion=YawFusion(**kw)), SensorState(fusion=YawFusion(**kw))
    for i in range(5):
        a.feed(_quat_frame(BASE_Q))
        b.feed(_quat_frame(BASE_Q))
        b.feed(_raw_frame(tick0=i * SAMPLES_PER_FRAME * TICKS_PER_SAMPLE))
    assert a.fused_quat() == b.fused_quat()
    assert a.fusion_status() == b.fusion_status()


def test_opt_in_fusion_actually_takes_over():
    """The flip side of the guard: when explicitly enabled, the output IS the filter's,
    so the default-off test above is proving something."""
    st = SensorState(imu_fusion=ImuFusion())
    st.feed(_quat_frame(BASE_Q))
    for i in range(3):
        st.feed(_raw_frame(q=BASE_Q, tick0=i * SAMPLES_PER_FRAME * TICKS_PER_SAMPLE))
    assert st._imu_fusion.status == "active"
    assert st.fused_quat() == st._imu_fusion.fused_quat()
    st.reset_fusion()
    assert st._imu_fusion.fused_quat() is None
    assert st.fused_quat() == st.latest_quat()      # falls back cleanly after a reset
