"""Frame-to-model SLAM orchestrator. Per-frame: deproject -> predict pose from the
SFLP prior -> raycast model -> point-to-plane ICP -> baro soft-Z -> integrate.
See docs/superpowers/specs/completed/2026-07-10-phase6-slam-design.md sections 3, 5.

`device` (str or o3d.core.Device, default "CPU:0") is resolved once in
__init__ and forwarded to every Open3D piece it owns (TsdfMap, the pinhole
intrinsic, source_cloud) so the whole per-frame pipeline runs on a
single compute device -- CPU today, and unchanged "CUDA:0" once a CUDA build
of Open3D is installed. Any tensor pulled off that device (e.g. the raycast
model's `positions` here) is moved home with `.cpu()` before `.numpy()`.

The ONE deliberate exception is `icp_device` (default "CPU:0"), which selects
the device for ICP's nearest-neighbour index only -- see `__init__`. Measured:
the shipped translation solve is already all-numpy, so a host index is
bit-identical output for less wall time (2026-08-02 CUDA ICP study)."""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import open3d as o3d

from ..colors import normalize as _percentile_normalize
from ..deproject import Deprojector
from .cloud import source_cloud
from .frames import apply_quat_phase as _quat_phase_correct
from .frames import baro_height_m, predict_pose, slerp, world_up
from .intrinsics import pinhole
from .motion import StationarityGate, ZuptDetector
from .odometry import _COND_CAP, _ROT_PRIOR_WEIGHT, register_escalating
from .tsdf import DEFAULT_BLOCK_COUNT, TsdfMap

_G = 9.80665

_MIN_VALID_POINTS = 100
_DEFAULT_MIN_CONFIDENCE = 20.0  # tuned against captures/phase6_motion_ref.bin, see task-quality-report.md

# BUG-037: frames of pressure averaged into the barometric datum before the
# height constraint is allowed to act. A single sample carries ~267 mm RMS of
# apparent altitude (see _apply_baro_z), and the old code froze the datum on
# exactly one -- baking that error in as a constant offset for the whole run.
# 90 frames (~3 s at 30 fps) cuts it to ~28 mm; the captures are parked at the
# start, and the low-pass below makes the residual harmless anyway.
_BARO_REF_FRAMES = 90


#: How often the TSDF hashmap's occupancy is sampled, in seconds.
#:
#: `TsdfMap.block_usage()` reads the hashmap's size, which is a DEVICE SYNC on
#: CUDA -- doing it per frame would put a pipeline stall in the hot path for a
#: number nobody reads faster than the 4 Hz metrics cadence. 0.25 s matches
#: that cadence; between samples `FrameStep` carries the last reading.
_BLOCK_USAGE_INTERVAL_S = 0.25


def _batch_gyro_mean(imu_raw) -> np.ndarray | None:
    """Bias-corrected mean body-frame gyro rate (deg/s, (3,)) over a stream-11
    batch, or None if the batch carries no gyro words. Subtracts the SFLP live
    gyro-bias estimate when present -- the same correction `imufusion` applies."""
    if imu_raw is None:
        return None
    gyro = getattr(imu_raw, "gyro_dps", None)
    if gyro is None or len(gyro) == 0:
        return None
    w = np.asarray(gyro, dtype=np.float64).mean(axis=0)
    gbias = getattr(imu_raw, "gbias_dps", None)
    if gbias is not None and len(gbias) > 0:
        w = w - np.asarray(gbias, dtype=np.float64).mean(axis=0)
    return w


def _batch_accel_mag(imu_raw) -> float | None:
    """Specific-force magnitude (m/s^2) from a stream-11 batch: the norm of the
    mean raw accel over the batch. None if the batch carries no accel words. At
    rest this is ~1 g regardless of orientation -- the ZUPT signal (see motion.py)."""
    if imu_raw is None:
        return None
    accel = getattr(imu_raw, "accel_g", None)
    if accel is None or len(accel) == 0:
        return None
    mean_g = np.asarray(accel, dtype=np.float64).mean(axis=0)
    return float(np.linalg.norm(mean_g) * _G)


@dataclass
class FrameStep:
    pose: np.ndarray
    fitness: float
    rmse: float
    tracking_lost: bool
    slam_ms: float
    # TSDF block-grid occupancy (BUG-035). New fields WITH DEFAULTS so every
    # existing constructor (remote.py, tests) is unaffected. This is the
    # ceiling that silently killed 18% of a real sweep: map growth stalls and
    # frame-to-model tracking collapses ~30 frames later with no error at all,
    # so it has to be visible while the scan is running, not after it dies.
    # `blocks_capacity` is the hashmap's LIVE capacity (Open3D rehashes to
    # grow); `blocks_configured` is the `block_count` it was built with, which
    # is the number the operator can actually raise. None until the first
    # sample (see `_BLOCK_USAGE_INTERVAL_S`).
    blocks_used: int | None = None
    blocks_capacity: int | None = None
    blocks_configured: int | None = None
    # Stage timing (plan item 2, 2026-08-02 SLAM compute/transport follow-ups).
    # Wall-clock milliseconds around each stage's Python call, via the same
    # `self._clock` as `slam_ms` -- NOT a CUDA `synchronize()`, per that plan's
    # explicit instruction not to add sync calls to the hot path just to time
    # it. Whether a given number is therefore a real kernel-completion time or
    # only an async-dispatch time depends on whether that stage's OWN code
    # already forces a device->host copy before returning:
    #   - raycast_ms: TsdfMap.raycast() always ends in `.cpu().numpy()` on
    #     vertex/normal/depth (tsdf.py), so this already includes a sync on
    #     CUDA -- real elapsed time, not dispatch-only.
    #   - icp_ms: odometry.register()'s 'translation' path pulls source/target
    #     positions to `.cpu().numpy()` before iterating, and the '6dof' path
    #     ends in `result.transformation.cpu().numpy()` -- also sync-forced,
    #     also real elapsed time.
    #   - integrate_ms: TsdfMap.integrate() calls `_vbg.integrate(...)` and
    #     returns with NO device->host copy of its own. `_check_saturation()`
    #     forces one hashmap-size sync per call ONLY until the 90% warning has
    #     fired once (then it early-outs without reading), and
    #     `_check_rehash_headroom()` only reads every 25th call and only on
    #     CUDA. So on CUDA, integrate_ms is a DISPATCH-TIME LOWER BOUND most
    #     frames, not a measurement of when the integrate kernel actually
    #     finished -- do not read it as "integrate cost this many ms of GPU
    #     time" without also checking whether a sync happened to coincide.
    #     On CPU it is always the true synchronous cost (there is no device
    #     queue to be async against).
    # 0.0 when the stage did not run this frame (e.g. bootstrap/lost frames
    # skip raycast+ICP; a lost frame also skips integrate).
    raycast_ms: float = 0.0
    icp_ms: float = 0.0
    integrate_ms: float = 0.0
    # Exact ICP match count plus its denominator. Zero on bootstrap/pre-ICP
    # frames; defaults preserve remote workers and older test constructors.
    inliers: int = 0
    source_points: int = 0


class Mapper:
    def __init__(self, width: int, height: int, fov_h: float = 55.0, fov_v: float = 42.0,
                 icp_mode: str = "translation", voxel_size: float = 0.01,
                 baro_authority: float = 0.05, baro_tau_frames: int = 900,
                 max_dist: float = 0.05,
                 icp_retry_dist: float = 0.10,
                 icp_cond_cap: float = _COND_CAP,
                 icp_rot_prior_weight: float = _ROT_PRIOR_WEIGHT,
                 min_fitness: float = 0.3, max_rmse: float = 0.05,
                 max_iter: int = 6,
                 adapt_min_fitness: float = 0.6, adapt_max_rmse: float = 0.03,
                 adapt_max_corr_deg: float = 20.0, adapt_rot_cond_cap: float = 100.0,
                 min_confidence: float | None = _DEFAULT_MIN_CONFIDENCE,
                 weight_threshold: float = 3.0,
                 device: str | o3d.core.Device = "CPU:0",
                 icp_device: str | o3d.core.Device | None = "CPU:0",
                 release_cache_every: int = 1,
                 block_count: int = DEFAULT_BLOCK_COUNT,
                 stationary_hold: bool = True,
                 stationary_window: int = 10,
                 stationary_coherence: float = 0.5,
                 stationary_step_ceiling: float = 0.03,
                 stationary_rot_ceiling: float = 0.3,
                 prior_smooth_alpha: float = 0.0,
                 apply_quat_phase: bool = False,
                 zupt_enabled: bool = True,
                 zupt_window: int = 6,
                 zupt_accel_tol_g: float = 0.04,
                 zupt_coherence: float = 0.5,
                 imu_spike_deg: float = 0.0,
                 baro_reject_m: float = 50.0,
                 clock=time.perf_counter):
        self.width, self.height = width, height
        self.icp_mode = icp_mode
        self.baro_authority = baro_authority
        self.baro_tau_frames = baro_tau_frames
        self.min_confidence = min_confidence
        # IMU spike pre-gate ceiling, deg of inter-frame SFLP rotation. Above
        # this the quat is treated as a glitch and the last good orientation is
        # held as the prior (see step()). DEFAULT 0 = DISABLED: this is a
        # defensive guard against a genuine per-frame rotation glitch, but no
        # real capture here exercises it -- 2026-08-05-crazySLAM.bin looked like
        # an IMU spike (capture_motion read 14737 deg/s) but the mapper sees only
        # 2.9 deg/frame; that capture's fabrication was a BAROMETER dropout (see
        # `baro_reject_m`). Shipping it active would be an unmeasured behaviour
        # change, so it is off until a capture proves a threshold. ~30 deg/frame
        # (900 deg/s) is a sane value if enabled. `imu_spike_count` reports hits.
        self.imu_spike_deg = imu_spike_deg
        self.imu_spike_count = 0
        # Barometer outlier rejection (2026-08-05-crazySLAM.bin): a pressure
        # dropout reads 0.0 Pa, which baro_height_m maps to 44330 m; a single
        # such sample injects a ~2.45 m vertical step even at 0.05 authority, and
        # 4 of them fabricated that capture's entire 8.6 m of vertical "motion".
        # Reject any pressure whose implied altitude vs the datum exceeds
        # `baro_reject_m` (an indoor handheld scan never moves that far
        # vertically), plus a hard reject of non-physical pressure <= 0.
        # 0 disables. `baro_rejected` reports how many samples were dropped.
        self.baro_reject_m = baro_reject_m
        self.baro_rejected = 0
        self._device = device if isinstance(device, o3d.core.Device) else o3d.core.Device(device)
        # Item 5 (2026-08-02): the device that runs ICP's nearest-neighbour
        # index, SEPARATE from the compute device above. Default "CPU:0" --
        # i.e. on a CUDA rig the TSDF integrate/raycast stay on the GPU and only
        # the hybrid NN search comes home. `None` means "follow `device`",
        # which restores the old behaviour exactly.
        #
        # Why this is not a semantic change: `odometry.register`'s translation
        # path ALREADY downloads source positions, target positions and target
        # normals and does every bit of its arithmetic (residual, condition
        # check, 3x3 solve) in numpy. The only thing `device` selects there is
        # which hybrid index runs the search, so moving it removes a device
        # round-trip rather than adding one. Measured bit-identical over 3177
        # ICP calls across two captures and a full 1979-frame x 10-perturbation
        # ensemble (docs/superpowers/plans/2026-08-02-cuda-icp-study.md SS B/C).
        #
        # Overridable rather than hard-coded on purpose: it converts GPU wait
        # into CPU work, and `roomscan-web` runs its asyncio loop, reader thread
        # and broadcaster on the same CPU. On a CPU-starved box the win shrinks
        # (measured -10.1% of a SLAM step on a quiet box, -1.5% under 12.7 cores
        # of external load); set `[slam] icp_device = "CUDA:0"` to put it back.
        self._icp_device = (
            self._device if icp_device is None
            else (icp_device if isinstance(icp_device, o3d.core.Device)
                  else o3d.core.Device(icp_device)))
        self._deproj = Deprojector(width, height, fov_h, fov_v)
        self._intr = pinhole(width, height, fov_h, fov_v, device=self._device)
        self._tsdf = TsdfMap(voxel_size=voxel_size, weight_threshold=weight_threshold,
                             device=self._device,
                             release_cache_every=release_cache_every,
                             block_count=block_count)
        self._gate = dict(max_dist=max_dist, min_fitness=min_fitness, max_rmse=max_rmse,
                          cond_cap=icp_cond_cap,
                          rot_prior_weight=icp_rot_prior_weight,
                          max_iter=max(1, int(max_iter)),
                          adapt_min_fitness=adapt_min_fitness,
                          adapt_max_rmse=adapt_max_rmse,
                          adapt_max_corr_deg=adapt_max_corr_deg,
                          adapt_rot_cond_cap=adapt_rot_cond_cap)
        # icp_mode="adaptive" only: how often the point cloud (LiDAR) overrode the
        # IMU rotation prior vs fell back to it. Both zero in translation/6dof mode.
        self.icp_lidar_count = 0
        self.icp_imu_fallback_count = 0
        self._retry_dist = icp_retry_dist
        # Frames where the tight radius failed and the wider retry was tried.
        # Expected to be ~0 on a clean scan; a non-trivial count means the scan
        # was repeatedly on the edge of the terminal failure this guards against.
        self.icp_escalations = 0
        self._clock = clock
        # Stationarity hold (owner: "device is stationary, tweak it until this
        # is true in our model"): the ICP translation noise random-walks the
        # position when still. `StationarityGate` holds the pose during
        # incoherent jitter while passing coherent motion; None disables it,
        # restoring byte-identical pre-hold behavior. See slam/motion.py.
        self._stationary_gate = (
            StationarityGate(window=stationary_window,
                             coherence_thresh=stationary_coherence,
                             step_ceiling_m=stationary_step_ceiling,
                             rot_ceiling_deg=stationary_rot_ceiling)
            if stationary_hold else None)
        self.held_count = 0         # frames whose reported translation was frozen
        self._display_pos = None    # de-jittered reported position (hold target)
        # Accelerometer ZUPT (BUG-069): a zero-velocity constraint that, unlike the
        # display-only StationarityGate above, reaches the MAP. It fires on the
        # accelerometer (|a| ~= 1 g => not translating), so it works during a pan --
        # the case the coherence gate structurally cannot see. Needs the raw IMU
        # batch in step(); a no-op (never fires) when zupt_enabled is False or no
        # imu_raw is supplied, so existing callers are byte-identical. See motion.py.
        self._zupt = (ZuptDetector(window=zupt_window, accel_tol_g=zupt_accel_tol_g,
                                   coherence_thresh=zupt_coherence)
                      if zupt_enabled else None)
        self.zupt_count = 0         # frames whose TRUE translation was held to t_prev
        # +7.76 ms quat-phase compensation (BUG-031/067). Off by default: it changes
        # the rotation prior and wants its own before/after on a moving capture.
        self._apply_quat_phase = bool(apply_quat_phase)
        self.quat_phase_count = 0   # frames the phase correction actually moved
        self._quat_prev = None      # for the stationarity gate's rotation signal
        # Rotation-prior smoothing (BUG-067 lever). A causal EMA/slerp of the SFLP
        # prior quaternion fed to predict_pose: `prior_smooth_alpha` is the weight
        # on the PREVIOUS smoothed value (0 = off, byte-identical raw prior; higher
        # = more smoothing). The rotation prior is what a translation/soft-prior ICP
        # cannot disagree with, so a per-frame-noisy prior turns into fabricated
        # translation; the phase-sweep in BUG-067 showed any low-pass of the prior
        # collapses the tripod instability (sd 0.489 -> ~0.03). Applied to the prior
        # only; the raw quat still drives rot_delta_deg (spike/stationarity signals).
        self._prior_smooth_alpha = float(prior_smooth_alpha)
        self._quat_smooth = None
        self._t_prev = np.zeros(3)
        self._ref_pa: float | None = None
        self._ref_acc = 0.0          # running sum for the averaged baro datum
        self._ref_n = 0
        # Low-passed baro-vs-ICP height disagreement (m). Opens at 0, not at the
        # first innovation: at that instant the pose height IS 0 and the datum
        # is the averaged start pressure, so the true disagreement is ~0 and a
        # single noisy sample (~267 mm) is the worst possible seed -- seeding
        # from it put a k*267 mm step into the very first corrected frame.
        self._baro_lp = 0.0
        # Cumulative height the barometer has pushed the pose by, in metres
        # along world-up. Reported (not just applied) so a run can say how much
        # of its height came from the barometer rather than from ICP.
        self.baro_correction_m = 0.0
        # TSDF occupancy, sampled at `_BLOCK_USAGE_INTERVAL_S` (a device sync
        # on CUDA -- see that constant). Reported on every FrameStep so the
        # live UI can show headroom against BUG-035's ceiling.
        self._block_usage_at: float | None = None
        self._blocks_used: int | None = None
        self._blocks_capacity: int | None = None
        self.trajectory: list[np.ndarray] = []
        self.tracking_lost_count = 0
        # Frames submitted vs. frames actually stepped are a WORKER-level
        # concept (the latest-wins input slot lives in SlamWorker/
        # RemoteSlamWorker, not here) -- see those classes for
        # frames_submitted/frames_overwritten. This Mapper only ever sees
        # frames that already made it through that slot.
        # Per-frame lost flag. Kept because the COUNT alone hides the failure
        # that matters: a run that ends in an unbroken lost streak is
        # dead-reckoning a frozen pose (predict_pose holds t_prev), so its
        # trajectory tail is fabricated, not measured. See metrics.tracking_stats.
        self.lost_flags: list[bool] = []
        self._bootstrapped = False

    @property
    def device(self) -> str:
        """The Open3D compute device this Mapper actually resolved and built
        its TSDF/pipeline on (e.g. "CUDA:0", "CPU:0") -- read from `self._device`
        (an `o3d.core.Device`, set once in `__init__` from whatever `device`
        was requested), never re-derived. Plan item 2 (2026-08-02): the SLAM
        message used to report the HOST's `preferred_device()` guess rather
        than asking the object that was actually built; that is a real
        discrepancy for a remote worker, whose compute device lives in a
        different process (the GPU container) than whatever the host would
        infer for itself. Callers (SlamWorker.device, the `slam` message)
        should read this rather than re-inferring."""
        return str(self._device)

    @property
    def icp_device(self) -> str:
        """The device this Mapper resolved for ICP's nearest-neighbour index
        (item 5, 2026-08-02). Usually "CPU:0" even when `device` is "CUDA:0" --
        see `__init__`. Read it to confirm the knob actually took effect: a
        dataclass/kwarg that silently failed to apply reports "no difference",
        which is also what a correctly-equivalent change reports."""
        return str(self._icp_device)

    @property
    def _register_device(self) -> o3d.core.Device:
        """Device passed to `odometry.register`.

        `icp_device` applies to the **translation** and **soft_prior** modes,
        whose solves are hand-written numpy over a host/device NN index. The
        `6dof` path is Open3D's own tensor ICP, which runs on the device its
        point clouds live on and takes `device` only to build the init tensor;
        handing it a host device while `source`/`target` sit on CUDA is a device
        mismatch, not an optimization. `icp_mode` is a public attribute, so this
        is resolved per call rather than frozen in `__init__`."""
        return (self._icp_device if self.icp_mode in ("translation", "soft_prior")
                else self._device)

    def set_imu_rate_hz(self, imu_rate_hz: float | None) -> None:
        """Live IMU/env poll-rate update (Task 7's decoupled rate; Task 8).

        Recomputes `baro_tau_frames` to hold the barometer low-pass's ~30
        SECOND time constant at whatever rate is actually driving pressure
        readings: ``baro_tau_frames = round(30 * imu_rate_hz)``, so 900 @
        30 Hz and 2700 @ 90 Hz are the SAME 30 s of averaging at each rate's
        own cadence (see test_slam_mapper.py's equivalence tests). Callers
        resolve coupled mode (the applied IMU/env rate equals the concurrent
        ToF rate, as before Task 7) to a concrete Hz value themselves before
        calling this -- `Mapper` has no notion of "ToF rate" at all.

        Intended to be called from the WORKER thread (`SlamWorker.run_once`
        / the container's worker via `SlamService`), never from `submit()`'s
        producer thread, so a rate change lands cleanly between `step()`
        calls, never mid-step.

        `None` or `0` (no rate known yet, or a caller that never calls this
        at all) is a deliberate no-op: `baro_tau_frames` keeps whatever it
        was constructed with or last set to, so existing callers that never
        touch this see byte-identical behavior. This never resets
        `_baro_lp`, `baro_correction_m`, `_ref_pa`/`_ref_acc`/`_ref_n`, the
        trajectory, or the map -- a rate change changes only how fast
        FUTURE samples are averaged, not what has already been integrated."""
        if not imu_rate_hz:
            return
        self.baro_tau_frames = max(1, round(30.0 * float(imu_rate_hz)))

    def _smooth_prior(self, quat):
        """Causal slerp EMA of the prior quaternion. `prior_smooth_alpha` is the
        weight on history: 0 returns the raw quat (off), higher lags more. Seeds
        on the first sample so nothing is smoothed toward an arbitrary origin."""
        q = np.asarray(quat, dtype=np.float64)
        if self._quat_smooth is None:
            self._quat_smooth = q.copy()
            return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        out = slerp(q, self._quat_smooth, self._prior_smooth_alpha)
        self._quat_smooth = np.asarray(out, dtype=np.float64)
        return out

    def _apply_baro_z(self, pose: np.ndarray, pressure_pa: float | None) -> np.ndarray:
        """Barometric height as a bounded, low-passed complementary correction
        (BUG-037). The cumulative correction ever applied is exactly

            baro_correction = baro_authority * LPF_tau(h_baro - h_icp)

        i.e. the barometer nudges a fraction of a heavily smoothed disagreement;
        it never owns the height. Three measurements set that shape, taken on
        the owner's room circuits (captures/coffeeRoomCircuit{Mnt,NoMnt}.bin):

        1. **The signal is mostly white noise.** Frame-rate pressure carries
           ~3.1 Pa RMS of sample-to-sample noise = **~267 mm RMS of apparent
           altitude**, 380 mm frame to frame. The old code fed that raw into a
           0.66 s blend, which injected **11.5-15.1 mm of vertical step per
           frame** (measured on all three captures) -- 34/29/37% of the whole
           reported path length was motion that never happened, flattering
           every %-of-path drift figure. Hence `baro_tau_frames` -- it must be
           low-passed before it is allowed anywhere near the pose. (The noise
           is measured; its cause is not proven. Best hypothesis: `rs_lsm.c`
           writes LPS22DF `CTRL_REG1 = 0x20`, which by that register map is
           25 Hz with the *minimum* averaging setting, and the sensor hub
           delivers ~1 sample per ToF frame so there is no second bite.)
        2. **What survives the filter is still worse than ICP.** The smoothed
           barometer wanders ~0.4-0.5 m per minute (0.43 m net on the mounted
           run) against ICP's own vertical drift of ~20-100 mm/min. A blend
           gives the barometer *full* authority below its corner frequency --
           precisely the band where it is ~20x the worse instrument. Hence
           `baro_authority`: the least-squares share of two drifting estimates
           is q_icp^2/(q_icp^2 + q_baro^2) ~= 0.09^2/(0.09^2 + 0.45^2) ~= 0.04,
           rounded to 0.05. (That it equals the retired `baro_weight`'s value
           is a coincidence of arithmetic, not of meaning: `baro_weight = 0.05`
           was a per-frame gain whose DC authority was 1.0.)
        3. **The datum was one noisy sample** -- see `_BARO_REF_FRAMES`.

        The honest consequence, measured: with *this* barometer the optimal
        authority is small enough that the constraint is worth ~nothing on a
        1-minute room scan -- its whole-run correction is ~10 mm, and outcome
        differences at that scale are chaos, not signal (a deliberate 3 mm
        one-shot nudge moves the final height error by 146 mm and the loop
        closure by 0.37 m). It is kept, in this shape, because the parameters
        are now measurable quantities rather than a magic gain: a quieter
        barometer, or a scan long enough for ICP drift to overtake (2), makes
        it earn its place again by moving a number we can measure."""
        if pressure_pa is None or self.baro_authority <= 0.0:
            return pose
        if self._baro_is_outlier(pressure_pa):
            # Drop a dropout/glitch BEFORE it reaches the datum or the low-pass:
            # no _ref_acc update, no _baro_lp update, pose unchanged this frame.
            self.baro_rejected += 1
            return pose
        if self._ref_pa is None:
            self._ref_acc += pressure_pa
            self._ref_n += 1
            if self._ref_n < _BARO_REF_FRAMES:
                return pose
            self._ref_pa = self._ref_acc / self._ref_n
            return pose
        up = world_up()
        cur = pose[:3, 3]
        h_baro = baro_height_m(pressure_pa, self._ref_pa)    # metres up in world
        # Disagreement against ICP's OWN height: the pose already carries every
        # correction applied so far (it feeds t_prev and the next prediction),
        # so subtract it back out or the loop would chase its own tail and the
        # steady-state authority would be k/(1+k) instead of k.
        innov = h_baro - (float(np.dot(cur, up)) - self.baro_correction_m)
        alpha = 1.0 if self.baro_tau_frames <= 1 else 1.0 / float(self.baro_tau_frames)
        self._baro_lp += alpha * (innov - self._baro_lp)
        target = self.baro_authority * self._baro_lp
        step = target - self.baro_correction_m
        self.baro_correction_m = target
        out = pose.copy()
        out[:3, 3] = cur + step * up
        return out

    def _baro_is_outlier(self, pressure_pa: float) -> bool:
        """True for a pressure sample that is a sensor dropout/glitch rather than
        a reading. Two rules:

        * ``pressure_pa <= 0`` -- non-physical; the exact 2026-08-05-crazySLAM.bin
          failure mode was `pressure == 0.0` (baro_height_m(0, ref) = 44330 m).
        * implied altitude vs the datum (or, before the datum is fixed, the
          running mean of accepted samples) beyond ``baro_reject_m`` -- an indoor
          handheld scan never moves that far vertically in one frame.

        Disabled by ``baro_reject_m <= 0``. The first sample (no datum, no running
        mean yet) can only be judged by the ``<= 0`` rule -- there is nothing to
        compare a plausible-but-wrong value against yet."""
        if self.baro_reject_m <= 0.0:
            return False
        if pressure_pa <= 0.0:
            return True
        ref = self._ref_pa
        if ref is None:
            ref = self._ref_acc / self._ref_n if self._ref_n else None
        if ref is None:
            return False
        return abs(baro_height_m(pressure_pa, ref)) > self.baro_reject_m

    def _gate_confidence(self, depth_mm: np.ndarray, confidence: np.ndarray | None) -> np.ndarray:
        """Invalidate (zero) depth pixels whose confidence is below
        `min_confidence` -- higher confidence = better (verified on the real
        capture: values range ~0-460, median ~75; see task-quality-report.md
        for the chosen threshold). A no-op when no confidence was supplied or
        gating is disabled (`min_confidence=None`), so existing callers that
        never pass `confidence` see byte-identical depth. NaN confidence
        fails the `>=` comparison (False), so unknown-confidence pixels are
        also invalidated rather than trusted."""
        if confidence is None or self.min_confidence is None:
            return depth_mm
        mask = np.asarray(confidence) >= self.min_confidence
        return np.where(mask, depth_mm, 0.0).astype(np.float32)

    @staticmethod
    def _reflectance_color(reflectance: np.ndarray) -> np.ndarray:
        """Reflectance -> an (h, w, 3) float32 [0,1] grayscale image, via the
        same percentile-clip normalization the IR monitor uses (`colors.
        normalize`), so the mesh's reflectance look matches the live IR
        panel."""
        norm = _percentile_normalize(reflectance).astype(np.float32)
        return np.repeat(norm[..., None], 3, axis=-1)

    def step(self, depth_mm: np.ndarray, quat, pressure_pa=None,
             reflectance=None, confidence=None,
             imu_raw=None, quat_offset_us=None) -> FrameStep:
        t0 = self._clock()
        depth_mm = self._gate_confidence(depth_mm, confidence)
        # +7.76 ms quat-phase compensation (BUG-031/067), applied FIRST so the
        # spike gate, rot_delta, smoothing and the pose all use the frame-instant
        # orientation. No-op unless enabled AND this frame carries both the lead
        # (quat_offset_us) and a gyro batch to roll the quat back with.
        if (self._apply_quat_phase and quat is not None and quat_offset_us
                and imu_raw is not None):
            gyro = _batch_gyro_mean(imu_raw)
            if gyro is not None:
                corrected = _quat_phase_correct(quat, gyro, quat_offset_us)
                if corrected is not quat:
                    quat = corrected
                    self.quat_phase_count += 1
        color = self._reflectance_color(reflectance) if reflectance is not None else None
        pts, valid = self._deproj.grid(depth_mm)
        n_valid = int(valid.sum())
        # Per-frame rotation magnitude (deg) from the SFLP prior. Computed
        # BEFORE predict_pose so the IMU spike pre-gate can act on it: the prior
        # feeds both the pose AND the raycast viewpoint (T_pred below), so a
        # glitch has to be caught here or it corrupts the model cloud too.
        # angle = 2*acos(|<q_prev, q>|). Also the stationarity gate's rotation
        # signal (separates a still tripod ~0 from an actively aimed scan).
        rot_delta_deg = 0.0
        if self._quat_prev is not None and quat is not None:
            dot = abs(float(np.dot(self._quat_prev, quat)))
            rot_delta_deg = float(np.degrees(2.0 * np.arccos(min(1.0, dot))))

        # IMU spike pre-gate: a physically-impossible inter-frame rotation is an
        # SFLP quaternion glitch, not motion -- this capture's SFLP stream spiked
        # to 14737 deg/s (40 rev/s) while the operator moved smoothly. In
        # translation mode ICP holds rotation at this prior and is structurally
        # unable to disagree, so the glitch comes out as fabricated translation
        # and is baked into the TSDF permanently (one frame here jumped 2.5 m).
        # When the delta exceeds `imu_spike_deg`, hold the last good orientation
        # as the prior instead; a physical-ceiling reject needs no point-cloud
        # corroboration, so it is unambiguous and works even in translation mode.
        # `imu_spike_deg <= 0` disables it, restoring the pre-gate behaviour
        # exactly. The stricter, point-cloud-corroborated rejection (reject a
        # spike the LiDAR does not confirm) is `icp_mode="adaptive"`.
        quat_prior = quat
        spike = (self._quat_prev is not None and quat is not None
                 and self.imu_spike_deg > 0.0 and rot_delta_deg > self.imu_spike_deg)
        if spike:
            quat_prior = self._quat_prev            # hold last good orientation
            rot_delta_deg = 0.0                     # nothing rotated this frame
            self.imu_spike_count += 1
        elif quat is not None:
            self._quat_prev = np.asarray(quat, dtype=np.float64)

        # Rotation-prior smoothing (BUG-067 lever): low-pass the prior orientation
        # fed to the pose/raycast before it becomes a hard constraint ICP cannot
        # argue with. No-op at alpha=0. Uses `quat_prior` (post spike-gate) so a
        # held glitch is smoothed like any other sample.
        if self._prior_smooth_alpha > 0.0 and quat_prior is not None:
            quat_prior = self._smooth_prior(quat_prior)

        T_pred = predict_pose(quat_prior, self._t_prev)

        lost = False
        held = False
        fitness = rmse = 0.0
        inliers = source_points = 0
        raycast_ms = icp_ms = 0.0

        if n_valid < _MIN_VALID_POINTS:
            lost = True
            pose = T_pred
        elif not self._bootstrapped:
            pose = T_pred                                   # bootstrap: accept prior
        else:
            src = source_cloud(pts, valid, device=self._device)
            # Bound raycast to the current view frustum (Task 9.5 Lever 1):
            # the current depth frame at the predicted pose is our best
            # estimate of which voxel blocks the live camera can see, so pass
            # it as a depth hint instead of raycasting every active block
            # ever integrated (whose cost scales with total map size).
            # TsdfMap.raycast checks its own empty-map guard before deriving
            # frustum coords from the hint, so this is safe even if the map
            # has never been integrated into yet (e.g. an earlier bootstrap
            # frame was lost).
            t_raycast0 = self._clock()
            # `with_count`: raycast already knows how many points survived its
            # own validity mask, so ask for the number instead of downloading
            # the whole position array a SECOND time just to read `.shape[0]`
            # (item 5, 2026-08-02 -- 0.016 ms/frame and one fewer device->host
            # transfer, measured by `slam_icp_bench --what raycast`).
            model, n_model = self._tsdf.raycast(self._intr, np.linalg.inv(T_pred),
                                                self.width, self.height,
                                                depth_hint=depth_mm, with_count=True)
            raycast_ms = (self._clock() - t_raycast0) * 1000.0
            if model is None or n_model < _MIN_VALID_POINTS:
                lost = True
                pose = T_pred
            else:
                # TsdfMap.raycast()'s "vertex" output is expressed in the LOCAL camera
                # frame of the raycast pose (T_pred), not world frame -- i.e. it is the
                # depth-camera-style vertex map you'd get from a real sensor sitting at
                # T_pred (verified empirically: raycasting the same static map from a
                # translated viewpoint shifts the returned points by exactly that
                # translation). `src` (this frame's deprojected points) is likewise in
                # the LIVE camera's own local frame. Since T_pred is our best guess of
                # the live pose, src and model already live in approximately the same
                # local frame -- so ICP's initial guess is identity (not T_pred), and
                # the resulting correction must be composed onto T_pred afterward to
                # get a world pose: pose_world = T_pred @ correction.
                t_icp0 = self._clock()
                res, escalated = register_escalating(
                    src, model, np.eye(4), retry_dist=self._retry_dist,
                    mode=self.icp_mode, device=self._register_device, **self._gate)
                icp_ms = (self._clock() - t_icp0) * 1000.0
                if escalated:
                    self.icp_escalations += 1
                if res.source == "lidar":
                    self.icp_lidar_count += 1
                elif res.source == "imu":
                    self.icp_imu_fallback_count += 1
                fitness, rmse = res.fitness, res.rmse
                inliers, source_points = res.inliers, res.source_points
                if res.ok:
                    pose = self._apply_baro_z(T_pred @ res.pose, pressure_pa)
                    # Accelerometer ZUPT (BUG-069): a MAP-REACHING zero-velocity
                    # constraint. When the raw accel says the sensor is not
                    # translating (|a| ~= 1 g, true even during a pan), freeze the
                    # TRUE pose's translation at t_prev BEFORE it feeds integrate
                    # and _t_prev -- so the TSDF never absorbs the invented motion
                    # in the first place. Distinct from the display-only hold
                    # below: this changes the reconstruction, and is why it keys on
                    # the physically-grounded, LiDAR-independent accelerometer
                    # rather than on ICP coherence (which cannot fire on a pan).
                    if self._zupt is not None:
                        increment = pose[:3, 3] - self._t_prev
                        if self._zupt.update(_batch_accel_mag(imu_raw), increment):
                            pose = pose.copy()
                            pose[:3, 3] = self._t_prev
                            self.zupt_count += 1
                    # Stationarity gate (owner: "device is stationary -> model
                    # should be too"). Feed the RAW ICP-estimated increment
                    # (never a held value, or the gate could never see motion
                    # resume). A True verdict de-jitters the REPORTED pose only
                    # (see report_pose below): the map integration and tracking
                    # prior always use the true ICP `pose`, so a false hold can
                    # never corrupt the reconstruction -- final-map accuracy is
                    # identical to gate-off. The hold just stops the previewed
                    # camera/trajectory from random-walking while the sensor
                    # sits still.
                    held = (self._stationary_gate is not None and
                            self._stationary_gate.update(pose[:3, 3] - self._t_prev,
                                                         rot_delta_deg))
                    if held:
                        self.held_count += 1
                else:
                    lost = True
                    pose = T_pred

        integrate_ms = 0.0
        if not lost:
            # Map + tracking prior use the TRUE ICP pose -- accuracy is
            # unaffected by the stationarity gate (see the `held` comment).
            # Timed, but see FrameStep.integrate_ms's docstring: on CUDA this
            # is usually a DISPATCH-time lower bound, not the kernel's actual
            # completion time, because `integrate()` doesn't always force a
            # device->host sync (only `_check_saturation`'s pre-warning reads
            # and `_check_rehash_headroom`'s every-25th-call read do).
            t_integrate0 = self._clock()
            self._tsdf.integrate(depth_mm, self._intr, np.linalg.inv(pose), color=color)
            integrate_ms = (self._clock() - t_integrate0) * 1000.0
            self._t_prev = pose[:3, 3].copy()
            self._bootstrapped = True
        else:
            self.tracking_lost_count += 1

        # Reported/preview pose: during a stationary hold, freeze the reported
        # translation at the last non-held position so the previewed camera and
        # trajectory don't jitter while the sensor sits still. Rotation (the
        # already-stable SFLP prior) always passes through, and the map/tracking
        # above are untouched, so this is a display-only de-jitter.
        report_pose = pose
        if held and self._display_pos is not None:
            report_pose = pose.copy()
            report_pose[:3, 3] = self._display_pos
        else:
            self._display_pos = pose[:3, 3].copy()

        self.trajectory.append(report_pose.copy())
        self.lost_flags.append(bool(lost))
        self._sample_block_usage()
        slam_ms = (self._clock() - t0) * 1000.0
        return FrameStep(pose=report_pose, fitness=fitness, rmse=rmse,
                         tracking_lost=lost, slam_ms=slam_ms,
                         blocks_used=self._blocks_used,
                         blocks_capacity=self._blocks_capacity,
                         blocks_configured=self._tsdf.block_count or None,
                         raycast_ms=raycast_ms, icp_ms=icp_ms, integrate_ms=integrate_ms,
                         inliers=inliers, source_points=source_points)

    def _sample_block_usage(self) -> None:
        """Refresh the cached TSDF occupancy, at most every
        `_BLOCK_USAGE_INTERVAL_S`. Deliberately NOT per frame: the hashmap size
        read behind `block_usage()` is a device sync on CUDA. Failures are
        swallowed -- a gauge must never be able to kill a scan."""
        now = self._clock()
        if (self._block_usage_at is not None
                and (now - self._block_usage_at) < _BLOCK_USAGE_INTERVAL_S):
            return
        self._block_usage_at = now
        try:
            self._blocks_used, self._blocks_capacity = self._tsdf.block_usage()
        except Exception:
            pass          # keep the previous reading rather than reporting a fake 0

    def mesh(self):
        return self._tsdf.mesh()

    def map_point_cloud(self):
        return self._tsdf.point_cloud()
