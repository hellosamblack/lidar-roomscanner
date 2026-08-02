"""Frame-to-model SLAM orchestrator. Per-frame: deproject -> predict pose from the
SFLP prior -> raycast model -> point-to-plane ICP -> baro soft-Z -> integrate.
See docs/superpowers/specs/completed/2026-07-10-phase6-slam-design.md sections 3, 5.

`device` (str or o3d.core.Device, default "CPU:0") is resolved once in
__init__ and forwarded to every Open3D piece it owns (TsdfMap, the pinhole
intrinsic, source_cloud, register) so the whole per-frame pipeline runs on a
single compute device -- CPU today, and unchanged "CUDA:0" once a CUDA build
of Open3D is installed. Any tensor pulled off that device (e.g. the raycast
model's `positions` here) is moved home with `.cpu()` before `.numpy()`."""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import open3d as o3d

from ..colors import normalize as _percentile_normalize
from ..deproject import Deprojector
from .cloud import source_cloud
from .frames import baro_height_m, predict_pose, world_up
from .intrinsics import pinhole
from .motion import StationarityGate
from .odometry import register_escalating
from .tsdf import DEFAULT_BLOCK_COUNT, TsdfMap

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


class Mapper:
    def __init__(self, width: int, height: int, fov_h: float = 55.0, fov_v: float = 42.0,
                 icp_mode: str = "translation", voxel_size: float = 0.01,
                 baro_authority: float = 0.05, baro_tau_frames: int = 900,
                 max_dist: float = 0.05,
                 icp_retry_dist: float = 0.10,
                 min_fitness: float = 0.3, max_rmse: float = 0.05,
                 max_iter: int = 6,
                 min_confidence: float | None = _DEFAULT_MIN_CONFIDENCE,
                 weight_threshold: float = 3.0,
                 device: str | o3d.core.Device = "CPU:0",
                 release_cache_every: int = 1,
                 block_count: int = DEFAULT_BLOCK_COUNT,
                 stationary_hold: bool = True,
                 stationary_window: int = 10,
                 stationary_coherence: float = 0.5,
                 stationary_step_ceiling: float = 0.03,
                 stationary_rot_ceiling: float = 0.3,
                 clock=time.perf_counter):
        self.width, self.height = width, height
        self.icp_mode = icp_mode
        self.baro_authority = baro_authority
        self.baro_tau_frames = baro_tau_frames
        self.min_confidence = min_confidence
        self._device = device if isinstance(device, o3d.core.Device) else o3d.core.Device(device)
        self._deproj = Deprojector(width, height, fov_h, fov_v)
        self._intr = pinhole(width, height, fov_h, fov_v, device=self._device)
        self._tsdf = TsdfMap(voxel_size=voxel_size, weight_threshold=weight_threshold,
                             device=self._device,
                             release_cache_every=release_cache_every,
                             block_count=block_count)
        self._gate = dict(max_dist=max_dist, min_fitness=min_fitness, max_rmse=max_rmse,
                          max_iter=max(1, int(max_iter)))
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
        self._quat_prev = None      # for the stationarity gate's rotation signal
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
        # Per-frame lost flag. Kept because the COUNT alone hides the failure
        # that matters: a run that ends in an unbroken lost streak is
        # dead-reckoning a frozen pose (predict_pose holds t_prev), so its
        # trajectory tail is fabricated, not measured. See metrics.tracking_stats.
        self.lost_flags: list[bool] = []
        self._bootstrapped = False

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
             reflectance=None, confidence=None) -> FrameStep:
        t0 = self._clock()
        depth_mm = self._gate_confidence(depth_mm, confidence)
        color = self._reflectance_color(reflectance) if reflectance is not None else None
        pts, valid = self._deproj.grid(depth_mm)
        n_valid = int(valid.sum())
        T_pred = predict_pose(quat, self._t_prev)

        # Per-frame rotation magnitude (deg) from the SFLP prior, for the
        # stationarity gate: separates a still tripod (~0) from an actively
        # aimed handheld scan. angle = 2*acos(|<q_prev, q>|).
        rot_delta_deg = 0.0
        if self._quat_prev is not None and quat is not None:
            dot = abs(float(np.dot(self._quat_prev, quat)))
            rot_delta_deg = float(np.degrees(2.0 * np.arccos(min(1.0, dot))))
        if quat is not None:
            self._quat_prev = np.asarray(quat, dtype=np.float64)

        lost = False
        held = False
        fitness = rmse = 0.0

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
            model = self._tsdf.raycast(self._intr, np.linalg.inv(T_pred),
                                       self.width, self.height, depth_hint=depth_mm)
            if model is None or model.point.positions.cpu().numpy().shape[0] < _MIN_VALID_POINTS:
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
                res, escalated = register_escalating(
                    src, model, np.eye(4), retry_dist=self._retry_dist,
                    mode=self.icp_mode, device=self._device, **self._gate)
                if escalated:
                    self.icp_escalations += 1
                fitness, rmse = res.fitness, res.rmse
                if res.ok:
                    pose = self._apply_baro_z(T_pred @ res.pose, pressure_pa)
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

        if not lost:
            # Map + tracking prior use the TRUE ICP pose -- accuracy is
            # unaffected by the stationarity gate (see the `held` comment).
            self._tsdf.integrate(depth_mm, self._intr, np.linalg.inv(pose), color=color)
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
                         blocks_configured=self._tsdf.block_count or None)

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
