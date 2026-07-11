"""Frame-to-model SLAM orchestrator. Per-frame: deproject -> predict pose from the
SFLP prior -> raycast model -> point-to-plane ICP -> baro soft-Z -> integrate.
See docs/superpowers/specs/2026-07-10-phase6-slam-design.md sections 3, 5.

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
from .odometry import register
from .tsdf import TsdfMap

_MIN_VALID_POINTS = 100
_DEFAULT_MIN_CONFIDENCE = 20.0  # tuned against captures/phase6_motion_ref.bin, see task-quality-report.md


@dataclass
class FrameStep:
    pose: np.ndarray
    fitness: float
    rmse: float
    tracking_lost: bool
    slam_ms: float


class Mapper:
    def __init__(self, width: int, height: int, fov_h: float = 55.0, fov_v: float = 42.0,
                 icp_mode: str = "translation", voxel_size: float = 0.01,
                 baro_weight: float = 0.05, max_dist: float = 0.05,
                 min_fitness: float = 0.3, max_rmse: float = 0.05,
                 min_confidence: float | None = _DEFAULT_MIN_CONFIDENCE,
                 weight_threshold: float = 3.0,
                 device: str | o3d.core.Device = "CPU:0",
                 stationary_hold: bool = True,
                 stationary_window: int = 10,
                 stationary_coherence: float = 0.5,
                 stationary_step_ceiling: float = 0.03,
                 stationary_rot_ceiling: float = 0.3,
                 clock=time.perf_counter):
        self.width, self.height = width, height
        self.icp_mode = icp_mode
        self.baro_weight = baro_weight
        self.min_confidence = min_confidence
        self._device = device if isinstance(device, o3d.core.Device) else o3d.core.Device(device)
        self._deproj = Deprojector(width, height, fov_h, fov_v)
        self._intr = pinhole(width, height, fov_h, fov_v, device=self._device)
        self._tsdf = TsdfMap(voxel_size=voxel_size, weight_threshold=weight_threshold,
                             device=self._device)
        self._gate = dict(max_dist=max_dist, min_fitness=min_fitness, max_rmse=max_rmse)
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
        self.trajectory: list[np.ndarray] = []
        self.tracking_lost_count = 0
        self._bootstrapped = False

    def _apply_baro_z(self, pose: np.ndarray, pressure_pa: float | None) -> np.ndarray:
        if pressure_pa is None or self.baro_weight <= 0.0:
            return pose
        if self._ref_pa is None:
            self._ref_pa = pressure_pa
            return pose
        h = baro_height_m(pressure_pa, self._ref_pa)        # metres up in world
        up = world_up()
        target_up = h                                       # height along world_up axis
        cur = pose[:3, 3]
        cur_up = np.dot(cur, up)
        blended = cur + self.baro_weight * (target_up - cur_up) * up
        out = pose.copy()
        out[:3, 3] = blended
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
                res = register(src, model, np.eye(4), mode=self.icp_mode,
                              device=self._device, **self._gate)
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
        slam_ms = (self._clock() - t0) * 1000.0
        return FrameStep(pose=report_pose, fitness=fitness, rmse=rmse,
                         tracking_lost=lost, slam_ms=slam_ms)

    def mesh(self):
        return self._tsdf.mesh()

    def map_point_cloud(self):
        return self._tsdf.point_cloud()
