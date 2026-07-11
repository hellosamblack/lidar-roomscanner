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

        lost = False
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
                else:
                    lost = True
                    pose = T_pred

        if not lost:
            self._tsdf.integrate(depth_mm, self._intr, np.linalg.inv(pose), color=color)
            self._t_prev = pose[:3, 3].copy()
            self._bootstrapped = True
        else:
            self.tracking_lost_count += 1

        self.trajectory.append(pose.copy())
        slam_ms = (self._clock() - t0) * 1000.0
        return FrameStep(pose=pose, fitness=fitness, rmse=rmse,
                         tracking_lost=lost, slam_ms=slam_ms)

    def mesh(self):
        return self._tsdf.mesh()

    def map_point_cloud(self):
        return self._tsdf.point_cloud()
