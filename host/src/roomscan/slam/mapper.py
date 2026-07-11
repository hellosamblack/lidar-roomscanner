"""Frame-to-model SLAM orchestrator. Per-frame: deproject -> predict pose from the
SFLP prior -> raycast model -> point-to-plane ICP -> baro soft-Z -> integrate.
See docs/superpowers/specs/2026-07-10-phase6-slam-design.md sections 3, 5."""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from ..deproject import Deprojector
from .cloud import source_cloud
from .frames import baro_height_m, predict_pose, world_up
from .intrinsics import pinhole
from .odometry import register
from .tsdf import TsdfMap

_MIN_VALID_POINTS = 100


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
                 clock=time.perf_counter):
        self.width, self.height = width, height
        self.icp_mode = icp_mode
        self.baro_weight = baro_weight
        self._deproj = Deprojector(width, height, fov_h, fov_v)
        self._intr = pinhole(width, height, fov_h, fov_v)
        self._tsdf = TsdfMap(voxel_size=voxel_size)
        self._gate = dict(max_dist=max_dist, min_fitness=min_fitness, max_rmse=max_rmse)
        self._clock = clock
        self._t_prev = np.zeros(3)
        self._ref_pa: float | None = None
        self.trajectory: list[np.ndarray] = []
        self.tracking_lost_count = 0

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

    def step(self, depth_mm: np.ndarray, quat, pressure_pa=None) -> FrameStep:
        t0 = self._clock()
        pts, valid = self._deproj.grid(depth_mm)
        n_valid = int(valid.sum())
        T_pred = predict_pose(quat, self._t_prev)

        empty = not self.trajectory
        lost = False
        fitness = rmse = 0.0

        if n_valid < _MIN_VALID_POINTS:
            lost = True
            pose = T_pred
        elif empty:
            pose = T_pred                                   # bootstrap: accept prior
        else:
            src = source_cloud(pts, valid)
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
            if model is None or model.point.positions.numpy().shape[0] < _MIN_VALID_POINTS:
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
                res = register(src, model, np.eye(4), mode=self.icp_mode, **self._gate)
                fitness, rmse = res.fitness, res.rmse
                if res.ok:
                    pose = self._apply_baro_z(T_pred @ res.pose, pressure_pa)
                else:
                    lost = True
                    pose = T_pred

        if not lost:
            self._tsdf.integrate(depth_mm, self._intr, np.linalg.inv(pose))
            self._t_prev = pose[:3, 3].copy()
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
