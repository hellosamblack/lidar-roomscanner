"""Synthetic scene + camera walk for SLAM verification (sub-phase 6.G).

Measurement/verification scaffolding, not part of the live pipeline. It exists
because the long-scan GPU OOM needs a scan LONGER than any recorded capture we
have (every capture in `captures/` is stationary or a braced tilt sweep, and a
real closed-loop walk needs the owner), and because a generated walk can be run
to any length on demand and is deterministic frame-for-frame.

What it gives the caller: `(depth_mm, quat)` frames that a real `Mapper.step`
can track -- poses are NOT injected, the mapper still runs its own raycast +
ICP + integrate, so the code path under measurement is the production one.

Design constraints that matter:
  * Pillars are not decoration. A bare box viewed from its middle is nearly
    plane-degenerate for point-to-plane ICP, and a mapper that loses tracking
    stops moving, which stops the map growing and defeats the measurement.
  * The camera keeps a FIXED orientation (identity SFLP quaternion, so the
    mapper's rotation prior is exactly the identity body/world sandwich) and
    only translates. That avoids needing a matrix->quaternion inverse of the
    sandwich to stay honest, and translation grows the map fastest anyway.

Used by `host/tools/slam_gpu_memory.py` and `tools/slam-container/cuda_smoke.py`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Room:
    """An inverted box (the room) plus solid AABB pillars, ray-cast by the slab
    method vectorised over pixels and boxes."""

    size: tuple[float, float, float] = (10.0, 2.6, 10.0)   # x, y (height), z
    n_pillars: int = 12
    pillar_w: float = 0.35
    seed: int = 7

    def __post_init__(self):
        sx, sy, sz = self.size
        # Interior: [-sx/2, sx/2] x [0, sy] x [-sz/2, sz/2], authored Y-up.
        self.lo = np.array([-sx / 2, 0.0, -sz / 2])
        self.hi = np.array([sx / 2, sy, sz / 2])
        rng = np.random.default_rng(self.seed)
        mins, maxs = [], []
        for _ in range(self.n_pillars):
            cx = rng.uniform(-sx / 2 + 1.0, sx / 2 - 1.0)
            cz = rng.uniform(-sz / 2 + 1.0, sz / 2 - 1.0)
            h = rng.uniform(0.8, sy)
            w = self.pillar_w * rng.uniform(0.7, 1.6)
            mins.append([cx - w, 0.0, cz - w])
            maxs.append([cx + w, h, cz + w])
        self.box_min = np.asarray(mins)                     # (n, 3)
        self.box_max = np.asarray(maxs)

    def raycast(self, origin: np.ndarray, dirs: np.ndarray, max_range: float) -> np.ndarray:
        """Nearest hit distance (m) along each unit `dirs` row from `origin`;
        `inf` where nothing is hit within `max_range`.

        The room shell is hit from the INSIDE, so its crossing is the far slab
        boundary (`t_far`); pillars are hit from the outside, so theirs is the
        near boundary (`t_near`), valid only when `t_far >= max(t_near, 0)`."""
        with np.errstate(divide="ignore", invalid="ignore"):
            inv = 1.0 / dirs                                # (n, 3)
            t_lo = (self.lo - origin) * inv
            t_hi = (self.hi - origin) * inv
            t_far = np.maximum(t_lo, t_hi).min(axis=1)
        best = np.where(t_far > 0, t_far, np.inf)

        for lo, hi in zip(self.box_min, self.box_max):
            with np.errstate(divide="ignore", invalid="ignore"):
                p_lo = (lo - origin) * inv
                p_hi = (hi - origin) * inv
                p_near = np.minimum(p_lo, p_hi).max(axis=1)
                p_far = np.maximum(p_lo, p_hi).min(axis=1)
            hit = (p_far >= np.maximum(p_near, 0.0)) & (p_near > 0.0)
            best = np.where(hit & (p_near < best), p_near, best)

        return np.where(best <= max_range, best, np.inf)


class SyntheticWalk:
    """Ground-truth camera walk through `Room`, rendering (h, w) depth frames.

    `next_frame()` returns `(depth_mm, quat)` ready for `Mapper.step`, and
    `path_length_m` accumulates the true distance walked so a run can be quoted
    in metres against the 68 m walk that OOM'd."""

    def __init__(self, width: int, height: int, fov_h: float = 55.0, fov_v: float = 42.0,
                 room: Room | None = None, speed_m_s: float = 0.6, fps: float = 30.0,
                 radius: float = 3.2, max_range_m: float = 4.9):
        from .frames import prior_rotation

        self.width, self.height = width, height
        self.room = room or Room()
        self.max_range_m = max_range_m
        self.step_m = speed_m_s / fps
        self.radius = radius
        self._t = 0.0
        self.path_length_m = 0.0
        self._prev_pos = None

        # Pixel ray directions in the camera frame, matching Deprojector's
        # separable zone-centre model (x right, y down, z forward).
        ax = np.deg2rad(((np.arange(width) + 0.5) / width - 0.5) * fov_h)
        ay = np.deg2rad(((np.arange(height) + 0.5) / height - 0.5) * fov_v)
        tx = np.tan(ax)[None, :] * np.ones((height, 1))
        ty = np.tan(ay)[:, None] * np.ones((1, width))
        d = np.stack([tx, ty, np.ones_like(tx)], axis=-1).reshape(-1, 3)
        self._d_cam = d / np.linalg.norm(d, axis=1, keepdims=True)
        self._z_cam = self._d_cam[:, 2]        # slant -> perpendicular-Z factor

        # Camera orientation in the mapper's CV world = the identity prior.
        self._R = prior_rotation((1.0, 0.0, 0.0, 0.0))      # world <- camera
        # The scene is authored Y-up; the CV world is Y-down. One flip converts.
        self._flip = np.diag([1.0, -1.0, 1.0])
        self._dirs_world = self._d_cam @ (self._flip @ self._R).T

    def _position(self, s: float) -> np.ndarray:
        """Lissajous ground track (scene frame, Y-up), eye height ~1.4 m."""
        return np.array([self.radius * math.sin(s),
                         1.4 + 0.05 * math.sin(3.0 * s),
                         self.radius * 0.75 * math.sin(2.0 * s + 0.7)])

    def next_frame(self) -> tuple[np.ndarray, tuple[float, float, float, float]]:
        # Advance by arc length so `step_m` is a true metres-per-frame speed
        # regardless of where on the Lissajous the camera currently is.
        ds = 1e-3
        p0 = self._position(self._t)
        speed = np.linalg.norm(self._position(self._t + ds) - p0) / ds
        self._t += self.step_m / max(speed, 1e-6)
        pos = self._position(self._t)
        if self._prev_pos is not None:
            self.path_length_m += float(np.linalg.norm(pos - self._prev_pos))
        self._prev_pos = pos

        dist = self.room.raycast(pos, self._dirs_world, self.max_range_m)
        depth_m = dist * self._z_cam
        depth_mm = np.where(np.isfinite(depth_m), depth_m * 1000.0, 0.0)
        return (depth_mm.reshape(self.height, self.width).astype(np.float32),
                (1.0, 0.0, 0.0, 0.0))
