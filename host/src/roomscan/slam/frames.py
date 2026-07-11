"""Pose, prior, and constraint construction in the frames of docs/coordinate-frames.md.

World frame = Open3D CV world (Y-down). Body->world uses the documented sandwich
T_WORLD_TO_CV @ R @ T_CV_TO_BODY. Baro 'up' is world -Y."""
from __future__ import annotations

import numpy as np

from ..sensors import quat_to_matrix, T_CV_TO_BODY, T_WORLD_TO_CV


def prior_rotation(quat: tuple[float, float, float, float]) -> np.ndarray:
    R = quat_to_matrix(*quat)                       # body -> SFLP world
    return T_WORLD_TO_CV @ R @ T_CV_TO_BODY          # -> Open3D CV world


def predict_pose(quat: tuple[float, float, float, float], t_prev: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = prior_rotation(quat)
    T[:3, 3] = np.asarray(t_prev, dtype=np.float64)
    return T


def world_up() -> np.ndarray:
    return np.array([0.0, -1.0, 0.0], dtype=np.float64)


def baro_height_m(pressure_pa: float, ref_pa: float) -> float:
    return 44330.0 * (1.0 - (pressure_pa / ref_pa) ** 0.190284)
