"""Pose, prior, and constraint construction in the frames of docs/coordinate-frames.md.

World frame = Open3D CV world (Y-down). Body->world uses the documented sandwich
T_WORLD_TO_CV @ R @ T_CV_TO_BODY. Baro 'up' is world -Y."""
from __future__ import annotations

import numpy as np

from ..sensor_time import slerp
from ..sensors import quat_mul, quat_to_matrix, T_CV_TO_BODY, T_WORLD_TO_CV

__all__ = ["apply_quat_phase", "slerp", "prior_rotation", "predict_pose",
           "world_up", "baro_height_m"]


def apply_quat_phase(quat, gyro_dps_body, offset_us: float):
    """Propagate the SFLP orientation `quat` (body->world [w,x,y,z]) BACKWARD by
    `offset_us` microseconds using the body-frame gyro rate, returning the
    orientation at the depth-frame instant.

    BUG-031 measured the stream-9 quaternion (a batch MEAN) as sitting +7.76 ms
    AFTER the FRAME_READY edge -- it LEADS the depth frame -- so the orientation
    that belongs with this frame is the quat rolled back by that offset, not
    forward. During a fast pan that lead is a real rotation-prior error (~1.2 deg
    at 156 deg/s) that translation/soft-prior ICP turns into fabricated
    translation (BUG-067). `quat_offset_us` is positive when the quat leads;
    `offset_us <= 0` or a None/zero gyro is a no-op.

    Kinematics: for a body->world quaternion, q(t+dt) = q ⊗ exp(1/2 w_body dt),
    so rolling back by `offset` right-multiplies by the conjugate increment."""
    if offset_us is None or offset_us <= 0.0 or gyro_dps_body is None:
        return quat
    w = np.asarray(gyro_dps_body, dtype=np.float64).reshape(3)
    dt = offset_us / 1e6
    theta_vec = np.radians(w) * dt                  # body-frame rotation over the lead
    half = 0.5 * theta_vec
    ang = float(np.linalg.norm(half))
    if ang < 1e-12:
        return quat
    axis = half / ang
    dq = (np.cos(ang), *(np.sin(ang) * axis))       # forward increment exp(1/2 w dt)
    dq_conj = (dq[0], -dq[1], -dq[2], -dq[3])        # roll BACK by the lead
    return quat_mul(tuple(float(c) for c in quat), dq_conj)


# slerp lives in roomscan.sensor_time since #155 (shared with the timestamped
# quaternion buffer without pulling Open3D); re-exported above for existing callers.


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
