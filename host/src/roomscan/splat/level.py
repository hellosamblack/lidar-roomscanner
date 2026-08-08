"""Estimate an upright/centred/scaled transform for a COLMAP model.

COLMAP's world frame has an arbitrary origin, orientation and scale, so a raw
splat renders tilted and either microscopic or vast.  A phone walkthrough is held
roughly upright, so the average camera "down" axis approximates gravity -- we use
it to level the model, recentre it on the camera path, and normalise scale to a
few viewer units.  The result is a 4x4 the web client applies to the splat; the
UI's manual sliders fine-tune from there (COLMAP up is only ever approximate).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np


def _rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotation matrix taking unit vector ``a`` onto unit vector ``b`` (Rodrigues)."""
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c > 1 - 1e-8:
        return np.eye(3)
    if c < -1 + 1e-8:                       # antiparallel: 180deg about any perp axis
        axis = np.cross(a, [1.0, 0, 0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0, 1.0, 0])
        axis /= np.linalg.norm(axis)
        vx = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return np.eye(3) + 2 * (vx @ vx)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def estimate_upright_transform(model_dir: str | Path, *, target_radius: float = 3.0) -> list[list[float]]:
    """Return a 4x4 (row-major list) mapping COLMAP world -> a levelled viewer frame.

    Gravity (world +Y, "down", matching the viewer's ``camera.up = (0,-1,0)``) is
    estimated from the mean camera down-axis; the model is recentred on the
    camera-path centroid and scaled so its 90th-percentile point radius is
    ``target_radius`` units.  Degrades to identity-ish (centre + scale only) if
    the reconstruction can't be read.
    """
    try:
        import pycolmap
        rec = pycolmap.Reconstruction(str(model_dir))
    except Exception:
        return np.eye(4).tolist()

    downs, centers = [], []
    for img in rec.images.values():
        if not img.has_pose:
            continue
        w2c = img.cam_from_world().matrix()               # 3x4 world->cam
        downs.append(w2c[:3, :3].T @ np.array([0.0, 1.0, 0.0]))  # cam +Y (down) in world
        centers.append(np.linalg.inv(np.vstack([w2c, [0, 0, 0, 1]]))[:3, 3])
    if not centers:
        return np.eye(4).tolist()

    centers = np.stack(centers)
    gravity = np.mean(downs, axis=0)
    R = _rotation_between(gravity, np.array([0.0, 1.0, 0.0]))

    centroid = centers.mean(axis=0)
    pts = np.stack([p.xyz for p in rec.points3D.values()]) if rec.num_points3D() else centers
    r90 = float(np.percentile(np.linalg.norm(pts - centroid, axis=1), 90)) or 1.0
    scale = target_radius / r90

    M = scale * R
    t = -M @ centroid
    T = np.eye(4)
    T[:3, :3] = M
    T[:3, 3] = t
    return T.tolist()
