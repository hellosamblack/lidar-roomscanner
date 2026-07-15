"""Pure geometry for the first-person IR overlay: a camera-locked billboard
quad spanning the sensor FoV at a fixed distance in front of the eye (spec
§5.2). No Open3D imports -- unit-tested. The panel textures this with the live
IR image and material defaultUnlitTransparency at base_color alpha == opacity.
"""
from __future__ import annotations

import numpy as np


def camera_locked_quad(eye, forward, up, fov_h_deg, fov_v_deg, dist):
    eye = np.asarray(eye, dtype=np.float64)
    fwd = np.asarray(forward, dtype=np.float64)
    fwd = fwd / (np.linalg.norm(fwd) + 1e-12)
    up = np.asarray(up, dtype=np.float64)
    right = np.cross(fwd, up)
    right /= (np.linalg.norm(right) + 1e-12)
    quad_up = np.cross(right, fwd)
    quad_up /= (np.linalg.norm(quad_up) + 1e-12)
    half_w = dist * np.tan(np.deg2rad(fov_h_deg) / 2.0)
    half_v = dist * np.tan(np.deg2rad(fov_v_deg) / 2.0)
    center = eye + dist * fwd
    # Corners match capture_square_corners' vertical convention: its camera-y
    # is physically down, so its "top" corners use -half_v along camera-y,
    # which is == +half_v along quad_up here. Order: TL, TR, BR, BL.
    tl = center - half_w * right + half_v * quad_up
    tr = center + half_w * right + half_v * quad_up
    br = center + half_w * right - half_v * quad_up
    bl = center - half_w * right - half_v * quad_up
    verts = np.vstack([tl, tr, br, bl]).astype(np.float64)
    # v is flipped (row 0 of the source image -> v=1) because Open3D/Filament
    # samples textures with a bottom-left origin (OpenGL convention) while the
    # reflectance image (reflectance_to_rgb / o3d.geometry.Image) is row-major
    # top-down like every other array in this codebase. Mapping TL -> v=0
    # verbatim rendered the billboard upside down (owner, on-rig 2026-07-15).
    uvs = np.array([[0, 1], [1, 1], [1, 0], [0, 0]], dtype=np.float32)
    tris = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    return verts, uvs, tris
