"""Off-GUI-thread mesh preparation for the live SLAM view (Component A).

Takes the newest worker mesh, adaptively decimates it (display-only), bakes the
same shading `panel._upload_slam_mesh` uses, splits walls from floor/ceiling,
and extracts the floor grid -- all the O(map-size) work -- into a plain-data
`MeshPacket` the GUI tick can upload cheaply. The saved/offline map always comes
from the full-resolution `mapper.mesh()`; decimation here never touches it.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np

_IDLE_SLEEP_S = 0.005


@dataclass
class MeshPacket:
    non_wall_verts: np.ndarray     # (N,3) f64
    non_wall_colors: np.ndarray    # (N,3) f64
    non_wall_tris: np.ndarray      # (M,3) i32 -- dense indices into non_wall_verts
    wall_verts: np.ndarray         # (P,3) f64
    wall_colors: np.ndarray        # (P,3) f64
    wall_tris: np.ndarray          # (Q,3) i32 -- dense indices into wall_verts
    floor_pts: np.ndarray          # (K,3) f64
    floor_lines: np.ndarray        # (L,2) i64
    mesh_seq: int
    source_vertex_count: int
    decimated: bool
    wall_mode: str


def _submesh_arrays(verts: np.ndarray, colors: np.ndarray, tris: np.ndarray):
    """Dense-remap a triangle subset to 0..K-1, carrying the referenced verts +
    colors. Numpy twin of panel._wall_submesh (which builds a legacy mesh); this
    returns arrays so the packet stays GUI-handle-free."""
    if tris.shape[0] == 0:
        return (np.zeros((0, 3), np.float64), np.zeros((0, 3), np.float64),
                np.zeros((0, 3), np.int32))
    uniq, remap = np.unique(tris.reshape(-1), return_inverse=True)
    new_tris = remap.reshape(tris.shape).astype(np.int32)
    return verts[uniq], colors[uniq], new_tris


def prepare_packet(mesh, *, wall_mode: str, glow_origin, mesh_seq: int,
                   vertex_budget: int, decimate: bool, up=None) -> MeshPacket:
    """Pure: tensor SLAM/TSDF `mesh` -> ready-to-upload `MeshPacket`.

    Shading mirrors panel._upload_slam_mesh exactly (reflectance-meaningful ->
    grey * brightness * height-hue; else height-cued base * shade_colors), plus
    the live wavefront glow when `glow_origin` is not None. `decimate` (True when
    the adaptive controller says the last upload blew the frame budget) triggers
    quadric decimation to ~`vertex_budget` verts; below budget, or when False,
    the mesh passes through full-res (`decimated=False`)."""
    from .shading import (height_base_colors, height_tint_hue,
                          mesh_colors_are_meaningful, shade_brightness,
                          shade_colors, wall_triangle_mask, wavefront_glow)
    from .frames import world_up
    from ..theme import floor_grid_lines
    if up is None:
        up = world_up()

    legacy = mesh.cpu().to_legacy()
    source_vertex_count = len(legacy.vertices)

    decimated = False
    n_tris = len(legacy.triangles)
    if decimate and source_vertex_count > vertex_budget and n_tris > 0:
        target_tris = max(4, int(n_tris * vertex_budget / source_vertex_count))
        legacy = legacy.simplify_quadric_decimation(
            target_number_of_triangles=target_tris)
        decimated = True

    legacy.compute_vertex_normals()
    normals = np.asarray(legacy.vertex_normals)
    verts = np.asarray(legacy.vertices)
    raw_colors = np.asarray(legacy.vertex_colors)
    if mesh_colors_are_meaningful(raw_colors):
        brightness = shade_brightness(normals)
        hue = height_tint_hue(verts, up)
        final_colors = np.clip(raw_colors * brightness[:, None] * hue, 0.0, 1.0)
    else:
        base = height_base_colors(verts, up)
        final_colors = shade_colors(normals, base=base)
    if glow_origin is not None:
        final_colors = wavefront_glow(verts, glow_origin, final_colors)

    floor_pts, floor_lines = (np.zeros((0, 3)), np.zeros((0, 2), np.int64))
    if len(verts) > 0:
        mn, mx = verts.min(axis=0), verts.max(axis=0)
        floor_pts, floor_lines = floor_grid_lines(mn, mx, up=up, spacing=0.5)

    tris = np.asarray(legacy.triangles)
    if wall_mode == "solid" or tris.shape[0] == 0:
        return MeshPacket(
            non_wall_verts=verts, non_wall_colors=final_colors, non_wall_tris=tris.astype(np.int32),
            wall_verts=np.zeros((0, 3)), wall_colors=np.zeros((0, 3)),
            wall_tris=np.zeros((0, 3), np.int32),
            floor_pts=floor_pts, floor_lines=floor_lines,
            mesh_seq=mesh_seq, source_vertex_count=source_vertex_count,
            decimated=decimated, wall_mode=wall_mode)

    legacy.compute_triangle_normals()
    wall_mask = wall_triangle_mask(np.asarray(legacy.triangle_normals), up=up)
    nw_v, nw_c, nw_t = _submesh_arrays(verts, final_colors, tris[~wall_mask])
    w_v, w_c, w_t = _submesh_arrays(verts, final_colors, tris[wall_mask])
    return MeshPacket(
        non_wall_verts=nw_v, non_wall_colors=nw_c, non_wall_tris=nw_t,
        wall_verts=w_v, wall_colors=w_c, wall_tris=w_t,
        floor_pts=floor_pts, floor_lines=floor_lines,
        mesh_seq=mesh_seq, source_vertex_count=source_vertex_count,
        decimated=decimated, wall_mode=wall_mode)
