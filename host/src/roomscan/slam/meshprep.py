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
    # Wire bytes, filled by `MeshPrep` when it was given a `packer` so the
    # serialisation happens on its worker thread instead of the caller's event
    # loop (BUG-060). None when no packer was supplied.
    packed: bytes | None = None


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


class MeshPrep:
    """Runs `prepare_packet` off the GUI thread with latest-wins in/out slots and
    an adaptive decimation controller. Mirrors slam.worker.SlamWorker's threading
    shape (daemon thread, lock-guarded slots, bounded-join stop)."""

    def __init__(self, vertex_budget: int = 150_000, fps_budget_ms: float = 8.0,
                 up=None, packer=None):
        self._vertex_budget = int(vertex_budget)
        self._fps_budget_ms = float(fps_budget_ms)
        self._up = up
        # NOTE (BUG-060): `roomscan-web` deliberately does NOT arm this
        # controller. Its input is `note_upload_ms`, which only a frontend that
        # measures its own GPU upload can supply -- i.e. only panel.py -- so
        # `vertex_budget` is dead code on the web path. That was tried the
        # other way and measured: forcing decimation on took `prepare_packet`
        # from 178 ms p50 to 2440 ms p50 (max 8.4 s) on 1500 frames of
        # roomSweepFull, and GIL starvation from 11.9% of wall to 94.3%,
        # because `simplify_quadric_decimation` is C++ holding the GIL the
        # whole way. It cost 14x what it saved. web.py bounds the PUBLISH RATE
        # by payload size instead (`SlamRunner._mesh_bytes_per_s`), which is
        # free. Do not "fix" the dead budget by latching this on.
        # Optional off-thread packer (BUG-060). When set, `run_once` also
        # serialises the packet HERE, on this worker thread, so the caller's
        # event loop never pays for it. Kept optional so the shape of the
        # packed bytes stays a frontend concern (web.py's `pack_mesh`).
        self._packer = packer
        self._last_upload_ms = 0.0
        self._decimating = False
        # Plan item 2 (2026-08-02): stage timing for the mesh-prep half of the
        # live pipeline, mirroring SlamWorker.mesh_extract_ms. `prep_ms` is
        # `prepare_packet` (decimation/shading/wall-split, all host-side numpy
        # + Open3D legacy-mesh work -- no device queue involved, so this is a
        # true wall-clock cost on any device); `pack_ms` is the optional
        # `packer` call (BUG-060's off-thread wire serialization);
        # `payload_bytes` is `len(pkt.packed)` when a packer is set, else 0
        # (nothing was serialized here -- the caller does it, and does not
        # report back through this object).
        self._last_prep_ms = 0.0
        self._last_pack_ms = 0.0
        self._last_payload_bytes = 0

        self._in_lock = threading.Lock()
        self._in_slot = None            # (mesh, mesh_seq, glow_origin, wall_mode) | None
        self._out_lock = threading.Lock()
        self._out_slot = None           # MeshPacket | None

        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    @property
    def fps_budget_ms(self) -> float:
        return self._fps_budget_ms

    def submit(self, mesh, *, mesh_seq: int, glow_origin, wall_mode: str) -> None:
        with self._in_lock:
            self._in_slot = (mesh, mesh_seq, glow_origin, wall_mode)

    def note_upload_ms(self, ms: float) -> None:
        self._last_upload_ms = float(ms)

    def run_once(self) -> bool:
        with self._in_lock:
            item, self._in_slot = self._in_slot, None
        if item is None:
            return False
        mesh, mesh_seq, glow_origin, wall_mode = item
        if self._last_upload_ms > self._fps_budget_ms:
            self._decimating = True
        decimate = self._decimating
        t_prep0 = time.perf_counter()
        pkt = prepare_packet(mesh, wall_mode=wall_mode, glow_origin=glow_origin,
                             mesh_seq=mesh_seq, vertex_budget=self._vertex_budget,
                             decimate=decimate, up=self._up)
        self._last_prep_ms = (time.perf_counter() - t_prep0) * 1000.0
        if self._packer is not None:
            t_pack0 = time.perf_counter()
            pkt.packed = self._packer(pkt)
            self._last_pack_ms = (time.perf_counter() - t_pack0) * 1000.0
            self._last_payload_bytes = len(pkt.packed) if pkt.packed is not None else 0
        else:
            self._last_pack_ms = 0.0
            self._last_payload_bytes = 0
        with self._out_lock:
            self._out_slot = pkt
        return True

    # ---- instrumentation (plan item 2, 2026-08-02) ---------------------------
    @property
    def prep_ms(self) -> float:
        """Wall time of the most recent `prepare_packet` call. 0.0 before the
        first packet."""
        return self._last_prep_ms

    @property
    def pack_ms(self) -> float:
        """Wall time of the most recent `packer` call (0.0 if no packer was
        configured, or none has run yet)."""
        return self._last_pack_ms

    @property
    def payload_bytes(self) -> int:
        """Byte length of the most recently packed wire payload (0 if no
        packer was configured, or none has run yet)."""
        return self._last_payload_bytes

    def latest(self):
        with self._out_lock:
            pkt, self._out_slot = self._out_slot, None
        return pkt

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self) -> None:
        while not self._stop_evt.is_set():
            if not self.run_once():
                time.sleep(_IDLE_SLEEP_S)

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None
