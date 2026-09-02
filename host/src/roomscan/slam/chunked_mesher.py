"""Chunked mesh extraction for `TsdfMap` -- issue #111 / the mesh-extraction
ceiling spec (`docs/superpowers/specs/2026-08-19-slam-mesh-ceiling-design.md`).

Open3D 0.19's marching-cubes extractor (`extract_triangle_mesh`,
`extract_point_cloud`) crashes the *process* -- a host segfault or a CUDA
illegal-memory-access that aborts in a destructor -- above ~260k active
blocks, on either device (see `tsdf.py`'s `_MAX_SAFE_EXTRACT_BLOCKS`
bisection). Raising `block_count` does not help; the trigger is the absolute
active-block count handed to a SINGLE extraction call.

This module never makes that call on the whole grid. It partitions the
VoxelBlockGrid's active blocks into small, disjoint, block-coordinate-aligned
CHUNKS; for each chunk it builds a small standalone `VoxelBlockGrid`
containing that chunk's blocks PLUS a `halo` of immediate neighbor blocks
(copied straight out of the source hashmap's key/value tensors, which are
addressed by the same buffer index -- see `_active_block_table`); extracts a
mesh from that small grid (always far below the crash threshold, since a
chunk's block count is bounded by `chunk_blocks**3`); and welds the per-chunk
meshes back into one.

Why the halo is necessary, and why welding (not cropping) is how the seam is
closed: Open3D's block-local marching cubes needs each block's *neighbor's*
first voxel layer to interpolate the isosurface across the block boundary --
confirmed empirically (a chunk extracted with NO halo leaves gaps at every
chunk face; halo=1 closes them). Giving a block's neighbor context via the
halo means that block's own interior gets recomputed by BOTH the chunk that
owns it as "core" and every neighboring chunk that includes it only as halo
context -- but since marching cubes is a pure function of a block's own
voxels + its immediate neighbors' voxels, and every copy of a given block's
voxel data is byte-identical (same source tensors), the redundant
recomputation produces BIT-IDENTICAL vertices and triangles wherever it
overlaps. So welding by exact (quantized) vertex-position matching, then
dropping triangles whose vertex-id set repeats, exactly reconstructs the
single-extraction result (verified: 553-active-block synthetic wall, chunk
size 4, halo 1 -> 20 chunks, 20829 raw verts / 39370 raw tris collapsing to
9213 / 18040 after welding -- bit-identical to `extract_triangle_mesh()` on
the same grid unchunked).
"""
from __future__ import annotations

from typing import Iterable

import numpy as np
import open3d as o3d

# Conservative per-chunk ceiling, independent of `tsdf.py`'s
# `_MAX_SAFE_EXTRACT_BLOCKS` (250,000, itself already well below the
# bisected 273,521-block crash) -- a chunk is meant to stay orders of
# magnitude smaller than either. Guards against a caller passing an
# unreasonably large `chunk_blocks`/`halo` and accidentally recreating the
# whole-grid crash one "chunk" at a time.
_MAX_CHUNK_EXTRACT_BLOCKS = 100_000

DEFAULT_CHUNK_BLOCKS = 24
DEFAULT_HALO = 1

_ATTR_NAMES = ("tsdf", "weight", "color")
_ATTR_DTYPES = (o3d.core.float32, o3d.core.float32, o3d.core.float32)
_ATTR_CHANNELS = ((1,), (1,), (3,))


def partition_block_coords(block_coords: np.ndarray, chunk_blocks: int
                            ) -> dict[tuple[int, int, int], np.ndarray]:
    """Group active block coordinates into disjoint cubic chunks.

    `block_coords` is (N, 3) int block-space coordinates (as returned by
    `hashmap().key_tensor()[active_buf_indices()]`). Chunk id is
    `floor(coord / chunk_blocks)` per axis -- an axis-aligned partition of
    block-coordinate space with no gaps and no overlap, so every input block
    lands in EXACTLY one chunk. Returns {chunk_id: block_coords_in_chunk}.
    """
    if chunk_blocks <= 0:
        raise ValueError(f"chunk_blocks must be positive, got {chunk_blocks}")
    block_coords = np.asarray(block_coords, dtype=np.int64).reshape(-1, 3)
    chunk_ids = np.floor_divide(block_coords, chunk_blocks)
    chunks: dict[tuple[int, int, int], list] = {}
    for coord, cid in zip(block_coords, chunk_ids):
        chunks.setdefault(tuple(int(x) for x in cid), []).append(tuple(int(x) for x in coord))
    return {k: np.array(v, dtype=np.int64) for k, v in chunks.items()}


def _halo_expand(core_coords: Iterable[tuple[int, int, int]],
                  active_set: set[tuple[int, int, int]], halo: int
                  ) -> list[tuple[int, int, int]]:
    """`core_coords` plus every neighbor within `halo` blocks (Chebyshev
    distance -- a full cube neighborhood, not just face-adjacent) THAT IS
    ACTUALLY ACTIVE in `active_set`. A neighbor coordinate with no voxel data
    (never integrated) contributes nothing and is correctly omitted -- it is
    not in the source hashmap, so there is nothing to copy for it."""
    out: set[tuple[int, int, int]] = set()
    offsets = range(-halo, halo + 1)
    for (x, y, z) in core_coords:
        for dx in offsets:
            for dy in offsets:
                for dz in offsets:
                    cand = (x + dx, y + dy, z + dz)
                    if cand in active_set:
                        out.add(cand)
    return sorted(out)


def _active_block_table(vbg: o3d.t.geometry.VoxelBlockGrid):
    """(coord_to_buf, value_tensors, device) for every currently-active block
    in `vbg`'s hashmap. `coord_to_buf` maps a block coordinate tuple to the
    buffer index that indexes BOTH `key_tensor()` and every tensor in
    `value_tensors()` -- confirmed against the installed Open3D 0.19 build
    (both come back shaped (capacity, ...), sliced by the same active
    `active_buf_indices()`)."""
    hm = vbg.hashmap()
    idx = hm.active_buf_indices()
    keys = hm.key_tensor()[idx].cpu().numpy()
    idx_np = idx.cpu().numpy()
    coord_to_buf = {tuple(int(v) for v in k): int(i) for k, i in zip(keys, idx_np)}
    return coord_to_buf, hm.value_tensors(), hm.device


def _extract_chunk_mesh(coord_to_buf: dict, value_tensors: list,
                         chunk_coords: list[tuple[int, int, int]], *,
                         voxel_size: float, block_resolution: int,
                         weight_threshold: float,
                         device: o3d.core.Device) -> o3d.t.geometry.TriangleMesh:
    """Build a standalone VBG containing exactly `chunk_coords`' blocks
    (copied from the source's key/value tensors) and extract a mesh from it.
    `chunk_coords` is expected to already include the halo -- this function
    does not expand it."""
    if len(chunk_coords) > _MAX_CHUNK_EXTRACT_BLOCKS:
        raise ValueError(
            f"chunk (with halo) has {len(chunk_coords)} blocks, above the "
            f"per-chunk safety ceiling of {_MAX_CHUNK_EXTRACT_BLOCKS} -- pass "
            f"a smaller chunk_blocks/halo.")
    bufs = [coord_to_buf[c] for c in chunk_coords]
    sub = o3d.t.geometry.VoxelBlockGrid(
        attr_names=_ATTR_NAMES, attr_dtypes=_ATTR_DTYPES, attr_channels=_ATTR_CHANNELS,
        voxel_size=voxel_size, block_resolution=block_resolution,
        block_count=max(len(chunk_coords) * 2, 16), device=device)
    sel_keys = o3d.core.Tensor(np.array(chunk_coords, dtype=np.int32), device=device)
    sel_vals = [vt[bufs] for vt in value_tensors]
    sub.hashmap().insert(sel_keys, sel_vals)
    return sub.extract_triangle_mesh(weight_threshold)


def weld_meshes(meshes: list[o3d.t.geometry.TriangleMesh], *,
                 device: o3d.core.Device, tolerance_m: float = 1e-6
                 ) -> o3d.t.geometry.TriangleMesh:
    """Merge per-chunk meshes into one, deduplicating vertices that came from
    blocks recomputed in more than one chunk's halo (see the module
    docstring), and dropping the resulting duplicate triangles.

    `tolerance_m`: vertices within this distance (per-axis, via position
    quantization) are treated as the SAME vertex. Overlapping-block
    recomputation is deterministic and produces bit-identical positions (see
    module docstring), so the default is tight -- this is a welding pass, not
    a smoothing one, and a loose tolerance could incorrectly merge distinct
    nearby vertices on genuinely different surfaces.
    """
    verts = [m.vertex.positions.numpy() for m in meshes if len(m.vertex.positions) > 0]
    cols = [m.vertex.colors.numpy() for m in meshes if len(m.vertex.positions) > 0]
    tris = [m.triangle.indices.numpy() for m in meshes if len(m.vertex.positions) > 0]
    if not verts:
        empty = o3d.t.geometry.TriangleMesh(device=device)
        empty.vertex.positions = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=device)
        empty.vertex.colors = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=device)
        empty.triangle.indices = o3d.core.Tensor(np.zeros((0, 3), dtype=np.int32), device=device)
        return empty

    offsets = np.cumsum([0] + [len(v) for v in verts[:-1]])
    V = np.concatenate(verts, axis=0)
    C = np.concatenate(cols, axis=0)
    T = np.concatenate([t + off for t, off in zip(tris, offsets)], axis=0) if any(len(t) for t in tris) \
        else np.zeros((0, 3), dtype=np.int64)

    # Weld: quantize each vertex position to an integer grid of pitch
    # `tolerance_m` and dedupe on that key -- positions within tolerance
    # collide to the same key regardless of float noise between two
    # independently-run extractions.
    quant = np.round(V / tolerance_m).astype(np.int64)
    uniq: dict[tuple[int, int, int], int] = {}
    remap = np.empty(len(V), dtype=np.int64)
    new_pos: list = []
    new_col: list = []
    for i in range(len(V)):
        key = (int(quant[i, 0]), int(quant[i, 1]), int(quant[i, 2]))
        j = uniq.get(key)
        if j is None:
            j = len(new_pos)
            uniq[key] = j
            new_pos.append(V[i])
            new_col.append(C[i])
        remap[i] = j
    Vn = np.asarray(new_pos, dtype=np.float32) if new_pos else np.zeros((0, 3), dtype=np.float32)
    Cn = np.asarray(new_col, dtype=np.float32) if new_col else np.zeros((0, 3), dtype=np.float32)
    Tn = remap[T] if len(T) else np.zeros((0, 3), dtype=np.int64)

    # Dedupe triangles that now reference the same three welded vertices --
    # produced when an entire block's isosurface was recomputed in more than
    # one chunk (see module docstring). Order-independent (sorted vertex-id
    # triple) so winding order can't hide a duplicate.
    seen: set[tuple[int, int, int]] = set()
    keep_rows: list = []
    for row in Tn:
        key = tuple(sorted(int(x) for x in row))
        if key in seen:
            continue
        seen.add(key)
        keep_rows.append(row)
    Tf = np.asarray(keep_rows, dtype=np.int32) if keep_rows else np.zeros((0, 3), dtype=np.int32)

    out = o3d.t.geometry.TriangleMesh(device=device)
    out.vertex.positions = o3d.core.Tensor(Vn, device=device)
    out.vertex.colors = o3d.core.Tensor(Cn, device=device)
    out.triangle.indices = o3d.core.Tensor(Tf, device=device)
    return out


def boundary_edge_count(triangle_indices: np.ndarray) -> int:
    """Number of undirected edges belonging to exactly ONE triangle -- the
    mesh's open boundary (a hole or an unwelded seam). Used to compare a
    chunked-and-welded mesh against its unchunked reference: chunking must
    not CHANGE this count (a broken weld shows up as extra boundary edges
    running exactly along the partition planes)."""
    if len(triangle_indices) == 0:
        return 0
    edges: dict[tuple[int, int], int] = {}
    for a, b, c in triangle_indices:
        for u, v in ((a, b), (b, c), (c, a)):
            key = (int(u), int(v)) if u < v else (int(v), int(u))
            edges[key] = edges.get(key, 0) + 1
    return sum(1 for n in edges.values() if n == 1)


def chunked_extract_triangle_mesh(vbg: o3d.t.geometry.VoxelBlockGrid, *,
                                   voxel_size: float, block_resolution: int = 8,
                                   weight_threshold: float = 3.0,
                                   chunk_blocks: int = DEFAULT_CHUNK_BLOCKS,
                                   halo: int = DEFAULT_HALO,
                                   weld_tolerance_m: float = 1e-6,
                                   ) -> o3d.t.geometry.TriangleMesh:
    """Extract a triangle mesh from `vbg` in chunks small enough that no
    single `extract_triangle_mesh()` call ever sees the process-killing
    block count -- see the module docstring. Result is equivalent (vertex
    count, triangle count, boundary-edge count) to calling
    `vbg.extract_triangle_mesh(weight_threshold)` directly, which chunking
    exists precisely because that call cannot survive on a big-enough map.

    `chunk_blocks`: chunk side length in BLOCKS (not voxels/metres) -- a
    chunk can hold up to `chunk_blocks**3` blocks before halo expansion, so
    keep this well under the cube root of `_MAX_CHUNK_EXTRACT_BLOCKS`
    minus halo overhead for a dense map.
    `halo`: neighbor blocks (Chebyshev distance) included per chunk so
    marching cubes has the boundary context it needs; 1 is sufficient
    (Open3D's block-local MC only interpolates into the immediate
    neighbor's first voxel layer -- verified empirically, see module
    docstring) and is the default.
    """
    coord_to_buf, value_tensors, device = _active_block_table(vbg)
    if not coord_to_buf:
        empty = o3d.t.geometry.TriangleMesh(device=device)
        empty.vertex.positions = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=device)
        empty.vertex.colors = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32), device=device)
        empty.triangle.indices = o3d.core.Tensor(np.zeros((0, 3), dtype=np.int32), device=device)
        return empty
    active_set = set(coord_to_buf.keys())
    all_coords = np.array(list(coord_to_buf.keys()), dtype=np.int64)
    chunks = partition_block_coords(all_coords, chunk_blocks)

    meshes = []
    for core_coords in chunks.values():
        full = _halo_expand([tuple(c) for c in core_coords], active_set, halo)
        meshes.append(_extract_chunk_mesh(
            coord_to_buf, value_tensors, full,
            voxel_size=voxel_size, block_resolution=block_resolution,
            weight_threshold=weight_threshold, device=device))
    return weld_meshes(meshes, device=device, tolerance_m=weld_tolerance_m)
