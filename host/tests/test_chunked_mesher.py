"""Tests for `roomscan.slam.chunked_mesher` -- issue #111.

Partitioning, halo/weld parity against unchunked extraction, and the
boundary-edge seam check the spec calls for (`docs/superpowers/specs/
2026-08-19-slam-mesh-ceiling-design.md`, "Benchmark plan" -> Correctness).
"""
import numpy as np
import open3d as o3d
import pytest

from roomscan.slam.chunked_mesher import (
    partition_block_coords,
    _halo_expand,
    weld_meshes,
    boundary_edge_count,
    chunked_extract_triangle_mesh,
)
from roomscan.slam.tsdf import TsdfMap

CPU = o3d.core.Device("CPU:0")


# --------------------------------------------------------------- partitioning

def test_partition_is_disjoint_and_complete():
    rng = np.random.default_rng(0)
    coords = np.unique(rng.integers(-20, 20, size=(500, 3)), axis=0)
    chunks = partition_block_coords(coords, chunk_blocks=4)

    seen = set()
    total = 0
    for chunk_id, block_coords in chunks.items():
        total += len(block_coords)
        for c in block_coords:
            key = tuple(int(x) for x in c)
            assert key not in seen, "a block landed in more than one chunk"
            seen.add(key)
            # every block in this chunk must actually floor-divide to this id
            assert tuple(int(x) // 4 for x in c) == chunk_id
    assert total == len(coords)
    assert seen == {tuple(int(x) for x in c) for c in coords}


def test_partition_rejects_nonpositive_chunk_blocks():
    with pytest.raises(ValueError):
        partition_block_coords(np.zeros((1, 3), dtype=np.int64), chunk_blocks=0)


def test_partition_single_chunk_when_chunk_blocks_covers_everything():
    coords = np.array([[0, 0, 0], [1, 1, 1], [3, 3, 3]])
    chunks = partition_block_coords(coords, chunk_blocks=100)
    assert len(chunks) == 1
    only = next(iter(chunks.values()))
    assert len(only) == 3


# ---------------------------------------------------------------------- halo

def test_halo_expand_includes_only_active_neighbors_within_distance():
    # a 3x3x3 cube of active blocks centered on the origin
    active = {(x, y, z) for x in range(-1, 2) for y in range(-1, 2) for z in range(-1, 2)}
    core = [(0, 0, 0)]
    out = _halo_expand(core, active, halo=1)
    # all 27 neighbors (including self) are active, so all 27 come back
    assert len(out) == 27
    assert set(out) == active


def test_halo_expand_omits_coordinates_absent_from_active_set():
    active = {(0, 0, 0)}   # (1, 0, 0) etc. were never integrated
    out = _halo_expand([(0, 0, 0)], active, halo=1)
    assert out == [(0, 0, 0)]


def test_halo_expand_zero_halo_is_identity_on_active_core():
    active = {(0, 0, 0), (5, 5, 5)}
    out = _halo_expand([(0, 0, 0)], active, halo=0)
    assert out == [(0, 0, 0)]


# --------------------------------------------------------------- weld_meshes

def _mesh_from_arrays(verts, tris, cols=None):
    m = o3d.t.geometry.TriangleMesh(device=CPU)
    m.vertex.positions = o3d.core.Tensor(np.asarray(verts, dtype=np.float32), device=CPU)
    m.vertex.colors = o3d.core.Tensor(
        np.zeros((len(verts), 3), dtype=np.float32) if cols is None else np.asarray(cols, dtype=np.float32),
        device=CPU)
    m.triangle.indices = o3d.core.Tensor(np.asarray(tris, dtype=np.int32), device=CPU)
    return m


def test_weld_merges_exact_duplicate_vertices_and_drops_duplicate_triangles():
    # two single-triangle meshes sharing all three vertices exactly (the
    # "recomputed halo block" case) -- weld must collapse to ONE triangle,
    # not two, and to 3 vertices, not 6.
    v = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
    t = [[0, 1, 2]]
    a = _mesh_from_arrays(v, t)
    b = _mesh_from_arrays(v, t)
    out = weld_meshes([a, b], device=CPU)
    assert len(out.vertex.positions) == 3
    assert len(out.triangle.indices) == 1


def test_weld_keeps_distinct_geometry_from_non_overlapping_chunks():
    a = _mesh_from_arrays([[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 1, 2]])
    b = _mesh_from_arrays([[10, 0, 0], [11, 0, 0], [10, 1, 0]], [[0, 1, 2]])
    out = weld_meshes([a, b], device=CPU)
    assert len(out.vertex.positions) == 6
    assert len(out.triangle.indices) == 2


def test_weld_merges_a_shared_seam_edge_between_two_chunks():
    # two triangles sharing exactly one edge (two vertices), as chunks
    # abutting along a partition plane would after independent extraction
    # with a halo -- the shared vertices must collapse, the distinct ones
    # must not.
    a = _mesh_from_arrays([[0, 0, 0], [0, 1, 0], [1, 0, 0]], [[0, 1, 2]])
    b = _mesh_from_arrays([[0, 0, 0], [0, 1, 0], [-1, 0, 0]], [[0, 1, 2]])
    out = weld_meshes([a, b], device=CPU)
    assert len(out.vertex.positions) == 4   # 2 shared + 2 distinct
    assert len(out.triangle.indices) == 2


def test_weld_empty_input_returns_empty_mesh():
    out = weld_meshes([], device=CPU)
    assert len(out.vertex.positions) == 0
    assert len(out.triangle.indices) == 0


# --------------------------------------------------------------- boundary edges

def test_boundary_edge_count_of_a_single_open_triangle_is_three():
    # a lone triangle: all 3 edges belong to exactly 1 triangle
    assert boundary_edge_count(np.array([[0, 1, 2]])) == 3


def test_boundary_edge_count_of_two_triangles_sharing_an_edge_is_four():
    # a quad made of 2 triangles: the shared edge is interior (count 2,
    # excluded), the other 4 edges are boundary
    tris = np.array([[0, 1, 2], [0, 2, 3]])
    assert boundary_edge_count(tris) == 4


def test_boundary_edge_count_of_closed_tetrahedron_is_zero():
    tris = np.array([[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]])
    assert boundary_edge_count(tris) == 0


# ------------------------------------------------------- integration: parity

W, H = 200, 150


def _wide_intrinsic():
    fx = fy = 90.0
    cx, cy = W / 2, H / 2
    return o3d.core.Tensor(
        np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64), device=CPU)


def _wall_depth(z_m=1.0):
    return np.full((H, W), z_m * 1000.0, dtype=np.float32)


def _build_wide_wall_map(voxel_size=0.02, integrations=4):
    m = TsdfMap(voxel_size=voxel_size, depth_max=5.0, block_count=4000)
    K = _wide_intrinsic()
    depth = _wall_depth(1.0)
    for _ in range(integrations):
        m.integrate(depth, K, np.eye(4))
    return m


def test_chunked_matches_unchunked_vertex_and_triangle_counts_on_small_grid():
    m = _build_wide_wall_map()
    blocks, _cap = m.block_usage()
    assert blocks > 200   # exercise real chunking, not a single-chunk no-op

    whole = m.mesh()
    chunked = chunked_extract_triangle_mesh(
        m._vbg, voxel_size=m.voxel_size, block_resolution=m.block_resolution,
        weight_threshold=m.weight_threshold, chunk_blocks=4, halo=1)

    n_whole_v, n_whole_t = len(whole.vertex.positions), len(whole.triangle.indices)
    n_chunk_v, n_chunk_t = len(chunked.vertex.positions), len(chunked.triangle.indices)
    assert n_whole_v > 1000   # sanity: this is a real mesh, not a degenerate one
    assert n_chunk_v == n_whole_v
    assert n_chunk_t == n_whole_t


def test_chunked_seam_introduces_zero_boundary_edge_delta():
    m = _build_wide_wall_map()
    whole = m.mesh()
    chunked = chunked_extract_triangle_mesh(
        m._vbg, voxel_size=m.voxel_size, block_resolution=m.block_resolution,
        weight_threshold=m.weight_threshold, chunk_blocks=4, halo=1)

    whole_boundary = boundary_edge_count(whole.triangle.indices.numpy())
    chunked_boundary = boundary_edge_count(chunked.triangle.indices.numpy())
    # this open-plane wall mesh DOES have real boundary edges (the finite
    # depth-image FOV) -- assert the count is unchanged by chunking, and
    # nonzero so this check has something to see (not a vacuous 0 == 0).
    assert whole_boundary > 0
    assert chunked_boundary == whole_boundary


def test_chunked_matches_unchunked_bounding_box_and_area():
    m = _build_wide_wall_map()
    whole = m.mesh()
    chunked = chunked_extract_triangle_mesh(
        m._vbg, voxel_size=m.voxel_size, block_resolution=m.block_resolution,
        weight_threshold=m.weight_threshold, chunk_blocks=4, halo=1)

    def area(mesh):
        v = mesh.vertex.positions.numpy()
        t = mesh.triangle.indices.numpy()
        a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
        return 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum()

    whole_area = area(whole)
    chunked_area = area(chunked)
    assert whole_area > 0.5   # sanity: not a degenerate/empty mesh
    assert abs(chunked_area - whole_area) < 1e-4   # metres^2

    wv = whole.vertex.positions.numpy()
    cv = chunked.vertex.positions.numpy()
    assert np.allclose(wv.min(axis=0), cv.min(axis=0), atol=1e-6)
    assert np.allclose(wv.max(axis=0), cv.max(axis=0), atol=1e-6)


def test_chunked_extract_on_empty_map_returns_empty_mesh():
    m = TsdfMap(voxel_size=0.02)
    out = chunked_extract_triangle_mesh(
        m._vbg, voxel_size=m.voxel_size, block_resolution=m.block_resolution,
        weight_threshold=m.weight_threshold, chunk_blocks=4, halo=1)
    assert len(out.vertex.positions) == 0
    assert len(out.triangle.indices) == 0


def test_chunked_with_single_chunk_covering_everything_matches_whole_grid_call():
    # chunk_blocks large enough that partition_block_coords produces exactly
    # ONE chunk (no halo neighbors needed at all) -- degenerate case that
    # should reduce to "extract once", same numbers as calling
    # extract_triangle_mesh() directly.
    m = _build_wide_wall_map()
    whole = m.mesh()
    chunked = chunked_extract_triangle_mesh(
        m._vbg, voxel_size=m.voxel_size, block_resolution=m.block_resolution,
        weight_threshold=m.weight_threshold, chunk_blocks=1000, halo=1)
    assert len(chunked.vertex.positions) == len(whole.vertex.positions)
    assert len(chunked.triangle.indices) == len(whole.triangle.indices)
