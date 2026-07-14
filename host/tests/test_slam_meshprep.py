import numpy as np
import pytest

pytest.importorskip("open3d")
import open3d as o3d

from roomscan.slam.meshprep import MeshPacket, prepare_packet, _submesh_arrays


def _corner_tensor_mesh():
    """One unambiguous wall triangle (normal ~world-Z, perpendicular to
    world-up [0,-1,0]) + one unambiguous floor triangle (normal ~world-Y).
    Same fixture shape as test_panel_walls.py."""
    verts = np.array([
        [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0],   # wall triangle
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0],   # floor triangle
    ], dtype=np.float32)
    tris = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int32)
    m = o3d.t.geometry.TriangleMesh()
    m.vertex.positions = o3d.core.Tensor(verts)
    m.triangle.indices = o3d.core.Tensor(tris)
    return m


def _grid_tensor_mesh(n=40):
    """A dense flat grid mesh with ~n*n vertices, for exercising decimation."""
    xs, ys = np.meshgrid(np.linspace(0, 1, n), np.linspace(0, 1, n))
    verts = np.stack([xs.ravel(), ys.ravel(), np.zeros(n * n)], axis=1).astype(np.float32)
    tris = []
    for r in range(n - 1):
        for c in range(n - 1):
            a = r * n + c; b = a + 1; d = a + n; e = d + 1
            tris.append([a, b, d]); tris.append([b, e, d])
    m = o3d.t.geometry.TriangleMesh()
    m.vertex.positions = o3d.core.Tensor(verts)
    m.triangle.indices = o3d.core.Tensor(np.asarray(tris, np.int32))
    return m


def test_submesh_arrays_remaps_to_dense_indices():
    verts = np.arange(18, dtype=np.float64).reshape(6, 3)
    colors = np.zeros((6, 3))
    tris = np.array([[3, 4, 5]], dtype=np.int32)   # references only verts 3,4,5
    v, c, t = _submesh_arrays(verts, colors, tris)
    assert v.shape == (3, 3)
    np.testing.assert_array_equal(v, verts[3:6])
    np.testing.assert_array_equal(t, np.array([[0, 1, 2]], dtype=np.int32))


def test_solid_mode_puts_whole_mesh_in_non_wall_no_walls():
    m = _corner_tensor_mesh()
    pkt = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=7,
                         vertex_budget=10_000, decimate=False)
    assert isinstance(pkt, MeshPacket)
    assert pkt.mesh_seq == 7
    assert pkt.decimated is False
    assert pkt.wall_mode == "solid"
    assert len(pkt.non_wall_tris) == 2       # whole mesh, unsplit
    assert len(pkt.wall_tris) == 0
    assert pkt.non_wall_colors.shape == pkt.non_wall_verts.shape


def test_translucent_mode_splits_wall_from_floor():
    m = _corner_tensor_mesh()
    pkt = prepare_packet(m, wall_mode="translucent", glow_origin=None, mesh_seq=1,
                         vertex_budget=10_000, decimate=False)
    assert len(pkt.non_wall_tris) == 1       # the floor triangle
    assert len(pkt.wall_tris) == 1           # the wall triangle
    # dense-remapped: each submesh's triangle indices point inside its own verts
    assert pkt.non_wall_tris.max() < len(pkt.non_wall_verts)
    assert pkt.wall_tris.max() < len(pkt.wall_verts)


def test_floor_grid_populated_from_vertices():
    m = _corner_tensor_mesh()
    pkt = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                         vertex_budget=10_000, decimate=False)
    # the corner fixture spans x,z in [0,1] -> a non-degenerate floor grid
    assert len(pkt.floor_pts) >= 2
    assert len(pkt.floor_lines) >= 1


def test_decimation_kicks_in_past_budget():
    m = _grid_tensor_mesh(40)                # ~1600 verts
    n_src = len(m.vertex.positions)
    pkt = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                         vertex_budget=200, decimate=True)
    assert pkt.decimated is True
    assert pkt.source_vertex_count == n_src
    assert len(pkt.non_wall_verts) < n_src   # actually reduced


def test_no_decimation_below_budget_or_when_disabled():
    m = _grid_tensor_mesh(20)                # ~400 verts
    n_src = len(m.vertex.positions)
    # below budget: stays full-res even with decimate=True
    pkt = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                         vertex_budget=10_000, decimate=True)
    assert pkt.decimated is False
    assert len(pkt.non_wall_verts) == n_src
    # over budget but decimate=False: still full-res
    pkt2 = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                          vertex_budget=10, decimate=False)
    assert pkt2.decimated is False
    assert len(pkt2.non_wall_verts) == n_src


def test_translucent_mode_no_triangles_takes_full_res_branch():
    # vertices but ZERO triangles -> the full-res branch fires even in
    # translucent mode: everything lands in non_wall_*, walls stay empty.
    verts = np.array([
        [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0],
    ], dtype=np.float32)
    m = o3d.t.geometry.TriangleMesh()
    m.vertex.positions = o3d.core.Tensor(verts)
    m.triangle.indices = o3d.core.Tensor(np.zeros((0, 3), np.int32))
    pkt = prepare_packet(m, wall_mode="translucent", glow_origin=None, mesh_seq=3,
                         vertex_budget=10_000, decimate=False)
    assert len(pkt.non_wall_tris) == 0
    assert len(pkt.wall_tris) == 0
    assert len(pkt.non_wall_verts) == len(verts)


def test_glow_origin_changes_colors():
    m = _corner_tensor_mesh()
    base = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                          vertex_budget=10_000, decimate=False)
    glowed = prepare_packet(m, wall_mode="solid", glow_origin=np.array([0.0, 0.0, 0.0]),
                            mesh_seq=0, vertex_budget=10_000, decimate=False)
    assert not np.allclose(base.non_wall_colors, glowed.non_wall_colors)


from roomscan.slam.meshprep import MeshPrep


def test_meshprep_run_once_publishes_packet_and_consumes_input():
    m = _corner_tensor_mesh()
    prep = MeshPrep(vertex_budget=10_000, fps_budget_ms=8.0)
    assert prep.latest() is None
    assert prep.run_once() is False        # empty input slot
    prep.submit(m, mesh_seq=3, glow_origin=None, wall_mode="solid")
    assert prep.run_once() is True
    pkt = prep.latest()
    assert pkt is not None and pkt.mesh_seq == 3
    assert prep.latest() is None           # consume-once


def test_meshprep_latest_wins_input():
    m = _corner_tensor_mesh()
    prep = MeshPrep(vertex_budget=10_000)
    prep.submit(m, mesh_seq=1, glow_origin=None, wall_mode="solid")
    prep.submit(m, mesh_seq=2, glow_origin=None, wall_mode="solid")  # overwrites
    prep.run_once()
    assert prep.latest().mesh_seq == 2      # only the newest survives


def test_meshprep_adaptive_decimates_after_slow_upload():
    m = _grid_tensor_mesh(40)               # ~1600 verts
    prep = MeshPrep(vertex_budget=200, fps_budget_ms=8.0)
    # last upload was fast -> full-res
    prep.submit(m, mesh_seq=1, glow_origin=None, wall_mode="solid")
    prep.run_once()
    assert prep.latest().decimated is False
    # report a slow upload -> next packet decimates
    prep.note_upload_ms(50.0)
    prep.submit(m, mesh_seq=2, glow_origin=None, wall_mode="solid")
    prep.run_once()
    assert prep.latest().decimated is True
