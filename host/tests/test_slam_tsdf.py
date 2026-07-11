import numpy as np
import open3d as o3d
from roomscan.slam.tsdf import TsdfMap
from roomscan.slam.intrinsics import pinhole

W, H = 54, 42

def _wall_depth(z_m=1.0):
    # a flat wall at constant z; depth image in millimetres
    return np.full((H, W), z_m * 1000.0, dtype=np.float32)

def test_raycast_none_before_integrate():
    m = TsdfMap(voxel_size=0.02)
    K = pinhole(W, H)
    assert m.raycast(K, np.eye(4), W, H) is None

def test_integrate_then_raycast_recovers_wall():
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    depth = _wall_depth(1.0)
    m.integrate(depth, K, np.eye(4))               # identity pose: world==camera
    model = m.raycast(K, np.eye(4), W, H)
    assert model is not None
    pts = model.point.positions.numpy()
    assert len(pts) > 500
    # the wall sits near z=1.0 m in camera/world frame
    assert abs(np.median(pts[:, 2]) - 1.0) < m_voxel_tol()
    # normals point roughly back toward the camera (-z)
    nz = model.point.normals.numpy()[:, 2]
    assert np.median(nz) < -0.5

def m_voxel_tol():
    return 0.05  # within a few voxels of the true plane

def test_raycast_with_depth_hint_matches_full_raycast():
    # Task 9.5 Lever 1: raycast() bounded to the current-view frustum (via a
    # depth_hint) must return geometry equivalent to the original
    # all-active-blocks path, not merely "some points".
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    depth = _wall_depth(1.0)
    m.integrate(depth, K, np.eye(4))
    full = m.raycast(K, np.eye(4), W, H)
    bounded = m.raycast(K, np.eye(4), W, H, depth_hint=depth)
    assert full is not None and bounded is not None
    full_pts = full.point.positions.numpy()
    bounded_pts = bounded.point.positions.numpy()
    # same wall recovered from the bounded query
    assert abs(np.median(bounded_pts[:, 2]) - np.median(full_pts[:, 2])) < 1e-6
    assert abs(len(bounded_pts) - len(full_pts)) <= 2   # frustum == whole map here

def test_raycast_with_explicit_block_coords():
    # frustum_block_coords() + raycast(block_coords=...) is the lower-level
    # entry point Mapper uses when it wants to reuse computed coords.
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    depth = _wall_depth(1.0)
    m.integrate(depth, K, np.eye(4))
    coords = m.frustum_block_coords(depth, K, np.eye(4))
    assert coords.shape[0] > 0
    model = m.raycast(K, np.eye(4), W, H, block_coords=coords)
    assert model is not None
    pts = model.point.positions.numpy()
    assert len(pts) > 500
    assert abs(np.median(pts[:, 2]) - 1.0) < m_voxel_tol()

def test_mesh_and_point_cloud_on_empty_map_return_empty_not_raise():
    # Task 10 bugfix: extract_triangle_mesh()/extract_point_cloud() raise a
    # C++ HashMap error ("Input number of keys should > 0") when nothing has
    # ever been integrated. mesh()/point_cloud() must guard this and return
    # an empty geometry of the correct type instead.
    m = TsdfMap(voxel_size=0.02)
    mesh = m.mesh()
    assert isinstance(mesh, o3d.t.geometry.TriangleMesh)
    assert len(mesh.vertex.positions) == 0

    pc = m.point_cloud()
    assert isinstance(pc, o3d.t.geometry.PointCloud)
    assert pc.point.positions.numpy().shape[0] == 0

def test_raycast_empty_map_with_depth_hint_returns_none():
    # The empty-map guard must fire before any block-coord computation, even
    # when a depth_hint is supplied (Mapper may pass one before any
    # integration has happened, e.g. after a lost bootstrap frame).
    m = TsdfMap(voxel_size=0.02)
    K = pinhole(W, H)
    assert m.raycast(K, np.eye(4), W, H, depth_hint=_wall_depth(1.0)) is None
