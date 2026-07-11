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
