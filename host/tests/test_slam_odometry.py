import numpy as np
import open3d as o3d
import pytest

from roomscan.slam.odometry import register, RegistrationResult


def _plane_cloud(n=40, z=1.0):
    xs, ys = np.meshgrid(np.linspace(-0.5, 0.5, n), np.linspace(-0.4, 0.4, n))
    pts = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)], axis=1).astype(np.float32)
    # add mild curvature so ICP has translational grip in x and y too
    pts[:, 2] += 0.15 * (pts[:, 0] ** 2 + pts[:, 1] ** 2)
    pc = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    pc.point.positions = o3d.core.Tensor(pts)
    pc.estimate_normals()
    return pc


def test_translation_recovered():
    target = _plane_cloud()
    src_pts = target.point.positions.numpy().copy()
    shift = np.array([0.03, -0.02, 0.04], dtype=np.float32)
    src_pts += shift
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(src_pts)
    res = register(source, target, np.eye(4), mode="translation")
    assert res.ok
    # source-to-target moves source back by -shift
    assert np.allclose(res.pose[:3, 3], -shift, atol=0.01)
    assert np.allclose(res.pose[:3, :3], np.eye(3), atol=1e-9)   # rotation held


def test_6dof_leaves_rotation_free():
    # Strengthened beyond the brief: identical clouds registered in 6dof mode
    # from init=eye(4) must converge to (near-)identity — this genuinely
    # exercises the "6dof runs unmodified ICP" behavior instead of merely
    # checking isinstance/shape, which would pass even for a stub.
    target = _plane_cloud()
    src = _plane_cloud()
    res = register(src, target, np.eye(4), mode="6dof")
    assert isinstance(res, RegistrationResult)
    assert res.pose.shape == (4, 4)
    assert res.ok
    assert np.allclose(res.pose[:3, :3], np.eye(3), atol=1e-2)
    assert np.allclose(res.pose[:3, 3], np.zeros(3), atol=1e-2)


def test_low_overlap_trips_gate():
    target = _plane_cloud()
    far = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    far.point.positions = o3d.core.Tensor((target.point.positions.numpy() + 5.0).astype(np.float32))
    res = register(far, target, np.eye(4), mode="translation", max_dist=0.05)
    assert not res.ok
