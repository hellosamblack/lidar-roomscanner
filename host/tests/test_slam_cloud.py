import numpy as np
import open3d as o3d
from roomscan.slam.cloud import source_cloud


def test_source_cloud_keeps_only_valid():
    pts = np.zeros((2, 3, 3), dtype=np.float64)
    for r in range(2):
        for c in range(3):
            pts[r, c] = [c * 0.1, r * 0.1, 1.0 + 0.01 * (r * 3 + c)]
    valid = np.array([[True, False, True], [True, True, False]])
    pc = source_cloud(pts, valid)
    assert isinstance(pc, o3d.t.geometry.PointCloud)
    xyz = pc.point.positions.numpy()
    assert xyz.shape == (4, 3)                       # 4 valid cells
    assert xyz.dtype == np.float32
    # spot-check one kept point
    assert np.allclose(xyz[0], [0.0, 0.0, 1.0], atol=1e-6)


def test_source_cloud_attaches_intensity():
    pts = np.ones((1, 2, 3), dtype=np.float64)
    valid = np.array([[True, True]])
    inten = np.array([[0.25, 0.75]], dtype=np.float64)
    pc = source_cloud(pts, valid, inten)
    got = pc.point["intensity"].numpy()
    assert got.shape == (2, 1) and got.dtype == np.float32
    assert np.allclose(got.ravel(), [0.25, 0.75])
