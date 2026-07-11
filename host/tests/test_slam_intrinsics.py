import math
import numpy as np
import open3d as o3d
from roomscan.slam.intrinsics import pinhole

def test_pinhole_matches_fov():
    K = pinhole(54, 42, 55.0, 42.0)
    assert K.shape == (3, 3)
    k = K.numpy()
    assert k[0, 2] == 27.0 and k[1, 2] == 21.0          # cx, cy
    assert math.isclose(k[0, 0], 27.0 / math.tan(math.radians(55.0) / 2), rel_tol=1e-9)  # fx
    assert math.isclose(k[1, 1], 21.0 / math.tan(math.radians(42.0) / 2), rel_tol=1e-9)  # fy
    assert k[2, 2] == 1.0 and k[0, 1] == 0.0 and k[1, 0] == 0.0

def test_pinhole_is_cpu_float64():
    K = pinhole(54, 42)
    assert K.dtype == o3d.core.Dtype.Float64
    assert K.device.get_type() == o3d.core.Device.DeviceType.CPU
