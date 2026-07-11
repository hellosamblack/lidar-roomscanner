import numpy as np
from roomscan.slam.metrics import trajectory_stats, timing_stats, write_tum, compare_kiss

def _pose(t):
    T = np.eye(4); T[:3, 3] = t; return T

def test_trajectory_stats():
    poses = [_pose([0, 0, 0]), _pose([0, 0, 1]), _pose([0, 0, 1.5])]
    s = trajectory_stats(poses)
    assert s["n"] == 3
    assert np.isclose(s["path_length_m"], 1.5)
    assert np.isclose(s["start_end_gap_m"], 1.5)
    assert np.isclose(s["max_step_m"], 1.0)

def test_timing_stats():
    s = timing_stats([10.0, 20.0, 40.0, 50.0])
    assert s["n"] == 4
    assert s["median_ms"] == 30.0
    assert s["max_ms"] == 50.0
    assert np.isclose(s["over_budget_frac"], 0.5)   # 2 of 4 > 35 ms

def test_write_tum_roundtrip(tmp_path):
    p = tmp_path / "traj.tum"
    write_tum(p, [0.0, 0.1], [_pose([0, 0, 0]), _pose([1, 2, 3])])
    lines = p.read_text().strip().splitlines()
    assert len(lines) == 2
    parts = lines[1].split()
    assert len(parts) == 8
    assert np.allclose([float(x) for x in parts[1:4]], [1, 2, 3])

def test_compare_kiss_optional(monkeypatch):
    # if kiss-icp missing, returns None gracefully (does not raise)
    import builtins, importlib
    real = builtins.__import__
    def fake(name, *a, **k):
        if name.startswith("kiss_icp"):
            raise ImportError("no kiss")
        return real(name, *a, **k)
    monkeypatch.setattr(builtins, "__import__", fake)
    assert compare_kiss([], None, 55.0, 42.0) is None
