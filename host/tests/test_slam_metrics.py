import numpy as np
import pytest

from roomscan.slam import metrics
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


# --- tracking_stats: separate "a few dropped frames" from "the run died"


def test_tracking_stats_empty():
    s = metrics.tracking_stats([])
    assert s == {"n": 0, "lost": 0, "lost_frac": 0.0, "trailing_lost": 0,
                 "longest_lost_run": 0, "died": False}


def test_tracking_stats_clean_run():
    s = metrics.tracking_stats([False] * 100)
    assert s["lost"] == 0 and s["trailing_lost"] == 0
    assert s["longest_lost_run"] == 0
    assert s["died"] is False


def test_tracking_stats_scattered_losses_are_not_death():
    """Isolated dropouts recover; the run is still a real measurement."""
    flags = [False] * 100
    for i in (10, 40, 41, 70):
        flags[i] = True
    s = metrics.tracking_stats(flags)
    assert s["lost"] == 4
    assert s["longest_lost_run"] == 2
    assert s["trailing_lost"] == 0
    assert s["died"] is False


def test_tracking_stats_flags_a_run_that_never_recovered():
    """The coffeeRoomCircuitMnt.bin shape: healthy, then an unbroken lost
    streak to the end. That tail is a frozen dead-reckoned pose, so the run
    must be reported as died even though 78% of frames tracked fine."""
    flags = [False] * 1466 + [True] * 423
    s = metrics.tracking_stats(flags)
    assert s["lost"] == 423
    assert s["trailing_lost"] == 423
    assert s["longest_lost_run"] == 423
    assert s["lost_frac"] == pytest.approx(423 / 1889)
    assert s["died"] is True


def test_tracking_stats_short_trailing_tail_is_not_death():
    """A handful of trailing lost frames is a normal end-of-scan tail."""
    s = metrics.tracking_stats([False] * 100 + [True] * 5)
    assert s["trailing_lost"] == 5
    assert s["died"] is False
