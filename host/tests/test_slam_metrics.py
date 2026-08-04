import numpy as np
import pytest

from roomscan.slam import metrics
from roomscan.slam.metrics import trajectory_stats, timing_stats, write_tum, compare_kiss

def _pose(t):
    T = np.eye(4)
    T[:3, 3] = t
    return T

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
    import builtins
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


# ---------------------------------------------------------------------------
# footprint_area_m2 -- "area covered" for the View browser's tiles (§12)
# ---------------------------------------------------------------------------

def _floor_patch(side_m=2.0, n=50):
    """A dense square patch of points on the ground plane (Open3D CV: Y-down,
    so the up axis is 1 and the ground is X/Z)."""
    xs, zs = np.meshgrid(np.linspace(0, side_m, n), np.linspace(0, side_m, n))
    return np.stack([xs.ravel(), np.zeros(xs.size), zs.ravel()], axis=1)


def test_footprint_area_matches_the_swept_square():
    """A 2 x 2 m patch on a 0.1 m grid occupies 21 x 21 cells (both edges land
    on a cell boundary), i.e. 4.41 m2 -- the quantization is deliberate."""
    assert metrics.footprint_area_m2(_floor_patch()) == pytest.approx(4.41)


def test_footprint_ignores_height_so_a_wall_adds_nothing():
    """This is the whole reason it is NOT `mesh.get_surface_area()`: "area
    covered" is the floor swept, and a tall wall standing on already-covered
    floor contributes no new coverage. A surface-area metric would let a narrow
    corridor with high walls outscore a large open room."""
    floor = _floor_patch()
    wall = np.stack([floor[:, 0], np.linspace(0, 3, len(floor)), np.zeros(len(floor))], axis=1)
    assert metrics.footprint_area_m2(np.vstack([floor, wall])) == pytest.approx(
        metrics.footprint_area_m2(floor))


def test_footprint_is_bounded_by_a_stray_speckle():
    """A single TSDF speckle 5 m off the map adds ONE cell (0.01 m2), not a
    convex hull's worth -- the difference between a quantized count and an
    extent-based measure."""
    base = metrics.footprint_area_m2(_floor_patch())
    with_speckle = np.vstack([_floor_patch(), [[5.0, 0.0, 5.0]]])
    assert metrics.footprint_area_m2(with_speckle) == pytest.approx(base + 0.01)


def test_footprint_up_axis_is_selectable():
    """Y-down is the default because the Open3D CV world is; a caller in a
    Z-up frame passes up_axis=2 and gets the same answer for the same shape."""
    pts = _floor_patch()
    zup = pts[:, [0, 2, 1]]                      # move the flat axis to index 2
    assert metrics.footprint_area_m2(zup, up_axis=2) == pytest.approx(
        metrics.footprint_area_m2(pts, up_axis=1))


@pytest.mark.parametrize("bad", [np.zeros((0, 3)), np.full((10, 3), np.nan),
                                 np.zeros((5, 2)), np.zeros(3)])
def test_footprint_degenerate_inputs_are_zero_not_an_exception(bad):
    """It feeds a UI tile; a mesh with no vertices renders 0, never a traceback."""
    assert metrics.footprint_area_m2(bad) == 0.0


@pytest.mark.parametrize("kw", [{"up_axis": 3}, {"up_axis": -1}, {"cell_m": 0.0},
                                {"cell_m": -0.1}])
def test_footprint_rejects_nonsense_parameters(kw):
    with pytest.raises(ValueError):
        metrics.footprint_area_m2(_floor_patch(), **kw)
