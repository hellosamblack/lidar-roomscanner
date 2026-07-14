"""Pure state-model predicates for the two-mode / two-camera panel redesign."""
import roomscan.panel as p


def test_follow_active_only_slam_first_person():
    assert p.follow_active(p.VIEW_SLAM, p.CAM_FIRST_PERSON) is True
    assert p.follow_active(p.VIEW_SLAM, p.CAM_ORBIT) is False
    assert p.follow_active(p.VIEW_REAL_TIME, p.CAM_FIRST_PERSON) is False


def test_gizmo_should_update_only_orbit():
    assert p.gizmo_should_update(p.CAM_ORBIT, True) is True
    assert p.gizmo_should_update(p.CAM_FIRST_PERSON, True) is False
    assert p.gizmo_should_update(p.CAM_ORBIT, False) is False


def test_real_time_first_person():
    assert p.real_time_first_person(p.VIEW_REAL_TIME, p.CAM_FIRST_PERSON) is True
    assert p.real_time_first_person(p.VIEW_SLAM, p.CAM_FIRST_PERSON) is False


def test_load_kind_by_suffix():
    assert p.load_kind("captures/panel_x.bin") == "capture"
    assert p.load_kind("results/showcase_y.PLY") == "mesh"
    assert p.load_kind("foo.txt") == "unknown"
