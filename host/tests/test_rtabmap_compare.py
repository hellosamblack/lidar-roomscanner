"""Independent RTAB-Map/roomscanner trajectory comparison (issue #188)."""
from __future__ import annotations

import json
import sqlite3
import zlib
from pathlib import Path

import numpy as np
import pytest

from roomscan.rtabmap_db import RtabmapDatabaseError, read_optimized_trajectory
from roomscan.trajectory_compare import (
    align_clock_by_angular_speed,
    compare_matched_trajectories,
    compare_rtabmap_to_roomscan,
    read_tum_trajectory,
)

FIXTURE = Path(__file__).parent / "fixtures" / "rtabmap_ashoffice_matched.npz"


def _yaw_rotations(yaw: np.ndarray) -> np.ndarray:
    result = np.repeat(np.eye(3)[None, :, :], len(yaw), axis=0)
    result[:, 0, 0] = np.cos(yaw)
    result[:, 0, 1] = -np.sin(yaw)
    result[:, 1, 0] = np.sin(yaw)
    result[:, 1, 1] = np.cos(yaw)
    return result


def _write_db(path: Path, node_ids: np.ndarray, timestamps: np.ndarray,
              poses: np.ndarray) -> None:
    stored = np.asarray(poses[:, :3, :], dtype="<f4").tobytes()
    ids = np.asarray(node_ids, dtype="<i4").tobytes()
    with sqlite3.connect(path) as db:
        db.execute("CREATE TABLE Admin(opt_ids BLOB, opt_poses BLOB)")
        db.execute("CREATE TABLE Node(id INTEGER PRIMARY KEY, stamp FLOAT)")
        db.execute("INSERT INTO Admin VALUES (?, ?)", (zlib.compress(ids), zlib.compress(stored)))
        db.executemany("INSERT INTO Node VALUES (?, ?)",
                       [(int(i), float(t)) for i, t in zip(node_ids, timestamps)])


def test_read_optimized_trajectory_decodes_raw_zlib_and_joins_stamps_by_id(tmp_path):
    path = tmp_path / "phone.db"
    node_ids = np.array([7, 2, 11])
    timestamps = np.array([100.0, 101.0, 102.5])
    poses = np.repeat(np.eye(4)[None, :, :], 3, axis=0)
    poses[:, 0, 3] = [0.0, 1.0, 3.0]
    _write_db(path, node_ids, timestamps, poses)

    result = read_optimized_trajectory(path)

    assert result.node_ids.tolist() == [7, 2, 11]
    assert result.timestamps_s.tolist() == [100.0, 101.0, 102.5]
    assert np.allclose(result.poses, poses, atol=1e-6)


def test_read_optimized_trajectory_rejects_qcompress_style_or_corrupt_blob(tmp_path):
    path = tmp_path / "bad.db"
    with sqlite3.connect(path) as db:
        db.execute("CREATE TABLE Admin(opt_ids BLOB, opt_poses BLOB)")
        db.execute("CREATE TABLE Node(id INTEGER PRIMARY KEY, stamp FLOAT)")
        db.execute("INSERT INTO Admin VALUES (?, ?)", (b"\0\0\0\4not-zlib", zlib.compress(b"x")))

    with pytest.raises(RtabmapDatabaseError, match="raw-zlib"):
        read_optimized_trajectory(path)


def test_read_tum_is_strict_about_columns_and_time(tmp_path):
    bad_columns = tmp_path / "columns.tum"
    bad_columns.write_text("0 0 0 0 0 0 0\n")
    with pytest.raises(ValueError, match="expected 8"):
        read_tum_trajectory(bad_columns)

    backwards = tmp_path / "backwards.tum"
    backwards.write_text(
        "0 0 0 0 0 0 0 1\n"
        "2 0 0 0 0 0 0 1\n"
        "1 0 0 0 0 0 0 1\n")
    with pytest.raises(ValueError, match="strictly increasing"):
        read_tum_trajectory(backwards)


def test_angular_speed_alignment_recovers_offset_without_mount_extrinsic():
    room_t = np.arange(0.0, 30.0, 0.01)
    # A non-periodic, smoothly changing turn rate makes the correlation peak unique.
    room_yaw = 0.03 * room_t ** 1.35 + 0.12 * np.sin(room_t * 0.73)
    room_r = _yaw_rotations(room_yaw)
    rtab_t = np.arange(0.0, 14.0, 1.0)
    true_offset = 7.24
    rtab_yaw = np.interp(rtab_t + true_offset, room_t, room_yaw)
    # Fixed unknown mount rotation: relative-rotation angle remains unchanged.
    mount = _yaw_rotations(np.array([1.1]))[0]
    rtab_r = np.einsum("nij,jk->nik", _yaw_rotations(rtab_yaw), mount)

    result = align_clock_by_angular_speed(
        rtab_t, rtab_r, room_t, room_r, search_step_s=0.02)

    assert result["roomscan_time_at_rtab_start_s"] == pytest.approx(true_offset, abs=0.08)
    assert result["correlation"] > 0.95


def test_ashoffice_golden_keeps_physical_path_ratio_separate_from_fit_scale():
    """Small real-data fixture pins the published #188 findings without the 49 MB DB."""
    with np.load(FIXTURE) as data:
        report = compare_matched_trajectories(
            data["rtab_timestamps_s"], data["rtab_poses"],
            data["room_positions_m"], data["room_rotations"])

    assert report["distance"]["path_length_ratio_roomscanner_over_rtabmap"] == pytest.approx(
        0.956, abs=0.002)
    assert report["rotation"]["ratio_roomscanner_over_rtabmap"] == pytest.approx(1.012, abs=0.002)
    assert report["error"]["final_window_mean_abs_xyz_m"] == pytest.approx(
        [0.54, 0.57, 2.57], abs=0.02)
    assert report["alignment"]["full_overlap_similarity_scale_diagnostic"] == pytest.approx(
        0.497, abs=0.002)
    assert report["distance"]["path_length_ratio_is_physical_scale"] is True
    assert report["alignment"]["similarity_scale_is_physical_scale"] is False


def test_compare_reports_errors_as_data(tmp_path):
    report = compare_rtabmap_to_roomscan(tmp_path / "missing.db", tmp_path / "missing.tum")
    assert report["ok"] is False
    assert "not found" in report["error"]


def test_cli_emits_the_same_structured_report(monkeypatch, capsys, tmp_path):
    from tools import rtabmap_compare

    expected = {"ok": True, "distance": {"path_length_ratio_roomscanner_over_rtabmap": 0.956}}
    monkeypatch.setattr(rtabmap_compare, "compare_rtabmap_to_roomscan",
                        lambda *_args, **_kwargs: expected)
    out = tmp_path / "report.json"
    rc = rtabmap_compare.main(["phone.db", "scan.tum", "--json", str(out)])

    assert rc == 0
    assert json.loads(capsys.readouterr().out) == expected
    assert json.loads(out.read_text()) == expected


def test_mcp_wrapper_resolves_repo_relative_paths(monkeypatch):
    from roomscan.mcp_server import tools_data
    import roomscan.trajectory_compare as compare_module

    seen = {}

    def fake_compare(db, tum, **kwargs):
        seen.update(db=db, tum=tum, kwargs=kwargs)
        return {"ok": True}

    monkeypatch.setattr(compare_module, "compare_rtabmap_to_roomscan", fake_compare)
    result = tools_data.rtabmap_trajectory_compare("phone.db", "scan.tum")

    assert result == {"ok": True}
    assert seen["db"] == tools_data.REPO / "phone.db"
    assert seen["tum"] == tools_data.REPO / "scan.tum"
