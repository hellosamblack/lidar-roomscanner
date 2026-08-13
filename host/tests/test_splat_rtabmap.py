"""RTAB-Map export ingest (issue #158) -- posed-capture contract + reader.

Two independent layers get tested:

1. `rtabmap_format1_pose_to_world_T_camera_optical` -- the pose-convention math -- against
   hand-derived identity/translation/90-degree-rotation cases, checked via the recovered
   camera CENTER and BASIS vectors (never just "the matrix looks plausible"), so an
   inverse/transpose/handedness mistake fails loudly (BUG-051/BUG-058 failure mode).
2. `load_rtabmap_export`/`summarize_rtabmap_export` against the tiny golden fixture in
   `tests/fixtures/rtabmap_export_min/` -- association, strict validation, and the CLI
   inspection path.

The golden numbers in part 1 come from hand-deriving RTAB-Map's own
`Graph::exportPoses()` format==1 algebra (`corelib/src/Graph.cpp:86-99`,
`introlab/rtabmap` @ `2e193ee1`) independently of `rtabmap.py`, not by calling the module
under test -- see the derivation comment above `rtabmap.py`'s
`rtabmap_format1_pose_to_world_T_camera_optical`.
"""
from __future__ import annotations

import ast
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from roomscan.splat import rtabmap
from roomscan.splat.posed import PosedFrameError
from roomscan.splat.rtabmap import (RtabmapExportError, load_rtabmap_export,
                                    rtabmap_format1_pose_to_world_T_camera_optical,
                                    summarize_rtabmap_export)

FIXTURE = Path(__file__).parent / "fixtures" / "rtabmap_export_min"
_SQRT2_2 = 0.70710678


def _copy_fixture(tmp_path: Path) -> Path:
    dst = tmp_path / "export"
    shutil.copytree(FIXTURE, dst)
    return dst


# --------------------------------------------------------------------------------------
# 1. Pose-convention regression tests -- identity / translation / 90-degree rotation.

def test_pose_conversion_identity():
    # Ground truth: world_T_camera_optical = Identity. Forward-derived by hand through
    # rtabmap's own format-1 algebra (t1^-1 * pose; t2^-1 * pose * t2, both zero-translation
    # pure rotations) -- see this file's module docstring.
    xyz = np.array([0.0, 0.0, 0.0])
    quat_xyzw = np.array([0.0, -_SQRT2_2, _SQRT2_2, 0.0])
    world_T_cam = rtabmap_format1_pose_to_world_T_camera_optical(xyz, quat_xyzw)
    assert np.allclose(world_T_cam, np.eye(4), atol=1e-6)


def test_pose_conversion_translation_recovers_camera_center():
    # Ground truth: world_T_camera_optical = pure translation (1, 2, 3), R = I.
    xyz = np.array([2.0, -1.0, 3.0])
    quat_xyzw = np.array([0.0, -_SQRT2_2, _SQRT2_2, 0.0])
    world_T_cam = rtabmap_format1_pose_to_world_T_camera_optical(xyz, quat_xyzw)
    assert np.allclose(world_T_cam[:3, :3], np.eye(3), atol=1e-6)
    assert np.allclose(world_T_cam[:3, 3], [1.0, 2.0, 3.0], atol=1e-6)

    # And the canonical viewmat (camera_from_world, our PosedFrame contract) inverts back
    # to the same camera center via -R^T t -- this is the check that would fail if the
    # loader forgot the final SE3 inversion (a missing-invert bug leaves the translation
    # column at -(1,2,3) instead of +(1,2,3), see rtabmap._invert_rigid).
    viewmat = rtabmap._invert_rigid(world_T_cam)
    center = -viewmat[:3, :3].T @ viewmat[:3, 3]
    assert np.allclose(center, [1.0, 2.0, 3.0], atol=1e-6)


def test_pose_conversion_90_degree_rotation_basis_vectors():
    # Ground truth: world_T_camera_optical rotation = Rz(90) = [[0,-1,0],[1,0,0],[0,0,1]],
    # i.e. camera "right" (optical +x) = world +y, "down" (optical +y) = world -x,
    # "forward" (optical +z) = world +z. Translation zero (isolates rotation).
    xyz = np.array([0.0, 0.0, 0.0])
    quat_xyzw = np.array([0.5, -0.5, 0.5, -0.5])
    world_T_cam = rtabmap_format1_pose_to_world_T_camera_optical(xyz, quat_xyzw)

    right, down, forward = world_T_cam[:3, 0], world_T_cam[:3, 1], world_T_cam[:3, 2]
    assert np.allclose(right, [0.0, 1.0, 0.0], atol=1e-6)
    assert np.allclose(down, [-1.0, 0.0, 0.0], atol=1e-6)
    assert np.allclose(forward, [0.0, 0.0, 1.0], atol=1e-6)
    assert np.isclose(np.linalg.det(world_T_cam[:3, :3]), 1.0, atol=1e-6)

    # The canonical viewmat's basis-vector accessor must recover the SAME world-frame right/
    # down/forward -- catches a transpose mistake even when the rotation is zero-translation
    # (where a center-only check reads (0,0,0) either way and cannot see the bug).
    from roomscan.splat.posed import PosedFrame
    frame = PosedFrame(frame_id="f", image_path=Path("x.jpg"), width=4, height=4,
                       k=np.array([[10.0, 0, 2], [0, 10.0, 2], [0, 0, 1]]),
                       pose_camera_from_world=rtabmap._invert_rigid(world_T_cam))
    r2, d2, f2 = frame.camera_basis_world()
    assert np.allclose(r2, right, atol=1e-6)
    assert np.allclose(d2, down, atol=1e-6)
    assert np.allclose(f2, forward, atol=1e-6)


def test_format1_round_trip_matches_pinned_source_algebra():
    """Forward-simulate rtabmap's own Graph.cpp format==1 steps for a random rigid pose,
    then confirm the reader recovers exactly that pose -- proves the conversion is the exact
    algebraic inverse of the pinned source, not merely self-consistent by construction."""
    rng = np.random.default_rng(7)
    A = rng.normal(size=(3, 3))
    Q, _ = np.linalg.qr(A)
    if np.linalg.det(Q) < 0:
        Q[:, 0] *= -1
    t_gt = rng.normal(size=3)

    # Graph.cpp:86-99, transcribed directly as SE3 composition (t1, t2 pure rotations, zero
    # translation, so `X.inverse() * pose` scales BOTH the rotation and translation of
    # `pose` by X.T on the left -- this is `pose = t1.inverse() * pose_in` then
    # `pose = t2.inverse() * pose * t2`, expanded by hand rather than via an SE3 helper
    # class, to keep this test independent of rtabmap.py's own `_invert_rigid`).
    t1 = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
    t2 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    R_out = t2.T @ (t1.T @ Q) @ t2
    t_out = t2.T @ (t1.T @ t_gt)

    recovered = rtabmap_format1_pose_to_world_T_camera_optical(t_out, _matrix_to_quat_xyzw(R_out))
    assert np.allclose(recovered[:3, :3], Q, atol=1e-6)
    assert np.allclose(recovered[:3, 3], t_gt, atol=1e-6)


def _matrix_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Standalone (not reused from rtabmap.py) matrix->quaternion, for the round-trip test."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


# --------------------------------------------------------------------------------------
# 2. Reading the golden export directory.

def test_load_golden_fixture_frame_ids_count_order():
    capture = load_rtabmap_export(FIXTURE)
    assert [f.frame_id for f in capture.frames] == ["1699999900.100000", "1699999900.600000"]
    assert len(capture.frames) == 2
    assert capture.source == "rtabmap_export"


def test_load_golden_fixture_preserves_distinct_per_frame_calibration():
    capture = load_rtabmap_export(FIXTURE)
    k0 = capture.frames[0].k
    k1 = capture.frames[1].k
    assert k0[0, 0] == pytest.approx(50.0) and k0[1, 1] == pytest.approx(50.0)
    assert k1[0, 0] == pytest.approx(52.0) and k1[1, 1] == pytest.approx(53.0)
    assert not np.allclose(k0, k1)   # never coalesced despite both being "the" camera


def test_load_golden_fixture_associates_depth_and_confidence_by_stamp_not_globbing():
    capture = load_rtabmap_export(FIXTURE)
    for f in capture.frames:
        assert f.depth_path is not None and f.depth_path.name == f"{f.frame_id}.png"
        assert f.confidence_path is not None and f.confidence_path.name == f"{f.frame_id}.png"
        assert f.image_path.name == f"{f.frame_id}.jpg"


def test_load_rtabmap_export_end_to_end_pose_direction(tmp_path):
    """Load a real (tmp_path-copied) export through `load_rtabmap_export` -- not the pure
    conversion function directly -- with the hand-verified translation-case pose from part 1
    substituted for frame 1's row, and confirm the loader's OWN final inversion is applied.
    This is the end-to-end companion to the pure-function tests above: it would fail if
    `_associate_frames` stored `world_T_camera` directly instead of its inverse (a bug the
    pure-function tests alone cannot see, since they call the conversion helper directly)."""
    export_dir = _copy_fixture(tmp_path)
    poses = export_dir / "session_camera_poses.txt"
    lines = poses.read_text().splitlines()
    lines[1] = f"1699999900.100000 2.000000 -1.000000 3.000000 0.000000 {-_SQRT2_2:.8f} {_SQRT2_2:.8f} 0.000000"
    poses.write_text("\n".join(lines) + "\n")

    capture = load_rtabmap_export(export_dir)
    frame = capture.frames[0]
    assert frame.frame_id == "1699999900.100000"
    assert np.allclose(frame.camera_center_world(), [1.0, 2.0, 3.0], atol=1e-5)


def test_load_golden_fixture_geometry_present():
    capture = load_rtabmap_export(FIXTURE)
    assert len(capture.geometry_paths) == 1
    assert capture.geometry_paths[0].name == "session_cloud.ply"
    assert capture.geometry_frame == capture.world_frame


def test_load_geometry_absent(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    (export_dir / "session_cloud.ply").unlink()
    capture = load_rtabmap_export(export_dir)
    assert capture.geometry_paths == ()
    assert capture.geometry_frame is None


def test_load_golden_fixture_timestamps_preserved():
    capture = load_rtabmap_export(FIXTURE)
    assert [f.timestamp for f in capture.frames] == [1699999900.1, 1699999900.6]
    assert all(f.timestamp_domain == "rtabmap_export_stamp_s" for f in capture.frames)


def test_import_reader_does_not_pull_heavy_deps():
    """No torch/gsplat/pycolmap/CUDA import anywhere `rtabmap.py`/`posed.py` touch --
    checked via a fresh subprocess import (so already-imported heavy modules elsewhere in
    the test session cannot mask a bad import) AND via an AST scan (so a *lazy* heavy
    import buried in a function body -- which the subprocess run would never execute --
    is still caught)."""
    proc = subprocess.run(
        [sys.executable, "-c",
         "import sys; import roomscan.splat.rtabmap; "
         "bad = [m for m in ('torch', 'gsplat', 'pycolmap') if m in sys.modules]; "
         "assert not bad, bad"],
        cwd=Path(__file__).resolve().parents[1] / "src", capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr

    src_dir = Path(__file__).resolve().parents[1] / "src" / "roomscan" / "splat"
    for name in ("rtabmap.py", "posed.py"):
        tree = ast.parse((src_dir / name).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [a.name.split(".")[0] for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module:
                names = [node.module.split(".")[0]]
            else:
                continue
            assert not ({"torch", "gsplat", "pycolmap"} & set(names)), (name, names)


# --------------------------------------------------------------------------------------
# 3. Strict-validation failure paths.

def test_missing_depth_reports_offending_frame(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    (export_dir / "session_depth" / "1699999900.600000.png").unlink()
    with pytest.raises(RtabmapExportError, match=r"1699999900\.600000.*missing.*depth"):
        load_rtabmap_export(export_dir)


def test_missing_confidence_not_silently_dropped(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    (export_dir / "session_confidence" / "1699999900.100000.png").unlink()
    with pytest.raises(RtabmapExportError) as exc:
        load_rtabmap_export(export_dir)
    assert "1699999900.100000" in str(exc.value) and "confidence" in str(exc.value)
    # The frame that WAS complete must not appear as if the whole export were fine --
    # summarize must also report failure, not a partial "1 frame" success.
    report = summarize_rtabmap_export(export_dir)
    assert not report["ok"]
    assert "1699999900.100000" in report["reason"]


def test_non_strict_depth_confidence_can_be_relaxed_explicitly(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    (export_dir / "session_depth" / "1699999900.600000.png").unlink()
    (export_dir / "session_confidence" / "1699999900.100000.png").unlink()
    capture = load_rtabmap_export(export_dir, require_depth=False, require_confidence=False)
    assert len(capture.frames) == 2
    by_id = {f.frame_id: f for f in capture.frames}
    assert by_id["1699999900.600000"].depth_path is None
    assert by_id["1699999900.100000"].confidence_path is None


def test_missing_calibration(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    (export_dir / "session_calib" / "1699999900.100000.yaml").unlink()
    with pytest.raises(RtabmapExportError, match=r"1699999900\.100000.*calibration"):
        load_rtabmap_export(export_dir)


def test_missing_rgb(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    (export_dir / "session_rgb" / "1699999900.600000.jpg").unlink()
    with pytest.raises(RtabmapExportError, match=r"1699999900\.600000.*rgb"):
        load_rtabmap_export(export_dir)


def test_duplicate_stamp_in_poses_file(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    poses = export_dir / "session_camera_poses.txt"
    lines = poses.read_text().splitlines()
    poses.write_text("\n".join(lines + [lines[-1]]) + "\n")
    with pytest.raises(RtabmapExportError, match="duplicate stamp"):
        load_rtabmap_export(export_dir)


def test_malformed_pose_row_wrong_column_count(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    poses = export_dir / "session_camera_poses.txt"
    poses.write_text(poses.read_text() + "1699999901.000000 0 0 0\n")
    with pytest.raises(RtabmapExportError, match=r"column\(s\), expected 8"):
        load_rtabmap_export(export_dir)


def test_malformed_pose_row_non_finite(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    poses = export_dir / "session_camera_poses.txt"
    poses.write_text(poses.read_text() + "1699999901.000000 nan 0 0 0 0 0 1\n")
    with pytest.raises(RtabmapExportError, match="non-finite"):
        load_rtabmap_export(export_dir)


def test_invalid_focal_length(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    calib = export_dir / "session_calib" / "1699999900.100000.yaml"
    calib.write_text(calib.read_text().replace("5.0000000000000000e+01, 0., 3.1500000000000000e+01",
                                               "0., 0., 3.1500000000000000e+01"))
    with pytest.raises(RtabmapExportError, match="focal length"):
        load_rtabmap_export(export_dir)


def test_invalid_image_dimensions_calibration_mismatch(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    calib = export_dir / "session_calib" / "1699999900.100000.yaml"
    calib.write_text(calib.read_text().replace("image_width: 64", "image_width: 999"))
    with pytest.raises(RtabmapExportError, match="calibration size"):
        load_rtabmap_export(export_dir)


def test_orphan_rgb_file_with_no_matching_pose(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    orphan = export_dir / "session_rgb" / "1699999999.999999.jpg"
    orphan.write_bytes((export_dir / "session_rgb" / "1699999900.100000.jpg").read_bytes())
    with pytest.raises(RtabmapExportError, match="orphan"):
        load_rtabmap_export(export_dir)


def test_ambiguous_multiple_exports_in_one_directory(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    other = export_dir / "session_camera_poses.txt"
    shutil.copy(other, export_dir / "second_camera_poses.txt")
    with pytest.raises(RtabmapExportError, match="ambiguous"):
        load_rtabmap_export(export_dir)


def test_multi_camera_export_unsupported(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    poses = export_dir / "session_camera_poses.txt"
    shutil.copy(poses, export_dir / "session_camera_poses_0.txt")
    with pytest.raises(RtabmapExportError, match="multi-camera"):
        load_rtabmap_export(export_dir)


def test_raw_db_path_rejected_with_actionable_message(tmp_path):
    db = tmp_path / "room.db"
    db.write_bytes(b"sqlite")
    with pytest.raises(RtabmapExportError, match="rtabmap-export"):
        load_rtabmap_export(db)


def test_nonexistent_directory():
    with pytest.raises(RtabmapExportError):
        load_rtabmap_export("/does/not/exist/anywhere")


def test_other_poses_format_rejected(tmp_path):
    export_dir = _copy_fixture(tmp_path)
    poses = export_dir / "session_camera_poses.txt"
    # --poses_format 11 appends a trailing node-id column: 9 fields, not 8.
    poses.write_text("#timestamp x y z qx qy qz qw id\n"
                     "1699999900.100000 0 0 0 0 0 0 1 42\n")
    with pytest.raises(RtabmapExportError, match=r"column\(s\), expected 8"):
        load_rtabmap_export(export_dir)


# --------------------------------------------------------------------------------------
# 4. `summarize_rtabmap_export` / CLI.

def test_summarize_golden_fixture_reports_expected_counts():
    report = summarize_rtabmap_export(FIXTURE)
    assert report["ok"] is True
    assert report["frame_count"] == 2
    assert report["frames_with_depth"] == 2
    assert report["frames_with_confidence"] == 2
    assert report["distinct_calibrations"] == 2
    assert report["poses_valid"] == 2
    assert report["timestamps_present"] is True
    assert report["timestamp_domains"] == ["rtabmap_export_stamp_s"]
    assert report["geometry_paths"] and report["geometry_paths"][0].endswith("session_cloud.ply")


def test_cli_inspect_rtabmap_success(capsys):
    from roomscan.splat.cli import main

    rc = main(["inspect-rtabmap", str(FIXTURE)])
    assert rc == 0
    out = capsys.readouterr().out
    assert '"frame_count": 2' in out
    assert '"ok": true' in out


def test_cli_inspect_rtabmap_failure_nonzero_exit(tmp_path, capsys):
    from roomscan.splat.cli import main

    export_dir = _copy_fixture(tmp_path)
    (export_dir / "session_rgb" / "1699999900.100000.jpg").unlink()
    rc = main(["inspect-rtabmap", str(export_dir)])
    assert rc == 2
    out = capsys.readouterr().out
    assert '"ok": false' in out
    assert "1699999900.100000" in out and "rgb" in out


# --------------------------------------------------------------------------------------
# 5. Regressions on `PosedFrame`/`PosedCapture` validation itself.

def test_posed_frame_rejects_non_orthonormal_rotation():
    from roomscan.splat.posed import PosedFrame

    bad_pose = np.eye(4)
    bad_pose[:3, :3] *= 2.0   # scaled, not a rotation
    with pytest.raises(PosedFrameError, match="orthonormal|proper rotation"):
        PosedFrame(frame_id="f", image_path=Path("x.jpg"), width=4, height=4,
                  k=np.eye(3) * 10, pose_camera_from_world=bad_pose)


def test_posed_frame_rejects_non_positive_focal_length():
    from roomscan.splat.posed import PosedFrame

    with pytest.raises(PosedFrameError, match="focal length"):
        PosedFrame(frame_id="f", image_path=Path("x.jpg"), width=4, height=4,
                  k=np.array([[0.0, 0, 2], [0, 10.0, 2], [0, 0, 1]]),
                  pose_camera_from_world=np.eye(4))


def test_posed_capture_rejects_duplicate_frame_ids():
    from roomscan.splat.posed import PosedCapture, PosedFrame

    frame = PosedFrame(frame_id="dup", image_path=Path("x.jpg"), width=4, height=4,
                       k=np.eye(3) * 10, pose_camera_from_world=np.eye(4))
    with pytest.raises(PosedFrameError, match="duplicate"):
        PosedCapture(source="test", frames=(frame, frame), world_frame="test")
