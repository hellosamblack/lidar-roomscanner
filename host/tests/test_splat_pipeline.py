"""Splat pipeline control flow + levelling math.

The heavy stages (ffmpeg/COLMAP/gsplat) are exercised by an on-box integration
run, not here -- CI has no GPU and no COLMAP. These cover the pure-Python paths:
the skip/guard logic in `build_splat` (which returns before importing torch) and
the levelling rotation helper.
"""
import numpy as np

from roomscan.splat import SplatPreset, build_splat, level, sidecar


def test_rotation_between_maps_a_onto_b():
    R = level._rotation_between(np.array([0.0, 0, 1]), np.array([0.0, 1, 0]))
    assert np.allclose(R @ np.array([0.0, 0, 1]), [0, 1, 0], atol=1e-6)


def test_rotation_between_antiparallel():
    R = level._rotation_between(np.array([0.0, 1, 0]), np.array([0.0, -1, 0]))
    assert np.allclose(R @ np.array([0.0, 1, 0]), [0, -1, 0], atol=1e-6)


def test_rotation_between_identity():
    R = level._rotation_between(np.array([1.0, 0, 0]), np.array([1.0, 0, 0]))
    assert np.allclose(R, np.eye(3), atol=1e-9)


def test_build_splat_missing_video(tmp_path):
    rep = build_splat(tmp_path / "nope.mp4", "X", tmp_path)
    assert not rep["ok"] and "not found" in rep["reason"]


def test_build_splat_skips_current(tmp_path):
    # A current sidecar + force=False must short-circuit BEFORE any heavy import,
    # so this test runs on a GPU-less CI box.
    video = tmp_path / "room.mp4"
    video.write_bytes(b"x" * 100)
    preset = SplatPreset()
    paths = sidecar.sidecar_paths("sam-office", tmp_path)
    paths["dir"].mkdir(parents=True)
    paths["ply"].write_text("ply")
    man = sidecar.build_manifest("Sam Office", "sam-office", video, preset,
                                 stats={"gaussians": 3},
                                 transform=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    sidecar.write_manifest_atomic(paths["manifest"], man)

    rep = build_splat(video, "Sam Office", tmp_path, preset=preset, force=False)
    assert rep["ok"] and rep["built"] is False and "already exists" in rep["reason"]
