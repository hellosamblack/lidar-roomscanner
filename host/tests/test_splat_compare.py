"""splat-vs-scan compare: alignment + shape-error metrics on synthetic rooms.

No 600 MB fixture -- everything is a small analytic box room. The "scan" carries a
BUG-084-style ceiling fork (a second, dropped ceiling) that the ground-truth
reference does not, and the reference is written upside-down (as a real Scaniverse
export is) so the flip-seeded alignment is exercised.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d
from plyfile import PlyData, PlyElement

from roomscan.splat import compare


# --------------------------------------------------------------------------- #
# Synthetic geometry helpers (world is Y-down: height = -y, ceiling at -y large)
# --------------------------------------------------------------------------- #
def _slab(xr, yr, zr, n=1800, seed=0):
    """Uniform points in a box; a degenerate axis (a==b) makes a plane. Walls are
    vertical planes (span the full y range) so they add XZ structure without
    creating a spurious horizontal height peak."""
    rng = np.random.default_rng(seed)
    return np.stack([rng.uniform(*xr, n), rng.uniform(*yr, n), rng.uniform(*zr, n)], axis=1)


def _room(fork: bool) -> np.ndarray:
    """A 4x3 m room, world Y-down (floor y=0, ceiling y=-2.5 i.e. height 2.5).
    With ``fork``, an extra dropped ceiling at height 1.5 over half the room --
    the BUG-084 second surface the ground truth does not have."""
    parts = [
        _slab((0, 4), (0, 0), (0, 3), 2000, 1),          # floor      (height 0)
        _slab((0, 4), (-2.5, -2.5), (0, 3), 2000, 2),    # ceiling    (height 2.5)
        _slab((0, 0), (-2.5, 0), (0, 3), 1500, 3),       # wall x=0   (vertical)
        _slab((4, 4), (-2.5, 0), (0, 3), 1500, 4),       # wall x=4
        _slab((0, 4), (-2.5, 0), (0, 0), 1500, 5),       # wall z=0
        _slab((0, 4), (-2.5, 0), (3, 3), 1500, 6),       # wall z=3
    ]
    if fork:
        parts.append(_slab((0, 2), (-1.5, -1.5), (0, 3), 1500, 7))  # dropped ceiling
    return np.vstack(parts)


def _cloud(pts, voxel=0.0):
    return compare._finalize_cloud(pts, None, voxel, denoise=False)


def _flip_upside_down(pts: np.ndarray) -> np.ndarray:
    """180 deg about X, as an imported Scaniverse splat sits relative to our frame."""
    R = np.diag([1.0, -1.0, -1.0])
    return pts @ R.T


# --------------------------------------------------------------------------- #
# Loading / floater rejection
# --------------------------------------------------------------------------- #
def _write_inria_ply(path, pts, opacity):
    logit = np.log(opacity / (1 - opacity))               # sigmoid(stored) = opacity
    f_dc = np.full((len(pts), 3), 0.5)                    # mid-grey
    arr = np.zeros(len(pts), dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"),
                                    ("opacity", "f4"), ("f_dc_0", "f4"),
                                    ("f_dc_1", "f4"), ("f_dc_2", "f4")])
    arr["x"], arr["y"], arr["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    arr["opacity"] = logit
    arr["f_dc_0"], arr["f_dc_1"], arr["f_dc_2"] = f_dc.T
    PlyData([PlyElement.describe(arr, "vertex")], text=False).write(str(path))


def test_load_reference_drops_floaters(tmp_path):
    solid = _room(fork=False)
    floaters = np.random.default_rng(1).uniform(-5, 5, (2000, 3))   # scattered snowglobe
    pts = np.vstack([solid, floaters])
    opacity = np.concatenate([np.full(len(solid), 0.9), np.full(len(floaters), 0.02)])
    ply = tmp_path / "ref.ply"
    _write_inria_ply(ply, pts, opacity)

    kept = compare.load_reference_cloud(ply, opacity_min=0.5, voxel=0.0)
    n = len(kept.points)
    assert len(solid) * 0.8 < n < len(solid) * 1.2       # ~solids only, floaters gone


# --------------------------------------------------------------------------- #
# Alignment
# --------------------------------------------------------------------------- #
def test_alignment_recovers_upside_down(tmp_path):
    scan = _cloud(_room(fork=False), voxel=0.03)
    ref = _cloud(_flip_upside_down(_room(fork=False)), voxel=0.03)

    align = compare.align_reference_to_scan(scan, ref, voxel=0.03, use_global=False)
    assert align["fitness"] > 0.9

    ref_aligned = o3d.geometry.PointCloud(ref).transform(align["transform"])
    d = np.asarray(scan.compute_point_cloud_distance(ref_aligned))
    assert np.median(d) < 0.05                            # flipped truth now overlays scan


# --------------------------------------------------------------------------- #
# Shape-error metrics
# --------------------------------------------------------------------------- #
def test_vertical_fork_detection():
    scan = _cloud(_room(fork=True))
    ref = _cloud(_room(fork=False))
    rep = compare._vertical_report(scan, ref)
    assert rep["fork_suspected"] is True
    assert len(rep["scan_height_modes"]) > len(rep["ref_height_modes"])


def test_healthy_room_is_not_a_fork():
    scan = _cloud(_room(fork=False))
    ref = _cloud(_room(fork=False))
    rep = compare._vertical_report(scan, ref)
    assert rep["fork_suspected"] is False                 # floor+ceiling alone != fork


def test_distance_zero_for_identical_nonzero_for_shifted():
    floor = _cloud(_slab((0, 4), (0, 0), (0, 3), 4000), voxel=0.03)
    _, same = compare._dist_stats(floor, floor)
    assert same["p95"] < 1e-6
    shifted = o3d.geometry.PointCloud(floor).translate((0, 0.3, 0))   # off the plane
    _, moved = compare._dist_stats(floor, shifted)
    assert moved["median"] > 0.25


def test_footprint_area_matches_box():
    floor = _slab((0, 4), (0, 0), (0, 3), 20000)          # dense 4x3 plane
    assert 10.0 < compare._footprint_area_m2(_cloud(floor)) < 12.5


# --------------------------------------------------------------------------- #
# End-to-end orchestration + artifacts
# --------------------------------------------------------------------------- #
def _write_scan_mesh(path, pts):
    """Write a point set as a triangle-mesh .ply (vertices only) -- the shape
    load_scan_cloud reads from a real Detailed-SLAM output."""
    m = o3d.geometry.TriangleMesh()
    m.vertices = o3d.utility.Vector3dVector(pts)
    o3d.io.write_triangle_mesh(str(path), m, write_ascii=False)


def test_compare_end_to_end_writes_artifacts(tmp_path):
    results = tmp_path / "results"
    results.mkdir()
    _write_scan_mesh(results / "myroom.ply", _room(fork=True))

    ref_dir = results / "splats" / "gt-room"
    ref_dir.mkdir(parents=True)
    ref_pts = _flip_upside_down(_room(fork=False))
    _write_inria_ply(ref_dir / "GtRoom.ply", ref_pts, np.full(len(ref_pts), 0.9))

    rep = compare.compare_scan_to_reference(
        "myroom", "gt-room", results_dir=results, voxel=0.03, write_artifacts=True)

    assert rep["ok"] is True, rep
    assert rep["vertical"]["fork_suspected"] is True
    assert rep["distance_m"]["scan_to_reference"]["p95"] > 0
    out = results / "compare" / "myroom__vs__gt-room"
    for name in ("overlay.ply", "error_heatmap.ply", "floorplan.png",
                 "elevation.png", "report.json"):
        assert (out / name).is_file(), f"missing artifact {name}"


def test_compare_missing_scan_is_reported(tmp_path):
    results = tmp_path / "results"
    results.mkdir()
    rep = compare.compare_scan_to_reference("nope", results_dir=results, write_artifacts=False)
    assert rep["ok"] is False and "no SLAM reconstruction" in rep["error"]


# --------------------------------------------------------------------------- #
# Reference auto-selection (not hardcoded to one splat)
# --------------------------------------------------------------------------- #
def test_tokens_split_camelcase_and_drop_stopwords():
    assert compare._tokens("officeFullScanAug6") == {"office"}   # Full/Scan/Aug are stopwords
    assert compare._tokens("Sam Office") == {"sam", "office"}
    assert compare._tokens("scaniverse-splat") == set()          # both stopwords -> no scene token


def _imported_splat(results, slug, name=None, plyname="ref.ply"):
    """A dropped-in imported reference: a dir with a .ply and (optionally) an
    import manifest giving it a display name. No manifest -> named by slug."""
    from roomscan.splat.sidecar import splats_root, write_import_manifest
    d = splats_root(results) / slug
    d.mkdir(parents=True)
    _write_inria_ply(d / plyname, _flip_upside_down(_room(fork=False)),
                     np.full(len(_room(fork=False)), 0.9))
    if name:
        write_import_manifest(slug, results, name=name, gaussians=1)
    return d


def test_choose_reference_prefers_name_match(tmp_path):
    results = tmp_path / "results"
    results.mkdir()
    _imported_splat(results, "kitchen-truth", name="Kitchen")
    _imported_splat(results, "office-truth", name="Sam Office")
    sel = compare.choose_reference("officeFullScanAug6", results)
    assert sel["chosen"] == "office-truth"
    assert "office" in sel["reason"]


def test_choose_reference_ignores_our_video_builds(tmp_path):
    """A non-imported (our own) build must never be picked as ground truth, even
    when its name matches better than the only imported reference."""
    from roomscan.splat import SplatPreset, sidecar
    results = tmp_path / "results"
    results.mkdir()
    _imported_splat(results, "scaniverse-splat", name=None)         # imported, no name tokens
    # Our own office build (imported=False via a real build manifest).
    d = sidecar.sidecar_paths("sam-office", results)
    d["dir"].mkdir(parents=True)
    d["ply"].write_text("ply")
    (tmp_path / "office.mp4").write_bytes(b"x" * 10)
    man = sidecar.build_manifest("Sam Office", "sam-office", tmp_path / "office.mp4",
                                 SplatPreset(), stats={"gaussians": 5},
                                 transform=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    sidecar.write_manifest_atomic(d["manifest"], man)
    sel = compare.choose_reference("officeFullScanAug6", results)
    assert sel["chosen"] == "scaniverse-splat"                     # the imported one, not our build


def test_choose_reference_none_when_no_import(tmp_path):
    results = tmp_path / "results"
    results.mkdir()
    sel = compare.choose_reference("anything", results)
    assert sel["ply"] is None and "no imported" in sel["reason"]


def test_compare_auto_selects_reference(tmp_path):
    results = tmp_path / "results"
    results.mkdir()
    _write_scan_mesh(results / "officeSweep.ply", _room(fork=True))
    _imported_splat(results, "office-truth", name="Sam Office")
    rep = compare.compare_scan_to_reference("officeSweep", results_dir=results, voxel=0.03,
                                            write_artifacts=False)
    assert rep["ok"] is True, rep
    assert rep["reference_selection"]["chosen"] == "office-truth"
