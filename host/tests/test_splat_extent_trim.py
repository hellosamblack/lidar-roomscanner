"""Regression tests for #176: the Splat camera's auto-frame (#108) backs off
~1.7x too far because it fits the camera to the RAW bounding box of the
splat's centers, and that box is dominated by a handful of low-density
outlier gaussians -- the spiky streaks radiating out of a photometric
COLMAP/gsplat build. Measured on `results/splats/sam-office` (144356
gaussians): full min/max radius 7.01, trimming the outer 1% of centers per
axis brings it to 3.59 (-49%); see `test_against_real_manifest_data` below,
which re-derives those numbers from the actual PLY on disk when it is
present, and `splat.js`'s own extent-trimming comment for the full survey
across every build under `results/splats/`.

The fix lives entirely in `splat.js` (`trimmedCentersBBox` / `percentile` /
`sampleSplatCenters`) -- `scene.js`'s `frameRoomBBox` is untouched, it just
gets fed a smaller box. These tests EXECUTE the shipped `splat.js` functions
under Node (the same cross-check pattern as `test_room_framing.py`, added
for #108) rather than reimplementing the trim arithmetic in Python -- a
Python port could stay green while the shipped code regressed. They also
run the untouched `frameRoomBBox` from `scene.js` on both the raw and the
trimmed box for the same synthetic data, to prove the actual user-visible
effect (a shorter camera distance) end to end, in the same two functions
`splat.js` really calls.

Skips (not fails) if Node is unavailable, matching `test_room_framing.py`'s
handling of a missing toolchain.
"""

from __future__ import annotations

import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

SPLAT_JS = Path(__file__).parent.parent / "src" / "roomscan" / "static" / "splat.js"
SCENE_JS = Path(__file__).parent.parent / "src" / "roomscan" / "static" / "scene.js"
RESULTS_SPLATS = Path(__file__).parent.parent.parent / "results" / "splats"


def _node() -> str | None:
    return shutil.which("node")


def _extract_braced_block(text: str, start_marker: str, fn_open_marker: str) -> str:
    """From `start_marker` through the matching closing brace of the function
    whose signature+opening-brace is `fn_open_marker` (last char of the
    marker must be the `{`). Mirrors `test_room_framing.py`'s
    `_extract_frame_room_bbox_source` -- same reasoning: brace-counting must
    start AT the function's own opening brace, or an earlier `{}` (e.g. a
    default-parameter object literal) double-counts and truncates the
    extraction mid-body.
    """
    start = text.index(start_marker)
    fn_start = text.index(fn_open_marker, start)
    body_open = fn_start + len(fn_open_marker) - 1
    assert text[body_open] == "{"
    depth = 1
    i = body_open + 1
    end = None
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
        i += 1
    assert end is not None, f"could not find matching closing brace for {fn_open_marker!r}"
    return text[start:end]


def _extract_splat_trim_source() -> str:
    text = SPLAT_JS.read_text(encoding="utf-8")
    return _extract_braced_block(
        text,
        "const SPLAT_TRIM_PERCENT = 1;",
        # NB: this marker must end at `sampleSplatCenters`' own opening brace.
        # The signature is multi-line since the `makeVec` parameter was added
        # (the fix for the inert-trim defect), so match its LAST line.
        "makeVec = () => new THREE.Vector3()) {",
    )


def _extract_frame_room_bbox_source() -> str:
    text = SCENE_JS.read_text(encoding="utf-8")
    src = _extract_braced_block(
        text,
        "export const ROOM_EYE_HEIGHT_M",
        "export function frameRoomBBox(bbox, fovDeg, opts = {}) {",
    )
    return src.replace("export const", "const").replace("export function", "function")


def _run_node(js_body: str):
    """Write the harness to a temp .js FILE and run it, rather than passing
    it as a `node -e` argv string -- the large center arrays some of these
    tests build (millions of floats, JSON-encoded) blow past the OS argv
    size limit (`OSError: [Errno 7] Argument list too long`) as an argument
    but not as file content."""
    node = _node()
    if node is None:
        pytest.skip("no node on PATH")
    with tempfile.NamedTemporaryFile(mode="w", suffix=".js", delete=False) as f:
        f.write(js_body)
        path = f.name
    try:
        proc = subprocess.run([node, path], capture_output=True, text=True, timeout=60)
        assert proc.returncode == 0, f"node harness failed:\n{proc.stderr}"
        return proc.stdout
    finally:
        Path(path).unlink(missing_ok=True)


def _trim(centers_flat, trim_percent=None):
    """Run `trimmedCentersBBox` from splat.js on a flat [x0,y0,z0,...] list."""
    src = _extract_splat_trim_source()
    args = json.dumps(trim_percent) if trim_percent is not None else ""
    harness = f"""
    {src}
    const result = trimmedCentersBBox({json.dumps(centers_flat)}{', ' + args if args else ''});
    console.log(JSON.stringify(result));
    """
    return json.loads(_run_node(harness))


def _radius(bbox):
    dx = bbox["maxX"] - bbox["minX"]
    dy = bbox["maxY"] - bbox["minY"]
    dz = bbox["maxZ"] - bbox["minZ"]
    return 0.5 * math.sqrt(dx * dx + dy * dy + dz * dz)


def _raw_bbox(centers_flat):
    xs = centers_flat[0::3]
    ys = centers_flat[1::3]
    zs = centers_flat[2::3]
    return {
        "minX": min(xs), "maxX": max(xs),
        "minY": min(ys), "maxY": max(ys),
        "minZ": min(zs), "maxZ": max(zs),
    }


# --------------------------------------------------------------------------
# Synthetic data: a dense "room" cluster in [-1, 1]^3 plus a small population
# of far outliers extending along each axis -- the same shape as a photo-
# metric build's streaks (a real minority of points, extreme in one axis).
# Built deterministically (no RNG) so the test is exact and reproducible.
# --------------------------------------------------------------------------


def _dense_cluster(n=3000):
    """n points on a deterministic low-discrepancy-ish grid inside [-1, 1]^3
    (not random -- fully reproducible), so trimming near the tails still
    lands predictably close to the cluster's own bound."""
    out = []
    for i in range(n):
        # Three independent pseudo-uniform sequences in [-1, 1] via distinct
        # irrational-multiple fractional parts (a cheap deterministic
        # low-discrepancy sequence -- no python `random` needed).
        x = 2 * ((i * 0.6180339887) % 1.0) - 1
        y = 2 * ((i * 0.7548776662) % 1.0) - 1
        z = 2 * ((i * 0.5698402909) % 1.0) - 1
        out.extend([x, y, z])
    return out


def _outlier_streaks(per_direction=7, magnitude=30.0):
    """Points extreme in exactly one axis/direction at a time, near
    `magnitude`, small on the other two axes -- mimics a photometric build's
    radiating low-density streaks."""
    out = []
    axes = [0, 1, 2]
    for axis in axes:
        for sign in (1, -1):
            for k in range(per_direction):
                jitter = (k - per_direction / 2) * 0.2
                point = [0.1 * ((k * 0.37) % 1.0)] * 3
                point[axis] = sign * magnitude + jitter
                out.extend(point)
    return out


DENSE = _dense_cluster(3000)
OUTLIERS = _outlier_streaks(per_direction=7, magnitude=30.0)
SYNTHETIC_CENTERS = DENSE + OUTLIERS  # 3000 + 42 = 3042 points total


# --------------------------------------------------------------------------
# 1. The trim actually recovers the extent: dominated by outliers before,
#    close to the dense cluster's own bound after.
# --------------------------------------------------------------------------


def test_raw_bbox_is_dominated_by_the_outliers():
    """Sanity check on the synthetic fixture itself (not the code under
    test): confirms the raw box really is inflated the way #176 describes,
    so the trimmed-vs-raw comparison below is meaningful."""
    raw = _raw_bbox(SYNTHETIC_CENTERS)
    assert _radius(raw) > 25, "fixture's outliers should dominate the raw box"


def test_trim_recovers_the_dense_clusters_own_extent():
    trimmed = _trim(SYNTHETIC_CENTERS)
    assert trimmed is not None
    # Every trimmed bound must land back inside (a little past) the dense
    # cluster's own [-1, 1] extent, nowhere near the magnitude-30 outliers.
    for k in ("minX", "maxX", "minY", "maxY", "minZ", "maxZ"):
        assert -1.5 <= trimmed[k] <= 1.5, f"{k}={trimmed[k]} should be inside the dense cluster, not the outliers"


def test_trim_shrinks_the_radius_by_more_than_the_measured_real_ratio():
    """Quantitative form of '~1.7x too far': the issue's own measurement on
    `sam-office` was radius 6.20 -> 3.58, a 1.73x shrink. This synthetic
    fixture is deliberately more extreme (magnitude-30 outliers vs a
    radius-~1.7 dense cluster) so demand at least that same 1.7x recovery,
    with headroom -- reintroducing the pre-#176 defect (skip the trim, use
    the raw box) collapses this ratio to 1.0 and fails the assertion below."""
    raw = _raw_bbox(SYNTHETIC_CENTERS)
    trimmed = _trim(SYNTHETIC_CENTERS)
    ratio = _radius(raw) / _radius(trimmed)
    assert ratio > 1.7, f"trim should recover most of the outlier-inflated radius, got ratio {ratio:.2f}"


def test_trim_percent_is_tunable():
    """5% trims further than 1% -- both directions of the knob work, so a
    later tuning pass has something to turn."""
    t1 = _trim(SYNTHETIC_CENTERS, trim_percent=1)
    t5 = _trim(SYNTHETIC_CENTERS, trim_percent=5)
    assert _radius(t5) < _radius(t1)


# --------------------------------------------------------------------------
# 2. Degenerate input: too few samples to trim meaningfully.
# --------------------------------------------------------------------------


def test_too_few_samples_returns_null():
    tiny = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0]  # 3 points, well under the floor
    assert _trim(tiny) is None


def test_empty_returns_null():
    assert _trim([]) is None


# --------------------------------------------------------------------------
# 3. End-to-end through the SAME `frameRoomBBox` splat.js actually calls:
#    the raw box parks the camera measurably farther back than the trimmed
#    one, for identical input data. This is the user-visible regression.
# --------------------------------------------------------------------------


def _frame_distance(bbox, fov_deg=60):
    src = _extract_frame_room_bbox_source()
    harness = f"""
    {src}
    const result = frameRoomBBox({json.dumps(bbox)}, {json.dumps(fov_deg)}, {{}});
    console.log(JSON.stringify(result));
    """
    return json.loads(_run_node(harness))["distance_m"]


def test_end_to_end_trimmed_camera_sits_closer_than_raw():
    raw_box = _raw_bbox(SYNTHETIC_CENTERS)
    trimmed_box = _trim(SYNTHETIC_CENTERS)
    d_raw = _frame_distance(raw_box)
    d_trimmed = _frame_distance(trimmed_box)
    assert d_trimmed < d_raw
    assert d_raw / d_trimmed > 1.7, (
        "the whole point of #176: the camera must back off substantially "
        "less once outlier gaussians are excluded from the framed box"
    )


# --------------------------------------------------------------------------
# 4. Stride sampling: bounded cost, and it must not distort the trim.
# --------------------------------------------------------------------------


# The vendor's `SplatBuffer.getSplatCenter(i, out, applySceneTransform)` calls
# `out.applyMatrix4(...)` when the transform flag is set -- it does NOT merely
# assign `.x/.y/.z`. The first version of this suite faked it as a plain
# assignment and a plain `{x, y, z}` scratch object, so all 11 tests passed
# while the real viewer threw `outCenter.applyMatrix4 is not a function` on
# every frame, silently fell back to the raw bounding box, and left #176
# completely inert (camera distance stayed at the raw 16.12 m for `sam-office`
# instead of the intended ~8 m).
#
# So the fake must be faithful to the part of the contract the code depends on.
# `makeVec` here is the minimum vendor-shaped scratch object; a fake that
# accepts a plain object would re-open exactly the hole this closes.
_VENDOR_FAITHFUL_FAKE = """
    const makeVec = () => ({
        x: 0, y: 0, z: 0,
        applyMatrix4() { return this; },   // identity: the vendor's transform hook
    });
    function makeFakeMesh(base, nBase, count) {
        return {
            getSplatCount: () => count,
            getSplatCenter: (i, out, applySceneTransform) => {
                const b = (i % nBase) * 3;
                out.x = base[b]; out.y = base[b + 1]; out.z = base[b + 2];
                // Faithful to the vendor: this is what throws on a plain object.
                if (applySceneTransform) out.applyMatrix4(null);
                return out;
            },
        };
    }
"""


def _sample_and_trim(splat_count, target_samples=None):
    """Run `sampleSplatCenters` + `trimmedCentersBBox` from splat.js against
    a fake mesh (duck-typed: `getSplatCount`/`getSplatCenter`) backed by the
    synthetic centers, repeated (via modulo indexing, not materialised) up
    to `splat_count` entries -- avoids building a multi-million-element
    Python list just to serialise it into the harness."""
    src = _extract_splat_trim_source()
    base = SYNTHETIC_CENTERS
    n_base = len(base) // 3
    target_arg = json.dumps(target_samples) if target_samples is not None else ""
    harness = f"""
    {src}
    {_VENDOR_FAITHFUL_FAKE}
    const base = {json.dumps(base)};
    const nBase = {n_base};
    const count = {splat_count};
    const mesh = makeFakeMesh(base, nBase, count);
    const samples = sampleSplatCenters(mesh, {target_arg or 'undefined'}, makeVec);
    const box = trimmedCentersBBox(samples);
    console.log(JSON.stringify({{ sampleCount: samples.length / 3, box }}));
    """
    return json.loads(_run_node(harness))


def test_sample_count_is_bounded_regardless_of_splat_count():
    """The whole reason to stride-sample: walking every centre on a
    2.46M-gaussian import is not free (measured ~0.6s in Python/numpy for a
    full walk vs ~4-9ms for a ~20-25k-point stride sample). Assert the
    sample count for a huge splat population stays near the 20000 target,
    not anywhere close to the full 2.46M."""
    result = _sample_and_trim(2_455_660)
    assert 15_000 <= result["sampleCount"] <= 25_000, (
        f"expected roughly the 20000-sample target, got {result['sampleCount']}"
    )


def test_small_splat_counts_sample_everything():
    """Below the target sample count, the stride must be 1 -- no silent
    under-sampling of a small build."""
    result = _sample_and_trim(500)
    assert result["sampleCount"] == 500


def test_stride_sampling_still_recovers_the_extent():
    """The bounded sample must still trim the outliers away -- not just run
    fast. Uses the same synthetic outlier/cluster fixture, tiled up to a
    large splat count and walked through the real stride path."""
    result = _sample_and_trim(2_000_000)
    box = result["box"]
    for k in ("minX", "maxX", "minY", "maxY", "minZ", "maxZ"):
        assert -1.5 <= box[k] <= 1.5, f"{k}={box[k]} should still land inside the dense cluster after striding"


# --------------------------------------------------------------------------
# 5. Against real captured data, when present (results/ is gitignored --
#    skip rather than fail when it is absent, same handling as the missing
#    Node toolchain above).
# --------------------------------------------------------------------------


def _load_manifest_transform_and_centers(slug: str, ply_name: str):
    plyfile = pytest.importorskip("plyfile")
    import numpy as np

    manifest_path = RESULTS_SPLATS / slug / "manifest.json"
    ply_path = RESULTS_SPLATS / slug / ply_name
    if not manifest_path.exists() or not ply_path.exists():
        pytest.skip(f"no captured splat build at {ply_path} (results/ is gitignored)")

    manifest = json.loads(manifest_path.read_text())
    transform = np.array(manifest["transform"])
    ply = plyfile.PlyData.read(str(ply_path))
    v = ply["vertex"].data
    centers = np.stack([v["x"], v["y"], v["z"]], axis=1).astype(np.float64)
    homog = np.concatenate([centers, np.ones((len(centers), 1))], axis=1)
    world = (homog @ transform.T)[:, :3]
    return world


def test_against_real_manifest_data():
    """Re-derives the actual measurement #176 is based on: on `sam-office`
    (144356 gaussians), the raw radius should be well over the trimmed
    radius, and the trimmed radius should sit close to the 3.59 this issue
    measured (`splat.js`'s own comment) -- loose bounds because a rebuild of
    this fixture will move the exact number slightly."""
    world = _load_manifest_transform_and_centers("sam-office", "point_cloud.ply")
    flat = world.reshape(-1).tolist()
    raw = _raw_bbox(flat)
    trimmed = _trim(flat)
    r_raw = _radius(raw)
    r_trimmed = _radius(trimmed)
    assert r_raw > 6.0, f"raw radius should match the issue's own ~7.0 measurement, got {r_raw:.2f}"
    assert 2.5 < r_trimmed < 4.5, f"trimmed radius should be near the issue's own ~3.6 measurement, got {r_trimmed:.2f}"
    assert r_raw / r_trimmed > 1.5


def test_sampling_requires_a_vector3_like_scratch_object():
    """Regression guard for the defect that made #176 inert on landing.

    `sampleSplatCenters` passes ONE scratch object to the vendor's
    `getSplatCenter(i, out, true)`. With the transform flag set the vendor
    calls `out.applyMatrix4(...)`, so a plain `{x, y, z}` throws
    `outCenter.applyMatrix4 is not a function` -- which `frameCamera`'s
    try/catch swallowed, falling back to the raw box. Every unit test passed
    and the camera never moved.

    This asserts the failure is REAL against a vendor-faithful fake, so the
    duck-typed version cannot come back. It is the mirror of
    `test_stride_sampling_still_recovers_the_extent`, which proves the
    Vector3-like path works.
    """
    src = _extract_splat_trim_source()
    harness = f"""
    {src}
    {_VENDOR_FAITHFUL_FAKE}
    const mesh = makeFakeMesh([0,0,0, 1,1,1], 2, 40);
    let plainErr = null, vecOk = null;
    try {{
        sampleSplatCenters(mesh, undefined, () => ({{ x: 0, y: 0, z: 0 }}));
    }} catch (e) {{ plainErr = e.message; }}
    try {{
        const s = sampleSplatCenters(mesh, undefined, makeVec);
        vecOk = s.length / 3;
    }} catch (e) {{ vecOk = 'THREW: ' + e.message; }}
    console.log(JSON.stringify({{ plainErr, vecOk }}));
    """
    out = json.loads(_run_node(harness))
    assert out["plainErr"] is not None, (
        "a plain {x,y,z} scratch object must throw against a vendor-faithful "
        "getSplatCenter -- if it does not, the fake has drifted from the vendor "
        "again and this suite has no power over the real code path"
    )
    assert "applyMatrix4" in out["plainErr"]
    assert out["vecOk"] == 40, f"Vector3-like scratch must sample all 40 centers, got {out['vecOk']!r}"
