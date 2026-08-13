"""Reader for a documented RTAB-Map `rtabmap-export` output directory (issue #158).

Supports exactly the export produced by the command already pinned in
`docs/rtabmap-pixel10-capture.md` section 6::

    rtabmap-export --images --poses_camera --poses_format 1 --output_dir <dir> <name>.db

against `introlab/rtabmap` @ `2e193ee1` (v0.23.10, local checkout `~/git/personal/rtabmap`,
`tools/Export/main.cpp`). A raw `.db` is rejected with a pointer to that export step -- this
reader does not shell out to `rtabmap-export`/Docker itself, and does not parse SQLite.

GPU-free and torch/gsplat/pycolmap-free by construction (only `numpy`, `PIL.Image`, stdlib).

Export layout (`tools/Export/main.cpp:1610-1682`), for `--output_dir <dir>` and a database
whose export base name is `<base>` (defaults to the `.db` stem; `--output` overrides it)::

    <dir>/<base>_rgb/<stamp>.jpg           -- RGB, one per accepted node
    <dir>/<base>_depth/<stamp>.png         -- registered depth (uint16 mm typically)
    <dir>/<base>_confidence/<stamp>.png    -- per-pixel depth confidence
    <dir>/<base>_calib/<stamp>.yaml        -- per-frame calibration (autofocus changes it)
    <dir>/<base>_camera_poses.txt          -- "#timestamp x y z qx qy qz qw", format 1
    <dir>/<base>_cloud.ply | _mesh.ply|.obj | _mesh_multiband.obj   -- optional geometry

`<stamp>` is the literal `%f`-formatted double RTAB-Map stores per node
(`main.cpp:1516` `getNodeInfo` -> `stamp`), reused verbatim for every artifact filename
*and* as column 1 of the poses file (`main.cpp:1808`: `cameraStamps[i][node] = stamp`) --
that shared string is the exporter's own cross-artifact identifier, so association here is
by that string, never by directory-listing order.
"""
from __future__ import annotations

import math
import re
from pathlib import Path

import numpy as np

from .posed import PosedCapture, PosedFrame, PosedFrameError

RTABMAP_DOC = "docs/rtabmap-pixel10-capture.md"

# The `PosedCapture.world_frame` / `PosedCapture.geometry_frame` label for everything this
# reader produces: RTAB-Map's own graph-optimized map/world frame (recovered from the
# `--poses_format 1` file by undoing its axis remap below), camera child axes in the
# standard pinhole/optical convention. NOT viewer-leveled, NOT the OpenGL-world frame the
# Android app additionally carries (`rtabmap_world_T_opengl_world`, `RTABMapApp.cpp:4690`)
# -- that transform is app-side only and never appears in a desktop `rtabmap-export` output.
RTABMAP_WORLD_FRAME = "rtabmap_map_optical"

# Column-0 of `<base>_camera_poses.txt` is RTAB-Map's own per-node capture timestamp
# (`main.cpp:1516`), not something inferred from a node id or filename.
RTABMAP_TIMESTAMP_DOMAIN = "rtabmap_export_stamp_s"

_GEOMETRY_SUFFIXES = ("_cloud.ply", "_cloud.las", "_mesh.ply", "_mesh.obj", "_mesh_multiband.obj")


class RtabmapExportError(Exception):
    """A documented RTAB-Map export directory failed strict validation."""


# --------------------------------------------------------------------------------------
# Pose-convention conversion -- verified against the pinned source, not visual plausibility.
#
# `tools/Export/main.cpp:1807`: `cameraPoses[node] = robotPose * model.localTransform()`.
# That is already `map_T_optical` (camera-to-world, RTAB's own map frame, camera child axes
# = the standard pinhole/optical convention -- `CameraModel::opticalRotation()`,
# `CameraModel.h:63`, is the *default* `localTransform`, and its columns verify as optical
# basis vectors expressed in the robot/base frame: x-right -> -body_y, y-down -> -body_z,
# z-forward -> +body_x, matching ROS REP103 optical<->base).
#
# `corelib/src/Graph.cpp:86-99` (`exportPoses`, `format==1` branch) then remaps that pose
# into "RGBD-SLAM/motion-capture" convention before writing it, in two steps that are each a
# *pure rotation* (zero translation), so they factor cleanly:
#
#   t1 = Transform(0,0,1,0, 0,-1,0,0, 1,0,0,0)          # world-axis remap
#   t2 = Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0)          # == CameraModel::opticalRotation()
#   pose = t1.inverse() * pose_in                        # world-axis remap
#   pose = t2.inverse() * pose * t2                       # + a SECOND local-axis remap
#
# Composing (both t1, t2 have zero translation, so this is exact, not an approximation):
#   R_out = (t2^-1 t1^-1) @ R_in @ t2   =  W_inv @ R_in @ O_inv          [W = O = as below]
#   t_out = (t2^-1 t1^-1) @ t_in         =  W_inv @ t_in
#
# `rtabmap_format1_pose_to_world_T_camera_optical` below undoes exactly that (W = W_inv^-1,
# O = O_inv^-1; both are pure rotations so inverse == transpose), recovering `pose_in` --
# i.e. `map_T_optical` in RTAB-Map's own map frame. Numerically verified by round-tripping a
# random rigid pose through the forward formula above and back (see
# `test_splat_rtabmap.py::test_format1_round_trip_matches_pinned_source_algebra`), and by the
# hand-derived identity/translation/90-degree-rotation cases in the same test module.
_T1_ROT = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
_OPTICAL_ROT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # opticalRotation()
_WORLD_REMAP_INV = _T1_ROT @ _OPTICAL_ROT          # == W: undoes the world-axis remap
_LOCAL_REMAP_INV = _OPTICAL_ROT.T                  # == O: undoes the local-axis remap


def _quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _invert_rigid(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rtabmap_format1_pose_to_world_T_camera_optical(xyz: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    """Undo RTAB-Map's `--poses_format 1` (RGBD-SLAM/"motion capture") remap.

    Input: one row of `<base>_camera_poses.txt` from
    `rtabmap-export --poses_camera --poses_format 1` -- `xyz` (3,) and `quat_xyzw` (4,)
    (x,y,z,w order, as rtabmap writes it), together the RGBD-SLAM-convention camera pose
    `Graph::exportPoses()`'s `format==1` branch writes.

    Output: the 4x4 camera-to-world matrix (`world_T_camera`, i.e. `map_T_optical` in the
    exporter's own terms) *before* that remap -- RTAB-Map's own map/world frame
    (`RTABMAP_WORLD_FRAME`), camera child axes in the standard pinhole/optical convention
    (x-right, y-down, z-forward). See the module-level derivation above this function.
    """
    R_out = _quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=float))
    t_out = np.asarray(xyz, dtype=float)
    T = np.eye(4)
    T[:3, :3] = _WORLD_REMAP_INV @ R_out @ _LOCAL_REMAP_INV
    T[:3, 3] = _WORLD_REMAP_INV @ t_out
    return T


# --------------------------------------------------------------------------------------
# `<base>_camera_poses.txt` parsing.

def _parse_camera_poses_format1(path: Path) -> list[tuple[str, float, np.ndarray, np.ndarray]]:
    """Return `[(stamp_token, stamp_value, xyz, quat_xyzw), ...]` in file order.

    `stamp_token` is the raw text of column 1, kept verbatim (not re-formatted) so it can be
    used directly to look up `<stamp_token>.jpg`/`.png`/`.yaml` -- guaranteed to match the
    exporter's own filenames, which is `printf`'d from the exact same double.
    """
    text = path.read_text(encoding="utf-8")
    data_lines = [ln for ln in text.splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
    if not data_lines:
        raise RtabmapExportError(f"{path}: no pose rows found (empty or header-only file)")

    rows: list[tuple[str, float, np.ndarray, np.ndarray]] = []
    seen: set[str] = set()
    for i, line in enumerate(data_lines, start=1):
        tokens = line.split()
        if len(tokens) != 8:
            raise RtabmapExportError(
                f"{path}: line {i} has {len(tokens)} column(s), expected 8 "
                "(\"stamp x y z qx qy qz qw\" -- --poses_format 1/RGBD-SLAM). "
                "Other --poses_format values (including 11's id suffix) are not supported by "
                f"this reader; re-export with --poses_camera --poses_format 1 per {RTABMAP_DOC}.")
        stamp_token = tokens[0]
        try:
            values = [float(tok) for tok in tokens]
        except ValueError as e:
            raise RtabmapExportError(f"{path}: line {i} ({stamp_token!r}): non-numeric field: {e}") from e
        if not all(math.isfinite(v) for v in values):
            raise RtabmapExportError(f"{path}: line {i} ({stamp_token!r}): non-finite pose value")
        if stamp_token in seen:
            raise RtabmapExportError(f"{path}: duplicate stamp {stamp_token!r} (line {i})")
        seen.add(stamp_token)
        quat = np.array(values[4:8], dtype=float)
        qnorm = float(np.linalg.norm(quat))
        if qnorm < 1e-6:
            raise RtabmapExportError(f"{path}: line {i} ({stamp_token!r}): zero-norm quaternion")
        rows.append((stamp_token, values[0], np.array(values[1:4], dtype=float), quat / qnorm))
    return rows


def _find_camera_poses_file(export_dir: Path, base_name: str | None) -> tuple[str, Path]:
    if base_name is not None:
        p = export_dir / f"{base_name}_camera_poses.txt"
        if not p.exists():
            raise RtabmapExportError(f"{p} not found (base_name={base_name!r} was given explicitly)")
        return base_name, p

    multi = sorted(export_dir.glob("*_camera_poses_*.txt"))
    if multi:
        raise RtabmapExportError(
            f"{export_dir}: multi-camera pose export found ({[p.name for p in multi]}); "
            "this reader supports single-camera (phone RGB-D) exports only.")

    candidates = sorted(export_dir.glob("*_camera_poses.txt"))
    if not candidates:
        raise RtabmapExportError(
            f"{export_dir}: no *_camera_poses.txt found. Run `rtabmap-export --images "
            f"--poses_camera --poses_format 1 --output_dir {export_dir} <name>.db` first -- "
            f"see {RTABMAP_DOC}.")
    if len(candidates) > 1:
        raise RtabmapExportError(
            f"{export_dir}: ambiguous export directory, multiple *_camera_poses.txt found: "
            f"{[p.name for p in candidates]}; pass base_name= explicitly.")
    p = candidates[0]
    return p.name[: -len("_camera_poses.txt")], p


# --------------------------------------------------------------------------------------
# Per-frame calibration (`CameraModel::save()`, `corelib/src/CameraModel.cpp:404-434`).
#
# NOT an `!!opencv-matrix`-tagged cv::FileStorage node -- `CameraModel::save()` builds each
# block manually (`fs << "camera_matrix" << "{"; fs << "rows" << ...; fs << "}"`), so the
# file is a plain nested YAML mapping. This is a small dedicated parser for exactly that
# fixed structure, not a general YAML/OpenCV-FileStorage reader.
_BLOCK_KEY_RE = re.compile(r"^(\w+):\s*(.*)$")


def _parse_opencv_yaml_lite(text: str) -> dict:
    lines = [ln for ln in text.splitlines()
             if ln.strip() and not ln.startswith("%") and ln.strip() != "---"]
    sections: dict[str, list[str]] = {}
    cur_key = None
    for ln in lines:
        if not ln[:1].isspace():
            m = _BLOCK_KEY_RE.match(ln)
            if not m:
                continue
            cur_key = m.group(1)
            rest = m.group(2).strip()
            sections[cur_key] = [rest] if rest else []
        elif cur_key is not None:
            sections[cur_key].append(ln.strip())

    result: dict = {}
    for key, body in sections.items():
        if len(body) == 1 and body[0] and "rows" not in body[0] and "data" not in body[0]:
            result[key] = body[0].strip().strip('"')
            continue
        block_text = " ".join(body)
        rows_m = re.search(r"rows:\s*(\d+)", block_text)
        cols_m = re.search(r"cols:\s*(\d+)", block_text)
        data_m = re.search(r"data:\s*\[(.*?)\]", block_text)
        if rows_m and cols_m and data_m:
            values = [float(x) for x in data_m.group(1).split(",") if x.strip()]
            result[key] = {"rows": int(rows_m.group(1)), "cols": int(cols_m.group(1)), "data": values}
    return result


def _load_calibration(path: Path) -> tuple[np.ndarray, int | None, int | None]:
    if not path.exists():
        raise RtabmapExportError(f"calibration file not found: {path}")
    parsed = _parse_opencv_yaml_lite(path.read_text(encoding="utf-8"))
    cm = parsed.get("camera_matrix")
    if not isinstance(cm, dict) or cm.get("rows") != 3 or cm.get("cols") != 3 or len(cm.get("data", [])) != 9:
        raise RtabmapExportError(f"{path}: missing or malformed camera_matrix "
                                 "(expected a 3x3 camera_matrix block)")
    k = np.array(cm["data"], dtype=float).reshape(3, 3)
    width = int(parsed["image_width"]) if "image_width" in parsed else None
    height = int(parsed["image_height"]) if "image_height" in parsed else None
    return k, width, height


def _image_size(path: Path) -> tuple[int, int]:
    from PIL import Image
    with Image.open(path) as im:
        return im.size  # (width, height)


# --------------------------------------------------------------------------------------
# Association + top-level loader.

def _associate_frames(export_dir: Path, base_name: str,
                       rows: list[tuple[str, float, np.ndarray, np.ndarray]], *,
                       require_depth: bool, require_confidence: bool) -> tuple[list[dict], list[str]]:
    rgb_dir = export_dir / f"{base_name}_rgb"
    depth_dir = export_dir / f"{base_name}_depth"
    conf_dir = export_dir / f"{base_name}_confidence"
    calib_dir = export_dir / f"{base_name}_calib"

    frames: list[dict] = []
    problems: list[str] = []
    for stamp_token, stamp_value, xyz, quat in rows:
        rgb_path = rgb_dir / f"{stamp_token}.jpg"
        calib_path = calib_dir / f"{stamp_token}.yaml"
        depth_path = depth_dir / f"{stamp_token}.png"
        conf_path = conf_dir / f"{stamp_token}.png"

        missing = []
        if not rgb_path.exists():
            missing.append("rgb")
        if not calib_path.exists():
            missing.append("calibration")
        if require_depth and not depth_path.exists():
            missing.append("depth")
        if require_confidence and not conf_path.exists():
            missing.append("confidence")
        if missing:
            problems.append(f"frame {stamp_token}: missing {', '.join(missing)}")
            continue

        try:
            width, height = _image_size(rgb_path)
        except OSError as e:
            problems.append(f"frame {stamp_token}: unreadable RGB image {rgb_path}: {e}")
            continue

        try:
            k, calib_w, calib_h = _load_calibration(calib_path)
        except RtabmapExportError as e:
            problems.append(str(e))
            continue

        if calib_w and calib_h and (calib_w, calib_h) != (width, height):
            problems.append(f"frame {stamp_token}: calibration size {calib_w}x{calib_h} "
                            f"!= image size {width}x{height} ({calib_path} vs {rgb_path})")
            continue

        world_T_camera = rtabmap_format1_pose_to_world_T_camera_optical(xyz, quat)
        frames.append({
            "frame_id": stamp_token, "image_path": rgb_path, "width": width, "height": height,
            "k": k, "pose_camera_from_world": _invert_rigid(world_T_camera),
            "depth_path": depth_path if depth_path.exists() else None,
            "confidence_path": conf_path if conf_path.exists() else None,
            "timestamp": stamp_value, "timestamp_domain": RTABMAP_TIMESTAMP_DOMAIN,
        })
    return frames, problems


def _find_orphans(export_dir: Path, base_name: str, known_stamps: set[str]) -> list[str]:
    problems = []
    for subdir, ext in ((f"{base_name}_rgb", ".jpg"), (f"{base_name}_depth", ".png"),
                        (f"{base_name}_confidence", ".png"), (f"{base_name}_calib", ".yaml")):
        d = export_dir / subdir
        if not d.is_dir():
            continue
        for p in sorted(d.iterdir()):
            if p.is_file() and p.suffix == ext and p.stem not in known_stamps:
                problems.append(f"orphan file with no matching pose row: {p}")
    return problems


def _find_geometry(export_dir: Path, base_name: str) -> list[Path]:
    return [p for suf in _GEOMETRY_SUFFIXES
            if (p := export_dir / f"{base_name}{suf}").exists()]


def load_rtabmap_export(export_dir, *, base_name: str | None = None,
                        require_depth: bool = True, require_confidence: bool = True) -> PosedCapture:
    """Strictly load a documented `rtabmap-export` directory into a `PosedCapture`.

    GPU-free; imports no torch/gsplat/pycolmap/CUDA. Every accepted frame carries its own
    RGB path, dimensions, calibration, canonical world-to-camera pose, and (by default)
    registered-depth/confidence paths. No frame is silently dropped: any missing, duplicate,
    malformed, or ambiguously-associated record raises `RtabmapExportError` naming the
    offending frame stamp(s)/path(s).

    `require_depth`/`require_confidence` default to strict (True) for the Phase-7 RGB-D
    path; pass False only for ad-hoc RGB-only diagnostics, never as a silent default.
    """
    export_dir = Path(export_dir)
    if export_dir.is_file() or export_dir.suffix == ".db":
        raise RtabmapExportError(
            f"{export_dir} looks like an RTAB-Map .db, not an export directory. Run "
            f"`rtabmap-export --images --poses_camera --poses_format 1 --output_dir <dir> "
            f"{export_dir}` first -- see {RTABMAP_DOC}.")
    if not export_dir.is_dir():
        raise RtabmapExportError(f"{export_dir} does not exist or is not a directory "
                                 f"(see {RTABMAP_DOC} for the export step).")

    base_name, poses_path = _find_camera_poses_file(export_dir, base_name)
    rows = _parse_camera_poses_format1(poses_path)

    frames, problems = _associate_frames(export_dir, base_name, rows,
                                         require_depth=require_depth,
                                         require_confidence=require_confidence)
    problems += _find_orphans(export_dir, base_name, {r[0] for r in rows})
    if problems:
        raise RtabmapExportError(
            f"{export_dir} (base={base_name!r}): {len(problems)} problem(s):\n  "
            + "\n  ".join(problems))

    posed_frames = []
    for kwargs in frames:
        try:
            posed_frames.append(PosedFrame(**kwargs))
        except PosedFrameError as e:
            raise RtabmapExportError(f"{export_dir} (base={base_name!r}): frame "
                                     f"{kwargs['frame_id']}: {e}") from e

    geometry_paths = _find_geometry(export_dir, base_name)
    return PosedCapture(source="rtabmap_export", frames=tuple(posed_frames),
                        world_frame=RTABMAP_WORLD_FRAME,
                        geometry_paths=tuple(geometry_paths),
                        geometry_frame=RTABMAP_WORLD_FRAME if geometry_paths else None)


def summarize_rtabmap_export(export_dir, *, base_name: str | None = None,
                             require_depth: bool = True, require_confidence: bool = True) -> dict:
    """GPU-free validation summary -- the `roomscan-splat inspect-rtabmap` payload.

    Never raises: on a failed `load_rtabmap_export`, returns `{"ok": False, "reason": ...}`
    with the same actionable message the exception carries, so a corrupt export is diagnosed
    without a traceback. Reports what would be trained on, not what was requested; validates,
    does not train.
    """
    export_dir = Path(export_dir)
    try:
        capture = load_rtabmap_export(export_dir, base_name=base_name,
                                      require_depth=require_depth,
                                      require_confidence=require_confidence)
    except RtabmapExportError as e:
        return {"ok": False, "export_dir": str(export_dir), "reason": str(e)}

    frames = capture.frames
    calib_signatures = {tuple(np.round(np.asarray(f.k, dtype=float).flatten(), 6)) for f in frames}
    domains = sorted({f.timestamp_domain for f in frames if f.timestamp_domain is not None})
    return {
        "ok": True,
        "export_dir": str(export_dir),
        "source": capture.source,
        "world_frame": capture.world_frame,
        "frame_count": len(frames),
        "frames_with_depth": sum(1 for f in frames if f.depth_path is not None),
        "frames_with_confidence": sum(1 for f in frames if f.confidence_path is not None),
        "distinct_calibrations": len(calib_signatures),
        "poses_valid": len(frames),  # every constructed PosedFrame already passed pose validation
        "timestamps_present": len(frames) > 0 and all(f.timestamp is not None for f in frames),
        "timestamp_domains": domains,
        "geometry_paths": [str(p) for p in capture.geometry_paths],
        "geometry_frame": capture.geometry_frame,
    }
