"""Compare roomscanner SLAM against an independent RTAB-Map trajectory (#188).

Clock alignment uses angular-speed magnitude because it is unchanged by the
unknown fixed phone-to-scanner extrinsic.  Position comparison then resamples the
roomscanner trajectory at RTAB-Map keyframe times and rigidly aligns only a short
leading window.  The rest of the path remains held out, so error growth is visible.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .rtabmap_db import RtabmapDatabaseError, RtabmapTrajectory, read_optimized_trajectory


class TrajectoryCompareError(ValueError):
    """Two trajectories cannot be compared under the documented contract."""


@dataclass(frozen=True)
class TumTrajectory:
    """A TUM trajectory: timestamp plus world-from-camera pose per sample."""

    timestamps_s: np.ndarray
    positions_m: np.ndarray
    rotations: np.ndarray


def _quat_xyzw_to_matrices(quaternions: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternions, dtype=np.float64)
    if q.ndim != 2 or q.shape[1] != 4:
        raise TrajectoryCompareError("quaternions must have shape (N, 4)")
    norms = np.linalg.norm(q, axis=1)
    if np.any(~np.isfinite(norms)) or np.any(norms < 1e-9):
        raise TrajectoryCompareError("trajectory contains a non-finite or zero quaternion")
    x, y, z, w = (q / norms[:, None]).T
    rotations = np.empty((len(q), 3, 3), dtype=np.float64)
    rotations[:, 0, 0] = 1 - 2 * (y * y + z * z)
    rotations[:, 0, 1] = 2 * (x * y - w * z)
    rotations[:, 0, 2] = 2 * (x * z + w * y)
    rotations[:, 1, 0] = 2 * (x * y + w * z)
    rotations[:, 1, 1] = 1 - 2 * (x * x + z * z)
    rotations[:, 1, 2] = 2 * (y * z - w * x)
    rotations[:, 2, 0] = 2 * (x * z - w * y)
    rotations[:, 2, 1] = 2 * (y * z + w * x)
    rotations[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return rotations


def read_tum_trajectory(path: str | Path) -> TumTrajectory:
    """Read ``timestamp tx ty tz qx qy qz qw`` rows from a TUM file."""
    tum_path = Path(path)
    if not tum_path.is_file():
        raise TrajectoryCompareError(f"roomscanner TUM trajectory not found: {tum_path}")
    try:
        text = tum_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        raise TrajectoryCompareError(f"cannot read {tum_path}: {exc}") from exc

    rows: list[list[float]] = []
    for line_number, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        tokens = stripped.split()
        if len(tokens) != 8:
            raise TrajectoryCompareError(
                f"{tum_path}: line {line_number} has {len(tokens)} columns, expected 8")
        try:
            rows.append([float(token) for token in tokens])
        except ValueError as exc:
            raise TrajectoryCompareError(
                f"{tum_path}: line {line_number} contains a non-numeric value: {exc}") from exc

    if len(rows) < 3:
        raise TrajectoryCompareError(f"{tum_path}: need at least 3 trajectory rows")
    data = np.asarray(rows, dtype=np.float64)
    if not np.isfinite(data).all():
        raise TrajectoryCompareError(f"{tum_path}: trajectory contains non-finite values")
    timestamps = data[:, 0]
    if np.any(np.diff(timestamps) <= 0):
        raise TrajectoryCompareError(f"{tum_path}: timestamps must be strictly increasing")
    return TumTrajectory(
        timestamps_s=timestamps,
        positions_m=data[:, 1:4],
        rotations=_quat_xyzw_to_matrices(data[:, 4:8]),
    )


def _rotation_angles(rotations: np.ndarray) -> np.ndarray:
    relative = np.einsum("nji,njk->nik", rotations[:-1], rotations[1:])
    cosine = np.clip((np.trace(relative, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    return np.arccos(cosine)


def _correlation(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) < 3 or np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def align_clock_by_angular_speed(
    rtab_timestamps_s: np.ndarray,
    rtab_rotations: np.ndarray,
    room_timestamps_s: np.ndarray,
    room_rotations: np.ndarray,
    *,
    search_step_s: float = 0.02,
) -> dict:
    """Find roomscanner time at RTAB time zero by scalar speed correlation.

    A full RTAB trajectory is required to fit inside the roomscanner timeline.
    That is the normal paired-capture case: the phone may start late or die early,
    while the scanner keeps recording.  Requiring full phone coverage prevents a
    short accidental partial match from winning the correlation search.
    """
    if search_step_s <= 0 or not np.isfinite(search_step_s):
        raise TrajectoryCompareError("clock search_step_s must be finite and positive")
    rt = np.asarray(rtab_timestamps_s, dtype=np.float64)
    ot = np.asarray(room_timestamps_s, dtype=np.float64)
    if len(rt) < 4 or len(ot) < 4:
        raise TrajectoryCompareError("clock alignment needs at least four poses per trajectory")
    if (np.asarray(rtab_rotations).shape != (len(rt), 3, 3) or
            np.asarray(room_rotations).shape != (len(ot), 3, 3)):
        raise TrajectoryCompareError("clock-alignment rotations have incompatible shapes")
    if (not np.isfinite(rt).all() or not np.isfinite(ot).all() or
            np.any(np.diff(rt) <= 0) or np.any(np.diff(ot) <= 0)):
        raise TrajectoryCompareError(
            "clock-alignment timestamps must be finite and strictly increasing")
    rt = rt - rt[0]
    ot = ot - ot[0]
    if rt[-1] > ot[-1]:
        raise TrajectoryCompareError(
            "RTAB-Map trajectory is longer than the roomscanner trajectory; "
            "cannot find a full-coverage clock alignment")

    rtab_speed = _rotation_angles(rtab_rotations) / np.diff(rt)
    room_speed = _rotation_angles(room_rotations) / np.diff(ot)
    rtab_mid = (rt[:-1] + rt[1:]) * 0.5
    room_mid = (ot[:-1] + ot[1:]) * 0.5
    max_offset = float(ot[-1] - rt[-1])
    offsets = np.arange(0.0, max_offset + search_step_s * 0.5, search_step_s)

    best_offset = None
    best_score = -np.inf
    for offset in offsets:
        sampled = np.interp(rtab_mid + offset, room_mid, room_speed)
        score = _correlation(rtab_speed, sampled)
        if np.isfinite(score) and score > best_score:
            best_score = score
            best_offset = float(offset)
    if best_offset is None:
        raise TrajectoryCompareError(
            "angular-speed clock alignment is undefined (insufficient rotational variation)")
    return {
        "method": "angular_speed_magnitude_correlation",
        "roomscan_time_at_rtab_start_s": best_offset,
        "correlation": best_score,
        "search_step_s": float(search_step_s),
        "candidates": int(len(offsets)),
    }


def _nearest_indices(samples: np.ndarray, targets: np.ndarray) -> np.ndarray:
    right = np.searchsorted(samples, targets, side="left")
    right = np.clip(right, 0, len(samples) - 1)
    left = np.maximum(right - 1, 0)
    choose_left = np.abs(targets - samples[left]) <= np.abs(samples[right] - targets)
    return np.where(choose_left, left, right)


def _rigid_umeyama(source: np.ndarray, target: np.ndarray, *, allow_scale: bool) -> tuple:
    """Return ``(R, t, scale)`` mapping source points onto target points."""
    if len(source) < 3 or source.shape != target.shape:
        raise TrajectoryCompareError("Umeyama alignment needs at least 3 paired 3-D points")
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = source_centered.T @ target_centered / len(source)
    u, singular, vt = np.linalg.svd(covariance)
    sign = np.ones(3)
    if np.linalg.det(vt.T @ u.T) < 0:
        sign[-1] = -1.0
    rotation = vt.T @ np.diag(sign) @ u.T
    variance = float(np.mean(np.sum(source_centered * source_centered, axis=1)))
    if variance < 1e-12:
        raise TrajectoryCompareError("Umeyama source trajectory has no positional variation")
    fitted_scale = float(np.sum(singular * sign) / variance)
    scale = fitted_scale if allow_scale else 1.0
    translation = target_mean - scale * (rotation @ source_mean)
    return rotation, translation, fitted_scale


def _path_length(positions: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def compare_matched_trajectories(
    rtab_timestamps_s: np.ndarray,
    rtab_poses: np.ndarray,
    room_positions_m: np.ndarray,
    room_rotations: np.ndarray,
    *,
    alignment_window_s: float = 30.0,
    final_window_s: float = 60.0,
) -> dict:
    """Score already time-matched poses, holding post-alignment motion out.

    This pure numerical half is separate so the real AshOffice matched-keyframe
    result can be a small regression fixture without checking a 49 MB phone DB
    and a full capture trajectory into git.
    """
    rt = np.asarray(rtab_timestamps_s, dtype=np.float64)
    poses = np.asarray(rtab_poses, dtype=np.float64)
    room_positions = np.asarray(room_positions_m, dtype=np.float64)
    room_rots = np.asarray(room_rotations, dtype=np.float64)
    if (len(rt) < 4 or poses.shape != (len(rt), 4, 4) or
            room_positions.shape != (len(rt), 3) or room_rots.shape != (len(rt), 3, 3)):
        raise TrajectoryCompareError("matched trajectories have incompatible shapes")
    if (not np.isfinite(rt).all() or np.any(np.diff(rt) <= 0) or
            not np.isfinite(poses).all() or not np.isfinite(room_positions).all() or
            not np.isfinite(room_rots).all()):
        raise TrajectoryCompareError(
            "matched trajectories must be finite with strictly increasing timestamps")
    if alignment_window_s <= 0 or final_window_s <= 0:
        raise TrajectoryCompareError("alignment and final windows must be positive")
    relative_t = rt - rt[0]
    align_mask = relative_t <= alignment_window_s
    if int(align_mask.sum()) < 3:
        raise TrajectoryCompareError("alignment window contains fewer than three keyframes")

    rtab_positions = poses[:, :3, 3]
    rotation, translation, leading_scale = _rigid_umeyama(
        room_positions[align_mask], rtab_positions[align_mask], allow_scale=False)
    aligned_room = (rotation @ room_positions.T).T + translation
    error = aligned_room - rtab_positions

    _, _, full_similarity_scale = _rigid_umeyama(
        room_positions, rtab_positions, allow_scale=True)
    rtab_path = _path_length(rtab_positions)
    room_path = _path_length(room_positions)
    rtab_rotation = _rotation_angles(poses[:, :3, :3])
    room_rotation = _rotation_angles(room_rots)
    rtab_rotation_total = float(rtab_rotation.sum())
    room_rotation_total = float(room_rotation.sum())
    if rtab_path < 1e-12:
        raise TrajectoryCompareError("RTAB-Map path length is zero; physical scale is undefined")
    if rtab_rotation_total < 1e-12:
        raise TrajectoryCompareError("RTAB-Map total rotation is zero; rotation ratio is undefined")

    final_start = max(0.0, float(relative_t[-1] - final_window_s))
    final_mask = relative_t >= final_start
    bins = []
    bin_start = 0.0
    while bin_start < relative_t[-1]:
        bin_end = min(bin_start + 30.0, float(relative_t[-1]) + 1e-9)
        mask = (relative_t >= bin_start) & (
            relative_t <= bin_end if bin_end >= relative_t[-1] else relative_t < bin_end)
        if mask.any():
            bins.append({
                "start_s": round(bin_start, 6),
                "end_s": round(bin_end, 6),
                "samples": int(mask.sum()),
                "mean_abs_xyz_m": np.mean(np.abs(error[mask]), axis=0).tolist(),
                "rmse_m": float(np.sqrt(np.mean(np.sum(error[mask] ** 2, axis=1)))),
            })
        bin_start += 30.0

    return {
        "frames": {
            "comparison": "rtabmap_map_frame",
            "rtabmap_up": "+Z",
            "roomscanner_input_up": "-Y",
            "error_definition": "aligned_roomscanner_minus_rtabmap",
        },
        "alignment": {
            "method": "rigid_umeyama_leading_window",
            "leading_window_s": float(alignment_window_s),
            "leading_samples": int(align_mask.sum()),
            "applied_scale": 1.0,
            "leading_window_similarity_scale_diagnostic": leading_scale,
            "full_overlap_similarity_scale_diagnostic": full_similarity_scale,
            "similarity_scale_is_physical_scale": False,
            "rotation_roomscanner_to_rtabmap": rotation.tolist(),
            "translation_roomscanner_to_rtabmap_m": translation.tolist(),
        },
        "distance": {
            "rtabmap_path_length_m": rtab_path,
            "roomscanner_path_length_m": room_path,
            "path_length_ratio_roomscanner_over_rtabmap": room_path / rtab_path,
            "path_length_ratio_is_physical_scale": True,
        },
        "rotation": {
            "rtabmap_total_rad": rtab_rotation_total,
            "roomscanner_total_rad": room_rotation_total,
            "ratio_roomscanner_over_rtabmap": room_rotation_total / rtab_rotation_total,
            "per_keyframe_speed_correlation": _correlation(
                rtab_rotation / np.diff(relative_t), room_rotation / np.diff(relative_t)),
        },
        "error": {
            "whole_overlap_rmse_m": float(np.sqrt(np.mean(np.sum(error ** 2, axis=1)))),
            "end_xyz_m": error[-1].tolist(),
            "end_norm_m": float(np.linalg.norm(error[-1])),
            "final_window_s": float(final_window_s),
            "final_window_start_s": final_start,
            "final_window_samples": int(final_mask.sum()),
            "final_window_mean_signed_xyz_m": np.mean(error[final_mask], axis=0).tolist(),
            "final_window_mean_abs_xyz_m": np.mean(np.abs(error[final_mask]), axis=0).tolist(),
            "time_bins": bins,
        },
    }


def compare_rtabmap_to_roomscan(
    rtabmap_db: str | Path,
    roomscan_tum: str | Path,
    *,
    alignment_window_s: float = 30.0,
    final_window_s: float = 60.0,
    clock_step_s: float = 0.02,
) -> dict:
    """Compare a phone RTAB-Map graph trajectory with roomscanner TUM output.

    Returns structured data and converts expected input/validation failures into
    ``{"ok": false, "error": ...}`` so both the CLI and MCP wrapper have the
    same non-traceback contract.
    """
    try:
        rtab: RtabmapTrajectory = read_optimized_trajectory(rtabmap_db)
        room = read_tum_trajectory(roomscan_tum)
        clock = align_clock_by_angular_speed(
            rtab.timestamps_s, rtab.poses[:, :3, :3],
            room.timestamps_s, room.rotations, search_step_s=clock_step_s)

        rtab_relative = rtab.timestamps_s - rtab.timestamps_s[0]
        room_relative = room.timestamps_s - room.timestamps_s[0]
        targets = rtab_relative + clock["roomscan_time_at_rtab_start_s"]
        if targets[0] < room_relative[0] or targets[-1] > room_relative[-1]:
            raise TrajectoryCompareError("clock match falls outside roomscanner coverage")
        matched_positions = np.column_stack([
            np.interp(targets, room_relative, room.positions_m[:, axis]) for axis in range(3)
        ])
        matched_indices = _nearest_indices(room_relative, targets)
        matched_rotations = room.rotations[matched_indices]
        comparison = compare_matched_trajectories(
            rtab.timestamps_s, rtab.poses, matched_positions, matched_rotations,
            alignment_window_s=alignment_window_s, final_window_s=final_window_s)

        rtab_duration = float(rtab_relative[-1])
        room_duration = float(room_relative[-1])
        return {
            "ok": True,
            "inputs": {
                "rtabmap_db": str(Path(rtabmap_db)),
                "roomscan_tum": str(Path(roomscan_tum)),
            },
            "clock_alignment": clock,
            "coverage": {
                "rtabmap_nodes": int(len(rtab.node_ids)),
                "rtabmap_duration_s": rtab_duration,
                "roomscanner_samples": int(len(room.timestamps_s)),
                "roomscanner_duration_s": room_duration,
                "rtabmap_start_in_roomscanner_s": float(targets[0]),
                "rtabmap_end_in_roomscanner_s": float(targets[-1]),
                "rtabmap_fraction_of_roomscanner_duration": rtab_duration / room_duration,
                "full_rtabmap_coverage": True,
            },
            **comparison,
        }
    except (OSError, RtabmapDatabaseError, TrajectoryCompareError, ValueError) as exc:
        return {
            "ok": False,
            "inputs": {
                "rtabmap_db": str(Path(rtabmap_db)),
                "roomscan_tum": str(Path(roomscan_tum)),
            },
            "error": str(exc),
        }
