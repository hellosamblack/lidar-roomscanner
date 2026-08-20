"""Read graph-optimized trajectories directly from an RTAB-Map database.

RTAB-Map stores ``Admin.opt_ids`` and ``Admin.opt_poses`` as independent raw-zlib
blobs.  The first expands to little-endian int32 node ids; the second expands to
one little-endian float32 3x4 rigid pose per id.  Node timestamps live separately
in ``Node.stamp`` and must be joined by id -- database row order is not an
association contract.

This small reader is shared infrastructure for trajectory comparison (#188) and
future RTAB-Map pose import (#159).  It deliberately reads only the optimized
trajectory, not images, depth, calibration, or graph links.
"""
from __future__ import annotations

import sqlite3
import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np


class RtabmapDatabaseError(ValueError):
    """An RTAB-Map database has no usable graph-optimized trajectory."""


@dataclass(frozen=True)
class RtabmapTrajectory:
    """Graph-optimized RTAB-Map node poses in database order."""

    node_ids: np.ndarray
    timestamps_s: np.ndarray
    poses: np.ndarray


def _inflate(blob: bytes | None, field: str, path: Path) -> bytes:
    if not blob:
        raise RtabmapDatabaseError(f"{path}: Admin.{field} is empty")
    try:
        return zlib.decompress(blob)
    except zlib.error as exc:
        raise RtabmapDatabaseError(
            f"{path}: Admin.{field} is not a raw-zlib blob: {exc}") from exc


def read_optimized_trajectory(path: str | Path) -> RtabmapTrajectory:
    """Decode ``Admin.opt_ids``/``opt_poses`` and join ``Node.stamp`` by id.

    Poses are returned as ``(N, 4, 4)`` camera/base-to-map matrices.  The 3x4
    storage layout is RTAB-Map's row-major ``Transform`` representation.  This
    function does not guess a Qt ``qCompress`` size prefix: the two Admin fields
    are raw zlib streams beginning with the normal ``0x78`` header.
    """
    db_path = Path(path)
    if not db_path.is_file():
        raise RtabmapDatabaseError(f"RTAB-Map database not found: {db_path}")

    try:
        uri = f"file:{db_path.resolve().as_posix()}?mode=ro"
        with sqlite3.connect(uri, uri=True) as db:
            row = db.execute("SELECT opt_ids, opt_poses FROM Admin").fetchone()
            if row is None:
                raise RtabmapDatabaseError(f"{db_path}: Admin has no row")
            ids_raw = _inflate(row[0], "opt_ids", db_path)
            poses_raw = _inflate(row[1], "opt_poses", db_path)

            if len(ids_raw) % 4:
                raise RtabmapDatabaseError(
                    f"{db_path}: Admin.opt_ids expands to {len(ids_raw)} bytes, "
                    "not a whole number of int32 ids")
            node_ids = np.frombuffer(ids_raw, dtype="<i4").astype(np.int64)
            if node_ids.size == 0:
                raise RtabmapDatabaseError(f"{db_path}: optimized trajectory has no node ids")
            if len(set(node_ids.tolist())) != len(node_ids):
                raise RtabmapDatabaseError(f"{db_path}: Admin.opt_ids contains duplicate node ids")

            expected_pose_bytes = len(node_ids) * 12 * 4
            if len(poses_raw) != expected_pose_bytes:
                raise RtabmapDatabaseError(
                    f"{db_path}: Admin.opt_poses expands to {len(poses_raw)} bytes; "
                    f"expected {expected_pose_bytes} for {len(node_ids)} 3x4 poses")
            stored = np.frombuffer(poses_raw, dtype="<f4").reshape(-1, 3, 4)

            stamp_by_id = {int(node_id): float(stamp) for node_id, stamp in
                           db.execute("SELECT id, stamp FROM Node")}
    except sqlite3.Error as exc:
        raise RtabmapDatabaseError(f"{db_path}: cannot read RTAB-Map trajectory: {exc}") from exc

    missing = [int(node_id) for node_id in node_ids if int(node_id) not in stamp_by_id]
    if missing:
        preview = ", ".join(str(x) for x in missing[:8])
        suffix = "..." if len(missing) > 8 else ""
        raise RtabmapDatabaseError(
            f"{db_path}: optimized node id(s) missing from Node: {preview}{suffix}")

    timestamps = np.asarray([stamp_by_id[int(i)] for i in node_ids], dtype=np.float64)
    if not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0):
        raise RtabmapDatabaseError(
            f"{db_path}: optimized node timestamps must be finite and strictly increasing")
    if not np.isfinite(stored).all():
        raise RtabmapDatabaseError(f"{db_path}: optimized poses contain non-finite values")

    poses = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], len(node_ids), axis=0)
    poses[:, :3, :] = stored
    determinants = np.linalg.det(poses[:, :3, :3])
    if np.any(np.abs(determinants - 1.0) > 0.05):
        raise RtabmapDatabaseError(
            f"{db_path}: optimized pose rotation determinant is not near +1")

    return RtabmapTrajectory(node_ids=node_ids, timestamps_s=timestamps, poses=poses)
