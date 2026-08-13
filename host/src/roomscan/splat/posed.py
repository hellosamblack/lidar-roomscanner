"""Source-agnostic posed-capture contract (issue #158).

A `PosedCapture` is what any capture *source* (an RTAB-Map export today --
`roomscan.splat.rtabmap`; a future fused ToF+phone rig capture later) normalizes into
before it reaches `roomscan.splat.train`. It holds **paths and metadata, not decoded
image/depth arrays**, so a multi-hundred-frame capture is cheap to inspect and does not
duplicate image memory before CUDA training even begins.

Canonical camera-pose contract (binding -- matches `train.py::_load_views()`'s existing
``viewmat``): ``PosedFrame.pose_camera_from_world`` is a 4x4 **world-to-camera** matrix
(``camera_from_world``: ``x_cam = R @ x_world + t``), the same direction pycolmap's
``img.cam_from_world().matrix()`` returns. Camera axes are the standard pinhole/optical
convention (x-right, y-down, z-forward). Wiring this representation into training/COLMAP
seeding is deliberately deferred to #159 -- this module only defines the contract.

No torch/gsplat/pycolmap import here, or anywhere else this module touches -- it must stay
importable without the training stack.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# Exported poses are float32-ish by the time they reach us (JPEG/PNG-adjacent metadata,
# printf `%f` round-tripping); allow a little slack past exact float64 orthonormality.
_ROT_ORTHONORMAL_ATOL = 1e-3


class PosedFrameError(ValueError):
    """A `PosedFrame`/`PosedCapture` failed validation."""


@dataclass(frozen=True)
class PosedFrame:
    """One posed RGB(+D) observation. Holds paths + metadata, never decoded pixels.

    Attributes:
        frame_id: stable identifier from the *source* (e.g. an RTAB-Map export's node stamp).
        image_path: path to the RGB image.
        width, height: this frame's own image dimensions.
        k: this frame's own 3x3 intrinsic matrix (never coalesced across frames --
            autofocus/exposure changes intrinsics within a single capture).
        pose_camera_from_world: 4x4 world-to-camera matrix. See module docstring for the
            canonical direction/axis contract.
        depth_path, confidence_path: optional registered-depth / depth-confidence images,
            in whatever native units/encoding the source emits (never rescaled here).
        timestamp: source capture time, only set when the source actually carries one.
        timestamp_domain: required alongside `timestamp` -- names the clock/field it came
            from (e.g. ``"rtabmap_export_stamp_s"``), never invented from an id or filename.
    """

    frame_id: str
    image_path: Path
    width: int
    height: int
    k: np.ndarray
    pose_camera_from_world: np.ndarray
    depth_path: Path | None = None
    confidence_path: Path | None = None
    timestamp: float | None = None
    timestamp_domain: str | None = None

    def __post_init__(self):
        _validate_frame(self)

    def camera_center_world(self) -> np.ndarray:
        """World-frame position of the camera (``-R^T t`` of `pose_camera_from_world`)."""
        R = self.pose_camera_from_world[:3, :3]
        t = self.pose_camera_from_world[:3, 3]
        return -R.T @ t

    def camera_basis_world(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(right, down, forward) camera axes, expressed as world-frame unit vectors.

        These are the columns of ``inverse(pose_camera_from_world)``'s rotation block --
        i.e. the world-frame directions that map to the camera's local +x/+y/+z under
        `pose_camera_from_world`. Separate from `camera_center_world` so a transpose/
        inverse pose-direction bug shows up even on a zero-translation frame.
        """
        R = self.pose_camera_from_world[:3, :3]
        return R[0, :].copy(), R[1, :].copy(), R[2, :].copy()

    def to_trainer_view(self) -> dict:
        """Narrow conversion to `train.py::_load_views()`'s existing view-dict contract."""
        return {"K": np.asarray(self.k, dtype=np.float32),
                "w": int(self.width), "h": int(self.height),
                "viewmat": np.asarray(self.pose_camera_from_world, dtype=np.float32),
                "image_path": str(self.image_path)}


@dataclass(frozen=True)
class PosedCapture:
    """An ordered, posed image capture from one source (e.g. one RTAB-Map export)."""

    source: str
    frames: tuple[PosedFrame, ...]
    world_frame: str
    geometry_paths: tuple[Path, ...] = field(default_factory=tuple)
    geometry_frame: str | None = None

    def __post_init__(self):
        if not self.frames:
            raise PosedFrameError("PosedCapture has no frames")
        ids = [f.frame_id for f in self.frames]
        dupes = sorted({x for x in ids if ids.count(x) > 1})
        if dupes:
            raise PosedFrameError(f"duplicate frame_id(s) in PosedCapture: {dupes}")
        if self.geometry_paths and not self.geometry_frame:
            raise PosedFrameError("geometry_paths given without geometry_frame")


def _validate_frame(f: PosedFrame) -> None:
    if f.width <= 0 or f.height <= 0:
        raise PosedFrameError(f"{f.frame_id}: non-positive image dimensions ({f.width}x{f.height})")

    k = np.asarray(f.k, dtype=float)
    if k.shape != (3, 3):
        raise PosedFrameError(f"{f.frame_id}: K must be 3x3, got shape {k.shape}")
    if not np.all(np.isfinite(k)):
        raise PosedFrameError(f"{f.frame_id}: K has non-finite entries")
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise PosedFrameError(f"{f.frame_id}: non-positive focal length (fx={k[0, 0]}, fy={k[1, 1]})")

    pose = np.asarray(f.pose_camera_from_world, dtype=float)
    if pose.shape != (4, 4):
        raise PosedFrameError(f"{f.frame_id}: pose_camera_from_world must be 4x4, got shape {pose.shape}")
    if not np.all(np.isfinite(pose)):
        raise PosedFrameError(f"{f.frame_id}: pose_camera_from_world has non-finite entries")
    if not np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise PosedFrameError(f"{f.frame_id}: pose_camera_from_world last row must be "
                              f"[0,0,0,1], got {pose[3].tolist()}")

    R = pose[:3, :3]
    det = float(np.linalg.det(R))
    if abs(det - 1.0) > _ROT_ORTHONORMAL_ATOL:
        raise PosedFrameError(f"{f.frame_id}: pose rotation block is not a proper rotation "
                              f"(det={det:.6f}, want ~1.0)")
    if not np.allclose(R @ R.T, np.eye(3), atol=_ROT_ORTHONORMAL_ATOL):
        raise PosedFrameError(f"{f.frame_id}: pose rotation block is not orthonormal")

    if (f.timestamp is None) != (f.timestamp_domain is None):
        raise PosedFrameError(f"{f.frame_id}: timestamp and timestamp_domain must both be "
                              f"set or both unset (timestamp={f.timestamp!r}, "
                              f"timestamp_domain={f.timestamp_domain!r})")
