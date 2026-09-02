"""Relocalization candidate-search primitives (#112).

Spec: docs/superpowers/specs/2026-08-19-slam-relocalization-design.md, "Live
relocalization v1 = re-anchor, not recognition" -- on a detected tracking
kill, freeze integration, HOLD the SFLP orientation (trustworthy through a
kill; it is not ICP-derived), and run the existing escalating ICP against
the raycast model from a zero-velocity re-anchored pose with a **widening
translation search**. Consequently every candidate this module generates
shares the anchor pose's rotation and height -- only the horizontal (X/Z)
position is searched, on widening rings.

Frame: Open3D CV world (X=Right, Y=Down, Z=Forward; world up = **-Y**),
`docs/coordinate-frames.md` / `roomscan.slam.frames`. The horizontal plane a
ring lives in is therefore X/Z, never Y -- this repo has shipped that axis
swap four times (BUG-004, BUG-039, BUG-048/051/058), so every candidate's
offset is checked (in tests) to be orthogonal to `frames.world_up()` rather
than assumed. Nothing here derives a scalar "yaw": rotation is never
computed from an attitude, only carried through unchanged from the anchor.

This module is deliberately decoupled from `Mapper` -- pure functions and a
small state object, no ICP, no wire/service/remote coupling. See the module
docstring's "Integration hook" note at the bottom of this file for how the
next wave should wire it in.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

__all__ = [
    "RelocCandidate",
    "RelocalizationSearchConfig",
    "RelocalizationOutcome",
    "RelocalizationState",
    "candidate_ring",
    "generate_candidates",
]

# Open3D CV world basis vectors that span the horizontal plane (orthogonal to
# world_up() == [0, -1, 0]). Kept as plain constants rather than re-derived
# per call -- the whole point is these never vary with attitude.
_X_AXIS = np.array([1.0, 0.0, 0.0], dtype=np.float64)
_Z_AXIS = np.array([0.0, 0.0, 1.0], dtype=np.float64)


@dataclass(frozen=True)
class RelocCandidate:
    """One candidate re-anchor pose: the anchor's rotation and height,
    offset horizontally by `radius_m` at `angle_deg` (0 deg = +X, 90 deg =
    +Z, measured in the X/Z plane -- a plain geometric parametrization of
    the ring, not a heading/bearing off any attitude)."""
    pose: np.ndarray          # (4, 4) SE3
    radius_m: float
    angle_deg: float


@dataclass(frozen=True)
class RelocalizationSearchConfig:
    """Widening-ring search parameters. `radii_m` need not be sorted; rings
    are always emitted in ascending radius order (closest to the anchor,
    i.e. the zero-velocity re-anchor itself, tried first)."""
    radii_m: tuple[float, ...] = (0.0, 0.15, 0.3, 0.5, 0.75, 1.0)
    angles_per_ring: int = 8
    max_candidates: int = 32
    min_separation_m: float = 0.05


class RelocalizationOutcome(Enum):
    IDLE = "idle"                # no search in progress (normal tracking)
    SEARCHING = "searching"      # candidates pending, none yet confirmed
    RELOCALIZED = "relocalized"  # a candidate cleared the caller's ICP gate
    EXHAUSTED = "exhausted"      # every candidate failed the caller's gate


def candidate_ring(anchor_pose: np.ndarray, radius_m: float, n_angles: int) -> list[RelocCandidate]:
    """`n_angles` candidates evenly spaced around a horizontal ring of
    `radius_m` centred on `anchor_pose`'s position, holding its rotation and
    height fixed. `radius_m == 0` collapses every angle onto the anchor
    itself (still returns `n_angles` entries -- `generate_candidates`'s
    dedup is what collapses those to one)."""
    if n_angles < 1:
        raise ValueError(f"n_angles must be >= 1, got {n_angles}")
    anchor_pose = np.asarray(anchor_pose, dtype=np.float64)
    out: list[RelocCandidate] = []
    for k in range(n_angles):
        angle_deg = 360.0 * k / n_angles
        theta = np.radians(angle_deg)
        offset = radius_m * (np.cos(theta) * _X_AXIS + np.sin(theta) * _Z_AXIS)
        pose = anchor_pose.copy()
        pose[:3, 3] = anchor_pose[:3, 3] + offset
        out.append(RelocCandidate(pose=pose, radius_m=float(radius_m), angle_deg=float(angle_deg)))
    return out


def _dedup(candidates: list[RelocCandidate], min_separation_m: float) -> list[RelocCandidate]:
    """Drop any candidate whose position is within `min_separation_m` of one
    already kept (first-seen wins, so smaller radii -- earlier in the
    ascending-radius ordering `generate_candidates` builds -- survive over
    later duplicates). `min_separation_m <= 0` disables dedup entirely."""
    if min_separation_m <= 0.0:
        return list(candidates)
    kept: list[RelocCandidate] = []
    kept_pts: list[np.ndarray] = []
    for cand in candidates:
        pt = cand.pose[:3, 3]
        if any(np.linalg.norm(pt - p) < min_separation_m for p in kept_pts):
            continue
        kept.append(cand)
        kept_pts.append(pt)
    return kept


def generate_candidates(
    anchor_pose: np.ndarray,
    config: RelocalizationSearchConfig | None = None,
) -> list[RelocCandidate]:
    """The full widening-translation search: rings at each of `config.radii_m`
    (ascending), `config.angles_per_ring` candidates per ring, deduplicated
    by `config.min_separation_m`, truncated to `config.max_candidates`.

    Truncation keeps the smallest-radius candidates -- generation order is
    ascending radius, and a bounded-count search should exhaust the nearby
    rings before it would have reached farther ones anyway."""
    config = config or RelocalizationSearchConfig()
    raw: list[RelocCandidate] = []
    for radius in sorted(config.radii_m):
        raw.extend(candidate_ring(anchor_pose, radius, config.angles_per_ring))
    deduped = _dedup(raw, config.min_separation_m)
    return deduped[: config.max_candidates]


@dataclass
class RelocalizationState:
    """Session bookkeeping for one relocalization attempt: `begin()` on a
    detected kill, pull candidates via `next_candidate()` in order, report
    each try's outcome via `record_success`/`record_failure`. Terminal
    states (`RELOCALIZED`, `EXHAUSTED`) already clear pending candidates;
    call `reset()` to return to `IDLE` before the next `begin()`."""
    config: RelocalizationSearchConfig = field(default_factory=RelocalizationSearchConfig)
    status: RelocalizationOutcome = RelocalizationOutcome.IDLE
    anchor_pose: np.ndarray | None = None
    recovered_pose: np.ndarray | None = None
    tried: int = 0
    _pending: list[RelocCandidate] = field(default_factory=list, repr=False)

    def begin(self, anchor_pose: np.ndarray) -> None:
        """Start (or restart) a search around `anchor_pose`."""
        self.anchor_pose = np.asarray(anchor_pose, dtype=np.float64)
        self._pending = generate_candidates(self.anchor_pose, self.config)
        self.tried = 0
        self.recovered_pose = None
        self.status = RelocalizationOutcome.SEARCHING

    def next_candidate(self) -> RelocCandidate | None:
        """The next untried candidate, or None if not searching / exhausted."""
        if self.status != RelocalizationOutcome.SEARCHING or not self._pending:
            return None
        return self._pending[0]

    def record_success(self, pose: np.ndarray) -> None:
        """The caller's ICP gate accepted `pose` (from the current
        candidate, presumably refined by ICP). Resets pending bookkeeping
        -- the search is over -- and stores the recovered pose."""
        self.recovered_pose = np.asarray(pose, dtype=np.float64)
        self._pending = []
        self.tried = 0
        self.status = RelocalizationOutcome.RELOCALIZED

    def record_failure(self) -> None:
        """The current candidate failed the caller's ICP gate. Advances to
        the next candidate; transitions to EXHAUSTED (and clears
        bookkeeping) once none remain."""
        if self._pending:
            self._pending.pop(0)
            self.tried += 1
        if not self._pending:
            self.status = RelocalizationOutcome.EXHAUSTED
            self.tried = 0

    def reset(self) -> None:
        """Return to IDLE, ready for the next `begin()`."""
        self.status = RelocalizationOutcome.IDLE
        self.anchor_pose = None
        self.recovered_pose = None
        self.tried = 0
        self._pending = []


# --- Integration hook (not wired this wave) --------------------------------
#
# `Mapper` owns kill detection (`tracking_stats` died/freeze runs) and the
# escalating ICP registration (`odometry.register_escalating`) this module
# does not touch. The intended call site, once wired:
#
#   1. On detecting a kill, `Mapper` calls `RelocalizationState.begin(anchor)`
#      with the last-good pose (zero-velocity re-anchor: same position/
#      rotation as the last accepted frame) and the held SFLP orientation
#      already baked into that pose's rotation block.
#   2. While `state.status is SEARCHING`, `Mapper` pulls
#      `state.next_candidate()`, runs its existing escalating ICP
#      (`odometry.register_escalating`, mode="translation" so rotation stays
#      pinned to the candidate's -- i.e. the anchor's -- SFLP-held rotation)
#      against the candidate pose as the init, and checks the result against
#      the wrong-anchor rejection ratio the spec calls for (decision point 3:
#      post-optimization residual over per-link sigma).
#   3. A gate pass calls `state.record_success(icp_result.pose)`; a gate
#      fail calls `state.record_failure()`. `Mapper` resumes normal
#      integration from `state.recovered_pose` on RELOCALIZED, or falls back
#      to the pre-#112 behavior (operator walk-back) on EXHAUSTED.
#   4. `Mapper` calls `state.reset()` once it has consumed a terminal status,
#      so the next kill starts a fresh search.
