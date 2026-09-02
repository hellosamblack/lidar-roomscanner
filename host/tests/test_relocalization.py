"""Tests for the relocalization candidate-search primitives (#112).

Spec: docs/superpowers/specs/2026-08-19-slam-relocalization-design.md
("Live relocalization v1 = re-anchor, not recognition") -- the SFLP
orientation is held through a kill, so the search widens only in
TRANSLATION, on horizontal (world X/Z) rings around a last-good anchor
pose. World frame is Open3D CV world (X=Right, Y=Down, Z=Forward,
world up = -Y) per docs/coordinate-frames.md and
`roomscan.slam.frames.world_up`.
"""
from __future__ import annotations

import numpy as np
import pytest

from roomscan.slam.frames import world_up
from roomscan.slam.relocalization import (
    RelocalizationOutcome,
    RelocalizationSearchConfig,
    RelocalizationState,
    RelocCandidate,
    candidate_ring,
    generate_candidates,
)


def _anchor_pose(x=1.0, y=-0.4, z=2.0, rot=None) -> np.ndarray:
    T = np.eye(4)
    if rot is not None:
        T[:3, :3] = rot
    T[:3, 3] = [x, y, z]
    return T


def _tilted_rotation() -> np.ndarray:
    # An arbitrary, non-identity rotation (15 deg about Y) so tests can't pass
    # by accident if a candidate silently reset rotation to identity.
    a = np.radians(15.0)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# --- candidate_ring -----------------------------------------------------

def test_candidate_ring_count_and_radius():
    anchor = _anchor_pose()
    ring = candidate_ring(anchor, radius_m=0.5, n_angles=8)
    assert len(ring) == 8
    for cand in ring:
        assert isinstance(cand, RelocCandidate)
        dist = float(np.linalg.norm(cand.pose[:3, 3] - anchor[:3, 3]))
        assert dist == pytest.approx(0.5, abs=1e-6)
        assert cand.radius_m == pytest.approx(0.5)


def test_candidate_ring_angles_evenly_spaced():
    anchor = _anchor_pose()
    ring = candidate_ring(anchor, radius_m=1.0, n_angles=4)
    angles = sorted(cand.angle_deg for cand in ring)
    assert angles == pytest.approx([0.0, 90.0, 180.0, 270.0], abs=1e-6)


def test_candidate_ring_zero_radius_is_single_point():
    anchor = _anchor_pose()
    ring = candidate_ring(anchor, radius_m=0.0, n_angles=8)
    # All 8 requested points collapse onto the anchor itself.
    for cand in ring:
        assert np.allclose(cand.pose[:3, 3], anchor[:3, 3], atol=1e-9)


def test_candidate_ring_holds_anchor_rotation():
    rot = _tilted_rotation()
    anchor = _anchor_pose(rot=rot)
    ring = candidate_ring(anchor, radius_m=0.4, n_angles=6)
    for cand in ring:
        assert np.allclose(cand.pose[:3, :3], rot, atol=1e-12)


def test_candidate_ring_holds_height_offset_is_horizontal():
    anchor = _anchor_pose(y=-0.7)
    ring = candidate_ring(anchor, radius_m=0.6, n_angles=12)
    up = world_up()
    for cand in ring:
        # Height (world-up component) unchanged from the anchor.
        assert cand.pose[1, 3] == pytest.approx(-0.7, abs=1e-9)
        offset = cand.pose[:3, 3] - anchor[:3, 3]
        # Regression guard for the X/Z horizontal-plane convention (world up
        # is -Y in this frame, docs/coordinate-frames.md): the ring offset
        # must be orthogonal to world-up, i.e. live entirely in X/Z.
        assert float(offset @ up) == pytest.approx(0.0, abs=1e-9)
        assert offset[1] == pytest.approx(0.0, abs=1e-9)


def test_candidate_ring_regression_wrong_axis_caught():
    """Prove test_candidate_ring_holds_height_offset_is_horizontal actually
    detects a wrong-axis defect (docs/engineering-practices.md: prove a
    regression test by reintroducing the defect). Build a "candidate" the way
    a Y/Z-swapped implementation would (varying height instead of Z) and
    confirm the same assertions this suite relies on would fail on it."""
    anchor = _anchor_pose(y=-0.7)
    up = world_up()
    radius = 0.6
    angle = np.radians(37.0)
    # Defective offset: puts the ring's "radius" into Y (height) and X
    # instead of X and Z -- the BUG-004-style axis swap this repo has hit
    # four times.
    bad_offset = np.array([radius * np.cos(angle), radius * np.sin(angle), 0.0])
    bad_pose = anchor.copy()
    bad_pose[:3, 3] = anchor[:3, 3] + bad_offset
    assert bad_pose[1, 3] != pytest.approx(-0.7, abs=1e-9)
    assert float(bad_offset @ up) != pytest.approx(0.0, abs=1e-9)


# --- generate_candidates -------------------------------------------------

def test_generate_candidates_starts_at_anchor():
    anchor = _anchor_pose()
    config = RelocalizationSearchConfig(radii_m=(0.0, 0.3, 0.6), angles_per_ring=6)
    candidates = generate_candidates(anchor, config)
    assert candidates[0].radius_m == pytest.approx(0.0)
    assert np.allclose(candidates[0].pose[:3, 3], anchor[:3, 3], atol=1e-9)


def test_generate_candidates_widens_in_ascending_radius_order():
    anchor = _anchor_pose()
    config = RelocalizationSearchConfig(radii_m=(0.2, 0.5, 1.0), angles_per_ring=4,
                                         min_separation_m=0.01, max_candidates=64)
    candidates = generate_candidates(anchor, config)
    radii_seen = [c.radius_m for c in candidates]
    assert radii_seen == sorted(radii_seen)
    assert min(radii_seen) == pytest.approx(0.2)
    assert max(radii_seen) == pytest.approx(1.0)


def test_generate_candidates_bounded_count():
    anchor = _anchor_pose()
    # Unbounded this would be 6 rings * 16 angles = 96 candidates.
    config = RelocalizationSearchConfig(
        radii_m=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
        angles_per_ring=16,
        min_separation_m=0.0,
        max_candidates=20,
    )
    candidates = generate_candidates(anchor, config)
    assert len(candidates) == 20


def test_generate_candidates_dedup_collapses_near_identical():
    anchor = _anchor_pose()
    # Two rings close enough in radius that, combined with a shared angle
    # grid, several points fall within min_separation_m of each other.
    config_dedup = RelocalizationSearchConfig(
        radii_m=(0.30, 0.31), angles_per_ring=8, min_separation_m=0.05,
        max_candidates=64,
    )
    config_raw = RelocalizationSearchConfig(
        radii_m=(0.30, 0.31), angles_per_ring=8, min_separation_m=0.0,
        max_candidates=64,
    )
    deduped = generate_candidates(anchor, config_dedup)
    raw = generate_candidates(anchor, config_raw)
    assert len(raw) == 16
    assert len(deduped) < len(raw)
    # No pair in the deduped output is closer than min_separation_m.
    pts = np.array([c.pose[:3, 3] for c in deduped])
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            assert np.linalg.norm(pts[i] - pts[j]) >= 0.05 - 1e-9


def test_generate_candidates_regression_dedup_removed_no_longer_shrinks():
    """Prove the dedup test above is load-bearing: with min_separation_m
    forced to 0 (the "defect" of disabling dedup) the raw and would-be-
    deduped counts must be equal -- i.e. removing dedup makes the assertion
    `len(deduped) < len(raw)` above false, which is what proves the test can
    see the defect it guards against."""
    anchor = _anchor_pose()
    config_no_dedup = RelocalizationSearchConfig(
        radii_m=(0.30, 0.31), angles_per_ring=8, min_separation_m=0.0,
        max_candidates=64,
    )
    candidates = generate_candidates(anchor, config_no_dedup)
    assert len(candidates) == 16  # dedup disabled -> nothing collapsed


# --- RelocalizationState --------------------------------------------------

def test_state_starts_idle():
    state = RelocalizationState()
    assert state.status == RelocalizationOutcome.IDLE
    assert state.next_candidate() is None


def test_state_begin_enters_searching_with_first_candidate():
    anchor = _anchor_pose()
    state = RelocalizationState(config=RelocalizationSearchConfig(radii_m=(0.0, 0.3), angles_per_ring=4))
    state.begin(anchor)
    assert state.status == RelocalizationOutcome.SEARCHING
    first = state.next_candidate()
    assert first is not None
    assert first.radius_m == pytest.approx(0.0)


def test_state_record_success_resets_pending_and_stores_pose():
    anchor = _anchor_pose()
    state = RelocalizationState(config=RelocalizationSearchConfig(radii_m=(0.0, 0.3), angles_per_ring=4))
    state.begin(anchor)
    recovered = _anchor_pose(x=1.05, z=2.0)
    state.record_success(recovered)
    assert state.status == RelocalizationOutcome.RELOCALIZED
    assert np.allclose(state.recovered_pose, recovered)
    assert state.next_candidate() is None  # bookkeeping cleared


def test_state_record_failure_advances_then_exhausts():
    anchor = _anchor_pose()
    config = RelocalizationSearchConfig(radii_m=(0.0, 0.3), angles_per_ring=2, min_separation_m=0.0)
    state = RelocalizationState(config=config)
    state.begin(anchor)
    total = len(generate_candidates(anchor, config))
    assert total >= 2
    seen_radii = []
    for _ in range(total):
        cand = state.next_candidate()
        assert cand is not None
        seen_radii.append(cand.radius_m)
        state.record_failure()
    assert state.status == RelocalizationOutcome.EXHAUSTED
    assert state.next_candidate() is None
    assert len(seen_radii) == total


def test_state_reset_returns_to_idle():
    anchor = _anchor_pose()
    state = RelocalizationState(config=RelocalizationSearchConfig(radii_m=(0.0,), angles_per_ring=1))
    state.begin(anchor)
    state.record_success(anchor)
    state.reset()
    assert state.status == RelocalizationOutcome.IDLE
    assert state.anchor_pose is None
    assert state.recovered_pose is None
    assert state.next_candidate() is None
    # Ready to begin a fresh search.
    state.begin(anchor)
    assert state.status == RelocalizationOutcome.SEARCHING
