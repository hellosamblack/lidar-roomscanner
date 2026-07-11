"""Tests for the stationarity gate (slam/motion.py): coherence math + the
hold decision that de-jitters a stationary sensor without eating real motion."""
import numpy as np
import pytest

from roomscan.slam.motion import coherence, StationarityGate


def test_coherence_straight_motion_is_one():
    """Consistent straight-line increments reinforce: coherence ~ 1.0."""
    inc = np.tile([0.02, 0.0, 0.0], (10, 1))
    assert coherence(inc) == pytest.approx(1.0)


def test_coherence_zero_mean_jitter_is_low():
    """Symmetric back-and-forth increments cancel: coherence ~ 0."""
    inc = np.array([[0.02, 0, 0], [-0.02, 0, 0]] * 8, dtype=float)
    assert coherence(inc) < 0.05


def test_coherence_empty_is_one_no_motion_is_zero():
    assert coherence(np.zeros((0, 3))) == 1.0        # no evidence -> "moving"
    assert coherence(np.zeros((5, 3))) == 0.0        # literally no motion


def test_gate_holds_on_incoherent_low_rotation_jitter():
    """Stationary case: small, directionally-random steps with ~no rotation ->
    the gate reports stationary once its window fills."""
    rng = np.random.default_rng(0)
    g = StationarityGate(window=10, coherence_thresh=0.5, step_ceiling_m=0.03,
                         rot_ceiling_deg=0.3)
    held = []
    for _ in range(30):
        step = rng.normal(0, 0.012, size=3)   # ~12 mm zero-mean jitter
        held.append(g.update(step, rot_delta_deg=0.05))
    assert not any(held[:9])          # never before the window fills
    assert sum(held[10:]) >= 15       # steadily holds once stationary is clear


def test_gate_passes_coherent_translation():
    """Real straight motion (coherent, larger steps) is never held even with
    zero rotation (walking forward while aiming straight ahead)."""
    g = StationarityGate(window=10, coherence_thresh=0.5, step_ceiling_m=0.03)
    held = [g.update([0.035, 0.0, 0.0], rot_delta_deg=0.0) for _ in range(30)]
    assert not any(held)


def test_gate_passes_when_rotating_even_if_translation_jitters():
    """Actively aiming the sensor (rotation above the ceiling) must never be
    classified stationary, even if the translation looks like jitter -- this
    is what stops a handheld scan's pauses from being frozen."""
    rng = np.random.default_rng(1)
    g = StationarityGate(window=10, coherence_thresh=0.5, step_ceiling_m=0.03,
                         rot_ceiling_deg=0.3)
    held = [g.update(rng.normal(0, 0.012, size=3), rot_delta_deg=1.0)
            for _ in range(30)]
    assert not any(held)


def test_gate_passes_large_incoherent_jumps():
    """Big incoherent jumps are tracking trouble, not stationarity -- the step
    ceiling keeps the gate from hiding them."""
    rng = np.random.default_rng(2)
    g = StationarityGate(window=10, coherence_thresh=0.5, step_ceiling_m=0.03)
    held = [g.update(rng.normal(0, 0.1, size=3), rot_delta_deg=0.0)
            for _ in range(30)]
    assert not any(held)


def test_gate_disabled_semantics_via_none():
    """A window that never fills (fed fewer than `window` samples) never
    holds -- mirrors how Mapper leaves the gate un-tripped at startup."""
    g = StationarityGate(window=10)
    assert not g.update([0.0, 0.0, 0.0], 0.0)
