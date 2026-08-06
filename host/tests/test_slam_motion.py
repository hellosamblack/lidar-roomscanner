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


# ---- BUG-069: accelerometer ZUPT (a map-reaching, pan-capable zero-velocity gate)

from roomscan.slam.motion import ZuptDetector, _G


def test_zupt_holds_on_one_g_specific_force():
    """|a| ~= 1 g for `window` consecutive frames -> zero-velocity verdict."""
    z = ZuptDetector(window=4, accel_tol_g=0.04)
    verdicts = [z.update(_G) for _ in range(5)]
    assert verdicts == [False, False, False, True, True]   # trips once the window fills


def test_zupt_fires_during_a_pan_unlike_the_coherence_gate():
    """A pure pan (rotation, zero translation) still reads 1 g of specific force,
    so the ZUPT holds -- the exact case StationarityGate's rot ceiling excludes."""
    z = ZuptDetector(window=3, accel_tol_g=0.04)
    # gravity magnitude is orientation-invariant, so a rotating sensor at rest
    # still reads ~1 g; feed 1 g repeatedly (the accel norm the pan produces)
    for _ in range(3):
        z.update(_G)
    assert z.update(_G) is True


def test_zupt_releases_when_specific_force_leaves_the_band():
    z = ZuptDetector(window=3, accel_tol_g=0.04)
    for _ in range(4):
        z.update(_G)
    assert z.update(_G) is True
    # a footfall / real linear accel pushes |a| well off 1 g -> released immediately
    assert z.update(_G * 1.5) is False


def test_zupt_none_signal_never_holds_and_clears():
    z = ZuptDetector(window=2, accel_tol_g=0.04)
    z.update(_G)
    assert z.update(None) is False        # missing signal -> never hold
    assert z.update(_G) is False          # and it cleared the streak


def test_zupt_transient_does_not_latch():
    """A single still frame amid motion must not trip a window>1 gate."""
    z = ZuptDetector(window=3, accel_tol_g=0.02)
    z.update(_G * 1.3)
    z.update(_G)                          # one still frame
    assert z.update(_G * 1.3) is False    # still moving overall


def test_zupt_coherence_veto_rejects_a_directed_walk():
    """Accel says 1 g but the ICP increments are directionally coherent (a walk):
    the veto must refuse to hold, even with the accel window full."""
    z = ZuptDetector(window=4, accel_tol_g=0.04, coherence_thresh=0.5)
    for _ in range(6):
        held = z.update(_G, increment=[0.02, 0.0, 0.0])   # 1 g + steady forward motion
    assert held is False


def test_zupt_coherence_veto_still_holds_incoherent_jitter():
    """Accel says 1 g and the ICP increments cancel (stationary jitter): hold."""
    z = ZuptDetector(window=4, accel_tol_g=0.04, coherence_thresh=0.5)
    jitter = [[0.01, 0, 0], [-0.01, 0, 0]] * 4
    held = False
    for inc in jitter:
        held = z.update(_G, increment=inc)
    assert held is True


def test_zupt_coherence_veto_disabled_is_accel_only():
    z = ZuptDetector(window=3, accel_tol_g=0.04, coherence_thresh=0.0)
    for _ in range(4):
        held = z.update(_G, increment=[0.02, 0.0, 0.0])   # coherent, but veto off
    assert held is True
