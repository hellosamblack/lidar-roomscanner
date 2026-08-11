"""Unit tests for the VRAM-sweep decision logic (roomscan.splat.vram) and the
SfM-probe reduce logic (tools.splat_sfm_probe).

The torch/gsplat measurement pass (`measure_peak_at_count`, `sweep_vram`'s loop)
needs a real CUDA GPU and is covered by an on-box integration run, not here -- the
same split as the trainer's tests. These cover the pure arithmetic a wrong answer
would ship silently: budget derivation, the fit metric, recommendation selection,
the capture-limited flags, and worst-view ranking. All CPU, no torch.
"""
import numpy as np

from roomscan.splat.vram import (_caveats, _density_flags, _derive_budget,
                                  _estimated_real_peak, _fit_metric_gib, _recommend,
                                  _worst_view_subset)


def _row(n, *, alloc=None, reserved=None, nvml=None, oom=False):
    r = {"n": n, "peak_alloc_gib": alloc, "peak_reserved_gib": reserved,
         "peak_nvml_gib": nvml, "oom": oom}
    r["fit_metric_gib"] = _fit_metric_gib(r)
    return r


# --- fit metric: never the optimistic allocated number -------------------------

def test_fit_metric_prefers_the_conservative_of_reserved_and_nvml():
    # reserved and device-wide NVML both available -> take the LARGER; allocated
    # (the optimistic undercount) is ignored when a conservative number exists.
    r = _row(1_000_000, alloc=1.6, reserved=1.95, nvml=2.4)
    assert _fit_metric_gib(r) == 2.4
    r2 = _row(1_000_000, alloc=1.6, reserved=2.9, nvml=2.4)
    assert _fit_metric_gib(r2) == 2.9


def test_fit_metric_falls_back_to_allocated_only_when_nothing_better_exists():
    # NVML absent (device-less budget path) and reserved somehow missing -> the
    # allocated number is all we have; better a weak metric than None.
    r = _row(1_000_000, alloc=1.6, reserved=None, nvml=None)
    assert _fit_metric_gib(r) == 1.6


def test_fit_metric_is_none_on_oom():
    r = _row(3_000_000, oom=True)
    assert _fit_metric_gib(r) is None


# --- budget derivation ---------------------------------------------------------

def test_budget_cli_override_wins():
    assert _derive_budget(6.5, 7.99, 0.8, 0.0, nvml_ok=True) == (6.5, "cli")


def test_budget_from_nvml_total_minus_margin_and_reserve():
    budget, source = _derive_budget(None, 7.99, 0.8, 1.7, nvml_ok=True)
    assert budget == round(7.99 - 0.8 - 1.7, 2)
    assert source == "nvml-total-margin"


def test_budget_source_names_the_fallback_when_nvml_absent():
    _, source = _derive_budget(None, 8.0, 0.8, 0.0, nvml_ok=False)
    assert source == "torch-total-margin"


# --- recommendation: largest fitting count, monotone-safe ----------------------

def test_recommend_picks_largest_count_within_budget():
    rows = [_row(500_000, reserved=1.9), _row(1_000_000, reserved=3.0),
            _row(1_500_000, reserved=6.5), _row(2_000_000, reserved=7.8)]
    cap, peak = _recommend(rows, budget=7.0)   # safety_factor default 1.0
    assert cap == 1_500_000 and peak == 6.5


def test_recommend_excludes_oom_and_over_budget_rows():
    rows = [_row(1_000_000, reserved=3.0), _row(2_000_000, reserved=7.9),
            _row(2_500_000, oom=True)]
    cap, _ = _recommend(rows, budget=7.0)
    assert cap == 1_000_000


def test_recommend_is_none_when_even_the_smallest_fails():
    rows = [_row(500_000, reserved=8.5), _row(1_000_000, oom=True)]
    assert _recommend(rows, budget=7.0) == (None, None)


def test_safety_factor_shrinks_the_recommended_cap():
    # The clone under-measures the real training peak ~2x, so with safety_factor=2
    # a rung measuring 3.6 GiB is treated as ~7.2 real and rejected against a 7.0 budget.
    rows = [_row(1_000_000, reserved=3.0), _row(1_500_000, reserved=3.6),
            _row(2_000_000, reserved=4.5)]
    assert _recommend(rows, budget=7.0, safety_factor=1.0)[0] == 2_000_000  # raw: all fit
    cap, peak = _recommend(rows, budget=7.0, safety_factor=2.0)
    assert cap == 1_000_000 and peak == 6.0    # 3.0 x 2 = 6.0 <= 7.0; 3.6 x 2 = 7.2 > 7.0


def test_estimated_real_peak_scales_the_measured_metric():
    assert _estimated_real_peak(_row(1_000_000, reserved=2.4), 2.0) == 4.8
    assert _estimated_real_peak(_row(3_000_000, oom=True), 2.0) is None


# --- capture-limited flags -----------------------------------------------------

def test_density_flags_marks_capture_limited_below_half_registered():
    capture_limited, _ = _density_flags(40_000, 1_500_000, registered_ratio=0.16)
    assert capture_limited is True


def test_density_flags_not_capture_limited_when_well_registered():
    capture_limited, _ = _density_flags(40_000, 1_500_000, registered_ratio=0.72)
    assert capture_limited is False


def test_effective_ceiling_caps_the_recommendation_at_what_the_seed_can_grow_to():
    # A sparse seed (10k points) cannot be densified past ~20x, so the effective
    # ceiling is the seed bound, not the (larger) VRAM cap.
    _, ceiling = _density_flags(10_000, 1_500_000, registered_ratio=0.5)
    assert ceiling == 200_000
    # A rich seed -> the VRAM cap is the binding number.
    _, ceiling2 = _density_flags(500_000, 1_500_000, registered_ratio=0.9)
    assert ceiling2 == 1_500_000


def test_effective_ceiling_is_none_without_a_recommendation():
    _, ceiling = _density_flags(40_000, None, registered_ratio=0.5)
    assert ceiling is None


# --- caveats -------------------------------------------------------------------

def test_caveats_warn_about_the_worst_frame_the_fit_metric_and_the_lower_bound():
    text = " ".join(_caveats(capture_limited=False, safety_factor=2.0)).lower()
    assert "worst-case frame" in text
    assert "max_memory_allocated" in text
    assert "lower bound" in text          # the clone under-measures the real training peak
    assert "safety_factor=2.0" in text


def test_caveats_add_the_capture_limited_line_only_when_flagged():
    assert not any("capture-limited" in c.lower() for c in _caveats(False, 2.0))
    assert any("capture-limited" in c.lower() for c in _caveats(True, 2.0))


# --- worst-view subset ---------------------------------------------------------

def _view_at(x):
    """A view whose camera center sits at (x,0,0): viewmat is world->cam, so its
    inverse's translation is the camera center."""
    c2w = np.eye(4, dtype=np.float32)
    c2w[:3, 3] = [x, 0.0, 0.0]
    return {"viewmat": np.linalg.inv(c2w)}


def test_worst_view_subset_returns_the_k_nearest_the_scene_centroid():
    # Centers at -10, 0, 1, 12 -> centroid 0.75; nearest two are 1 and 0.
    views = [_view_at(-10), _view_at(0), _view_at(1), _view_at(12)]
    picked = _worst_view_subset(views, k=2)
    xs = sorted(round(float(np.linalg.inv(v["viewmat"])[0, 3])) for v in picked)
    assert xs == [0, 1]


def test_worst_view_subset_k_zero_returns_empty():
    views = [_view_at(0), _view_at(5)]
    assert _worst_view_subset(views, k=0) == []
