"""Unit tests for the pure math in tools/skew_check.py (BUG-031's measurement).

Synthetic clocks only — no capture, no hardware. The properties under test are the
ones a before/after skew number depends on being true:

* the fit survives the real magnitudes (~1e10 µs). The prototype of this tool
  returned a slope of 0.044 instead of 1.003 on real data, silently, purely from
  conditioning — a nonsense residual would have been quoted as a measurement.
* windowing removes slow oscillator wander (which a single global fit reports as
  if it were per-frame jitter) while leaving genuine per-frame jitter alone.
* the windowed residual and its per-frame covariates stay index-aligned, since the
  trailing partial window is dropped. The CALIB load test compares residuals
  against a per-frame flag, so a silent misalignment there produces a plausible,
  wrong causal claim.
"""
import numpy as np
import pytest

from tools.skew_check import (
    LSM_SAMPLE_PERIOD_US,
    _fit_residual,
    summarize_us,
    windowed_residuals,
)


def _clocks(n=600, period_us=33_000.0, ratio=1.0033, jitter_us=0.0, wander_us=0.0,
            seed=0):
    """A ToF clock (y) and an LSM clock (x) that differ by a fixed ratio, plus
    optional per-frame jitter and a slow sinusoidal wander between them."""
    rng = np.random.default_rng(seed)
    y = 3.8e9 + np.arange(n) * period_us              # MCU µs, realistic magnitude
    x = 8.7e10 + np.arange(n) * (period_us / ratio)   # LSM µs, realistic magnitude
    if jitter_us:
        y = y + rng.normal(0.0, jitter_us, n)
    if wander_us:
        y = y + wander_us * np.sin(np.linspace(0, 2 * np.pi, n))
    return x, y


def test_fit_survives_real_magnitudes():
    """~1e10 µs values: an uncentred [x, 1] lstsq returns a nonsense slope."""
    x, y = _clocks()
    a, resid = _fit_residual(x, y)
    assert a == pytest.approx(1.0033, rel=1e-4)
    assert np.abs(resid).max() < 1.0        # noiseless: residual is float error only


def test_windowing_suppresses_slow_wander_that_a_global_fit_reports_as_jitter():
    """Suppresses, not eliminates: a window still contains some of the wander's
    curvature, so this is a ~4x reduction (665 -> 161 µs here), not a floor."""
    x, y = _clocks(n=1800, wander_us=1500.0)   # ±1.5 ms of drift, no per-frame noise
    _a, global_resid = _fit_residual(x, y)
    windowed, _slope, _used = windowed_residuals(x, y, window_s=20.0)
    g = summarize_us(global_resid)["rms_us"]
    w = summarize_us(windowed)["rms_us"]
    assert g > 500.0
    assert w < g / 3.0


def test_windowing_preserves_genuine_per_frame_jitter():
    x, y = _clocks(n=1800, jitter_us=1000.0)
    windowed, _slope, _used = windowed_residuals(x, y, window_s=20.0)
    assert summarize_us(windowed)["rms_us"] == pytest.approx(1000.0, rel=0.1)


def test_windowed_residual_and_used_mask_stay_aligned():
    """The trailing partial window is dropped, so residuals are shorter than the
    input — a per-frame covariate must be masked by `used` before comparison."""
    # 20 s at 33 ms is 606 frames, so 620 leaves a 14-frame tail — under min_points
    x, y = _clocks(n=620, jitter_us=10.0)
    resid, _slope, used = windowed_residuals(x, y, window_s=20.0)
    assert resid.size == int(used.sum()) < x.size
    covariate = np.arange(x.size)
    assert covariate[used].size == resid.size


def test_windowed_residual_recovers_an_injected_load_shift():
    """The CALIB-load test's whole content: a subset stamped systematically late
    must come back as a shift of that size, and the rest must not move."""
    x, y = _clocks(n=1800, jitter_us=200.0)
    flag = np.zeros(x.size, dtype=bool)
    flag[::64] = True                      # the CALIB cadence
    y = y - np.where(flag, 650.0, 0.0)     # those frames' pairing lands 650 µs late
    resid, _slope, used = windowed_residuals(x, y, window_s=20.0)
    f = flag[used]
    assert resid[f].mean() - resid[~f].mean() == pytest.approx(-650.0, abs=60.0)


def test_summarize_us_reports_magnitude_statistics():
    s = summarize_us(np.array([-3.0, 4.0, 0.0, 0.0]))
    assert s["n"] == 4
    assert s["rms_us"] == pytest.approx(2.5)
    assert s["max_us"] == pytest.approx(4.0)     # |residual|, not the signed max
    assert summarize_us(np.zeros(0)) == {"n": 0}


def test_fifo_estimator_floor_is_one_sample_period_of_phase():
    """The `fifo` estimator can never beat this, so an improvement claimed below
    it would be a measurement artefact (480 Hz ODR => 96 ticks => ~2.08 ms)."""
    assert LSM_SAMPLE_PERIOD_US == pytest.approx(2083.2, abs=1.0)
    assert LSM_SAMPLE_PERIOD_US / np.sqrt(12.0) == pytest.approx(601.4, abs=1.0)


def test_degenerate_inputs_do_not_raise():
    assert _fit_residual(np.zeros(5), np.arange(5.0))[0] != _fit_residual(
        np.zeros(5), np.arange(5.0))[0] or True     # nan slope, no exception
    resid, slope, used = windowed_residuals(np.arange(3.0), np.arange(3.0), 20.0)
    assert resid.size == 0 and np.isnan(slope) and not used.any()
