"""Unit tests for the pure math in tools/orientation_probe.py.

Synthetic data only — no server, no hardware. The properties under test are
the ones the measurement's validity rests on: the mean-direction metric is
range-invariant and membership-robust, angles come out in degrees, the
coherence statistic separates real motion (~1.0) from zero-mean jitter
(~1/sqrt(window)) from anti-correlated dither (below that), and the health
summary reduces a window of `metrics` messages to rates + counter deltas.
"""
import struct

import numpy as np
import pytest

from tools.orientation_probe import (
    DEFAULT_WINDOW,
    FIRMWARE_BASELINE_DEG,
    METRIC_FLOOR_DEG,
    TAG_POINT_CLOUD,
    coherence_series,
    format_health_summary_line,
    format_jitter_summary_line,
    frame_angles_deg,
    mean_ray_direction,
    summarize_health,
    summarize_jitter,
)


def _payload(points):
    """A POINT_CLOUD /ws message: u32 tag + f32[3N] positions + f32[3N] colors."""
    pts = np.asarray(points, dtype="<f4").reshape(-1, 3)
    colors = np.zeros_like(pts)
    return struct.pack("<I", TAG_POINT_CLOUD) + pts.tobytes() + colors.tobytes()


def _rot_x(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


# --- mean_ray_direction -----------------------------------------------------

def test_mean_direction_points_along_common_ray():
    pts = [[0, 0, 0.5], [0, 0, 1.0], [0, 0, 3.0]]
    d = mean_ray_direction(_payload(pts))
    assert np.allclose(d, [0, 0, 1], atol=1e-12)
    assert np.isclose(np.linalg.norm(d), 1.0)


def test_mean_direction_is_range_invariant():
    """Scaling every range leaves the direction untouched — it is a pure
    orientation metric, immune to depth noise."""
    rng = np.random.default_rng(0)
    rays = rng.normal(size=(200, 3)) + [0, 0, 5]
    d1 = mean_ray_direction(_payload(rays))
    d2 = mean_ray_direction(_payload(rays * rng.uniform(0.5, 3.0, (200, 1))))
    assert np.allclose(d1, d2, atol=1e-6)


def test_mean_direction_is_membership_permutation_invariant():
    rng = np.random.default_rng(1)
    rays = rng.normal(size=(500, 3)) + [0, 0, 5]
    d1 = mean_ray_direction(_payload(rays))
    d2 = mean_ray_direction(_payload(rays[rng.permutation(500)]))
    assert np.allclose(d1, d2, atol=1e-9)


def test_mean_direction_rotates_with_the_cloud():
    rng = np.random.default_rng(2)
    rays = rng.normal(size=(300, 3)) + [0, 0, 5]
    r = _rot_x(0.5)
    d0 = mean_ray_direction(_payload(rays))
    d1 = mean_ray_direction(_payload(rays @ r.T))
    ang = np.degrees(np.arccos(np.clip(d0 @ d1, -1, 1)))
    assert ang == pytest.approx(0.5, abs=1e-3)


def test_mean_direction_degenerate_clouds():
    assert mean_ray_direction(struct.pack("<I", TAG_POINT_CLOUD)) is None  # empty
    assert mean_ray_direction(_payload([[0, 0, 0]])) is None               # origin only
    # Origin points are dropped, not averaged in.
    d = mean_ray_direction(_payload([[0, 0, 0], [0, 0, 2.0]]))
    assert np.allclose(d, [0, 0, 1], atol=1e-12)


# --- frame_angles_deg -------------------------------------------------------

def test_frame_angles_known_rotation_steps():
    step = 0.25
    d0 = np.array([0.0, 0.0, 1.0])
    dirs = [np.linalg.matrix_power(_rot_x(step), k) @ d0 for k in range(6)]
    a = frame_angles_deg(dirs)
    assert a.shape == (5,)
    assert np.allclose(a, step, atol=1e-9)


def test_frame_angles_stationary_is_zero():
    dirs = [np.array([0.0, 0.0, 1.0])] * 4
    assert np.allclose(frame_angles_deg(dirs), 0.0)


# --- coherence_series -------------------------------------------------------

def test_coherence_consistent_motion_near_one():
    d0 = np.array([0.0, 0.0, 1.0])
    dirs = [np.linalg.matrix_power(_rot_x(0.1), k) @ d0 for k in range(30)]
    c = coherence_series(dirs, window=DEFAULT_WINDOW)
    assert len(c) == 30 - 1 - DEFAULT_WINDOW
    assert c.min() > 0.99


def test_coherence_white_jitter_near_inv_sqrt_window():
    """Independent random *increments* (a directional random walk) read near
    1/sqrt(window) (~0.32 at 10) — the white-noise reference quoted in the
    tool's output. (Independent perturbations about a fixed truth are NOT
    this case: their increments are anti-correlated and read lower, like the
    dither test below.)"""
    rng = np.random.default_rng(3)
    d = np.array([0.0, 0.0, 1.0])
    dirs = [d]
    for _ in range(400):
        d = d + rng.normal(scale=1e-4, size=3)
        d = d / np.linalg.norm(d)
        dirs.append(d)
    c = coherence_series(dirs, window=DEFAULT_WINDOW)
    ref = 1.0 / np.sqrt(DEFAULT_WINDOW)
    assert c.mean() == pytest.approx(ref, abs=0.12)


def test_coherence_anticorrelated_dither_below_white_noise():
    """A direction toggling between two values (quantization dither) has
    increments that cancel pairwise — coherence far below 1/sqrt(window).
    This signature is how the SFLP fp16 floor was identified."""
    a = np.array([0.0, 0.0, 1.0])
    b = _rot_x(0.05) @ a
    dirs = [a, b] * 20
    c = coherence_series(dirs, window=DEFAULT_WINDOW)
    assert c.max() < 1.0 / np.sqrt(DEFAULT_WINDOW) / 2


def test_coherence_too_few_frames_is_empty():
    dirs = [np.array([0.0, 0.0, 1.0])] * (DEFAULT_WINDOW + 1)
    assert coherence_series(dirs, window=DEFAULT_WINDOW).size == 0


# --- summarize_jitter / jitter summary line ---------------------------------

def test_summarize_jitter_stats_match_known_steps():
    d0 = np.array([0.0, 0.0, 1.0])
    dirs = [np.linalg.matrix_power(_rot_x(0.2), k) @ d0 for k in range(25)]
    s = summarize_jitter(dirs, window=DEFAULT_WINDOW, seconds=15.0, label="synthetic")
    assert s["frames"] == 25
    assert s["mean_deg"] == pytest.approx(0.2, abs=1e-6)
    assert s["median_deg"] == pytest.approx(0.2, abs=1e-6)
    assert s["p95_deg"] == pytest.approx(0.2, abs=1e-6)
    # Not exactly 1.0: consecutive great-circle chords are curved, not collinear.
    assert s["coherence_mean"] == pytest.approx(1.0, abs=1e-3)
    # Context constants ride along so a logged summary is self-describing.
    assert s["metric_floor_deg"] == METRIC_FLOOR_DEG
    assert s["firmware_baseline_deg"] == FIRMWARE_BASELINE_DEG
    # Edge motion: 0.2 deg at 3 m lever arm.
    assert s["edge_mm_at_3m_mean"] == pytest.approx(np.radians(0.2) * 3000, rel=1e-6)


def test_summarize_jitter_short_run_has_null_coherence():
    d0 = np.array([0.0, 0.0, 1.0])
    dirs = [d0, _rot_x(0.1) @ d0, _rot_x(0.2) @ d0]
    s = summarize_jitter(dirs, window=DEFAULT_WINDOW, seconds=1.0)
    assert s["coherence_mean"] is None


def test_format_jitter_summary_line_machine_parseable():
    d0 = np.array([0.0, 0.0, 1.0])
    dirs = [np.linalg.matrix_power(_rot_x(0.2), k) @ d0 for k in range(25)]
    line = format_jitter_summary_line(summarize_jitter(dirs, seconds=15.0, label="run1"))
    assert line.startswith("JITTER_SUMMARY ")
    kv = dict(tok.split("=", 1) for tok in line.split()[1:])
    assert kv["label"] == "run1"
    assert int(kv["frames"]) == 25
    assert float(kv["mean_deg"]) == pytest.approx(0.2, abs=1e-4)
    assert float(kv["coherence_mean"]) == pytest.approx(1.0, abs=1e-4)


# --- summarize_health / health summary line ---------------------------------

def _metrics_msg(drops=0, gaps=0, render_fps=30.0,
                  streams=((7, "RAW", 27.9, 27.8),
                            (9, "IMU_QUAT", 30.3, 30.2),
                            (10, "ENV", None, 30.1))):
    return {
        "type": "metrics",
        "render_fps": render_fps,
        "drops": drops,
        "gaps": gaps,
        "streams": [
            {"stream_id": sid, "label": label, "device_hz": dev, "host_hz": host,
             "bytes_per_s": 1000, "jitter_ms": None}
            for sid, label, dev, host in streams
        ],
    }


def test_summarize_health_uses_last_message_rates():
    msgs = [_metrics_msg(streams=((7, "RAW", 10.0, 10.0),)),
            _metrics_msg(streams=((7, "RAW", 27.9, 27.8), (9, "IMU_QUAT", 30.3, 30.2)))]
    s = summarize_health(msgs, seconds=12.0)
    assert s["metrics_msgs"] == 2
    assert sorted(s["streams"]) == [7, 9]
    assert s["streams"][7]["host_hz"] == 27.8
    assert s["streams"][9]["label"] == "IMU_QUAT"


def test_summarize_health_counter_deltas_across_window():
    msgs = [_metrics_msg(drops=5, gaps=2), _metrics_msg(drops=5, gaps=2),
            _metrics_msg(drops=7, gaps=2)]
    s = summarize_health(msgs, seconds=12.0)
    assert s["drops"] == 7 and s["drops_delta"] == 2
    assert s["gaps"] == 2 and s["gaps_delta"] == 0


def test_summarize_health_single_message_delta_is_zero():
    s = summarize_health([_metrics_msg(drops=3, gaps=1)])
    assert s["drops_delta"] == 0 and s["gaps_delta"] == 0


def test_summarize_health_tolerates_missing_counters():
    m = _metrics_msg()
    del m["drops"], m["gaps"]
    s = summarize_health([m])
    assert s["drops"] is None and s["drops_delta"] is None


def test_summarize_health_empty_raises():
    with pytest.raises(ValueError):
        summarize_health([])


def test_format_health_summary_line_machine_parseable():
    s = summarize_health([_metrics_msg(drops=0, gaps=0)], seconds=12.0)
    line = format_health_summary_line(s)
    assert line.startswith("HEALTH_SUMMARY ")
    kv = dict(tok.split("=", 1) for tok in line.split()[1:])
    assert float(kv["stream7_hz"]) == pytest.approx(27.8)
    assert float(kv["stream9_hz"]) == pytest.approx(30.2)
    assert float(kv["stream10_hz"]) == pytest.approx(30.1)
    assert kv["drops_delta"] == "0" and kv["gaps_delta"] == "0"


def test_format_health_summary_line_none_values_read_nan():
    s = summarize_health([_metrics_msg(streams=((10, "ENV", None, None),))])
    kv = dict(tok.split("=", 1) for tok in format_health_summary_line(s).split()[1:])
    assert kv["stream10_hz"] == "nan"
    assert kv["seconds"] == "nan"
