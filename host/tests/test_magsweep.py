"""Magnetometer sweep coverage + calibration-quality tests (owner ask, 2026-07-29).

Strategy mirrors test_web.py: the binning/coverage/quality math lives in pure
module-level helpers in `roomscan.magsweep`, so almost everything here runs
without a server or an event loop. The `magcal` workflow tests drive
`web._handle_magcal` directly with a hand-built `SimpleNamespace` state, the
same pattern the other inbound-handler tests use.

The load-bearing test is `test_incomplete_tumble_*`: a cap-only sample cloud
must report empty cells and a bad verdict. That is the exact failure that
shipped on 2026-07-15 and went unnoticed for two weeks.
"""
from __future__ import annotations

import asyncio
import json
import math
import struct
import types

import numpy as np
import pytest

from roomscan import magsweep as ms
from roomscan.protocol import Frame, FrameHeader, FrameType, StreamId
from roomscan import web
from roomscan.magcal import MagCalibration, fit_ellipsoid
from roomscan.sensors import AXIS_CONVENTION

FIELD = 50.0
# The rig's hard-iron offset as fitted 2026-07-15 -- the SUPERSEDED value: it was
# wrong by ~59 uT and is what BUG-030 was. Kept as the test fixture because it is
# a realistic magnitude, and because it is larger
# in magnitude than the field itself -- which is exactly why raw directions are
# useless for binning (see the magsweep module docstring).
HARD_IRON = np.array([44.4, -27.6, -41.7])
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _sphere(n=4000, seed=0):
    """(n, 3) unit vectors spread over the whole sphere."""
    rng = np.random.default_rng(seed)
    v = rng.normal(size=(n, 3))
    return v / np.linalg.norm(v, axis=1, keepdims=True)


def _raw_from_body(body_dirs, offset=HARD_IRON, field=FIELD, soft=None):
    """Body-frame field directions -> the RAW sensor samples that would produce
    them: undo AXIS_CONVENTION, scale to `field`, optionally apply a soft-iron
    distortion, then add the hard-iron offset."""
    v = np.asarray(body_dirs, dtype=np.float64).reshape(-1, 3) * field
    v = v @ np.linalg.inv(np.asarray(AXIS_CONVENTION, dtype=np.float64)).T
    if soft is not None:
        v = v @ np.linalg.inv(np.asarray(soft, dtype=np.float64)).T
    return v + np.asarray(offset, dtype=np.float64)


def _cal(offset=HARD_IRON, matrix=IDENTITY, field=FIELD):
    return MagCalibration(offset=tuple(float(v) for v in offset), matrix=matrix, field_ut=field)


def _cells(raw, cal):
    return ms.assign_cells(ms.calibrated_directions(raw, cal))


def _sframe(sid, payload: bytes, t_us: int = 1000) -> Frame:
    """Same shape as test_web.py's `_sframe` -- a bare sensor DATA frame."""
    return Frame(FrameHeader(FrameType.DATA, sid, 0, 1, t_us, 0, 0, len(payload)), payload)


# =============================================================================
# 1. The sphere lattice + binning
# =============================================================================

def test_lattice_is_unit_vectors_and_deterministic():
    a = ms.sphere_lattice()
    b = ms.sphere_lattice()
    assert a.shape == (ms.SPHERE_CELLS, 3)
    assert np.allclose(np.linalg.norm(a, axis=1), 1.0)
    assert a is b                      # cached
    assert not a.flags.writeable       # callers can't corrupt the cache


def test_lattice_cells_are_near_equal_area():
    """The whole point of a Fibonacci lattice over lat/lon bins: no pole
    over-weighting. Nearest-neighbour spacing must be nearly uniform -- a
    lat/lon grid's would vary by ~50x between pole and equator."""
    lat = ms.sphere_lattice()
    dots = lat @ lat.T
    np.fill_diagonal(dots, -2.0)
    nn_deg = np.degrees(np.arccos(np.clip(dots.max(axis=1), -1.0, 1.0)))
    assert nn_deg.max() / nn_deg.min() < 1.2


def test_lattice_covers_the_sphere_within_a_cell_radius():
    """Every direction is within ~a cell radius of some cell centre, so no
    orientation is unrepresentable on the map."""
    lat = ms.sphere_lattice()
    worst = np.degrees(np.arccos(np.clip((_sphere(20000, seed=7) @ lat.T).max(axis=1), -1.0, 1.0)))
    assert worst.max() < 17.0          # ~ the 20 deg lattice spacing / 2, plus corner slack
    assert worst.mean() < 9.0


def test_assign_cells_picks_the_nearest_centre():
    lat = ms.sphere_lattice()
    idx = ms.assign_cells(lat)
    assert np.array_equal(idx, np.arange(ms.SPHERE_CELLS))


def test_assign_cells_empty_input():
    assert ms.assign_cells(np.zeros((0, 3))).shape == (0,)


def test_neighbours_are_symmetric_and_connect_the_whole_sphere():
    adj = ms.cell_neighbours()
    for a, nbs in enumerate(adj):
        for b in nbs:
            assert a in adj[b]
    # one connected component
    seen, stack = {0}, [0]
    while stack:
        for nb in adj[stack.pop()]:
            if nb not in seen:
                seen.add(nb)
                stack.append(nb)
    assert len(seen) == ms.SPHERE_CELLS


# =============================================================================
# 2. Coverage -- the incomplete tumble must be visible
# =============================================================================

def test_full_tumble_reports_full_coverage_and_no_gaps():
    raw = _raw_from_body(_sphere())
    cov = ms.coverage_stats(_cells(raw, _cal()))
    assert cov["occupied"] == ms.SPHERE_CELLS
    assert cov["empty"] == 0
    assert cov["fraction"] == 1.0
    assert cov["verdict"] == "good"
    assert ms.empty_regions(cov["counts"]) == []


def test_incomplete_tumble_reports_empty_cells():
    """The 2026-07-15 failure mode: a cap of attitudes, nothing else. Coverage
    must call that out rather than reporting a healthy-looking fit."""
    body = _sphere()
    cap = body[body[:, 2] > 0.4]        # a cap around the device's Front axis
    cov = ms.coverage_stats(_cells(_raw_from_body(cap), _cal()))
    assert cov["empty"] > 0
    assert cov["fraction"] < 0.5
    assert cov["verdict"] == "bad"


def test_incomplete_tumble_gap_is_one_contiguous_region_facing_the_right_way():
    body = _sphere()
    cap = body[body[:, 2] > 0.4]
    cov = ms.coverage_stats(_cells(_raw_from_body(cap), _cal()))
    regions = ms.empty_regions(cov["counts"])
    assert len(regions) >= 1
    top = regions[0]
    assert top["size"] == cov["empty"]          # the missing sphere is one patch
    # The samples covered +Front, so the hole is centred on Back.
    assert top["face"] == "Back"
    assert np.dot(top["centroid"], [0.0, 0.0, -1.0]) > 0.8


def test_empty_regions_separates_disjoint_holes():
    counts = np.ones(ms.SPHERE_CELLS, dtype=np.int64)
    lat = ms.sphere_lattice()
    counts[int(np.argmax(lat @ [0.0, 0.0, 1.0]))] = 0
    counts[int(np.argmax(lat @ [0.0, 0.0, -1.0]))] = 0
    regions = ms.empty_regions(counts)
    assert [r["size"] for r in regions] == [1, 1]


def test_coverage_verdict_thresholds():
    n = ms.SPHERE_CELLS
    def verdict_for(occupied):
        return ms.coverage_stats(np.arange(occupied))["verdict"]
    assert verdict_for(n) == "good"
    assert verdict_for(int(n * ms.COVERAGE_GOOD) + 1) == "good"
    assert verdict_for(int(n * ms.COVERAGE_MARGINAL) + 1) == "marginal"
    assert verdict_for(int(n * ms.COVERAGE_MARGINAL) - 2) == "bad"


def test_binning_uses_calibrated_not_raw_directions():
    """Regression guard for the subtle bug this design avoids: the rig's
    hard-iron offset (65 uT) exceeds the field (50 uT), so RAW sample directions
    live in a cone and can never cover the sphere however well you tumble.
    Binned calibrated they cover it completely."""
    raw = _raw_from_body(_sphere())
    raw_cov = ms.coverage_stats(ms.assign_cells(ms.calibrated_directions(raw, None)))
    cal_cov = ms.coverage_stats(_cells(raw, _cal()))
    assert cal_cov["fraction"] == 1.0
    assert raw_cov["fraction"] < 0.55


# =============================================================================
# 3. Quality metrics -- flag a known hard-iron error, pass after fitting
# =============================================================================

def test_field_consistency_flags_a_known_hard_iron_offset():
    """Feed samples with a KNOWN hard-iron offset and an identity (uncalibrated)
    correction: |B| must vary hugely and the verdict must be bad."""
    raw = _raw_from_body(_sphere())
    stats = ms.field_consistency(raw, _cal(offset=(0.0, 0.0, 0.0)))
    assert stats["std_pct"] > 20.0
    assert stats["ratio"] > 2.0
    assert stats["verdict"] == "bad"


def test_field_consistency_passes_after_fitting():
    """...and the same samples, once fitted with the reused `fit_ellipsoid`,
    must come out consistent."""
    raw = _raw_from_body(_sphere())
    cal = fit_ellipsoid(raw)
    stats = ms.field_consistency(raw, cal)
    assert stats["std_pct"] < ms.FIELD_GOOD_PCT
    assert stats["ratio"] < 1.05
    assert stats["residual_rms_ut"] < 0.5
    assert stats["verdict"] == "good"
    assert np.allclose(cal.offset, HARD_IRON, atol=1e-6)


def test_field_consistency_recovers_soft_iron_too():
    soft = np.array([[1.25, 0.06, -0.02], [0.06, 0.9, 0.04], [-0.02, 0.04, 1.08]])
    raw = _raw_from_body(_sphere(), soft=soft)
    assert ms.field_consistency(raw, _cal())["verdict"] == "bad"
    assert ms.field_consistency(raw, fit_ellipsoid(raw))["verdict"] == "good"


def test_field_consistency_verdict_thresholds():
    """Synthesize clouds with an exact target std/mean by scaling radii."""
    def cloud(std_pct):
        dirs = _sphere(2000, seed=3)
        rng = np.random.default_rng(1)
        radii = FIELD * (1.0 + (std_pct / 100.0) * np.sign(rng.normal(size=dirs.shape[0])))
        return dirs * radii[:, None]
    ident = _cal(offset=(0.0, 0.0, 0.0))
    assert ms.field_consistency(cloud(1.0), ident)["verdict"] == "good"
    assert ms.field_consistency(cloud(3.0), ident)["verdict"] == "marginal"
    assert ms.field_consistency(cloud(9.0), ident)["verdict"] == "bad"


def test_field_consistency_none_without_samples_or_calibration():
    assert ms.field_consistency(np.zeros((0, 3)), _cal()) is None
    assert ms.field_consistency(_raw_from_body(_sphere(50)), None) is None


def test_cell_deviation_exposes_a_direction_dependent_error():
    """The headline diagnostic: colouring by |B| deviation under a DEFECTIVE
    calibration must make the error direction-dependent on the map -- some
    cells near zero, others far off -- rather than a uniform smear."""
    raw = _raw_from_body(_sphere())
    wrong = _cal(offset=HARD_IRON + np.array([0.0, 0.0, 18.0]))
    idx = _cells(raw, _cal())
    dev = [v for v in ms.cell_deviation_pct(raw, idx, wrong) if v is not None]
    assert len(dev) == ms.SPHERE_CELLS
    assert min(dev) < -20.0 and max(dev) > 20.0     # both signs, big spread


def test_cell_deviation_is_none_for_empty_cells():
    body = _sphere()
    raw = _raw_from_body(body[body[:, 2] > 0.4])
    idx = _cells(raw, _cal())
    dev = ms.cell_deviation_pct(raw, idx, _cal())
    counts = ms.coverage_stats(idx)["counts"]
    for i, c in enumerate(counts):
        assert (dev[i] is None) == (c == 0)


def test_quality_report_headline_is_the_worst_component_and_names_it():
    body = _sphere()
    raw = _raw_from_body(body[body[:, 2] > 0.4])
    idx = _cells(raw, _cal())
    # A calibration that is perfectly self-consistent over this cap -- exactly
    # the trap: `field` alone would say "good".
    rep = ms.quality_report(raw, idx, _cal())
    assert rep["field"]["verdict"] == "good"
    assert rep["coverage"]["verdict"] == "bad"
    assert rep["verdict"] == "bad"
    assert rep["limited_by"] == "coverage"
    # The reason must point at the SWEEP, not blame the calibration -- the two
    # failures need opposite responses from the user.
    assert "sphere sampled" in rep["reason"] and "sweep more" in rep["reason"]
    # Components are always present -- never a bare score.
    assert set(rep) >= {"field", "coverage", "samples", "samples_verdict",
                        "verdict", "limited_by", "reason"}


def test_field_verdict_catches_a_self_consistent_but_biased_calibration():
    """The hole found on-rig 2026-07-29: 255 stationary samples read |B| =
    101.96 uT against the calibration's own expected 49.87 uT (x2.04) yet
    scored std_pct = 0.22%. Spread alone must not be able to call that good."""
    dirs = _sphere(500, seed=13)
    samples = dirs * 100.0                                   # tight sphere, wrong radius
    cal = _cal(offset=(0.0, 0.0, 0.0), field=50.0)
    f = ms.field_consistency(samples, cal)
    assert f["std_pct"] < ms.FIELD_GOOD_PCT                  # perfectly self-consistent
    assert f["spread_verdict"] == "good"
    assert f["bias_pct"] == pytest.approx(100.0, abs=0.5)    # ...at 2x the expected field
    assert f["bias_verdict"] == "bad"
    assert f["verdict"] == "bad"                             # the worse of the two wins
    rep = ms.quality_report(samples, ms.assign_cells(ms.calibrated_directions(samples, cal)), cal)
    assert rep["limited_by"] == "field consistency"
    assert "off this calibration" in rep["reason"]


def test_field_verdict_reports_spread_when_spread_is_the_worse_failure():
    """A modest uncorrected hard iron over a FULL sphere: |B| swings widely with
    orientation but averages back to about the right radius, so spread is the
    failure and bias is not. The two components must not be redundant."""
    raw = _raw_from_body(_sphere(), offset=(6.0, -4.0, 3.0))
    f = ms.field_consistency(raw, _cal(offset=(0.0, 0.0, 0.0)))
    assert f["bias_verdict"] == "good"
    assert f["spread_verdict"] == "bad"
    assert f["verdict"] == "bad"
    assert "varies" in ms.quality_report(
        raw, _cells(raw, _cal()), _cal(offset=(0.0, 0.0, 0.0)))["reason"]


def test_quality_report_samples_verdict():
    dirs = _sphere(80, seed=5)
    raw = _raw_from_body(dirs)
    assert ms.quality_report(raw, _cells(raw, _cal()), _cal())["samples_verdict"] == "bad"
    raw = _raw_from_body(_sphere(150, seed=5))
    assert ms.quality_report(raw, _cells(raw, _cal()), _cal())["samples_verdict"] == "marginal"
    raw = _raw_from_body(_sphere(400, seed=5))
    assert ms.quality_report(raw, _cells(raw, _cal()), _cal())["samples_verdict"] == "good"


def test_provisional_calibration_bins_usably_with_no_saved_calibration():
    """A fresh install has no calibration at all; the bounding-box hard-iron
    estimate must still put directions in roughly the right cells."""
    body = _sphere(3000, seed=11)
    raw = _raw_from_body(body)
    prov = ms.provisional_calibration(raw)
    assert prov is not None
    assert np.allclose(prov.offset, HARD_IRON, atol=2.0)
    got = ms.calibrated_directions(raw, prov)
    assert float(np.mean(np.sum(got * body, axis=1))) > 0.97   # ~aligned with truth
    assert ms.coverage_stats(ms.assign_cells(got))["fraction"] > 0.98


def test_provisional_calibration_needs_samples():
    assert ms.provisional_calibration(np.zeros((0, 3))) is None
    assert ms.provisional_calibration(np.zeros((10, 3))) is None


def test_guidance_names_the_face_the_axis_and_the_countdown():
    body = _sphere()
    raw = _raw_from_body(body[body[:, 0] > 0.4])       # covered the Top face only
    regions = ms.empty_regions(ms.coverage_stats(_cells(raw, _cal()))["counts"])
    text = ms.guidance_text(regions, live_dir=[1.0, 0.0, 0.0])
    assert "Bottom" in text                   # the gap's face
    assert "axis" in text                     # the body axis to turn about
    assert "cells left in this gap" in text   # the countdown
    assert ms.guidance_text([]) == "Full sphere covered — every orientation has samples."


def test_guidance_makes_no_dip_or_compass_assumption():
    """The old text ("point the Top face toward magnetic north and downward")
    assumed northern-hemisphere dip AND that the user knows where north is.
    Both are gone: the instruction is an exact body-axis rotation (§5)."""
    body = _sphere()
    raw = _raw_from_body(body[body[:, 0] > 0.4])
    regions = ms.empty_regions(ms.coverage_stats(_cells(raw, _cal()))["counts"])
    text = ms.guidance_text(regions, live_dir=[1.0, 0.0, 0.0])
    for banned in ("north", "North", "downward", "magnetic"):
        assert banned not in text


def test_nearest_face_names_all_six():
    assert ms.nearest_face([1, 0, 0]) == "Top"
    assert ms.nearest_face([-1, 0, 0]) == "Bottom"
    assert ms.nearest_face([0, 1, 0]) == "Right"
    assert ms.nearest_face([0, -1, 0]) == "Left"
    assert ms.nearest_face([0, 0, 1]) == "Front"
    assert ms.nearest_face([0, 0, -1]) == "Back"
    assert ms.nearest_face([0, 0, 0]) == "?"


# =============================================================================
# 4. MagSweepSession
# =============================================================================

def test_session_collects_only_while_started_and_dedupes():
    s = ms.MagSweepSession()
    assert s.add((1.0, 2.0, 3.0), t_us=1) is False      # not collecting
    s.start()
    assert s.add((1.0, 2.0, 3.0), t_us=1) is True
    assert s.add((1.0, 2.0, 3.0), t_us=1) is False      # same env sample, polled twice
    assert s.add((1.0, 2.0, 4.0), t_us=2) is True
    assert s.samples.shape == (2, 3)


def test_session_rejects_malformed_samples():
    s = ms.MagSweepSession()
    s.start()
    assert s.add((1.0, 2.0), t_us=1) is False
    assert s.add((1.0, float("nan"), 3.0), t_us=2) is False
    assert s.samples.shape == (0, 3)


def test_session_caps_sample_count():
    s = ms.MagSweepSession(max_samples=10)
    s.start()
    for i in range(50):
        s.add((float(i), 0.0, 0.0), t_us=i)
    assert s.samples.shape == (10, 3)
    assert s.samples[-1][0] == 49.0                    # newest kept


def test_session_stop_fits_a_candidate():
    s = ms.MagSweepSession()
    s.start()
    for i, v in enumerate(_raw_from_body(_sphere(600, seed=2))):
        s.add(v, t_us=i)
    s.stop()
    assert s.collecting is False
    assert s.candidate is not None
    assert s.fit_error is None
    assert np.allclose(s.candidate.offset, HARD_IRON, atol=0.5)


def test_session_stop_with_too_few_samples_reports_not_raises():
    s = ms.MagSweepSession()
    s.start()
    for i in range(5):
        s.add((float(i), 1.0, 2.0), t_us=i)
    s.stop()
    assert s.candidate is None
    assert "at least" in s.fit_error


def test_session_stop_on_a_degenerate_cloud_reports_not_raises():
    s = ms.MagSweepSession()
    s.start()
    for i in range(200):                               # a straight line: rank-deficient
        s.add((float(i), 0.0, 0.0), t_us=i)
    s.stop()
    assert s.candidate is None
    assert s.fit_error


def test_session_reset_clears_everything():
    s = ms.MagSweepSession()
    s.start()
    for i, v in enumerate(_raw_from_body(_sphere(400, seed=4))):
        s.add(v, t_us=i)
    s.stop()
    s.reset()
    assert s.samples.shape == (0, 3)
    assert s.candidate is None
    assert s.elapsed() == 0.0


def test_session_binning_calibration_freezes_on_first_determinable_value():
    """BUG-046 / issue #57: binning is FROZEN, not recomputed every call --
    the first determinable value (same candidate > current > provisional
    preference order as before) is latched and held from then on, whatever
    is passed or fitted afterwards. Only `reset()` re-arms it."""
    s = ms.MagSweepSession()
    saved = _cal()
    assert s.binning_calibration(None) is None          # nothing determinable: stays live
    assert s.binning_calibration(saved) is saved        # first determinable value: latches
    assert s.binning_calibration(None) is saved         # still latched, whatever `current` now is
    other = _cal(offset=(1.0, 2.0, 3.0))
    assert s.binning_calibration(other) is saved         # a DIFFERENT saved cal doesn't move it

    s.start()
    for i, v in enumerate(_raw_from_body(_sphere(400, seed=6))):
        s.add(v, t_us=i)
    s.stop()
    assert s.candidate is not None
    assert s.binning_calibration(saved) is saved         # candidate exists now -- still ignored
    s.reset()
    assert s.binning_calibration(None) is None           # re-armed: nothing determinable again


def test_session_binning_calibration_prefers_candidate_on_first_call():
    """With a candidate already fitted before `binning_calibration` is ever
    called, the candidate wins over whatever `current` is passed -- same
    preference order as before, just evaluated once instead of every tick."""
    s = ms.MagSweepSession()
    s.start()
    for i, v in enumerate(_raw_from_body(_sphere(400, seed=6))):
        s.add(v, t_us=i)
    s.stop()
    assert s.candidate is not None
    assert s.binning_calibration(_cal()) is s.candidate
    assert s.coverage_binning_kind() == "candidate"


def test_session_binning_calibration_falls_back_to_provisional_then_latches():
    s = ms.MagSweepSession()
    s.start()
    for i, v in enumerate(_raw_from_body(_sphere(400, seed=6))):
        s.add(v, t_us=i)
    prov = s.binning_calibration(None)          # no candidate, no saved cal
    assert prov is not None
    assert s.coverage_binning_kind() == "provisional"
    assert s.binning_calibration(_cal()) is prov          # latched -- a saved cal shows up too late


# =============================================================================
# 5. build_report -- the wire message
# =============================================================================

def _filled_session(n=600, seed=2, mask=None):
    s = ms.MagSweepSession()
    s.start()
    body = _sphere(n, seed=seed)
    if mask is not None:
        body = body[mask(body)]
    for i, v in enumerate(_raw_from_body(body)):
        s.add(v, t_us=i)
    return s


def test_build_report_shape_and_json_safety():
    s = _filled_session()
    rep = ms.build_report(s, _cal(), view="current", saved_path="mag_cal.json")
    assert rep["type"] == "magcal"
    assert rep["cells"] == ms.SPHERE_CELLS
    assert len(rep["cell_dirs"]) == ms.SPHERE_CELLS
    assert len(rep["cell_counts"]) == ms.SPHERE_CELLS
    assert len(rep["cell_dev_pct"]) == ms.SPHERE_CELLS
    assert rep["saved_path"] == "mag_cal.json"
    assert rep["binning"] == "current"
    json.dumps(rep)          # no numpy scalars leak onto the wire


def test_build_report_marks_the_live_cell():
    s = _filled_session()
    rep = ms.build_report(s, _cal())
    assert rep["live_cell"] is not None
    assert 0 <= rep["live_cell"] < ms.SPHERE_CELLS
    assert len(rep["live_dir"]) == 3


def test_build_report_carries_both_calibrations_quality():
    s = _filled_session()
    s.stop()
    rep = ms.build_report(s, _cal())
    assert rep["current"] is not None and rep["candidate"] is not None
    assert rep["has_current"] and rep["has_candidate"]
    assert rep["candidate"]["field"]["verdict"] == "good"


def test_build_report_view_selects_the_colouring_calibration():
    s = _filled_session()
    s.stop()
    wrong = _cal(offset=HARD_IRON + np.array([0.0, 0.0, 18.0]))
    cur = ms.build_report(s, wrong, view="current")
    cand = ms.build_report(s, wrong, view="candidate")
    cur_dev = [v for v in cur["cell_dev_pct"] if v is not None]
    cand_dev = [v for v in cand["cell_dev_pct"] if v is not None]
    assert max(cur_dev) - min(cur_dev) > 30.0        # the defect, visible
    assert max(cand_dev) - min(cand_dev) < 2.0       # the good fit, flat


def test_build_report_with_no_calibration_at_all():
    s = _filled_session()
    rep = ms.build_report(s, None)
    assert rep["has_current"] is False
    assert rep["current"] is None
    assert rep["binning"] == "provisional"
    json.dumps(rep)


def test_build_report_on_an_empty_session():
    rep = ms.build_report(ms.MagSweepSession(), None)
    assert rep["sample_count"] == 0
    assert rep["live_cell"] is None
    assert set(rep["cell_counts"]) == {0}
    assert rep["gaps"][0]["size"] == ms.SPHERE_CELLS   # nothing covered -> one huge gap
    json.dumps(rep)


def test_empty_session_reports_no_verdict_for_the_saved_calibration():
    """Both quality blocks measure THIS session's samples. With none collected
    there is nothing to say about the saved calibration -- null, not a
    `bad, 0% coverage` verdict that reads as a claim about the file."""
    rep = ms.build_report(ms.MagSweepSession(), _cal())
    assert rep["has_current"] is True
    assert rep["current"] is None
    assert rep["candidate"] is None
    assert rep["current_field_ut"] == pytest.approx(FIELD)


def test_guidance_on_a_totally_empty_sphere_does_not_name_an_arbitrary_face():
    """With zero coverage the "largest gap" is the whole sphere and its
    centroid is meaningless, so the text must not pretend to aim the user."""
    text = ms.guidance_text(ms.empty_regions(np.zeros(ms.SPHERE_CELLS, dtype=np.int64)))
    assert "Nothing collected yet" in text
    assert "Biggest gap" not in text
    for face, _ in ms.FACES:
        assert f"device's {face}" not in text


def test_build_report_incomplete_tumble_says_so_loudly():
    """The stationary/partial case (also what the on-rig screenshot shows): the
    report must read `bad` and point at a large gap."""
    s = _filled_session(mask=lambda b: b[:, 2] > 0.6)
    s.stop()
    rep = ms.build_report(s, _cal())
    assert rep["candidate"]["coverage"]["verdict"] == "bad"
    assert rep["candidate"]["verdict"] == "bad"
    assert rep["gaps"][0]["size"] > ms.SPHERE_CELLS // 2
    assert "Biggest gap" in rep["guidance"]


def test_coverage_progress_does_not_regress_across_the_fit_swap():
    """BUG-046 / issue #57: pressing Fit at "92/92 cells complete" must never
    un-complete the sweep. Re-bin a FIXED sample cloud under two calibrations
    that genuinely disagree (`saved`, in force when the sweep starts, and
    `fitted`, standing in for whatever `stop()` produces) and assert the
    reported coverage does not go backwards.

    The two calibrations are a deliberately engineered worst case, not a
    hopeful one: `raw` is built so `calibrated_directions(raw, saved)` lands
    EXACTLY one sample on every one of the 92 lattice cell centres (verified
    below), and `fitted` is `saved` rotated 15 deg -- comfortably enough to
    push a lattice-spaced (~24 deg) sample across a cell boundary. A naive
    re-bin under `fitted` alone (also asserted below, independent of the
    session) measurably empties cells. Through a `MagSweepSession`, with the
    coverage frame frozen at Start, it must not.

    PROVEN by reintroducing the defect: reverting
    `MagSweepSession.binning_calibration` to recompute fresh on every call
    (as it did before this fix) makes this test fail with
    `occ_after == 85 < occ_before == 92`."""
    lattice = ms.sphere_lattice()
    raw = lattice @ np.asarray(AXIS_CONVENTION, dtype=np.float64)
    saved = _cal(offset=(0.0, 0.0, 0.0), matrix=IDENTITY, field=1.0)
    assert np.array_equal(ms.assign_cells(ms.calibrated_directions(raw, saved)),
                           np.arange(ms.SPHERE_CELLS))          # one sample, each cell, exactly

    axis = np.array([0.3, 0.4, 0.8660254])
    axis /= np.linalg.norm(axis)
    rot = ms.axis_angle_matrix(axis, math.radians(15.0))
    fitted = _cal(offset=(0.0, 0.0, 0.0), matrix=tuple(map(tuple, rot)), field=1.0)

    # The disagreement is real: binning `raw` fresh under `fitted` alone
    # empties cells (this is the mechanism the fix guards against).
    naive_after = ms.coverage_stats(ms.assign_cells(ms.calibrated_directions(raw, fitted)))
    assert naive_after["occupied"] < ms.SPHERE_CELLS

    s = ms.MagSweepSession()
    s.start()
    for i, v in enumerate(raw):
        s.add(v, t_us=i)
    before = ms.build_report(s, saved)
    assert before["binning"] == "current"
    occ_before = sum(1 for c in before["cell_counts"] if c)
    assert occ_before == ms.SPHERE_CELLS

    s.stop()                        # fits its own candidate from `raw`; overridden below
    s.candidate = fitted            # stand in for whatever the real fit produced
    after = ms.build_report(s, saved)
    occ_after = sum(1 for c in after["cell_counts"] if c)

    assert occ_after >= occ_before, f"coverage regressed: {occ_after} < {occ_before}"
    assert occ_after == ms.SPHERE_CELLS
    assert after["cell_counts"] == before["cell_counts"]      # frozen: literally unchanged
    assert after["binning"] == "current"                      # never flips to "candidate"


# =============================================================================
# 6. The /ws workflow: collect -> fit -> preview -> save / discard
# =============================================================================

def _ws_state(tmp_path, mag_cal=None, sensor_state=None):
    from roomscan.logbus import LogBus
    return types.SimpleNamespace(
        clients=set(), magcal_clients=set(), magcal_view="current",
        magcal_session=ms.MagSweepSession(), mag_cal=mag_cal,
        mag_cal_path=str(tmp_path / "mag_cal.json"),
        sensor_state=sensor_state, fusion=None, bus=LogBus(),
        ui_state=web.UiState(), config=None, controller=None)


def _route(state, msg):
    asyncio.run(web._handle_inbound(state, msg))


def _fill(state, n=600, seed=2):
    for i, v in enumerate(_raw_from_body(_sphere(n, seed=seed))):
        state.magcal_session.add(v, t_us=i)


def test_workflow_start_stop_fit_then_save(tmp_path):
    state = _ws_state(tmp_path)
    _route(state, {"type": "magcal", "action": "start"})
    assert state.magcal_session.collecting is True
    _fill(state)
    _route(state, {"type": "magcal", "action": "stop"})
    assert state.magcal_session.collecting is False
    cand = state.magcal_session.candidate
    assert cand is not None
    # NOT saved yet -- a preview must never touch the file or the live cal.
    assert not (tmp_path / "mag_cal.json").exists()
    assert state.mag_cal is None

    _route(state, {"type": "magcal", "action": "save"})
    saved = MagCalibration.load(tmp_path / "mag_cal.json")
    assert saved is not None
    assert np.allclose(saved.offset, cand.offset)
    assert np.allclose(saved.matrix, cand.matrix)
    assert saved.field_ut == pytest.approx(cand.field_ut)
    assert state.mag_cal == cand                        # hot-reloaded, no restart
    assert state.magcal_session.candidate is None       # consumed


def test_saved_file_carries_the_field_ut_the_rest_of_the_system_reads(tmp_path):
    state = _ws_state(tmp_path)
    _route(state, {"type": "magcal", "action": "start"})
    _fill(state)
    _route(state, {"type": "magcal", "action": "stop"})
    _route(state, {"type": "magcal", "action": "save"})
    raw = json.loads((tmp_path / "mag_cal.json").read_text(encoding="utf-8"))
    assert set(raw) == {"offset", "matrix", "field_ut"}      # same format as tools/mag_calibrate
    assert raw["field_ut"] == pytest.approx(FIELD, rel=0.02)


def test_save_refuses_without_a_candidate(tmp_path):
    state = _ws_state(tmp_path)
    _fill(state)                                   # samples, but never fitted
    _route(state, {"type": "magcal", "action": "save"})
    assert not (tmp_path / "mag_cal.json").exists()
    assert state.mag_cal is None


def test_discard_drops_the_candidate_but_keeps_the_samples(tmp_path):
    state = _ws_state(tmp_path)
    _route(state, {"type": "magcal", "action": "start"})
    _fill(state)
    _route(state, {"type": "magcal", "action": "stop"})
    assert state.magcal_session.candidate is not None
    _route(state, {"type": "magcal", "action": "discard"})
    assert state.magcal_session.candidate is None
    assert state.magcal_session.samples.shape[0] == 600     # keep tumbling into these
    assert not (tmp_path / "mag_cal.json").exists()


def test_reset_clears_the_samples(tmp_path):
    state = _ws_state(tmp_path)
    _route(state, {"type": "magcal", "action": "start"})
    _fill(state)
    _route(state, {"type": "magcal", "action": "reset"})
    assert state.magcal_session.samples.shape == (0, 3)


def test_existing_calibration_is_never_replaced_by_a_failed_fit(tmp_path):
    existing = _cal()
    state = _ws_state(tmp_path, mag_cal=existing)
    _route(state, {"type": "magcal", "action": "start"})
    for i in range(200):                                   # degenerate: a line
        state.magcal_session.add((float(i), 0.0, 0.0), t_us=i)
    _route(state, {"type": "magcal", "action": "stop"})
    _route(state, {"type": "magcal", "action": "save"})
    assert state.mag_cal is existing
    assert not (tmp_path / "mag_cal.json").exists()


def test_view_action_switches_and_rejects_junk(tmp_path):
    state = _ws_state(tmp_path)
    _route(state, {"type": "magcal", "action": "view", "cal": "candidate"})
    assert state.magcal_view == "candidate"
    _route(state, {"type": "magcal", "action": "view", "cal": "bogus"})
    assert state.magcal_view == "current"


def test_unknown_magcal_action_is_ignored(tmp_path):
    state = _ws_state(tmp_path)
    _route(state, {"type": "magcal", "action": "nope"})
    _route(state, {"type": "magcal"})
    assert state.magcal_session.collecting is False


def test_install_mag_calibration_hot_reloads_every_consumer(tmp_path):
    """Save must reach all three holders of the calibration, or the FUSED
    heading silently keeps running on the old one until a server restart."""
    from roomscan.sensors import SensorState, YawFusion
    old, new = _cal(), _cal(offset=(1.0, 2.0, 3.0), field=48.0)
    fusion = YawFusion(calibration=old)
    sensor_state = SensorState(fusion=fusion)
    fusion._delta, fusion._have_delta = 33.0, True
    state = _ws_state(tmp_path, mag_cal=old, sensor_state=sensor_state)
    state.fusion = fusion
    web.install_mag_calibration(state, new)
    assert state.mag_cal is new
    assert fusion.cal is new
    assert fusion._have_delta is False        # re-snaps instead of low-passing a stale delta


def test_open_and_close_manage_the_subscriber_set(tmp_path):
    class FakeWs:
        def __init__(self):
            self.sent = []
        async def send_text(self, text):
            self.sent.append(text)
    ws = FakeWs()
    state = _ws_state(tmp_path)
    asyncio.run(web._handle_inbound(state, {"type": "magcal", "action": "open"}, ws))
    assert ws in state.magcal_clients
    assert json.loads(ws.sent[0])["type"] == "magcal"     # answered immediately
    asyncio.run(web._handle_inbound(state, {"type": "magcal", "action": "close"}, ws))
    assert ws not in state.magcal_clients


# =============================================================================
# 7. Guard: the diagnostic must not perturb the display / SLAM path
# =============================================================================

def test_magcal_preview_does_not_touch_display_path(tmp_path):
    """Collecting, fitting, previewing, STREAMING POSE and discarding are pure
    observation: the gravity-alignment rotation, the point-cloud bytes,
    `fused_quat()` and the loaded calibration must all be bit-identical
    afterwards. (Save is the one deliberate exception -- covered by
    `test_install_mag_calibration_hot_reloads_every_consumer`.)

    Extended 2026-07-29 over the 30 Hz MAGPOSE channel: it reads `fused_quat()`
    and `latest_imu_raw()` every tick, which is exactly the kind of "harmless"
    read that acquires a side effect the moment someone adds smoothing to it.
    Drives a full open -> pose-stream -> close cycle."""
    from roomscan.deproject import Deprojector
    from roomscan.sensors import SensorState, YawFusion

    existing = _cal()
    fusion = YawFusion(calibration=existing)
    sensor_state = SensorState(fusion=fusion)
    quat = (0.9238795, 0.0, 0.3826834, 0.0)
    sensor_state.feed(_sframe(StreamId.IMU_QUAT, np.asarray(quat, dtype="<f4").tobytes()))

    depth = np.linspace(0.5, 3.0, 54 * 42, dtype=np.float32).reshape(42, 54)
    deproj = Deprojector(54, 42, 55.0, 42.0)
    outputs = {"depth": depth}

    before_quat = sensor_state.fused_quat()
    before_rot = web.display_rotation(before_quat)
    pts, colors, _ = web.select_colors(outputs, deproj, "depth", "turbo")
    before_bytes = web.pack_point_cloud(web.rotate_points(pts, before_rot), colors)

    class _Ws:
        def __init__(self):
            self.sent = []

        async def send_text(self, text):
            self.sent.append(text)

    ws = _Ws()
    state = _ws_state(tmp_path, mag_cal=existing, sensor_state=sensor_state)
    state.fusion = fusion
    asyncio.run(web._handle_inbound(state, {"type": "magcal", "action": "open"}, ws))
    _route(state, {"type": "magcal", "action": "start"})
    _fill(state)
    # The pose channel, driven exactly as the broadcaster drives it.
    poses = [web.build_magpose(state.magcal_session, existing, sensor_state, i, stored=True)
             for i in range(30)]
    assert all(p is not None and len(p) == web.MAGPOSE_SIZE for p in poses)
    _route(state, {"type": "magcal", "action": "stop"})
    assert state.magcal_session.candidate is not None            # a real fit happened
    ms.build_report(state.magcal_session, existing, view="candidate")
    web.build_magpose(state.magcal_session, existing, sensor_state, 99, view="candidate")
    _route(state, {"type": "magcal", "action": "discard"})
    asyncio.run(web._handle_inbound(state, {"type": "magcal", "action": "close"}, ws))

    after_quat = sensor_state.fused_quat()
    after_rot = web.display_rotation(after_quat)
    pts2, colors2, _ = web.select_colors(outputs, deproj, "depth", "turbo")
    after_bytes = web.pack_point_cloud(web.rotate_points(pts2, after_rot), colors2)

    assert after_quat == before_quat
    assert np.array_equal(after_rot, before_rot)
    assert after_bytes == before_bytes
    assert state.mag_cal is existing
    assert fusion.cal is existing


def test_magcal_module_does_not_mutate_axis_convention():
    """`calibrated_directions` uses AXIS_CONVENTION; the module constant is
    write-protected and shared with the display path."""
    before = np.array(AXIS_CONVENTION, copy=True)
    ms.calibrated_directions(_raw_from_body(_sphere(100)), _cal())
    assert np.array_equal(AXIS_CONVENTION, before)


def test_build_sensor_message_unchanged_by_an_open_sweep_session():
    """A running sweep is invisible to the `sensor` message every tab renders."""
    from roomscan.sensors import SensorState
    sensor_state = SensorState()
    sensor_state.feed(_sframe(StreamId.IMU_QUAT,
                              np.asarray((1.0, 0.0, 0.0, 0.0), dtype="<f4").tobytes()))
    before = web.build_sensor_message(sensor_state, _cal())
    session = ms.MagSweepSession()
    session.start()
    for i, v in enumerate(_raw_from_body(_sphere(300, seed=8))):
        session.add(v, t_us=i)
    session.stop()
    after = web.build_sensor_message(sensor_state, _cal())
    for key in ("rot", "heading", "orientation_raw", "orientation_view", "has_mag_cal"):
        assert after[key] == before[key]


# =============================================================================
# 8. The exact-rotation guidance (3D feedback, 2026-07-29)
# =============================================================================

def _unit(v):
    v = np.asarray(v, dtype=np.float64)
    return v / np.linalg.norm(v)


@pytest.mark.parametrize("d,t", [
    ([0, 0, 1], [1, 0, 0]),
    ([1, 0, 0], [0, 1, 0]),
    ([0.3, -0.5, 0.81], [-0.7, 0.2, 0.68]),
    ([1, 0, 0], [1, 0, 0]),            # already there
    ([1, 0, 0], [-1, 0, 0]),           # exactly antipodal: any perpendicular axis
])
def test_rotation_to_round_trips_d_onto_t(d, t):
    """THE guidance invariant: applying the returned body rotation `dR` moves
    the body-frame field direction `d` exactly onto the target `t`.

    `dR^T . d == t` because rotating the device body by `dR` (`R' = R.dR`) gives
    `d' = dR^T . d`. If this ever silently flips sign the arrow points the user
    the wrong way round the sphere, which is worse than no arrow at all."""
    d, t = _unit(d), _unit(t)
    axis, angle = ms.rotation_to(d, t)
    assert abs(np.linalg.norm(axis) - 1.0) < 1e-12
    moved = ms.axis_angle_matrix(axis, angle).T @ d
    assert np.allclose(moved, t, atol=1e-9)


def test_rotation_to_is_the_minimal_rotation():
    d, t = _unit([0, 0, 1]), _unit([1, 1, 0])
    axis, angle = ms.rotation_to(d, t)
    assert angle <= math.pi + 1e-12
    assert math.degrees(angle) == pytest.approx(90.0, abs=1e-6)
    # axis = unit(t x d): perpendicular to BOTH, per the design's derivation.
    assert abs(float(np.dot(axis, d))) < 1e-12
    assert abs(float(np.dot(axis, t))) < 1e-12


def test_rotation_to_rejects_degenerate_input():
    assert ms.rotation_to([0, 0, 0], [1, 0, 0]) is None
    assert ms.rotation_to([1, 0, 0], [0, 0, 0]) is None


def test_axis_pair_names_are_signless():
    assert ms.axis_pair_name([1, 0, 0]) == ms.axis_pair_name([-1, 0, 0]) == "Top–Bottom"
    assert ms.axis_pair_name([0, 1, 0]) == ms.axis_pair_name([0, -1, 0]) == "Right–Left"
    assert ms.axis_pair_name([0, 0, 1]) == ms.axis_pair_name([0, 0, -1]) == "Front–Back"


def test_target_selection_prefers_the_biggest_region_over_the_nearest_singleton():
    """Chasing a stray singleton is busywork; the big hole is the one that moves
    the coverage number. Guarded because it is the whole reason `empty_regions`
    exists rather than a flat list of empty cells."""
    lat = ms.sphere_lattice()
    counts = np.ones(ms.SPHERE_CELLS, dtype=np.int64)
    live = _unit([0.0, 0.0, 1.0])
    # A singleton right next to `live`...
    near = int(np.argmax(lat @ live))
    counts[near] = 0
    # ...and a big connected region on the far side.
    far = int(np.argmin(lat @ live))
    adj = ms.cell_neighbours()
    region = {far, *adj[far]}
    for c in adj[far]:
        region.update(adj[c])
    for c in region:
        counts[c] = 0
    regions = ms.empty_regions(counts)
    cell, direction, size = ms.select_target(regions, live)
    assert size >= ms.MIN_GUIDANCE_REGION
    assert cell in region and cell != near


def test_target_selection_falls_back_to_the_nearest_cell_for_scattered_misses():
    lat = ms.sphere_lattice()
    counts = np.ones(ms.SPHERE_CELLS, dtype=np.int64)
    live = _unit([0.0, 0.0, 1.0])
    near = int(np.argmax(lat @ live))
    far = int(np.argmin(lat @ live))
    counts[near] = 0
    counts[far] = 0
    regions = ms.empty_regions(counts)
    assert max(r["size"] for r in regions) < ms.MIN_GUIDANCE_REGION
    cell, _, _ = ms.select_target(regions, live)
    assert cell == near                       # nearest wins when nothing is big


def test_guidance_axis_shape_and_target_agreement():
    body = _sphere()
    raw = _raw_from_body(body[body[:, 0] > 0.4])
    regions = ms.empty_regions(ms.coverage_stats(_cells(raw, _cal()))["counts"])
    live = [0.0, 0.0, 1.0]
    ga = ms.guidance_axis(regions, live)
    assert set(ga) == {"target_cell", "target", "region_size", "to_face", "axis",
                       "angle_deg", "from_face", "text"}
    json.dumps(ga)
    # The shipped axis/angle really do carry the live direction onto the target.
    moved = ms.axis_angle_matrix(ga["axis"], math.radians(ga["angle_deg"])).T @ _unit(live)
    assert np.allclose(moved, ga["target"], atol=1e-3)
    assert ga["to_face"] == ms.nearest_face(ga["target"])


def test_guidance_axis_without_a_live_direction_still_names_a_target():
    counts = np.ones(ms.SPHERE_CELLS, dtype=np.int64)
    counts[:10] = 0
    ga = ms.guidance_axis(ms.empty_regions(counts), None)
    assert ga["axis"] is None and ga["angle_deg"] is None
    assert ga["target_cell"] is not None
    assert ga["region_size"] >= 1


def test_guidance_axis_is_none_on_a_full_sphere():
    assert ms.guidance_axis([], [0, 0, 1]) is None


# =============================================================================
# 9. Rolling provisional fit + stationary detection
# =============================================================================

def test_rolling_fit_recovers_the_calibration_from_a_full_tumble():
    raw = _raw_from_body(_sphere(800, seed=4))
    lf = ms.rolling_fit(raw)
    assert lf["error"] is None
    assert lf["field_ut"] == pytest.approx(FIELD, rel=0.02)
    assert lf["std_pct"] < ms.FIELD_GOOD_PCT
    assert lf["spread_verdict"] == "good"
    assert lf["samples"] == 800 and lf["used"] == 800
    json.dumps(lf)


def test_rolling_fit_reports_not_yet_fittable_instead_of_raising():
    lf = ms.rolling_fit(_raw_from_body(_sphere(5)))
    assert lf["error"] and lf["std_pct"] is None
    assert ms.rolling_fit(np.zeros((0, 3))) is None


def test_rolling_fit_decimates_rather_than_truncating():
    """Truncating would drop the NEWEST samples -- exactly the coverage the user
    is actively adding -- so the live readout would lag the motion it rewards."""
    n = ms.LIVE_FIT_MAX_SAMPLES + 2500
    raw = _raw_from_body(_sphere(n, seed=11))
    lf = ms.rolling_fit(raw)
    assert lf["samples"] == n
    assert lf["used"] == ms.LIVE_FIT_MAX_SAMPLES
    assert lf["error"] is None
    assert lf["field_ut"] == pytest.approx(FIELD, rel=0.02)


def test_rolling_fit_of_a_cap_only_cloud_is_read_next_to_coverage():
    """The 2026-07-15 defect was a self-consistent fit through a CAP of the
    sphere: the spread alone can look fine there, which is exactly why the UI
    never shows it without coverage beside it. Assert the pair."""
    body = _sphere(1500, seed=5)
    raw = _raw_from_body(body[body[:, 0] > 0.55])
    lf = ms.rolling_fit(raw)
    cov = ms.coverage_stats(_cells(raw, _cal()))
    assert lf is not None
    assert cov["verdict"] == "bad"


def test_motion_state_calls_a_still_device_stationary():
    cal = _cal()
    raw = _raw_from_body([[0.0, 0.0, 1.0]])[0]
    hist = [(t * 0.033, tuple(raw)) for t in range(60)]
    m = ms.motion_state(hist, cal)
    assert m["stationary"] is True
    assert m["spread_deg"] == pytest.approx(0.0, abs=1e-6)


def test_motion_state_calls_a_turning_device_moving():
    cal = _cal()
    body = [[math.cos(a), math.sin(a), 0.0] for a in np.linspace(0.0, 1.2, 60)]
    raw = _raw_from_body(body)
    hist = [(i * 0.033, tuple(v)) for i, v in enumerate(raw)]
    m = ms.motion_state(hist, cal)
    assert m["stationary"] is False
    assert m["spread_deg"] > ms.STATIONARY_SPREAD_DEG


def test_motion_state_says_nothing_without_enough_history():
    assert ms.motion_state([], _cal())["stationary"] is False
    assert ms.motion_state([(0.0, (1.0, 2.0, 3.0))], _cal())["n"] == 1


def test_session_tracks_the_live_vector_even_while_not_collecting():
    """"Is the board sitting still" is a question the modal answers BEFORE you
    press Start, not only after."""
    s = ms.MagSweepSession()
    for i in range(20):
        s.add((10.0 + i, 2.0, 3.0), t_us=i)
    assert s.samples.shape[0] == 0        # nothing stored
    assert len(s.recent) >= 10            # but the motion history is live


# =============================================================================
# 10. The report split: the deterministic constants ride `open` only
# =============================================================================

def test_report_omits_the_deterministic_constants_unless_full():
    s = _filled_session()
    lean = ms.build_report(s, _cal(), full=False)
    full = ms.build_report(s, _cal(), full=True)
    assert "cell_dirs" not in lean and "t_world_to_cv" not in lean
    assert len(full["cell_dirs"]) == ms.SPHERE_CELLS
    assert len(full["t_world_to_cv"]) == 9
    # Everything the renderers need per tick is still in the lean message.
    for key in ("cell_counts", "cell_dev_pct", "live_cell", "live_dir",
                "guidance", "guidance_axis", "live_fit", "motion"):
        assert key in lean
    assert len(json.dumps(lean)) < len(json.dumps(full))
    json.dumps(full)


def test_t_world_to_cv_is_the_shared_constant_not_a_local_redefinition():
    from roomscan.sensors import T_WORLD_TO_CV
    rep = ms.build_report(ms.MagSweepSession(), _cal(), full=True)
    assert np.array_equal(np.asarray(rep["t_world_to_cv"]).reshape(3, 3),
                          np.asarray(T_WORLD_TO_CV))


def test_open_sends_the_constants_and_later_ticks_do_not(tmp_path):
    class FakeWs:
        def __init__(self):
            self.sent = []

        async def send_text(self, text):
            self.sent.append(text)

    ws = FakeWs()
    state = _ws_state(tmp_path, mag_cal=_cal())
    asyncio.run(web._handle_inbound(state, {"type": "magcal", "action": "open"}, ws))
    first = json.loads(ws.sent[0])
    assert "cell_dirs" in first and "t_world_to_cv" in first
    assert "cell_dirs" not in web._magcal_report(state)


# =============================================================================
# 11. MAGPOSE (binary tag 5)
# =============================================================================

def _decode_magpose(blob):
    f = struct.unpack("<II13fhhHH", blob)
    return {
        "tag": f[0], "seq": f[1], "quat": f[2:6], "dir": f[6:9], "gravity": f[9:12],
        "field_ut": f[12], "dev_pct": f[13], "dip_deg": f[14],
        "live_cell": f[15], "filled_cell": f[16], "flags": f[17], "pad": f[18],
    }


def test_magpose_golden_byte_layout():
    """The wire layout, pinned. 68 bytes, tag 5 first, LE throughout -- if this
    changes, `magcal3d.decodeMagpose` and docs/web-protocol.md change with it
    (the `protocol-change` skill)."""
    blob = web.pack_magpose(
        seq=7, quat=(1.0, 0.0, 0.0, 0.0), field_dir=(0.0, 0.0, 1.0),
        gravity=(-1.0, 0.0, 0.0), field_ut=50.0, dev_pct=-2.5, dip_deg=114.25,
        live_cell=47, filled_cell=-1, flags=0b1001)
    assert len(blob) == 68 == web.MAGPOSE_SIZE
    got = _decode_magpose(blob)
    assert got["tag"] == web.TAG_MAGPOSE == 5
    assert got["seq"] == 7
    assert got["quat"] == (1.0, 0.0, 0.0, 0.0)
    assert got["dir"] == (0.0, 0.0, 1.0)
    assert got["gravity"] == (-1.0, 0.0, 0.0)
    assert got["field_ut"] == pytest.approx(50.0)
    assert got["dev_pct"] == pytest.approx(-2.5)
    assert got["dip_deg"] == pytest.approx(114.25)
    assert got["live_cell"] == 47
    assert got["filled_cell"] == -1          # -1 = none, and SIGNED, not 65535
    assert got["flags"] == 0b1001
    assert got["pad"] == 0
    # Field offsets, spelled out so a reordering can't pass silently.
    assert blob[0:4] == b"\x05\x00\x00\x00"
    assert blob[4:8] == b"\x07\x00\x00\x00"


def _pose_session(quat=(1.0, 0.0, 0.0, 0.0)):
    from roomscan.sensors import SensorState
    sensor_state = SensorState()
    if quat is not None:
        sensor_state.feed(_sframe(StreamId.IMU_QUAT,
                                  np.asarray(quat, dtype="<f4").tobytes()))
    session = ms.MagSweepSession()
    session.start()
    for i, v in enumerate(_raw_from_body(_sphere(300, seed=3))):
        session.add(v, t_us=i)
    return session, sensor_state


def test_magpose_reports_the_live_cell_and_the_dev_pct_of_the_viewed_cal():
    session, sensor_state = _pose_session()
    cal = _cal()
    got = _decode_magpose(web.build_magpose(session, cal, sensor_state, 1))
    dirs = ms.calibrated_directions([session.live_raw], cal)
    assert got["live_cell"] == int(ms.assign_cells(dirs)[0])
    assert np.allclose(got["dir"], dirs[0], atol=1e-6)
    assert abs(got["dev_pct"]) < 1.0            # a clean synthetic cloud
    assert got["flags"] & web.MAGPOSE_HAVE_QUAT
    assert got["flags"] & web.MAGPOSE_COLLECTING


def test_magpose_filled_cell_is_a_one_shot_delta():
    """The trick that keeps the JSON slow: the fast channel reports a cell the
    FIRST time a stored sample lights it, and never again."""
    from roomscan.sensors import SensorState
    session = ms.MagSweepSession()
    session.start()
    sensor_state = SensorState()
    cal = _cal()
    raw = _raw_from_body([[0.0, 0.0, 1.0]])[0]
    session.add(raw, t_us=1)
    a = _decode_magpose(web.build_magpose(session, cal, sensor_state, 1, stored=True))
    b = _decode_magpose(web.build_magpose(session, cal, sensor_state, 2, stored=True))
    assert a["filled_cell"] == a["live_cell"] >= 0
    assert b["filled_cell"] == -1
    # An un-stored (de-duplicated) sample can never claim to have filled anything.
    session.occupied.clear()
    c = _decode_magpose(web.build_magpose(session, cal, sensor_state, 3, stored=False))
    assert c["filled_cell"] == -1


def test_magpose_survives_a_tof_only_session():
    """No stream 9 -> `have_quat` clear, so the client shows the Steering
    placeholder. The HERO is unaffected, which is the whole reason it is the
    hero: it needs no orientation at all."""
    session, sensor_state = _pose_session(quat=None)
    got = _decode_magpose(web.build_magpose(session, _cal(), sensor_state, 1))
    assert not (got["flags"] & web.MAGPOSE_HAVE_QUAT)
    assert got["quat"] == (1.0, 0.0, 0.0, 0.0)      # identity placeholder
    assert math.isnan(got["dip_deg"])               # no gravity -> unknown, not a lie
    assert got["live_cell"] >= 0                    # the shell still works


def test_magpose_is_none_when_no_magnetometer_data_is_arriving():
    """The modal is a diagnostic: it must say *nothing is arriving* rather than
    animate a convincingly empty sphere."""
    from roomscan.sensors import SensorState
    assert web.build_magpose(ms.MagSweepSession(), _cal(), SensorState(), 1) is None


def test_magpose_marks_a_provisional_binning():
    from roomscan.sensors import SensorState
    session = ms.MagSweepSession()
    session.start()
    for i, v in enumerate(_raw_from_body(_sphere(80, seed=6))):
        session.add(v, t_us=i)
    got = _decode_magpose(web.build_magpose(session, None, SensorState(), 1))
    assert got["flags"] & web.MAGPOSE_PROVISIONAL
    assert math.isnan(got["dev_pct"])       # no calibration -> no colour claim


def test_json_binning_and_magpose_provisional_flag_agree_across_fit():
    """BUG-046 / issue #57 follow-up: the 5 Hz JSON `binning` label
    (`build_report`) and the 30 Hz MAGPOSE `MAGPOSE_PROVISIONAL` bit
    (`build_magpose`) must never contradict each other -- both have to derive
    from the SAME latch (`session.coverage_binning_kind()`), not two
    independent fresh guesses. Reproduces the primary first-ever-calibration
    flow (no saved calibration) across the Fit swap: before this fix, the bit
    re-guessed off `session.candidate`/`current` on every call, so it flipped
    to "not provisional" the instant Fit produced a candidate while the JSON
    label -- and the actual binning frame, frozen -- correctly stayed
    "provisional"."""
    from roomscan.sensors import SensorState

    session = ms.MagSweepSession()
    session.start()
    for i, v in enumerate(_raw_from_body(_sphere(300, seed=6))):
        session.add(v, t_us=i)

    def _check(current):
        report = ms.build_report(session, current)
        pose = _decode_magpose(web.build_magpose(session, current, SensorState(), 1))
        json_provisional = report["binning"] in ("provisional", "raw")
        pose_provisional = bool(pose["flags"] & web.MAGPOSE_PROVISIONAL)
        assert json_provisional == pose_provisional, (
            f"channels disagree: JSON binning={report['binning']!r} "
            f"(provisional={json_provisional}) vs MAGPOSE bit4 "
            f"(provisional={pose_provisional})")
        return report["binning"]

    before = _check(None)                 # no saved calibration: binning on provisional
    assert before == "provisional"

    session.stop()                        # fits a real candidate from the same cloud
    assert session.candidate is not None

    after = _check(None)                  # frame is still frozen on the provisional estimate
    assert after == before == "provisional"


def test_magpose_flags_a_stationary_board(monkeypatch):
    """A board on the desk must SAY so. The recorded trap: 255 stationary
    samples scored `std_pct 0.22%` against a x2.04-biased calibration, i.e. a
    still device can make every quality number look excellent."""
    from roomscan.sensors import SensorState
    clock = {"t": 1000.0}
    monkeypatch.setattr(ms.time, "monotonic", lambda: clock["t"])
    session = ms.MagSweepSession()
    session.start()
    raw = _raw_from_body([[0.0, 0.0, 1.0]])[0]
    for i in range(40):
        clock["t"] += 0.033           # ~1.3 s of real time at the 30 Hz pose tick
        session.add(tuple(raw + np.array([1e-6 * i, 0.0, 0.0])), t_us=i)
    got = _decode_magpose(web.build_magpose(session, _cal(), SensorState(), 1))
    assert got["flags"] & web.MAGPOSE_STATIONARY


def test_magpose_does_not_call_a_board_stationary_before_the_window_fills():
    """Two samples 30 ms apart prove nothing; the flag must stay clear until
    there is a real window to judge over."""
    from roomscan.sensors import SensorState
    session = ms.MagSweepSession()
    session.start()
    raw = _raw_from_body([[0.0, 0.0, 1.0]])[0]
    for i in range(40):
        session.add(tuple(raw + np.array([1e-9 * i, 0.0, 0.0])), t_us=i)
    got = _decode_magpose(web.build_magpose(session, _cal(), SensorState(), 1))
    assert not (got["flags"] & web.MAGPOSE_STATIONARY)


def test_magpose_dip_is_the_angle_between_field_and_gravity():
    """The scale-immune diagnostic: for a correct calibration this angle is a
    constant of the location, because both vectors are fixed in the room."""
    from roomscan.sensors import SensorState, quat_to_matrix
    session, sensor_state = _pose_session()
    got = _decode_magpose(web.build_magpose(session, _cal(), sensor_state, 1))
    g = quat_to_matrix(1.0, 0.0, 0.0, 0.0).T @ np.array([0.0, 0.0, -1.0])
    expect = math.degrees(math.acos(float(np.clip(np.dot(got["dir"], g), -1.0, 1.0))))
    assert got["dip_deg"] == pytest.approx(expect, abs=1e-3)
    assert np.allclose(got["gravity"], g, atol=1e-6)
    assert isinstance(sensor_state, SensorState)


# --- attitude_locked_error (2026-07-30, BUG-030 validation) -----------------
#
# The load-bearing case is `test_attitude_locked_ignores_a_walking_room_field`:
# a CORRECT calibration measured while the ambient field level drifts (the
# operator walking a room) must not be blamed for the room. `field_consistency`
# scores that same data "bad" -- which is what happened on the 2026-07-30 sweep.

def _walk(n=3600, rate_hz=30.0, seed=3, drift_pct=6.0):
    """(dirs, t_s, drift): a tumble-through-attitudes walk whose AMBIENT field
    level breathes slowly, the way a building's ferrous mass moves it."""
    t = np.arange(n) / rate_hz
    drift = 1.0 + (drift_pct / 100.0) * np.sin(2 * np.pi * t / 60.0)
    return _sphere(n, seed=seed), t, drift


def test_attitude_locked_ignores_a_walking_room_field():
    dirs, t, drift = _walk()
    raw = _raw_from_body(dirs * drift[:, None])
    cal = _cal()                                    # exactly right for this rig
    att = ms.attitude_locked_error(raw, t, cal)
    assert att["attitude_locked_pct"] < ms.FIELD_GOOD_PCT
    assert att["verdict"] == "good"
    # the drift is real and IS reported -- just not as calibration error
    assert att["spatial_std_ut"] > att["residual_std_ut"]
    # ...while the tumble-time metric condemns the very same good calibration
    assert ms.field_consistency(raw, cal)["verdict"] in ("marginal", "bad")


def test_attitude_locked_still_catches_a_hard_iron_error():
    dirs, t, drift = _walk()
    raw = _raw_from_body(dirs * drift[:, None])
    att = ms.attitude_locked_error(raw, t, _cal(offset=(0.0, 0.0, 0.0)))
    assert att["attitude_locked_pct"] > ms.FIELD_MARGINAL_PCT
    assert att["verdict"] == "bad"


def test_attitude_locked_says_unknown_rather_than_zero_on_one_attitude():
    """A rig held in a single pose has no attitude dependence to measure. That
    is not the same as measuring none, and must not read as a clean bill."""
    t = np.arange(600) / 30.0
    raw = _raw_from_body(np.tile([0.0, 0.0, 1.0], (600, 1)))
    att = ms.attitude_locked_error(raw, t, _cal())
    assert att["verdict"] == "unknown"
    assert att["attitude_locked_ut"] is None
    assert "cells" in att["reason"]


def test_attitude_locked_needs_matching_lengths_and_samples():
    dirs, t, _ = _walk(n=100)
    raw = _raw_from_body(dirs)
    assert ms.attitude_locked_error(raw, t[:50], _cal()) is None
    assert ms.attitude_locked_error(raw, t, None) is None
    assert ms.attitude_locked_error(raw[:5], t[:5], _cal()) is None
