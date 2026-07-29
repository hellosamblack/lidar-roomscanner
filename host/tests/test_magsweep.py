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
import types

import numpy as np
import pytest

from roomscan import magsweep as ms
from roomscan.protocol import Frame, FrameHeader, FrameType, StreamId
from roomscan import web
from roomscan.magcal import MagCalibration, fit_ellipsoid
from roomscan.sensors import AXIS_CONVENTION

FIELD = 50.0
# The real rig's hard-iron offset (host/mag_cal.json, fitted 2026-07-15). Larger
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


def test_guidance_names_the_face_and_the_angle():
    body = _sphere()
    raw = _raw_from_body(body[body[:, 0] > 0.4])       # covered the Top face only
    regions = ms.empty_regions(ms.coverage_stats(_cells(raw, _cal()))["counts"])
    text = ms.guidance_text(regions, live_dir=[1.0, 0.0, 0.0])
    assert "Bottom" in text
    assert "175" in text          # the hole is (near-)antipodal to where we are
    assert ms.guidance_text([]) == "Full sphere covered — every orientation has samples."


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


def test_session_binning_calibration_precedence():
    s = ms.MagSweepSession()
    saved = _cal()
    assert s.binning_calibration(None) is None          # nothing at all yet
    assert s.binning_calibration(saved) is saved
    s.start()
    for i, v in enumerate(_raw_from_body(_sphere(400, seed=6))):
        s.add(v, t_us=i)
    assert s.binning_calibration(None) is not None      # provisional from the cloud
    s.stop()
    assert s.binning_calibration(saved) is s.candidate  # candidate wins once fitted


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
    """Collecting, fitting, previewing and discarding are pure observation:
    the gravity-alignment rotation, the point-cloud bytes, `fused_quat()` and
    the loaded calibration must all be bit-identical afterwards. (Save is the
    one deliberate exception -- covered by
    `test_install_mag_calibration_hot_reloads_every_consumer`.)"""
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

    state = _ws_state(tmp_path, mag_cal=existing, sensor_state=sensor_state)
    state.fusion = fusion
    _route(state, {"type": "magcal", "action": "start"})
    _fill(state)
    _route(state, {"type": "magcal", "action": "stop"})
    assert state.magcal_session.candidate is not None            # a real fit happened
    ms.build_report(state.magcal_session, existing, view="candidate")
    _route(state, {"type": "magcal", "action": "discard"})

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
