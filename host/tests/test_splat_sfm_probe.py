"""Unit tests for the SfM-probe reduce logic (tools.splat_sfm_probe).

The COLMAP passes (extract/match/map) run on-box, not here -- CI has no COLMAP and
they are minutes each. These cover the pure reduce: which config the probe
recommends and how it phrases the lift, over synthetic per-config rows.
"""
from tools.splat_sfm_probe import _pick_best, _reco_note


def _cfg(name, *, ok=True, largest_ratio=0.0, points=0, subs=1, sizes=None):
    return {"config": name, "ok": ok, "largest_ratio": largest_ratio,
            "points3D": points, "n_submodels": subs,
            "submodel_sizes": sizes or [0], "registered_ratio": largest_ratio}


def test_pick_best_maximizes_registration_times_points():
    rows = [_cfg("seq", largest_ratio=0.16, points=36000),
            _cfg("exhaustive", largest_ratio=0.74, points=210000),
            _cfg("seq_overlap20", largest_ratio=0.30, points=60000)]
    assert _pick_best(rows)["config"] == "exhaustive"


def test_pick_best_ignores_failed_configs():
    rows = [_cfg("seq_loop", ok=False, largest_ratio=0.0, points=0),
            _cfg("seq", largest_ratio=0.16, points=36000)]
    assert _pick_best(rows)["config"] == "seq"


def test_pick_best_none_when_nothing_registered():
    rows = [_cfg("seq", ok=False), _cfg("exhaustive", ok=False)]
    assert _pick_best(rows) is None


def test_reco_note_reports_the_lift_over_the_sequential_baseline():
    best = _cfg("exhaustive", largest_ratio=0.74, points=210000)
    baseline = _cfg("seq", largest_ratio=0.16, points=36000, subs=4, sizes=[47, 30, 12, 8])
    note = _reco_note(best, baseline)
    assert "exhaustive" in note
    assert "74%" in note and "16%" in note      # both ratios stated
    assert "+58%" in note                        # the lift
    assert "4 sub-models" in note                # fragmentation surfaced


def test_reco_note_handles_no_usable_config():
    assert "no config" in _reco_note(None, None).lower()
