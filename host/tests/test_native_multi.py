"""Multi-output transform coverage: mask selection, dtypes/shapes, determinism, and the
byte-identity tripwire against the legacy depth-only path (test_equivalence.py already
gates PC-vs-MCU; this gates multi-output-vs-single-output within the PC transform itself).
"""
import numpy as np
import pytest

from roomscan.native import Transform
from tests.golden import load_golden_pairs

pytestmark = pytest.mark.skipif(not Transform.available(), reason="native transform DLL not built")

_FLOAT_PLANES = ("depth", "reflectance", "confidence", "ambient")


def test_mask_selection_returns_requested_planes_with_shapes_and_dtypes():
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    with Transform(calib, outputs=_FLOAT_PLANES) as t:
        result = t.process(raw)

    assert set(result) == set(_FLOAT_PLANES)
    for name in _FLOAT_PLANES:
        arr = result[name]
        assert arr.shape == (42, 54), f"{name}: {arr.shape}"
        assert arr.dtype == np.float32, f"{name}: {arr.dtype}"
        assert np.isfinite(arr).all() or name != "depth"  # depth is finite on this fixture


def test_single_output_selection_returns_only_that_plane():
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    with Transform(calib, outputs=("reflectance",)) as t:
        result = t.process(raw)

    assert set(result) == {"reflectance"}
    assert result["reflectance"].shape == (42, 54)
    assert result["reflectance"].dtype == np.float32


def test_zapc_shape_and_dtype():
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    with Transform(calib, outputs=("zapc",)) as t:
        result = t.process(raw)

    assert set(result) == {"zapc"}
    zapc = result["zapc"]
    assert zapc.shape == (42, 54, 4)
    assert zapc.dtype == np.float32


def test_depth_and_zapc_together_uses_two_instances_without_error():
    # DEPTH|ZAPC together is only needed by Task 3's validation, not the live viewer, but the
    # shim must still support it correctly (second transform instance behind one handle).
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    with Transform(calib, outputs=("depth", "zapc")) as t:
        result = t.process(raw)

    assert set(result) == {"depth", "zapc"}
    assert result["depth"].shape == (42, 54)
    assert result["zapc"].shape == (42, 54, 4)


def test_determinism_across_fresh_instances():
    # Both instances start from the same just-reset TNR state processing the stream's true
    # first frame (see test_equivalence.py's docstring on why pairs[0] matters here) -- so two
    # independently-constructed instances fed the same raw frame must agree exactly.
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    a = Transform(calib, outputs=_FLOAT_PLANES).process(raw)
    b = Transform(calib, outputs=_FLOAT_PLANES).process(raw)

    for name in _FLOAT_PLANES:
        assert np.array_equal(a[name], b[name]), f"{name} differs across fresh instances"


def test_multi_output_depth_matches_legacy_single_output_depth():
    # Tripwire: if this ever diverges, capability negotiation for the added output streams
    # changed the depth pipeline itself -- investigate, don't relax this assert.
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    legacy_depth = Transform(calib, outputs=("depth",)).process(raw)["depth"]
    multi_depth = Transform(calib, outputs=_FLOAT_PLANES).process(raw)["depth"]

    assert np.array_equal(legacy_depth, multi_depth)


def test_outputs_must_be_non_empty():
    calib, _ = load_golden_pairs()
    with pytest.raises(ValueError):
        Transform(calib, outputs=())


def test_unknown_output_name_rejected():
    calib, _ = load_golden_pairs()
    with pytest.raises(ValueError):
        Transform(calib, outputs=("bogus",))
