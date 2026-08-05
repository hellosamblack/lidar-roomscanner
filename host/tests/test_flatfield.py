import json

import numpy as np
import pytest

from roomscan.config import ViewerConfig
from roomscan.flatfield import (
    FlatField, FlatFieldSet, build_flatfield, RANGING_PRECISION, RANGING_AMBIENT)
from roomscan.native import Transform
from roomscan.pipeline import TransformStage
from roomscan.protocol import Frame, FrameHeader, FrameType, StreamId
from tests.golden import load_golden_pairs

needs_dll = pytest.mark.skipif(not Transform.available(), reason="native transform DLL not built")
H, W = 42, 54


def _fpn_gain(seed=0):
    """A high-frequency per-zone gain ripple (what real FPN looks like)."""
    rng = np.random.default_rng(seed)
    g = 1.0 + 0.15 * rng.standard_normal((H, W))
    return np.clip(g, 0.5, 1.6)


def test_build_recovers_and_flattens_a_held_out_frame():
    gain = _fpn_gain()
    # "panned uniform surface": uniform albedo * fixed per-zone gain, N frames.
    frames = np.stack([gain * (1.0 + 0.01 * i) for i in range(40)])
    ff = build_flatfield(frames)
    assert ff.shape == (H, W)
    assert abs(ff.gain.mean() - 1.0) < 1e-3          # unit-mean: level-preserving

    # A fresh uniform frame carries the raw FPN ripple; correction should flatten it.
    fresh = gain * 123.0
    corrected = ff.apply(fresh.astype(np.float32))
    cv_in = fresh.std() / fresh.mean()
    cv_out = corrected.std() / corrected.mean()
    assert cv_out < cv_in / 4.0                       # ripple cut by >4x


def test_apply_passes_through_on_shape_mismatch():
    ff = FlatField(gain=np.ones((H, W), np.float32))
    other = np.arange(100, dtype=np.float32).reshape(10, 10)
    out = ff.apply(other)
    assert out.shape == (10, 10)
    assert np.array_equal(out, other)                 # untouched, not corrupted


def test_apply_is_multiplicative_and_preserves_dtype():
    gain = _fpn_gain(1).astype(np.float32)
    ff = FlatField(gain=gain)
    plane = np.full((H, W), 50.0, np.float32)
    out = ff.apply(plane)
    assert out.dtype == np.float32
    assert np.allclose(out, plane * gain)


def test_save_load_roundtrip(tmp_path):
    ff = build_flatfield(np.stack([_fpn_gain(2) for _ in range(10)]), note="unit-test")
    p = tmp_path / "map.npz"
    ff.save(p)
    back = FlatField.load(p)
    assert back is not None
    assert np.allclose(back.gain, ff.gain)
    assert back.meta.get("note") == "unit-test"


def test_load_is_tolerant(tmp_path):
    assert FlatField.load(tmp_path / "missing.npz") is None
    bad = tmp_path / "bad.npz"
    bad.write_bytes(b"not a real npz")
    assert FlatField.load(bad) is None


def test_load_rejects_nonpositive_gain(tmp_path):
    p = tmp_path / "neg.npz"
    np.savez(p, gain=np.zeros((H, W), np.float32), meta=json.dumps({}))
    assert FlatField.load(p) is None                  # zero/negative gain -> invalid


def test_load_configured_default_is_none():
    assert FlatField.load_configured(ViewerConfig()) is None   # disabled by default


def test_load_configured_reads_path(tmp_path):
    ff = build_flatfield(np.stack([_fpn_gain(3) for _ in range(8)]))
    p = tmp_path / "cfg_map.npz"
    ff.save(p)
    cfg = ViewerConfig(flatfield_path=str(p))
    loaded = FlatField.load_configured(cfg)
    assert loaded is not None and np.allclose(loaded.gain, ff.gain)


def test_config_field_roundtrips(tmp_path):
    p = tmp_path / "roomscan.toml"
    ViewerConfig(flatfield_path="maps/ff.npz").save(p)
    assert ViewerConfig.load(p).flatfield_path == "maps/ff.npz"
    # default stays None (empty string in TOML -> None, like `port`)
    ViewerConfig().save(p)
    assert ViewerConfig.load(p).flatfield_path is None


def test_stage_without_flatfield_is_unchanged():
    stage = TransformStage(outputs=("reflectance",))
    assert stage._flatfield is None                    # opt-in; default off
    assert not stage.has_flatfield


# --- mode-aware map selection (FlatFieldSet) --------------------------------

def _const_ff(v):
    return FlatField(gain=np.full((H, W), v, np.float32))


def test_flatfieldset_selects_by_ranging_mode():
    prec, amb = _const_ff(1.1), _const_ff(0.9)
    s = FlatFieldSet({RANGING_PRECISION: prec, RANGING_AMBIENT: amb})
    assert not s.is_empty
    assert s.for_mode(RANGING_PRECISION) is prec
    assert s.for_mode(RANGING_AMBIENT) is amb


def test_flatfieldset_unknown_mode_falls_back_to_default():
    default, prec = _const_ff(1.0), _const_ff(1.1)
    s = FlatFieldSet({RANGING_PRECISION: prec}, default=default)
    assert s.for_mode(None) is default              # replay: no device mode known
    assert s.for_mode(RANGING_AMBIENT) is default   # no ambient map -> default
    assert s.for_mode(RANGING_PRECISION) is prec


def test_flatfieldset_empty_returns_none():
    s = FlatFieldSet()
    assert s.is_empty
    assert s.for_mode(RANGING_PRECISION) is None


def test_flatfieldset_load_configured_mode_and_legacy(tmp_path):
    prec = build_flatfield(np.stack([_fpn_gain(1) * 100 for _ in range(5)]))
    amb = build_flatfield(np.stack([_fpn_gain(2) * 100 for _ in range(5)]))
    pp, ap = tmp_path / "prec.npz", tmp_path / "amb.npz"
    prec.save(pp)
    amb.save(ap)
    s = FlatFieldSet.load_configured(ViewerConfig(
        flatfield_precision_path=str(pp), flatfield_ambient_path=str(ap)))
    assert np.allclose(s.for_mode(RANGING_PRECISION).gain, prec.gain)
    assert np.allclose(s.for_mode(RANGING_AMBIENT).gain, amb.gain)

    # legacy single path becomes the fallback default for every mode
    lp = tmp_path / "legacy.npz"
    prec.save(lp)
    s2 = FlatFieldSet.load_configured(ViewerConfig(flatfield_path=str(lp)))
    assert s2.for_mode(RANGING_AMBIENT).gain.shape == (H, W)
    assert FlatFieldSet.load_configured(ViewerConfig()).is_empty


def test_stage_flatfieldset_selection_and_toggle():
    prec, amb = _const_ff(1.1), _const_ff(0.9)
    stage = TransformStage(outputs=("reflectance",),
                           flatfield=FlatFieldSet({RANGING_PRECISION: prec, RANGING_AMBIENT: amb}))
    assert stage.has_flatfield
    stage.set_ranging_mode(RANGING_PRECISION)
    assert stage._active_flatfield() is prec
    stage.set_ranging_mode(RANGING_AMBIENT)
    assert stage._active_flatfield() is amb
    stage.flatfield_enabled = False              # "Uncalibrated"
    assert stage._active_flatfield() is None
    stage.flatfield_enabled = True
    stage.set_ranging_mode(None)                 # unknown mode, no default
    assert stage._active_flatfield() is None


def test_config_flatfield_mode_paths_roundtrip_calibration_table(tmp_path):
    p = tmp_path / "roomscan.toml"
    ViewerConfig(flatfield_precision_path="a.npz", flatfield_ambient_path="b.npz",
                 flatfield_enabled=False).save(p)
    # persisted under [calibration], not [viewer]
    text = p.read_text()
    assert "[calibration]" in text
    assert "flatfield_precision_path = \"a.npz\"" in text
    viewer_block, _, calib_block = text.partition("[calibration]")
    assert "flatfield_enabled" not in viewer_block   # host/sensor keys leave [viewer]
    assert "flatfield_enabled = false" in calib_block

    c = ViewerConfig.load(p)
    assert c.flatfield_precision_path == "a.npz"
    assert c.flatfield_ambient_path == "b.npz"
    assert c.flatfield_enabled is False
    # defaults: mode paths None (empty string -> None), flatfield_enabled True
    ViewerConfig().save(p)
    c2 = ViewerConfig.load(p)
    assert c2.flatfield_precision_path is None
    assert c2.flatfield_ambient_path is None
    assert c2.flatfield_enabled is True


def test_config_legacy_viewer_flatfield_path_still_loads(tmp_path):
    # An old file that predates the [calibration] table carried the key under
    # [viewer]; load() must still honour it (back-compat).
    p = tmp_path / "roomscan.toml"
    p.write_text('[viewer]\ncolor = "reflectance"\nflatfield_path = "legacy.npz"\n')
    c = ViewerConfig.load(p)
    assert c.flatfield_path == "legacy.npz"
    assert c.color == "reflectance"
    assert c.flatfield_enabled is True   # absent -> default


@needs_dll
def test_pipeline_applies_mode_selected_flatfield():
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]
    prec_gain = _fpn_gain(1).astype(np.float32)
    amb_gain = _fpn_gain(2).astype(np.float32)
    fset = FlatFieldSet({RANGING_PRECISION: FlatField(gain=prec_gain),
                         RANGING_AMBIENT: FlatField(gain=amb_gain)})

    def _run(mode, enabled=True):
        stage = TransformStage(outputs=("depth", "reflectance"),
                               flatfield=fset, flatfield_enabled=enabled)
        stage.set_ranging_mode(mode)
        stage.feed(Frame(FrameHeader(FrameType.DATA, StreamId.CALIB, 0, 1, 0, 0, 0, len(calib)), calib))
        _, out = stage.feed(
            Frame(FrameHeader(FrameType.DATA, StreamId.RAW_3DMD, 0, 2, 0, 0, 0, len(raw)), raw))
        return out["reflectance"]

    base = _run(RANGING_PRECISION, enabled=False)          # toggle off => untouched
    assert np.allclose(_run(RANGING_PRECISION), base * prec_gain, rtol=1e-5, atol=1e-4)
    assert np.allclose(_run(RANGING_AMBIENT), base * amb_gain, rtol=1e-5, atol=1e-4)


@needs_dll
def test_pipeline_applies_flatfield_to_reflectance():
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    def _run(flatfield):
        stage = TransformStage(outputs=("depth", "reflectance"), flatfield=flatfield)
        stage.feed(Frame(FrameHeader(FrameType.DATA, StreamId.CALIB, 0, 1, 0, 0, 0, len(calib)), calib))
        _, outputs = stage.feed(
            Frame(FrameHeader(FrameType.DATA, StreamId.RAW_3DMD, 0, 2, 0, 0, 0, len(raw)), raw))
        return outputs

    base = _run(None)["reflectance"]
    gain = _fpn_gain(7).astype(np.float32)
    corrected = _run(FlatField(gain=gain))["reflectance"]

    assert np.allclose(corrected, base * gain, rtol=1e-5, atol=1e-4)
