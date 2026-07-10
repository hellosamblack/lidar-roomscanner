import numpy as np
import pytest

from roomscan.magcal import MagCalibration


def test_apply_offset_and_matrix():
    cal = MagCalibration(offset=(1.0, 2.0, 3.0),
                         matrix=((2.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.5)),
                         field_ut=50.0)
    out = cal.apply((4.0, 4.0, 5.0))
    assert np.allclose(out, [2.0 * 3.0, 1.0 * 2.0, 0.5 * 2.0])  # matrix @ (raw - offset)


def test_save_load_roundtrip(tmp_path):
    cal = MagCalibration(offset=(0.1, -0.2, 0.3),
                         matrix=((1.0, 0.01, 0.0), (0.01, 1.0, 0.0), (0.0, 0.0, 1.0)),
                         field_ut=48.5)
    p = tmp_path / "mag_cal.json"
    cal.save(p)
    back = MagCalibration.load(p)
    assert back is not None
    assert back.offset == pytest.approx(cal.offset)
    assert np.allclose(back.matrix, cal.matrix)
    assert back.field_ut == pytest.approx(cal.field_ut)


def test_load_missing_returns_none(tmp_path):
    assert MagCalibration.load(tmp_path / "nope.json") is None


def test_load_corrupt_returns_none(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{not json", encoding="utf-8")
    assert MagCalibration.load(p) is None
