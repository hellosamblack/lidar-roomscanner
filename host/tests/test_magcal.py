import numpy as np
import pytest

from roomscan.magcal import MagCalibration, fit_ellipsoid


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


def _distorted_sphere(n=500, radius=45.0, offset=(5.0, -3.0, 2.0),
                      soft=((1.3, 0.1, 0.0), (0.1, 0.9, 0.05), (0.0, 0.05, 1.1)), seed=0):
    rng = np.random.default_rng(seed)
    dirs = rng.normal(size=(n, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    clean = dirs * radius                      # perfect sphere
    A = np.asarray(soft)
    raw = clean @ A.T + np.asarray(offset)     # apply soft-iron then hard-iron
    return raw, np.asarray(offset)


def test_fit_recovers_center_and_spherizes():
    raw, offset = _distorted_sphere()
    cal = fit_ellipsoid(raw)
    # center (hard-iron) recovered
    assert np.allclose(cal.offset, offset, atol=1.0)
    # calibrated samples have near-constant magnitude ~ field_ut
    norms = np.linalg.norm(np.array([cal.apply(r) for r in raw]), axis=1)
    assert np.std(norms) / np.mean(norms) < 0.02
    assert np.mean(norms) == pytest.approx(cal.field_ut, rel=0.05)


def test_fit_too_few_points_raises():
    with pytest.raises(ValueError):
        fit_ellipsoid(np.zeros((5, 3)))


def test_fit_degenerate_planar_raises():
    # Points confined to the z=0 plane make the shape matrix rank-deficient
    # (singular Q / non-positive-definite), exercising the degenerate-math
    # guards rather than the <20-point shape guard.
    rng = np.random.default_rng(3)
    theta = np.linspace(0.0, 2.0 * np.pi, 200, endpoint=False)
    planar = np.column_stack([45.0 * np.cos(theta) + 5.0,
                              45.0 * np.sin(theta) - 3.0,
                              np.zeros_like(theta)])
    with pytest.raises(ValueError):
        fit_ellipsoid(planar)


def test_fit_large_offset_succeeds():
    # Verify that a sphere with a hard-iron offset larger than the radius
    # (very common on real rigs) compiles and fits correctly.
    raw, offset = _distorted_sphere(radius=45.0, offset=(80.0, -40.0, -80.0))
    cal = fit_ellipsoid(raw)
    assert np.allclose(cal.offset, offset, atol=1.0)
    norms = np.linalg.norm(np.array([cal.apply(r) for r in raw]), axis=1)
    assert np.std(norms) / np.mean(norms) < 0.02
    assert np.mean(norms) == pytest.approx(cal.field_ut, rel=0.05)
