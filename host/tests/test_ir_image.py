"""Tests for roomscan.ir_image — pure array->image math for the IR reflectance panel."""
from __future__ import annotations

import numpy as np
import pytest

from roomscan.ir_image import ir_range, reflectance_to_rgb


def gradient(h: int = 42, w: int = 54) -> np.ndarray:
    """Horizontal gradient: brightness increases left->right, constant per column."""
    return np.tile(np.linspace(0.0, 100.0, w, dtype=np.float32), (h, 1))


# --- reflectance_to_rgb: shape / dtype / range -----------------------------------------


def test_output_shape_no_upscale():
    refl = gradient(42, 54)
    img = reflectance_to_rgb(refl)
    assert img.shape == (42, 54, 3)


def test_output_shape_with_upscale():
    refl = gradient(42, 54)
    img = reflectance_to_rgb(refl, upscale=4)
    assert img.shape == (42 * 4, 54 * 4, 3)


def test_output_dtype_uint8():
    refl = gradient()
    img = reflectance_to_rgb(refl)
    assert img.dtype == np.uint8


def test_output_value_range():
    refl = gradient()
    img = reflectance_to_rgb(refl)
    assert img.min() >= 0
    assert img.max() <= 255


# --- grayscale monotonicity -------------------------------------------------------------


def test_gray_monotonic_increasing_across_columns():
    refl = gradient(10, 20)
    img = reflectance_to_rgb(refl, colormap="gray")
    row = img[5, :, 0].astype(int)
    assert np.all(np.diff(row) >= 0)
    assert row[-1] > row[0]


def test_gray_r_equals_g_equals_b():
    refl = gradient()
    img = reflectance_to_rgb(refl, colormap="gray")
    assert np.array_equal(img[..., 0], img[..., 1])
    assert np.array_equal(img[..., 1], img[..., 2])


# --- turbo vs gray -----------------------------------------------------------------------


def test_turbo_differs_from_gray():
    refl = gradient()
    img_gray = reflectance_to_rgb(refl, colormap="gray")
    img_turbo = reflectance_to_rgb(refl, colormap="turbo")
    assert not np.array_equal(img_gray, img_turbo)


def test_turbo_uint8_in_range():
    refl = gradient()
    img = reflectance_to_rgb(refl, colormap="turbo")
    assert img.dtype == np.uint8
    assert img.min() >= 0
    assert img.max() <= 255


# --- NaN / inf handling --------------------------------------------------------------------


def test_nonfinite_values_do_not_raise_and_map_to_dark_end():
    refl = gradient(10, 10)
    refl[0, 0] = np.nan
    refl[1, 1] = np.inf
    refl[2, 2] = -np.inf
    img = reflectance_to_rgb(refl, colormap="gray")
    assert img[0, 0, 0] == 0
    assert img[1, 1, 0] == 0
    assert img[2, 2, 0] == 0


def test_all_nan_does_not_raise():
    refl = np.full((5, 5), np.nan, dtype=np.float32)
    img = reflectance_to_rgb(refl)
    assert img.shape == (5, 5, 3)
    assert np.all(img == 0)


# --- flat / all-equal input ---------------------------------------------------------------


def test_flat_input_no_divide_by_zero():
    refl = np.full((5, 5), 42.0, dtype=np.float32)
    img = reflectance_to_rgb(refl)
    assert img.shape == (5, 5, 3)
    assert np.all(np.isfinite(img))


# --- frozen range vs auto-range ------------------------------------------------------------


def test_frozen_range_differs_from_auto_when_range_differs():
    refl = gradient()
    img_auto = reflectance_to_rgb(refl, colormap="gray")
    img_frozen = reflectance_to_rgb(refl, colormap="gray", vmin=-1000.0, vmax=1000.0)
    assert not np.array_equal(img_auto, img_frozen)


def test_frozen_range_matches_auto_when_equal_to_computed_range():
    refl = gradient()
    lo, hi = ir_range(refl)
    img_auto = reflectance_to_rgb(refl, colormap="gray")
    img_frozen = reflectance_to_rgb(refl, colormap="gray", vmin=lo, vmax=hi)
    assert np.array_equal(img_auto, img_frozen)


# --- upscale nearest-neighbor blocks --------------------------------------------------------


def test_upscale_produces_exact_nearest_neighbor_blocks():
    refl = gradient(10, 10)
    img = reflectance_to_rgb(refl, colormap="gray", upscale=3)
    src = reflectance_to_rgb(refl, colormap="gray", upscale=1)
    # block at source pixel (2, 4) should be a uniform 3x3 block equal to src[2, 4]
    block = img[6:9, 12:15]
    assert np.all(block == src[2, 4])
    # another block, source pixel (7, 1)
    block2 = img[21:24, 3:6]
    assert np.all(block2 == src[7, 1])


def test_upscale_one_returns_native_size():
    refl = gradient(6, 8)
    img = reflectance_to_rgb(refl, upscale=1)
    assert img.shape == (6, 8, 3)


# --- unknown colormap ------------------------------------------------------------------------


def test_unknown_colormap_raises_value_error():
    refl = gradient()
    with pytest.raises(ValueError):
        reflectance_to_rgb(refl, colormap="jet")


# --- ir_range ----------------------------------------------------------------------------------


def test_ir_range_ignores_nonfinite():
    refl = np.array([1.0, 2.0, 3.0, np.nan, np.inf, -np.inf, 4.0, 5.0])
    lo, hi = ir_range(refl, lo_pct=0.0, hi_pct=100.0)
    assert lo == pytest.approx(1.0)
    assert hi == pytest.approx(5.0)


def test_ir_range_vmin_le_vmax_always():
    cases = [
        np.array([1.0, 2.0, 3.0]),
        np.full(5, 7.0),
        np.full(5, np.nan),
        np.array([np.nan, np.inf, -np.inf]),
        np.array([]),
    ]
    for arr in cases:
        lo, hi = ir_range(arr)
        assert lo <= hi


def test_ir_range_all_equal_degenerate():
    refl = np.full((4, 4), 3.5)
    lo, hi = ir_range(refl)
    assert lo == pytest.approx(3.5)
    assert hi > lo


def test_ir_range_all_nan_degenerate():
    refl = np.full((4, 4), np.nan)
    lo, hi = ir_range(refl)
    assert lo == 0.0
    assert hi == 1.0
