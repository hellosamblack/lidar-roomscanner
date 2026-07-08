import numpy as np

from roomscan.colors import turbo


def test_output_shape_and_bounds():
    zn = np.linspace(0.0, 1.0, 256)
    rgb = turbo(zn)
    assert rgb.shape == (256, 3)
    assert rgb.min() >= 0.0 and rgb.max() <= 1.0


def test_low_end_blue_dominant_high_end_red_dominant():
    lo = turbo(np.array([0.1]))[0]
    hi = turbo(np.array([0.9]))[0]
    assert lo[2] > lo[0]          # blue beats red at the cold end
    assert hi[0] > hi[2]          # red beats blue at the hot end
    assert hi[0] > lo[0] and lo[2] > hi[2]   # red rises, blue falls across the range


def test_out_of_range_input_clipped():
    rgb = turbo(np.array([-1.0, 2.0]))
    assert np.allclose(rgb[0], turbo(np.array([0.0]))[0])
    assert np.allclose(rgb[1], turbo(np.array([1.0]))[0])


def test_midpoint_is_greenish():
    r, g, b = turbo(np.array([0.5]))[0]
    assert g > r and g > b
