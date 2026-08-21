import numpy as np

from roomscan.colors import amber, turbo


def _ir_thermal_rgb(v: int) -> tuple[int, int, int]:
    """Reference port of the CrowPanel firmware's `ir_thermal_rgb565` pre-565
    RGB (`CrowPanelProp/main/prop_ui.c`), the ramp `roomscan.colors.amber`
    replicates. Kept independent of the LUT under test so the test can't pass by
    sharing the implementation."""
    if v < 96:
        return (v * 255 // 95, 0, 0)
    if v < 192:
        return (255, (v - 96) * 255 // 95, 0)
    return (255, 255, (v - 192) * 255 // 63)


def test_amber_shape_and_bounds():
    zn = np.linspace(0.0, 1.0, 256)
    rgb = amber(zn)
    assert rgb.shape == (256, 3)
    assert rgb.min() >= 0.0 and rgb.max() <= 1.0


def test_amber_matches_crowpanel_ir_ramp_at_every_level():
    # The whole point of indexing the 8-bit LUT is a per-level match to the
    # panel's own IR tone, not just at the endpoints (#215).
    zn = np.arange(256) / 255.0
    rgb = amber(zn)
    for v in range(256):
        expected = np.array(_ir_thermal_rgb(v), dtype=np.float64) / 255.0
        assert np.allclose(rgb[v], expected), (v, rgb[v], expected)


def test_amber_endpoints_black_red_yellow_white():
    black, red, yellow, white = amber(np.array([0.0, 96 / 255, 192 / 255, 1.0]))
    assert np.allclose(black, [0, 0, 0])
    assert np.allclose(red, [1, 0, 0])
    assert np.allclose(yellow, [1, 1, 0])
    assert np.allclose(white, [1, 1, 1])


def test_amber_out_of_range_input_clipped():
    rgb = amber(np.array([-1.0, 2.0]))
    assert np.allclose(rgb[0], amber(np.array([0.0]))[0])
    assert np.allclose(rgb[1], amber(np.array([1.0]))[0])


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
