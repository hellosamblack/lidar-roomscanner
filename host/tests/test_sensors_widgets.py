import numpy as np

from roomscan.sensors_widgets import (
    render_compass,
    render_sensors_overlay,
    render_sparkline,
)


def test_compass_shape_and_dtype():
    img = render_compass(0.0, size=120)
    assert img.shape == (120, 120, 3)
    assert img.dtype == np.uint8


def test_compass_needle_moves_with_heading():
    # The needle tip pixel region differs between N (0°) and E (90°).
    north = render_compass(0.0, size=120)
    east = render_compass(90.0, size=120)
    assert not np.array_equal(north, east)


def test_sparkline_shape():
    img = render_sparkline(np.linspace(1000.0, 1010.0, 50), width=200, height=50)
    assert img.shape == (50, 200, 3)
    assert img.dtype == np.uint8


def test_sparkline_empty_is_safe():
    img = render_sparkline(np.array([]), width=200, height=50)
    assert img.shape == (50, 200, 3)  # no exception, flat baseline


def test_sparkline_rising_trend_nonflat():
    flat = render_sparkline(np.full(50, 5.0), width=200, height=50)
    rising = render_sparkline(np.linspace(0.0, 10.0, 50), width=200, height=50)
    assert not np.array_equal(flat, rising)


def test_sensors_overlay_shape_and_dtype():
    img = render_sensors_overlay(123.0, np.linspace(1000, 1010, 30),
                                 np.linspace(20, 22, 30))
    assert img.ndim == 3 and img.shape[2] == 3 and img.dtype == np.uint8


def test_sensors_overlay_no_data_is_safe():
    # empty histories + invalid heading must not raise (startup, pre-IMU/env)
    img = render_sensors_overlay(0.0, np.zeros(2), np.zeros(2), heading_valid=False)
    assert img.dtype == np.uint8


def test_sensors_overlay_reflects_heading_change():
    a = render_sensors_overlay(0.0, np.linspace(1000, 1010, 30), np.linspace(20, 22, 30))
    b = render_sensors_overlay(90.0, np.linspace(1000, 1010, 30), np.linspace(20, 22, 30))
    assert not np.array_equal(a, b)      # compass needle + heading text moved
