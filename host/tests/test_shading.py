"""Tests for cloud_colors near-contrast modes (roomscan.shading)."""
import numpy as np

from roomscan.shading import FAR_GREY, cloud_colors


def _path_len(colors):
    """Total distance travelled through color space for depth-ordered points --
    a proxy for how much of the colormap the set spans (robust to turbo's
    non-monotone per-channel shape, unlike a per-channel range)."""
    if len(colors) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(colors, axis=0), axis=1).sum())


def _ramp(n=20):
    """A monotone depth ramp 0.5..3.0 m; vals == z (depth-mode coloring)."""
    z = np.linspace(0.5, 3.0, n)
    return z, z


def test_empty_input():
    out = cloud_colors(np.array([]), np.array([]), mode="window")
    assert out.shape == (0, 3)


def test_off_is_linear_full_range():
    z, vals = _ramp()
    out = cloud_colors(vals, z, mode="off")
    assert out.shape == (len(z), 3)
    assert np.all((out >= 0) & (out <= 1))
    # nearest and farthest map to the colormap ends (blue-ish vs red-ish) -> differ a lot
    assert np.abs(out[0] - out[-1]).max() > 0.3


def test_window_greys_far_and_colors_near():
    z, vals = _ramp()
    out = cloud_colors(vals, z, mode="window", cutoff_m=1.5)
    near = z <= 1.5
    far = ~near
    # far points are all the flat grey
    assert np.allclose(out[far], np.array(FAR_GREY))
    # near points are NOT grey and span a wide color range (full colormap over the near set)
    assert not np.allclose(out[near], np.array(FAR_GREY))
    near_out = out[near]
    assert np.abs(near_out[0] - near_out[-1]).max() > 0.3


def test_window_gives_near_more_color_range_than_off():
    # A person (dense near cluster 0.6-0.9 m) in front of a wall (2.4-2.6 m).
    person = np.linspace(0.6, 0.9, 40)
    wall = np.linspace(2.4, 2.6, 40)
    z = np.concatenate([person, wall])
    off = cloud_colors(z, z, mode="off")
    win = cloud_colors(z, z, mode="window", cutoff_m=1.5)
    # the PERSON's points traverse far more of the colormap under window mode
    assert _path_len(win[:40]) > 2.0 * _path_len(off[:40])


def test_window_all_far_is_all_grey():
    z = np.linspace(2.0, 3.0, 10)
    out = cloud_colors(z, z, mode="window", cutoff_m=1.5)
    assert np.allclose(out, np.array(FAR_GREY))


def test_emphasis_expands_near_band_vs_linear():
    z, vals = _ramp(50)
    lin = cloud_colors(vals, z, mode="emphasis", emphasis=0.0)   # p=1 -> linear
    strong = cloud_colors(vals, z, mode="emphasis", emphasis=1.0)
    off = cloud_colors(vals, z, mode="off")
    # emphasis=0 reproduces the linear/off mapping
    assert np.allclose(lin, off)
    # under strong emphasis the near HALF of points traverses more colormap than linear
    near = slice(0, 25)
    assert _path_len(strong[near]) > _path_len(off[near])


def test_equalize_ties_share_a_color():
    # a flat wall (many identical depths) + a few near points
    z = np.concatenate([np.full(30, 2.5), np.linspace(0.6, 0.9, 10)])
    out = cloud_colors(z, z, mode="equalize")
    wall = out[:30]
    assert np.allclose(wall, wall[0])          # all identical-depth points share one color
    # the near ramp is spread across colors
    near = out[30:]
    assert np.abs(near[0] - near[-1]).max() > 0.2


def test_equalize_stretches_dense_region():
    # dense near surface (fine gradient) + sparse far points
    near = np.linspace(0.60, 0.75, 60)   # 60 pts over 15 cm (the "face")
    far = np.linspace(2.0, 3.0, 6)       # 6 pts over 1 m (background)
    z = np.concatenate([near, far])
    eq = cloud_colors(z, z, mode="equalize")
    off = cloud_colors(z, z, mode="off")
    # equalize allocates color by point count, so the dense near face traverses
    # far more of the colormap than under linear depth normalization
    assert _path_len(eq[:60]) > 3.0 * _path_len(off[:60])
