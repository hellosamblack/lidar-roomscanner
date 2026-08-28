"""Runtime behaviour of the frame-tagged types (#225, BUG-026).

The static half lives in `tests/pyright_negative/frames_negative.py` (asserted by
`test_pyright_strict.py`). This file pins what the tags do when Python actually
runs them: the numerics match the bare-matrix path exactly, and a frame mismatch
that an untyped caller could construct fails loudly instead of silently.
"""
import numpy as np
import pytest

from roomscan.domain import CV, Body, Cloud, CvWorld, Rotation, View, World
from roomscan.sensors import (
    R_BODY_FROM_CV,
    R_CVWORLD_FROM_WORLD,
    T_CV_TO_BODY,
    T_WORLD_TO_CV,
    body_to_world_rotation,
    display_frame_rotation,
    display_rotation,
    quat_to_matrix,
)
from roomscan.server.session import rotate_cloud, rotate_points, view_frame_rotation, view_rotation

_Q = (0.92388, 0.0, 0.38268, 0.0)   # ~45 deg about body Y
_PTS = np.array([[1.0, 2.0, 3.0], [-4.0, 0.5, 2.0], [0.0, 0.0, 1.0]], dtype=np.float32)


def test_tagged_constants_wrap_the_documented_matrices():
    assert R_BODY_FROM_CV.matrix is T_CV_TO_BODY
    assert R_CVWORLD_FROM_WORLD.matrix is T_WORLD_TO_CV
    assert (R_BODY_FROM_CV.dst, R_BODY_FROM_CV.src) == (Body, CV)
    assert (R_CVWORLD_FROM_WORLD.dst, R_CVWORLD_FROM_WORLD.src) == (CvWorld, World)


def test_display_frame_rotation_is_the_sandwich_with_frames_on():
    tagged = display_frame_rotation(_Q)
    assert tagged is not None
    assert (tagged.dst, tagged.src) == (CvWorld, CV)
    expect = T_WORLD_TO_CV @ quat_to_matrix(*_Q) @ T_CV_TO_BODY
    assert np.allclose(tagged.matrix, expect)
    # The bare-matrix function is the same numbers, byte for byte.
    assert np.array_equal(display_rotation(_Q), tagged.matrix)
    assert display_frame_rotation(None) is None
    assert display_rotation(None) is None


def test_apply_matches_matrix_product_and_keeps_grids():
    rot = display_frame_rotation(_Q)
    assert rot is not None
    flat = rot @ Cloud(_PTS, CV)
    assert flat.frame is CvWorld
    assert np.allclose(flat.xyz, (rot.matrix @ _PTS.T).T, atol=1e-5)
    grid = rot.apply(Cloud(_PTS.reshape(1, 3, 3), CV))     # (h, w, 3) surface grid
    assert grid.xyz.shape == (1, 3, 3)
    assert np.allclose(grid.xyz.reshape(-1, 3), flat.xyz)
    assert len(grid) == 3


def test_inverse_round_trips():
    rot = display_frame_rotation(_Q)
    assert rot is not None
    back = rot.inverse() @ (rot @ Cloud(_PTS, CV))
    assert back.frame is CV
    assert np.allclose(back.xyz, _PTS, atol=1e-5)
    ident = rot.inverse() @ rot
    assert (ident.dst, ident.src) == (CV, CV)
    assert np.allclose(ident.matrix, np.eye(3), atol=1e-12)


def test_runtime_rejects_frame_mismatch_an_untyped_caller_could_build():
    """Pyright cannot see an untyped call site; the identity check can."""
    with pytest.raises(ValueError, match="frame mismatch"):
        R_CVWORLD_FROM_WORLD @ R_BODY_FROM_CV          # sandwich with its middle missing
    with pytest.raises(ValueError, match="frame mismatch"):
        R_CVWORLD_FROM_WORLD @ Cloud(_PTS, Body)      # wrong source frame
    with pytest.raises(ValueError, match="frame mismatch"):
        R_BODY_FROM_CV.apply(Cloud(_PTS, World))
    # The right composition, for contrast.
    assert (R_CVWORLD_FROM_WORLD @ body_to_world_rotation(_Q) @ R_BODY_FROM_CV).src is CV


def test_shape_validation():
    with pytest.raises(ValueError):
        Cloud(np.zeros((4, 2)), CV)
    with pytest.raises(ValueError):
        Cloud(np.zeros(3), CV)
    with pytest.raises(ValueError):
        Rotation(np.eye(4), CvWorld, CV)


def test_view_frame_rotation_matches_bare_view_rotation():
    grav = display_frame_rotation(_Q)
    assert grav is not None
    for mode in ("world", "fpv", "mirror"):
        typed = view_frame_rotation(grav, mode)
        bare = view_rotation(grav.matrix, mode)
        assert typed is not None and bare is not None
        assert (typed.dst, typed.src) == (View, CV)
        assert np.array_equal(typed.matrix, bare)
    # ToF-only session: no orientation.
    assert view_frame_rotation(None, "world") is None
    assert view_frame_rotation(None, "fpv") is None
    mirror = view_frame_rotation(None, "mirror")
    assert mirror is not None and np.allclose(mirror.matrix, np.diag([-1.0, 1.0, 1.0]))


def test_rotate_cloud_matches_rotate_points_and_no_copy_without_rotation():
    grav = display_frame_rotation(_Q)
    rot = view_frame_rotation(grav, "fpv")
    assert rot is not None
    out = rotate_cloud(Cloud(_PTS, CV), rot)
    assert out.frame is View
    assert out.xyz.dtype == np.float32
    assert np.array_equal(out.xyz, rotate_points(_PTS, rot.matrix))
    same = rotate_cloud(Cloud(_PTS, CV), None)
    assert same.frame is View and same.xyz is _PTS    # sensor frame IS the view; no allocation
