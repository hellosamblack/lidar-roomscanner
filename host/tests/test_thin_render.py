"""Golden-vector + value tests for `roomscan.thin_render` (#194).

Deliberately fake-backed: constructing a real `OffscreenRenderer` costs ~4 s and
a GL context, and a *second* one in the same process aborts the interpreter --
which would take the whole pytest run down. Every renderer here either has a
failed init (open3d import poisoned) or is a plain fake object. Image scenes
are queued to the render thread like everything else (#197 review finding 3),
but a failed-init renderer's `_drain()` still services them without ever
touching Filament, so nothing in this file ever touches the real thing.
"""

import struct
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from roomscan import thin_render as tr
from roomscan.thin_render import (
    DEFAULT_MODE, PITCH_LIMIT_DEG, THIN_HEADER, THIN_HEIGHT, THIN_MODES,
    THIN_TAG, THIN_WIDTH, ZOOM_MAX, ZOOM_MIN,
    ThinCamera, ThinRenderUnavailable, ThinRenderer, ThinScene,
    _letterbox, image_scene, pack_thin_frame, points_scene, rgba_to_rgb565,
    unpack_mesh_scene,
)
from roomscan import web


# --------------------------------------------------------------------------
# 1 -- rgba_to_rgb565
# --------------------------------------------------------------------------


def _u16(data: bytes) -> int:
    assert len(data) == 2
    return struct.unpack("<H", data)[0]


@pytest.mark.parametrize("rgb,expect", [
    ((255, 0, 0), 0xF800),   # pure red   -- top 5 bits
    ((0, 255, 0), 0x07E0),   # pure green -- middle 6 bits
    ((0, 0, 255), 0x001F),   # pure blue  -- bottom 5 bits
    ((255, 255, 255), 0xFFFF),
    ((0, 0, 0), 0x0000),
])
def test_rgb565_exact_channel_values(rgb, expect):
    """Hand-computed hex per primary: catches both bit-order (which bits each
    channel occupies) and channel-order (R/B swap) errors, neither of which a
    round-trip test could see."""
    img = np.array([[list(rgb)]], dtype=np.uint8)
    out = rgba_to_rgb565(img)
    assert len(out) == 2
    assert _u16(out) == expect


def test_rgb565_byte_order_is_little_endian():
    """The u16 value alone cannot see an endianness bug -- assert the bytes.
    Red is 0xF800, so on the wire it must be 00 F8, not F8 00."""
    red = rgba_to_rgb565(np.array([[[255, 0, 0]]], dtype=np.uint8))
    assert red == b"\x00\xf8"
    blue = rgba_to_rgb565(np.array([[[0, 0, 255]]], dtype=np.uint8))
    assert blue == b"\x1f\x00"


def test_rgb565_drops_alpha():
    rgb = np.array([[[10, 200, 90], [255, 0, 0]]], dtype=np.uint8)
    rgba = np.concatenate(
        [rgb, np.array([[[7], [250]]], dtype=np.uint8)], axis=2)
    assert rgba.shape == (1, 2, 4)
    assert rgba_to_rgb565(rgba) == rgba_to_rgb565(rgb)


def test_rgb565_mask_and_shift_not_just_extremes():
    """0x08 >> 3 == 1 must survive in the red field; 0x07 must truncate to 0.
    Both endpoints of the primaries test pass under a naive `r << 8`, this does
    not."""
    keep = rgba_to_rgb565(np.array([[[0x08, 0, 0]]], dtype=np.uint8))
    assert _u16(keep) == 0x0800          # red field == 1
    lost = rgba_to_rgb565(np.array([[[0x07, 0, 0]]], dtype=np.uint8))
    assert _u16(lost) == 0x0000
    # green keeps 6 bits: 0x04 survives, 0x03 does not
    assert _u16(rgba_to_rgb565(np.array([[[0, 0x04, 0]]], np.uint8))) == 0x0020
    assert _u16(rgba_to_rgb565(np.array([[[0, 0x03, 0]]], np.uint8))) == 0x0000
    # blue keeps 5 bits: 0x08 survives, 0x07 does not
    assert _u16(rgba_to_rgb565(np.array([[[0, 0, 0x08]]], np.uint8))) == 0x0001
    assert _u16(rgba_to_rgb565(np.array([[[0, 0, 0x07]]], np.uint8))) == 0x0000


def test_rgb565_output_length_is_two_bytes_per_pixel():
    img = np.zeros((7, 5, 3), dtype=np.uint8)
    assert len(rgba_to_rgb565(img)) == 7 * 5 * 2


def test_rgb565_row_major_order():
    img = np.zeros((2, 2, 3), dtype=np.uint8)
    img[0, 0] = (255, 0, 0)
    img[0, 1] = (0, 255, 0)
    img[1, 0] = (0, 0, 255)
    img[1, 1] = (255, 255, 255)
    out = rgba_to_rgb565(img)
    assert [_u16(out[i:i + 2]) for i in range(0, 8, 2)] == [
        0xF800, 0x07E0, 0x001F, 0xFFFF]


@pytest.mark.parametrize("bad", [
    np.zeros((4, 4), dtype=np.uint8),            # 2-D
    np.zeros((4, 4, 2), dtype=np.uint8),         # 2 channels
    np.zeros((4, 4, 5), dtype=np.uint8),         # 5 channels
    np.zeros((4,), dtype=np.uint8),              # 1-D
])
def test_rgb565_rejects_wrong_shape(bad):
    with pytest.raises(ValueError):
        rgba_to_rgb565(bad)


# --------------------------------------------------------------------------
# 2 -- pack_thin_frame
# --------------------------------------------------------------------------


def test_pack_thin_frame_header_bytes_are_hand_computed():
    payload = b"\x00" * (4 * 3 * 2)
    frame = pack_thin_frame(payload, 4, 3)
    assert frame[0:4] == b"\x01\x00\x00\x00"     # u32 tag = 1, LE
    assert frame[4:6] == b"\x04\x00"             # u16 width = 4, LE
    assert frame[6:8] == b"\x03\x00"             # u16 height = 3, LE
    assert frame[8:] == payload


def test_pack_thin_frame_default_size_and_total_length():
    payload = bytes(THIN_WIDTH * THIN_HEIGHT * 2)
    frame = pack_thin_frame(payload)
    assert len(frame) == 8 + THIN_WIDTH * THIN_HEIGHT * 2
    assert frame[0:8] == struct.pack("<IHH", 1, 480, 480)


def test_pack_thin_frame_header_parses_back():
    frame = pack_thin_frame(bytes(THIN_WIDTH * THIN_HEIGHT * 2))
    tag, w, h = THIN_HEADER.unpack(frame[:8])
    assert (tag, w, h) == (1, 480, 480)
    assert (tag, w, h) == struct.unpack("<IHH", frame[:8])
    assert THIN_TAG == 1 and THIN_HEADER.size == 8


@pytest.mark.parametrize("nbytes", [0, 4 * 3 * 2 - 1, 4 * 3 * 2 + 1])
def test_pack_thin_frame_rejects_wrong_payload_size(nbytes):
    with pytest.raises(ValueError):
        pack_thin_frame(b"\x00" * nbytes, 4, 3)


def test_rgb565_feeds_pack_thin_frame_exactly():
    img = np.zeros((3, 4, 3), dtype=np.uint8)
    frame = pack_thin_frame(rgba_to_rgb565(img), 4, 3)
    assert len(frame) == 8 + 24


# --------------------------------------------------------------------------
# 2b -- pack_thin_jpeg (THIN_FRAME_JPEG, tag 2, #197; v2 seq'd layout #202)
# --------------------------------------------------------------------------


def test_pack_thin_jpeg_header_bytes_are_hand_computed():
    payload = b"\xff\xd8\xff\xd9"   # fake JFIF: SOI + EOI markers
    frame = tr.pack_thin_jpeg(payload, 320, 240, seq=0x0102)
    assert frame[0:4] == b"\x02\x00\x00\x00"     # u32 tag = 2, LE
    assert frame[4:6] == b"\x40\x01"             # u16 width = 320 (0x0140), LE
    assert frame[6:8] == b"\xf0\x00"             # u16 height = 240 (0x00F0), LE
    assert frame[8:12] == b"\x02\x01\x00\x00"    # u32 seq = 0x0102, LE
    assert frame[12:16] == b"\x04\x00\x00\x00"   # u32 payload_len = 4, LE
    assert frame[16:] == payload
    assert len(frame) == 16 + len(payload)


def test_thin_tag_jpeg_and_header_constants():
    # 16-byte v2 layout with `seq` (#202, CrowPanel bandwidth spec) -- `seq`
    # is what a v2 client echoes in `thin_ready` to grant send credit.
    assert tr.THIN_TAG_JPEG == 2
    assert tr.THIN_HEADER_JPEG.size == 16
    assert tr.THIN_HEADER_JPEG.format == "<IHHII"


def test_pack_thin_jpeg_header_parses_back():
    payload = b"\x01\x02\x03"
    frame = tr.pack_thin_jpeg(payload, 480, 480, seq=7)
    tag, w, h, seq, plen = tr.THIN_HEADER_JPEG.unpack(frame[:16])
    assert (tag, w, h, seq, plen) == (2, 480, 480, 7, 3)
    assert frame[16:] == payload


def test_pack_thin_jpeg_empty_payload_is_a_valid_zero_length_frame():
    frame = tr.pack_thin_jpeg(b"", 4, 4)
    assert len(frame) == 16
    tag, w, h, seq, plen = tr.THIN_HEADER_JPEG.unpack(frame)
    assert (tag, w, h, seq, plen) == (2, 4, 4, 0, 0)


def test_pack_thin_jpeg_seq_wraps_at_32_bits_instead_of_raising():
    frame = tr.pack_thin_jpeg(b"", 4, 4, seq=0x1_0000_0001)
    _tag, _w, _h, seq, _plen = tr.THIN_HEADER_JPEG.unpack(frame)
    assert seq == 1


# --------------------------------------------------------------------------
# 2c -- jpeg_available / encode_jpeg_frame (#197)
# --------------------------------------------------------------------------


def test_jpeg_available_is_true_with_the_real_dependency_installed():
    assert tr.jpeg_available() is True


def test_jpeg_available_is_false_when_simplejpeg_is_poisoned(monkeypatch):
    """Mirrors `broken_renderer`'s technique for `open3d`: poisoning
    `sys.modules` makes the deferred `import simplejpeg` fail without
    actually uninstalling the real dependency."""
    monkeypatch.setitem(sys.modules, "simplejpeg", None)
    assert tr.jpeg_available() is False


def test_encode_jpeg_frame_raises_import_error_when_simplejpeg_is_poisoned(monkeypatch):
    """Reintroduces the defect this guards against: without the deferred
    import + poison test, a caller could believe JPEG encoding "worked" on a
    host where it cannot -- the #197 requirement 5 graceful-degrade path."""
    monkeypatch.setitem(sys.modules, "simplejpeg", None)
    with pytest.raises(ImportError):
        tr.encode_jpeg_frame(np.zeros((4, 4, 3), dtype=np.uint8))


def _gradient(n: int) -> np.ndarray:
    """An n x n RGB gradient with real structure (not flat) so JPEG DCT error
    is meaningful rather than a degenerate all-one-value block."""
    img = np.zeros((n, n, 3), dtype=np.uint8)
    xs = np.linspace(0, 255, n)
    ys = np.linspace(0, 255, n)
    img[:, :, 0] = xs[None, :]
    img[:, :, 1] = ys[:, None]
    img[:, :, 2] = (xs[None, :] + ys[:, None]) / 2
    return img.astype(np.uint8)


def test_encode_jpeg_frame_round_trips_within_a_quantitative_error_bound():
    """Not just 'decodes' -- a quantitative similarity bound at the default
    quality, per the #197 test requirement. simplejpeg is a real dependency
    here (not mocked), so this exercises the actual codec."""
    import simplejpeg

    img = _gradient(64)
    encoded = tr.encode_jpeg_frame(img, quality=75)
    assert encoded[:2] == b"\xff\xd8"          # JFIF SOI marker
    decoded = simplejpeg.decode_jpeg(encoded, colorspace="rgb")
    assert decoded.shape == img.shape
    mae = float(np.mean(np.abs(decoded.astype(np.int16) - img.astype(np.int16))))
    assert mae < 8.0
    # a real JPEG at q75 is meaningfully smaller than the raw RGB it encodes
    assert len(encoded) < img.nbytes


def test_encode_jpeg_frame_default_quality_is_75():
    assert tr.DEFAULT_JPEG_QUALITY == 75


def test_encode_jpeg_frame_is_baseline_420_for_the_p4_hardware_decoder():
    """The CrowPanel spec pins baseline JPEG, YCbCr 4:2:0 -- that is what the
    ESP32-P4's `esp_driver_jpeg` block consumes. Asserted on the actual JFIF
    bytes: SOF0 (baseline, 0xFFC0) present with the luma component's sampling
    factor 0x22 (2x2 = 4:2:0), and no SOF2 (progressive) marker. simplejpeg's
    own default is 4:4:4 (0x11), so this catches the argument being dropped.
    """
    encoded = tr.encode_jpeg_frame(_gradient(64), quality=75)
    assert b"\xff\xc2" not in encoded            # no progressive SOF2
    sof0 = encoded.find(b"\xff\xc0")
    assert sof0 != -1                            # baseline SOF0 present
    # SOF0 payload: len(2) precision(1) height(2) width(2) ncomp(1), then per
    # component: id(1) sampling(1) qtable(1). Luma sampling 0x22 = 4:2:0.
    ncomp = encoded[sof0 + 9]
    assert ncomp == 3
    assert encoded[sof0 + 11] == 0x22


# --------------------------------------------------------------------------
# 2d -- resize_nearest (host-side downscale for a negotiated resolution)
# --------------------------------------------------------------------------


def test_resize_nearest_is_a_noop_at_matching_size():
    img = _ramp(6)
    out = tr.resize_nearest(img, 6, 6)
    assert out is img


def test_resize_nearest_shrinks_to_the_requested_shape():
    img = _ramp(8)
    out = tr.resize_nearest(img, 4, 4)
    assert out.shape == (4, 4, 3)
    assert out.dtype == np.uint8


def test_resize_nearest_corners_map_to_source_corners():
    """The four corners of a downscale must still be the source's own
    corners -- proves this samples the source rather than, say, cropping or
    interpolating a blend that would land somewhere else."""
    img = _ramp(8)
    out = tr.resize_nearest(img, 4, 4)
    np.testing.assert_array_equal(out[0, 0], img[0, 0])
    np.testing.assert_array_equal(out[-1, -1], img[-1, -1])
    np.testing.assert_array_equal(out[0, -1], img[0, -1])
    np.testing.assert_array_equal(out[-1, 0], img[-1, 0])


# --------------------------------------------------------------------------
# 3 -- ThinCamera
# --------------------------------------------------------------------------


def test_camera_defaults():
    cam = ThinCamera()
    assert (cam.yaw, cam.pitch, cam.zoom) == (30.0, 20.0, 1.0)
    assert cam.mode == DEFAULT_MODE == "point_cloud"


def test_camera_pitch_clamps_at_exactly_the_limit():
    cam = ThinCamera()
    cam.apply_orbit(dpitch=10_000.0)
    assert cam.pitch == PITCH_LIMIT_DEG == 89.0
    cam.apply_orbit(dpitch=-10_000.0)
    assert cam.pitch == -89.0


def test_camera_zoom_clamps_to_range():
    cam = ThinCamera()
    cam.apply_orbit(dzoom=1000.0)
    assert cam.zoom == ZOOM_MAX == 8.0
    cam.apply_orbit(dzoom=-1000.0)
    assert cam.zoom == ZOOM_MIN == 0.25


def test_camera_yaw_wraps_modulo_360():
    cam = ThinCamera(yaw=350.0)
    cam.apply_orbit(dyaw=20.0)
    assert cam.yaw == 10.0            # not 370
    cam2 = ThinCamera(yaw=5.0)
    cam2.apply_orbit(dyaw=-20.0)
    assert cam2.yaw == 345.0          # not -15
    cam3 = ThinCamera(yaw=0.0)
    cam3.apply_orbit(dyaw=720.0)
    assert cam3.yaw == 0.0


def test_camera_deltas_accumulate():
    """Assert the running value after each call, so a 'last write wins' bug
    (self.yaw = dyaw) is visible -- a single-call test cannot see it."""
    cam = ThinCamera(yaw=0.0, pitch=0.0, zoom=1.0)
    cam.apply_orbit(dyaw=10.0, dpitch=5.0, dzoom=0.5)
    assert (cam.yaw, cam.pitch, cam.zoom) == (10.0, 5.0, 1.5)
    cam.apply_orbit(dyaw=10.0, dpitch=5.0, dzoom=0.5)
    assert (cam.yaw, cam.pitch, cam.zoom) == (20.0, 10.0, 2.0)
    cam.apply_orbit(dyaw=10.0, dpitch=-2.0, dzoom=-0.25)
    assert (cam.yaw, cam.pitch, cam.zoom) == (30.0, 8.0, 1.75)


@pytest.mark.parametrize("kwargs", [
    {"dyaw": float("nan")}, {"dpitch": float("nan")}, {"dzoom": float("nan")},
    {"dyaw": float("inf")}, {"dpitch": float("-inf")}, {"dzoom": float("inf")},
    {"dyaw": 5.0, "dpitch": float("nan"), "dzoom": 0.5},
])
def test_camera_non_finite_delta_leaves_all_three_untouched(kwargs):
    cam = ThinCamera(yaw=12.0, pitch=-3.0, zoom=2.0)
    cam.apply_orbit(**kwargs)
    assert (cam.yaw, cam.pitch, cam.zoom) == (12.0, -3.0, 2.0)


@pytest.mark.parametrize("mode", list(THIN_MODES))
def test_camera_set_mode_accepts_each_valid_mode(mode):
    cam = ThinCamera()
    assert cam.set_mode(mode) is True
    assert cam.mode == mode


@pytest.mark.parametrize("junk", ["", "POINT_CLOUD", "wireframe", None, 3])
def test_camera_set_mode_rejects_junk_without_changing_mode(junk):
    cam = ThinCamera(mode="slam")
    assert cam.set_mode(junk) is False
    assert cam.mode == "slam"


# --------------------------------------------------------------------------
# 4 -- ThinCamera.eye
# --------------------------------------------------------------------------


def test_eye_distance_follows_radius_over_zoom():
    center = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    cam = ThinCamera(yaw=37.0, pitch=11.0, zoom=1.0)
    eye = cam.eye(center, 4.0)
    assert np.linalg.norm(eye - center) == pytest.approx(4.0 * 2.5, rel=1e-5)
    cam.zoom = 2.0
    assert np.linalg.norm(cam.eye(center, 4.0) - center) == pytest.approx(5.0, rel=1e-5)


def test_eye_moves_closer_as_zoom_increases():
    center = np.zeros(3, dtype=np.float32)
    dists = [np.linalg.norm(ThinCamera(zoom=z).eye(center, 2.0) - center)
             for z in (0.5, 1.0, 2.0, 4.0)]
    assert dists == sorted(dists, reverse=True)
    assert dists[0] > dists[-1] * 5


def test_eye_yaw_0_vs_90_moves_between_named_axes():
    """yaw=0 puts the eye on +Z with x==0; yaw=90 puts it on +X with z==0.
    Asserting only 'they differ' would pass for a sign flip or a swapped pair."""
    center = np.zeros(3, dtype=np.float32)
    e0 = ThinCamera(yaw=0.0, pitch=0.0, zoom=1.0).eye(center, 2.0)
    e90 = ThinCamera(yaw=90.0, pitch=0.0, zoom=1.0).eye(center, 2.0)
    d = 2.0 * 2.5
    assert e0[0] == pytest.approx(0.0, abs=1e-5)
    assert e0[1] == pytest.approx(0.0, abs=1e-5)
    assert e0[2] == pytest.approx(d, rel=1e-5)
    assert e90[0] == pytest.approx(d, rel=1e-5)
    assert e90[1] == pytest.approx(0.0, abs=1e-5)
    assert e90[2] == pytest.approx(0.0, abs=1e-5)


def test_eye_positive_pitch_raises_the_eye():
    center = np.zeros(3, dtype=np.float32)
    up = ThinCamera(yaw=0.0, pitch=45.0).eye(center, 2.0)
    down = ThinCamera(yaw=0.0, pitch=-45.0).eye(center, 2.0)
    assert up[1] == pytest.approx(5.0 * np.sin(np.radians(45.0)), rel=1e-5)
    assert down[1] == pytest.approx(-up[1], rel=1e-5)


def test_eye_is_offset_from_center_not_absolute():
    center = np.array([10.0, -4.0, 7.0], dtype=np.float32)
    eye = ThinCamera(yaw=0.0, pitch=0.0).eye(center, 1.0)
    np.testing.assert_allclose(eye, [10.0, -4.0, 9.5], rtol=1e-5)


def test_eye_zoom_below_minimum_is_floored():
    center = np.zeros(3, dtype=np.float32)
    cam = ThinCamera(zoom=0.0)          # bypassing apply_orbit's clamp
    d = np.linalg.norm(cam.eye(center, 1.0) - center)
    assert d == pytest.approx(2.5 / ZOOM_MIN, rel=1e-5)


# --------------------------------------------------------------------------
# 5 -- ThinScene.bounds / point_count
# --------------------------------------------------------------------------


def test_bounds_center_and_radius_hand_computed():
    pts = np.array([[0.0, 0.0, 0.0],
                    [2.0, 0.0, 0.0],
                    [0.0, 2.0, 0.0],
                    [0.0, 0.0, 2.0]], dtype=np.float32)
    scene = points_scene(pts, np.zeros_like(pts))
    center, radius = scene.bounds()
    np.testing.assert_allclose(center, [1.0, 1.0, 1.0], rtol=1e-6)
    # diagonal of the 2x2x2 AABB is sqrt(12); radius is half that
    assert radius == pytest.approx(np.sqrt(3.0), rel=1e-6)


def test_bounds_asymmetric_box():
    pts = np.array([[-1.0, 4.0, 0.0], [3.0, 6.0, 0.0]], dtype=np.float32)
    center, radius = ThinScene(kind="points", points=pts).bounds()
    np.testing.assert_allclose(center, [1.0, 5.0, 0.0], rtol=1e-6)
    assert radius == pytest.approx(np.hypot(4.0, 2.0) * 0.5, rel=1e-6)


def test_bounds_single_point_gets_the_floor_radius():
    scene = ThinScene(kind="points", points=np.array([[5.0, 5.0, 5.0]], np.float32))
    center, radius = scene.bounds()
    np.testing.assert_allclose(center, [5.0, 5.0, 5.0])
    assert radius == 1e-3


@pytest.mark.parametrize("pts", [None, np.zeros((0, 3), dtype=np.float32)])
def test_bounds_empty_is_the_unit_ball_fallback(pts):
    center, radius = ThinScene(kind="points", points=pts).bounds()
    np.testing.assert_array_equal(center, np.zeros(3))
    assert radius == 1.0


def test_point_count():
    assert ThinScene(kind="points").point_count == 0
    assert ThinScene(kind="points", points=np.zeros((0, 3))).point_count == 0
    assert ThinScene(kind="points", points=np.zeros((17, 3))).point_count == 17
    assert points_scene(np.zeros((5, 3)), np.zeros((5, 3))).point_count == 5


def test_scene_constructors_set_kind_and_dtype():
    p = points_scene([[1, 2, 3]], [[0.5, 0.5, 0.5]], generation="g1")
    assert p.kind == "points" and p.generation == "g1"
    assert p.points.dtype == np.float32 and p.colors.dtype == np.float32
    m = tr.mesh_scene([[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                      np.zeros((3, 3)), [[0, 1, 2]])
    assert m.kind == "mesh" and m.triangles.dtype == np.uint32
    i = image_scene(np.zeros((2, 2, 3), np.uint8), generation=7)
    assert i.kind == "image" and i.generation == 7 and i.points is None


# --------------------------------------------------------------------------
# 6 -- unpack_mesh_scene (against the real web.pack_mesh)
# --------------------------------------------------------------------------


def _packet(*, nw_v, nw_c, nw_t, w_v, w_c, w_t, mesh_seq=42,
            decimated=False, wall_mode="solid"):
    z3 = np.zeros((0, 3), dtype=np.float64)
    return SimpleNamespace(
        non_wall_verts=np.asarray(nw_v, np.float64),
        non_wall_colors=np.asarray(nw_c, np.float64),
        non_wall_tris=np.asarray(nw_t, np.int32).reshape(-1, 3),
        wall_verts=np.asarray(w_v, np.float64).reshape(-1, 3) if len(w_v) else z3,
        wall_colors=np.asarray(w_c, np.float64).reshape(-1, 3) if len(w_c) else z3,
        wall_tris=np.asarray(w_t, np.int32).reshape(-1, 3),
        floor_pts=z3,
        floor_lines=np.zeros((0, 2), dtype=np.int64),
        mesh_seq=mesh_seq, source_vertex_count=len(nw_v),
        decimated=decimated, wall_mode=wall_mode)


NW_V = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
NW_C = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
NW_T = [[0, 1, 2]]


def test_unpack_mesh_scene_roundtrips_values():
    pkt = _packet(nw_v=NW_V, nw_c=NW_C, nw_t=NW_T,
                  w_v=[], w_c=[], w_t=np.zeros((0, 3), np.int32),
                  mesh_seq=1234)
    scene = unpack_mesh_scene(web.pack_mesh(pkt), generation="gen-a")
    assert scene is not None and scene.kind == "mesh"
    np.testing.assert_allclose(scene.points, NW_V, atol=1e-6)
    np.testing.assert_allclose(scene.colors, NW_C, atol=1e-6)
    np.testing.assert_array_equal(scene.triangles, np.array(NW_T, np.uint32))
    assert scene.meta["mesh_seq"] == 1234
    assert scene.meta["flags"] == 0
    assert scene.generation == "gen-a"
    assert scene.point_count == 3


def test_unpack_mesh_scene_reindexes_wall_triangles_by_non_wall_vertex_count():
    """The merge's real failure mode: wall indices are submesh-local, so wall
    tri index i must become i + len(non_wall_verts) or the walls reference the
    wrong vertices."""
    w_v = [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0], [1.0, 1.0, 1.0]]
    w_c = [[0.5, 0.5, 0.5]] * 4
    w_t = [[0, 1, 2], [1, 2, 3]]
    pkt = _packet(nw_v=NW_V, nw_c=NW_C, nw_t=NW_T,
                  w_v=w_v, w_c=w_c, w_t=w_t, mesh_seq=9, wall_mode="split")
    scene = unpack_mesh_scene(web.pack_mesh(pkt))
    assert scene is not None
    # vertices are non-wall first, then wall
    np.testing.assert_allclose(scene.points, np.array(NW_V + w_v), atol=1e-6)
    np.testing.assert_allclose(scene.colors, np.array(NW_C + w_c), atol=1e-6)
    n = len(NW_V)
    assert n == 3
    expect = np.array(NW_T + [[i + n for i in tri] for tri in w_t], np.uint32)
    np.testing.assert_array_equal(scene.triangles, expect)
    np.testing.assert_array_equal(scene.triangles[1:], [[3, 4, 5], [4, 5, 6]])
    # every index still addresses a real vertex, and the wall ones address walls
    assert scene.triangles.max() == len(scene.points) - 1
    assert scene.meta["flags"] == 2          # walls_split
    assert scene.meta["mesh_seq"] == 9


def test_unpack_mesh_scene_flags_carry_decimated_bit():
    pkt = _packet(nw_v=NW_V, nw_c=NW_C, nw_t=NW_T, w_v=[], w_c=[],
                  w_t=np.zeros((0, 3), np.int32), decimated=True,
                  wall_mode="split")
    scene = unpack_mesh_scene(web.pack_mesh(pkt))
    assert scene.meta["flags"] == 3          # decimated | walls_split


def test_unpack_mesh_scene_truncated_returns_none():
    pkt = _packet(nw_v=NW_V, nw_c=NW_C, nw_t=NW_T, w_v=[], w_c=[],
                  w_t=np.zeros((0, 3), np.int32))
    buf = web.pack_mesh(pkt)
    assert unpack_mesh_scene(buf) is not None            # control
    for cut in (36 + 4, 36 + 12, len(buf) - 1):
        assert unpack_mesh_scene(buf[:cut]) is None


@pytest.mark.parametrize("payload", [b"", b"\x00" * 35, None])
def test_unpack_mesh_scene_short_or_missing_payload_returns_none(payload):
    assert unpack_mesh_scene(payload) is None


def test_unpack_mesh_scene_empty_mesh_returns_none():
    empty = _packet(nw_v=np.zeros((0, 3)), nw_c=np.zeros((0, 3)),
                    nw_t=np.zeros((0, 3), np.int32), w_v=[], w_c=[],
                    w_t=np.zeros((0, 3), np.int32))
    assert unpack_mesh_scene(web.pack_mesh(empty)) is None


# --------------------------------------------------------------------------
# 7 -- _letterbox, and the unavailable-renderer path (no GL context)
# --------------------------------------------------------------------------


def _ramp(n: int) -> np.ndarray:
    """n x n image whose pixel (y, x) is (10*y+x, 200-y, 3*x) -- every pixel
    distinguishable, so a transpose or an off-by-one block is visible."""
    img = np.zeros((n, n, 3), dtype=np.uint8)
    for y in range(n):
        for x in range(n):
            img[y, x] = (10 * y + x, 200 - y, 3 * x)
    return img


def test_letterbox_centres_source_and_is_blocky():
    src = _ramp(7)                       # 480 // 7 == 68 -> 476x476, 2px border
    out = _letterbox(src, THIN_WIDTH, THIN_HEIGHT)
    assert out.shape == (480, 480, 3)
    assert out.dtype == np.uint8
    scale, y0, x0 = 68, 2, 2
    # top-left of the upscaled image sits at (2, 2) and equals src[0, 0]
    np.testing.assert_array_equal(out[y0, x0], src[0, 0])
    # nearest-neighbour: the whole 68x68 block is one source pixel...
    np.testing.assert_array_equal(out[y0 + scale - 1, x0 + scale - 1], src[0, 0])
    # ...and the very next row/column is the next source pixel (no blending)
    np.testing.assert_array_equal(out[y0 + scale, x0], src[1, 0])
    np.testing.assert_array_equal(out[y0, x0 + scale], src[0, 1])
    # a mid-image sample, hand-indexed
    np.testing.assert_array_equal(out[y0 + 3 * scale + 5, x0 + 4 * scale + 9],
                                  src[3, 4])
    # bottom-right source pixel lands at the far corner of the letterbox
    np.testing.assert_array_equal(out[y0 + 7 * scale - 1, x0 + 7 * scale - 1],
                                  src[6, 6])


def test_letterbox_border_is_background():
    src = _ramp(7)
    out = _letterbox(src, THIN_WIDTH, THIN_HEIGHT)
    assert out[0].max() == 0 and out[1].max() == 0        # top border rows
    assert out[478].max() == 0 and out[479].max() == 0    # bottom border rows
    assert out[:, 0].max() == 0 and out[:, 479].max() == 0
    assert out[2:478, 2:478].max() > 0                    # the image itself


def test_letterbox_expands_greyscale_and_drops_alpha():
    grey = np.arange(9, dtype=np.uint8).reshape(3, 3) * 20
    out = _letterbox(grey, 12, 12)
    assert out.shape == (12, 12, 3)
    np.testing.assert_array_equal(out[0, 0], [0, 0, 0])
    np.testing.assert_array_equal(out[0, 4], [20, 20, 20])
    rgba = np.dstack([_ramp(3), np.full((3, 3, 1), 99, np.uint8)])
    np.testing.assert_array_equal(_letterbox(rgba, 12, 12),
                                  _letterbox(_ramp(3), 12, 12))


def test_letterbox_empty_source_is_all_background():
    out = _letterbox(np.zeros((0, 4, 3), np.uint8), 8, 8)
    assert out.shape == (8, 8, 3) and out.max() == 0


# -- the failed-init renderer ------------------------------------------------


@pytest.fixture
def broken_renderer(monkeypatch):
    """A real `ThinRenderer` whose deferred `import open3d` fails.

    Poisoning `sys.modules["open3d"]` makes `import open3d` raise ImportError
    ("import of open3d halted; None in sys.modules") on the render thread, so we
    exercise the genuine `_run` failure path without ever creating an
    `OffscreenRenderer` (which costs ~4 s, and aborts the process on a second
    instance)."""
    ThinRenderer.reset_singleton()
    monkeypatch.setitem(sys.modules, "open3d", None)
    r = ThinRenderer(width=THIN_WIDTH, height=THIN_HEIGHT)
    try:
        yield r
    finally:
        r.close()
        ThinRenderer.reset_singleton()


def test_failed_init_reports_unavailable(broken_renderer):
    with pytest.raises(ThinRenderUnavailable):
        broken_renderer.ensure_available(timeout=5.0)
    assert broken_renderer.available is False


def test_failed_init_serves_the_failure_to_submitted_futures(broken_renderer):
    fut = broken_renderer.submit(points_scene(np.zeros((1, 3)), np.zeros((1, 3))),
                                 ThinCamera())
    with pytest.raises(ThinRenderUnavailable):
        fut.result(timeout=5.0)
    with pytest.raises(ThinRenderUnavailable):
        broken_renderer.render(
            points_scene(np.zeros((1, 3)), np.zeros((1, 3))),
            ThinCamera(), timeout=5.0)


def test_image_scenes_resolve_without_a_render_context(broken_renderer):
    """`kind="image"` is a numpy upscale, so IR mode must still work on a host
    with no offscreen context at all."""
    assert broken_renderer.available is False
    src = _ramp(7)
    frame = broken_renderer.submit(image_scene(src), ThinCamera()).result(timeout=5.0)
    assert len(frame) == 8 + 480 * 480 * 2
    assert frame[:8] == struct.pack("<IHH", 1, 480, 480)
    # pixel (2, 2) is src[0, 0], as RGB565 little-endian
    px = 2 * 480 + 2
    expect = rgba_to_rgb565(src[0:1, 0:1])
    assert frame[8 + px * 2:8 + px * 2 + 2] == expect
    assert frame[8:10] == b"\x00\x00"      # top-left corner is background


def test_image_scene_unpacked_returns_the_array(broken_renderer):
    rgb = broken_renderer.submit(image_scene(_ramp(7)), ThinCamera(),
                                 pack=False).result(timeout=5.0)
    assert isinstance(rgb, np.ndarray)
    assert rgb.shape == (480, 480, 3) and rgb.dtype == np.uint8


# --- fmt="jpeg" / negotiated resolution through `submit` (#197) ------------
#
# Image scenes are resolved INLINE by `submit` (no render thread), so they
# are also the cheapest way to exercise the fmt/quality/out_width/out_height
# plumbing end-to-end without a real OffscreenRenderer.


def test_submit_jpeg_format_produces_a_tag2_frame(broken_renderer):
    frame = broken_renderer.submit(image_scene(_ramp(7)), ThinCamera(),
                                   fmt="jpeg", quality=80,
                                   seq=42).result(timeout=5.0)
    tag, w, h, seq, plen = tr.THIN_HEADER_JPEG.unpack_from(frame, 0)
    assert tag == tr.THIN_TAG_JPEG
    assert (w, h) == (480, 480)
    assert seq == 42
    assert plen == len(frame) - tr.THIN_HEADER_JPEG.size
    import simplejpeg
    decoded = simplejpeg.decode_jpeg(
        frame[tr.THIN_HEADER_JPEG.size:], colorspace="rgb")
    assert decoded.shape == (480, 480, 3)


def test_submit_raw_format_is_the_default_and_unchanged(broken_renderer):
    """No `fmt=` kwarg at all must be byte-identical to the pre-#197 frame."""
    a = broken_renderer.submit(image_scene(_ramp(7)), ThinCamera()).result(timeout=5.0)
    b = broken_renderer.submit(image_scene(_ramp(7)), ThinCamera(),
                               fmt="raw").result(timeout=5.0)
    assert a == b
    assert a[:4] == struct.pack("<I", tr.THIN_TAG)


def test_submit_honours_a_negotiated_resolution_below_native(broken_renderer):
    frame = broken_renderer.submit(image_scene(_ramp(7)), ThinCamera(),
                                   out_width=320, out_height=320).result(timeout=5.0)
    tag, w, h = THIN_HEADER.unpack_from(frame, 0)
    assert (w, h) == (320, 320)
    assert len(frame) == 8 + 320 * 320 * 2


def test_submit_negotiated_resolution_also_applies_to_jpeg(broken_renderer):
    frame = broken_renderer.submit(image_scene(_ramp(7)), ThinCamera(),
                                   fmt="jpeg", out_width=320,
                                   out_height=320).result(timeout=5.0)
    tag, w, h, _seq, plen = tr.THIN_HEADER_JPEG.unpack_from(frame, 0)
    assert (w, h) == (320, 320)
    assert plen == len(frame) - tr.THIN_HEADER_JPEG.size


def test_submit_jpeg_raises_when_simplejpeg_unavailable(broken_renderer, monkeypatch):
    """`submit` itself does not degrade -- graceful fallback is a `web.py`
    negotiation-time decision (#197 requirement 5); a caller that asks for
    `fmt="jpeg"` when it is genuinely unavailable gets the failure reported."""
    monkeypatch.setitem(sys.modules, "simplejpeg", None)
    fut = broken_renderer.submit(image_scene(_ramp(7)), ThinCamera(), fmt="jpeg")
    with pytest.raises(ImportError):
        fut.result(timeout=5.0)


def test_submit_image_scene_pack_runs_on_the_render_thread_not_the_caller(
        broken_renderer, monkeypatch):
    """#197 review finding 3: an image scene's letterbox + pack/encode used
    to run INLINE inside `submit()`, i.e. on the CALLER's thread -- which in
    production is the asyncio event loop. Now every scene kind, including
    `"image"`, is queued to the render thread and resolved there, so this
    must run on `broken_renderer._thread`, never on the thread that called
    `submit()`."""
    idents: list[int] = []
    orig_finish = ThinRenderer._finish

    def _spy_finish(self, rgb, pack, **kw):
        idents.append(threading.get_ident())
        return orig_finish(self, rgb, pack, **kw)

    monkeypatch.setattr(ThinRenderer, "_finish", _spy_finish)
    caller_ident = threading.get_ident()

    broken_renderer.submit(image_scene(_ramp(7)), ThinCamera()).result(timeout=5.0)

    assert idents == [broken_renderer._thread.ident]
    assert idents[0] != caller_ident


def test_submit_image_scene_jpeg_pack_runs_on_the_render_thread(
        broken_renderer, monkeypatch):
    """Same proof, but with the actual JPEG encode path (the one that made
    finding 3 a real cost rather than a theoretical one)."""
    idents: list[int] = []
    orig_encode = tr.encode_jpeg_frame

    def _spy_encode(rgb, quality=tr.DEFAULT_JPEG_QUALITY):
        idents.append(threading.get_ident())
        return orig_encode(rgb, quality)

    monkeypatch.setattr(tr, "encode_jpeg_frame", _spy_encode)
    caller_ident = threading.get_ident()

    broken_renderer.submit(image_scene(_ramp(7)), ThinCamera(),
                           fmt="jpeg").result(timeout=5.0)

    assert idents == [broken_renderer._thread.ident]
    assert idents[0] != caller_ident


def test_closed_renderer_fails_new_submissions(broken_renderer):
    broken_renderer.close()
    fut = broken_renderer.submit(points_scene(np.zeros((1, 3)), np.zeros((1, 3))),
                                 ThinCamera())
    with pytest.raises(ThinRenderUnavailable):
        fut.result(timeout=5.0)


# --------------------------------------------------------------------------
# 8 -- singleton identity
# --------------------------------------------------------------------------


class _FakeRenderer:
    """Stand-in for `ThinRenderer` -- no thread, no context, no Filament."""

    made = 0

    def __init__(self):
        _FakeRenderer.made += 1
        self.width = THIN_WIDTH
        self.height = THIN_HEIGHT

    def ensure_available(self, timeout=30.0):
        return None

    @property
    def available(self):
        return True


@pytest.fixture
def clean_singleton():
    ThinRenderer.reset_singleton()
    _FakeRenderer.made = 0
    try:
        yield
    finally:
        ThinRenderer.reset_singleton()


def test_instance_is_a_singleton(clean_singleton):
    a = ThinRenderer.instance(factory=_FakeRenderer)
    b = ThinRenderer.instance(factory=_FakeRenderer)
    assert a is b
    assert isinstance(a, _FakeRenderer)
    assert _FakeRenderer.made == 1          # constructed exactly once


def test_factory_is_ignored_once_an_instance_exists(clean_singleton):
    first = ThinRenderer.instance(factory=_FakeRenderer)
    second = ThinRenderer.instance(factory=lambda: pytest.fail("re-created"))
    assert second is first


def test_reset_singleton_drops_the_instance(clean_singleton):
    first = ThinRenderer.instance(factory=_FakeRenderer)
    ThinRenderer.reset_singleton()
    second = ThinRenderer.instance(factory=_FakeRenderer)
    assert second is not first
    assert _FakeRenderer.made == 2


def test_rgb_to_bytes_is_the_rgb565_encoder():
    img = _ramp(4)
    assert tr.rgb_to_bytes(img) == rgba_to_rgb565(img)


# --- mesh decimation budget (#194; upstream cause is #190) -------------------

def _big_mesh_bytes(n_verts: int, mesh_seq: int = 3) -> bytes:
    """A packed MESH with `n_verts` vertices and a triangle per 3 of them."""
    from types import SimpleNamespace
    from roomscan import web
    rng = np.random.default_rng(0)
    v = rng.normal(0, 1, (n_verts, 3)).astype(np.float32)
    c = rng.random((n_verts, 3)).astype(np.float32)
    t = np.arange(n_verts - (n_verts % 3), dtype=np.uint32).reshape(-1, 3)
    return web.pack_mesh(SimpleNamespace(
        non_wall_verts=v, non_wall_colors=c, non_wall_tris=t,
        wall_verts=np.zeros((0, 3), np.float32),
        wall_colors=np.zeros((0, 3), np.float32),
        wall_tris=np.zeros((0, 3), np.uint32),
        floor_pts=np.zeros((0, 3), np.float32),
        floor_lines=np.zeros((0, 2), np.uint32),
        decimated=False, wall_mode="single", mesh_seq=mesh_seq))


def test_oversized_mesh_is_decimated_under_the_budget():
    """A mesh past the budget must come back SMALLER and still be a valid mesh.

    Measured cause: a >1M-vertex Detailed mesh took tens of seconds to upload on
    llvmpipe -- one frame in 40 s, then none in 90 s -- and since all thin
    clients share one render thread it starved the others too.
    """
    scene = tr.unpack_mesh_scene(_big_mesh_bytes(9_000), vert_budget=1_000)
    assert scene is not None
    assert len(scene.points) <= 1_000
    assert scene.meta["decimated_from_verts"] == 9_000
    # still a usable mesh, not just a truncated vertex array
    assert len(scene.triangles) > 0
    assert scene.triangles.max() < len(scene.points)   # every index in range
    assert len(scene.colors) == len(scene.points)      # colors stayed aligned


def test_mesh_within_budget_is_untouched():
    scene = tr.unpack_mesh_scene(_big_mesh_bytes(300), vert_budget=1_000)
    assert scene is not None
    assert len(scene.points) == 300
    assert "decimated_from_verts" not in scene.meta


# --- 9. extract_ir_grid tests (#199) -----------------------------------------


def test_extract_ir_grid_none_or_empty_returns_none():
    assert tr.extract_ir_grid(None) is None
    assert tr.extract_ir_grid(np.zeros((0, 0))) is None


def test_extract_ir_grid_uniform_rgb():
    img = np.full((16, 16, 3), 128, dtype=np.uint8)
    grid = tr.extract_ir_grid(img)
    assert grid is not None
    assert len(grid) == 64
    assert all(val == 128 for val in grid)


def test_extract_ir_grid_2d_reflectance():
    refl = np.arange(54 * 42, dtype=np.float32).reshape(42, 54)
    grid = tr.extract_ir_grid(refl)
    assert grid is not None
    assert len(grid) == 64
    assert all(0 <= val <= 255 for val in grid)
    # top-left block should be smaller than bottom-right block
    assert grid[0] < grid[63]


# --- 9b. extract_ir_full tests (#202 follow-on) -------------------------------


def test_extract_ir_full_none_or_empty_returns_none():
    assert tr.extract_ir_full(None, 4096) is None
    assert tr.extract_ir_full(np.zeros((0, 0)), 4096) is None


def test_extract_ir_full_keeps_the_native_zone_grid():
    """The whole point: 54x42 in, 54x42 out -- no 8x8 block-mean in sight."""
    img = np.full((42, 54, 3), 128, dtype=np.uint8)
    got = tr.extract_ir_full(img, 4096)
    assert got is not None
    w, h, cells = got
    assert (w, h) == (54, 42)
    assert len(cells) == 54 * 42
    assert set(cells) == {128}


def test_extract_ir_full_reports_the_rotated_shape_not_a_fixed_one():
    """`ir_gravity_rot` transposes the pane, so w/h are read per frame."""
    img = np.full((54, 42, 3), 7, dtype=np.uint8)   # already rot90'd
    w, h, cells = tr.extract_ir_full(img, 4096)
    assert (w, h) == (42, 54)
    assert len(cells) == 42 * 54


def test_extract_ir_full_row_major_matches_the_source():
    refl = np.arange(6 * 4, dtype=np.float32).reshape(4, 6)   # 0..23 -> 0..255
    w, h, cells = tr.extract_ir_full(refl, 4096)
    assert (w, h) == (6, 4)
    vals = list(cells)
    assert vals[0] == 0 and vals[-1] == 255       # normalised endpoints
    assert vals == sorted(vals)                    # row-major, monotonic source


def test_extract_ir_full_refuses_rather_than_resamples_when_over_budget():
    """A client that asked for 64 cells gets None here and falls back to the 8x8 --
    it must never be handed a third resolution it never negotiated."""
    img = np.full((42, 54, 3), 128, dtype=np.uint8)
    assert tr.extract_ir_full(img, 64) is None
    assert tr.extract_ir_full(img, 54 * 42) is not None      # exactly fits

