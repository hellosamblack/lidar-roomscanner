"""Headless unit tests for the Task 14 panel-UX batch's testable helpers.

None of these open a GUI (Filament fails headless on the dev box, same as the
other test_panel_* modules) -- they cover the pure helpers the GUI wiring
delegates to: ETA math (#3), FoV frustum geometry (#2), banner ASCII-ness
(#4), save-path formatting (#6), and the reflectance-vs-shade colour decision
(#1). GUI-only behaviour stays supervised-run verified, as elsewhere.
"""
import numpy as np
import pytest

import roomscan.panel as panel_mod
from roomscan.panel import (
    _eta_seconds,
    _format_eta,
    _fov_frustum_lines,
    _showcase_result_paths,
)
from roomscan.slam.shading import (
    mesh_colors_are_meaningful,
    shade_brightness,
    shade_colors,
)


# ---- Issue #3: ETA math ----------------------------------------------------

def test_eta_seconds_halfway():
    # 10 s elapsed at 50% -> ~10 s remaining
    assert _eta_seconds(10.0, 0.5) == pytest.approx(10.0)


def test_eta_seconds_quarter():
    # 10 s elapsed at 25% -> 30 s remaining
    assert _eta_seconds(10.0, 0.25) == pytest.approx(30.0)


def test_eta_seconds_guards_tiny_fraction():
    # frac -> 0 would blow up: return None instead of +inf
    assert _eta_seconds(10.0, 0.0) is None
    assert _eta_seconds(10.0, 1e-4) is None


def test_eta_seconds_guards_done():
    assert _eta_seconds(10.0, 1.0) is None
    assert _eta_seconds(10.0, 1.5) is None


def test_format_eta_minutes_seconds():
    assert _format_eta(38.0) == "~0:38 left"
    assert _format_eta(95.0) == "~1:35 left"
    assert _format_eta(600.0) == "~10:00 left"


def test_format_eta_none_and_negative_are_blank():
    assert _format_eta(None) == ""
    assert _format_eta(-5.0) == ""


def test_format_eta_is_ascii():
    for s in (0.0, 38.0, 95.0, 3600.0):
        assert _format_eta(s).isascii()


# ---- Issue #2: FoV frustum geometry ----------------------------------------

def test_fov_frustum_shape():
    pts, lines = _fov_frustum_lines(np.eye(4), 55.0, 42.0)
    assert pts.shape == (5, 3)          # origin + 4 corners
    assert lines.shape == (8, 2)        # 4 rays + 4 far-rectangle edges
    assert lines.min() >= 0 and lines.max() <= 4


def test_fov_frustum_apex_is_camera_origin():
    pose = np.eye(4)
    pose[:3, 3] = [1.0, 2.0, 3.0]
    pts, _ = _fov_frustum_lines(pose, 55.0, 42.0)
    assert np.allclose(pts[0], [1.0, 2.0, 3.0])   # apex == translation


def test_fov_frustum_corners_are_in_front_identity_pose():
    # Identity pose, CV convention (z forward): all 4 far corners have z>0.
    pts, _ = _fov_frustum_lines(np.eye(4), 55.0, 42.0, range_m=0.5)
    corners = pts[1:]
    assert np.all(corners[:, 2] > 0.0)
    # corners sit at ~range_m from the apex (unit-normalised rays * range)
    dists = np.linalg.norm(corners - pts[0], axis=1)
    assert np.allclose(dists, 0.5)


def test_fov_frustum_follows_pose_rotation():
    # A 180-deg yaw about world-up should flip the corners' z sign.
    pose = np.eye(4)
    c, s = np.cos(np.pi), np.sin(np.pi)
    pose[:3, :3] = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    pts, _ = _fov_frustum_lines(pose, 55.0, 42.0, range_m=0.5)
    assert np.all(pts[1:, 2] < 0.0)     # now behind the origin along -z


def test_fov_frustum_wider_hfov_spreads_corners():
    narrow, _ = _fov_frustum_lines(np.eye(4), 30.0, 42.0, range_m=1.0)
    wide, _ = _fov_frustum_lines(np.eye(4), 90.0, 42.0, range_m=1.0)
    # wider horizontal FoV -> larger |x| spread at the far corners
    assert np.abs(wide[1:, 0]).max() > np.abs(narrow[1:, 0]).max()


# ---- Issue #6: save-path formatting ----------------------------------------

def test_showcase_result_paths_format():
    mesh, traj = _showcase_result_paths("20260711_120000")
    assert mesh == "results/showcase_20260711_120000.ply"
    assert traj == "results/showcase_20260711_120000.tum"


def test_showcase_result_paths_custom_dir():
    mesh, traj = _showcase_result_paths("ts", results_dir="out")
    assert mesh == "out/showcase_ts.ply"
    assert traj == "out/showcase_ts.tum"


# ---- Issue #4: banner ASCII-ness (module-level static strings) -------------

def test_showcase_banner_static_strings_are_ascii():
    """Every literal banner string the panel sets must be pure ASCII so the
    GUI font never renders tofu ("?"). We can't run the GUI headless, so grep
    the source for _set_showcase_banner("...") literals and assert each is
    ASCII. This is a real regression guard: the bug was unicode bullets/em
    dashes/ellipses in exactly these calls."""
    import ast
    import pathlib

    src = pathlib.Path(panel_mod.__file__).read_text(encoding="utf-8")
    tree = ast.parse(src)
    literals = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "_set_showcase_banner" and node.args):
            arg = node.args[0]
            # Only plain string literals / f-strings' constant parts are
            # checkable statically; collect Constant str args and the constant
            # pieces of JoinedStr (f-strings).
            if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                literals.append(arg.value)
            elif isinstance(arg, ast.JoinedStr):
                for v in arg.values:
                    if isinstance(v, ast.Constant) and isinstance(v.value, str):
                        literals.append(v.value)
    assert literals, "no _set_showcase_banner string literals found -- test wiring broke"
    non_ascii = [s for s in literals if not s.isascii()]
    assert non_ascii == [], f"non-ASCII banner strings: {non_ascii!r}"


# ---- Issue #1: reflectance-vs-shade colour decision ------------------------

def test_mesh_colors_meaningful_all_black_is_false():
    assert mesh_colors_are_meaningful(np.zeros((100, 3))) is False


def test_mesh_colors_meaningful_empty_is_false():
    assert mesh_colors_are_meaningful(np.zeros((0, 3))) is False


def test_mesh_colors_meaningful_real_colors_is_true():
    colors = np.zeros((100, 3))
    colors[5] = [0.4, 0.4, 0.4]
    assert mesh_colors_are_meaningful(colors) is True


def test_shade_brightness_matches_shade_colors_base():
    # shade_colors == brightness[:,None] * base_color (clamped). So dividing
    # shade_colors back out by brightness must recover the fixed base color.
    from roomscan.slam.shading import _BASE_COLOR
    rng = np.random.default_rng(1)
    normals = rng.normal(size=(50, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    b = shade_brightness(normals)
    sc = shade_colors(normals)
    assert b.shape == (50,)
    assert np.allclose(sc, np.clip(b[:, None] * _BASE_COLOR, 0.0, 1.0))


def test_shade_brightness_empty():
    assert shade_brightness(np.zeros((0, 3))).shape == (0,)


def test_reflectance_modulation_darkens_not_brightens():
    # The live-render rule (#1): reflectance_rgb * shade_brightness. Since
    # brightness in [_AMBIENT, _AMBIENT+_DIFFUSE] <= 1.0, modulation can only
    # dim a reflectance colour (add form), never blow it past its own value.
    refl = np.full((20, 3), 0.7)
    normals = np.tile([0.0, 0.0, 1.0], (20, 1))
    b = shade_brightness(normals)
    modulated = np.clip(refl * b[:, None], 0.0, 1.0)
    assert np.all(modulated <= refl + 1e-9)
    assert np.all(modulated >= 0.0)


# ---- Issue #1: end-to-end through the real _upload_slam_mesh ----------------
#
# Reuses test_panel_walls' fake-scene / unbound-method pattern (no GUI): build
# a TSDF mesh integrated WITH a reflectance colour image, run it through the
# real ControlPanel._upload_slam_mesh, and assert the uploaded geometry's
# vertex colours came from the mesh's OWN (varying) reflectance modulated by
# shade brightness -- NOT the fixed, near-uniform shade_colors base tint.

import open3d as o3d  # noqa: E402
from roomscan.logbus import LogBus  # noqa: E402
from roomscan.slam.intrinsics import pinhole  # noqa: E402
from roomscan.slam.tsdf import TsdfMap  # noqa: E402


class _FakeScene:
    def __init__(self):
        self.geoms = {}

    def has_geometry(self, name):
        return name in self.geoms

    def add_geometry(self, name, geom, material):
        self.geoms[name] = (geom, material)

    def remove_geometry(self, name):
        del self.geoms[name]


class _FakeUploadPanel:
    def __init__(self, wall_mode="solid"):
        self._o3d = o3d
        self.scene_widget = type("SW", (), {"scene": _FakeScene()})()
        self.wall_mode = wall_mode
        self.mesh_material = "MESH"
        self.wall_translucent_material = "WT"
        self.wall_wire_material = "WW"
        self.bus = LogBus()
        self._slam_last_mesh_obj = None
        self._showcase_last_mesh_obj = None
        self._fov_last_pose = None


def _reflectance_tsdf_mesh():
    W, H = 54, 42
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    rows = np.linspace(-0.4, 0.4, H)[:, None]
    cols = np.linspace(-0.5, 0.5, W)[None, :]
    for z in (1.30, 1.28, 1.26, 1.24):
        depth = ((z + 0.15 * (rows ** 2 + cols ** 2)) * 1000.0).astype(np.float32)
        grad = (np.linspace(0.2, 0.9, W)[None, :] * np.ones((H, 1))).astype(np.float32)
        color = np.repeat(grad[..., None], 3, axis=-1)
        m.integrate(depth, K, np.eye(4), color=color)
    return m.mesh()


def test_upload_uses_reflectance_colors_when_present():
    mesh = _reflectance_tsdf_mesh()
    assert mesh.vertex.colors.numpy().max() > 1e-6   # precondition: real colours

    fake = _FakeUploadPanel(wall_mode="solid")
    panel_mod.ControlPanel._upload_slam_mesh(fake, mesh)
    geom, _mat = fake.scene_widget.scene.geoms[panel_mod._MESH_GEOM]
    uploaded = np.asarray(geom.vertex_colors)

    # The uploaded colours must VARY a lot across vertices (they carry the
    # reflectance gradient), far more than the near-flat fixed shade tint that
    # a depth-only mesh would get.
    refl_spread = uploaded.std(axis=0).max()
    from roomscan.slam.shading import shade_colors
    legacy = mesh.cpu().to_legacy()
    legacy.compute_vertex_normals()
    shade_only = shade_colors(np.asarray(legacy.vertex_normals))
    assert refl_spread > shade_only.std(axis=0).max()


def test_upload_falls_back_to_shade_for_depth_only_mesh():
    # A depth-only TSDF mesh has all-black vertex colours -> fall back to the
    # fixed shade_colors tint (byte-identical to pre-Task-14 behaviour).
    W, H = 54, 42
    m = TsdfMap(voxel_size=0.02, depth_max=5.0)
    K = pinhole(W, H)
    rows = np.linspace(-0.4, 0.4, H)[:, None]
    cols = np.linspace(-0.5, 0.5, W)[None, :]
    for z in (1.30, 1.28, 1.26, 1.24):
        depth = ((z + 0.15 * (rows ** 2 + cols ** 2)) * 1000.0).astype(np.float32)
        m.integrate(depth, K, np.eye(4))   # NO color -> black vertex colours
    mesh = m.mesh()
    assert np.allclose(mesh.vertex.colors.numpy(), 0.0)

    fake = _FakeUploadPanel(wall_mode="solid")
    panel_mod.ControlPanel._upload_slam_mesh(fake, mesh)
    geom, _mat = fake.scene_widget.scene.geoms[panel_mod._MESH_GEOM]
    uploaded = np.asarray(geom.vertex_colors)

    from roomscan.slam.shading import shade_colors
    legacy = mesh.cpu().to_legacy()
    legacy.compute_vertex_normals()
    expected = shade_colors(np.asarray(legacy.vertex_normals))
    assert np.allclose(uploaded, expected)   # exact fallback, unchanged behaviour


# ---- Issue #6: save-to-disk (real files, off the GUI thread) ---------------


class _FakeSavePanel:
    def __init__(self):
        self._o3d = o3d
        self.bus = LogBus()
        self._showcase_save_thread = None


def test_save_showcase_result_writes_ply_and_tum(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    mesh = _reflectance_tsdf_mesh()
    traj = [np.eye(4), np.eye(4), np.eye(4)]
    fake = _FakeSavePanel()

    panel_mod.ControlPanel._save_showcase_result(fake, mesh, traj)
    assert fake._showcase_save_thread is not None
    fake._showcase_save_thread.join(timeout=10.0)

    results = list((tmp_path / "results").glob("showcase_*"))
    suffixes = sorted(p.suffix for p in results)
    assert suffixes == [".ply", ".tum"]
    tum = next(p for p in results if p.suffix == ".tum")
    # 3 poses -> 3 TUM lines, each 8 whitespace-separated fields
    lines = tum.read_text().strip().splitlines()
    assert len(lines) == 3
    assert all(len(ln.split()) == 8 for ln in lines)


def test_save_showcase_result_noop_on_empty_mesh(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    empty = o3d.t.geometry.TriangleMesh()
    empty.vertex.positions = o3d.core.Tensor(np.zeros((0, 3), dtype=np.float32))
    fake = _FakeSavePanel()
    panel_mod.ControlPanel._save_showcase_result(fake, empty, [])
    # nothing written, no thread spawned
    assert fake._showcase_save_thread is None
    assert not (tmp_path / "results").exists()
