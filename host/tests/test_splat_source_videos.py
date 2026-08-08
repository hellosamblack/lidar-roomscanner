"""Splat capture-viewer backend: list_source_videos, list_splats metadata,
splat_defaults. Torch-free -- mirrors test_splat_sidecar.py."""
import dataclasses

from roomscan.splat import SplatPreset, sidecar


def _seed_splat(root, slug, video, preset, *, name="Sam Office", gaussians=5):
    paths = sidecar.sidecar_paths(slug, root)
    paths["dir"].mkdir(parents=True, exist_ok=True)
    paths["ply"].write_text("ply-bytes")
    man = sidecar.build_manifest(name, slug, video, preset,
                                 stats={"gaussians": gaussians},
                                 transform=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    sidecar.write_manifest_atomic(paths["manifest"], man)


def _imported_ply(root, slug, n=7):
    """A splat dir with a non-canonical .ply and NO manifest (external import)."""
    d = sidecar.splats_root(root) / slug
    d.mkdir(parents=True, exist_ok=True)
    (d / "External.ply").write_text(
        f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\n"
        "property float x\nend_header\n")


def test_splat_defaults_covers_every_field():
    d = sidecar.splat_defaults()
    assert set(d) == {f.name for f in dataclasses.fields(SplatPreset)}
    assert d["depth_lambda"] == 0.0


def test_list_splats_carries_preset_stats_and_imported(tmp_path):
    video = tmp_path / "captures" / "room.mp4"
    video.parent.mkdir()
    video.write_bytes(b"x" * 10)
    _seed_splat(tmp_path, "sam-office", video, SplatPreset(iters=1234), gaussians=42)
    _imported_ply(tmp_path, "scaniverse-splat", n=2455660)

    by_slug = {e["slug"]: e for e in sidecar.list_splats(tmp_path)}
    ours = by_slug["sam-office"]
    assert ours["stats"]["gaussians"] == 42 and ours["preset"]["iters"] == 1234
    assert ours["imported"] is False
    imp = by_slug["scaniverse-splat"]
    assert imp["imported"] is True
    assert imp["gaussians"] == 2455660                      # read from the PLY header
    assert imp["ply_url"].endswith("/scaniverse-splat/External.ply")


def test_list_source_videos_states(tmp_path):
    captures = tmp_path / "captures"
    captures.mkdir()
    a = captures / "a.mp4"; a.write_bytes(b"x" * 10)
    b = captures / "b.mov"; b.write_bytes(b"y" * 10)
    (captures / "notes.txt").write_text("ignore me")     # non-video ignored
    # Two builds from a.mp4 (photometry + depth), none from b.mov.
    _seed_splat(tmp_path, "a-photo", a, SplatPreset(), name="A")
    _seed_splat(tmp_path, "a-depth", a, SplatPreset(depth_lambda=0.1), name="A depth")

    vids = {v["video"]: v for v in sidecar.list_source_videos(captures, tmp_path)}
    assert set(vids) == {"a.mp4", "b.mov"}                # .txt excluded
    va = vids["a.mp4"]
    assert va["has_splat"] and va["has_current"] and len(va["splats"]) == 2
    assert all(s["state"] == "current" for s in va["splats"])
    assert {s["depth_lambda"] for s in va["splats"]} == {0.0, 0.1}
    assert not vids["b.mov"]["has_splat"]

    # Re-record a.mp4 -> both its builds go stale.
    a.write_bytes(b"z" * 20)
    va2 = [v for v in sidecar.list_source_videos(captures, tmp_path) if v["video"] == "a.mp4"][0]
    assert not va2["has_current"]
    assert all(s["state"] == "stale" for s in va2["splats"])


def test_list_source_videos_empty_dirs(tmp_path):
    assert sidecar.list_source_videos(tmp_path / "nope", tmp_path) == []
