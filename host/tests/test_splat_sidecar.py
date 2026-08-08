"""Splat sidecar/manifest helpers -- torch-free, mirrors slam/detailed.py's tests."""
from roomscan.splat import SplatPreset, sidecar


def test_slugify():
    assert sidecar.slugify("Sam Office") == "sam-office"
    assert sidecar.slugify("  A/B  c! ") == "a-b-c"
    assert sidecar.slugify("!!!") == "splat"       # never empty -> always creatable


def _seed(root, slug, video, preset, gaussians=5):
    paths = sidecar.sidecar_paths(slug, root)
    paths["dir"].mkdir(parents=True, exist_ok=True)
    paths["ply"].write_text("ply-bytes")
    man = sidecar.build_manifest("Sam Office", slug, video, preset,
                                 stats={"gaussians": gaussians},
                                 transform=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    sidecar.write_manifest_atomic(paths["manifest"], man)
    return paths


def test_status_absent(tmp_path):
    st = sidecar.sidecar_status("sam-office", tmp_path)
    assert not st["exists"] and not st["current"] and not st["stale"]


def test_status_current_then_stale(tmp_path):
    video = tmp_path / "room.mp4"
    video.write_bytes(b"x" * 100)
    preset = SplatPreset()
    _seed(tmp_path, "sam-office", video, preset)

    st = sidecar.sidecar_status("sam-office", tmp_path, video=video, preset=preset)
    assert st["exists"] and st["current"] and not st["stale"]

    # Source bytes change -> stale (identity mismatch).
    video.write_bytes(b"y" * 200)
    st2 = sidecar.sidecar_status("sam-office", tmp_path, video=video, preset=preset)
    assert st2["exists"] and not st2["current"] and st2["stale"]

    # Preset fingerprint change -> stale (even with identity restored).
    video.write_bytes(b"x" * 100)
    st3 = sidecar.sidecar_status("sam-office", tmp_path, video=video, preset=SplatPreset(iters=999))
    assert st3["stale"]


def test_status_partial_build_is_not_current(tmp_path):
    # A .ply with no manifest (build died before the commit marker) is not current.
    paths = sidecar.sidecar_paths("half", tmp_path)
    paths["dir"].mkdir(parents=True)
    paths["ply"].write_text("ply")
    st = sidecar.sidecar_status("half", tmp_path)
    assert not st["exists"] and not st["current"]


def test_list_and_delete(tmp_path):
    video = tmp_path / "room.mp4"
    video.write_bytes(b"x" * 10)
    _seed(tmp_path, "sam-office", video, SplatPreset(), gaussians=42)

    lst = sidecar.list_splats(tmp_path)
    assert len(lst) == 1
    e = lst[0]
    assert e["name"] == "Sam Office" and e["slug"] == "sam-office" and e["gaussians"] == 42
    assert e["ply_url"] == "/results/splats/sam-office/point_cloud.ply"
    assert e["transform"] and len(e["transform"]) == 4

    ok, freed = sidecar.delete_splat("sam-office", tmp_path)
    assert ok and freed > 0
    assert sidecar.list_splats(tmp_path) == []


def test_note_excluded_from_fingerprint():
    assert SplatPreset().fingerprint() == SplatPreset(note="anything else").fingerprint()
    assert SplatPreset().fingerprint() != SplatPreset(iters=1).fingerprint()


def test_import_manifest_keeps_imported_badge_and_transform(tmp_path):
    """An imported reference splat may carry a manifest solely for its display name +
    an orientation transform (e.g. righting an upside-down Scaniverse export) and must
    still list as `imported` so the picker doesn't mistake it for one of our builds."""
    paths = sidecar.sidecar_paths("scaniverse-splat", tmp_path)
    paths["dir"].mkdir(parents=True)
    (paths["dir"] / "SamOffice.ply").write_text("ply")            # not point_cloud.ply
    flip = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    sidecar.write_import_manifest("scaniverse-splat", tmp_path, name="Sam Office",
                                  transform=flip, gaussians=2455660)

    e = next(s for s in sidecar.list_splats(tmp_path) if s["slug"] == "scaniverse-splat")
    assert e["imported"] is True                                  # still badged external
    assert e["name"] == "Sam Office"
    assert e["transform"] == flip                                 # viewer will right it
    assert e["gaussians"] == 2455660
    assert e["ply_url"].endswith("/SamOffice.ply")
