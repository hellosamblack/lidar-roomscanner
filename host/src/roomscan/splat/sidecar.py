"""Filesystem helpers for splat reconstructions -- the ``slam/detailed.py`` shape.

A splat artifact is keyed by a **slug** derived from its display name (the source
is a video, not a ``.bin`` capture, so there is no capture stem to key on).  Each
splat lives in its own directory ``results/splats/<slug>/`` holding
``point_cloud.ply`` (the trained gaussians) and ``manifest.json`` (identity +
preset + stats + the levelling transform).  The manifest is written LAST, so its
presence is the commit marker: a half-written build has a ``.ply`` but no manifest
and reads as "not current".

Pure filesystem/JSON only -- no torch, no CUDA, no device I/O -- so the web
server and tests can import it without the heavy training stack installed.
"""
from __future__ import annotations

import dataclasses
import json
import os
import re
import shutil
import time
from pathlib import Path

from .config import SplatPreset

SPLATS_SUBDIR = "splats"
PLY_NAME = "point_cloud.ply"
MANIFEST_NAME = "manifest.json"
VIDEO_EXTS = (".mp4", ".mov")   # phone-video source formats the splat pipeline accepts


def slugify(name: str) -> str:
    """Human name -> filesystem-safe slug (``"Sam Office" -> "sam-office"``).

    Falls back to ``"splat"`` for an all-punctuation name so a directory is
    always creatable.
    """
    slug = re.sub(r"[^a-z0-9]+", "-", str(name).strip().lower()).strip("-")
    return slug or "splat"


def splats_root(results_dir: str | Path) -> Path:
    return Path(results_dir) / SPLATS_SUBDIR


def sidecar_paths(slug: str, results_dir: str | Path) -> dict[str, Path]:
    root = splats_root(results_dir) / slug
    return {"dir": root, "ply": root / PLY_NAME, "manifest": root / MANIFEST_NAME}


def source_identity(video: str | Path) -> dict:
    st = Path(video).stat()
    return {"name": Path(video).name, "bytes": st.st_size, "mtime_ns": st.st_mtime_ns}


def load_manifest(slug: str, results_dir: str | Path) -> dict | None:
    p = sidecar_paths(slug, results_dir)["manifest"]
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def sidecar_status(slug: str, results_dir: str | Path, *,
                   video: str | Path | None = None,
                   preset: SplatPreset | None = None) -> dict:
    """Describe a splat's build state.

    ``current`` is true only when both files exist AND (when a ``video`` and
    ``preset`` are supplied to compare against) the manifest's source identity
    and preset fingerprint still match.  With no ``video``/``preset`` the check
    degrades to "the files are present and the manifest parses" -- which is all
    the web server needs to offer an existing splat for viewing.
    """
    paths = sidecar_paths(slug, results_dir)
    manifest = load_manifest(slug, results_dir)
    present = paths["ply"].is_file() and paths["manifest"].is_file()
    current = bool(present and manifest)
    if current and preset is not None:
        current = manifest.get("preset_fingerprint") == preset.fingerprint()
    if current and video is not None:
        current = manifest.get("source") == source_identity(video)
    return {"exists": present, "current": current,
            "stale": bool(present and manifest and not current),
            "paths": {k: str(v) for k, v in paths.items()}, "manifest": manifest}


def build_manifest(name: str, slug: str, video: str | Path, preset: SplatPreset, *,
                   stats: dict, transform: list[list[float]]) -> dict:
    return {"schema": 1, "created_unix_s": round(time.time(), 3),
            "name": name, "slug": slug,
            "source": source_identity(video),
            "preset": {k: v for k, v in vars(preset).items() if not k.startswith("_")},
            "preset_fingerprint": preset.fingerprint(),
            "transform": transform, "stats": stats}


def write_import_manifest(slug: str, results_dir: str | Path, *, name: str,
                          transform: list[list[float]] | None = None,
                          gaussians: int | None = None) -> Path:
    """Record a display name + orientation for an externally-imported reference
    splat (e.g. a Scaniverse export) without pretending it is one of our builds.

    Unlike ``build_manifest``, this carries no ``source``/``preset``/fingerprint --
    it only names the splat, stores an optional 4x4 orientation ``transform`` (the
    web viewer applies it, so an upside-down export can be righted non-destructively),
    and marks ``imported: true`` so the picker still badges it as external. Returns
    the manifest path written.
    """
    paths = sidecar_paths(slug, results_dir)
    manifest = {"schema": 1, "created_unix_s": round(time.time(), 3),
                "name": name, "slug": slug, "imported": True,
                "transform": transform, "stats": {"gaussians": gaussians}}
    write_manifest_atomic(paths["manifest"], manifest)
    return paths["manifest"]


def write_manifest_atomic(path: str | Path, manifest: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def _resolve_ply(d: Path) -> Path | None:
    """The renderable ``.ply`` in a splat dir: our ``point_cloud.ply`` if present,
    else any single ``.ply`` (so an externally-produced reference splat dropped in
    -- e.g. a Scaniverse export named ``SamOffice.ply`` -- is viewable without being
    renamed or given a manifest)."""
    canonical = d / PLY_NAME
    if canonical.is_file():
        return canonical
    plys = sorted(d.glob("*.ply"))
    return plys[0] if plys else None


def _ply_vertex_count(ply: Path) -> int | None:
    """Gaussian count from a PLY header alone (``element vertex N``) -- a bounded
    read, so a 600 MB reference splat with no manifest still shows a real count."""
    try:
        with ply.open("rb") as f:
            for _ in range(60):                      # header is tiny; cap the scan
                line = f.readline()
                if not line or line.strip() == b"end_header":
                    break
                if line.startswith(b"element vertex"):
                    return int(line.split()[2])
    except (OSError, ValueError, IndexError):
        return None
    return None


def list_splats(results_dir: str | Path) -> list[dict]:
    """Every built splat, newest first, for the web picker.

    Reads each manifest for the display name / gaussian count; a directory with a
    ``.ply`` but an unreadable/absent manifest still lists (named by its slug, count
    read from the PLY header) so a partial build -- or an externally imported
    reference splat like the Scaniverse comparison -- is visible.
    """
    root = splats_root(results_dir)
    out: list[dict] = []
    if not root.is_dir():
        return out
    for d in root.iterdir():
        if not d.is_dir():
            continue
        ply = _resolve_ply(d)
        if ply is None:
            continue
        man = load_manifest(d.name, results_dir) or {}
        st = ply.stat()
        gaussians = (man.get("stats") or {}).get("gaussians")
        if gaussians is None:
            gaussians = _ply_vertex_count(ply)
        out.append({
            "slug": d.name,
            "name": man.get("name") or d.name,
            "gaussians": gaussians,
            "bytes": st.st_size,
            "mtime": st.st_mtime,
            "ply_url": f"/results/{SPLATS_SUBDIR}/{d.name}/{ply.name}",
            "transform": man.get("transform"),
            # `imported` = an external reference splat (e.g. Scaniverse), not one of
            # our video builds. True when there is no manifest, OR when a manifest
            # explicitly marks it imported -- a reference splat may carry a manifest
            # solely to record its display name and an orientation `transform` (e.g.
            # righting an upside-down Scaniverse export) while still being badged
            # imported, since it has no COLMAP frame / preset of ours.
            "imported": bool(man.get("imported")) or not bool(man),
            # The preset + quality stats a build was made with -- surfaced so the
            # picker can show "which settings produced this splat" (metadata on the
            # splat). Already recorded by build_manifest; here we just pass them up.
            "preset": man.get("preset"),
            "stats": man.get("stats"),
        })
    out.sort(key=lambda e: e["mtime"], reverse=True)
    return out


def splat_defaults() -> dict:
    """Every ``SplatPreset`` field at its default -- seeds the web build-settings form
    so the client never hardcodes defaults (they stay owned by the dataclass)."""
    return dataclasses.asdict(SplatPreset())


def list_source_videos(captures_dir: str | Path, results_dir: str | Path) -> list[dict]:
    """Source phone videos in ``captures_dir`` (``.mp4``/``.mov``), newest first, each
    cross-referenced against the splats built from it.

    A single video can back several splats (e.g. a photometry build and a
    depth-prior build of the same footage), so each entry carries a ``splats`` list.
    Per built splat, ``state`` is ``current`` when the manifest's recorded source
    identity still matches the file on disk, or ``stale`` when the video was
    re-recorded since that build. ``has_splat``/``has_current`` summarise the row.
    """
    captures_dir = Path(captures_dir)
    # Map source video name -> [splat manifest summaries]. Built once over all splats.
    by_source: dict[str, list[tuple[str, dict]]] = {}
    root = splats_root(results_dir)
    if root.is_dir():
        for d in root.iterdir():
            if not d.is_dir() or not (d / PLY_NAME).is_file():
                continue
            man = load_manifest(d.name, results_dir)
            src_name = (man or {}).get("source", {}).get("name") if man else None
            if src_name:
                by_source.setdefault(src_name, []).append((d.name, man))

    out: list[dict] = []
    if not captures_dir.is_dir():
        return out
    for p in sorted(captures_dir.iterdir()):
        if not p.is_file() or p.suffix.lower() not in VIDEO_EXTS:
            continue
        st = p.stat()
        ident = source_identity(p)
        splats = []
        for slug, man in by_source.get(p.name, []):
            man = man or {}
            state = "current" if man.get("source") == ident else "stale"
            splats.append({
                "slug": slug,
                "name": man.get("name") or slug,
                "gaussians": (man.get("stats") or {}).get("gaussians"),
                "depth_lambda": (man.get("preset") or {}).get("depth_lambda"),
                "state": state,
                "preset": man.get("preset"),
                "stats": man.get("stats"),
                "ply_url": f"/results/{SPLATS_SUBDIR}/{slug}/{PLY_NAME}",
            })
        splats.sort(key=lambda e: e["name"])
        out.append({
            "video": p.name,
            "bytes": st.st_size,
            "mtime": st.st_mtime,
            "splats": splats,
            "has_splat": bool(splats),
            "has_current": any(s["state"] == "current" for s in splats),
        })
    out.sort(key=lambda e: e["mtime"], reverse=True)
    return out


def delete_splat(slug: str, results_dir: str | Path) -> tuple[bool, int]:
    """Remove a splat's whole directory. Returns (removed, bytes_freed)."""
    d = sidecar_paths(slug, results_dir)["dir"]
    if not d.is_dir():
        return False, 0
    freed = sum(p.stat().st_size for p in d.rglob("*") if p.is_file())
    shutil.rmtree(d, ignore_errors=True)
    return True, freed
