"""Offline Detailed SLAM helpers shared by the web app and MCP tools.

This module deliberately does no device I/O and never alters a capture.  The
web server owns presentation and uses these pure-ish filesystem helpers to
describe, validate, and atomically commit a capture's reconstruction sidecar.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import asdict
from pathlib import Path

from .config import DetailedSlamPreset


def sidecar_paths(capture: str | Path, results_dir: str | Path) -> dict[str, Path]:
    stem = Path(capture).stem
    root = Path(results_dir)
    return {"ply": root / f"{stem}.ply", "tum": root / f"{stem}.tum",
            "manifest": root / f"{stem}.slam.json"}


def capture_identity(capture: str | Path) -> dict:
    st = Path(capture).stat()
    return {"name": Path(capture).name, "bytes": st.st_size, "mtime_ns": st.st_mtime_ns}


def load_manifest(capture: str | Path, results_dir: str | Path) -> dict | None:
    p = sidecar_paths(capture, results_dir)["manifest"]
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def sidecar_status(capture: str | Path, results_dir: str | Path,
                   preset: DetailedSlamPreset | None = None) -> dict:
    preset = preset or DetailedSlamPreset.load()
    paths = sidecar_paths(capture, results_dir)
    manifest = load_manifest(capture, results_dir)
    present = all(p.is_file() for p in paths.values())
    current = bool(present and manifest and
                   manifest.get("preset_fingerprint") == preset.fingerprint() and
                   manifest.get("capture") == capture_identity(capture))
    return {"exists": present, "current": current, "stale": bool(present and not current),
            "paths": {k: p.name for k, p in paths.items()}, "manifest": manifest}


def estimate_seconds(frames: int, preset: DetailedSlamPreset, *, cuda: bool) -> dict:
    calibrated = preset.per_frame_ms > 0 and preset.global_opt_ms >= 0
    seconds = max(0, frames) * max(0.0, preset.per_frame_ms) / 1000.0 + max(0.0, preset.global_opt_ms) / 1000.0
    return {"frames": int(frames), "seconds": round(seconds, 1), "calibrated": calibrated,
            "cpu_warning": not cuda,
            "note": None if calibrated else preset.benchmark_note}


def build_manifest(capture: str | Path, preset: DetailedSlamPreset, *, stats: dict,
                   estimate: dict, loop_closure: dict | None = None) -> dict:
    return {"schema": 1, "created_unix_s": round(time.time(), 3),
            "capture": capture_identity(capture), "preset": asdict(preset),
            "preset_fingerprint": preset.fingerprint(), "estimate": estimate,
            "loop_closure": loop_closure or {"enabled": False, "decision": "offline-only pending gate"},
            "stats": stats}


def write_manifest_atomic(path: str | Path, manifest: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp, path)
