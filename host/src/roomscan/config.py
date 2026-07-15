"""Viewer config persistence: ``%APPDATA%/roomscan/roomscan.toml``.

Read with ``tomllib`` (stdlib, Python >=3.11, matches ``pyproject.toml``'s
floor). The stdlib has no TOML *writer*, and this project takes no
third-party dependency for one -- ``save()`` hand-emits a minimal flat TOML
(one ``[viewer]`` table, ``key = value`` lines) covering exactly the field
set below. Do not grow this file's shape (nested tables, arrays, ...)
without upgrading the writer to match.

Priority for effective viewer settings is CLI flag > config file > built-in
default; ``apply_config_defaults`` implements the CLI-over-config half here,
`ViewerConfig.load` implements the config-file-over-built-in half by simply
using the dataclass defaults for anything missing/invalid in the file.
"""
from __future__ import annotations

import os
import tomllib
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Optional


def config_dir() -> Path:
    """``%APPDATA%/roomscan`` on Windows. Falls back to the user's home
    directory if ``APPDATA`` isn't set (non-Windows dev shells, tests) --
    read fresh from the environment on every call so tests can monkeypatch
    it without needing to reload this module."""
    appdata = os.environ.get("APPDATA")
    base = Path(appdata) if appdata else Path.home()
    return base / "roomscan"


def config_path() -> Path:
    return config_dir() / "roomscan.toml"


@dataclass
class ViewerConfig:
    color: str = "reflectance"   # falls back to depth coloring when the plane is absent
    fov_h: float = 55.0
    fov_v: float = 42.0
    replay_fps: float = 0.0
    port: Optional[str] = None
    point_size: float = 5.0            # larger default closes the inter-zone gaps
    ir_colormap: str = "gray"
    ir_freeze_range: bool = False
    panel_width: int = 340
    near_mode: str = "window"          # near-contrast: off|window|emphasis|equalize
    near_cutoff_m: float = 1.5         # window-mode near/far boundary (metres)
    near_emphasis: float = 0.5         # emphasis-mode strength 0..1
    surface_enabled: bool = False
    surface_mode: str = "grid"          # "grid" | "spatial"
    surface_threshold_pct: float = 4.0
    imu_gizmo: bool = True             # show the orientation gizmo in the scene
    sensors_panel: bool = True         # show the Sensors panel group
    gizmo_scale: float = 0.15          # gizmo axis length (metres)
    metrics_overlay: bool = True       # show the on-scene metrics HUD (rates/fps/resources)
    mode: str = "real_time"            # UI redesign: "real_time" | "slam" (owner: default to real-time)
    camera: str = "first_person"       # UI redesign: "first_person" | "orbit"
    ir_overlay: bool = False           # first-person IR billboard overlay on/off
    ir_opacity: float = 0.5            # IR overlay opacity 0..1
    yaw_fusion: bool = True                 # graft mag heading onto SFLP yaw
    yaw_fusion_tau: float = 20.0            # complementary-filter time constant (s)
    mag_cal_path: str = "mag_cal.json"      # hard/soft-iron calibration JSON
    yaw_anomaly_frac: float = 0.3           # |mag| deviation from field to reject
    yaw_motion_rate_dps: float = 40.0       # quat angular rate above which to freeze
    yaw_gimbal_margin_deg: float = 15.0     # freeze within this of |pitch|=90

    @classmethod
    def load(cls, path: Optional[Path] = None) -> "ViewerConfig":
        """Missing file, unreadable file, or corrupt/malformed TOML are all
        tolerated -- return the built-in defaults rather than raising. Only
        recognized fields are pulled from a present ``[viewer]`` table;
        anything else in the file (unknown keys, other tables) is ignored."""
        path = Path(path) if path is not None else config_path()
        try:
            raw = path.read_text(encoding="utf-8")
        except OSError:
            return cls()
        try:
            data = tomllib.loads(raw)
        except tomllib.TOMLDecodeError:
            return cls()
        viewer = data.get("viewer")
        if not isinstance(viewer, dict):
            return cls()
        known = {f.name for f in fields(cls)}
        kwargs = {k: v for k, v in viewer.items() if k in known}
        if kwargs.get("port") == "":
            kwargs["port"] = None  # TOML has no null; empty string round-trips "unset"
        try:
            return cls(**kwargs)
        except TypeError:
            return cls()  # a field held a value of the wrong shape/type

    def save(self, path: Optional[Path] = None) -> Path:
        path = Path(path) if path is not None else config_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        lines = ["[viewer]"]
        for f in fields(self):
            lines.append(f"{f.name} = {_toml_value(getattr(self, f.name))}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return path


def _toml_value(value) -> str:
    if value is None:
        return '""'
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, int):
        return str(value)
    escaped = str(value).replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def apply_config_defaults(args, config: ViewerConfig) -> None:
    """Mutate an argparse.Namespace in place: any of the five viewer flags
    left at argparse's ``None`` sentinel (i.e. the user didn't pass it) is
    filled from `config` (which already resolved file-vs-built-in); anything
    the user did pass on the CLI is left untouched. Call once, right after
    ``parse_args()``, before any of these fields are read for anything else."""
    if args.color is None:
        args.color = config.color
    if args.fov_h is None:
        args.fov_h = config.fov_h
    if args.fov_v is None:
        args.fov_v = config.fov_v
    if args.replay_fps is None:
        args.replay_fps = config.replay_fps
    if args.port is None:
        args.port = config.port
