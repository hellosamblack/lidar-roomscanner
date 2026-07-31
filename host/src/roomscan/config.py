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
    view_colormap: str = "turbo"
    web_point_size: float = 0.025
    web_point_size_auto: bool = True   # scale each point with its range from the sensor
                                       # (then web_point_size is the size at 1 m of range)
    web_view_mode: str = "world"       # web real-time view: "world" (orbit a gravity-aligned
                                       # scene) | "fpv" (camera locked to the sensor) | "mirror"
                                       # (fpv, left-right flipped). Distinct from `camera` below,
                                       # which is the desktop panel's own first_person|orbit field.
    # Camera framing per web view mode, as an offset from the FPV baseline (a
    # camera at the sensor looking down its boresight; all zeros reproduces it
    # exactly). Nine flat floats because the TOML writer is scalar-only --
    # grouped and validated in web.py (`_VIEW_CAM_CONFIG_KEYS`).
    web_cam_world_distance_m: float = 4.2
    web_cam_world_height_m: float = 2.6
    web_cam_world_rotation_deg: float = 0.0
    web_cam_fpv_distance_m: float = 0.30
    web_cam_fpv_height_m: float = 0.20
    web_cam_fpv_rotation_deg: float = 0.0
    web_cam_mirror_distance_m: float = 0.30
    web_cam_mirror_height_m: float = 0.20
    web_cam_mirror_rotation_deg: float = 0.0
    web_orbit_enabled: bool = False    # world view only: slow auto-orbit (azimuth only)
    web_orbit_speed_deg_s: float = 6.0 # 60 s per revolution; negative reverses
    web_orbit_mode: str = "continuous" # "continuous" (the original behaviour) | "oscillate"
                                       # (owner ask, 2026-07-31): a triangle wave about the
                                       # azimuth where oscillation started, +-web_orbit_amplitude_deg
    web_orbit_amplitude_deg: float = 45.0  # oscillate mode's half-swing, degrees
    web_see_through: float = 0.0       # x-ray strength 0..1: how strongly geometry hidden BEHIND
                                       # other geometry is drawn back over its occluder. 0 = off
                                       # (opaque, the historical look), 1 = the hidden layer wins
    imu_gizmo: bool = True             # show the orientation gizmo in the scene
    sensors_panel: bool = True         # show the Sensors panel group
    gizmo_scale: float = 0.15          # gizmo axis length (metres)
    metrics_overlay: bool = True       # show the on-scene metrics HUD (rates/fps/resources)
    mode: str = "real_time"            # UI redesign: "real_time" | "slam" (owner: default to real-time)
    camera: str = "first_person"       # UI redesign: "first_person" | "orbit"
    slam_trajectory: bool = True       # web SLAM display toggle: draw the trajectory tail
    slam_walls: str = "split"          # web SLAM wall render: "solid" | "split" (MeshPrep wall_mode)
    slam_follow: bool = True           # web SLAM display toggle: follow-camera on/off
    ir_overlay: bool = True             # first-person IR billboard overlay on/off (matches
                                        # ir_opacity's non-zero default -- see _set_ir_opacity's
                                        # "opacity > 0.02 implies enabled" invariant; a fresh
                                        # install shouldn't start with the slider and the
                                        # visibility gate disagreeing)
    ir_opacity: float = 0.5            # IR overlay opacity 0..1
    sensor_idle_enabled: bool = True        # auto-idle the ToF laser (VCSEL) when no web
                                            # viewer is connected, to reduce wear
    sensor_idle_level: str = "soft"         # depth of the auto-idle: "soft" (FSM standby,
                                            # instant resume) | "hard" (XSHUT power-down)
    sensor_idle_delay_s: float = 5.0        # debounce after the last viewer disconnects
                                            # before idling (so a tab reload doesn't thrash
                                            # the sensor FSM)
    yaw_fusion: bool = True                 # graft mag heading onto SFLP yaw
    flatfield_path: Optional[str] = None    # path to a per-zone reflectance FPN
                                            # correction (.npz from tools/build_flatfield.py);
                                            # None disables correction (see flatfield.py)
    yaw_fusion_tau: float = 20.0            # complementary-filter time constant (s)
    mag_cal_path: str = "mag_cal.json"      # hard/soft-iron calibration JSON
    yaw_anomaly_frac: float = 0.3           # |mag| deviation from field to reject
    yaw_motion_rate_dps: float = 40.0       # quat angular rate above which to freeze
    yaw_gimbal_margin_deg: float = 15.0     # freeze within this of |pitch|=90
    orientation_mode: str = "world"         # selected orientation decomposition (owner ask,
                                            # 2026-07-28): "zyx" | "zxy" | "boresight" | "world".
                                            # Defaulted to "world" 2026-07-31 -- the Sensors card's
                                            # decomposition picker was removed (the owner only ever
                                            # used World); the other modes stay valid on the wire
                                            # for the deprecated desktop panel, but a fresh install
                                            # (and `ui_from_config` coercing any stored non-world
                                            # value, so an old config can't strand the UI in a mode
                                            # it no longer offers a picker for) now starts here.
    orientation_labels: str = "Roll,Tilt,Heading"  # user-renamable axis labels, comma-joined (the
                                            # flat-TOML writer is scalar-only -- no list/array
                                            # support, see the module docstring). In World mode
                                            # these are NOT Roll/Pitch/Yaw -- see
                                            # web.py's DEFAULT_AXIS_LABELS.
    yaw_offset_deg: float = 0.0             # "Zero yaw here" (owner ask, 2026-07-29): a
                                            # user-set world-Z graft applied to the relative
                                            # yaw-like slot of zyx/zxy/boresight ONLY -- World
                                            # mode's heading is absolute (magnetic) and is never
                                            # offset. Presentation-only, scalar so it fits this
                                            # writer -- see web.py's `build_sensor_message`.

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
        for _optkey in ("port", "flatfield_path"):
            if kwargs.get(_optkey) == "":
                kwargs[_optkey] = None  # TOML has no null; empty string round-trips "unset"
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
