"""Web-based real-time instrument.

Decomposed into modular subpackages in ``roomscan.server`` (#196). This module
is a 100% backwards-compatible facade that re-exports all symbols from
``roomscan.server`` and retains the primary CLI entry point ``main()``.
"""
from __future__ import annotations

import os
import queue
import shutil
import subprocess
import sys
import threading
import time
import types
import webbrowser
from dataclasses import replace
from pathlib import Path

import uvicorn

from . import commstatus
from . import thumbs as thumbs_mod
from .config import ViewerConfig
from .control import (
    CommandClient,
    CommandDispatcher,
)
from .flatfield import FlatFieldSet
from .logbus import LogBus
from .magcal import MagCalibration
from .magsweep import MagSweepSession
from .metrics import MetricsRegistry, ResourceSampler
from .pipeline import TransformStage
from .protocol import (
    CommandCode,
    RangingConfigAck,
    ResultCode,
    StandbyLevel,
)
from .reader import _Pacer
from .sensors import (
    SensorState,
    YawFusion,
    display_rotation,
    wrap180,
)
from .slam.detailed import sidecar_paths
from .sources import (
    Recorder,
    SerialSource,
    UdpSource,
    get_best_source,
)
from .viewer import Stats, resolve_args
from .weather import MslPressure

# Re-export all server subpackage symbols for 100% backwards compatibility
from .server import *
from . import server as _server_mod

for _k, _v in vars(_server_mod).items():
    if not _k.startswith("__"):
        globals()[_k] = _v


# --- CLI entry point --------------------------------------------------------

def main(argv=None) -> int:
    args = resolve_args(argv)

    # Web Phase 3: the reader lifecycle is owned by a SessionController so the
    # source can be swapped live<->replay at runtime. Launched with --replay we
    # start with NO live device (has_live False, Go Live disabled); otherwise we
    # open the live source once and keep it for the whole process, reused across
    # replay excursions via the _NoCloseSource proxy.
    live_source = None if args.replay else get_best_source(args.port, args.baud)
    # Name the transport up front: the #1 "no data" question is whether we're on
    # Ethernet, serial, or a dead serial fallback. Flushed so it shows even when
    # stdout is block-buffered (not a tty).
    if isinstance(live_source, UdpSource):
        live_label = f"Ethernet/UDP · {live_source.target_ip}:{live_source.target_port}"
    elif isinstance(live_source, SerialSource):
        live_label = f"Serial CDC · {getattr(live_source, 'port', '?')}"
    else:
        live_label = "no device"
    if live_source is not None:
        print(f"[source] {live_label}", flush=True)
    else:
        print(f"[source] Replay -> {args.replay}", flush=True)

    # client is None in replay (no device to command); the reader passes it only
    # in live mode, and the cmd handler reports "not available in replay" itself.
    client = CommandClient(live_source.write) if isinstance(live_source, (SerialSource, UdpSource)) else None
    stats = Stats()
    bus = LogBus()
    # Resource monitor (owner ask, 2026-07-31). The sampler has existed since
    # Phase 1 but was never constructed, so `metrics.resources` was hardcoded
    # null. It is a daemon thread doing psutil + NVML sampling every 0.7 s; on
    # a GPU-less box the NVML fields report "n/a" and everything still works.
    # The ctypes device-utilization fallback belongs on the sampler thread.
    # `resources_to_dict` runs on the 4 Hz asyncio broadcaster and must remain
    # a cached projection: opening/querying NVML there measured ~40 ms and was
    # enough to pull a negotiated 30 fps thin feed down to ~26 fps (#197).
    resource_sampler = ResourceSampler(device_util_probe=_device_gpu_util)
    resource_sampler.start()
    metrics = MetricsRegistry(window_s=2.0, sampler=resource_sampler)
    dispatcher = CommandDispatcher(client, on_message=bus.publish)

    # Always compute all three planes: marginal cost per plane is ~zero and it
    # makes color mode a pure runtime choice (no reader restart) -- §5.1/§7.2.
    # Mode-aware flat-field: the map is chosen by the device's ranging mode
    # (`stage.set_ranging_mode`, driven from the ranging-config ACK) and gated by
    # the "Calibrated" toggle (`stage.flatfield_enabled`, seeded from config below
    # once ui_state exists). Empty set => no correction, so this is safe with no
    # maps configured.
    stage = TransformStage(outputs=("depth", "reflectance", "confidence"),
                           flatfield=FlatFieldSet.load_configured())
    slot: queue.Queue = queue.Queue(maxsize=1)
    fault: dict = {}

    # Sensor state (streams 9/10) -- built exactly like the desktop panel
    # (panel.py:525-541), reusing SensorState + YawFusion + MagCalibration.
    # getattr defaults cover viewer.resolve_args not defining the panel's sensor
    # flags; a missing mag_cal.json just leaves fusion in gated:no-cal.
    mag_cal = None
    fusion = None
    # Resolved ONCE so the web calibration modal saves to exactly the file the
    # loader (and panel.py:440) reads -- same expression, no second convention.
    mag_cal_path = getattr(args, "mag_cal_path", "mag_cal.json") or "mag_cal.json"
    if getattr(args, "yaw_fusion", True):
        mag_cal = MagCalibration.load(mag_cal_path)
        fusion = YawFusion(
            tau_s=float(getattr(args, "yaw_fusion_tau", 20.0) or 20.0),
            calibration=mag_cal,
            anomaly_frac=float(getattr(args, "yaw_anomaly_frac", 0.3) or 0.3),
            motion_rate_dps=float(getattr(args, "yaw_motion_rate_dps", 40.0) or 40.0),
        )
    sensor_state = SensorState(fusion=fusion)

    # Keep the selected speed on the controller even while live. A later View
    # swap reuses it, so seeding live with Max here would make ordinary playback
    # start at Max despite the replay-launch path defaulting to 1×.
    initial_speed_fps = (float(args.replay_fps) if (args.replay and args.replay_fps and args.replay_fps > 0)
                         else _SPEED_BASE_FPS)
    pacer = _Pacer(interval=speed_to_interval(initial_speed_fps) if args.replay else 0.0)
    recorder = Recorder()

    controller = SessionController(
        live_source=live_source, live_label=live_label, stage=stage, stats=stats,
        slot=slot, fault=fault, bus=bus, client=client, recorder=recorder, pacer=pacer,
        sensor_state=sensor_state, metrics=metrics, captures_dir=CAPTURES_DIR,
        initial_replay_path=args.replay, initial_speed_fps=initial_speed_fps,
        # Feeds the SLAM mapper at stream rate, off the broadcaster (BUG-060).
        # Reads `app.state` lazily, so it is safe to build before the rest of
        # app.state exists -- the reader thread only starts at controller.start().
        on_frame=make_slam_feed(app.state),
        # Fresh ephemeral map on every timeline discontinuity (BUG-091), run in
        # the no-reader window inside the swap so nothing can feed the old map.
        on_discontinuity=make_slam_discontinuity(app.state))

    # SLAM mode (web Phase 4): armed lazily on the first `set_mode slam`; builds
    # no Open3D/GPU state until then, so real-time launches are unaffected.
    # `resource_sampler` is already running above for the `metrics` message;
    # reusing it here (plan item 2) means the `slam` message's GPU/VRAM fields
    # cost nothing extra on the broadcaster's tick -- see `_slam_gpu_fields`.
    slam_runner = SlamRunner(bus=bus, fov_h=args.fov_h, fov_v=args.fov_v,
                             resource_sampler=resource_sampler)

    # Shared app state, built once (§5.1).
    app.state.args = args
    app.state.source = live_source
    app.state.controller = controller
    app.state.recorder = recorder
    app.state.client = client
    app.state.stage = stage
    app.state.slot = slot
    app.state.bus = bus
    app.state.metrics = metrics
    app.state.resource_sampler = resource_sampler
    app.state.dispatcher = dispatcher
    app.state.fault = fault
    app.state.fault_reported = False
    app.state.stats = stats
    app.state.pacer = pacer
    # Settings persistence (Web Phase 5): seed the UI from the shared
    # roomscan.toml [viewer] table and keep the loaded config around so runtime
    # display-pref changes write straight back to it. `mode` is not restored --
    # SLAM is armed lazily, so a restart always comes up in real-time.
    config = ViewerConfig.load()
    app.state.config = config
    app.state.ui_state = ui_from_config(config)
    # Seed the "Calibrated" toggle from persisted config now that ui_state exists
    # (the stage was built earlier, before ui_state). The ranging mode is pushed in
    # later by the first ranging-config ACK via `_commit_ranging_ack`.
    stage.flatfield_enabled = app.state.ui_state.calibrated
    if controller.mode == "replay":
        app.state.ui_state.source = "view"
        app.state.ui_state.selected_capture = os.path.basename(controller.replay_path)
    app.state.sensor_state = sensor_state
    app.state.mag_cal = mag_cal
    # Magnetometer sweep modal (owner ask, 2026-07-29). `fusion` is held so a
    # Save can hot-reload the filter's calibration too (install_mag_calibration);
    # `magcal_clients` is the per-tab subscriber set (empty => zero cost).
    app.state.mag_cal_path = mag_cal_path
    app.state.fusion = fusion
    app.state.magcal_session = MagSweepSession()
    app.state.magcal_clients = set()
    app.state.magcal_view = "current"
    app.state.slam_runner = slam_runner
    app.state.detailed_runner = DetailedRunner(bus=bus, results_dir=RESULTS_DIR)
    app.state.splat_runner = SplatRunner(bus=bus, results_dir=RESULTS_DIR,
                                         captures_dir=CAPTURES_DIR)
    app.state.deproj = None
    # Latest ToF per-frame metadata (exposure/die-temp/health), decoded host-side
    # from the RAW tail in pipeline.py and cached here so the ~15 Hz sensor tick
    # (decoupled from the ToF frame tick) always has the freshest value.
    app.state.tof_meta = None
    app.state.orientation_smoother = OrientationSmoother()
    app.state.orientation_jitter = OrientationJitter()
    # Elevation readout (owner ask, 2026-07-31): the display EMA and the cached
    # sea-level reference. Built here so `set_elevation_datum` can read the
    # current smoothed value even before the first broadcaster tick.
    app.state.elevation_smoother = ElevationSmoother()
    app.state.msl_pressure = MslPressure(
        float(getattr(config, "latitude", 45.014060)),
        float(getattr(config, "longitude", -93.245526)),
        refresh_s=float(getattr(config, "msl_refresh_s", 1800.0) or 1800.0))
    app.state.clients = set()
    # `/ws-mesh` credit-gated MESH transport (BUG-061 Part A): the per-flow
    # map and the cached latest mesh the pump resends from -- see
    # `_mesh_clients`/`_cache_latest_mesh`/`_pump_mesh`.
    app.state.mesh_clients = {}
    app.state.latest_mesh = None
    app.state.latest_mesh_seq = 0
    # `/ws-thin` server-rendered raster feed (#194): the per-connection flow map
    # and the generation-tagged stashes the render loop draws from -- see
    # `_thin_clients` / `thin_scene_for`.
    app.state.thin_clients = {}
    app.state.thin_latest_pc = None
    app.state.thin_latest_ir = None
    app.state.thin_latest_sensor = None
    app.state.thin_mesh_cache = None
    app.state.command_labels = set()
    app.state.debounce = {}
    # Ranging profiles & manual sensor control (Task 10). Populated for real by
    # `_init_ranging_state` (GET_RANGING_CONFIG/GET_IMU_ENV_RATE) once the
    # broadcaster starts -- these are the "not yet initialized" defaults, not
    # an assumed firmware default.
    app.state.ranging_state = RangingState()
    app.state.imu_env_state = ImuEnvRateState()
    # Sensor auto-idle (SET_STANDBY): idle_enabled/idle_level live in ui_state
    # (persisted); the debounce delay and the runtime bookkeeping live here.
    app.state.idle_delay_s = float(getattr(config, "sensor_idle_delay_s", 5.0) or 5.0)
    app.state.sensor_idled = False   # whether we've commanded the device into standby
    app.state.idle_timer = None      # asyncio TimerHandle for the debounced idle
    # "Actively engaged" bookkeeping (Phase A/A.5): which `/ws` and `/ws-mesh`
    # sockets belong to which `client_id`, and when each `client_id` was last
    # seen doing something -- see the module comment above `_resolve_client_id`.
    app.state.ws_client_id = {}
    app.state.client_active = {}
    app.state.activity_timeout_s = float(
        getattr(config, "sensor_idle_activity_timeout_s", 60.0) or 60.0)
    app.state.ready = True

    # The controller owns the reader thread now (Web Phase 3): it runs the same
    # reader._run_reader body, but can stop+respawn it against a new source for
    # capture load / Go Live / seek, and tees raw bytes into the Recorder.
    controller.start()

    # A previous server may have left the sensor in standby (SET_STANDBY persists
    # on the device across a host restart). Guarantee it is streaming now, whatever
    # this launch's idle setting is -- a harmless no-op if the device is already
    # active (firmware acks without touching the sensor). Fire-and-forget; the ACK
    # (or a startup-race timeout) just lands on the log bus.
    if live_source is not None:
        _dispatch_standby(app.state, int(StandbyLevel.ACTIVE), "startup-wake")

    port = 8000
    # `_lifespan` advertises this over mDNS and cannot see a local.
    app.state.port = port
    url = f"http://localhost:{port}/static/index.html"
    print("\n=== roomscan web viewer ===")
    print(f"Starting server on {url}")
    print("Press Ctrl+C to stop.")

    # Small delay to let the server start before opening the browser.
    threading.Timer(1.0, lambda: _open_browser(url)).start()

    # permessage-deflate OFF (BUG-060). uvicorn defaults it ON, and
    # `_broadcast_bytes` awaits `send_bytes` PER CLIENT -- so every binary
    # payload is zlib-deflated once per connected tab, synchronously on the
    # event loop. Measured on the live rig with a ~63k-vertex SLAM mesh
    # (3.2 MB): +130 ms of whole-process freeze per additional client (0->1
    # +128 ms, 1->2 +133 ms), against +0 ms for clients that negotiate no
    # compression. Six clients (one real browser, five stale headless/MCP
    # connections) froze the server 630 ms out of every second. What the cost
    # bought: float32 geometry deflates to 80%. Turning it off trades ~20% more
    # LAN bytes for the loop back.
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning",
                ws_per_message_deflate=False)
    return 0


def _open_browser(url: str) -> None:
    """Open the viewer, and on Linux launch Chrome/Chromium with software-WebGL
    enabled. On a headless host (no GPU -- the whole point of this deployment)
    Chrome refuses to create a WebGL context by default, so the Three.js viewer
    dies with 'Error creating WebGL context' and the page is stuck at 'Offline'
    (confirmed on-box 2026-07-15: baseline Chrome -> NO-WEBGL; with the flag ->
    WEBGL-OK via SwiftShader/llvmpipe). `--enable-unsafe-swiftshader` only
    *permits* the software fallback -- a machine with a real GPU still uses it,
    so this is safe to pass unconditionally. Set ROOMSCAN_NO_BROWSER=1 to skip
    the auto-open entirely (e.g. when viewing from another machine)."""
    if os.environ.get("ROOMSCAN_NO_BROWSER"):
        print(f"[browser] auto-open disabled; open {url} yourself.", flush=True)
        return
    if sys.platform.startswith("linux"):
        for exe in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser"):
            path = shutil.which(exe)
            if path:
                try:
                    subprocess.Popen(
                        [path, "--enable-unsafe-swiftshader", url],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print(f"[browser] opened {exe} with software WebGL enabled.", flush=True)
                    return
                except Exception as exc:
                    print(f"[browser] {exe} launch failed ({exc}); falling back.", flush=True)
                    break
        print("[browser] no Chrome/Chromium found. If the viewer shows 'Offline' "
              "with a WebGL error, launch your browser with software WebGL "
              "(Chrome: --enable-unsafe-swiftshader).", flush=True)
    webbrowser.open(url)


class _WebFacade(types.ModuleType):
    """Facade module synchronizing monkeypatched attributes across subpackages."""

    def __setattr__(self, name: str, value: object) -> None:
        super().__setattr__(name, value)
        if name.startswith("__"):
            return
        for submod in (
            getattr(_server_mod, "_session_mod", None),
            getattr(_server_mod, "_ws_broadcaster_mod", None),
            getattr(_server_mod, "_ws_thin_mod", None),
            getattr(_server_mod, "_routes_mod", None),
            getattr(_server_mod, "_app_mod", None),
            getattr(_server_mod, "_mdns_mod", None),
            _server_mod,
        ):
            if submod is not None:
                submod.__dict__[name] = value


sys.modules[__name__].__class__ = _WebFacade


if __name__ == "__main__":
    sys.exit(main())
