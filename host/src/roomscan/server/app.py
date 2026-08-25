"""FastAPI application factory, lifespan handlers, CORS, and middleware."""
from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from zeroconf import Zeroconf

from ..thin_render import ThinRenderer
from .session import (
    RESULTS_DIR,
    _bridge_health_loop,
    _comm_probe_loop,
    _idle_reconcile_loop,
    _init_ranging_state,
)
from .mdns import register_mdns, unregister_mdns
from .routes import (
    api_bridge_logs,
    api_bridge_reboot,
    api_restart,
    get_capture_video,
    get_thumb,
    root_redirect,
)
from .ws_broadcaster import (
    _broadcaster,
    websocket_endpoint,
    websocket_mesh_endpoint,
)
from .ws_thin import websocket_thin_endpoint

log = logging.getLogger("roomscan.web")

@asynccontextmanager
async def _lifespan(app: FastAPI):
    # Start the single broadcast task once the server begins serving. app.state
    # is fully populated by main() before uvicorn.run(); guard the (test/import)
    # case where it isn't so a bare `import roomscan.web` never spins a task.
    if getattr(app.state, "ready", False):
        app.state.broadcast_task = asyncio.create_task(_broadcaster())
        app.state.idle_reconcile_task = asyncio.create_task(_idle_reconcile_loop())
        # Task 10 item 1: restore ranging/IMU-env state from the device's own
        # GET_RANGING_CONFIG/GET_IMU_ENV_RATE readback, never an assumed firmware
        # default. One-shot; runs concurrently with the broadcaster so server
        # startup isn't gated on the device's ACK latency.
        app.state.ranging_init_task = asyncio.create_task(_init_ranging_state(app.state))
        # Top-bar comm indicator (owner ask, 2026-08-05): a slow, independent
        # probe loop -- decoupled from the frame broadcaster so a ping stall can
        # never touch the stream, and vice versa.
        app.state.comm_probe_task = asyncio.create_task(_comm_probe_loop(app.state))
        # Pi bridge ssh health (#193): independent, much slower loop -- see its
        # own docstring for why this must not share the ICMP cadence above.
        app.state.bridge_health_task = asyncio.create_task(_bridge_health_loop(app.state))
        # Advertise this server so a thin client (and any future one) can find
        # it without a hardcoded IP. Off the event loop: `register_service`
        # blocks on multicast announcement.
        app.state.mdns = await asyncio.to_thread(
            register_mdns, getattr(app.state, "port", 8000),
            zeroconf_factory=getattr(app.state, "zeroconf_factory", Zeroconf))
    yield
    task = getattr(app.state, "broadcast_task", None)
    if task is not None:
        task.cancel()
    idle_task = getattr(app.state, "idle_reconcile_task", None)
    if idle_task is not None:
        idle_task.cancel()
    ranging_task = getattr(app.state, "ranging_init_task", None)
    if ranging_task is not None:
        ranging_task.cancel()
    comm_task = getattr(app.state, "comm_probe_task", None)
    if comm_task is not None:
        comm_task.cancel()
    bridge_health_task = getattr(app.state, "bridge_health_task", None)
    if bridge_health_task is not None:
        bridge_health_task.cancel()
    detailed = getattr(app.state, "detailed_runner", None)
    if detailed is not None:
        await asyncio.to_thread(detailed.close)
    splat_runner = getattr(app.state, "splat_runner", None)
    if splat_runner is not None:
        await asyncio.to_thread(splat_runner.close)
    mdns = getattr(app.state, "mdns", None)
    if mdns is not None:
        await asyncio.to_thread(unregister_mdns, mdns)
        app.state.mdns = None
    # Destroy the offscreen context on its owning thread -- letting the
    # interpreter collect a live one aborts the process (see thin_render).
    # `instance_if_created`, never `instance`: a process that never served a
    # thin client must not build a renderer just to close it.
    thin_renderer = ThinRenderer.instance_if_created()
    if thin_renderer is not None:
        await asyncio.to_thread(thin_renderer.close)




static_dir = Path(__file__).resolve().parents[1] / "static"
static_dir.mkdir(exist_ok=True)

_results_dir = Path(RESULTS_DIR)
_results_dir.mkdir(exist_ok=True)


def create_app() -> FastAPI:
    """FastAPI application factory."""
    application = FastAPI(lifespan=_lifespan)

    application.mount("/static", StaticFiles(directory=str(static_dir), html=True), name="static")
    application.mount("/results", StaticFiles(directory=str(_results_dir)), name="results")

    application.add_api_route("/", root_redirect, methods=["GET"], include_in_schema=False)
    application.add_api_route("/api/bridge/reboot", api_bridge_reboot, methods=["POST"], include_in_schema=False)
    application.add_api_route("/api/bridge/logs", api_bridge_logs, methods=["POST"], include_in_schema=False)
    application.add_api_route("/api/restart", api_restart, methods=["POST"], include_in_schema=False)
    application.add_api_route("/thumb/{name}", get_thumb, methods=["GET"], include_in_schema=False)
    application.add_api_route("/capture_video/{name}", get_capture_video, methods=["GET"], include_in_schema=False)

    application.add_api_websocket_route("/ws", websocket_endpoint)
    application.add_api_websocket_route("/ws-mesh", websocket_mesh_endpoint)
    application.add_api_websocket_route("/ws-thin", websocket_thin_endpoint)

    return application


app = create_app()

__all__ = [k for k in globals() if not k.startswith("__")]
