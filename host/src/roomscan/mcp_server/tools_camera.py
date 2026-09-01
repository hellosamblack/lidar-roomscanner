"""The rig-mounted Pi-bridge camera (#226 Stage A), read through `roomscan-web`.

Both tools are thin wrappers over pure functions in `roomscan.camera`; the
logic lives there and is shared with the HTTP routes and the browser card.

**These go through `roomscan-web`, never straight to the Pi.** The camera has
exactly one ingest connection, owned by the web server's lifespan, for the same
reason the MCP server never binds the device UDP/CDC stream: a second consumer
does not observe the system, it changes it. Asking the daemon directly would
open a second upstream connection, and its `clients` count -- the number this
tool reports -- would then be measuring the measurer.

If `camera_status()` says the server is not up, start it with `rig_up()`.
"""
from __future__ import annotations

from .server import mcp


@mcp.tool()
def camera_status() -> dict:
    """Live state of the rig camera: link state, real resolution, rates, drops.

    Reports what was measured, not what was requested:

      state              offline | connecting | live | stale. `stale` means the
                         upstream connection is open but frames stopped -- a
                         different fault from `offline`, and it is the one a
                         frozen preview usually is.
      width/height       read from the last JPEG's own SOF marker, so a daemon
                         that negotiated a lower mode than it advertises is
                         visible here rather than hidden.
      capture_fps        cadence on the PI's monotonic clock; null when the
                         daemon supplies no usable capture timestamp, and null
                         (not a stale number) once frames stop arriving.
      host_fps           arrival cadence measured on THIS host.
      age_ms             an arrival age -- how long ago the newest frame reached
                         this host. It is NOT a display latency and NOT a
                         capture-to-now delay: no camera clock has been mapped
                         to this one, and Stage A makes no synchronization
                         claim at all.
      source_drops       gaps in the daemon's sequence numbers -- frames the
                         camera produced that never reached this host.
      ingest_drops       frames this host received that a viewer was too slow
                         to take. A bounded queue that dropped silently would
                         read as healthy from here; this is the counter that
                         makes it not.
      clients            proxy viewers attached right now. Connection count is
                         a real performance variable on this server.
      source             the Pi daemon's own `/status.json`, passed through
                         verbatim -- device identity, controls, restart count.

    Null means unknown; zero means measured zero. Never conflate them.
    """
    from ..camera import fetch_camera_status

    from .paths import WEB_URL

    return fetch_camera_status(WEB_URL)


@mcp.tool()
def camera_snapshot(out: str = "") -> dict:
    """Save the newest camera frame to a file and report the path written.

    The JPEG is the camera's exact bytes as the host received them -- no
    re-encode, no canvas round trip -- so it is the same artifact Stage B
    (#227) will archive as authoritative.

    `out` defaults to `results/camera/camera-<unix-ms>.jpg`. A relative path is
    taken against the repo root, and any path is refused unless it resolves
    inside `results/` or `captures/`. The result reports the path and byte
    count actually on disk plus the frame's generation/sequence, not the
    arguments handed in.

    `{"ok": false}` with a `hint` when there is no frame: check
    `camera_status()` -- the usual causes are the camera daemon not running on
    the Pi and the camera not being plugged in.
    """
    from ..camera import save_camera_snapshot

    from .paths import REPO, WEB_URL

    return save_camera_snapshot(WEB_URL, out, repo=REPO)
