# Headless-host setup — running the web viewer on a GPU-less Linux box

The web viewer (`./view-web.sh`, `roomscan.web`) was developed on Windows with a
GPU and a USB-tethered board. Bringing it up on a **headless Linux host**
(Proxmox/LXC, no GPU, GUI over VNC, board on Ethernet) exposed four gaps that
were all implicit on the dev box. This is the 5-minute checklist so a fresh host
doesn't repeat the multi-hour dig (session 2026-07-15; BUG-020..022 in `BUGS.md`).

## TL;DR

```sh
host/.venv/bin/python host/tools/headless_doctor.py --build   # verify + build the .so
./view-web.sh                                                 # then launch
```

`headless_doctor.py` runs every check below and prints the exact fix for any
failure. Green across the board ⇒ `./view-web.sh` shows live data.

On a box with the systemd service installed (the dev box — see "Keeping the
server up" below), `./view-web.sh` **delegates to the service** instead of
spawning a competing server: a plain call just ensures the managed unit is up
and prints the URL; passing args (`./view-web.sh --replay …`) runs a one-off and
restores the live service on exit. Day-to-day you manage it directly with
`systemctl --user {start,stop,restart} roomscan-web`.

## The four gaps (why each check exists)

1. **Native transform library** — the board streams *raw* frames; the PC runs
   the `vl53l9-transform-c` pipeline through a compiled C lib. It must be built
   (`host/transform/build/libroomscan_transform.so`) and its sources
   (`firmware/vendor/53L9A1`) must be present. The loader is cross-platform as of
   BUG-020 (`.so`/`.dylib`/`.dll`, searches `build/` and `build/Release/`).
   Without it every frame throws and the viewer is silently blank.
   Build: `cmake -S host/transform -B host/transform/build -DCMAKE_BUILD_TYPE=Release && cmake --build host/transform/build`.

2. **Board must be woken continuously** — the firmware unicasts frames to
   whichever host last sent it a datagram and clears that only on reboot.
   `UdpSource` now sends a 1 s keepalive wake so the stream self-heals after a
   board reset / link flap. If data stops, check the board is powered with a live
   Ethernet LINK LED; `avahi-browse -rt _roomscan._udp` should list it live
   (a stale avahi cache can resolve `roomscanner.local` to a dead IP — the
   service browse is the real liveness test).

3. **No CDN at render time** — three.js is vendored under
   `host/src/roomscan/static/vendor/three/`; the import map points local. A
   headless/remote browser that can't reach unpkg would otherwise fail the
   `three` import and the page sits at "Offline" (BUG-021).

4. **Software WebGL** — a GPU-less host has no hardware WebGL, and Chrome refuses
   software WebGL by default, so `new THREE.WebGLRenderer()` throws "Error
   creating WebGL context" (BUG-022). The host *does* have Mesa llvmpipe
   (GL 4.5). `view-web.sh`'s auto-open launches Chrome with
   `--enable-unsafe-swiftshader`; to open manually:
   `google-chrome --enable-unsafe-swiftshader http://localhost:8000/static/index.html`
   (`--use-gl=angle --use-angle=gl` uses llvmpipe instead of SwiftShader).

## Diagnosing "Offline" — read the on-page diagnostic panel

The viewer has a **diagnostic panel (bottom-left)** and an inline error trap that
fires even when a module import fails silently. The trace pinpoints the layer:

| Panel shows | Meaning |
|---|---|
| no `app.js: module loaded` line (+ `window.error`) | a JS module/asset failed to load — the error line names it |
| `Error creating WebGL context` | no WebGL — launch Chrome with `--enable-unsafe-swiftshader` (gap 4) |
| `ws CLOSE code=1006` | websocket can't reach the server (wrong host / firewall / proxy) |
| `ws OPEN` then `ws TEXT frame: …error…` | connected, but the server reader faulted — message says why |
| ends in `first binary frame: N pts` | fully working |

The first panel line (`protocol=… host=…`) tells you *where the browser actually
is* — the most useful clue when the viewing browser isn't on the host itself.

## Keeping the server up — the supervised systemd service (#207)

On the dev box the server runs as an **always-on systemd *user* service** so a
session ending (or a crash) never leaves the host with no server. Linger is
enabled for the user, so it starts at boot and survives logout.

```sh
systemctl --user status roomscan-web        # is it up? how many restarts?
journalctl --user -u roomscan-web -f        # live logs
systemctl --user restart roomscan-web       # reload after a code change
systemctl --user stop roomscan-web          # free :8000 + the device stream
systemctl --user disable --now roomscan-web # turn the supervisor off entirely
```

Unit: `~/.config/systemd/user/roomscan-web.service`, tracked in the repo at
`host/deploy/roomscan-web.service` (with its one-time install steps in the header)
— runs `host/.venv/bin/python -m roomscan.web` from the repo root with
`Restart=always`, `RestartSec=2`, and
`Environment=ROOMSCAN_MANAGED=roomscan-web.service`.

**`Restart=always` makes systemd the single lifecycle authority — do not fight
it.** Anything that stops the server out-of-band (`kill`, a bare
`terminate()`, a second `python -m roomscan.web`) is either undone within
`RestartSec` or races the supervised copy for port 8000 + the device stream (a
double-bind crash-loop). So:

- **Reload new code with `systemctl --user restart`**, not `kill` + `setsid
  python -m roomscan.web`. The old manual-relaunch recipe now collides with the
  supervisor.
- **The `ROOMSCAN_MANAGED` marker teaches the server to cooperate**: `/api/restart`
  (the "Restart Server" button) detects it and simply exits, letting `Restart=always`
  respawn — it does *not* spawn its own relaunch child.
- **The agent lifecycle tools drive the unit** when it is installed: `rig_down()`
  `systemctl --user stop`s it (so it stays down for an unattended `capture.py --udp`
  or exclusive device access); `rig_up()` (no replay) `systemctl --user start`s it;
  `rig_up(replay=…)` stops it first so the replay can bind, and `rig_up()` restores
  the live server afterwards. On a box **without** the unit these tools fall back to
  the old bare-process path unchanged.
- **Recording while the UI stays live still goes through the server's own recorder**
  (`rig_record()` / the `/ws` `record` message), not `capture.py` — the service
  owning the stream doesn't change that (`recording-vs-web-server` memory).

## Environment notes

- **No USB link to this host** — Ethernet/UDP is the only transport here; the USB
  CDC fallback is dead. See the `headless-host-deployment` memory.
- **Running the server as an agent**: use `rig_up()` / `rig_down()` from
  `roomscan-mcp` (`docs/mcp-server.md`), which starts it detached and waits for the
  UI to answer. Verify the browser via `ui_screenshot()`.
  *Stale claim, re-measure before relying on it (2026-07-29):* this bullet used to
  say the agent Bash sandbox kills network-listener processes (uvicorn → exit 144),
  and the `agent-sandbox-port-binding` memory still does. That did **not** reproduce
  — uvicorn bound `0.0.0.0`, served a request and exited 0 from a normal Bash call.
  It may have been accurate when written.
