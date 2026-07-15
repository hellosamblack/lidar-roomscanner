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

## Environment notes

- **No USB link to this host** — Ethernet/UDP is the only transport here; the USB
  CDC fallback is dead. See the `headless-host-deployment` memory.
- **Running the server as an agent**: the agent Bash sandbox kills network-listener
  processes (uvicorn → exit 144). Verify the data path directly
  (`get_best_source`→`pump`→`TransformStage`) and the browser via headless-Chrome
  screenshot; see the `agent-sandbox-port-binding` memory.
