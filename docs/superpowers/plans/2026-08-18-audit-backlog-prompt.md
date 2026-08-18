# Future Agent Prompt & Backlog Execution Guide

**Date:** 2026-08-18  
**Reference Documents:**
* Comprehensive Audit: [`docs/engineering-audit-2026-08-18.md`](file:///home/sam/git/personal/lidar-roomscanner/docs/engineering-audit-2026-08-18.md)
* Thin-Client Architecture: [`docs/thin-client.md`](file:///home/sam/git/personal/lidar-roomscanner/docs/thin-client.md)
* Agent Governance & Invariants: [`AGENTS.md`](file:///home/sam/git/personal/lidar-roomscanner/AGENTS.md)

---

## Part 1: Ready-to-Use Agent Prompt

Copy and paste the prompt below into a new Claude / Antigravity / Codex agent session to begin execution immediately:

```markdown
You are an expert embedded systems + Python + real-time 3D vision engineer working in `lidar-roomscanner`.

### Context & Standing Invariants
1. Read `AGENTS.md` and follow all binding repo rules:
   - Run the `session-start` skill BEFORE your first edit (anchor to an open GitHub Issue, check for conflicts, comment on the issue, format commits with `Refs #NNN`).
   - Run `session-end` and `operator-request` skills before closing any issue. Never close on unverified work.
   - Respect Filament `OffscreenRenderer` singleton lifecycle: exactly one process-wide instance (`ThinRenderer.instance()`), strictly created and called on its owning render thread, explicit teardown before interpreter exit.
   - Never assume vendor defaults without checking (`VL53L9_TRANSFORM_LIGHT=0` is required for TNR/FP filters; `enableOptionalEffects=true` is required for gaussian splat opacity).
2. The implementation-grade engineering audit report is located at `docs/engineering-audit-2026-08-18.md`.

### Your Mission
Execute the highest-priority work items from the audit backlog. Choose from the following queued issues:

#### Track A: CrowPanel High-Framerate Pipeline (Issue #197 — Priority: Now)
- Add TurboJPEG/NVJPEG compression to `/ws-thin` (tag 2 binary frame with 8-byte header: `tag=2`, `u16 width`, `u16 height`, `u32 jpeg_len`).
- Replace 460 KB uncompressed RGB565 stream with ~20 KB JPEG payload (drops bandwidth from 110 Mbps to 5.4 Mbps @ 30 fps).
- Implement client dynamic rate and resolution negotiation via `thin_hello` JSON message (boost to 60 fps during touchscreen orbit gestures).
- Add unit and integration tests to `host/tests/test_thin_render.py`.

#### Track B: GitHub Actions Continuous Integration (Issue #195 — Priority: Now)
- Create `.github/workflows/ci.yml` matrix job for:
  - Python test suite (`pytest host/tests`) with virtualenv dependencies.
  - Native C-shim compilation (`host/transform/rs_transform_shim.c` -> `libroomscan_transform.so`).
  - STM32 firmware cross-compilation with `arm-none-eabi-gcc` in `firmware/scanner-stream/`.
- Ensure PRs and pushes to main automatically run the regression suite.

#### Track C: web.py Monolith Decomposition (Issue #196 — Priority: Next)
- Refactor 7,500-line `host/src/roomscan/web.py` into a clean subpackage `host/src/roomscan/server/`:
  - `server/app.py`: FastAPI initialization, lifespan, CORS, and top-level mounting.
  - `server/state.py`: Global server state dataclasses and generation barriers.
  - `server/routes_api.py`: REST endpoints (`/api/record`, `/api/profile`, `/api/captures`).
  - `server/routes_ws.py`: Primary `/ws` point cloud and telemetry streaming.
  - `server/routes_thin.py`: `/ws-thin` CrowPanel render loop and telemetry.
- Maintain 100% backward compatibility with all 572 existing web tests.

#### Track D: Non-Blocking USB CDC Flow Control (Issue #198 — Priority: Next)
- In `firmware/scanner-stream/Src/vl53l9_app.c`, remove the synchronous 100 ms blocking delay in `rs_cdc_send()`.
- Add `tud_cdc_connected()` and `tud_cdc_write_available()` checks to drop frames immediately if the host VCOM reader is detached, preventing sensor ranging stalls.

### Verification Commands
- Run test suite: `host/.venv/bin/pytest host/tests`
- Run web & thin tests: `host/.venv/bin/pytest host/tests/test_web.py host/tests/test_thin_render.py`
- Test C-shim: `cd host/transform && mkdir -p build && cd build && cmake .. && make`
- Test firmware build: `cd firmware/scanner-stream && make -j$(nproc)`
```

---

## Part 2: Active Issue Ledger & State

| Issue ID | Title | Labels | Status | Key Focus Area |
| :--- | :--- | :--- | :--- | :--- |
| **[#195](https://github.com/hellosamblack/lidar-roomscanner/issues/195)** | Add automated GitHub Actions workflow for pytest, C-shim and firmware | `area/environment`, `priority/now` | `READY` | CI pipeline definition |
| **[#196](https://github.com/hellosamblack/lidar-roomscanner/issues/196)** | Refactor web.py monolith into modular server subpackages | `area/host-web`, `priority/next` | `READY` | Maintainability & modularity |
| **[#197](https://github.com/hellosamblack/lidar-roomscanner/issues/197)** | Add TurboJPEG compression and dynamic framerate to /ws-thin for CrowPanel thin client | `area/host-web`, `priority/now` | `READY` | Framerate & bandwidth (30-60 fps) |
| **[#198](https://github.com/hellosamblack/lidar-roomscanner/issues/198)** | Implement non-blocking USB CDC flow control in rs_cdc_send | `area/firmware-scanner-stream`, `priority/next` | `READY` | Prevent MCU ranging stalls |
| **[#199](https://github.com/hellosamblack/lidar-roomscanner/issues/199)** | Update ws-thin telemetry and IR grid for CrowPanel Communicator Prop sidebar | `area/host-web`, `priority/now` | `IN-PROGRESS` / `VERIFIED` | Spatial attitude + 8x8 IR grid |

---

## Part 3: Architecture & Invariant Reminders

1. **Thin-Client Protocol (`/ws-thin`):**
   * Binary frames must be unfragmented (`FIN=1`).
   * Stale frame dropping on socket backpressure is required (`_engaged_clients` / drop on congested buffer).
   * Telemetry rides `thin_telemetry` at 2 Hz containing `heading_deg`, `pitch_deg`, `roll_deg`, `yaw_rate_dps`, and `ir_grid` (64 integers $0..255$).
2. **Open3D / Filament Threading:**
   * Open3D's Filament `OffscreenRenderer` **aborts the process** (`utils::PreconditionPanic`) if instantiated twice or called across different thread IDs. Always route rendering through `ThinRenderer.instance()`.
3. **Firmware Constraints:**
   * Cortex-M33 (STM32H563ZI) with 2 MB Flash and 640 KB SRAM.
   * Never execute synchronous sleeps or waits $>1\text{ ms}$ inside the core sensor loop.
