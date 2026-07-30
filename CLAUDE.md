# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`roomscanner/` is the **active development workspace** for a tethered handheld **3D room scanner**. The end goal: an STM32H563ZI board streams timestamped ToF (+ later IMU/env) frames to a PC that runs real-time SLAM (Open3D tensor ICP + TSDF), with an offline pass fusing 4K phone video into a ToF-seeded 3D Gaussian Splat.

New work — the PC-side visualizer, the binary frame protocol, and any new firmware — happens **here**. The existing STM32 firmware lives in a **reference package**, vendored in-repo at `firmware/vendor/53L9A1/`, that we treat as **read-only reference**, not something we edit in place.

## Repository layout

```
F:\git\personal\lidar\
├─ roomscanner\            ← YOU ARE HERE (active dev)
│  ├─ CLAUDE.md            ← this file
│  ├─ ROADMAP.md           ← phased plan (source of truth for sequencing; per-phase risks + reference-firmware bug list)
│  ├─ BUGS.md              ← bug tracker for OUR code (host + scanner-stream firmware); file new bugs here
│  ├─ .claude\skills\      ← project skills: firmware-loop (build/flash/monitor), protocol-change (wire-change checklist), status-sync (MANDATORY at ship time — docs move with the code), stack-electrical (jumpers/SBs/bus routing across the board stack)
│  ├─ docs\
│  │  ├─ engineering-practices.md            ← binding conventions (repo rules, protocol rules, firmware/host standards)
│  │  ├─ protocol.md                         ← wire protocol spec (created by Phase 1 Task 1)
│  │  ├─ headless-host-setup.md              ← 5-min bring-up for a GPU-less Linux host (web viewer); run host/tools/headless_doctor.py
│  │  ├─ web-ui-testing.md                   ← how to SEE + drive the web UI on this headless box (host/tools/web_ui_shot.py, CDP screenshots)
│  │  ├─ web-protocol.md                      ← the roomscan-web `/ws` app protocol (binary tags + JSON messages, in/out, Phases 1–3); hook new web messages here
│  │  └─ superpowers\plans\                  ← implementation plans (Phase 1 plan lives here)
│  ├─ firmware\            ← our firmware forks (scanner-stream; created by Phase 1 Task 6) + vendored deps
│  │  └─ vendor\
│  │     ├─ tinyusb\  lwip\                   ← vendored USB CDC + TCP/IP stacks
│  │     └─ 53L9A1\                           ← ST reference package (READ-ONLY reference), vendored in-repo
│  │        ├─ Drivers\  Middlewares\ST\  Utilities\vl53l9-common\
│  │        └─ Projects\NUCLEO-H563ZI\Applications\53L9A1\53L9A1_PostprocessSingle\  ← the firmware app
│  ├─ host\                ← PC Python package `roomscan` (created by Phase 1 Task 1)
│  └─ references\
│     ├─ roadmapResearch.md                  ← architecture design + critical review
│     └─ 3D Mapping Architecture Evaluation.md
```

Follow `docs/engineering-practices.md` for all work here. Known bugs in the reference firmware (do not
inherit them into forks) are catalogued in `ROADMAP.md` → "Reference-firmware bugs". Note the vendored `53L9A1/`
package ships **no USB middleware** (`Middlewares/ST/` = media-object + vl53l9-transform-c only) — USB CDC
work vendors TinyUSB (see the Phase 1 plan).

**Self-improvement rule (owner, 2026-07-08):** after every milestone (phase completion / major merge),
run the `milestone-retro` skill BEFORE starting the next phase — convert the push's friction into
skills (with references/scripts), shared tools under `host/tools/`, and doc fixes. A milestone isn't
done until the next one got easier.

**Agentic firmware loop (owner, 2026-07-10):** this is an agentic project — **Claude reads/writes firmware
and drives the full build → flash → observe → diagnose loop itself**, it does not write up "bench steps"
for a human to run. The toolchain, `STM32_Programmer_CLI`, `capture.py` (native CDC), ST-Link VCOM, and
on-target SWD register reads (`-r32 <addr>`, addresses from the `.map`) are all Claude's to use — see the
`firmware-loop` skill and `docs/engineering-practices.md` → Firmware. The human is asked **only** for
physical actions Claude cannot perform: moving IKS4A1/53L9A1 jumpers & solder bridges, scope probing, and
power-cycling (USB replug) to clear a warm-wedged I3C bus. Diagnose in firmware first; escalate to the
human only for a genuinely physical cause, and name the exact physical action.

**New tools land in the MCP server (2026-07-29):** the agent-facing surface is `roomscan-mcp`
(`host/src/roomscan/mcp_server/`, registered in `.mcp.json`) — typed tools returning structured
data, documented in `docs/mcp-server.md`. **Any new agent-facing capability lands as an MCP tool,
not only as a script under `host/tools/`.** Write the logic as a pure function returning a dict,
register a thin wrapper (its **docstring is the description the agent sees**), and add a CLI front
end only if a human will run it directly — one implementation, two front ends. If a script is
deliberately *not* exposed, record it in `EXCLUDED` in `host/tests/test_mcp_registry.py` with the
reason; that test fails on any script which is neither exposed nor excluded. Two invariants: the
server never binds the device stream (`roomscan-web` owns it — recording goes through
`rig_record()`), and every tool reports what actually happened rather than what was requested.

Throughout this doc, **`<APP>`** = `firmware/vendor/53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/` (the reference firmware app dir). File references like `Src/vl53l9_app.c` are relative to `<APP>`.

## The reference firmware (`<APP>`)

Bare-metal firmware for the **STM32H563ZI** (NUCLEO-H563ZI + X-NUCLEO-53L9A1 expansion) driving a single **VL53L9CX ToF 3D LiDAR**. It captures raw frames over **I3C + DMA**, runs them through the `vl53l9-transform-c` pipeline with per-device calibration, and produces a processed depth frame (float32 `ZF32`). Frame rate + an optional ASCII depth map print to the VCOM serial port (115200 8N1). Its dependencies (`Drivers/`, `Middlewares/ST/`, `Utilities/vl53l9-common/`) sit five levels up from `<APP>`, at the `53L9A1/` package root.

### Build (run from `<APP>`)

Toolchain: **arm-none-eabi-gcc** (on `PATH`), CMake ≥ 3.22, **Ninja**. Target: Cortex-M33, `fpv5-sp-d16` hard float; app code compiled `-Ofast`.

```sh
cmake --preset Debug      # or Release; configures into build/Debug
cmake --build build/Debug # produces .elf, then .bin, and prints size
```

Presets in `<APP>/CMakePresets.json`. Post-build emits `53L9A1_PostprocessSingle.bin` and runs `arm-none-eabi-size`. No unit tests — validation is on-target: flash and read VCOM. In STM32CubeIDE / VS Code, builds go through ST's `cube-cmake`/`cube` wrappers (`.vscode/settings.json`); on a plain shell use the bare `cmake`/`ninja`.

### Firmware architecture — three layers

1. **CubeMX platform (`Src/main.c`, `Src/stm32h5xx_*.c`, `cmake/stm32cubemx/`)** — generated HAL/LL init for clocks, GPIO, GPDMA1, I3C1, TIM3, USB, ICACHE. `main()` inits peripherals + COM1, then calls `vl53l9_app()` in a loop. **Do not hand-edit generated init outside the `/* USER CODE BEGIN/END */` guards** — it regenerates from `53L9A1_PostprocessSingle.ioc`. (Moot while we treat this package as read-only, but relevant if we ever regenerate.)

2. **Platform abstraction (`Utilities/vl53l9-common/`, shared)** — `vl53l9_interface.h` defines the `platform_*` API (power/reset, dynamic I3C address assignment, an **event system**: `platform_wait_for_event` / `_acknowledge_event` over `PLATFORM_GPIO_IT_EVT`, `PLATFORM_I3C_DMA_RX_EVT`, etc., plus a timestamp profiler) and the `vl53l9_device_t` descriptor. `platform_utils.c` implements it on the STM32 HAL. `vl53l9/vl53l9_device.c` holds the device table (`device[]`, indexed by `CONF_DEVICE_ID`); `vl53l9_utils.c` provides ranging profiles (`g_ranging_profiles[]`, keyed by `VL53L9_USECASE_*`) and resolution/binning helpers.

3. **Application (`Src/vl53l9_app.c`)** — the only genuinely app-specific file. Compile-time knobs: `CONF_DEVICE_ID`, `CONF_PRINT_FRAME`, `CONF_USECASE`. Wires the transform pipeline and runs the acquisition loop.

**The acquisition loop.** Setup (each step gated by a return code → `handle_error()`): reset sensor → assign I3C dynamic address → `vl53l9_init` → read `calib_data` → apply profile; `transform_initialize` → **set capabilities** (input `raw`/`3DMD` stream, then output `depth`/`ZF32` — order matters, input before output, no defaults); set the mandatory `calib-buffer` control → `transform_prepare`. Steady state uses **double-buffered raw input + DMA**: while the sensor DMA-transfers frame N into one buffer, the pipeline processes frame N-1 from the other (`raw_mem_index` toggles; pipeline pointed at the *previous* buffer via `in_raw_mems.items`). Per iteration: `vl53l9_trigger_frame` → wait `PLATFORM_GPIO_IT_EVT` → `vl53l9_get_frame_async` (kick DMA) → process previous frame → wait `PLATFORM_I3C_DMA_RX_EVT` → ack → parse metadata → print. First iteration skips processing. Binning drives sizes: binning 2 → raw width 14842, binning 4 → 3844 (height 1); output resolution from `vl53l9_utils_get_resolution`. Other binning unsupported.

**Gotchas.** Errors are non-zero `int` return codes funneled to `handle_error()`, which reads sensor status and **spins forever** (no recovery). HAL failures hit `Error_Handler()` in `main.c` (disables IRQs, spins). The transform pipeline uses an opaque handle + hand-built `properties_t`/`capabilities_t`/`stream_buffer_t`; frees are commented out (loop never exits). Linker scripts `STM32H563xx_FLASH.ld` (default) / `STM32H563xx_RAM.ld`; startup `startup_stm32h563xx.s`. `roomscanner/` is a git repository (branch `main`); `53L9A1/` is not.

## Target architecture (where this is going)

Two decisions that override the older parts of `references/roadmapResearch.md`:
- **Transport: native USB CDC OR Ethernet UDP (Phase 5).** The device streams over either USB CDC or Ethernet (UDP unicast). If Ethernet is plugged in, the device acts as a DHCP client (or falls back to a self-assigned IP server) and streams via UDP. This removes the USB cable length limit and prepares the plumbing for Phase 6's hardware time-sync (PTP). USB CDC remains supported as an automatic fallback.
- **Sensors: X-NUCLEO-IKS4A1** — **integrated (Phase 4, 2026-07-10)**: the LSM6DSV16X shares I3C1 with the ToF as a native I3C target (HUB1-only jumpering, PartID-keyed multi-device ENTDAA, slow-PP workaround for the NXS0108 translator); SFLP orientation quaternion = stream 9, sensor-hub env (baro/mag/temp) = stream 10, both one sample per ToF frame; host panel shows gizmo/compass/sparklines and runs 9-axis mag yaw fusion (`docs/yaw-fusion.md`). Full stack streams at 27.85 fps, 0 CRC. Stacking recipe + bus-conflict resolution history in `docs/iks4a1-stacking.md`. On-rig mag calibration + `AXIS_CONVENTION` check completed 2026-07-10 (BUG-004: `mag_cal.json`, `AXIS_CONVENTION = diag(1,-1,-1)`). *(2026-07-28 orientation-noise pass, BUG-027: the SFLP quat was being decimated 480 Hz → 30 Hz by keeping one sample of ~16 — unfiltered, so the whole noise band aliased in. Firmware now averages the FIFO batch, enables the gyro LPF1 at 28.4 Hz, and sets LIS2MDL `CFG_REG_B = OFF_CANC|LPF`: **2.8× less orientation noise**, 0.0329 → 0.0118 deg/frame, streams 7/9/10 at 30.3 Hz / 0 drops / 0 gaps. The floor is now the SFLP FIFO's **fp16** encoding (~0.056°/step), not the sensor — see `docs/iks4a1-stacking.md` → "Orientation-noise pass".)* ~~Still open: SHT40 humidity unstreamed; beating the fp16 floor (batch raw XL/GY, fuse host-side).~~
*(2026-07-29: the fp16-floor item is **superseded** — raw XL/GY now ship as **stream 11** and a host
complementary filter `roomscan.imufusion` exists but is **gated off**; the floor also turned out to be
**dither**-limited, not step-limited, so a quieter board measures worse. More importantly the visible
noise was the **eCompass**, traced to a direction-dependent magnetometer calibration — ~~**BUG-030**,
the top open item~~ **BUG-030, closed 2026-07-30**: the owner re-fit hand-held off the tripod and it
validated on an independent room sweep (attitude-locked error 0.56%, tilt ramp 1.042×, `YawFusion`
`gated:anomaly` 58.6% → 0%, `active` 6.2% → 64.8%). Two traps learned there, now encoded in
`host/tools/mag_check.py` / the `capture_magcheck` MCP tool: **the live calibration is `./mag_cal.json`
at the repo root** (`mag_cal_path` is cwd-relative — a stale `host/` copy shadowed it for two weeks and
is now deleted), and **raw |B| spread is not calibration error on a moving capture** — indoor ambient
field varies ~±6% with position (BUG-034), so detrend before judging a fit. Full state + resume
instructions: `docs/superpowers/plans/2026-07-29-orientation-resume.md`.)* Still open: SHT40 humidity
unstreamed; **heading *direction*** remains unvalidated — |B| flatness cannot see DT0103's rotation
ambiguity, so it needs a braced fixed-heading tilt sweep (resume doc §4.6).

### Roadmap

Full detail in `ROADMAP.md`. Summary:

- **Phase 0 — ✅ done.** On-device transform + ASCII depth map over ST-Link VCOM (`CONF_PRINT_FRAME = 1` in `<APP>/Src/vl53l9_app.c:31`).
- **Phase 1 — ✅ done. Real-time 3D visualizer**: versioned binary frame protocol (magic + seq + timestamp + payload + CRC32) over native USB CDC FS (TinyUSB, VID:PID `CAFE:4001`); PC package `roomscan` decodes, deprojects, and renders live (Open3D).
- **Phase 2 (+2.5) — ✅ done. Raw streaming + PC-side transform**: the MCU streams raw `3DMD` + CALIB; the `vl53l9-transform-c` pipeline runs on the PC (equivalence-gated), giving depth/IR/confidence/ambient host-side; trigger-early overlap → ~27.8 fps. *(Data-quality follow-up 2026-07-16: reflectance carries ~18% sensor-locked fixed-pattern noise (per-zone FPN); optional host **flat-field correction** built + shipped-disabled — `roomscan.flatfield` applied in `TransformStage`, gated by `[viewer] flatfield_path`, needs an on-rig panned-wall capture to enable. See `docs/flatfield-calibration.md`.)*
- **Phase 3 (+3.5) — ✅ done. UI & runtime configuration**: COMMAND/ACK control channel (usecase/exposure/reinit, + `SET_STANDBY` cmd 7 for laser-wear auto-idle, 2026-07-21), EVENT frames + bounded recovery, recording/playback, config persistence, the `roomscan-panel` GUI (IR monitor, device controls, capture, events), and the `roomscan-web` FastAPI/Three.js server for headless remote rendering.
- **Web replacement of `panel.py`** — a 5-phase program (Three.js web app supplants the Open3D desktop panel), **now complete: `roomscan-web` is the primary, supported UI and `panel.py` is deprecated legacy** (kept for a local-display box only; Web Phase 5, 2026-07-16). *(Was "6-phase": the old "Showcase" phase was a misnomer for SLAM mapping — the record→build→save flow — already delivered by Web Phases 3–4; owner clarification 2026-07-16.)* **Web Phase 1 (Core Real-Time Web Instrument) — ✅ done (2026-07-16)**: single-broadcast-task fix (kills the two-tab frame-stealing bug), multiplexed `/ws` protocol (tagged binary POINT_CLOUD/IR_IMAGE + metrics/event/log/cmd/state JSON), 7 vanilla ES modules, working command feedback + runtime color modes + IR monitor + metrics HUD; host-side only, verified in headless Chrome. **Web Phase 2 (Sensors) — ✅ done (2026-07-16)**: streams 9/10 fed through the shared reader (reuses the desktop `SensorState`/`YawFusion`/`MagCalibration`), new `sensor` JSON message on `/ws` (server-computed gizmo rotation + drift-free heading + pressure/temp history), new 2D-canvas `sensors.js` (gizmo/compass/sparklines in the left rail), IMU/Env rows in the metrics HUD; 610 tests green, headless-Chrome verified. **Web Phase 3 (Recording & Playback) — ✅ done (2026-07-16)**: full-remote record + capture library + runtime source-swap (new `SessionController` stops/respawns `panel._run_reader` against a new source; live device kept behind a `_NoCloseSource` proxy so Go Live is instant, no UDP re-probe) + transport (pause/speed ×0.5–Max/loop/seekable progress; seek re-injects the governing CALIB from a CRC-verified capture index); two new `/ws` JSON messages (`session`/`captures`), new `capture.js` (8th ES module), additive `FileSource(start=)`; 625 tests green, headless-Chrome verified. *(2026-07-29: post-recording naming — a skippable "Name Recording" modal pops on every stopped take, prefilled with the auto `web_<ts>.bin` name; `rename_capture` inbound + `session.recording.last_name` echo, no dedicated ack.)* *(2026-07-29: the only way back to live view was clicking the "● Live device" row buried in the Source list — owner hit that dead end mid-playback — so a dedicated "● Go Live" button now sits in the transport panel, sending the same existing `go_live` message.)* **Web Phase 4 (SLAM mode) — ✅ done (2026-07-16)**: top-bar Real-Time↔SLAM switch; a new `SlamRunner` in `web.py` reuses the desktop SLAM pipeline **unchanged** (`make_slam_worker` on **local CUDA:0** — the Proxmox host now passes an RTX 2000 Ada through, ~7 ms/frame — + `MeshPrep`), fed from the broadcaster only in SLAM mode; new binary **MESH (tag 3)** + `slam`/`saved` JSON + inbound `set_mode`/`slam_opt`/`save`; new `slam.js` (9th module) renders mesh+trajectory+follow-camera into `scene.js`'s single Three.js context; **Save** writes full-res `results/web_<ts>.ply`/`.tum` (downloadable); 637 tests green, GPU-verified + headless-Chrome-driven against `captures/verify_slam.bin`. **Web Phase 5 (settings persistence + retire `panel.py`) — ✅ done (2026-07-16)**: the web UI's display prefs (color/IR colormap+freeze/SLAM trajectory·walls·follow) now persist to the **shared `roomscan.toml` [viewer]` table** — `web.ui_from_config` seeds `UiState` on boot, `web._persist_ui` writes each change back (reloading first so desktop-only fields survive); `mode` is deliberately not restored (SLAM arms lazily → a restart always comes up real-time). Consequence: a fresh web install now adopts the shared `color` default (`reflectance`), not the old web-only `depth`. **`panel.py` deprecated in place** — the GUI-free reader plumbing (`_run_reader`/`_Pacer`/`follow_camera_target` + follow constants) moved to a neutral `reader.py` that both `web.py` and `panel.py` import, so the web server no longer depends on the panel module; `roomscan-panel`/`roomscan-view --panel` print a deprecation notice. 645 tests green; verified end-to-end by driving a real `/ws` `set_color` and confirming it survived a full server restart into a fresh client's first `state` message. The `/ws` app protocol (unchanged by Phase 5 — no new messages) is indexed in `docs/web-protocol.md`. *(2026-07-30 — **real-time view modes**, owner ask: World / FPV / Mirror, a segmented control in the View card. The frame swap is **server-side** (`web.view_rotation`): FPV composes `boresight_view_frame(R) @ R`, which nets out to a pure roll about the boresight — the sensor's aim, gravity-levelled, agreeing with the IR pane's own roll — and Mirror negates X on top (the IR pane flips with a client-side `scaleX(-1)`; it is otherwise untouched by view mode). Client-side it is a static locked pose, because the cloud is rotated by the smoothed quat at broadcast rate while `sensor.rot` is raw and half as fast. **Camera framing is per-mode, tunable and persisted**, all three expressed as `distance`/`height`/`rotation` offsets from **one baseline — the FPV ground truth**, a camera at the sensor looking down its boresight (all-zero = exactly that camera). A non-zero offset is mandatory, not cosmetic: a camera at the optical centre reproduces the depth image's own projection and renders flat. Also here: the scene grid was a vertical wall (a pre-gravity-alignment hangover) and is now an earth plane, and **BUG-033** — the live cloud was frustum-culled against a stale zero-radius bounding sphere, latent in World and fatal in FPV.)* Specs: `docs/superpowers/specs/2026-07-15-web-phase1-core-instrument-design.md`, `.../2026-07-16-web-phase2-sensors-design.md`, `.../2026-07-16-web-phase3-recording-playback-design.md`, `.../2026-07-16-web-phase4-slam-design.md`; details in `ROADMAP.md` → "Web replacement of `panel.py`".
- **Phase 4 — ✅ done. X-NUCLEO-IKS4A1 integrated** (2026-07-10): streams 9 (SFLP quat) + 10 (env via LSM sensor hub), panel sensors group, host yaw fusion — see the architecture bullet above for what's still open. Edge-AI (in-sensor MLC/ISPU) belongs at this tier, not on the M33 — see the edge-ai-tooling memory.
- **Phase 5 — ✅ Complete: transport upgrade to Ethernet** (lwIP/UDP + zero-config direct link). The device successfully streams raw frames over Ethernet. PTP support remains an optional future addition if required by SLAM. *(2026-07-21: the host→device COMMAND channel now works over Ethernet too — it was CDC-only, the ETH transport discarded inbound datagrams; inbound UDP now feeds the same command parser as CDC.)* *(2026-07-28: **the board now runs untethered** — the NUCLEO-H563ZI has no HSE crystal, so the system clock was the ST-LINK's MCO and the firmware wedged before `ETH_Init()` whenever CN1 was unplugged. PLL1 now sources from HSI unconditionally, same 250 MHz SYSCLK; power from USB_USER with JP2 at 9-10. Boot-progress LEDs added — LD1 green = clocks up, LD2 yellow blinking = acquisition loop alive, LD3 red = wedged. BUG-023/024/025.)*
- **Phase 6 — in progress. Real-time SLAM** on PC: SFLP rotation prior, 3-DoF constrained point-to-plane ICP frame-to-model vs. TSDF raycast (VoxelBlockGrid), IR-as-intensity, baro Z-constraint. Note (2026-07-10): Open3D has **no tensor G-ICP** — point-to-plane is primary, `small_gicp` is the GICP fallback; KISS-ICP kept as offline odometry benchmark; FAST-LIO2/Point-LIO/CT-ICP/PIN-SLAM/SHINE rejected (scanning-LiDAR assumptions vs. our 54×42 depth imager) — details in `ROADMAP.md` Phase 6. **Core pipeline shipped**: `roomscan.slam` + `roomscan-slam` CLI (offline-validated, `docs/phase6-slam-validation.md`); the primary live SLAM surface is the web app's SLAM mode on local CUDA:0 (~7 ms/frame). **6.G GPU-memory hardening — ✅ done (2026-07-29, BUG-032)**: the long-scan OOM was measured, and the stated cause was wrong — the per-frame integrate/raycast/ICP path is **byte-flat** over 4000 frames / 80 m, while the *throttled `mesh()` extraction* grew device memory **5.13 MiB/frame** (523 → 5483 MiB in 1500 frames) via `_extract_vbg()`'s whole-grid `.cpu()` copy leaving ever-larger temporaries in Open3D's CUDA cache. Fix: `TsdfMap.release_cache_every` (default 1, per *extraction* not per frame, no-op on CPU), plumbed through `Mapper` / `[slam]` / CLI / `web.SlamRunner`. After: peak 651 MiB over 80 m with identical step latency. New shared scaffolding `roomscan.slam.gpumem` (ctypes NVML) + `roomscan.slam.synthscene` (deterministic analytic walk), the rig `host/tools/slam_gpu_memory.py`, and a memory-ceiling guard in `cuda_smoke.py`. *(Validated 2026-07-30 on the owner's first real full room sweep, `captures/roomSweepFull20260730.bin` — unfixed grows **10.34 MiB/frame** there, 2× the synthetic estimate, and would need ~36 GB to finish.)* **Lifting that ceiling exposed the next one — BUG-035**: Open3D's `VoxelBlockGrid` pre-allocates `block_count` and **does not grow**, and the old hard-coded 40,000 sat *below* one real scan's demand (42,917). It saturated mid-scan and, because SLAM is frame-to-model, tracking collapsed 30 frames later — 560 lost frames, 18% of the sweep, **silently**. `DEFAULT_BLOCK_COUNT` is now 160,000, `block_count` is plumbed through `Mapper` / `[slam]` / CLI / `SlamRunner` / the rig, and `TsdfMap` warns once at 90% capacity. **A bigger room or finer `voxel_size` may still need it raised** — and note the grid is device-homogeneous (no managed memory), so maps larger than VRAM want `[slam] device = "CPU:0"`, which ran that same sweep with 0 lost frames in system RAM. **Open sub-phase**: **6.D drift correction / loop-closure evaluation** — its "measure first" gate needs an owner-recorded closed-loop walk (no capture has one; `recordings/2026-07-08-room-scan.bin` predates stream 9). See `ROADMAP.md` Phase 6. *(The 2026-07-14 panel UX redesign shipped, but the desktop panel is now deprecated legacy — its remaining on-rig eyeball items are non-blockers.)* **Orientation accuracy for handheld use (2026-07-29):** shipped stream 11 (480 Hz raw IMU FIFO), stream 12 (`INTERNAL_FREQ_FINE` — clock scale error cut 29790 → 3345 ppm), a TIM2 µs frame clock stamped at FRAME_READY, sensor-hub averaging, four UI orientation decomposition modes + jitter statistics + a magnetometer-calibration modal whose hero is a **body-fixed 3D "Shell & Steering" view** (92-cell shell, comet trail, B/g arrows + dip arc, ghost-target steering widget, binary `MAGPOSE` tag 5 at 30 Hz, 2D Lambert-disc fallback). *(2026-07-30, owner: the hero's camera is now **first-person** — parked behind the device on the boresight, `camera.up` tracking −g like the live view's FPV mode, so screen-down is room-down and the only camera motion is a gravity roll. The shell stays body-fixed; the cells **behind the camera** are drawn translucent (a four-mesh material split, since `InstancedMesh` has no per-instance alpha) because from that viewpoint the near cap would otherwise hide the hemisphere you are aiming into. Presentation only, no `/ws` change; §4.1 of the design spec is amended in place.)* *(Sensors card decluttered 2026-07-29, BUG-033: those readouts had grown to ~1600 px of flat, half-duplicated rows that overran the dock band. The raw full-precision ZYX numbers, the jitter table and the yaw-offset controls now live in a **collapsed `#sensor-diag` `<details>`** — open it before screenshotting or asserting them. Presentation only, no `/ws` change.)* Error budget during motion is now dominated by **BUG-030** (magnetometer calibration, up to ~90° heading error — needs an owner tumble), then **BUG-031** (~890 µs ToF↔IMU skew), *then* fp16 quantization. See `docs/superpowers/plans/2026-07-29-orientation-resume.md`.
- **Phase 7 — offline**: COLMAP pose priors + depth-regularized 3D Gaussian Splatting.

Guiding order (per project owner): mature the visualizer and UI/config on the ToF sensor alone **before** adding the IKS4A1 board. *(Satisfied — both are done; Phase 6 SLAM should likewise be validated against recorded captures before hardware-in-the-loop.)*
