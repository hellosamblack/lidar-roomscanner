# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`roomscanner/` is the **active development workspace** for a tethered handheld **3D room scanner**. The end goal: an STM32H563ZI board streams timestamped ToF (+ later IMU/env) frames to a PC that runs real-time SLAM (Open3D Tensor G-ICP + TSDF), with an offline pass fusing 4K phone video into a ToF-seeded 3D Gaussian Splat.

New work — the PC-side visualizer, the binary frame protocol, and any new firmware — happens **here**. The existing STM32 firmware lives in a separate **reference package** (`../53L9A1/`) that we treat as **read-only reference**, not something we edit in place.

## Repository layout

```
F:\git\personal\lidar\
├─ roomscanner\            ← YOU ARE HERE (active dev)
│  ├─ CLAUDE.md            ← this file
│  ├─ ROADMAP.md           ← phased plan (source of truth for sequencing; per-phase risks + reference-firmware bug list)
│  ├─ .claude\skills\      ← project skills: firmware-loop (build/flash/monitor), protocol-change (wire-change checklist)
│  ├─ docs\
│  │  ├─ engineering-practices.md            ← binding conventions (repo rules, protocol rules, firmware/host standards)
│  │  ├─ protocol.md                         ← wire protocol spec (created by Phase 1 Task 1)
│  │  └─ superpowers\plans\                  ← implementation plans (Phase 1 plan lives here)
│  ├─ firmware\            ← our firmware forks (scanner-stream; created by Phase 1 Task 6)
│  ├─ host\                ← PC Python package `roomscan` (created by Phase 1 Task 1)
│  └─ references\
│     ├─ roadmapResearch.md                  ← architecture design + critical review
│     └─ 3D Mapping Architecture Evaluation.md
└─ 53L9A1\                 ← ST reference package (READ-ONLY reference)
   ├─ Drivers\  Middlewares\ST\  Utilities\vl53l9-common\
   └─ Projects\NUCLEO-H563ZI\Applications\53L9A1\53L9A1_PostprocessSingle\  ← the firmware app
```

Follow `docs/engineering-practices.md` for all work here. Known bugs in the reference firmware (do not
inherit them into forks) are catalogued in `ROADMAP.md` → "Reference-firmware bugs". Note the `53L9A1/`
package ships **no USB middleware** (`Middlewares/ST/` = media-object + vl53l9-transform-c only) — USB CDC
work vendors TinyUSB (see the Phase 1 plan).

**Self-improvement rule (owner, 2026-07-08):** after every milestone (phase completion / major merge),
run the `milestone-retro` skill BEFORE starting the next phase — convert the push's friction into
skills (with references/scripts), shared tools under `host/tools/`, and doc fixes. A milestone isn't
done until the next one got easier.

Throughout this doc, **`<APP>`** = `../53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/` (the reference firmware app dir). File references like `Src/vl53l9_app.c` are relative to `<APP>`.

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
- **Transport: Ethernet, not USB.** The board has a 10/100 MAC (RMII pins already `AF11_ETH` in `<APP>/Src/main.c`; MAC + lwIP not yet enabled). Target link is lwIP/UDP with hardware PTP timestamping. USB CDC (native `USB_DRD_FS`) is a bring-up/fallback only. This removes the USB-bandwidth and timestamp-drift bottlenecks the doc spends most of its length on.
- **Sensors: X-NUCLEO-IKS4A1** adds an IMU (LSM6DSV16X, hardware SFLP orientation), magnetometer, barometer (Z-drift constraint), and temp/humidity. Not yet in code. Bus-sharing is **resolved**: the IKS4A1 rides the ToF's I3C1 bus as legacy-I2C targets (shared PB8/PB9) — no separate peripheral. Stacking/config recipe + bench checklist in `docs/iks4a1-stacking.md`.

### Roadmap

Full detail in `ROADMAP.md`. Summary:

- **Phase 0 — ✅ done.** On-device transform + ASCII depth map over ST-Link VCOM (`CONF_PRINT_FRAME = 1` in `<APP>/Src/vl53l9_app.c:31`).
- **Phase 1 — real-time 3D visualizer.** Replace ASCII with a **versioned binary frame protocol** (magic + seq + timestamp + payload + CRC32) over a link fast enough for real-time (VCOM @115200 ≈ 1 fps for a 9 KB frame — inadequate; use **native USB CDC FS** now, Ethernet later). PC app deprojects depth → point cloud and renders live (Python + Open3D, or a custom viewer). First capture what the transform library exposes: the app prints available **streams and controls** at startup via `streams_inspect`/`controls_inspect` — that list (depth/reflectance/confidence/possibly XYZ) decides what can be streamed and whether deprojection happens on-MCU or PC-side.
- **Phase 2 — IR + additional sensor streams.** Extend protocol + PC UI for IR reflectance, confidence, ambient; colorize the cloud by IR. Multi-stream from the start.
- **Phase 3 — UI & runtime configuration.** Host→device control channel to set usecase/binning/streams at runtime (transform lib exposes `controls`); recording/playback; config persistence.
- **Phase 4 — integrate X-NUCLEO-IKS4A1** (swapped ahead of Ethernet 2026-07-09): IMU/mag/baro drivers, fuse into the payload with hardware timestamps. Edge-AI (in-sensor MLC/ISPU) belongs here, not on the M33 — see the edge-ai-tooling memory.
- **Phase 5 — transport upgrade to Ethernet** (lwIP/UDP + PTP + zero-config direct link). Older docs may use the pre-swap numbering (Ethernet=4, IKS4A1=5).
- **Phase 6 — real-time SLAM** on PC: SFLP rotation prior, 3-DoF G-ICP, scalable TSDF, IR-as-intensity, baro Z-constraint.
- **Phase 7 — offline**: COLMAP pose priors + depth-regularized 3D Gaussian Splatting.

Guiding order (per project owner): mature the visualizer and UI/config on the ToF sensor alone **before** adding the IKS4A1 board.
