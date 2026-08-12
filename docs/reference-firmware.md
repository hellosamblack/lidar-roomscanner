# The reference firmware (`<APP>`)

Moved out of root `AGENTS.md` on 2026-08-11 so it loads on demand rather than in every agent session.
`AGENTS.md` keeps the failure contracts that bite regardless of what you are working on and points here
for the rest.

Throughout, **`<APP>`** = `firmware/vendor/53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/`.
File references like `Src/vl53l9_app.c` are relative to `<APP>`.

Bare-metal firmware for the **STM32H563ZI** (NUCLEO-H563ZI + X-NUCLEO-53L9A1 expansion) driving a single **VL53L9CX ToF 3D LiDAR**. It captures raw frames over **I3C + DMA**, runs them through the `vl53l9-transform-c` pipeline with per-device calibration, and produces a processed depth frame (float32 `ZF32`). Frame rate + an optional ASCII depth map print to the VCOM serial port (115200 8N1). Its dependencies (`Drivers/`, `Middlewares/ST/`, `Utilities/vl53l9-common/`) sit five levels up from `<APP>`, at the `53L9A1/` package root.

## Build (run from `<APP>`)

Toolchain: **arm-none-eabi-gcc** (on `PATH`), CMake ≥ 3.22, **Ninja**. Target: Cortex-M33, `fpv5-sp-d16` hard float; app code compiled `-Ofast`.

```sh
cmake --preset Debug      # or Release; configures into build/Debug
cmake --build build/Debug # produces .elf, then .bin, and prints size
```

Presets in `<APP>/CMakePresets.json`. Post-build emits `53L9A1_PostprocessSingle.bin` and runs `arm-none-eabi-size`. No unit tests — validation is on-target: flash and read VCOM. In STM32CubeIDE / VS Code, builds go through ST's `cube-cmake`/`cube` wrappers (`.vscode/settings.json`); on a plain shell use the bare `cmake`/`ninja`.

The full build → flash → observe → diagnose loop belongs to the `firmware-loop` skill; use it rather than
re-deriving the invocation.

## Firmware architecture — three layers

1. **CubeMX platform (`Src/main.c`, `Src/stm32h5xx_*.c`, `cmake/stm32cubemx/`)** — generated HAL/LL init for clocks, GPIO, GPDMA1, I3C1, TIM3, USB, ICACHE. `main()` inits peripherals + COM1, then calls `vl53l9_app()` in a loop. **Do not hand-edit generated init outside the `/* USER CODE BEGIN/END */` guards** — it regenerates from `53L9A1_PostprocessSingle.ioc`. (Moot while we treat this package as read-only, but relevant if we ever regenerate.)

2. **Platform abstraction (`Utilities/vl53l9-common/`, shared)** — `vl53l9_interface.h` defines the `platform_*` API (power/reset, dynamic I3C address assignment, an **event system**: `platform_wait_for_event` / `_acknowledge_event` over `PLATFORM_GPIO_IT_EVT`, `PLATFORM_I3C_DMA_RX_EVT`, etc., plus a timestamp profiler) and the `vl53l9_device_t` descriptor. `platform_utils.c` implements it on the STM32 HAL. `vl53l9/vl53l9_device.c` holds the device table (`device[]`, indexed by `CONF_DEVICE_ID`); `vl53l9_utils.c` provides ranging profiles (`g_ranging_profiles[]`, keyed by `VL53L9_USECASE_*`) and resolution/binning helpers.

3. **Application (`Src/vl53l9_app.c`)** — the only genuinely app-specific file. Compile-time knobs: `CONF_DEVICE_ID`, `CONF_PRINT_FRAME`, `CONF_USECASE`. Wires the transform pipeline and runs the acquisition loop.

## The acquisition loop

Setup (each step gated by a return code → `handle_error()`): reset sensor → assign I3C dynamic address → `vl53l9_init` → read `calib_data` → apply profile; `transform_initialize` → **set capabilities** (input `raw`/`3DMD` stream, then output `depth`/`ZF32` — order matters, input before output, no defaults); set the mandatory `calib-buffer` control → `transform_prepare`. Steady state uses **double-buffered raw input + DMA**: while the sensor DMA-transfers frame N into one buffer, the pipeline processes frame N-1 from the other (`raw_mem_index` toggles; pipeline pointed at the *previous* buffer via `in_raw_mems.items`). Per iteration: `vl53l9_trigger_frame` → wait `PLATFORM_GPIO_IT_EVT` → `vl53l9_get_frame_async` (kick DMA) → process previous frame → wait `PLATFORM_I3C_DMA_RX_EVT` → ack → parse metadata → print. First iteration skips processing. Binning drives sizes: binning 2 → raw width 14842, binning 4 → 3844 (height 1); output resolution from `vl53l9_utils_get_resolution`. Other binning unsupported.

## Gotchas

Errors are non-zero `int` return codes funneled to `handle_error()`, which reads sensor status and **spins forever** (no recovery). HAL failures hit `Error_Handler()` in `main.c` (disables IRQs, spins). The transform pipeline uses an opaque handle + hand-built `properties_t`/`capabilities_t`/`stream_buffer_t`; frees are commented out (loop never exits). Linker scripts `STM32H563xx_FLASH.ld` (default) / `STM32H563xx_RAM.ld`; startup `startup_stm32h563xx.s`. `roomscanner/` is a git repository (branch `main`); `53L9A1/` is not.

Known bugs in this package — **do not inherit them into forks** — are catalogued in `ROADMAP.md` →
"Reference-firmware bugs".
