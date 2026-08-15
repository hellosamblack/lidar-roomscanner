# The reference firmware (`<APP>`)

Scoped from the root `AGENTS.md` (2026-08-15): this only matters when touching the vendored
53L9A1 reference firmware, so it lives here instead of loading into every session.

Throughout this doc, **`<APP>`** = `firmware/vendor/53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/` (the reference firmware app dir). File references like `Src/vl53l9_app.c` are relative to `<APP>`.

Bare-metal firmware for the **STM32H563ZI** (NUCLEO-H563ZI + X-NUCLEO-53L9A1) driving a single
**VL53L9CX ToF 3D LiDAR** over **I3C + DMA** through the `vl53l9-transform-c` pipeline, emitting a
float32 `ZF32` depth frame. Build commands, the three-layer breakdown and the acquisition loop are in
**`docs/reference-firmware.md`**; the build → flash → observe → diagnose loop itself belongs to the
`firmware-loop` skill. Known bugs in this package — **do not inherit them into forks** — are in
`ROADMAP.md` → "Reference-firmware bugs".

Three contracts that bite regardless of what you are doing:

- **No unit tests.** Validation is on-target: flash and read VCOM (115200 8N1).
- **Errors do not recover.** Non-zero `int` return codes funnel to `handle_error()`, which reads
  sensor status and **spins forever**; HAL failures hit `Error_Handler()` in `main.c` (IRQs off,
  spin). Nothing retries, so a wedged rig is the expected symptom of any unhandled return code.
- **Do not hand-edit generated CubeMX init outside the `/* USER CODE BEGIN/END */` guards** — it
  regenerates from `53L9A1_PostprocessSingle.ioc`.
