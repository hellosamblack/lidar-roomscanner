---
name: firmware-loop
description: Use when building, flashing, or monitoring STM32 firmware in this project — the full edit→build→flash→observe loop for the NUCLEO-H563ZI, including serial capture over VCOM.
---

# Firmware build/flash/monitor loop (NUCLEO-H563ZI)

Firmware validation is on-target only — no simulator, no unit tests. Every firmware change ends with
flash-and-observe.

## Which app

- Our fork: `firmware/scanner-stream/` (once Phase 1 Task creates it).
- Reference (read-only, flash-ok, edit-never):
  `../53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/`

## Build

Requires `arm-none-eabi-gcc` on PATH, CMake ≥3.22, Ninja. On this machine the toolchain is NOT on the
default PATH — it ships with STM32CubeIDE 2.2.0; prepend
`C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\...\tools\bin` (glob for `com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*`)
to PATH for the build. Run **from the app dir**:

```sh
cmake --preset Debug          # once, or after CMakeLists changes; use Release for perf runs
cmake --build build/Debug     # emits .elf + .bin, prints arm-none-eabi-size
```

Success = `.bin` produced and size printed. FLASH is 2 MB / SRAM 640 KB — if `size` shows RAM near
limits, stop and rethink buffers before flashing.

## Flash

ST-Link (on-board V3EC) via STM32CubeProgrammer CLI:

```sh
STM32_Programmer_CLI -c port=SWD -w build/Debug/<APPNAME>.bin 0x08000000 -rst
```

If `STM32_Programmer_CLI` isn't on PATH, it lives under
`C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\`. Alternative: drag-drop the
`.bin` onto the `NOD_H563ZI` mass-storage drive.

## Monitor (ST-Link VCOM)

The VCOM appears as a `STMicroelectronics STLink Virtual COM Port`. Find it:

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()   # or: python -m serial.tools.list_ports -v
```

Text monitor (Phase 0-style ASCII output, 115200 unless our fork changed it):

```sh
python -m serial.tools.miniterm COM<N> 115200
```

Binary capture (never miniterm a binary stream):

```sh
python -m roomscan.viewer --port COM<N> --record capture.bin    # once host/ exists
```

## Observe checklist

- Startup banner / `streams_inspect` dump present → boot OK.
- For streaming firmware: report actual fps, CRC-failure count, and seq-gap count — numbers, not "works".
- Board dead-silent → likely stuck in `handle_error()`/`Error_Handler()` spin: attach a debugger or add
  an error frame/print before the spin; don't guess.

## Gotchas

- Native USB CDC and ST-Link VCOM are **two different COM ports**; select CDC by VID/PID
  (see `docs/protocol.md`).
- After flashing, the app may wait on sensor init — no output for ~1 s is normal; >3 s is a hang.
- `-Ofast` Release builds can reorder/skip debug prints; verify timing claims on Release, logic on Debug.
