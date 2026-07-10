---
name: firmware-loop
description: Use when building, flashing, or monitoring STM32 firmware in this project — the full edit→build→flash→observe loop for the NUCLEO-H563ZI, including serial capture over CDC/VCOM.
---

# Firmware build/flash/monitor loop (NUCLEO-H563ZI)

Firmware validation is on-target only — no simulator, no unit tests. Every firmware change ends with
flash-and-observe.

## Which app

- Our fork: `firmware/scanner-stream/` — the app that actually ships (raw-only streaming over native
  USB CDC, command channel, EVENT emission, bounded recovery).
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

Success = `.bin` produced and size printed. FLASH is 2 MB / SRAM 640 KB — current shipped build sits
around 84 KB FLASH (~4%) / 8 KB RAM (~1%); if `size` climbs sharply, stop and rethink buffers before
flashing.

## Flash

ST-Link (on-board V3EC) via STM32CubeProgrammer CLI. The full path (not always on `PATH` on this
machine — overridable via env `ROOMSCAN_PROGRAMMER`, same variable `host/tools/capture.py` reads):

```
C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe
```

Reflash:

```sh
STM32_Programmer_CLI -c port=SWD -w build/Debug/<APPNAME>.bin 0x08000000 -rst
```

Reset only, no reflash (the one-liner used for every [HW] re-verification pass that doesn't change the
binary — e.g. after a command-channel bench, or to recover from a wedged board):

```sh
STM32_Programmer_CLI -c port=SWD -rst
```

Alternative: drag-drop the `.bin` onto the `NOD_H563ZI` mass-storage drive.

**Stale-port care**: either a reflash or a bare `-rst` causes the MCU's USB peripheral to fully
power-cycle, so the native CDC port disappears and re-enumerates a moment later. Reopening too soon
raises a Windows `PermissionError`. Don't guess-and-retry by hand — `host/tools/capture.py`'s
`wait_for_port_cycle()` (used automatically by its `--reset` flag) waits for the vanish then the
reappearance before reopening; reuse it rather than re-deriving the wait loop.

## Capture / monitor

`host/tools/capture.py` is THE tool — it consolidates the whole ritual (optional SWD reset,
CDC port discovery by VID/PID, boot-hang retry, timed raw capture, decode-and-report) that used to be
rebuilt from prose by every `[HW]` task:

```sh
host/.venv/Scripts/python host/tools/capture.py --reset --seconds 15 --out captures/foo.bin
```

Report includes: frame counts by stream, fps under **both** conventions (labeled — see below), CRC
failures, seq gaps (with the known connect-time transient broken out separately), CALIB 64-frame
cadence check, and any EVENT frames decoded.

For live visual inspection instead of a raw dump, use the viewer (`roomscan-view`, or
`view-live.bat`) — it decodes and renders the point cloud in real time and doubles as a runtime control
surface (see below).

Command-channel bench: `host/tools/bench_commands.py` (subcommands `ping` / `calib` / `burst N` /
`corrupted-frame` / `mixed-burst` / `all`) exercises PING/SEND_CALIB/SET_*/REINIT against a live board
and reports per-scenario stream-continuity cost. For a single one-off command, use
`roomscan-ctl` (`host/src/roomscan/control.py`): `roomscan-ctl ping`, `roomscan-ctl usecase 1`, etc.

Never `miniterm` a binary stream — it's for the legacy ASCII/VCOM path only, not the shipped protocol.

## Device control at runtime

Two ways to send COMMAND frames to a live board without recompiling firmware:

- **`roomscan-ctl` CLI** — one command, one process: `roomscan-ctl {ping,calib,usecase,period,exposure,reinit}`.
- **Live viewer key bindings** (`roomscan-view`, only when connected to a real device, not `--replay`):
  `P` = ping, `C` = request an on-demand CALIB frame, `R` = REINIT (full sensor re-init), `1` = switch to
  usecase 0 (AR_RANGE), `2` = usecase 1 (AR_PRECISION, the shipped default).

## Boot-hang behavior (do not hand-roll an external retry loop)

Firmware **self-heals boot hangs internally**: `vl53l9_app()`'s bring-up now runs inside a bounded
5-attempt retry (100/200/400/800/1600 ms backoff) before falling back to the terminal spin, and a
mid-stream fault triggers the same bounded recovery via `handle_error()` → `rs_recover()` (EVENT
`SENSOR_INIT_FAIL`/`SENSOR_ERROR_STATUS`/etc. emitted per attempt). Verified 10/10 on a cold-boot soak
(Phase 3 Task 5). The old "external ~1-in-5 reset-and-retry by hand" workaround is **obsolete** — don't
reintroduce it. `capture.py --boot-timeout`/`--max-boot-retries` still exist as belt-and-braces (a
physical reset that genuinely wedges the board is not the same failure this internal retry targets),
not because the firmware needs external help under normal operation.

## fps convention (state which one you mean)

Two fps numbers are used across this project's reports and they read differently for a stalled-then-
recovered capture — always print **both**, labeled:

- **interval convention**: `(N-1) / ((t_us_last - t_us_first) / 1e6)` over the dominant DATA stream —
  reflects sustained per-frame cadence, insensitive to how long the capture window itself ran.
- **wall-clock convention**: `frames / measured_capture_seconds` — reflects what the host actually saw
  land in its capture window, penalized by any stall/recovery gaps.

`host/tools/capture.py`'s report prints both by name; do the same in any ad hoc script rather than
reporting one bare "fps" number (this exact ambiguity caused real confusion in the P2.5-era reports).

## Observe checklist

- Startup banner / `streams_inspect` dump present → boot OK (legacy ASCII/VCOM path only; the shipped
  raw-only CDC build has no such banner — first RAW/CALIB frame decoding is the success signal).
- For streaming firmware: report actual fps (both conventions), CRC-failure count, and seq-gap count —
  numbers, not "works". `capture.py`'s report gives you all of these in one run.
- Board dead-silent for the CDC port well past `--boot-timeout` → likely genuinely wedged (rare; the
  internal retry above handles the common case). Reset via SWD; if that doesn't recover it, attach a
  debugger rather than guessing.

## Gotchas

- Native USB CDC and ST-Link VCOM are **two different COM ports**; select CDC by VID/PID `CAFE:4001`
  (see `docs/protocol.md`; `roomscan.sources.SerialSource.find_port()` / `CDC_VID`/`CDC_PID`). On this
  machine they've typically enumerated as CDC = COM15, ST-Link VCOM = COM14 — examples, not guarantees;
  always resolve by VID/PID, never hardcode a COM number.
- After flashing or resetting, the app may wait on sensor init — no output for ~1 s is normal; the
  internal boot retry means a slow-but-successful bring-up can take several seconds before the first
  frame, which is expected, not a hang, as long as frames eventually arrive.
- `-Ofast` Release builds can reorder/skip debug prints; verify timing claims on Release, logic on Debug.
- A connect-time transient (~1 CRC failure + ~14-15 KB skipped right at port-open) is a known,
  root-caused, self-healing artifact (docs/connect-transient-forensics.md) — don't mistake it for a
  regression; `capture.py`'s report labels it explicitly as "connect transient: present/absent" and
  excludes it from the mid-stream anomaly count.
- **With the IKS4A1 stacked** (the normal config since Phase 4), a healthy capture shows streams
  **9 (IMU_QUAT) and 10 (ENV)** at ~1:1 with RAW. Streams 9/10 absent while RAW flows = LSM bring-up
  failed (look for its EVENT). A boot that hangs with both boards stacked was the
  NXS0108-vs-12.5 MHz-push-pull ENTDAA problem, **already fixed in firmware** (slow-PP ENTDAA in
  `rs_assign_dynamic_addresses()`) — if it recurs, read `docs/iks4a1-stacking.md` before theorizing
  about hardware. ENV dead but quat alive → jumpers J4/J5 must be **5-6 only**, and the LPS22DF
  barometer is at `0x5D` on this board.
- For a GUI surface with device buttons, IR monitor, and the sensors group (gizmo/compass/sparklines),
  prefer `roomscan-panel` (`roomscan-view --panel`) over the classic keyboard-only viewer.
