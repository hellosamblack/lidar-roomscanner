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
  `firmware/vendor/53L9A1/Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/`

## Build

> **On the current headless Linux dev box, skip to "Build + flash on Linux" below.** The
> STM32CubeIDE/`STM32_Programmer_CLI` paths in the next two sections are from the retired Windows box
> and do not exist here.

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

## Build + flash on Linux (the current headless dev box)

`arm-none-eabi-gcc` and `cmake` are on PATH; **Ninja is not**, and `CMakePresets.json` hardcodes the
Ninja generator — install it into the venv once (`host/.venv/bin/python -m pip install ninja`), then:

```sh
PATH="$PWD/host/.venv/bin:$PATH" cmake --build firmware/scanner-stream/build/Debug
```

Flashing works from this box over SWD, but **the apt `stlink-tools` 1.8.0 cannot identify the H5**
(`st-info --probe` → `chipid: 0x000`); H5 IDCODE support landed on stlink `develop` after the v1.8.0
tag. Build it locally, no root needed (~3 min) — full recipe in the `firmware-build-on-linux` memory;
the scratchpad it lands in is **not durable**, so expect to redo this on a fresh box:

```sh
export LD_LIBRARY_PATH=~/scratchpad/stlink-build/src/build/lib
~/scratchpad/stlink-build/src/build/bin/st-info --probe          # expect chipid 0x484 / STM32H5xx
~/scratchpad/stlink-build/src/build/bin/st-flash --connect-under-reset --reset \
    write firmware/scanner-stream/build/Debug/scanner_stream.bin 0x08000000
```

`--connect-under-reset` is **required** — the NUCLEO's NRST isn't bridged to the ST-Link, so a plain
hot-plug halt fails "Can not connect to target". Success = "Flash written and verified! jolly good!".
Observe over Ethernet/UDP (mDNS `roomscanner._roomscan._udp.local.`); no CDC needed.

**No packet capture here:** this is an unprivileged LXC, so `tcpdump` fails with "You don't have
permission to perform this capture on that device". Diagnose the network with ordinary sockets
instead — an ARP sweep (`ping` the /24, then `ip neigh`), a raw mDNS query bound to the board's NIC,
and a UDP wake+listen on :5000.

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
- **Read the boot LEDs first on a silent board** — they cost one glance and rule out whole theories
  (added 2026-07-28 for exactly this; see `BootLedsInit`/`rs_boot_heartbeat` in `main.c`). Healthy is
  **LD1 green solid + LD2 yellow blinking**:

  | LEDs | Meaning |
  |---|---|
  | all dark | never reached `main()` — held in reset, no MCU power, fault before `HAL_Init()` |
  | LD3 red solid | reached `main()`, then wedged in clock/peripheral init, or hit `Error_Handler()`/`handle_error()` |
  | LD1 green solid | clocks + peripherals up |
  | LD2 yellow blinking (~2 Hz) | the acquisition loop is turning over |

  The **PHY's own link/activity LEDs prove nothing** — they come from the LAN8742's power-on
  autonegotiation and the switch's broadcast traffic, and stay lit on a completely dead firmware.
  (LD4/LD6 belong to the ST-LINK block, LD5 = 5 V rail, LD7 = USB_USER VBUS present.)
- **Always check the local logs** when debugging crashes or freezes:
  - `logs/app.log` captures Python UI interactions (buttons, mode changes) and all uncaught Python exceptions.
  - `logs/firmware.log` captures the raw serial `printf` output from the ST-Link VCOM port (1Hz heartbeat + probes).

## Gotchas

- Native USB CDC and ST-Link VCOM are **two different COM ports**; select CDC by VID/PID `CAFE:4001`
  (see `docs/protocol.md`; `roomscan.sources.SerialSource.find_port()` / `CDC_VID`/`CDC_PID`). On this
  machine they've typically enumerated as CDC = COM15, ST-Link VCOM = COM14 — examples, not guarantees;
  always resolve by VID/PID, never hardcode a COM number.
- **ST-LINK VCOM Baud Rate**: The ST-Link VCOM translates host CDC settings to the actual USART pins on the MCU. If your Python host script opens the port at a mismatched baud rate (e.g. 921600) while the MCU's `BSP_COM_Init` is set to 115200, you will get **silent drops** (no output at all), not garbage text. Always match the MCU's `115200` baud rate when reading ST-LINK logs.
- **ST-Link clock dependency — FIXED 2026-07-28 (BUG-023), the board now runs untethered.** History,
  because the symptom is so misleading: the NUCLEO-H563ZI has **no HSE crystal**, so the generated
  `RCC_HSE_BYPASS_DIGITAL` config took its 8 MHz from `STLK_MCO`, the ST-Link MCU's MCO output, whose
  buffer is powered from the ST-Link USB cable (CN1). Unplug CN1 and the target halted in
  `Error_Handler()` before `ETH_Init()` — **powered board, PHY link LED on, activity LED flashing,
  network totally silent**. `SystemClock_Config` now sources PLL1 from **HSI unconditionally** (same
  250 MHz SYSCLK), so only the USB_USER cable is needed, with **JP2 at 9-10**. Two corrections to what
  this entry used to claim: detecting the HSE failure and falling back does **not** work (a floating
  `OSC_IN` in digital-bypass mode appears to set `HSERDY` on noise rather than time out — a forced HSE
  failure with the ST-Link attached is **not** a valid stand-in for the real unplugged case), and an
  unpowered ST-Link does **not** hold the target in reset — the board boots and streams fine with CN1
  removed.
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
  use `roomscan-web` — the browser instrument is the primary, supported UI (real-time + sensors +
  record/playback + SLAM + save; runs on the GPU-less headless host). To SEE/drive it on this box see
  `docs/web-ui-testing.md`. `roomscan-panel` (`roomscan-view --panel`, the Open3D desktop panel) is
  **deprecated legacy** as of Web Phase 5 — it needs a local display + Open3D GUI stack and prints a
  deprecation notice on launch; reach for it only on a box that has a display.
