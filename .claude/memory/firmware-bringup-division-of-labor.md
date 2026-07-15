---
name: firmware-bringup-division-of-labor
description: Agentic firmware workflow — Claude drives the full build/flash/monitor/diagnose loop itself; the human does physical-only actions
metadata:
  node_type: memory
  type: feedback
  originSessionId: e1df2767-16de-4ccc-a731-317a24e89441
---

**Superseded 2026-07-10 by an explicit owner directive.** This is an agentic project: Claude reads and
writes firmware and **drives the full edit → build → flash → observe → diagnose loop itself**. Do NOT write
up "next bench steps (owner)" or hand off a diagnosis as a plan for the human to implement — take it to the
hardware directly. (The earlier version of this memory said to stop at a written plan; the owner reversed
that: *"This is an agentic project — I want you to read/write firmware yourself to diagnose and fix issues."*)

**Why:** the owner wants throughput, not a relay. Reflashing to read a register or test a hypothesis is
Claude's job, not a task to queue for a human. The board is connected and the toolchain is local, so there
is no reason to defer.

**How to apply:**
- Own the loop: build (STM32CubeIDE gcc on PATH), flash (`STM32_Programmer_CLI -c port=SWD -w ... -rst`),
  monitor (native CDC via `host/tools/capture.py`, VID/PID `CAFE:4001`; ST-Link VCOM COM14 @921600 for
  `printf`/probe output), and read RAM/registers on-target over SWD (`-r32 <addr>`, addresses from
  `build/Debug/*.map`). Full paths + patterns in the `firmware-loop` skill. Implement production fixes
  directly — no "generate a plan and I'll do it" for anything firmware can accomplish.
- Diagnose in firmware to exhaustion before invoking the human. Prove/refute each hypothesis with a
  register readback (e.g. `IF_CFG` for SHUB_PU_EN, `CTRL7` for AH_QVAR_EN, `uwTick` for core-liveness,
  `g_lsm_ok`/`g_last_seq` for boot stage).
- **The human is asked ONLY for physical actions Claude cannot do**, and you name the exact action:
  move a specific IKS4A1/53L9A1 jumper or solder bridge, put a scope on a named net, or power-cycle
  (USB replug) to clear a warm-wedged I3C bus. See [[lsm6dsv16x-panel-integration]] and
  [[iks4a1-i3c-bus-conflict]].
- Caution learned 2026-07-10: rapid flash/reset cycles during probing can **warm-wedge the
  independently-powered ToF/LSM on the shared I3C bus** — it survives MCU software *and* hardware reset.
  Signature: `uwTick` still advances (core alive) but the CDC never enumerates and `g_last_seq`=0 (stuck
  in `rs_boot_bringup` before `tud_connect`). Recovery is a physical USB replug — ask for it, stop resetting.
