---
name: lsm6dsv16x-panel-integration
description: "LSM6DSV16X orientation+env panel: ALL RESOLVED & HW-verified 2026-07-10 — SFLP quat (stream 9, 480Hz), sensor-hub env (stream 10, J4/J5=5-6 + baro 0x5D), stacked ToF ENTDAA (slow-PP fix); full stack 27.85fps 0 CRC. Phase 4 closed."
metadata: 
  node_type: memory
  type: project
  originSessionId: 04f3762e-9751-4b07-9cc9-0b144beb33b8
---

Feature: stream the LSM6DSV16X's data to the roomscan panel (spec
`docs/superpowers/specs/2026-07-09-lsm6dsv16x-orientation-env-panel-design.md`, plan
`docs/superpowers/plans/2026-07-09-lsm6dsv16x-orientation-env-panel.md`). Built subagent-driven.
Commits on local main `abc9691..9e387f3` (unpushed as of 2026-07-09, unsigned). Builds on
[[iks4a1-i3c-bus-conflict]] (LSM at 0x50 on shared I3C1).

**DONE + hardware-verified: SFLP orientation (stream 9 IMU_QUAT).**
- Host lane (Tasks 1-6, all reviewed, 208 tests green): protocol streams 9 IMU_QUAT (4×f32 [w,x,y,z], 16B)
  + 10 ENV (f32 Pa + 3×f32 µT + f32 °C, 20B); `roomscan/sensors.py` (thread-safe SensorState + quat math +
  tilt-compensated heading), `sensors_widgets.py` (compass + sparklines), `panel.py` (live 3D gizmo in the
  SceneWidget + "Sensors" group), config fields imu_gizmo/sensors_panel/gizmo_scale.
- Firmware: `firmware/scanner-stream/Src/rs_lsm.c` (+ vendored `Drivers/lsm6dsv16x/lsm6dsv16x_reg.c`).
  SFLP game-rotation-vector via FIFO, fp16→quaternion (w=sqrt(1-|v|²)). Emitted per ToF frame from
  `vl53l9_app.c`. Verified: 284 IMU_QUAT paired with 284 RAW, 0 CRC, 0 gaps, 28.5 fps — ToF cadence
  unaffected.
- **KEY GOTCHA: do NOT software-reset the LSM (`GLOBAL_RST`)** — it drops the ENTDAA-assigned I3C dynamic
  address (device falls off 0x50). Configure from POR defaults instead. (This cost one debug cycle.)

**RESOLVED + WORKING (2026-07-10): sensor-hub environmental (baro/mag/temp, stream 10 ENV).**
Verified on-target via CONF_LSM_PROBE (ToF removed, LSM-only): all three slaves reading —
**P=982 hPa (LPS22DF), T=26.6 C (STTS22H), mag (LIS2MDL); shstat=0x01 (SENS_HUB_ENDOP), nack=0.** The long
"master never cycles / no NACK ever / STATUS_MASTER=0x00" hunt was TWO things, both OUTSIDE firmware — every
register was correct all along (MASTER_CONFIG=0x46, IF_CFG=0x40 SHUB_PU_EN=1, SLV0_ADD latched, CTRL7=0x00,
SFLP trigger alive; the famous "SHUB_PU_EN reads 0" was a misread of MASTER_CONFIG bit3 = not_used0, not
IF_CFG bit6):
1. **The aux bus was electrically dead — J4/J5 jumpers.** First shorted to GND (pos 11-12), then (after the
   user removed those) shorted to the STM **primary** bus (pos 1-2, which loops the LSM's aux-master output
   back onto its own primary interface → master never gets a free bus). **FIX: J4/J5 = pos 5-6 ONLY** (env
   sensors isolated on the LSM aux master). "No NACK ever" always meant a dead bus, never config.
2. **Barometer is at 0x5D (SA0=1) on this board, not 0x5C** — 0x5C NACKed (slave0_nack). Mag 0x1E, temp 0x38
   unchanged. Baro solder-bridge SB31=0x5C / SB15=0x5D per the IKS4A1 schematic.
`RS_LSM_ENABLE_SHUB`=1 now. Diagnostics kept in the fork: `g_lsm_if_cfg`, `g_lsm_slv0_add`, `g_lsm_ctrl7_pre`,
`g_lsm_master_config`, `rs_lsm_shub_status_raw()`, `g_lsm_tag_hist[]`; plus a harmless RST_MASTER_REGS pulse +
AH_QVAR_EN clear (defensive, not the fix). Full writeup: `docs/iks4a1-stacking.md` "Sensor hub (Mode 2)".

**RESOLVED (2026-07-10): ToF drops from ENTDAA when IKS4A1 stacked — FIXED IN FIRMWARE via slow-PP ENTDAA.**
Root cause (confirmed against the stack-electrical model + scope): the IKS4A1's **NXS0108 auto-direction level
translator** (U3, A-side on the shared PB8/PB9 I3C bus per `roomscanner-stack.net`) can't pass **12.5 MHz I3C
push-pull**; it mis-latches during ENTDAA, so the ToF (behind the 53L9A1 PI4ULS3V204, the double-shifted path)
drops out while the directly-wired LSM still enumerates. NOT pull-ups (IKS4A1 R1/R2 are sensor-side behind the
translator), NOT the J4/J5 jumper (routes the internal sub-bus, not PB8/PB9), NOT contact.
FIX in `rs_assign_dynamic_addresses` (`vl53l9_app.c`): **slow the push-pull clock for ENTDAA only**
(SCLPPLowDuration/SCLI3CHighDuration = 0xff, OD kept 0x7c) → ToF enumerates 100% (diagnosed 105/105 passes via
the continuous-ENTDAA CONF_IKS4A1_I3C_PROBE). **Ranging stays at full 0x0a/0x09** — steady reads tolerate
12.5 MHz PP; only ENTDAA's arbitration/handoff stresses the translator's direction-sensing.
**FULL STACK VERIFIED streaming:** RAW_3DMD + stream 9 (quat) + stream 10 (env) all 333/333/333 paired,
**27.85 fps, 0 CRC, 0 gaps** with ToF + orientation + baro/mag/temp all live. `rs_assign_dynamic_addresses`
also keeps a multi-device ENTDAA retry (handles a residual race). Debug ref §6/§7.4 hypothesis = CONFIRMED.

**Orientation tuning applied + verified on-target (2026-07-10):** SFLP & XL/GY ODR 120→**480 Hz** (probe
confirmed quat @ ~477 Hz), accel FS ±2g→**±4g**, gyro ±250→**±500 dps**, high-perf mode. Knobs at top of
`rs_lsm.c` (`RS_LSM_XL_GY_ODR`/`RS_LSM_SFLP_ODR`/`RS_LSM_XL_FS`/`RS_LSM_GY_FS`). Staged off:
`RS_LSM_SFLP_BATCH_AUX` (gravity+gbias FIFO batching, awaits host demux).

**GOTCHA (still true): LSM config persists across an MCU `-rst`** (independently powered; no GLOBAL_RST) —
set every state explicitly. And rapid probe flash/reset cycles can warm-wedge the shared I3C bus (ToF stuck
in bring-up, no CDC) → needs a physical USB replug; see [[firmware-bringup-division-of-labor]].

Env-side note: mag is streamed only for future host-side yaw correction — SFLP is 6-axis (no on-chip mag
fusion), so yaw drifts; datasheets dt0058/dt0060 give the tilt-compensated e-compass recipe once the hub
is alive. Accepted for viz scope (see the spec's drift subsection).
