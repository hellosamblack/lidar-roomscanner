---
name: iks4a1-i3c-bus-conflict
description: "IKS4A1/ToF I3C1 bus conflict — RESOLVED and merged: HUB1 native-I3C + fork-owned PartID-keyed multi-device ENTDAA assignment, hardware-verified, pushed to origin/main"
metadata: 
  node_type: memory
  type: project
  originSessionId: e1df2767-16de-4ccc-a731-317a24e89441
---

**RESOLVED and shipped (2026-07-09).** The stacked ToF (VL53L9CX) + IKS4A1 LSM6DSV16X now coexist as
two genuine I3C targets on the shared I3C1 bus; the board boots and streams normally with both stacked.
Fix is on `origin/main` (commits `a1dfdc4`..`8983d1d`, unsigned).

**The approach: jumper the IKS4A1 to HUB1 only (J4/J5 → HUB1_SDx/HUB1_SCx), treat the LSM6DSV16X as a
genuine I3C device, and assign the two devices distinct dynamic addresses in fork-owned firmware.**

- Root cause (confirmed): the read-only reference's `platform_assign_dynamic_address()` hardcodes
  "whoever answers ENTDAA first is address 0x52" and manages one device-table entry. With two genuine
  I3C arbiters on the bus that mis-assigns 0x52 → the ToF boot hangs and the native CDC never enumerates.
- The fix: `rs_assign_dynamic_addresses()` in `firmware/scanner-stream/Src/vl53l9_app.c` — enumerates both
  ENTDAA responders, assigns each a distinct dynamic address, and registers both via
  `HAL_I3C_Ctrl_ConfigBusDevices` (completing the TODO the reference's unfinished
  `platform_assign_dynamic_address_multisensor()` left). Wired into both `rs_boot_bringup()` and
  `rs_sensor_reinit()` with the return value checked (feeds the existing bounded boot/recovery retry).

- **KEY CORRECTION vs the plan (owner-approved): discriminate by `PID.PartID`, NOT `PID.MIPIID`.** The
  plan (and the reference's multisensor stub) keyed on MIPIID / instance_id, but on this hardware MIPIID
  is degenerate — both devices report identical `BCR=0x07` and near-identical MIPIID; the plan's original
  probe's "both 0x09" was an artifact of assigning both the same 0x52. PartID is the reliable 16-bit key.
- **Confirmed device constants (measured on hardware, proven both directions via a dual register read):**
  - ToF (VL53L9CX): `PartID = 0x0102`, `MODEL_ID = 0x394C3353` (ASCII "9L3S", 16-bit-addressed read) → kept at `0x52`.
  - LSM6DSV16X: `PartID = 0x0070`, `WHO_AM_I(0x0F) = 0x70` (8-bit-addressed read) → assigned `0x50`.
  - Identification technique that worked: only the ToF answers a 16-bit MODEL_ID read; only the LSM answers
    an 8-bit WHO_AM_I read — cross-checking both positively IDs each. Lives in the extended
    `iks4a1_i3c_probe()` diagnostic (`CONF_IKS4A1_I3C_PROBE`, `0` by default).
- Hardware verification (both stacked): CDC re-enumerated (was absent = the boot hang), 15 s capture =
  422 RAW + 7 CALIB, **0 CRC failures, 0 seq gaps, ~28.2 fps** (≥ the 27.76 baseline), CALIB cadence 64,
  0 EVENT. Host suite 186/186.
- Docs updated (Task 5): `docs/iks4a1-stacking.md` now leads with a "Resolved — HUB1 native-I3C" section
  (historical "Known conflict"/"Candidate workarounds" kept, tagged superseded); `ROADMAP.md` Phase 4
  bus-topology bullet rewritten to the resolved approach.
- Diagnostic probes kept in the fork, all `0` by default: `iks4a1_bus_probe`/`CONF_IKS4A1_BUS_PROBE`
  (legacy-I2C WHO_AM_I sweep) and `iks4a1_i3c_probe`/`CONF_IKS4A1_I3C_PROBE` (native-I3C ENTDAA +
  identification). Note: the probe assigns addresses by arbitration ORDER (0x50/0x52), unlike the real
  boot function's identity-keyed map — it's a never-ship diagnostic only.

**Tradeoff this approach accepts:** HUB1-only jumpering disconnects LPS22DF/LIS2MDL/STTS22H/SHT40 from the
shared bus. Reading them requires the LSM6DSV16X's own I2C sensor-hub feature (mode 2) — a separate,
not-yet-implemented driver task, out of scope for the bus-conflict fix.

**Known non-issue (adjudicated in review):** if an unrecognized device answers ENTDAA,
`rs_assign_dynamic_addresses()` returns `-2` leaving the bus half-configured — but it's unreachable with
the two known devices, and the boot/recovery retry re-enters the function (which RSTDAAs + re-inits timing
every entry), so it self-heals. `CONF_TRANSFORM_ONBOARD=1` builds (default `0`) still use the reference
single-device assign and would hang if stacked — that build is the ToF-only golden-pair path, not stacked
streaming.

**Earlier finding (still true, useful context): the original Mode-1 wiring (all IKS4A1 sensors on the shared
bus as legacy-I2C targets) genuinely fails at 12.5 MHz push-pull once stacked** — suspected NXS0108 level
shifter can't meet PP timing under the added load (not scope-confirmed). The candidate workarounds (lift
pull-ups → verify Vio → flying leads → second I2C peripheral) were never tried; HUB1 superseded them.

See also [[hardware-stack]], [[roadmap-review-notes]], [[firmware-bringup-division-of-labor]].
