# Stacking the X-NUCLEO-IKS4A1 on the ToF + NUCLEO-H563ZI

Bring-up recipe and bench-validation checklist for stacking the **X-NUCLEO-IKS4A1**
(IMU / mag / baro / temp-humidity) on top of the existing **X-NUCLEO-53L9A1** (VL53L9CX ToF)
+ **NUCLEO-H563ZI** stack.

**Status:** RESOLVED — see "Resolved — HUB1 native-I3C" below. The originally-planned shared-I3C1
approach (IKS4A1's legacy-I2C sensors riding the same bus as the ToF) **fails at operating speed when
the boards are physically stacked** — see "Known conflict" for that investigation. The IKS4A1 *driver*
work for the environmental sensors (via the LSM6DSV16X's own sensor-hub) still lands on a separate
branch (Phase 4).

Source of truth for pins is the `scanner-stream` firmware `.ioc` and the two board schematics under
`references/datasheets/`. Verify against those if anything here looks stale.

## TL;DR

The two shields are designed to stack (both are Arduino UNO R3 form factor), and the plan was for the
IKS4A1 to ride the **same I3C1 bus** the ToF already uses, as legacy-I2C targets, needing no separate
MCU peripheral. **That plan doesn't hold at full speed** — see "Known conflict" below. The three items
below are still necessary but are no longer sufficient on their own:

1. **Match the bus I/O voltage** on both boards (3.3 V).
2. **Keep the IKS4A1 interrupt line(s) off the ToF's control pins** (PB1/PB5/PB6/PB7) — or just poll.
3. **ToF shield on top** (needs a clear field of view and glass-holder clearance).

## Resolved — HUB1 native-I3C (ToF + LSM6DSV16X share the bus)

The conflict documented below is resolved — not by picking one of the "Candidate workarounds," but by
changing *which* IKS4A1 sensor is actually on the shared bus:

- **Jumper change:** the IKS4A1 is jumpered to **HUB1 only** (J4/J5 → `HUB1_SDx`/`HUB1_SCx`, the
  getting-started guide's "Mode 3"). In this configuration only the **LSM6DSV16X (HUB1)** rides the
  shared I3C1 bus — the environmental sensors (LIS2MDL mag, LPS22DF baro, STTS22H temp, SHT40 humidity)
  are no longer reachable from it (see the trade-off below).
- **The LSM6DSV16X is a genuine MIPI I3C v1.1 target** (datasheet DS13510 §5.2, `WHO_AM_I` reg `0x0F` =
  `0x70`). It answers the same ENTDAA the ToF uses and runs at the full 12.5 MHz push-pull speed — no
  legacy-I2C loading, no bus-speed downshift. That's what makes the fix possible: the failure below was
  specific to the IKS4A1's legacy-I2C targets loading the bus at PP speed, not to I3C sharing itself.
- **The fix:** a fork-owned `rs_assign_dynamic_addresses()` in `firmware/scanner-stream/Src/vl53l9_app.c`
  (commits: probe `43f42b9`, function `8c08ff7`, wiring `c84f79b`) replaces the read-only reference's
  single-device `platform_assign_dynamic_address()`. It enumerates both ENTDAA responders and assigns
  each a distinct dynamic address, keyed on **PID.PartID** (not MIPIID — see below), then registers both
  in the I3C controller's device table:
  - ToF (VL53L9CX): PartID `0x0102` (MODEL_ID `0x394C3353`) → kept at `0x52` (`VL53L9_DEFAULT_ADDRESS`).
  - LSM6DSV16X: PartID `0x0070` (`WHO_AM_I` `0x70`) → assigned `0x50` (clear of `0x52` and every IKS4A1
    static address: `0x1E`/`0x38`/`0x5C`/`0x5D`/`0x6A`/`0x6B`).
- **Why PartID, not the plan's MIPIID:** device identity was measured on hardware (an extended
  `iks4a1_i3c_probe()` diagnostic). PID.MIPIID is degenerate here — both devices report identical
  `BCR=0x07` and near-identical MIPIID; PartID is the reliable 16-bit discriminator. Identity was proven
  both ways: a 16-bit MODEL_ID read (only the ToF answers, `0x394C3353` = ASCII "9L3S") and an 8-bit
  `WHO_AM_I` read (only the LSM answers `0x70`).
- **Hardware verification, both stacked:** the native CDC port reappears (it did **not** before this
  fix — the boot hung), and a 15 s capture decoded 422 RAW + 7 CALIB frames, **0 CRC failures, 0 seq
  gaps**, **28.24 fps interval / 28.13 fps wall-clock** (at/above the ~27.76 fps Phase 2.5 baseline),
  CALIB cadence exactly 64, 0 EVENT frames. The known connect-time transient (see Phase 2 in
  `ROADMAP.md`) was present — 0 CRC, characterized-cosmetic.
- **Trade-off, stated explicitly:** HUB1-only routing **disconnects the environmental sensors**
  (LPS22DF baro, LIS2MDL mag, STTS22H temp, SHT40 humidity) from the shared bus. Reading them now
  requires the LSM6DSV16X's own I2C sensor-hub (mode 2) feature — a separate, not-yet-implemented
  driver task, out of this doc's scope.

Full writeup: `docs/superpowers/plans/2026-07-09-iks4a1-hub1-multidevice-i3c.md`.

## Sensor hub (Mode 2) — RESOLVED & WORKING (2026-07-10)

**Fix, for the impatient:** set **J4/J5 to pos `5-6` ONLY** (env sensors isolated on the LSM aux master —
no GND short on `11-12`, no primary-bus short on `1-2`) **and address the barometer at `0x5D`** (SA0=1 on
this board; `0x5C` NACKs). With that, all three env sensors read live over the hub (verified LSM-only:
`P≈982 hPa`, `T≈26.6 °C`, mag; `STATUS_MASTER=0x01` SENS_HUB_ENDOP, `nack=0`). Every firmware register was
correct the whole time — the "no NACK ever / `STATUS_MASTER=0x00`" signature always meant a dead aux bus, and
the two shorts above were it. The `1-2` short is subtle: it ties `SENS` to the STM primary bus, looping the
LSM's aux-master output back onto its own primary interface, so the master never sees a free bus. The
diagnostic history that pinned "electrical, not config" is preserved below.

## Sensor hub (Mode 2) — diagnostic history (how "electrical, not config" was proven)

With HUB1-only routing, the env sensors (LPS22DF/LIS2MDL/STTS22H) are reachable **only** through the
LSM6DSV16X's own I²C sensor hub (Mode 2): the LSM acts as an I²C *master* on its `SDx/SCx` pins, which in
IKS4A1 **Mode 3** (jumpers `J4:5-6`, `J5:5-6` → `HUB1_SDx/SCx = SENS_I2C`, per the getting-started guide)
carry the env-sensor sub-bus. Our `rs_lsm.c` `rs_lsm_shub_init()` configures this, gated behind
`RS_LSM_ENABLE_SHUB`. It does **not** work yet — and the cause is now pinned down.

**Symptom.** `MASTER_ON=1` and fully configured, yet the master *never issues a START*: `STATUS_MASTER`
stays `0x00`, **no slave NACK ever**, no FIFO sensor-hub tags. "No NACK ever" is the tell — a NACK requires
the master to have *addressed* a slave; it never gets that far. So the aux bus (`SENS_I2C`) never reaches
idle-high (SDA/SCL held low) and the master's bus-free check never lets it start. That is electrical.

**Firmware diagnosis is complete — every MCU-controllable cause was ruled out on-target** (built with
`CONF_LSM_PROBE=1` + `RS_LSM_ENABLE_SHUB=1`, flashed, read back over ST-Link VCOM):

| Check | Register readback | Meaning |
|-------|-------------------|---------|
| Enable + slave count latched | `MASTER_CONFIG=0x46` | MASTER_ON=1, AUX_SENS_ON=3-slaves, WR_ONCE=1, START_CFG=0 ✓ |
| Slave address latched | `SLV0_ADD=0xB9` | `0x5C<<1 \| read` — exactly right ✓ |
| Aux-bus pull-up | `IF_CFG=0x40` | **SHUB_PU_EN (bit6) = 1** — internal pull-up IS on ✓ |
| Pins not stolen | `CTRL7=0x00` | AH_QVAR_EN (bit7) = 0 — analog-hub/Qvar not holding `SDx/SCx` ✓ |
| Trigger source alive | SFLP quat @ ~477 Hz | XL/GY data-ready (the internal trigger) is running ✓ |
| Master not wedged | `RST_MASTER_REGS` pulse | no change — not a stale-state lockup ✓ |

Note the earlier note that *"SHUB_PU_EN reads 0"* was a **misdiagnosis**: it read `MASTER_CONFIG` bit3
(which is `not_used0`). SHUB_PU_EN lives in `IF_CFG (0x03) bit6`, and on-target it reads **1**. Likewise
*"trigger never fires"* was wrong — SFLP runs off the same XL/GY data-ready and streams fine.

**Remaining causes are all physical on the IKS4A1** (need a jumper check / scope — the one thing firmware
can't set), ranked:

1. **Leftover Mode-1 GND shunts.** `J4/J5` position `11-12` tie `HUB_SDx/SCx` to GND (Mode 1's default;
   *"HUB1 must be connected to GND if not used"*). If those shunts are still fitted alongside the Mode-3
   `5-6` shunts, the aux bus is clamped to ground and no pull-up can raise it. **Most likely.** Check that
   **only `5-6`** is populated on **both** `J4` and `J5`.
2. **J4/J5 not actually in Mode-3 (`5-6`)** → the env sensors were never on the LSM's aux bus.
3. **Open/cold joint on `SENS_SDA`/`SENS_SCL`**, or one env sensor stuck holding SDA low.

Pass-through mode (AN5763 §7.3) can't isolate this from firmware — it needs an I²C primary interface, and
ours is I3C. Once the jumper is confirmed, re-enable `RS_LSM_ENABLE_SHUB` and reflash; the probe prints
`SLAVEx_NACK`/`SENS_HUB_ENDOP` and env values directly. The `rs_lsm_shub_init()` path already carries the
defensive `RST_MASTER_REGS` pulse + `AH_QVAR_EN` clear (tested harmless) so the next attempt starts clean.

## LSM6DSV16X tuning (applied 2026-07-10, verified on-target)

This rig is **not power-constrained**, so the orientation path favours rate + range (the SFLP quaternion is
the SLAM rotation prior). Knobs live at the top of `rs_lsm.c`:

- **SFLP + XL/GY ODR 120 → 480 Hz** (`RS_LSM_SFLP_ODR`, `RS_LSM_XL_GY_ODR`). 480 Hz is the SFLP ceiling;
  quarters orientation latency and de-blurs fast handheld motion. **Verified live**: probe showed the quat
  tag counting at ~477 Hz (was ~120).
- **Accel full scale ±2g → ±4g, gyro ±250 → ±500 dps** (`RS_LSM_XL_FS`, `RS_LSM_GY_FS`). POR ranges clip on
  handheld shake / wrist flicks; clipping corrupts the fusion far more than the small LSB-resolution loss.
- **High-performance mode** (anti-alias filter on) — already set, kept.
- **Staged (`RS_LSM_SFLP_BATCH_AUX`, default off):** batching the SFLP gravity + gyro-bias vectors to FIFO,
  for host observability once the stream layer demuxes them. The game-rotation vector is internally
  bias-corrected regardless.
- **Follow-up once the hub is alive:** the game-rotation vector has **no magnetometer input → yaw drifts**.
  With the LIS2MDL readable via the hub, run a tilt-compensated e-compass (datasheets dt0058/dt0060) to pin
  absolute heading — the biggest orientation-accuracy win available, and the strongest reason to fix the hub.

## RESOLVED (2026-07-10) — stacked I3C now streams the full sensor suite at 27.85 fps

The "shared I3C fails at operating speed when stacked" conflict below is **fixed in firmware**. Root cause
(confirmed against `references/i2c-i3c-bus-debug-reference.md` §6 + the KiCad netlist + scope): the IKS4A1's
**NXS0108 auto-direction level translator** on the shared PB8/PB9 bus can't pass 12.5 MHz I3C **push-pull**,
so it mis-latches during **ENTDAA** and the ToF (behind the 53L9A1 PI4ULS3V204, the double-shifted path) drops
out while the directly-wired LSM still enumerates. **Fix in `rs_assign_dynamic_addresses()`: slow the push-pull
clock for ENTDAA only** (`SCLPPLowDuration`/`SCLI3CHighDuration = 0xff`; OD kept `0x7c`) → ToF enumerates
100% (105/105). **Ranging stays at full `0x0a`/`0x09`** (steady reads tolerate 12.5 MHz PP; only ENTDAA's
arbitration/handoff stresses the translator). Verified: full stack streams RAW + orientation (stream 9) + env
(stream 10), 333/333/333 paired, **27.85 fps, 0 CRC, 0 gaps**. It was NOT pull-ups, NOT the J4/J5 jumper, NOT
contact — those were all wrong turns. The investigation notes below are kept for history.

## Known conflict — shared I3C1 fails at operating speed when stacked (superseded — see "Resolved" above)

A bench session stacked both boards and drove the shared bus through the full ToF init sequence.
Findings:

- **ENTDAA / open-drain I3C** (`I3C1.SCL_OD_Freq=1852` in the `.ioc`, ~1.85 MHz) completes fine with
  the IKS4A1 stacked — dynamic address assignment is not the problem.
- **I3C push-pull SDR at the configured 12.5 MHz** (`I3C1.SCL_PP_Freq=1250`) fails once the IKS4A1 is
  physically stacked: the ToF init sequence errors out (`handle_error()` in `vl53l9_app.c`) after
  ENTDAA succeeds. The identical build works with the IKS4A1 removed.
- Bit-banged I2C reads against the IKS4A1's static addresses were used as a control and succeeded —
  the bus is electrically alive, this is specifically a PP-speed problem, not a dead bus or address
  collision.
- **Suspected root cause:** the 53L9A1's onboard **NXS0108** I2C/I3C level shifter, combined with the
  extra stub length and parallel pull-up loading the stacked IKS4A1 shield adds to PB8/PB9, can't meet
  I3C PP timing at 12.5 MHz. Not confirmed with a scope trace yet — see "Candidate workarounds" below.
- Diagnostic VCOM traces added to `vl53l9_app.c` for this investigation were reverted afterward; the
  firmware fork is back to its Phase 3 (ToF-only) state. No code trace of this investigation survives
  outside this doc.

**Conclusion:** the "no firmware change, just stack it" plan in the rest of this doc does **not**
hold as written. Read "Candidate workarounds" before the next bench pass.

## Candidate workarounds (superseded — see "Resolved" above)

Ranked least-invasive → most-invasive. Try each in order; each is cheap enough to rule out before
moving to the next.

1. **Lift the IKS4A1's onboard SDA/SCL pull-up solder bridges** (R1/R2 = 4.7 kΩ, see "Pull-ups" under
   Required jumper configuration below). Stacking adds these in parallel with the 53L9A1's own
   pull-ups, dropping effective bus resistance and slowing edge rates exactly where PP-mode timing is
   tightest. No rewiring required.
2. **Re-verify Vio/IOVDD with a meter**, not just jumper position — confirm both boards' bus I/O rail
   is actually 3.3 V under load, not just jumpered to the 3.3 V position. A marginal level-shifter
   threshold reads identically to a pure speed failure.
3. **Unstack the IKS4A1 and connect via short flying leads** instead of the full header stack. Tests
   whether stub length/reflection from the taller two-shield stack — independent of the NXS0108
   theory — is the proximate cause.
4. **Fall back to a second STM32 I2C peripheral** for the IKS4A1, bypassing the Arduino-header-shared
   SDA/SCL entirely. Per `firmware/scanner-stream/*.ioc`, only I3C1 is currently configured for
   I2C/I3C; the ToF already claims PB6 (`XSHUT`) and PB7 (`INTR`), so I2C1's default pins aren't free
   — this would need I2C2 or I2C3 on Morpho-header-only pins, with flying leads from the IKS4A1's I2C
   pads (it's a fixed-pinout Arduino shield; its SDA/SCL can't be rerouted through the header). Real
   hardware rework, not a config change — last resort. The `.ioc`/firmware changes for this are
   Phase 4 driver-work scope, not something to pre-build speculatively.

## Shared bus — the intended path

Both shields land SDA/SCL on the same Arduino pins, which on the H563 are the I3C1 peripheral:

| Signal | STM32 pin | Arduino | ToF (53L9A1) role | IKS4A1 role |
|--------|-----------|---------|-------------------|-------------|
| SCL    | PB8       | D15     | `I3C1_SCL`        | I²C SCL     |
| SDA    | PB9       | D14     | `I3C1_SDA`        | I²C SDA     |

I3C is backward-compatible with legacy I²C, and the H563's I3C1 controller in mixed mode drives both
the VL53L9CX (as an I3C target with a dynamically-assigned address) and the IKS4A1 sensors (as static-
address legacy-I²C targets) on the same two wires.

### No static-address collision

| Device                    | 7-bit addr | Notes |
|---------------------------|------------|-------|
| VL53L9CX (ToF)            | `0x29`     | Also gets an I3C **dynamic** address at runtime |
| LSM6DSV16X (IMU)          | `0x6A`/`0x6B` | SA0-selectable |
| LIS2MDL (magnetometer)    | `0x1E`     | |
| LPS22DF (barometer)       | `0x5C`/`0x5D` | SA0-selectable |
| STTS22H (temp)            | `0x38`     | ADDR-pin-selectable (`0x38`/`0x3C`/`0x3E`/`0x3F`) |

All IKS4A1 addresses are distinct from the ToF's `0x29`. **Requirement for the Phase 4 driver:** when
I3C assigns the ToF's dynamic address (ENTDAA), that address must avoid every IKS4A1 static address
above, and the IKS4A1 sensors must be declared to the I3C controller as legacy-I²C targets (so it uses
open-drain I²C framing for them). That work belongs to the IKS4A1 branch, not this one.

> The IKS4A1 also carries alternate sensors on its adapter/HUB sockets (e.g. LSM6DSO16IS at `0x6A`/`0x6B`,
> LIS2DUXS12). Only populate a combination whose addresses stay distinct — do not enable both the
> LSM6DSV16X and the LSM6DSO16IS at the same address on the shared bus.

## Pin allocation — ToF control lines to keep clear

From the `scanner-stream` `.ioc`, the ToF path already owns these pins beyond the bus:

| STM32 pin | Firmware label | Role |
|-----------|----------------|------|
| PB1       | `SYNC_IN`      | GPIO out — frame sync to sensor |
| PB5       | `CLK_IN` (TIM3_CH2) | sensor reference clock |
| PB6       | `XSHUT`        | GPIO out — sensor enable/reset |
| PB7       | `INTR` (EXTI falling) | ToF data-ready interrupt |
| PB8       | `I3C1_SCL`     | shared bus |
| PB9       | `I3C1_SDA`     | shared bus |

The IKS4A1 routes its sensor interrupts (LSM6DSV16X_INT1/INT2, DRDY, etc.) through a **jumper-selectable
"USER_INT" selector** (`J2` on the IKS4A1) to a single Arduino digital pin, and those lines are
**optional** — the board is fully usable over I²C polling.

- **Simplest (zero GPIO conflict):** leave the IKS4A1 interrupts unconnected and poll over I²C.
- **If a hardware interrupt is wanted** (e.g. LSM6DSV16X data-ready for the SFLP orientation prior):
  route it through the IKS4A1 selector to an Arduino pin that does **not** map to PB1/PB5/PB6/PB7.
  Cross-check the chosen pin against those four using UM3115 (the NUCLEO-H563ZI pinout) before wiring.

## Required jumper / hardware configuration

1. **Bus I/O voltage — match both boards to 3.3 V.** The 53L9A1 has level shifters and a selectable
   host-side I/O rail (`IOVDD` / `Nucleo_IOVDD`, jumper **J1 / JP1**: 3.3 V or 1.8 V). The IKS4A1 has a
   selectable `Vio` (jumper **JP1 / JP2**: 3.3 V / 5 V / 1.8 V). Set **both to 3.3 V** so SDA/SCL swing to
   the same level. A mismatch (one at 1.8 V, one at 3.3 V) is the classic "bus is dead when stacked"
   failure.
2. **Pull-ups — leave as-is unless the bus looks marginal.** Both shields populate SDA/SCL pull-ups
   (IKS4A1 R1/R2 = 4.7 kΩ; 53L9A1 host side ≈ 10 kΩ). In parallel ≈ 3.2 kΩ — fine for I²C/I3C open-drain.
   Only lift the IKS4A1 pull-ups (via its solder bridges) if a scope shows sluggish edges.
3. **ToF CLK_IN switch.** The 53L9A1 can clock the sensor from its onboard 12 MHz oscillator (default,
   `SW1 = INT`) or from the host (`SW1 = EXT`, fed by PB5/TIM3_CH2). Leave it wherever the current
   ToF-only build is known-good; the IKS4A1 does not affect this.
4. **Power.** Both shields run off the Nucleo 5 V / 3V3 rails and have their own regulators. Combined draw
   (ToF ≈ 150 mW + IKS4A1 tens of mW) is well within the ST-Link/E5V budget — no change needed.
5. **Physical stacking order.** Put the **53L9A1 (ToF) on top**: its LiDAR needs an unobstructed
   54×42° field of view and its glass holder stands proud of the PCB. IKS4A1 sits underneath. Confirm the
   header stack height clears the glass holder.

## Bench-validation checklist

The following are already confirmed (see "Known conflict" above) and don't need re-running unless the
hardware setup changes: boards configured at 3.3 V, IKS4A1 interrupts off the ToF control pins, ENTDAA
succeeds stacked, bit-banged I2C reads reach the IKS4A1's static addresses, ToF-only build works with
the IKS4A1 removed.

**What's still open** — re-run this against whichever candidate workaround from the list above is
under test:

- [ ] **ToF still streams with IKS4A1 stacked, at full I3C PP speed:** the actual failing case — confirm
      the ToF init sequence completes past `handle_error()` and frames stream at unchanged rate with
      the workaround applied.
- [ ] **IKS4A1 responds via I3C legacy-I2C (not just bit-bang):** `WHO_AM_I` read of the LSM6DSV16X
      (`0x6A`/`0x6B`) returns `0x70` through the actual I3C1 peripheral in mixed mode, not a bit-banged
      control read.
- [ ] **Scope the bus** during a ToF frame *and* an IKS4A1 read, with the workaround applied. Look
      specifically at PP-mode edge rates on SDA/SCL — this is where the prior failure showed up.
- [ ] **Dynamic address clear of statics:** confirm the ToF's assigned I3C dynamic address is not one of
      `0x1E / 0x38 / 0x5C / 0x5D / 0x6A / 0x6B`.

## Firmware ownership — fix lives in the fork, reference stays read-only

*(This section originally argued the fix should stay out of firmware entirely, deferring to a Phase 4
driver branch. The HUB1 native-I3C resolution above changed that — the fix DID land in firmware, but
only in our fork, never in the reference. Updated to match.)*

The I3C dynamic-address assignment the reference ships (`platform_assign_dynamic_address()` in the
read-only `../53L9A1/` platform layer) is single-device and hardcodes the ToF at `0x52`. Rather than
edit that reference (forbidden — see `CLAUDE.md`), the multi-device fix lives entirely in our fork:
`rs_assign_dynamic_addresses()` in `firmware/scanner-stream/Src/vl53l9_app.c` (see "Resolved" above).
The reference package is byte-for-byte untouched, and the single-ToF case still works — the new
function assigns the ToF `0x52` exactly as before whether or not the IKS4A1 is stacked; it only adds
the second device when one answers ENTDAA.

The environmental-sensor driver work (LIS2MDL / LPS22DF / STTS22H / SHT40, via the LSM6DSV16X's own
I2C sensor-hub) remains a separate, not-yet-started Phase 4 task.

## References

- `references/datasheets/IKS4A1/schematic.pdf` — sensor addresses, INT routing selector, Vio jumpers.
- `references/datasheets/VL53L9CX/x-nucleo-datasheet.pdf` — 53L9A1 schematic, level shifters, IOVDD jumper,
  "I3C on PB8/PB9 by default" note.
- `firmware/scanner-stream/53L9A1_PostprocessSingle.ioc` — authoritative pin map (`I3C1.BusUsage=MixedUsage`,
  PB1/PB5/PB6/PB7/PB8/PB9) and I3C timing (`SCL_OD_Freq`/`SCL_PP_Freq`) referenced in "Known conflict".
- `references/software/x-nucleo-iks4a1/` — STM32duino's official IKS4A1 sensor drivers (one repo per
  sensor: LSM6DSV16X, LSM6DSO16IS, LIS2DUXS12, LIS2MDL, LPS22DF, SHT40-AD1B, STTS22H). The per-sensor
  `src/*_reg.c`/`*_reg.h` files are HAL-agnostic register maps driven through a `ctx_t` with
  function-pointer `read_reg`/`write_reg` callbacks — portable to whatever bus this doc's workaround
  lands on. The `*Sensor.cpp` wrapper classes and `.ino` examples use the Arduino `Wire` API and aren't
  directly portable, but show correct per-sensor init sequences worth cross-checking against.
- UM3115 — NUCLEO-H563ZI board user manual (Arduino/Zio pin ↔ STM32 pin mapping).
- `ROADMAP.md` → Phase 4; bus topology status tracked there.
