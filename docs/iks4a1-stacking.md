# Stacking the X-NUCLEO-IKS4A1 on the ToF + NUCLEO-H563ZI

Bring-up recipe and bench-validation checklist for stacking the **X-NUCLEO-IKS4A1**
(IMU / mag / baro / temp-humidity) on top of the existing **X-NUCLEO-53L9A1** (VL53L9CX ToF)
+ **NUCLEO-H563ZI** stack.

**Status:** bench-tested, and the shared-I3C1 approach below **fails at operating speed when the
boards are physically stacked**. See "Known conflict" below before wiring anything up. The IKS4A1
*driver* still lands on a separate branch (Phase 4); this branch remains untouched on the firmware
side — the ToF-only build is unaffected whether or not the IKS4A1 is physically stacked.

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

## Known conflict — shared I3C1 fails at operating speed when stacked

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

## Candidate workarounds

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

## Why this branch stays firmware-untouched

The I3C dynamic-address assignment (`platform_assign_dynamic_address()`) lives in the read-only reference
platform layer, and adding IKS4A1 target declarations / a presence probe is exactly what the Phase 4
branch will implement — once a workaround from the list above is validated on the bench. Duplicating
that work here would only create merge conflicts. Keeping this branch's firmware untouched means the
ToF-only build is byte-for-byte unchanged and continues to function whether or not the IKS4A1 is
stacked, regardless of which workaround eventually lands.

This doc used to describe an unvalidated paper design ("prep only"); it now reflects an actual bench
result (see "Known conflict"). The open item is no longer "write the driver" — it's "pick and validate
a workaround" per the ranked list above, before the Phase 4 driver branch starts.

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
