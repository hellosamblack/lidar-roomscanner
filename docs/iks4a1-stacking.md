# Stacking the X-NUCLEO-IKS4A1 on the ToF + NUCLEO-H563ZI

Bring-up recipe and bench-validation checklist for stacking the **X-NUCLEO-IKS4A1**
(IMU / mag / baro / temp-humidity) on top of the existing **X-NUCLEO-53L9A1** (VL53L9CX ToF)
+ **NUCLEO-H563ZI** stack.

**Status:** hardware/config prep only. The IKS4A1 *driver* lands on a separate branch (Phase 5).
This branch is untouched on the firmware side — the ToF-only build is unaffected whether or not the
IKS4A1 is physically stacked. Follow this doc to make a merge test-ready and to configure the boards.

Source of truth for pins is the `scanner-stream` firmware `.ioc` and the two board schematics under
`references/datasheets/`. Verify against those if anything here looks stale.

## TL;DR

The two shields are designed to stack (both are Arduino UNO R3 form factor). The IKS4A1 does **not**
need a separate MCU peripheral: it rides the **same I3C1 bus** the ToF already uses, as legacy-I2C
targets. The firmware `.ioc` already has `I3C1.BusUsage=MixedUsage` with I2C timing configured, so no
firmware change is required for the bus itself. Get three things right and it just works:

1. **Match the bus I/O voltage** on both boards (3.3 V).
2. **Keep the IKS4A1 interrupt line(s) off the ToF's control pins** (PB1/PB5/PB6/PB7) — or just poll.
3. **ToF shield on top** (needs a clear field of view and glass-holder clearance).

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

All IKS4A1 addresses are distinct from the ToF's `0x29`. **Requirement for the Phase 5 driver:** when
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

Run this after the IKS4A1 branch is merged and flashed. The mixed I3C + legacy-I²C bus is the only thing
worth scoping — everything else is address bookkeeping.

- [ ] **Boards configured:** both I/O rails at 3.3 V; IKS4A1 interrupt(s) either unconnected or on a
      non-conflicting pin; ToF on top with FoV clear.
- [ ] **ToF-only regression:** with the IKS4A1 *removed*, the existing build streams frames exactly as
      before (no regression from any doc/merge change).
- [ ] **ToF still streams with IKS4A1 stacked:** re-fit the IKS4A1; confirm ToF frames are unaffected —
      no NACKs, no dropped frames, frame rate unchanged.
- [ ] **IKS4A1 responds:** legacy-I²C `WHO_AM_I` read of the LSM6DSV16X (`0x6A`/`0x6B`) returns the
      expected ID (`0x70`); repeat for at least one other sensor (e.g. LIS2MDL `0x1E` → `0x40`).
- [ ] **Scope the bus:** put a probe on SDA/SCL during a ToF frame *and* an IKS4A1 read. Confirm the
      I²C-only sensors (LIS2MDL, LPS22DF, STTS22H) tolerate the ToF's high-speed I3C traffic and the
      `0x7E` DAA broadcast without false-triggering or corrupting a transfer.
- [ ] **Dynamic address clear of statics:** confirm the ToF's assigned I3C dynamic address is not one of
      `0x1E / 0x38 / 0x5C / 0x5D / 0x6A / 0x6B`.

## Why this is prep-only (no firmware change here)

The I3C dynamic-address assignment (`platform_assign_dynamic_address()`) lives in the read-only reference
platform layer, and adding IKS4A1 target declarations / a presence probe is exactly what the Phase 5
branch will implement. Duplicating it here would only create merge conflicts. Keeping this branch's
firmware untouched means the ToF-only build is byte-for-byte unchanged and continues to function whether
or not the IKS4A1 is stacked — which is the requirement for landing this prep ahead of the driver.

## References

- `references/datasheets/IKS4A1/schematic.pdf` — sensor addresses, INT routing selector, Vio jumpers.
- `references/datasheets/VL53L9CX/x-nucleo-datasheet.pdf` — 53L9A1 schematic, level shifters, IOVDD jumper,
  "I3C on PB8/PB9 by default" note.
- `firmware/scanner-stream/53L9A1_PostprocessSingle.ioc` — authoritative pin map (`I3C1.BusUsage=MixedUsage`,
  PB1/PB5/PB6/PB7/PB8/PB9).
- UM3115 — NUCLEO-H563ZI board user manual (Arduino/Zio pin ↔ STM32 pin mapping).
- `ROADMAP.md` → Phase 5; resolves the former "bus topology unresolved" open question.
