# I²C / I3C bus debugging reference — roomscanner sensor stack

**Purpose.** A decision-support reference for debugging the I²C / I3C buses across
the stacked NUCLEO boards. It maps every bus segment to the **jumpers, solder
bridges, switches, test points, resistors, and STM32 IOs** that control or expose
it, so an agent (or human) can reason about *where a bus is broken and what to
change or probe*. Every claim cites its source file + page.

Companion docs in `references/kicad/roomscanner-stack/`:
`stack-pinmap.md` (pin map), `jumper-config.md` (default jumper positions),
`roomscanner-stack.kicad_sch` (consolidated schematic + netlist).

Source schematics (authoritative):
- H563 base — `references/datasheets/NUCLEO-H563ZI/schematic/schematic.pdf` (MB1404, 12 sheets)
- IKS4A1 — `references/datasheets/NUCLEO-IKS4A1/schematic/schematic.pdf` (5 pages)
- 53L9A1 (ToF) — `references/datasheets/NUCLEO-VL53L9CX/schematic/x-nucleo-53l9a1-schematic.pdf` (6 pages)

Firmware pin/bus config (authoritative for *this* project):
- `firmware/scanner-stream/Inc/main.h`, `firmware/scanner-stream/53L9A1_PostprocessSingle.ioc`

---

## 1. Bus at a glance

```
                 STM32H563  I3C1  (controller)
                 PB8 = SCL   PB9 = SDA          <- one physical 2-wire bus
                        │
        ST Zio / Arduino header  CN5.10 (SCL) / CN5.9 (SDA)   [shared by the stack]
                        │
        ┌───────────────┴────────────────────────────┐
        │                                             │
   X-NUCLEO-53L9A1 (top)                        X-NUCLEO-IKS4A1 (middle)
   U5 PI4ULS3V204 (SCL/SDA shifter)             U3 NXS0108 (SCL/SDA shifter)
   3V3 <-> sensor IOVDD (1V8/1V2)               3V3 <-> sensor Vio (1V8)
        │                                             │
   VL53L9CX (0x52 / I3C dyn 0x52)          internal I²C: STM_I2C ─ SENS_I2C ─ HUB2_I2C
   R13/R14 2.2k pull-ups (sensor side)     ─ DIL_I2C, routed by SB matrix + J4/J5
                                           LSM6DSV16X(HUB1) LSM6DSO16IS(HUB2)
                                           LIS2MDL LPS22DF LIS2DUXS12 STTS22H SHT40
```

**One bus, two personalities.** PB8/PB9 is a single 2-wire bus that the firmware
drives as **I3C1 in controller mode** (`53L9A1_PostprocessSingle.ioc`:
`PB8.Signal=I3C1_SCL`, `PB9.Signal=I3C1_SDA`, `PB8.Mode=Controller`,
`I3C1.BusUsage=MixedUsage`). Devices are reached either as:
- **I3C dynamic** — assigned by ENTDAA (PartID → dynamic address). Per project state:
  ToF PartID `0x0102` → `0x52`, LSM6DSV16X PartID `0x0070` → `0x50`.
- **Legacy-I²C static** — the sensors' hard addresses (below), used in MixedUsage.

> Debugging implication: an address that works in one mode may not in the other,
> and ENTDAA enumeration order/participation matters (see §7 playbook).

---

## 2. Master-side IO reference (STM32H563, from firmware)

| Signal | STM32 | Zio pin | Peripheral | Source |
|--------|-------|---------|-----------|--------|
| SCL | **PB8** | CN5.10 | I3C1_SCL (controller) | `main.h`, `.ioc` |
| SDA | **PB9** | CN5.9 | I3C1_SDA (controller) | `.ioc` |
| ToF CLK_IN | PB5 | CN5.4 | TIM3_CH2 (PWM) — *idle by default, see §5 SW1* | `.ioc` `PB5.Signal=S_TIM3_CH2` |
| ToF SYNC_IN | PB1 | CN8.4 | GPIO | `main.h` `SYNC_IN_Pin=PB1` |
| ToF XSHUT | PB6 | CN9.2 | GPIO | `main.h` `XSHUT_Pin=PB6` |
| ToF INTR | PB7 | CN9.1 | GPIO / EXTI7 | `main.h` `INTR_Pin=PB7`, `INTR_EXTI_IRQn=EXTI7` |

On the H563 the Zio Arduino-I²C pins are labelled `I2C_A_SCL/I2C_A_SDA` on
`PB8/PB9` (schematic.pdf p6, *Connectors* sheet). The SDA/SCL path from the MCU
to the header is **direct** — no solder bridge in series (the `SB41` near that
region is on `PF2/PB6/PB7`, not the I²C lines).

---

## 3. Address map (verify against §8 SB positions before trusting)

| Device | Board | 7-bit | ADDw (8-bit) | Addr control | Source (page) |
|--------|-------|-------|--------------|--------------|---------------|
| VL53L9CX (ToF) | 53L9A1 | 0x29 | **0x52** | fixed; I3C dyn 0x52 | 53L9A1 p5 |
| LSM6DSV16X (HUB1 IMU) | IKS4A1 | 0x6A / 0x6B | 0xD4 / 0xD6 | SA0 via **SB17 / SB15**; I3C dyn 0x50 | IKS4A1 p3 |
| LSM6DSO16IS (HUB2 IMU/ISPU) | IKS4A1 | 0x6A / 0x6B | 0xD4 / 0xD6 | SA0 via **SB35 / SB34** | IKS4A1 p3 |
| LPS22DF (baro) | IKS4A1 | 0x5C / **0x5D** | 0xB8 / 0xBA | SA0 via **SB31 / SB29** | IKS4A1 p3 |
| LIS2MDL (mag) | IKS4A1 | 0x1E | 0x3C | fixed | IKS4A1 p3 |
| STTS22H (temp) | IKS4A1 | 0x38 | **0x70** | ADDR pin | IKS4A1 p3 |
| LIS2DUXS12 (accel) | IKS4A1 | 0x18 / 0x19 | 0x30 / 0x32 | SA0 (verify on board) | IKS4A1 p3 |
| SHT40 (RH/T) | IKS4A1 | 0x44 | 0x88 | fixed | IKS4A1 p3 |
| STHS34PF80 (IR presence, on J7) | IKS4A1 | 0x5A | 0xB4 | fixed | IKS4A1 p3 |

> The roomscanner-confirmed live set (firmware): ToF `0x52` (I3C dyn), LSM6DSV16X
> `0x50` (I3C dyn), baro `0x5D`. See the `lsm6dsv16x-panel-integration` /
> `iks4a1-i3c-bus-conflict` memories.

---

## 4. Pull-up resistors (critical for bus health)

The bus SDA/SCL are open-drain; pull-ups from **all stacked boards sum in
parallel**. Too-low combined resistance (over-strong) or too-high (weak edges)
both break I²C/I3C.

| Board | R | Value | Domain / where | Source |
|-------|---|-------|----------------|--------|
| 53L9A1 | R5, R8 | 10k | host side of shifter, to `Nucleo_IOVDD` (=3V3, see J1) | 53L9A1 p4 |
| 53L9A1 | R13, R14 | 2.2k | **sensor-side I3C** SDA/SCL, to sensor rail | 53L9A1 p5 |
| IKS4A1 | R1, R2 | 4.7k | sensor-side bus, to `Vio` (1V8) | IKS4A1 p2 |
| IKS4A1 | R11–R15, R23, R34, R35 | 4.7k | per-sub-bus pull-ups (SENS/STM/HUB), to VDD | IKS4A1 p3 |
| IKS4A1 | R26, R27 | 2.2k | HUB2 / DIL24 bus, to 1V8_IO | IKS4A1 p4 |
| H563 | (SB-gated) | — | Nucleo Arduino-I²C pull-up SBs — usually **not fitted**; verify | H563 p6 |

> Debugging note: the **host-side** shared bus (PB8/PB9, 3V3) sees 53L9A1 R5/R8
> (10k each) plus whatever the IKS4A1 A-side presents; measure the actual bus
> resistance to ground with power off. The **sensor-side** rails are separate
> per board (behind the level shifters) and each have their own strong pull-ups.

---

## 5. Board-by-board control points

### 5.1 H563 base
- **I²C path:** `PB8/PB9` → Zio `CN5.10 / CN5.9`, direct, no series SB.
- **No jumpers on the bus.** JP2/JP4/JP5/JP6 are power/debug/USB (see
  `jumper-config.md`), not on the Zio pins.
- **Probe:** at the Zio header pins CN5.10 (SCL) / CN5.9 (SDA), or MCU pins PB8/PB9.

### 5.2 53L9A1 (ToF, top) — schematic p4–p5
- **Level shifters:** `U5 PI4ULS3V204` (SCL/SDA), `U6 PI4ULS3V204`
  (SYNC/CLK/XSHUT/INTR). Auto-direction translators. Host side ref = `Nucleo_IOVDD`.
- **Bypass/series 0R:** `R6, R7, R9, R10, R11, R12 = 0R` (in/out of shifters —
  can be lifted to isolate host↔sensor for probing). `R15 = 0R` (CLK_IN).
- **Pull-ups:** host `R5, R8 = 10k`; sensor I3C `R13, R14 = 2.2k`.
- **IOVDD selects:**
  - `J1` (3×1): `Nucleo_IOVDD` host-ref = **3V3** (default) or 1V8 / EXT — via `JP1[1-2]`/`JP1[2-3]`.
  - `J6` (3×1): sensor `IOVDD` = 1V8 or 1V2 — via `JP6[1-2]`/`JP6[2-3]`.
- **Sensor rail links:** `J2`=VBAT_LDD, `J3`=VBAT_RX, `J4`=AVDD(2V8), `J5`=DVDD(1V2)
  — **all linked** by default (see `jumper-config.md`), plus `JP2/JP3/JP4/JP5` selects.
- **Clock:** `SW1` SPDT — **INT** (on-board `Y1` 12 MHz) vs **EXT** (host PB5).
  Default INT → host CLK path idle. `SW2` (EVP-AA402W) = XSHUT push-button;
  `R16 180k`, `R17 1k` on XSHUT.
- **ToF address:** 0x52 (fixed), oscillator disable via OE resistor.
- **Test points (Cu), sensor-side signals — best probe points for the ToF bus:**

| TP | Signal | TP | Signal |
|----|--------|----|--------|
| TP1 | GND (clip) | TP6 | SYNC_IN |
| TP2 | GND (clip) | TP7 | CLK_IN |
| TP3 | 3V3 (NUCL) | TP8 | XSHUT |
| TP4 | **SDA** | TP9 | INTR |
| TP5 | **SCL** | | |

  (source: 53L9A1 p2 & p5; TP→signal mapping verified by coordinate proximity.)

### 5.3 IKS4A1 (middle) — schematic p2–p4
- **Level shifters:** `U2, U3 = NXS0108` (auto-direction I²C translators),
  3V3 host ↔ 1V8 sensor. `U1 = LDK130` LDO (Vout = 0.8·(1+R3/R4), R3=15k R4=12k → 1V8).
- **Internal sub-buses:** `STM_I2C` (host-facing), `SENS_I2C` (environmental
  sensors), `HUB2_I2C` (LSM6DSO16IS master aux), `DIL_I2C`/`DIL24_I2C` (adapter
  socket J6), plus `HUB1` = LSM6DSV16X sensor-hub aux master. Names appear on
  every sheet (p1 overview, p3 routing, p4 adapter).
- **Bus master routing — `J4` (SDA ROUTING) / `J5` (SCL ROUTING)**, TMM-106
  (6 positions each). The shunt "selects the master for the environmental
  sensors U6,U7,U8 and Adapter, or enables the Qvar electrode control. **HUB1
  must be connected to GND if not used.**" (IKS4A1 p3 note.) Positions route
  among `STM_Sx / SENS_Sx / HUB2_Sx / HUB1_Sx / DIL_Sx`.
  **Default for roomscanner: J4/J5 = 5-6** (see `jumper-config.md`; matches the
  working config in the `lsm6dsv16x-panel-integration` memory).
- **Vio / INT / QVAR jumpers:** `J1` = I²C2 Vio header (4-pin: GND/SDA/SCL/Vio,
  a convenient **probe/inject point**). `J2` = TMM-108 USER_INT selector (16-pin,
  **open** by default → sensor INTs unrouted). `JP1`=Vio, `JP2`=BT_Irq (both open).
  `JP3/JP4` = per-IMU INT (p3). `JP5` = DIL24 IO voltage 3V3_IO/1V8_IO (1-2).
  `JP6/JP7` = QVAR electrode. `D1/D2 = ESDAXLC6` QVAR ESD.
- **Address-select solder bridges** (fit one of a pair):

| Device | ADDw options | SB pair | Source |
|--------|--------------|---------|--------|
| LSM6DSV16X | 0xD6 / 0xD4 | SB15 / SB17 | p3 |
| LPS22DF | 0xBA (0x5D) / 0xB8 (0x5C) | SB29 / SB31 | p3 |
| LSM6DSO16IS | 0xD6 / 0xD4 | SB34 / SB35 | p3 |
| STTS22H | 0x70 (0x38) | SB32 / SB33 (SENS bus) | p3 |
| LIS2MDL | 0x3C (0x1E) | SB26 / SB30 (SENS bus) | p3 |

- **Bus-routing solder bridges (matrix, p2–p4).** ~56 SBs total; the ones you
  most often touch when a sub-bus is mis-routed: `SB24/SB27` (STM_SCL/STM_SDA),
  `SB25/SB28`, `SB32/SB33`, `SB37/SB39` (SENS), `SB22/SB16/SB19/SB20` (HUB1),
  `SB41/SB43/SB47/SB51/SB49/SB53` + `SB42/SB44/SB48/SB52` (DIL24 / SCL1/SDA1/
  SCL2/SDA2), `SB1–SB12` (connector-side SDA/SCL/USER_INT/SPI). **For the full
  per-SB net assignment, read IKS4A1 schematic p2, p3 ("I2C BUS ROUTING + QVAR"),
  and p4 ("DIL24 Socket") — the SB↔net pairs are printed beside each bridge.**
- **No dedicated TPs on IKS4A1** — probe at `J1` (Vio header), the `J4/J5`
  routing headers, or the SB pads.

---

## 6. Level-shifter caution for I3C — CONFIRMED & FIXED (2026-07-10)

Both expansion boards translate the bus with **auto-direction** I²C level
translators — 53L9A1 `PI4ULS3V204` (U5/U6) and IKS4A1 `NXS0108` (U2/U3). These
sense direction on an **open-drain** I²C bus. I3C SDR runs **push-pull** on parts
of a frame (and higher clocks). An auto-direction translator on a push-pull I3C
segment can mis-latch or fight the driver → corrupted SDA/SCL, failed ENTDAA, or
a device dropping off the bus.

**This hypothesis was CONFIRMED and the issue "ToF drops from ENTDAA when IKS4A1
stacked" is now FIXED in firmware.** The IKS4A1 NXS0108 A-side (U103 in
`roomscanner-stack.net`, on PB8/PB9 alongside the 53L9A1 PI4ULS3V204 U205) can't
pass 12.5 MHz I3C push-pull; it mis-latches during ENTDAA, dropping the ToF (the
double-shifted path) while the LSM still enumerates. **Fix:** slow the push-pull
clock **for ENTDAA only** (`SCLPPLowDuration`/`SCLI3CHighDuration = 0xff`, OD kept
`0x7c`) in `rs_assign_dynamic_addresses()` → ToF enumerates 100% (105/105 passes).
**Ranging stays at full `0x0a`/`0x09`** — steady reads tolerate 12.5 MHz PP; only
ENTDAA's arbitration/handoff stresses the translator. Full stack then streams at
27.85 fps, 0 CRC (RAW + orientation + env). See §7 and the
`lsm6dsv16x-panel-integration` memory.

---

## 7. Debugging playbook (symptom → checks)

**Bus completely dead (no clock/data edges):**
1. Probe SCL/SDA at 53L9A1 **TP5 / TP4** and at Zio CN5.10/CN5.9. No edges at the
   header → MCU/`PB8/PB9` config (`.ioc` I3C1) or power. Edges at header but not
   at TP4/TP5 → shifter (U5) or its 0R bypass (R6/R7/R9-R12) / IOVDD (J1/J6).
2. Check pull-ups (§4): measure bus-to-GND resistance, power off. Expect a few kΩ.

**A device NAKs / not found:**
1. Confirm address vs §3 and the fitted **address-select SB** (§5.3). E.g. baro
   at 0x5D needs SB29 (0xBA) fitted, not SB31.
2. Confirm the device's **sub-bus is routed to the master** — IKS4A1 `J4/J5`
   position and the SENS/STM/HUB SBs (§5.3). Environmental sensors sit on
   `SENS_I2C`; they only reach the STM host if J4/J5 + SBs route SENS↔STM.
3. Confirm the sensor is powered: Vio (JP1/J1), VDD rails, LDO U1 output = 1V8.

**I3C ENTDAA fails / device drops when IKS4A1 is stacked (open issue):**
1. Test the ToF **alone** (no IKS4A1) → if ENTDAA is clean, the IKS4A1 is the
   disturbance. Prime suspects: NXS0108 auto-direction on push-pull I3C (§6),
   combined pull-up loading (§4), or an address/PartID collision during ENTDAA.
2. Try isolating the IKS4A1 host bus: lift/open its host-side SDA/SCL routing
   (relevant SBs / `J1` Vio header removed) and re-run ENTDAA with only the ToF
   enumerated, then add IKS devices back one at a time.
3. Set IKS4A1 `J4/J5` to the known-good **5-6** and ensure **HUB1 tied to GND**
   if unused (p3 note) — a floating HUB1 master can drive the shared lines.
4. Reduce I3C push-pull reliance (MixedUsage / lower SCL_PP) to see if the
   translator tolerates it — diagnostic only.

**Wrong data / intermittent (marginal signal):**
- Over-strong combined pull-up (all boards' pull-ups in parallel, §4) or long
  stubs. Consider removing redundant pull-ups (e.g. lift IKS4A1 R1/R2 or 53L9A1
  R13/R14 when the other board already pulls that rail).

**ToF clock / sync issues:**
- CLK: `SW1` must be **INT** for the on-board Y1 (default). If using host PB5,
  set SW1 = EXT (and enable TIM3_CH2). Probe **TP7** (CLK_IN).
- SYNC/XSHUT/INTR: probe **TP6 / TP8 / TP9**; these pass through U6 (0R + shifter).

---

## 8. Where to read more (per topic → source)

| Topic | File · page |
|-------|-------------|
| STM32 I3C1 config, PB8/PB9, timing regs | `.../scanner-stream/53L9A1_PostprocessSingle.ioc` (`I3C1.*`, `PB8/PB9`) |
| ToF control pins (XSHUT/INTR/SYNC) | `.../scanner-stream/Inc/main.h` |
| Zio header ↔ STM32 pin map | 53L9A1 schematic **p2**; H563 schematic **p6** (Connectors) |
| ToF level shifters, IOVDD select, pull-ups | 53L9A1 schematic **p4** |
| ToF sensor bus, TPs, SW1/SW2, rails, address | 53L9A1 schematic **p5** |
| IKS4A1 connectors, host bus, NXS0108, Vio | IKS4A1 schematic **p2** |
| IKS4A1 I²C routing matrix, SBs, addresses, J4/J5, HUB1 | IKS4A1 schematic **p3** |
| IKS4A1 DIL24 socket, HUB2, QVAR, SB41–56 | IKS4A1 schematic **p4** |
| H563 base jumpers (power/debug) | H563 schematic **p3–p5**; `jumper-config.md` |
| Consolidated netlist (which pins share a net) | `references/kicad/roomscanner-stack/roomscanner-stack.net` |
| Default jumper/switch positions + net effects | `references/kicad/roomscanner-stack/jumper-config.md` |
| Full pin map + sensor roles | `references/kicad/roomscanner-stack/stack-pinmap.md` |
| Resolved I3C multi-device bring-up history | memories `iks4a1-i3c-bus-conflict`, `lsm6dsv16x-panel-integration` |

> When an SB/jumper position matters for a decision, open the cited schematic
> page and read the net labels printed beside the bridge/header — this reference
> lists the control points and defaults, but the schematic is ground truth for
> the exact net each SB connects.
