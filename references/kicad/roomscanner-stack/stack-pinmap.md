# Consolidated NUCLEO stack — pin map

Physical stack (bottom → top):

```
   ┌──────────────────────────────┐
   │  X-NUCLEO-53L9A1  (VL53L9CX)  │  top    — ToF 3D LiDAR
   ├──────────────────────────────┤
   │  X-NUCLEO-IKS4A1  (MEMS/env)  │  middle — IMU + environmental
   ├──────────────────────────────┤
   │  NUCLEO-H563ZI   (STM32H563)  │  base   — MCU + ST-LINK
   └──────────────────────────────┘
```

All three boards mate through the **ST Zio / Arduino Uno V3** headers
(`CN5 / CN6 / CN8 / CN9`). Stacking makes each header pin electrically common,
so the STM32 pin on the base appears on every board above it. That shared bus is
the whole point of the consolidated schematic — in KiCad the merge is realised
by giving the matching header pins the **same global-label net name** (the STM32
pin name).

Ground truth for the header→STM32 mapping is the **X-NUCLEO-53L9A1 schematic**
(which labels each Arduino pin with its STM32 net); the six ToF control-signal
assignments are additionally confirmed against the roomscanner firmware
(`firmware/scanner-stream/Inc/main.h` + `…/53L9A1_PostprocessSingle.ioc`).

## Shared stack bus (STM32 pin ↔ header ↔ function)

| STM32 | Header·pin | Function (H563) | VL53L9CX use | IKS4A1 use |
|-------|-----------|-----------------|--------------|------------|
| **PB8** | CN5.10 | **I3C1_SCL** (Controller) | I3C/I²C SCL → U5 shifter → sensor SCL | I²C SCL → U3 (NXS0108) → MEMS bus |
| **PB9** | CN5.9  | **I3C1_SDA** (Controller) | I3C/I²C SDA → U5 shifter → sensor SDA | I²C SDA → U3 (NXS0108) → MEMS bus |
| **PB5** | CN5.4  | **TIM3_CH2** | CLK_IN → U6 shifter → sensor AP_CLK | (SB-selectable) |
| **PB1** | CN8.4  | GPIO | SYNC_IN → U6 shifter → sensor SYNC_IN | (SB-selectable) |
| **PB6** | CN9.2  | GPIO | XSHUT → U6 shifter → sensor XSHUT | (SB-selectable) |
| **PB7** | CN9.1  | GPIO / **EXTI7** | INTR ← U6 shifter ← sensor INTR | (SB-selectable) |

> The IKS4A1 rides the ToF's PB8/PB9 bus as **legacy-I²C targets** (the resolved
> bus-sharing: ToF at I3C dynamic addr `0x52`, LSM6DSV16X at `0x50`).

## Full ST Zio / Arduino header → STM32 map

Derived from the X-NUCLEO-53L9A1 schematic (the H563 Zio pinout it plugs into).

**CN5 (1×10)** `PF3 · PD15 · PD14 · PB5 · PG9 · PA5 · — · — · PB9 · PB8` (pins 1→10)
**CN6 (1×8, power)** `— · IOREF · NRST · +3V3 · +5V · GND · GND · VIN`
**CN8 (1×6)** `PA6 · PC0 · PC3 · PB1 · PC2 · PF11` (pins 1→6)
**CN9 (1×8)** `PB7 · PB6 · PG14 · PE13 · PE14 · PE11 · PE9 · PG12` (pins 1→8)

Pins CN5.7/CN5.8 are not used by the ToF board (left as no-connect in the model).
`CN7 / CN10` are the STM32 **morpho** headers (2×19). They pass through 1:1 from
the H563 to the IKS4A1 but are **not part of the ToF/IMU sensor signal path**, so
their per-pin STM32 assignments (see UM3115) are not enumerated here — they are
drawn as present connectors with no-connect pins.

## VL53L9CX (top) — level-shifted sensor domain

`IOVDD` selectable 1.2 V / 1.8 V; host side 3.3 V. Two **PI4ULS3V204** shifters:

| Host net | Shifter | Sensor net (VL53L9CX pin) |
|----------|---------|---------------------------|
| PB9 (SDA) | U5 | `VL53_S_SDA` → U7.A12 |
| PB8 (SCL) | U5 | `VL53_S_SCL` → U7.A11 |
| PB1 (SYNC_IN) | U6 | `VL53_S_SYNC` → U7.D12 |
| PB5 (CLK_IN)  | U6 | `VL53_S_CLK` → U7.E11 (also driven by on-board Y1 12 MHz, JP-selectable) |
| PB6 (XSHUT)   | U6 | `VL53_S_XSHUT` → U7.B12 |
| PB7 (INTR)    | U6 | `VL53_S_INTR` → U7.A10 |

On-board LDOs (from CN6 +5V) derive IOVDD/AVDD(2.8 V)/DVDD(1.2 V); abstracted in
this model (VBAT_LDD tied to +3V3, other rails shown as internal nets).

## X-NUCLEO-IKS4A1 (middle) — MEMS + environmental

- **U1 LDK130** (`U101`): +3V3 → **+1V8** sensor-Vio rail (JP-selectable to 3V3).
- **U3 NXS0108** (`U103`): shifts host I²C (PB8/PB9, 3.3 V) → internal
  `IKS_SCL/IKS_SDA` (1.8 V).

All sensors sit on the internal 1.8 V I²C bus (`IKS_SCL/IKS_SDA`). In this model
they share one bus; the real board splits it into `STM_I2C / SENS_I2C / HUB2_I2C`
sub-buses via solder bridges, and HUB1/HUB2 can master sensors over their aux I²C.
`INT/DRDY` lines are SB-routable to Arduino pins (not used as EXTI by firmware) →
left no-connect here.

| Ref (model) | Real | Device | I²C addr (7-bit) | Role |
|-------------|------|--------|------------------|------|
| `U104` | U4  | **LSM6DSV16X** (HUB1) | 0x50/0x52 (dyn) | SFLP-orientation IMU → firmware **stream 9** |
| `U107` | U7  | LIS2MDL      | 0x1E | magnetometer |
| `U106` | U6  | LPS22DF      | 0x5D | barometer (Z-drift constraint) |
| `U105` | U5  | LIS2DUXS12   | 0x18/0x19 | low-power accelerometer / Qvar |
| `U108` | U8  | STTS22H      | 0x38 | temperature |
| `U109` | U9  | LSM6DSO16IS (HUB2) | 0x6A | IMU with ISPU (in-sensor edge AI) |
| `U110` | U10 | SHT40        | 0x44 | relative humidity + temperature |

Baro/mag/temp are live in firmware **stream 10** (sensor-hub env); the LSM6DSV16X
SFLP orientation is **stream 9**.

## Sources
- `references/datasheets/NUCLEO-VL53L9CX/schematic/x-nucleo-53l9a1-schematic.pdf` (header→STM32 ground truth)
- `references/datasheets/NUCLEO-IKS4A1/schematic/schematic.pdf`
- `firmware/scanner-stream/Inc/main.h`, `…/53L9A1_PostprocessSingle.ioc` (ToF pin confirmation)
