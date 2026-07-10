# Default jumper / switch configuration

The as-built default configuration of the three stacked boards, as set for the
roomscanner. Positions are recorded on each board section of the schematic (text
notes), and the one jumper that changes a modeled net — the 53L9A1 `SW1` clock
select — is drawn as an actual component (`SW1`) and wired accordingly.

Legend: **closed** = 2-pin shunt fitted; **1-2 / 2-3** = which pair of a 3-pin
header is bridged; **linked** = 2-pin jumper closed; **open** = no shunt.

## NUCLEO-H563ZI (base)

These configure the base Nucleo (power / debug / USB) and do **not** touch the ST
Zio / Arduino pins, so they have no effect on the stack netlist — recorded for
completeness. All four are at the board's factory default ("Shunt Fitted").

| Jumper | Position | Function | Net effect on stack |
|--------|----------|----------|---------------------|
| `JP2`  | **1-2**  | USB / UCPD power configuration. 1-2 = default (bus/VIN powered, UCPD source). 9-10 = "USB USER" for UCPD **sink** mode. | none |
| `JP4`  | **1-2**  | `VDD_MCU` logic-level select: 1-2 = **3V3**, 2-3 = 1V8. | none (MCU at 3.3 V) |
| `JP5`  | **closed** | `IDD` current-measurement jumper. Closed = MCU powered normally (ammeter bypassed). | none |
| `JP6`  | **closed** | ST-LINK **VCP** UART link (`T_VCP_TX`/`T_VCP_RX` to the MCU USART). | none |

Source: `references/datasheets/NUCLEO-H563ZI/schematic/schematic.pdf` (MB1404).

## X-NUCLEO-IKS4A1 (middle)

| Jumper | Position | Function | Net effect on stack |
|--------|----------|----------|---------------------|
| `J4` / `J5` | **5-6** | I²C bus-routing headers (SDA / SCL). 5-6 routes the STM host I²C through to the sensor bus. | Sets the internal I²C topology modeled as `IKS_SCL/IKS_SDA` |
| `J2`   | **open** | `USER_INT` routing selector (16-pin). Open = no sensor INT/DRDY routed to an Arduino pin. | Confirms sensor INT lines are **NC** in the model |
| `JP1`  | **open** | `Vio` source selector (I²C2 Vio header). | none in model |
| `JP2`  | **open** | `BT_Irq` selector. | none in model |
| `JP5`  | **1-2**  | DIL24 adapter-socket IO voltage: 1-2 = `3V3_IO`. (DIL24 unpopulated.) | none |

The `J4/J5 = 5-6` routing is the configuration the roomscanner settled on for the
shared bus. Source: `.../NUCLEO-IKS4A1/schematic/schematic.pdf` (pages 2 & 4).

## X-NUCLEO-53L9A1 (top)

| Jumper | Position | Function | Net effect on stack |
|--------|----------|----------|---------------------|
| `SW1`  | **INT**  | ToF clock source (SPDT). INT = on-board **Y1 12 MHz**; EXT = host clock (PB5/`CLK_IN`). | **Drawn as `SW1`.** `VL53_S_CLK` ← Y1; the host PB5 path dead-ends at `SW1` EXT |
| `J1`   | **3V3**  | `Nucleo_IOVDD` — host-side level-shifter reference. 3V3 = shifter A-side at 3.3 V. | Confirms shifter `VCCA = +3V3` |
| `J2`   | **linked** | `VBAT_LDD` sensor rail ← on-board 3V3. | Modeled: `VBAT_LDD → +3V3` |
| `J3`   | **linked** | `VBAT_RX` sensor rail ← on-board 3V3. | (abstracted) |
| `J4`   | **linked** | `AVDD` (2.8 V) sensor rail ← on-board LDO. | (LDO abstracted; U7.AVDD = NC in model) |
| `J5`   | **linked** | `DVDD` (1.2 V) sensor rail ← on-board LDO. | (LDO abstracted; U7.DVDD = NC in model) |

`SW1 = INT` is the meaningful one: the ToF runs off the on-board oscillator, so
the host `CLK_IN` (PB5/TIM3_CH2) is **not** used in this default. Source:
`.../NUCLEO-VL53L9CX/schematic/x-nucleo-53l9a1-schematic.pdf` (pages 4 & 5).

## Netlist confirmation (KiCad 10.0.3)

```
VL53_CLK_INT  ->  Y201.3 (Y1 OUT), SW1.1 (INT throw)
VL53_CLK_EXT  ->  U206.12 (host CLK via shifter, from PB5), SW1.3 (EXT throw)
VL53_S_CLK    ->  SW1.2 (COM), U207.E11 (VL53L9CX AP_CLK)
```
i.e. with `SW1` at INT the sensor clock comes from Y1; the PB5 host-clock path
terminates at the switch, unused.
