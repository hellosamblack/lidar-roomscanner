---
name: stack-electrical
description: Use when reasoning about or suggesting electrical changes to the stacked NUCLEO boards — jumpers, solder bridges, switches, test points, I2C/I3C bus routing, pull-ups, pin/IO assignments, addresses, or any wiring question across the H563 + IKS4A1 + 53L9A1 stack.
---

# Stack electrical model (H563 + IKS4A1 + 53L9A1)

The tethered scanner is three stacked ST boards. This skill is the map to the
electrical model + how to reason about and propose changes to it.

```
NUCLEO-H563ZI (base, STM32H563)  →  X-NUCLEO-IKS4A1 (middle, MEMS/env)  →  X-NUCLEO-53L9A1 (top, VL53L9CX ToF)
```
They mate through the **ST Zio / Arduino Uno V3 headers** (`CN5/CN6/CN8/CN9`);
stacking makes each header pin electrically common. The one shared bus that
matters is **I3C1 on PB8 (SCL) / PB9 (SDA)**, level-shifted onto each board.

## Where the truth lives (read before answering)

| Question | Authoritative source |
|----------|----------------------|
| Which STM32 pin ↔ which sensor, sensor roles, addresses | `references/kicad/roomscanner-stack/stack-pinmap.md` |
| Default jumper/switch positions + their net effect | `references/kicad/roomscanner-stack/jumper-config.md` |
| Jumpers, solder bridges, switches, test points, pull-ups, bus routing, debug playbook | `references/i2c-i3c-bus-debug-reference.md` |
| What actually shares a net (KiCad netlist) | `references/kicad/roomscanner-stack/roomscanner-stack.net` |
| The consolidated schematic (open in KiCad) | `references/kicad/roomscanner-stack/roomscanner-stack.kicad_pro` |
| Exact net beside any SB/jumper (ground truth) | the board schematic PDFs — see the source table in the debug reference |
| Firmware pin/bus config for THIS project | `firmware/scanner-stream/Inc/main.h`, `.../53L9A1_PostprocessSingle.ioc` |

**The board schematic PDFs are ground truth.** The consolidated KiCad project is a
*stack-interconnect model* (documentation), not a fab target: it draws the shared
connectors + the functional ICs on the signal path and abstracts on-board LDOs,
decoupling, ST-LINK/USB/Ethernet, and the morpho pass-through. Never state a pin
map, SB↔net, or address you haven't confirmed in a schematic page or the firmware
— cite the page/file. PDF text extracts via `pymupdf` (installed) if you can't
render pages (`pdftoppm`/poppler is NOT installed on this machine).

## Mental model for electrical decisions

1. **One bus, two personalities.** PB8/PB9 runs as **I3C1 controller, MixedUsage**.
   Devices appear as I3C-dynamic (ENTDAA: ToF `0x52`, LSM `0x50`) *or* legacy-I²C
   static (0x6A, 0x5D, …). Always say which mode a change assumes.
2. **Pull-ups sum in parallel across boards.** Each board adds its own (53L9A1
   R5/R8 10k + R13/R14 2.2k; IKS4A1 R1/R2 4.7k …). Proposing to add/remove a
   device or board changes the effective bus pull-up — flag it.
3. **Auto-direction translators on an I3C bus are suspect.** Both expansion boards
   use them (53L9A1 `PI4ULS3V204`, IKS4A1 `NXS0108`). They sense direction on
   open-drain I²C; I3C SDR is push-pull → mis-latch risk. This is the leading
   hypothesis for "ToF drops from ENTDAA when IKS4A1 stacked" (see debug ref §6–7
   and the `lsm6dsv16x-panel-integration` memory).
4. **Jumper vs SB vs switch:** jumpers/switches are user-settable (record in
   `jumper-config.md`); solder bridges are rework. The IKS4A1 sub-bus topology
   (STM/SENS/HUB1/HUB2/DIL) is set by the `J4/J5` routing headers (default **5-6**)
   + an SB matrix; the ToF clock source is `SW1` (default **INT** = on-board Y1).

## Proposing an electrical change — procedure

1. **Locate the control point.** Find the jumper/SB/switch/resistor and its exact
   net in the debug reference, then confirm against the cited schematic page.
2. **Trace the net effect** in `roomscanner-stack.net` (which pins share the net).
   State what moves and what else sits on that net (loading, address, power).
3. **Check the second-order effects:** pull-up budget, voltage domain (Vio/IOVDD),
   I3C-vs-I²C mode, ENTDAA participation, address collisions, whether a floating
   master (HUB1) must be grounded.
4. **Say whether it's user-settable** (jumper/switch → just move it, update
   `jumper-config.md`) **or rework** (solder bridge → note the pads + risk).
5. **If it changes a *modeled* net, reflect it in the KiCad model** (see below) so
   the netlist and docs stay honest. If it only touches an abstracted part, update
   the relevant `.md` and note the abstraction.

## Editing / regenerating the KiCad model

The schematic is generated, not hand-drawn — edit the generator, don't hand-edit
`.kicad_sch`:

```sh
# generator: references/kicad/roomscanner-stack/generate_stack.py  (uses kiutils, installed)
python references/kicad/roomscanner-stack/generate_stack.py    # rewrites .kicad_sch/.kicad_sym/.kicad_pro + re-copies datasheets
```

Connectivity is by **same-named labels**: global labels = STM32-pin nets shared
across the stack (`PB8`…); local labels = board-internal nets (`VL53_S_SCL`,
`IKS_SCL`…). A jumper/switch that changes routing is a real component (see `SW1`
for the pattern: SPDT with COM/INT/EXT, wired so the netlist reflects the default).
Expansion-board refs are offset +100 (IKS4A1) / +200 (53L9A1); `= CNx`/`= Ux` text
gives the real silk name.

**Always validate with KiCad 10 CLI after regenerating** (KiCad is installed):

```sh
KCLI="/c/Users/hello/AppData/Local/Programs/KiCad/10.0/bin/kicad-cli.exe"
"$KCLI" sch erc roomscanner-stack.kicad_sch -o roomscanner-stack-erc.rpt   # expect 0 violations
"$KCLI" sch export netlist -o roomscanner-stack.net roomscanner-stack.kicad_sch
"$KCLI" sch export pdf -o roomscanner-stack.pdf roomscanner-stack.kicad_sch
```

Confirm the intended net actually merged/split (parse the `.net`), not just that
ERC passed — a symbol with `on_board no` or a mismatched instance path silently
drops from the netlist (both were real bugs; the generator sets them correctly).

## Guardrails

- Don't invent pin maps, SB↔net pairs, or addresses — cite a schematic page or the
  firmware. When you can't verify, say so and point at the exact page to read.
- Keep the four docs above consistent when you change one (addresses, jumper
  positions, net effects). ERC-clean (0 violations) is the bar for the schematic.
- Base-board jumpers (JP2/JP4/JP5/JP6) are power/debug/USB — not on the Zio pins;
  they don't affect the sensor bus.
- Physical actions (moving a jumper, reflowing an SB, scope probing) are the
  human's per `docs/engineering-practices.md`; name the exact jumper/SB/test point
  and position, and where to probe (e.g. 53L9A1 TP4=SDA, TP5=SCL).
