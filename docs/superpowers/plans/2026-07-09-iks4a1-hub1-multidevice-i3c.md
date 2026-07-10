# IKS4A1 HUB1 Native-I3C Multi-Device Bus Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ToF (VL53L9CX) and the IKS4A1's LSM6DSV16X (HUB1) coexist as two genuine
I3C targets on the shared I3C1 bus, so the board boots and streams normally with both
physically stacked.

**Architecture:** Replace the single-device ENTDAA assignment the boot path currently calls
(`platform_assign_dynamic_address()`, read-only reference code that hardcodes "whoever
answers ENTDAA first is the ToF, address 0x52") with a new function owned by our fork that
distinguishes the two real devices by their MIPI instance ID during ENTDAA arbitration,
assigns each its own dynamic address, and registers both in the I3C1 controller's device
table (the read-only reference's multi-device variant does the first two but never finishes
the last — this plan completes it in our own code instead of editing the reference).

**Tech Stack:** STM32H563ZI, STM32H5 HAL I3C driver, CMake + Ninja + arm-none-eabi-gcc,
STM32CubeProgrammer CLI, `host/tools/capture.py`.

## Global Constraints

- **Never edit anything under `../53L9A1/`** (the read-only reference package — CLAUDE.md).
  `platform_utils.c`, `vl53l9_device.c`, and `vl53l9_device.h` all live there and must not be
  touched, even though `vl53l9_device.c` literally invites it ("add more entries to this
  array in case of multiple sensors") and `platform_utils.c`'s
  `platform_assign_dynamic_address_multisensor()` has a `// TODO: add
  HAL_I3C_Ctrl_ConfigBusDevices call`. Every fix in this plan lives in our fork instead.
- **This plan assumes the IKS4A1 is jumpered to HUB1 only** (J4/J5 → HUB1_SDx/HUB1_SCx,
  per the IKS4A1 getting-started guide's "Mode 3" description) — the physical state as of
  2026-07-09's bench session, already validated: with the IKS4A1 alone in this config, the
  LSM6DSV16X answers ENTDAA cleanly (BCR=0x07, matching the datasheet's documented
  `GETBCR` value) and reads back WHO_AM_I=0x70 over genuine native I3C at full 12.5 MHz
  push-pull speed, 5/5 reliable. This plan is what's needed for the *shared* bus (ToF +
  IKS4A1 both stacked) to work the same way.
- **The ToF's own I3C address must stay `VL53L9_DEFAULT_ADDRESS` (0x52).** It's hardcoded
  in the read-only `vl53l9_device.c`'s `device[]` table and used by every existing
  `vl53l9_*` driver call — this plan must not change it, only add a second, distinct
  address for the LSM6DSV16X alongside it.
- **Build/flash/monitor per `.claude/skills/firmware-loop/SKILL.md`**: Debug preset, ARM
  toolchain prepended to `PATH` from the STM32CubeIDE install, `STM32_Programmer_CLI -c
  port=SWD -w build/Debug/scanner_stream.bin 0x08000000 -rst`. **VCOM (COM14 on the bench
  machine) is configured at 921600 baud** (`main.c:140`), not the usual 115200 — every
  capture step below must use that rate or you'll see nothing.
- Every task's on-target verification is a flash-and-observe step (no unit test framework
  for this firmware — CLAUDE.md: "No unit tests — validation is on-target").

## File Structure

Only one file changes: `firmware/scanner-stream/Src/vl53l9_app.c`.

- **New** (inserted just before `rs_sensor_reinit()`, currently at line 544): two `#define`
  constants and one new static function, `rs_assign_dynamic_addresses()`.
- **Modified**: the two active `platform_assign_dynamic_address()` call sites — inside
  `rs_sensor_reinit()` (line ~549) and `rs_boot_bringup()` (line ~678) — switch to call the
  new function and check its return value (the original code never checked
  `platform_assign_dynamic_address()`'s result at all; this plan fixes that too, in the same
  spirit as the existing "Reference-firmware bugs — do not inherit" list in `ROADMAP.md`).
- A third, currently-inactive call site exists inside the `#else` branch of `#if
  CONF_TRANSFORM_ONBOARD` (the on-MCU-transform golden path, `CONF_TRANSFORM_ONBOARD` is
  `0` in this fork, i.e. dead code in the shipped build). Leave it alone — out of scope,
  not exercised.
- The existing diagnostic probes already in this file (`iks4a1_bus_probe`,
  `CONF_IKS4A1_BUS_PROBE`; `iks4a1_i3c_probe`, `CONF_IKS4A1_I3C_PROBE`, both currently `0`)
  stay as-is. Task 1 below reuses and extends `iks4a1_i3c_probe()` rather than writing a new
  probe from scratch.

---

### Task 1: Capture the real MIPI instance ID for both devices

**Files:**
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c` (inside `iks4a1_i3c_probe()`, the
  existing `CONF_IKS4A1_I3C_PROBE`-gated function)

**Interfaces:**
- Consumes: `HAL_I3C_Get_ENTDAA_Payload_Info(I3C_HandleTypeDef *hi3c, uint64_t
  ENTDAA_payload, I3C_ENTDAAPayloadTypeDef *pENTDAA_payload)` (stm32h5xx_hal_i3c.h:1270);
  `I3C_ENTDAAPayloadTypeDef` = `{ I3C_BCRTypeDef BCR; uint32_t DCR; I3C_PIDTypeDef PID; }`;
  `I3C_PIDTypeDef` = `{ uint16_t MIPIMID; uint8_t IDTSEL; uint16_t PartID; uint8_t MIPIID;
  }` (stm32h5xx_hal_i3c.h:360-386).
- Produces: two hex byte values (the ToF's and the LSM6DSV16X's `PID.MIPIID`), which Task 2
  hardcodes as constants. **These cannot be guessed or looked up — they only exist by
  running this step against the real hardware.** Do not proceed to Task 2 without them.

This is why the plan can't hand you final numeric constants for Task 2 up front: they're
measured, not designed.

- [ ] **Step 1: Extend `iks4a1_i3c_probe()`'s retry loop to print the decoded PID per attempt**

Find the existing `do { ... } while (daa_status == HAL_BUSY && attempts < 5);` loop inside
`iks4a1_i3c_probe()` (it currently prints raw `payload_hi`/`payload_lo` — this step adds a
properly-decoded PID line using the real HAL helper instead of manual byte-slicing). Add
right after the existing `printf("[IKS4A1 I3C PROBE] attempt %d: ...")` line, still inside
the `do { ... }` body:

```c
        I3C_ENTDAAPayloadTypeDef pinfo = { 0 };
        HAL_I3C_Get_ENTDAA_Payload_Info(&hi3c1, payload, &pinfo);
        printf("[IKS4A1 I3C PROBE]   decoded PID: MIPIMID=0x%04X IDTSEL=0x%02X PartID=0x%04X MIPIID=0x%02X BCR=0x%02X DCR=0x%02lX\n",
               pinfo.PID.MIPIMID, pinfo.PID.IDTSEL, pinfo.PID.PartID, pinfo.PID.MIPIID,
               (unsigned)pinfo.BCR.MaxDataSpeedLimitation /* placeholder read below */,
               (unsigned long)pinfo.DCR);
```

The `BCR` field of `I3C_ENTDAAPayloadTypeDef` is an `I3C_BCRTypeDef` struct (booleans, not a
raw byte) — printing it meaningfully needs the same `__HAL_I3C_GET_BCR(payload)` macro the
existing code already uses two lines above (`uint32_t bcr = __HAL_I3C_GET_BCR(payload);` is
already computed later in the function for the final status line — move that line up, or
just recompute it here):

```c
        I3C_ENTDAAPayloadTypeDef pinfo = { 0 };
        HAL_I3C_Get_ENTDAA_Payload_Info(&hi3c1, payload, &pinfo);
        uint32_t attempt_bcr = __HAL_I3C_GET_BCR(payload);
        printf("[IKS4A1 I3C PROBE]   decoded PID: MIPIMID=0x%04X IDTSEL=0x%02X PartID=0x%04X MIPIID=0x%02X BCR=0x%02lX DCR=0x%02lX\n",
               pinfo.PID.MIPIMID, pinfo.PID.IDTSEL, pinfo.PID.PartID, pinfo.PID.MIPIID,
               (unsigned long)attempt_bcr, (unsigned long)pinfo.DCR);
```

Set `#define CONF_IKS4A1_I3C_PROBE (1)` (and confirm `CONF_IKS4A1_BUS_PROBE` is `0`) before
building.

- [ ] **Step 2: Build**

From `firmware/scanner-stream/`, with the ARM toolchain on `PATH` (see Global Constraints):

```sh
cmake --build build/Debug
```

Expected: clean build, `.bin` produced, no new warnings beyond the pre-existing
`frame_rate`/`print_frame` unused-symbol warnings.

- [ ] **Step 3: Flash with both devices physically stacked**

```sh
STM32_Programmer_CLI -c port=SWD -w build/Debug/scanner_stream.bin 0x08000000 -rst
```

Confirm the ToF (53L9A1) and the IKS4A1 (jumpered to HUB1 only, per Global Constraints) are
both stacked before this step — the whole point is capturing both devices' arbitration in
one ENTDAA cycle.

- [ ] **Step 4: Capture VCOM output at 921600 baud**

Any serial tool works; a short Python one-liner (pyserial) is enough:

```sh
host/.venv/Scripts/python.exe -c "
import serial, time
ser = serial.Serial('COM14', 921600, timeout=0.5)
end = time.time() + 10
buf = b''
while time.time() < end:
    buf += ser.read(4096)
print(buf.decode(errors='replace'))
"
```

Expected output shape (exact hex values are what you're capturing — they will differ from
this example):

```
[IKS4A1 I3C PROBE] attempting ENTDAA against the shared bus (LSM6DSV16X is I3C-capable)
[IKS4A1 I3C PROBE] attempt 1: status=2 payload_hi=0x... payload_lo=0x...
[IKS4A1 I3C PROBE]   decoded PID: MIPIMID=0x0208 IDTSEL=0x00 PartID=0x7000 MIPIID=0x?? BCR=0x07 DCR=0x??
[IKS4A1 I3C PROBE] attempt 2: status=2 payload_hi=0x... payload_lo=0x...
[IKS4A1 I3C PROBE]   decoded PID: MIPIMID=0x???? IDTSEL=0x?? PartID=0x???? MIPIID=0x?? BCR=0x?? DCR=0x??
[IKS4A1 I3C PROBE] attempt 3: status=0 ...
```

You should see **exactly two distinct `MIPIID` values** across the `HAL_BUSY` attempts
(attempt 3 or later landing `status=0`/`HAL_OK` once both are assigned). One attempt's
`BCR=0x07` — that's the LSM6DSV16X (matches its datasheet's documented `GETBCR` value,
already confirmed in the 2026-07-09 bench session). The other is the ToF.

- [ ] **Step 5: Record the two `MIPIID` values**

Write down:
- `TOF_MIPI_ID` = the `MIPIID` byte from the attempt whose `BCR` is **not** `0x07`.
- `IKS4A1_LSM6DSV16X_MIPI_ID` = the `MIPIID` byte from the attempt whose `BCR` **is** `0x07`.

These feed directly into Task 2. If both attempts show the same `MIPIID` (shouldn't happen —
PIDs are per-device-instance unique per the MIPI I3C spec) or `BCR=0x07` doesn't appear at
all, stop and re-check the physical jumper state before proceeding (Global Constraints'
HUB1-only assumption may not hold).

- [ ] **Step 6: Set `CONF_IKS4A1_I3C_PROBE` back to `0`**

```c
#define CONF_IKS4A1_I3C_PROBE (0)
```

Leave the printf-extension code in place (still compiles out to nothing when disabled) —
it's a permanently useful diagnostic, matching `iks4a1_bus_probe()`'s own disabled-by-default
convention.

- [ ] **Step 7: Commit**

```bash
git add firmware/scanner-stream/Src/vl53l9_app.c
git commit -m "diag(host): decode ENTDAA PID.MIPIID in the I3C bus probe"
```

---

### Task 2: Add the multi-device ENTDAA assignment function

**Files:**
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c` (insert new code immediately before
  `static int rs_sensor_reinit(...)`, currently line 544)

**Interfaces:**
- Consumes: `TOF_MIPI_ID`, `IKS4A1_LSM6DSV16X_MIPI_ID` (measured in Task 1);
  `VL53L9_DEFAULT_ADDRESS` (`0x52`, already defined in `vl53l9.h:41`, already included);
  `hi3c1` (already `extern`'d at file scope, line 53); `HAL_I3C_Ctrl_DynAddrAssign`,
  `HAL_I3C_Get_ENTDAA_Payload_Info`, `HAL_I3C_Ctrl_SetDynAddr`, `HAL_I3C_Ctrl_ConfigBusDevices`,
  `__HAL_I3C_GET_BCR`, `__HAL_I3C_GET_IBI_CAPABLE`, `__HAL_I3C_GET_IBI_PAYLOAD`,
  `__HAL_I3C_GET_CR_CAPABLE` (all already used elsewhere in this file's probes, same
  patterns).
- Produces: `static int rs_assign_dynamic_addresses(void)` — returns `0` on success
  (both devices assigned and registered), non-zero on failure. Task 3 calls this in place of
  `platform_assign_dynamic_address()`.

- [ ] **Step 1: Insert the new constants and function**

In `firmware/scanner-stream/Src/vl53l9_app.c`, immediately before line 544
(`static int rs_sensor_reinit(vl53l9_device_t *p_dev, uint8_t *calib_data) {`), insert:

```c
/* ---- Multi-device I3C dynamic address assignment (IKS4A1 HUB1 native-I3C bus) ------
 *
 * Replaces platform_assign_dynamic_address() (platform_utils.c, read-only reference --
 * never edited in place per CLAUDE.md) for boards where the IKS4A1's LSM6DSV16X (HUB1)
 * shares I3C1 with the ToF as a genuine I3C target, not legacy I2C -- see
 * docs/iks4a1-stacking.md. The reference's single-device function hardcodes "whoever
 * answers ENTDAA first is the ToF, address 0x52" and only registers one device-table
 * entry; with two real I3C arbiters on the bus that either assigns the wrong device to
 * 0x52 or leaves the second device unmanaged, which is why the boot sequence hung with
 * both devices stacked (2026-07-09 bench session, see docs/iks4a1-stacking.md).
 *
 * Modeled on platform_utils.c's platform_assign_dynamic_address_multisensor() (same
 * ENTDAA/payload-decode/retry-on-BUSY shape), but matches PID.MIPIID against our own two
 * known devices instead of iterating the read-only device[] array (NB_DEVICES is fixed at
 * 1 in vl53l9_device.h, also read-only), and completes that reference function's own
 * noted "TODO: add HAL_I3C_Ctrl_ConfigBusDevices call" for both devices here instead.
 *
 * TOF_MIPI_ID / IKS4A1_LSM6DSV16X_MIPI_ID are measured hardware constants, captured via
 * the iks4a1_i3c_probe() diagnostic above with both the ToF and the IKS4A1 (HUB1-only
 * jumper config) stacked -- see docs/superpowers/plans/2026-07-09-iks4a1-hub1-multidevice-i3c.md
 * Task 1. Do not guess these; a wrong value makes rs_assign_dynamic_addresses() bail with
 * -2 (unknown device) rather than silently misconfigure the bus. */
#define TOF_MIPI_ID                (0x00 /* FILL IN: Task 1's non-BCR=0x07 MIPIID */)
#define IKS4A1_LSM6DSV16X_MIPI_ID  (0x00 /* FILL IN: Task 1's BCR=0x07 MIPIID */)
#define IKS4A1_LSM6DSV16X_I3C_ADDR (0x50) /* dynamic address to assign the LSM6DSV16X;
                                            * avoids 0x52 (ToF, VL53L9_DEFAULT_ADDRESS) and
                                            * every IKS4A1 static address (0x1E/0x38/0x5C/
                                            * 0x5D/0x6A/0x6B) per docs/iks4a1-stacking.md */

static int rs_assign_dynamic_addresses(void) {
    HAL_StatusTypeDef status;
    uint64_t payload;
    I3C_DeviceConfTypeDef dev_conf[2];
    uint8_t nb_configured = 0;

    hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x7c;
    hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x7c;
    hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x7c;
    if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
        return -1;
    }

    do {
        payload = 0;
        status = HAL_I3C_Ctrl_DynAddrAssign(&hi3c1, &payload, I3C_RSTDAA_THEN_ENTDAA, 5000);
        if (status == HAL_BUSY) {
            I3C_ENTDAAPayloadTypeDef pinfo = { 0 };
            HAL_I3C_Get_ENTDAA_Payload_Info(&hi3c1, payload, &pinfo);
            uint32_t bcr = __HAL_I3C_GET_BCR(payload);

            uint8_t address;
            if (pinfo.PID.MIPIID == TOF_MIPI_ID) {
                address = VL53L9_DEFAULT_ADDRESS;
            } else if (pinfo.PID.MIPIID == IKS4A1_LSM6DSV16X_MIPI_ID) {
                address = IKS4A1_LSM6DSV16X_I3C_ADDR;
            } else {
                return -2; /* unrecognized device answered ENTDAA -- bail rather than guess */
            }

            HAL_I3C_Ctrl_SetDynAddr(&hi3c1, address & 0x7F);

            if (nb_configured < 2) {
                dev_conf[nb_configured].DeviceIndex = (uint8_t)(nb_configured + 1);
                dev_conf[nb_configured].TargetDynamicAddr = address & 0x7F;
                dev_conf[nb_configured].IBIAck = __HAL_I3C_GET_IBI_CAPABLE(bcr);
                dev_conf[nb_configured].IBIPayload = __HAL_I3C_GET_IBI_PAYLOAD(bcr);
                dev_conf[nb_configured].CtrlRoleReqAck = __HAL_I3C_GET_CR_CAPABLE(bcr);
                dev_conf[nb_configured].CtrlStopTransfer = DISABLE;
                nb_configured++;
            }
        }
    } while (status == HAL_BUSY);

    if (status != HAL_OK) {
        return -3;
    }

    hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x0a;
    hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x09;
    hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x59;
    if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
        return -1;
    }

    if (nb_configured > 0 && HAL_I3C_Ctrl_ConfigBusDevices(&hi3c1, dev_conf, nb_configured) != HAL_OK) {
        return -4;
    }

    return 0;
}
```

- [ ] **Step 2: Fill in the two measured constants from Task 1**

Replace both `0x00 /* FILL IN: ... */` placeholders with the actual hex byte values recorded
in Task 1 Step 5. **Do not build with `0x00` left in place** — both constants would collide
(`TOF_MIPI_ID == IKS4A1_LSM6DSV16X_MIPI_ID == 0`), and depending on ENTDAA arbitration order
either device could silently take the wrong address.

- [ ] **Step 3: Build**

```sh
cmake --build build/Debug
```

Expected: clean build (this step only adds a new unused `static` function — expect one new
`-Wunused-function` warning for `rs_assign_dynamic_addresses`, same category as the
pre-existing `print_frame` one, harmless until Task 3 wires it in).

- [ ] **Step 4: Commit**

```bash
git add firmware/scanner-stream/Src/vl53l9_app.c
git commit -m "feat(host): add multi-device I3C ENTDAA assignment for ToF + IKS4A1 HUB1"
```

---

### Task 3: Wire the new function into the boot and recovery paths

**Files:**
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c:544-550` (`rs_sensor_reinit`)
- Modify: `firmware/scanner-stream/Src/vl53l9_app.c:675-680` (`rs_boot_bringup`)

(Line numbers are as of this plan being written — both call sites are easy to find by
searching for `platform_assign_dynamic_address();` and will have shifted down by the size
of Task 2's insertion; search rather than trusting the raw numbers.)

**Interfaces:**
- Consumes: `rs_assign_dynamic_addresses()` from Task 2 (returns `int`, `0` = success).
- Produces: nothing new — both functions keep their existing `int` return contracts
  (`rs_sensor_reinit` and `rs_boot_bringup` already propagate a non-zero `ret`/`boot_ret` to
  their own callers exactly the same way).

- [ ] **Step 1: Update `rs_sensor_reinit()`**

Find (inside `rs_sensor_reinit`, per the function comment above it starting "Full sensor
re-init cycle..."):

```c
    platform_power_reset(CONF_DEVICE_ID);
    if (p_dev->bus_type & PLATFORM_BUS_I3C) {
        platform_assign_dynamic_address();
    }

    ret = vl53l9_init(p_dev);
```

Replace with:

```c
    platform_power_reset(CONF_DEVICE_ID);
    if (p_dev->bus_type & PLATFORM_BUS_I3C) {
        int daa_ret = rs_assign_dynamic_addresses();
        if (daa_ret) {
            return daa_ret;
        }
    }

    ret = vl53l9_init(p_dev);
```

- [ ] **Step 2: Update `rs_boot_bringup()`**

Find (inside `rs_boot_bringup`):

```c
static int rs_boot_bringup(vl53l9_device_t *p_dev, uint8_t *out_calib_data, vl53l9_profile_t *p_profile) {
    platform_power_reset(CONF_DEVICE_ID);
    if (p_dev->bus_type & PLATFORM_BUS_I3C) {
        platform_assign_dynamic_address();
    }

    int ret = vl53l9_init(p_dev);
```

Replace with:

```c
static int rs_boot_bringup(vl53l9_device_t *p_dev, uint8_t *out_calib_data, vl53l9_profile_t *p_profile) {
    platform_power_reset(CONF_DEVICE_ID);
    if (p_dev->bus_type & PLATFORM_BUS_I3C) {
        int daa_ret = rs_assign_dynamic_addresses();
        if (daa_ret) {
            return daa_ret;
        }
    }

    int ret = vl53l9_init(p_dev);
```

- [ ] **Step 3: Build**

```sh
cmake --build build/Debug
```

Expected: clean build, no new warnings (the `rs_assign_dynamic_addresses` unused-function
warning from Task 2 Step 3 goes away now that it's called).

- [ ] **Step 4: Commit**

```bash
git add firmware/scanner-stream/Src/vl53l9_app.c
git commit -m "fix(host): use multi-device ENTDAA assignment in boot and recovery paths"
```

---

### Task 4: Verify full dual-device streaming on hardware

**Files:** none (verification only)

**Interfaces:**
- Consumes: `host/tools/capture.py` (existing tool, unchanged).

- [ ] **Step 1: Confirm physical stacking**

Both the 53L9A1 (ToF) and the IKS4A1 (jumpered to HUB1 only) physically stacked, per Global
Constraints.

- [ ] **Step 2: Flash the normal build**

```sh
STM32_Programmer_CLI -c port=SWD -w build/Debug/scanner_stream.bin 0x08000000 -rst
```

- [ ] **Step 3: Run the standard capture**

```sh
host/.venv/Scripts/python.exe host/tools/capture.py --reset --seconds 15 --out captures/iks4a1_hub1_dual_device.bin
```

Expected (this is the actual pass/fail gate for this whole plan):
- The native CDC port **reappears** within `capture.py`'s reset-wait window (it did **not**
  before this plan, per the 2026-07-09 bench session — `error: CDC port (VID cafe:PID 4001)
  did not reappear within 10.0s of reset`).
- `capture.py`'s report shows RAW/CALIB frames decoding, fps close to the established
  ~27.76 fps baseline (both the interval and wall-clock conventions — see
  `.claude/skills/firmware-loop/SKILL.md`'s "fps convention" section), 0 CRC failures, and
  no seq gaps beyond the known, already-documented connect-time transient
  (`docs/connect-transient-forensics.md`).

- [ ] **Step 4: If it fails**

Don't guess-and-retry. Re-run Task 1's probe (VCOM, 921600 baud, `CONF_IKS4A1_I3C_PROBE=1`)
with both devices stacked to see whether `rs_assign_dynamic_addresses()`'s own error paths
(`-1` HAL_I3C_Init failure, `-2` unrecognized device, `-3` final status not HAL_OK, `-4`
ConfigBusDevices failure) narrow down which stage broke — the function's return value is
now checked and propagated (Task 3), so `handle_error()`'s EVENT emission
(`RS_EVT_SENSOR_INIT_FAIL`) should also fire and be visible over CDC once it enumerates, if
it gets that far.

- [ ] **Step 5: Commit the capture as evidence (optional)**

```bash
git add captures/iks4a1_hub1_dual_device.bin
git commit -m "test(host): capture confirming ToF + IKS4A1 HUB1 dual-device I3C streaming"
```

(Skip this step if the project convention is not to commit capture binaries — check
`.gitignore` for `captures/` first.)

---

### Task 5: Update the docs to match reality (optional, recommended)

**Files:**
- Modify: `docs/iks4a1-stacking.md`
- Modify: `ROADMAP.md`

**Interfaces:** none — doc-only.

- [ ] **Step 1: Add a "Resolved: HUB1 native-I3C" section to `docs/iks4a1-stacking.md`**

Above the existing "Known conflict" / "Candidate workarounds" sections (added earlier in
the 2026-07-09 investigation), add a new section documenting: the HUB1-only jumper change
(J4/J5), the LSM6DSV16X's confirmed native I3C support (datasheet DS13510 sec 5.2, WHO_AM_I
0x70), the `rs_assign_dynamic_addresses()` fix from this plan, and the Task 4 verification
result (fps/CRC/seq-gap numbers actually observed). Note explicitly that the environmental
sensors (LPS22DF, LIS2MDL, STTS22H, SHT40) are no longer reachable via the shared bus in
this configuration (HUB1-only routing disconnects them) — reading them now requires the
LSM6DSV16X's own I2C sensor-hub feature (a separate, not-yet-implemented driver task,
out of this plan's scope).

- [ ] **Step 2: Update `ROADMAP.md` Phase 4's "Bus topology" bullet**

Replace the "bench-tested, shared-I3C1 approach blocked" bullet (from the earlier
2026-07-09 update) with the resolved HUB1 native-I3C approach and a pointer to this plan
and `docs/iks4a1-stacking.md`'s new section.

- [ ] **Step 3: Commit**

```bash
git add docs/iks4a1-stacking.md ROADMAP.md
git commit -m "docs: capture the HUB1 native-I3C bus fix (resolves the shared-bus conflict)"
```

---

## Self-Review Notes

- **Spec coverage:** Task 1 produces the measured constants Task 2 needs (no guessing);
  Task 2 completes the reference's noted TODO in fork-owned code (read-only rule honored);
  Task 3 wires it into both live call sites and fixes the pre-existing unchecked-return-value
  gap; Task 4 is the actual pass/fail gate against the original failure; Task 5 closes the
  loop on the documentation this session already started correcting.
- **Placeholder scan:** the two `0x00 /* FILL IN */` constants in Task 2 are the one
  deliberate exception to "no placeholders" — they are empirically measured hardware values
  that cannot be known without running Task 1 first; Task 2 Step 2 explicitly gates the
  build on filling them in and explains why leaving them at `0x00` is unsafe (address
  collision), which is the honest way to represent a hardware-measured constant in a plan
  written before that measurement exists.
- **Type consistency:** `rs_assign_dynamic_addresses(void) -> int` is defined once (Task 2)
  and called with that exact signature in both Task 3 call sites; `I3C_DeviceConfTypeDef`,
  `I3C_ENTDAAPayloadTypeDef`, `HAL_I3C_Ctrl_DynAddrAssign`, etc. are all used with the exact
  signatures already verified against `stm32h5xx_hal_i3c.h` and the working patterns already
  in this file's own probes.
