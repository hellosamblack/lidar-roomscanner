# High Frame-Rate Ranging Profiles & Manual Sensor Control

## Status

Specification for implementation. Builds upon Phase 3 runtime configuration (`docs/protocol.md`), Phase 4 sensor suite (`docs/iks4a1-stacking.md`), and Phase 5 Ethernet transport (`CLAUDE.md`).

> **Planning review, 2026-07-31:** The preset power/range anchors do not reproduce the
> equations below, the current driver exposes integer-millisecond—not 0.5 ms—exposure,
> and the manual command payload does not fit protocol v1. Treat those values and wire
> details as provisional until Task 1/2 of the reviewed
> [implementation plan](../plans/2026-07-31-high-framerate-and-manual-ranging-modes.md)
> reconciles them against the datasheet and hardware. The feature is not implemented.

> **Task 1 reconciliation, 2026-08-03:** Sections 2.1, 3.2, 3.3, and 5.2 below are
> **amended** against DS14879 rev 6 (`references/datasheets/NUCLEO-VL53L9CX/datasheet.pdf`)
> — the numbers in this document now match, exactly or to a documented tolerance, the
> canonical implementation in `host/src/roomscan/profiles.py` (tests:
> `host/tests/test_profiles.py`). Two items remain genuinely open and are marked
> **PENDING hardware measurement** where they occur: whether a sub-millisecond exposure
> setter is achievable at all (Task 4/5), and the true minimum schedulable blanking
> margin between exposure and frame period (Task 5). Protocol/codec work (Task 2) is not
> blocked by either. The measurement baseline (§8, new) is a 117 s live Ethernet capture
> at the current (pre-feature) firmware's fixed 30 fps profile.

---

## 1. Overview & Objectives

This feature expands the scanner's operating profiles from the existing two presets (`AR_RANGE` / Room Mapping @ 30 FPS and `AR_PRECISION` / Precision @ 30 FPS) to a 4-mode architecture:

1. **Room Mapping (Default / Preset 1):** `AR_RANGE` — Ambient mode with DSS, 30 FPS, 6 ms exposure, ULP power mode (8m max range, ~200 mW est., 28.5% I3C bus duty cycle).
2. **Precision Ranging (Preset 2):** `AR_PRECISION` — Precision mode, 30 FPS, 10 ms exposure, ULP power mode (8.8m max range, 5cm min distance, ~267 mW est., 28.5% I3C bus duty cycle).
3. **High Frame-Rate / Gaming (Preset 3):** `HIGH_FRAMERATE` — Precision mode (no DSS), 90 FPS, 4 ms exposure, Regular power mode (5m max range, ~415 mW est., 85.5% I3C bus duty cycle). Optimized for low-latency SLAM tracking. (Power figures amended 2026-08-03 — see §3.2/§8.)
4. **Manual / Custom Mode (Mode 4):** Live interactive controls letting operators set ranging mode, frame rate (1–100 FPS), exposure time (1–16 ms), and power mode, while displaying live computed consequences (Max Range, Power Consumption, and I3C Bus Utilization).

Additionally, a visual **I3C Bus Bandwidth Bar** is integrated directly into the Web UI control card beneath the mode selector, rendering real-time bus duty cycle and warning when airtime approaches bus saturation or USB transport ceilings.

---

## 2. Technical Specification & Profile Definitions

### 2.1 Profile Table

**Amended 2026-08-03** (Task 1): the Precision and High Frame-Rate "Typical Power"
cells below are recomputed from the reconciled model in §3.2 — they are no longer the
values a first draft asserted without deriving. Room Mapping's is unchanged because it
*is* the real DS14879 anchor the model was fit to reproduce. Manual's exposure step is
1 ms, not 0.5 ms (§7.1's exposure-granularity verdict — the current driver has no proven
sub-millisecond path).

| Mode Name | Key / Identifier | Ranging Mode | DSS | FPS (Target) | Exposure | Power Mode | Max Range (Est) | Min Distance | Typical Power (Est) | I3C Bus Utilization |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Room Mapping** | `room_mapping` | Ambient | Yes | 30 FPS | 6 ms | ULP | 8.0 m | 450 mm | 200 mW | 28.5% |
| **Precision** | `precision` | Precision | Yes | 30 FPS | 10 ms | ULP | 8.8 m | 50 mm | ~267 mW | 28.5% |
| **High Frame-Rate** | `high_framerate` | Precision | No | 90 FPS | 4 ms | Regular | 5.0 m | 50 mm | ~415 mW | 85.5% |
| **Manual** | `manual` | *Custom* | *Custom* | 1–100 FPS | 1–16 ms (1 ms step) | *Custom* | *Dynamic* | 50 mm / 450 mm | *Dynamic* | *Dynamic* |

Every cell above except "Typical Power" and "Max Range" was already correct in the
first draft; only those two columns' *equations* were wrong (§3.2/§3.3 explain why, with
the datasheet citations). Precision's power rose from an unfounded 220 mW guess to
~267 mW because no anchor at all previously existed for that exact (ranging mode, power
mode, resolution) combination — see §3.2's derivation for where the ~267 mW figure comes
from and how confident to be in it. High Frame-Rate's power moved from 420 mW (silently
borrowed from DS14879's own *100 fps* Gaming example) to ~415 mW, computed at the fps
this profile actually runs (90, not 100) — the two differ by only 1.2%, which is the
external check that the reconciled model is doing something reasonable.

---

## 3. Mathematical Models for Real-Time Consequence Estimation

**Amended 2026-08-03 (Task 1).** These models are now implemented once, in
`host/src/roomscan/profiles.py` — this section is the prose explanation of that
module, not an independent formula the UI reimplements; the module's own docstring
carries the full derivation with every intermediate number. In **Manual Mode**,
changes to FPS, exposure, ranging mode, and power mode still trigger the same
real-time client/server estimation this section originally proposed; only the
*models themselves* changed.

### 3.1 I3C Bus Airtime Model

Unchanged from the first draft except a units-notation fix. The I3C bus operates at
12.5 MHz Push-Pull (SDR). For a raw 3DMD frame (binning = 2, \(14{,}842\text{ bytes} =
118{,}736\text{ bits}\)):

$$T_{\text{xfer}} = \frac{118{,}736\text{ bits}}{12.5 \times 10^6\text{ bits/s}} = 9.49888\text{ ms}$$

$$\text{Bus Utilization (\%)} = \frac{T_{\text{xfer,ms}}}{\text{Frame Period}_{\text{ms}}} \times 100 = T_{\text{xfer,ms}} \times \text{FPS} / 10$$

(the first draft wrote this as "\(9.49888\text{ ms} \times \text{FPS} \times 100\%\)",
which is dimensionally wrong as literally read — dividing by the 1000 ms/s→ms/ms-period
conversion was silently folded into the "×100%"; the *resulting* percentages below were
already correct, only the written formula was not). This is the sensor's own I3C link
only ("ToF bus airtime" — Task 10's UI label), never the Ethernet/USB transport link §4.1
paces.

* **At 30 FPS:** \(9.5\text{ ms} / 33.3\text{ ms} = 28.5\%\) duty cycle (\(71.5\%\) idle airtime for IMU).
* **At 60 FPS:** \(9.5\text{ ms} / 16.7\text{ ms} = 57.0\%\) duty cycle (\(43.0\%\) idle airtime for IMU).
* **At 90 FPS:** \(9.5\text{ ms} / 11.1\text{ ms} = 85.5\%\) duty cycle (\(14.5\%\) idle airtime for IMU).
* **At 100 FPS:** \(9.5\text{ ms} / 10.0\text{ ms} = 95.0\%\) duty cycle (\(5.0\%\) idle airtime for IMU).

Implementation: `roomscan.profiles.i3c_bus_utilization_pct`.

### 3.2 Power Consumption Model

**Replaced.** The first draft's `P = P_baseline + P_laser(Exposure) × FPS + P_mode`
does not reproduce its own preset table (`50 + 5×6×30 = 950`, not 200 mW) and has no
datasheet citation for its coefficients. The reconciled model is duty-cycle-based and
traceable to two real DS14879 rev 6 two-point fits plus one real single-point anchor:

$$\text{duty} = \min\!\left(1, \frac{\text{exposure}_{\text{ms}} \times \text{FPS}}{1000}\right)
\qquad P_{\text{total}} = P_{\text{intercept}}(\text{ranging mode, power mode}) +
P_{\text{slope}}(\text{ranging mode}) \times \text{duty}$$

`duty` is the fraction of the frame period the laser spends integrating — the
physically meaningful quantity; the first draft's `Exposure × FPS` term had units of
ms/s and grew without bound.

**The two real fits** (DS14879 Table 36 "Power consumption", **indoor / 0 W/m² column
only** — see the caveat below for why the outdoor columns are excluded — cross-referenced
with Table 21 "Profile settings" for the exposure each row was measured at, all 54×42 /
binning 2):

| Ranging mode | Power mode | Point 1 | Point 2 | Slope (mW/duty) | Intercept (mW) |
| :--- | :--- | :--- | :--- | ---: | ---: |
| Precision | Regular | duty 0.12 → 235 mW (4 ms/30 fps, indoor) | duty 0.30 → 370 mW (10 ms/30 fps, outdoor cloudy) | 750.0 | 145.0 |
| Ambient | Regular | duty 0.12 → 225 mW (4 ms/30 fps, indoor) | duty 0.48 → 560 mW (16 ms/30 fps, outdoor sunny, gray 17%) | 930.6 | 113.3 |

**Caveat, carried forward rather than hidden:** DS14879's own test matrix never varies
exposure at *fixed* ambient light — a longer exposure is how the vendor's own profiles
compensate for brighter ambient conditions. So the Ambient/Regular pair's two points also
change ambient light (indoor → outdoor sunny) alongside exposure, and its fitted slope
folds in some real photon-current increase from brighter light, not pure laser-duty cost.
It is an upper bound on the duty term, not a clean separation — no cleaner number exists
in the public datasheet.

**ULP and LP intercepts are not directly measured at 54×42** (Table 36's only ULP row is
24×20). They are derived:

* **ULP, Ambient** — solved *exactly* from Table 9 "Profile examples"' own Room Mapping
  anchor (30 fps, 6 ms, ULP → 200 mW, duty 0.18) against the Ambient/Regular slope above:
  \(200 = \text{intercept} + 930.6 \times 0.18 \Rightarrow \text{intercept} = 32.5\text{ mW}\).
  This is a real anchor, not an assumption.
* **ULP, Precision** — no matching 54×42/ULP/Precision anchor exists anywhere in DS14879.
  Scaled from the Regular/Precision intercept (145.0 mW) by the ULP/Regular intercept
  ratio measured on the Ambient pair (32.5/113.3 ≈ 0.287) — i.e. *assuming* ULP's
  idle-current saving is ranging-mode-independent (DS14879 §3.2 "Power modes": "Low power
  and ultralow power modes disable circuitry in between frames," stated for both modes
  without distinction) → ≈41.6 mW. Documented assumption, not a table lookup.
* **LP (Low Power)** — the only LP row in either table (Table 9's "AR glasses", outdoor
  sunny / 100 klx) is confounded by ambient light the same way as the Ambient/Regular pair
  *and* is a different power mode, so it cannot anchor an intercept on its own. LP
  intercepts are the midpoint of the ULP and Regular intercepts for each ranging mode —
  the **weakest-grounded numbers in this model**, and flagged as such in
  `profiles.POWER_COEFFICIENTS`.

**External validation** (a point this fit was *not* built from): High Frame-Rate
(Precision/Regular/DSS-off, our 90 fps preset — not DS14879's 100 fps Gaming example) at
duty 0.36 predicts \(145.0 + 750.0 \times 0.36 = 415.0\text{ mW}\), against Table 9's
Gaming anchor of 420 mW at duty 0.40 (100 fps) — **1.2% off**, despite running at a
different fps and with DSS forced off (the fitted Precision slope came from DSS-on rows).
That is the strongest external check available and it passes.

Implementation: `roomscan.profiles.estimate_power_mw` / `POWER_COEFFICIENTS`. Values
remain labelled **estimated** everywhere they surface — no measured claim without a power
meter (Global constraint, unchanged).

### 3.3 Max Range Model

**Replaced**, not just re-fit. The first draft's `R_max = 8.5m × √(Exposure / 6.0ms)`
(Ambient) / `5.0m × √(Exposure / 4.0ms)` (Precision) does not reproduce its own preset
table (≈7.9 m at 10 ms, not the stated 8.8 m) — and, more fundamentally, **DS14879's own
range tables cannot support any continuous exposure→range formula**, because every row
that changes exposure also changes the ambient-light test condition (Tables 23/24 pair
"4 ms exposure / indoor" against "10 or 16 ms exposure / outdoor"). The tables even show
range *decreasing* with more exposure at the less-reflective (gray 17%) target, because
the brighter ambient light that motivated the longer exposure hurts SNR more than the
extra integration time helps. The host also has no ambient-light sensor to condition on.
A continuous formula here is therefore false precision, not a simplification error to fix
by re-deriving better coefficients.

The replacement is a categorical lookup on **(ranging mode, DSS enabled)** — exactly the
distinction DS14879's own profile examples make — using the conservative (gray 17%)
indoor figure:

| Ranging mode | DSS | Max range | Source |
| :--- | :--- | ---: | :--- |
| Ambient | On | 8.0 m | Table 9 Room Mapping anchor (ULP) — Table 24's Regular-mode figure is 8.5 m; ULP is the profile actually in force |
| Precision | On | 8.8 m | DS14879's own stated ranging ceiling ("up to 8.8 m"); also Table 23 gray 62% at both tested exposures |
| Precision | Off (>60 fps, forced) | 5.0 m | Table 9 Gaming anchor, exact |

Minimum distance is unchanged and exact — DS14879 Table 22 "Minimum ranging
capabilities", not exposure-dependent: **450 mm** ambient, **50 mm** precision.

Implementation: `roomscan.profiles.estimate_max_range_m` / `MAX_RANGE_M`,
`estimate_min_distance_mm` / `MIN_DISTANCE_MM`.

---

## 4. Hardware, Transport, and SLAM Safety Guards

1. **Transport Throughput Ceiling Guard (USB vs Ethernet):**
   * Raw payload bandwidth at \(\text{FPS}\) is \(14,842 \times \text{FPS}\) bytes/sec.
   * At \(\text{FPS} > 60\), bandwidth exceeds USB CDC Full-Speed's \(\sim 1.0\text{ MB/s}\) throughput cap.
   * The UI detects active transport (`CDC` vs `UDP`). If `CDC` is active and FPS is set \(> 60\), the control card displays a prominent warning: *"High frame rate (> 60 FPS) requires Ethernet UDP transport to prevent frame drops."*

2. **Host SLAM Parameter Auto-Scaling:**
   * **Barometer Drift Window (`baro_tau_frames`):** Scaled automatically with target frame rate:
     $$\text{baro\_tau\_frames} = \text{round}(30.0 \times \text{target\_fps})$$
     (e.g., 900 frames @ 30 FPS, 2700 frames @ 90 FPS).
   * **Host IMU Crossover Rate (`QUAT_REF_RATE_HZ`):** Dynamically set to match active ToF frame rate in `ImuFusion`.

3. **Web UI Pacing (`POINT_INTERVAL`):**
   * Broadcaster WebSocket pacing in `web.py` auto-adapts to incoming frame rate or decouples UI viewport rendering from frame ingest, ensuring 90 FPS stream processing while maintaining smooth browser rendering.

---

## 5. Control Protocol & Software Architecture

### 5.1 Binary & WebSocket Protocol Updates
Extends existing `COMMAND_CODE` set in `docs/protocol.md` and `/ws` JSON control channel:

* **Command Code 8: `SET_RANGING_PROFILE`** — Enum: `0=ROOM_MAPPING`, `1=PRECISION`, `2=HIGH_FRAMERATE`, `3=MANUAL`.
* **Command Code 9: `SET_MANUAL_PARAMS`** — Parameters: `ranging_mode (u8)`, `frame_period_us (u32)`, `exposure_ms (u16)`, `power_mode (u8)`.

WebSocket `/ws` JSON messages:
```json
{
  "type": "set_profile",
  "profile": "high_framerate",
  "manual_params": {
    "ranging_mode": "precision",
    "fps": 90,
    "exposure_ms": 4,
    "power_mode": "regular"
  }
}
```

### 5.2 Web UI Layout & Bus Bandwidth Bar

1. **Mode Selector:**
   * Segmented control with 4 options: `[ Room Mapping | Precision | High FPS | Manual ]`.
2. **Manual Adjustment Panel (visible when `Manual` is selected):**
   * **Ranging Mode Dropdown:** `Precision (min 5cm)` / `Ambient (min 45cm)`.
   * **FPS Slider / Input:** Range 1 to 100 FPS (steps of 1 FPS).
   * **Exposure Slider / Input:** Range 1 to 16 ms, **steps of 1 ms** (amended
     2026-08-03 — `vl53l9_set_exposure()` takes an integer millisecond parameter; see
     §7.1's exposure-granularity verdict for why 0.5 ms is not shipped).
   * **Power Mode Dropdown:** `Ultra Low Power (ULP)` / `Low Power (LP)` / `Regular`.
3. **Consequences Metrics Display:**
   * Stat readouts: **Est. Max Range** (e.g., `5.0 m`), **Est. Power** (e.g., `~415 mW`), **Min Distance** (e.g., `50 mm`).
4. **I3C Bus Bandwidth Bar (Located directly beneath mode selector):**
   * Visual progress bar displaying total bus duty cycle (0% to 100%).
   * **Color Coding:**
     * `Green` (0% – 70%): Plenty of airtime reserved for IMU/sensors.
     * `Yellow` (70% – 85%): High utilization, IMU airtime constrained.
     * `Red` (> 85%): Near saturation / bus capacity limit.
   * Sub-caption label: `I3C Bus: 85.5% used (9.5ms ToF / 11.1ms frame) • 14.5% airtime left for IMU`.

---

## 6. Implementation Plan & Milestones

1. **Firmware (`firmware/scanner-stream`):**
   * Add `HIGH_FRAMERATE` profile definition and manual override setters for `frame_period_us`, `exposure_ms`, `ranging_mode`, and `power_mode` in `vl53l9_app.c`.
   * Plumb Command Codes 8 & 9 into the CDC/UDP command parser.
2. **Host Library (`roomscan`):**
   * Update `imufusion.py` and `slam/mapper.py` to auto-scale time constants (`QUAT_REF_RATE_HZ` and `baro_tau_frames`) with measured frame rate.
   * Add consequence estimation helper functions (`roomscan.profiles` — shipped Task 1,
     2026-08-03).
3. **Web Server & UI (`roomscan-web`):**
   * Plumb WebSocket commands for `set_profile` and `set_manual_params`.
   * Implement 4-way Mode Selector card, Manual Parameter controls, and real-time I3C Bus Bandwidth Bar in `static/components/controls.js` and CSS.
4. **Validation & Verification:**
   * Hardware test over Ethernet UDP @ 90 FPS (verify 0 frame drops, 0 CRC errors, clean IMU sync).
   * Verify transport ceiling warning when switching to >60 FPS over CDC.

---

## 7. Notes & Key Discoveries for Plan Writer

### 7.1 Hardware & Driver Constraints (ST VL53L9CX Datasheet DS14879)
* **Driver API Flexibility:** `vl53l9_set_frame_period()` accepts any frame period from \(10,000\ \mu\text{s}\) (100 FPS) to \(1,000,000\ \mu\text{s}\) (1 FPS). You do NOT have to choose only discrete values (30 or 100); 60 FPS or 90 FPS are fully valid continuous settings.
* **Precision vs. Ambient Minimum Distance:** 
  * **Precision Mode** allows ranging down to **50 mm (5 cm)**.
  * **Ambient Mode** enforces a minimum distance floor of **450 mm (45 cm)**.
  * *For room scanning, Precision mode's 5 cm floor is essential for scanning near objects.*
* **DSS Cutoff Frame Rate (\(\mathbf{60\text{ FPS}}\)):**
  * **Dynamic SPAD Selection (DSS)** is supported only up to **60 FPS** (frame periods \(\ge 16.7\text{ ms}\)).
  * At frame rates above 60 FPS (61–100 FPS), the on-chip sensor MCU does not have sufficient inter-frame time to run SPAD selection algorithms. The sensor **must** run in **Precision Mode with DSS disabled** (fixed SPAD array map).
* **Onboard Transform CPU Bottleneck:** If `CONF_TRANSFORM_ONBOARD == 1` is enabled, the Cortex-M33 CPU math takes \(\sim 37\text{ ms/frame}\) (\(\sim 27\text{ FPS}\) max). High frame rates (>30 FPS) **MUST** run in **Raw Streaming Mode** (`CONF_TRANSFORM_ONBOARD == 0`), which offloads math to the host.
* **Exposure granularity verdict (amended 2026-08-03, Task 1).** The original ask was a
  0.5 ms exposure step. `vl53l9_set_exposure(void*, vl53l9_context_t, uint16_t
  exposure_ms)` (`firmware/vendor/53L9A1/Drivers/BSP/Components/vl53l9/vl53l9.c:536`) —
  the only public entry point the vendor app uses — takes an **integer-millisecond**
  parameter and validates `1 <= exposure_ms <= 30`. Internally it computes
  `shots_base = (500000 * exposure_ms) / blank_sum` (integer arithmetic) and writes
  per-step shot counts to `VL53L9_REGADDR_STREAM_NB_SHOT_STEP` through the **public**
  `vl53l9_write()` register-I/O function — the same pattern the DSS extension (finding
  #3) already uses for a local, non-vendor-edited setter. That means a scanner-owned
  low-level setter accepting a *fractional* exposure (replicating this math in float
  instead of integer ms, still through public register I/O, never touching
  `firmware/vendor/`) is **theoretically possible**. It is **NOT proven**: proving it
  needs new scanner-stream firmware code (Task 4) and an on-target timing measurement
  confirming the resulting shot-count register value actually changes integration time
  at sub-ms resolution rather than being masked by rounding or a hardware minimum step
  (Task 5). Both are outside Task 1's host-only scope. **Verdict: ship a truthful 1 ms
  step now** (`roomscan.profiles.EXPOSURE_STEP_MS`); sub-ms is **PENDING hardware
  verification**, tracked for Task 4/5, not silently dropped or promised.
* **Blanking margin — PENDING hardware measurement (Task 5).** DS14879 documents no
  minimum frame-period-minus-exposure margin, and `vl53l9_set_frame_period()`
  (`vl53l9.c:397`) validates only the absolute 10,000–1,000,000 µs range with no
  cross-check against exposure — that check is entirely the application's
  responsibility. The true minimum schedulable margin (how much internal FSM/
  housekeeping time the sensor needs between finishing one frame's ranging and being
  ready for the next) is an empirical quantity, not derivable from the public register
  math. Task 1 ships a conservative, clearly-labelled placeholder
  (`roomscan.profiles.BLANKING_MARGIN_US_PENDING_HW = 500` µs) that rejects only
  exposure that plainly cannot fit inside the requested period — **this is not a
  measured minimum and must not be read as hardware-verified.** Task 5 measures the
  real value on-target (sweep frame_period downward at fixed exposure until FRAME_READY
  timing/status degrades) and replaces the placeholder.

### 7.2 Bus Bandwidth & IMU Dynamics
* **I3C 12.5 MHz Bus Utilization:** Transferring a raw 3DMD frame (14,842 bytes) takes \(\approx 9.5\text{ ms}\).
  * 30 FPS = 28.5% bus utilization (71.5% airtime for IMU).
  * 60 FPS = 57.0% bus utilization (43.0% airtime for IMU).
  * 90 FPS = 85.5% bus utilization (14.5% airtime for IMU).
  * 100 FPS = 95.0% bus utilization (5.0% airtime for IMU — near total saturation).
* **IMU Sampling Rate (LSM6DSV16X):** The IMU samples continuously at **480 Hz** into an onboard hardware FIFO. Do **NOT** attempt to change the IMU ODR when increasing LiDAR FPS. At 90 FPS ToF, the MCU simply drains smaller FIFO batches (\(\approx 5\text{--}6\) samples per readout instead of \(\approx 16\)), which actually **cuts readout latency 3×**.

### 7.3 Transport Ceilings (USB CDC FS vs. Ethernet UDP)
* **USB CDC Full-Speed Ceiling:** Payload rate at 90 FPS is \(\sim 1.34\text{ MB/s}\) (\(10.7\text{ Mbps}\)). USB CDC Full-Speed PHY tops out at \(\sim 1.0\text{ MB/s}\) (\(\approx 67\text{ FPS}\) max). Running 90 FPS over USB CDC will choke TinyUSB endpoints and drop frames heavily.
* **Ethernet UDP Requirement:** Ethernet 100 Mbps handles \(1.34\text{ MB/s}\) easily (\(\sim 10.7\%\) of link bandwidth). High FPS testing **must be conducted over Ethernet UDP**.

### 7.4 Host Processing & SLAM Performance
* **GPU Compute Margin:** Live CUDA SLAM (`SlamRunner` / Open3D VoxelBlockGrid ICP) processes frames in **\(\sim 7\text{ ms}\)** on an RTX 2000 Ada GPU, which easily fits within 90 FPS's \(11.1\text{ ms}\) budget.
* **Barometer Window Scaling:** `Mapper.baro_tau_frames` defaults to 900 frames (\(30\text{ seconds}\) @ 30 FPS). The plan writer must ensure `baro_tau_frames` is dynamically scaled (`round(30.0 * target_fps)`) to prevent barometer drift correction from becoming 3× over-aggressive at 90 FPS.
* **Complementary Filter Retuning:** `imufusion.py`'s `QUAT_REF_RATE_HZ` constant should be updated dynamically to the active ToF frame rate to preserve filter phase alignment.

### 7.5 Repository Protocol & Development Conventions
* **Vendor Directory is Read-Only:** All new firmware changes go in `firmware/scanner-stream/`, NOT `firmware/vendor/53L9A1/`.
* **Wire Protocol Changes:** Modifying command codes requires following the `protocol-change` skill checklist (`docs/protocol.md`, firmware C, host Python, golden vectors).
* **MCP Integration:** Any host-side consequence helper functions added to `host/src/roomscan` must be registered/exposed in `host/src/roomscan/mcp_server/`.
* **Shipping Checklist:** Mandate running the `status-sync` skill before landing the completed implementation.

---

## 8. Measurement Baseline (Task 1, 2026-08-03)

The non-regression comparator Tasks 4–6 compare against, recorded via `rig_record()`
against the live rig on Ethernet/UDP, current (pre-feature) firmware — fixed manual-sync
30 fps, DSS on, Ambient/`AR_RANGE`-equivalent profile — analyzed with the new
`host/tools/profile_probe.py` (MCP: `capture_profile_probe`):

* **Capture:** `captures/web_20260803_121735.bin`, 117.36 s (longer than the plan's 60 s
  minimum — real elapsed time between the record-start and record-stop calls, not a
  deliberate extension).
* **RAW rate:** 3548 RAW_3DMD frames, measured **30.222 fps** against a 30 fps target
  (+0.74%, within the plan's own ±2% tolerance). Interval median/p05/p95 **33.0 / 32.997 /
  33.004 ms** — i.e. essentially jitter-free at today's fixed rate.
* **CRC / gaps:** **0 CRC failures**, 0 bytes skipped. RAW seq span 3555, received 3548,
  **7 missing (0.197%)** — small, non-zero loss exists even on a clean link and is the
  kind of number later tasks' gates need to beat, not assume away.
* **Per-stream pairing** (today's coupled 1:1 assumption): IMU_QUAT 100.0%, ENV 100.0%,
  IMU_SYNC 100.0%, IMU_RAW 100.0% paired against RAW_3DMD's seq.
* **Stream-11 effective rate:** 54,712 timestamp samples over the capture → **466.2 Hz**
  against the 480 Hz XL/GY/SFLP ODR ceiling (97.1% of nominal — consistent with the
  known ~3% oscillator trim documented for stream 12, not a new finding).
* **Link bytes/s, transform latency, TX queue health:** not captured by this tool (out of
  a pure-capture-file analyzer's reach) or by `rig_status()` at record time (`metrics` was
  `null` in the status snapshot taken alongside this recording); Task 4–6's own
  hardware-loop work should pull these from the live server's `metrics`/`session`
  broadcasts at record time, not reconstruct them after the fact.

**Tool validation** — `profile_probe.probe()` was additionally run against three
pre-existing captures to check it degrades gracefully rather than only ever seeing clean
data:

* `recordings/2026-07-08-room-scan.bin` (a v1-era capture predating streams 9–13
  entirely): 731 RAW frames, 11.25 fps measured (an old fixed-period profile), 0 CRC
  failures, all stream-pairing rows correctly report `0/0 (0.0%)` rather than raising.
* `captures/DebugCapB2.bin` (a capture with known real loss — see `ROADMAP.md` 6.D/6.G
  history): 4790/5004 RAW frames, measured 28.988 fps against a 30 fps request
  (**-3.37%, correctly flagged `fps_within_tolerance: false`**), 214 seq gaps (4.277%
  loss) — matches the capture's known-bad reputation rather than papering over it.
* `captures/web_20260730_175304.bin`: measured 30.258 fps against a naively-assumed
  100 fps request — correctly flagged out of tolerance (**-69.74%**) rather than trusting
  the filename/assumption; this capture's stream 13 is entirely absent (0 IMU_SYNC
  frames), and pairing correctly reports `0.0%` for it without crashing.
