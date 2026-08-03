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
>
> **Measured-ceiling amendment, 2026-08-03 (Task 5's stop point, plan §"Global
> constraints": *"do not label a lower measured rate as 90 FPS; fix the bottleneck or
> amend the specification with the measured ceiling"*):** Task 5's on-target sweep found
> the sensor has an intrinsic per-frame floor DS14879 does not document, and that any
> requested period shorter than it is silently delivered as an integer MULTIPLE of the
> request rather than rejected or clamped — a 90 Hz request measured 44.85 fps (a clean
> 2×), 100 Hz measured 33.2 fps (a clean 3×). Every "90 FPS" claim below is retired: the
> **High Frame-Rate preset is amended to 46 Hz** (Precision, 4 ms exposure, Regular
> power, DSS **on** — 46 ≤ the 60 Hz DSS ceiling), the measured 1× ceiling at this
> preset's own exposure. See §2.1/§3.2/§3.3 for the updated numbers and §8 for a summary
> of the measurement; the full sweep data lives in the implementation plan's Task 11/12
> and `ROADMAP.md`. **Manual mode is not re-capped** — a request above its exposure's 1×
> ceiling remains accepted by the sensor (1–100 fps, unchanged) — but the model
> (`roomscan.profiles`) now warns rather than silently mispredicting, and reports the
> honest **expected delivered rate**, not the request, once a candidate crosses that
> ceiling (§3.2 note, new `expected_delivered_fps`).

---

## 1. Overview & Objectives

This feature expands the scanner's operating profiles from the existing two presets (`AR_RANGE` / Room Mapping @ 30 FPS and `AR_PRECISION` / Precision @ 30 FPS) to a 4-mode architecture:

1. **Room Mapping (Default / Preset 1):** `AR_RANGE` — Ambient mode with DSS, 30 FPS, 6 ms exposure, ULP power mode (8m max range, ~200 mW est., 28.5% I3C bus duty cycle).
2. **Precision Ranging (Preset 2):** `AR_PRECISION` — Precision mode, 30 FPS, 10 ms exposure, ULP power mode (8.8m max range, 5cm min distance, ~267 mW est., 28.5% I3C bus duty cycle).
3. **High Frame-Rate / Gaming (Preset 3):** `HIGH_FRAMERATE` — Precision mode, DSS **on**, **46 FPS**, 4 ms exposure, Regular power mode (8.8 m max range, ~283 mW est., 43.7% I3C bus duty cycle). Optimized for low-latency SLAM tracking within the sensor's measured 1× delivery ceiling. **Amended 2026-08-03** from an original 90 FPS/DSS-off design that does not exist on real hardware — see §2.1/§3.2/§8 and the "Measured-ceiling amendment" note above.
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

**Amended again 2026-08-03** (Task 5, measured-ceiling): High Frame-Rate's FPS, DSS,
Max Range, Power, and I3C Bus Utilization cells all changed. Task 5's on-target sweep
found the sensor cannot actually deliver 90 FPS — that request is silently delivered as
44.85 fps, a clean 2× period-multiple, not a rate this UI may ever claim as "90 FPS" per
the plan's own gate. The preset now runs at **46 FPS**, the measured 1× delivery ceiling
at 4 ms exposure (see §8), with DSS **on** (46 ≤ the 60 Hz ceiling — the existing DSS
rule is unchanged, just now applicable because the FPS moved under it), which also
raises Max Range from the DSS-off 5.0 m to the DSS-on 8.8 m figure (same cell Precision
uses — §3.3).

| Mode Name | Key / Identifier | Ranging Mode | DSS | FPS (Target) | Exposure | Power Mode | Max Range (Est) | Min Distance | Typical Power (Est) | I3C Bus Utilization |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Room Mapping** | `room_mapping` | Ambient | Yes | 30 FPS | 6 ms | ULP | 8.0 m | 450 mm | ~208 mW | 35.6% |
| **Precision** | `precision` | Precision | Yes | 30 FPS | 10 ms | ULP | 8.8 m | 50 mm | ~256 mW | 35.6% |
| **High Frame-Rate** | `high_framerate` | Precision | **Yes** | **46 FPS** | 4 ms | Regular | **8.8 m** | 50 mm | **~274 mW** | **54.6%** |
| **Manual** | `manual` | *Custom* | *Custom* | 1–100 FPS | 1–16 ms (1 ms step) | *Custom* | *Dynamic* | 50 mm / 450 mm | *Dynamic* | *Dynamic* |

**Amended 2026-08-03 (I3C effective rate + power model, same-day refinement, see
§3.1/§3.2 below):** the I3C Bus Utilization column moved from the raw-12.5-MHz-clock
figures (28.5/28.5/43.7%) to the documented **effective 10 Mbps** figures
(35.6/35.6/54.6%) — two independent sources (AN6522 Table 5, the decompiled
`ProfileTuning.exe` planning tool) agree the achievable I3C throughput is 10 Mbps, not
the raw SDR clock rate. The Typical Power column moved from the DS14879-two-point-fit
model to ST's own decompiled `ProfileTuning.exe` equations (exact per-vendor
coefficients, not a fit), validated 5/5 against owner-run tool readings within 0.01% —
see §3.2's replacement subsection. Both changes are implementation refinements to an
already-shipped Task 1, not new findings that alter which profile is used where.

Every cell above except "Typical Power" and "Max Range" was already correct in the
first draft; only those two columns' *equations* were wrong (§3.2/§3.3 explain why, with
the datasheet citations). The power figures went through a further revision on
2026-08-03, superseding the paragraph that used to sit here: the DS14879-fit model
(Precision ~267 mW, High Frame-Rate ~283 mW via a duty-vs-420 mW/415 mW Gaming-anchor
comparison) was **replaced outright** by ST's own decompiled `ProfileTuning.exe`
equations (§3.2's new subsection), which are exact per-vendor coefficients rather than a
fit and were validated 5/5 against owner-run tool readings within 0.01%. Under the
replacement model: Room Mapping ~208 mW, Precision ~256 mW, High Frame-Rate ~274 mW (all
at `AMBIENT_LUX_DEFAULT`, ProfileTuning's own "Home, Theatres - 100 Lux" indoor
reference — a genuine ambient-light input the old fit had no way to represent). These no
longer target the old DS14879 Table 9/36 rows the way the retired fit did by
construction — e.g. Room Mapping's ~208 mW vs DS14879's own 200 mW "(5 klx)" footnote
differ because ProfileTuning's default ambient (100 lux) is not DS14879's 5000 lux test
condition for that row, not because either figure is wrong.

Manual requests above a given exposure's measured 1× ceiling remain **accepted** by the
sensor (unchanged — this was never a rejectable condition, and the sensor does not
reject it either), but now come with a **warning** and an honest **expected delivered
fps** distinct from the requested fps — see §3.2's new subsection and §8.

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

**Amended 2026-08-03 (same-day refinement, decompiled ProfileTuning.exe + AN6522
investigation):** the units-notation fix from the first draft stands, but the
underlying coefficient was ALSO wrong, not just the formula's notation. The original
9.49888 ms figure was derived from the raw 12.5 MHz I3C SDR clock — but that is not
the achievable transfer rate; per-byte protocol overhead brings the *effective*
throughput to a documented **10 Mbps**, corroborated by two independent sources: AN6522
Table 5 "Readout duration depending on the readout interface" states, in the vendor's
own words, "54x42 → I3C (10 Mbps): 11.8 ms"; and the decompiled `ProfileTuning.exe`
planning tool computes the identical quantity as `frame_size_bytes * 8 * 1000 / 10e6`.
For a raw 3DMD frame (binning = 2, \(14{,}842\text{ bytes} = 118{,}736\text{ bits}\)):

$$T_{\text{xfer}} = \frac{118{,}736\text{ bits}}{10 \times 10^6\text{ bits/s}} = 11.8736\text{ ms}$$

$$\text{Bus Utilization (\%)} = \frac{T_{\text{xfer,ms}}}{\text{Frame Period}_{\text{ms}}} \times 100 = T_{\text{xfer,ms}} \times \text{FPS} / 10$$

The formula itself is unchanged from the prior amendment (only the coefficient moved,
9.49888 → 11.8736 ms); this is the sensor's own I3C link only ("ToF bus airtime" —
Task 10's UI label), never the Ethernet/USB transport link §4.1 paces.

* **At 30 FPS:** \(11.87\text{ ms} / 33.3\text{ ms} \approx 35.6\%\) duty cycle (\(64.4\%\) idle airtime for IMU).
* **At 46 FPS (the amended High Frame-Rate preset):** \(11.87\text{ ms} / 21.7\text{ ms} \approx 54.6\%\) duty cycle (\(45.4\%\) idle airtime for IMU).
* **At 60 FPS:** \(11.87\text{ ms} / 16.7\text{ ms} \approx 71.2\%\) duty cycle (\(28.8\%\) idle airtime for IMU).
* **At 90 FPS:** \(11.87\text{ ms} / 11.1\text{ ms} \approx 106.9\%\) — the raw transfer alone no longer fits inside the requested period at all; the model clamps to **100%**, not "near saturation."
* **At 100 FPS:** \(11.87\text{ ms} / 10.0\text{ ms} \approx 118.7\%\) — same clamp, **100%**.

The 90/100 FPS rows above describe the *configured period* the bus schedule is asked to
hit — they are still correct as a bus-airtime model, now honestly reporting the bus as
fully saturated rather than merely 85.5%/95.0% busy — but per §8, a request in that range
does not actually get delivered at that rate (§3.2.1's quantization model); no preset
uses them any more, and a Manual request there will carry the §3.2.1 delivery-ceiling
warning alongside this bus-airtime figure.

Implementation: `roomscan.profiles.i3c_bus_utilization_pct` / `I3C_XFER_MS`
(`host/tests/test_profiles.py`'s "I3C bus airtime" section pins the corrected
coefficient and derives its expected percentages from it, rather than hand-computed
literals, so a future re-tune doesn't silently drift the test out of sync with the
constant it is meant to guard).

### 3.2 Power Consumption Model

**Note (2026-08-03): this whole model was replaced again**, by §3.2.2 below — the
DS14879-two-point-fit described in the rest of this §3.2 is kept as the historical
record of the *first* replacement (of the original draft's non-reproducing formula) but
is no longer what `roomscan.profiles.estimate_power_mw` computes. Read §3.2.2 for the
current model.

**Replaced (original, 2026-08-03 Task 1).** The first draft's `P = P_baseline + P_laser(Exposure) × FPS + P_mode`
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

**External validation** (a point this fit was *not* built from): Precision/Regular/
DSS-off at 90 fps/4 ms (the shape of DS14879's own 100 fps Gaming example, **not** the
High Frame-Rate preset — amended 2026-08-03, see below) at duty 0.36 predicts
\(145.0 + 750.0 \times 0.36 = 415.0\text{ mW}\), against Table 9's Gaming anchor of
420 mW at duty 0.40 (100 fps) — **1.2% off**, despite running at a different fps and
with DSS forced off (the fitted Precision slope came from DSS-on rows). That is the
strongest external check available and it passes; it validates the *model*, not any
one preset.

Implementation: `roomscan.profiles.estimate_power_mw` / `POWER_COEFFICIENTS`. Values
remain labelled **estimated** everywhere they surface — no measured claim without a power
meter (Global constraint, unchanged).

#### 3.2.1 Measured hardware ceiling & delivery quantization (amended 2026-08-03)

Task 5's on-target sweep (readback-exact `frame_period_us`, 54×42/binning 2, Precision
context, Regular power) found the sensor has an intrinsic per-frame floor DS14879 does
not document. A requested period shorter than the floor is **accepted**, not rejected or
clamped, and is delivered as an **integer multiple** of the request: a 90 Hz request
measured 44.85 fps (a clean 2×), 100 Hz measured 33.2 fps (a clean 3×). The floor
brackets by exposure (bracket upper bound — the conservative, under-promising side,
since the true floor lies somewhere inside it):

| Exposure | Measured floor bracket | Model uses (conservative) |
| :--- | :--- | ---: |
| 1–2 ms | (16.667, 20.0] ms | 20.0 ms |
| 4 ms | (20.833, 21.739] ms | 21.739 ms |
| 8 ms | (22.222, 23.529] ms | 23.529 ms |

Sub-linear in exposure (~0.5–0.7 ms of floor per ms of exposure), with a fixed
~16–17 ms component dominating. DSS on vs off made no measurable difference to the
floor — DSS is free, and the existing ≤60 Hz-on/>60 Hz-off rule is unchanged. DS14879's
own Table 9 "Gaming" anchor (54×42/Precision/100 fps/4 ms/Regular) does **not** reproduce
at its own stated configuration (2.2× shortfall), and Table 21's characterization matrix
only ever exercises 30 fps — nothing in the public datasheet actually validates a request
above ~46 fps at this resolution. The highest rate actually delivered anywhere in the
investigation was ~49.3 fps (from a 99 Hz request); the 90–100 Hz request band also
showed a reconfig-instability anomaly (BUG-073), a second reason not to park a preset
there.

Model (`roomscan.profiles`, measured-2026-08-03):

$$\text{expected\_delivered\_fps} = \frac{\text{requested\_fps}}{\left\lceil \dfrac{\text{floor\_ms}(\text{exposure})}{\text{period\_ms}} \right\rceil}$$

`validate_manual_params` now emits a **warning** (never a rejection — the sensor accepts
these requests) whenever a manual candidate's fps exceeds its exposure's measured 1×
ceiling, and `ProfileEstimate.expected_delivered_fps` reports the honest expected rate.
UI/MCP consequence readouts must display `expected_delivered_fps`, not echo the raw fps
request, for any configuration past this ceiling. Exposure above 8 ms is unmeasured.

**Amended 2026-08-03 (floor extrapolation, same-day refinement):** above 8 ms exposure,
`measured_floor_ms` no longer flat-holds the 8 ms bracket's 23.529 ms — it uses a
**derived** line from the same decompiled-ProfileTuning.exe/AN6522 investigation as
§3.1's I3C coefficient: `floor_ms(exposure_ms) = 1.6 ms (FW dead time, ProfileTuning's
own planning model) + 11.8736 ms (I3C readout, AN6522/ProfileTuning) + exposure_ms +
~3 ms margin (the line's largest residual against the three measured brackets) ≈
16.5 + exposure_ms`. This is clearly labelled DERIVED, not measured — Task 5's sweep
never ran above 8 ms — but it is a better-justified extrapolation than an unexplained
flat hold, and the same investigation showed *why* the tool's own equation for this
quantity (a flat 26.9 ms at 54×42/DSS-on, independent of exposure — see §3.2.3) diverges
from measured hardware there: the three measured brackets below 8 ms remain
authoritative in that range precisely because they are measurements, and measurement
outranks the tool's own formula wherever both exist for the same quantity. Above 8 ms,
no measurement exists to outrank the derived line, so `measured_floor_ms` returns it.
Implementation: `roomscan.profiles.measured_floor_ms` / `FLOOR_FW_DEADTIME_MS` /
`FLOOR_EXTRAPOLATION_MARGIN_MS`.

#### 3.2.2 Power model replacement: decompiled ProfileTuning.exe (2026-08-03)

The §3.2 DS14879-two-point-fit model above is **replaced**, not re-tuned, by ST's own
equations extracted from `references/software/53L9A1/ProfileTuning.exe` (support-gated,
kept locally untracked) — its `MainWindow.update()` computes exact
AVDD/DVDD/IOVDD/VBAT_Rx/VBAT_Tx terms from small per-(ranging_mode, power_mode)
coefficient tables, not a fit to sparse datasheet rows, and includes an **ambient-light
input** (`ambient_lux`) the fitted model had no way to represent at all. Extracted with
`pyinstxtractor-ng` + `decompyle3` (PyInstaller/Python 3.7 payload).

**Validation, 2026-08-03** — five owner-run ProfileTuning.exe readings (54×42, I3C, DSS
Enable, ambient = "Home, Theatres - 100 Lux" unless noted), reproduced by the extracted
equations to within **0.01%**, an order of magnitude inside the ~2% bar this replacement
was gated on:

| Config | Tool reading | Model |
| :--- | ---: | ---: |
| Ambient, ULP, 6 ms/30 fps | 208.4 mW | 208.41 mW |
| Precision, ULP, 10 ms/30 fps | 255.7 mW | 255.72 mW |
| Precision, Regular, 4 ms/37 fps | 243.7 mW | 243.73 mW |
| Ambient, Regular, 2 ms/37 fps | 210.5 mW | 210.48 mW |
| Ambient, Regular, 4 ms/37 fps | 260.0 mW | 260.01 mW |

Because this model is exact-per-ST rather than a fit, it no longer targets the two real
DS14879 Table 36/Table 9 anchors the retired fit was built from as exactly as that fit
did by construction (e.g. Room Mapping now estimates ~208 mW where DS14879 states 200 mW
"(5 klx)" — ProfileTuning's own default ambient, 100 lux, is not DS14879's 5000 lux
footnoted test condition for that row). `ProfileEstimate.power_mw`'s field name and
semantics ("estimated typical power, mW, not a measured claim") are unchanged, so
`roomscan-web`/UI consequence readouts pick up the new numbers with no call-site
changes. Implementation: `roomscan.profiles.estimate_power_mw` / `AMBIENT_LUX_DEFAULT`.

#### 3.2.3 DSS does not affect frame size or achievable frame rate (2026-08-03)

A short but important clarification the measured-ceiling investigation (§3.2.1/§8)
turned up: **DSS (Dynamic SPAD Selection) is a ranging-*quality* control, not a frame-
size or frame-rate control, on this firmware.** Our vendored driver
(`firmware/vendor/53L9A1/Drivers/BSP/Components/vl53l9/vl53l9.c`) fetches the DSS LUT
**unconditionally** — the source comment reads, verbatim, "in current implementation,
the dss is always enabled so no check needed" (present at both call sites, lines 681 and
789) — so toggling our own `STANDBY_DSS_MODE` register write changes SPAD selection
behavior but has **no effect** on the transmitted frame size or the achievable frame
rate over our I3C transport.

This resolves a question the §8 investigation left open: ST's own `ProfileTuning.exe`
and DS14879's Gaming anchor advertise 100+ fps figures under a "DSS Disable" setting,
and those never reproduced on our hardware. The reason is now clear from the decompiled
tool itself — "DSS Disable" in ProfileTuning switches the frame payload to a **106-byte
status-only** output (`frame_size = 106` regardless of resolution, vs. 14,742 bytes at
54×42 with DSS on), a mode the vendored I3C driver does not implement and which is
incompatible with the 14,842-byte 3DMD transform pipeline this project streams. The
100+ fps figures describe that different, unimplemented mode, not an achievable rate for
our 3DMD frames.

**Owner-run `ProfileTuning.exe` sweep, 2026-08-03** (54×42, I3C, Regular power, ambient
= "Home, Theatres - 100 Lux"), confirms both halves of this:

* **DSS Enable:** max fps is **flat at 37.2 fps across 2–8 ms exposure** — because at
  54×42 the tool's own DSS "housekeeping" window (13.5 ms) dominates its
  `max(exposure_ms×1.15+2.0, dss_duration_ms)` term for any exposure below ~10 ms, so
  frame duration (and therefore max fps) does not move with exposure in that range. This
  is the same "flat 26.9 ms" model §3.2.1 refers to (dead time 1.6 ms + I3C readout
  ~11.79 ms + 13.5 ms DSS window ≈ 26.9 ms → 1000/26.9 ≈ 37.2 fps) — and it is **slower**
  than our measured hardware (20.0–23.5 ms floors, i.e. faster max fps) at the same
  DSS-on/54×42 operating point, corroborating §3.2.1's finding that measurement outranks
  the tool's own equation for this quantity.
* **DSS Disable (106-byte status-only mode):** max fps **169.9 at 2 ms exposure** / **122.2
  at 4 ms exposure** — an order of magnitude higher, because the 106-byte payload's
  readout time is negligible (~0.08 ms) and the 13.5 ms DSS window no longer applies.
  Effective data rate at this frame size is ~9.5 kB/s at a 90 fps operating point
  (`106 bytes × 90 fps / 1000 ≈ 9.5 kB/s`) — a fraction of the 3DMD frame's own
  ~445.3 kB/s at the same rate (`14,842 bytes × 30 fps / 1000`, scaled), underscoring
  that this is a fundamentally smaller payload, not the same frame delivered faster.

Consequence: no change to any preset or to the ≤60 Hz-DSS-on/>60 Hz-DSS-off rule — DSS
was already correctly modelled as "free" (no measurable floor difference, §3.2.1). This
section exists to close out *why* the datasheet's headline frame-rate figures never
reproduced, so a future reader does not re-open that question assuming a driver bug.

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
   * Raw payload bandwidth at \(\text{FPS}\) is \(14,842 \times \text{FPS}\) bytes/sec — this is the
     *requested* rate; per §3.2.1/§8, a request above its exposure's measured 1× ceiling is not
     actually delivered at that rate, so real bytes/sec for such a request is lower by the same
     integer multiple that quantizes the fps. The warning below fires on the *request*
     regardless — a >60 FPS request is still an unsupported ask over CDC even though its real
     traffic may be smaller than the naive bandwidth figure suggests.
   * At \(\text{FPS} > 60\), bandwidth exceeds USB CDC Full-Speed's \(\sim 1.0\text{ MB/s}\) throughput cap.
   * The UI detects active transport (`CDC` vs `UDP`). If `CDC` is active and FPS is set \(> 60\), the control card displays a prominent warning: *"High frame rate (> 60 FPS) requires Ethernet UDP transport to prevent frame drops."* This rule is unchanged by the measured-ceiling amendment — Manual can still request up to 100 FPS, and the CDC warning still applies to the request.

2. **Host SLAM Parameter Auto-Scaling:**
   * **Barometer Drift Window (`baro_tau_frames`):** Scaled automatically with target frame rate:
     $$\text{baro\_tau\_frames} = \text{round}(30.0 \times \text{target\_fps})$$
     (e.g., 900 frames @ 30 FPS, 1380 frames @ the amended 46 FPS High Frame-Rate preset —
     the formula takes whatever `target_fps` actually is, including a Manual request above
     46; it is not specific to the retired 90 FPS design point).
   * **Host IMU Crossover Rate (`QUAT_REF_RATE_HZ`):** Dynamically set to match active ToF frame rate in `ImuFusion`.

3. **Web UI Pacing (`POINT_INTERVAL`):**
   * Broadcaster WebSocket pacing in `web.py` auto-adapts to incoming frame rate or decouples UI viewport rendering from frame ingest, ensuring smooth browser rendering up to the sensor's actual measured delivery rate (≤~49 fps at this resolution, per §8.1 — not the originally-assumed 90 FPS).

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
   * Hardware test over Ethernet UDP @ **46 FPS** (the amended High Frame-Rate preset — verify
     0 frame drops, 0 CRC errors, clean IMU sync, and that the applied period readback matches
     21,739 µs and the measured delivered rate is ≈46 fps, not merely that the ACK echoed the
     request). A 90 FPS hardware test is retired — see §8 for why the request itself was never
     deliverable at 1×.
   * Verify transport ceiling warning when switching to >60 FPS over CDC.

---

## 7. Notes & Key Discoveries for Plan Writer

> **Read alongside §3.2.1/§8 (2026-08-03).** The 90/100 FPS figures throughout §7.1–7.3
> below describe what the driver API *accepts* and what a *requested* configured period
> implies for bus/transport load — both still literally true. What they do **not** mean,
> as the original draft implied, is that the sensor *delivers* frames at that rate:
> Task 5 measured 90/100 Hz requests being silently quantized to ~44.85/~33.2 fps. Treat
> every "90 FPS"/"100 FPS" mention below as a *request*, not an achieved rate.

### 7.1 Hardware & Driver Constraints (ST VL53L9CX Datasheet DS14879)
* **Driver API Flexibility:** `vl53l9_set_frame_period()` accepts any frame period from \(10,000\ \mu\text{s}\) (100 FPS) to \(1,000,000\ \mu\text{s}\) (1 FPS). You do NOT have to choose only discrete values (30 or 100); 60 FPS or 90 FPS are fully valid continuous *settings to request* — but per §3.2.1/§8, "valid" here means "accepted without error," not "delivered 1:1." Above ~46 FPS (exposure-dependent) the sensor delivers an integer-multiple-quantized rate instead.
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

### 8.1 Measured ceiling investigation (Task 5, 2026-08-03) — summary

Full sweep data lives in the implementation plan's Task 11/12 records and
`ROADMAP.md`; this is the brief citation the amended §2.1/§3.2.1 numbers trace to, per
the plan's own gate ("do not label a lower measured rate as 90 FPS; fix the bottleneck
or amend the specification with the measured ceiling").

* **Method:** on-target sweep of `frame_period_us` (readback-exact — the applied value
  read back from the sensor, not merely the ACK'd request), 54×42/binning 2, Precision
  context, Regular power, at fixed exposures of 1, 2, 4, and 8 ms.
* **Finding:** any requested period shorter than an intrinsic per-frame floor is
  **accepted**, not rejected, and delivers an **integer multiple** of the requested
  period instead of the request: 90 Hz → 44.85 fps measured (clean 2×); 100 Hz →
  33.2 fps measured (clean 3×).
* **Floor brackets by exposure** (measured, bracket upper bound is what the model
  uses — conservative/under-promising): 1–2 ms → floor ∈ (16.667, 20.0] ms (50 Hz
  clean 1×, 60 Hz fell to 2×); 4 ms → floor ∈ (20.833, 21.739] ms (46 Hz clean 1× at
  45.84 fps, 48 Hz fell to 2×); 8 ms → floor ∈ (22.222, 23.529] ms. Sub-linear in
  exposure (~0.5–0.7 ms of floor per ms of exposure); a fixed ~16–17 ms component
  dominates.
* **DSS on vs off:** statistically indistinguishable floors — DSS is free; the
  existing ≤60 Hz-on/>60 Hz-off rule is unchanged by this finding.
* **DS14879 does not validate this:** Table 9's "Gaming" anchor (54×42/Precision/
  100 fps/4 ms/Regular) does not reproduce at its own stated configuration (2.2×
  shortfall), and Table 21's characterization matrix only ever exercises 30 fps —
  nothing in the public datasheet actually validates a request above ~46 fps at this
  resolution.
* **Highest rate ever delivered:** ~49.3 fps, from a 99 Hz request. The 90–100 Hz
  request band also showed a reconfig-instability anomaly (BUG-073) — a second,
  independent reason not to park a preset there.
* **Consequence:** the High Frame-Rate preset is amended 90 → **46 Hz** (§2.1), the
  measured 1× ceiling at its own 4 ms exposure, with DSS now on (46 ≤ 60). Manual mode
  is not re-capped — 1–100 fps requests remain accepted — but `roomscan.profiles` now
  warns and reports `expected_delivered_fps` (§3.2.1) instead of silently mispredicting
  a rate the sensor will not actually deliver.
