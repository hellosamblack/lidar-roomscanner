# VL53L9CX FoV / thermal datasheet notes

Phase 2.5 Task 1. Source PDFs (all under `references/datasheets/VL53L9CX/`):

- **`fov.pdf`** = ST application note **AN5894 rev 2** ("Description of the fields of view of
  STMicroelectronics' Time-of-Flight sensors"), 14 pages. **Generic across ST's ToF lineup** — it
  never mentions the VL53L9CX by name. It defines terminology (system FoV, field of illumination,
  nominal FoV, exclusion cone/zone, keep-out cone, detection volume) and, critically, the
  **diagonal-FoV computation rule** (Section 2.3): for a rectangular/square multizone FoV, the
  diagonal is **not** `sqrt(H² + V²)` (Pythagoras only applies to distances, not angles) — it must
  be computed via `DFoV = 2·asin( sqrt(2)·sin(HFoV/2) / sqrt(tan²(HFoV/2)+1) )`-style geometry
  (AN5894 Eq. 1–4, p.5). Example given: VL53L7CX at 60°×60° side gives **90° diagonal**, not the
  84.5° Pythagoras would suggest.
- **`datasheet.pdf`** = **DS14879 rev 6** (June 2026), the actual VL53L9CX datasheet. This is
  where the concrete numbers live (not one of the three PDFs named in the task brief, but it was
  already present in the folder and is the only source with device-specific FoV angles — read it
  to get numbers AN5894 deliberately omits).
- **`thermals.pdf`** = ST technical note **TN1579 rev 3** ("Thermal guidelines when using the
  VL53L9CX"), 8 pages — PCB/junction thermal-resistance design guide, not a depth-accuracy note.
- **`x-nucleo-datasheet.pdf`** = X-NUCLEO-53L9A1 data brief — one-page marketing/feature summary
  for the eval board.

## FoV numbers (DS14879)

| Quantity | Value | Source |
|---|---|---|
| Horizontal FoV | **55°** | DS14879 p.1 (Features bullet: "55°x42° (71° diagonal) FoV"); Table 3 "FoV angles", p.5; Figure 26, p.38 |
| Vertical FoV | **42°** | same three citations |
| Diagonal FoV | 71° | DS14879 p.1 feature bullet (per AN5894's non-Pythagorean diagonal formula — 71° is *given*, not independently re-derived here) |
| Rx (collector) exclusion-zone FoV | 62° (H) × 48° (V) | DS14879 Table 3, p.5 — larger envelope for cover-glass/aperture design, not the ranging FoV; not used for deprojection |

Table 3 (p.5) pairs cleanly with the Features-page bullet and the Figure 26 outline-drawing
callouts ("55°FOV" / "42°" labels) — three independent citations agree, so confidence is high.
`pdftotext -layout` scrambles this table's columns; verified against the non-layout extraction
(`pdftotext` without `-layout`), which lists it unambiguously as `Horizontal 55° / 62°`,
`Vertical 42° / 48°` for the `Field of view` / `Collector exclusion zone` rows respectively.

**Resolution basis:** DS14879 §2 states multizone ranging goes "up to 54 x 42 zones" (§3.3, p.9,
lists 54×42 as the Binning-2 / full-resolution output) with "a wide 55°x42° FoV, which can be
reduced by software" (p.2) — i.e. the 55°×42° figure is the FoV of the *full* 54×42 array; ROI/DSS
reduction narrows the effective FoV below this, not above it. Since our hard requirement is full
54×42 resolution (no ROI reduction), 55°×42° is the correct total-FoV figure to use.

## Per-zone angular pitch (derived, not published directly)

Simple linear division of total FoV by zone count at full (Binning 2) resolution:

- Horizontal: 55° / 54 zones = **1.0185°/zone**
- Vertical: 42° / 42 zones = **1.0000°/zone** (exact)

The vertical pitch coming out to *exactly* 1.0°/zone is a good sign that the datasheet's 42° V-FoV
is indeed meant edge-to-edge across the full 42-row array — consistent with the existing
`Deprojector` zone-center convention (`((idx + 0.5)/N - 0.5) * fov_deg`), which places zone centers
at half-pitch offsets from the array edges. This is circumstantial, not a datasheet statement (see
"does NOT answer" below).

## Optical vs. per-zone-center specification — NOT explicitly stated

Neither AN5894 nor DS14879 says in words whether the 55°/42° figures are "zone-center-to-zone-
center" or "outer-edge-to-outer-edge" of the full array. AN5894's generic definition ("nominal
FoV... angles are given for the X and Y orientations of the module", p.7) describes an overall
receiver angular envelope, which reads as edge-to-edge (i.e., the angular span the whole SPAD array
covers), not zone-center-to-zone-center. The `Deprojector`'s existing linear model already treats
the configured `fov_h_deg`/`fov_v_deg` as the edge-to-edge span and places zone centers at half-
pitch insets from those edges — this is the standard convention for this sensor family and is kept
unchanged; only the numeric defaults are updated here.

## Distortion / non-linearity — model exists, constants do not

DS14879 §8.3 "ROI description" (p.30) gives an explicit per-zone depth-correction formula:

```
Dperp(t,i,j) = D(t,i,j) / sqrt( ((i-ic)² + (j-jc)²)/f² · (1+α·((i-ic)²+(j-jc)²)) + 1 )
```

where `f` = focal length (pixels), `α` = distortion coefficient (pix⁻²), and `(ic, jc)` = optical
center coordinates. **This confirms the sensor has real per-zone angular/lens distortion** (the
FoV is not perfectly linear across zones) — but DS14879 does **not** publish numeric values for
`f`, `α`, `ic`, or `jc` anywhere in the document. These are almost certainly per-device calibration
constants (stored in the unit's NVM/`calib_data`, the same blob the firmware already reads via
`vl53l9_init`/`calib_data` in `<APP>/Src/vl53l9_app.c`), not published constants. This is exactly
why Task 2/3 of this plan derive an empirical ZAPC-based correction from the actual device instead
of trying to reverse-engineer `f`/`α`/`ic`/`jc` from the datasheet — there is nothing here to
reverse-engineer from.

## Thermal / depth-accuracy-vs-temperature (one paragraph each, per brief)

- **`thermals.pdf` (TN1579 rev 3):** Pure PCB/system thermal-design note — junction-temperature
  budgeting (`Tj_max` = 105°C, max ambient 70°C), thermal-resistance tables (`Rth` targets 30–190
  °C/W depending on profile/ambient, Table 2 p.5), and layout guidance (copper pour, thermal vias,
  keep-out from other heat sources). It says nothing about ranging/depth accuracy vs. temperature —
  that's in the main datasheet instead (see next line). Relevant later for Phase 5 mechanical/PCB
  integration, not for the deprojection model.
- **`datasheet.pdf` (DS14879) §8.4.2 "Ranging accuracy", p.32:** *"Self-heating or a change in
  ambient temperature increases silicon temperature. This can result in an offset drift of maximum
  ±0.1 mm/°C drift."* This is the one depth-accuracy-vs-temperature figure that exists in these
  documents. It's a per-degree **range offset** drift bound, unrelated to FoV/angle — noted here
  for Phase 5 (thermal compensation belongs at the sensor-fusion tier per the edge-ai-tooling
  memory), not acted on in this task.
- **`x-nucleo-datasheet.pdf`:** One-page eval-board data brief. Feature bullet reads "Wide 54 x 42°
  FoV" (p.1) — this conflates the 54-zone horizontal *resolution* with the 55° horizontal *angle*
  (should read "55° x 42° FoV"; likely a copy-paste/rounding slip in ST's marketing brief, since
  DS14879's own Table 3 and Figure 26 both independently say 55°, not 54°). No thermal or
  distortion content beyond the thermal-pad pinout diagram already covered by TN1579. Not used as a
  numeric source — DS14879 (Table 3 + Figure 26, two independent diagram citations) is treated as
  authoritative over this single marketing bullet.

## What the datasheets do NOT answer

- No explicit statement of edge-to-edge vs. zone-center-to-zone-center FoV convention (inferred
  from the exact 1.0°/zone vertical pitch, not stated outright).
- No published values for the Dperp distortion model's `f`, `α`, `ic`, `jc` — these are
  per-device/calibration-specific and not in any of these four documents.
- No FoV-vs-binning table — only the qualitative "FoV... can be reduced by software" (via ROI/DSS),
  with no numbers for anything other than full 54×42.
- No FoV tolerance/variance across units (the AN5894 "system FoV" concept implies unit-to-unit
  spread exists, but no percentage or degree tolerance is given anywhere).
- No depth-accuracy-vs-temperature curve — only the single ±0.1 mm/°C bound above; no breakdown by
  distance, profile, or FoV position (center vs. edge zones might drift differently; not stated).

## Disposition

Concrete H/V numbers exist (55°/42°, two independent citations each in DS14879 plus AN5894's
general methodology for how such numbers are defined) and are used to replace the `Deprojector`
placeholder defaults (60.0°/45.0°) in this commit. Per the task brief's fallback clause, because
the optical-vs-zone-center convention is inferred rather than stated, the new defaults are marked
in code as **"datasheet-derived, ZAPC validation pending in Task 3"** — Task 3 will empirically
check the linear tan model (with these defaults) against `ZAPC`-derived per-zone angles from a real
device and either confirm or refine.
