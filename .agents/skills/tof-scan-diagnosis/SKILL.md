---
name: tof-scan-diagnosis
description: Use when a capture's SLAM/ToF reconstruction looks wrong — warped, wrong-sized, tilted, or forked into a second room — and you want to quantify WHERE and WHY against ground truth. Diagnoses a lidar scan by diffing it against the best-matching imported ground-truth splat (reverse Phase 7). Triggers on "why is this scan wrong", "compare against the splat/Scaniverse", "did we miss the room shape", "diagnose the reconstruction".
---

# Diagnosing a ToF scan against ground truth

Phase 7 normally uses the splat to improve the lidar. This runs it **backwards**: a metric
Scaniverse (or other externally-captured) splat is treated as ground truth, and our SLAM
reconstruction is measured against it to find where the scanner got the room *shape* wrong.
The engine is the `splat_compare` MCP tool (`roomscan.splat.compare`); this skill is how to
drive it and read the result.

> **The `BUG-NNN` failure classes below are legacy IDs** (pre-2026-08-10, when defects moved to GitHub
> Issues and the title prefixes were stripped). Resolve one to its `#NNN` via
> `docs/issue-migration-map.md` — `gh issue list --search "BUG-084"` only matches the `Legacy ID:`
> line in the body.

## When to use

A capture's Detailed/live SLAM map looks off — dimensions wrong, walls skewed, floor/ceiling
tilted, or it forked into a second displaced room (the BUG-084 class) — and you want a number
and a picture instead of a hunch.

## Procedure

### 1. Make sure the scan exists
`splat_compare` diffs the capture's **Detailed-SLAM mesh** `results/<stem>.ply`. If it is
missing, build it first: `slam_rerender(capture=...)` (or Detailed SLAM in the web UI). No mesh
→ the tool returns `no SLAM reconstruction for <stem>`.

### 2. Choose the ground-truth reference (do NOT hardcode one)
Ground truth = an **imported** splat (external capture, e.g. Scaniverse) — check `splat_list()`
and look for `imported: true`. Our own video builds (`imported: false`) are photometry-only and
too rough to be truth; never use them as the reference.

- **Default:** call `splat_compare(capture=...)` with `reference` empty. It auto-selects the best
  imported reference for this capture by **scene-name match** (`officeFullScanAug6` ↔ "Sam Office"
  → match on `office`), falling back to the newest import if no name overlaps. The choice and the
  ranked candidates are echoed in `report.reference_selection` — **read it and sanity-check** the
  chosen splat is actually the same room.
- **Override** when the auto-choice is wrong or ambiguous: pass `reference=<slug>` (e.g.
  `scaniverse-splat`) or an explicit `.ply` path.
- **No imported reference for that room** → you cannot diagnose against truth. Say so; offer to
  import one (below).

### 3. Run it
`splat_compare(capture="<stem>")`. It is a subprocess, open3d, **~7–8 min** (dominated by loading
the ~600 MB splat) — expect that, don't treat it as hung. Artifacts land in
`results/compare/<stem>__vs__<ref>/`.

### 4. Read the report (numbers)
- `alignment.fitness` / `inlier_rmse` — a **rigid, room-cropped** fit. Low fitness / high rmse is
  the shape error itself, not a tool bug (both clouds are metric, so we do **not** fit scale away).
  ~0.5 fitness means only half the scan rigidly matches truth.
- `extent_obb_m.ratio_scan_over_ref` — per-axis (scan ÷ truth). **< 1 = the scan under-sized the
  room** (drift shrinks it); which axis is worst localizes the problem. (Office example: 0.77 / 0.80
  / 0.68 — the room is really ~8×7×4.7 m, we reconstructed ~6.2×5.7×3.2 m.)
- `footprint_m2` — floor area captured vs truth.
- `distance_m.{scan_to_reference,reference_to_scan}` — bidirectional nearest-point error.
  `scan_to_reference` = how wrong our geometry is; `reference_to_scan` = truth we missed. Watch
  `p95` and `frac_over_10cm`.
- `vertical.fork_suspected` + `scan_height_modes` vs `ref_height_modes` — a healthy room has two
  height peaks (floor + ceiling); an **extra** scan mode = a dropped-ceiling/second-room fork
  (BUG-084). `extent_ratio` > 1.3 also flags it.
- `points.reference_frac_in_room` — how much of the splat was room vs cropped-away far-field
  background (windows/outside/floaters). Very low = the alignment or crop may be off; check the overlay.

### 5. Read the pictures (Read the PNGs directly)
`floorplan.png` and `elevation.png` — **red = our scan, blue = ground truth, white = agreement**:
- **Room rotated/skewed** in the floorplan (red rectangle at an angle to blue) → **yaw / heading
  drift** (BUG-051 / BUG-058 / BUG-070 class).
- **Red with no blue** → geometry the scan **invented / misplaced**. **Blue with no red** → room
  the scan **missed**.
- **Diagonal white agreement band** in the elevation (instead of horizontal) → floor/ceiling
  **planes are tilted** → gravity / orientation-prior error.
- **A displaced red block** offset vertically from the blue room → the **forked second room**
  (BUG-084 tracking discontinuity).
- Load `error_heatmap.ply` / `overlay.ply` in a mesh viewer for the 3D version.

### 6. Turn the pattern into a cause
| Symptom in the diff | Likely cause | Follow-up |
|---|---|---|
| Room rotated in floorplan, extents shrunk | yaw/heading drift accumulating | `capture_heading`, `capture_skew`, `slam_ensemble` (closure/drift) |
| All extents < 1, no rotation | translation drift | `slam_ensemble`; check `icp_mode`/conditioning |
| Extra height mode / displaced block | BUG-084 map fork mid-scan | find the last-good frame; check tracking-lost gates |
| Tilted floor/ceiling planes | gravity / SFLP orientation prior | `capture_skew`, orientation checks |

Report the diagnosis with the number **and** the picture, and name the BUG class where one fits.

## Adding / righting a ground-truth splat

- **Import one:** drop `<Name>.ply` (an INRIA-3DGS export) into `results/splats/<slug>/`. With no
  manifest it lists as `imported` with an identity orientation. `splat_compare` aligns via ICP, so
  it works upside-down — orientation only matters for *viewing*.
- **Right an upside-down import for the viewer** (non-destructive — no PLY rewrite): give it a
  manifest carrying a 180°-about-X orientation via
  `roomscan.splat.write_import_manifest(slug, results_dir, name=..., transform=<4x4>, gaussians=N)`.
  Compute the 4×4 as `R @ translate(-centroid)` with `R = diag(1,-1,-1)` (recenters the room at the
  origin so the viewer frames it). The web viewer applies it and renders SH correctly under the
  scene transform; keep `imported: true` so it stays badged external. The Scaniverse "Sam Office"
  splat is already set up this way.

## Caveats
- The reference **must be metric** (Scaniverse/LiDAR-based is). A non-metric splat breaks the rigid
  comparison — an extent ratio far from 1 across all axes is the tell.
- The rigid fit is intentional: a warped scan *should* leave residual. Don't "fix" a low fitness by
  enabling `allow_scale` (that's a scale diagnostic only).
- `results/` is gitignored — the compare artifacts and imported splats are local data, not commits.
