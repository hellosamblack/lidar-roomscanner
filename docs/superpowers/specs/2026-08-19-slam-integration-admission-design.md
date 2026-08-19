# Integration admission control — displacement gate, confidence policy, time budget

**Status:** Specced (priority/later)
**Date:** 2026-08-19
**Issues:** #149 (keyframe-gate map updates by minimum displacement), #152 (working-memory
time budget), #153 (confidence and depth-bleeding filtering at cloud construction)
**Companion specs:** pose-graph backbone (needs #149's keyframe rate to bound its node
store), mesh ceiling (fewer integrations/metre slows block growth)

Three gates between the sensor and the map, specced together because they share one
principle: *what enters the map is a decision, not a default*.

## A. Displacement gate (#149)

### Spike result (2026-08-19, measured)

One full shipped-defaults SLAM pass per capture; the exact pose of every frame reaching
`TsdfMap.integrate` recorded, then the RTAB-Map gate (integrate iff moved ≥ T_lin or
rotated ≥ T_ang since last integrated frame) simulated over that sequence:

| capture | frames (all integrate today) | 0.02 m/0.02 rad | **0.05/0.05 (RTAB-Map)** | 0.10/0.10 |
|---|---|---|---|---|
| `imuTranslationError.bin` (tripod) | 3019 | 66.7% skipped | **85.0% skipped** | 92.6% |
| `roomSweepFull20260730.bin` (sweep) | 3525 | 32.2% skipped | **67.8% skipped** | 82.7% |

Summed path drops 13.20 → 7.80 m on the tripod at the 0.05 gate (~41% of the reported
path is sub-threshold jitter; ~12% on the real sweep). Two honesty notes: (1) #149's
"18–20 m fabricated path" figure is stale — the shipped ZUPT/stationarity fixes already
cut the tripod to ~13.2 m integrated / 11.6 m reported; (2) this is an **open-loop**
simulation — a real gate changes the raycast model and hence subsequent poses, so
closed-loop skip rates will differ. The numbers answer "what fraction of today's pose
stream is sub-threshold," which is the sizing question.

### Decision

Implement the gate **on `integrate()` only**. Location is pinned: `Mapper.step` calls
`self._tsdf.integrate(...)` unconditionally at `slam/mapper.py:681` (inside `if not
lost:` at :672); the gate compares `pose` against a new `_last_integrated_pose` and skips
the integrate call while still advancing `_t_prev`/`_bootstrapped` (:683-684). Gated
frames **still run ICP** (pose continuity) and **still update display/orientation** —
the gate starves the map of redundant viewpoints, never the tracker of frames. Gate on
the pose delta, not IMU rate: the pose is the thing we are deciding to trust.

### Benchmark plan

Matched ensembles (`slam_ensemble`, n=10) on ≥3 captures — tripod + both room circuits —
at thresholds {off, 0.02, 0.05, 0.10}. Win conditions: reduced fabricated path on the
tripod and reduced TSDF block growth (report blocks at end of run), with **no** closure
regression on the circuits (paired CI must not show a significant worsening of
`horizontal_closure_m`) and no increase in tracking loss or ICP escalations — the known
opposite failure is starving frame-to-model of recent geometry. Cost: ~30 min per 10-run
CPU ensemble per capture (measured).

### Kill criterion

Killed if every threshold that meaningfully reduces integrations (>30% skip on the
circuits) also degrades circuit closure or tracking-loss counts with a significant
paired CI — that would mean this sensor's frame-to-model genuinely needs redundant
integrations, and the finding (with numbers) closes #149.

## B. Confidence policy (#153)

### Audit result (2026-08-19): the issue's premise is stale — confidence is already a hard gate

- `Mapper._gate_confidence` (`mapper.py:492-504`) zeroes depth below `min_confidence`
  (default **20.0**, `mapper.py:37`, on in every path — live, CLI, Detailed, remote —
  since 2026-07-11). NaN confidence fails the `>=` and is invalidated. So the real
  question is **rejection vs. weight**, not "is it used".
- **The threshold's provenance is dangling:** three code comments cite
  `task-quality-report.md` for the 20.0 value; that file exists nowhere in the repo or
  its git history. The default has no auditable derivation.
- **The weight plumbing already exists, unused:** `source_cloud` (`slam/cloud.py:15,22-24`)
  accepts an `intensity` array and attaches `pc.point["intensity"]`; nothing passes it.
- **No host-side analogue of RTAB-Map's depth-bleeding or bilateral filters** exists; the
  only flying-pixel defense is the vendor transform's internal stage
  (`VL53L9_TRANSFORM_LIGHT=0` — load-bearing, see CLAUDE.md).
- **Silent-flat hazard:** if `TRANSFORM_LIGHT` ever reverts to its self-defined 1, the
  confidence plane flattens to ~54 and the ≥20 gate becomes a silent no-op. No runtime
  assertion on the confidence distribution exists anywhere.
- **Range asymmetry:** the deprojector admits points to 10 m (`deproject.py:29`) but the
  TSDF caps at `depth_max = 5.0` m (`tsdf.py:201`) — points in the 5–10 m band steer ICP
  against a model that structurally cannot contain them (BUG-067-adjacent).
- **Terminal at the margin:** an aggressive threshold plus `_MIN_VALID_POINTS = 100`
  turns a low-confidence frame into `tracking_lost` — the exact BUG-036 dynamic the
  issue warned about (pinned by `test_slam_mapper.py:119-125`).

### Decision

1. **Re-derive the threshold** with an ensemble A/B over {None, 10, 20, 40} on the three
   reference captures, replacing the unauditable 20.0 with a measured choice.
2. **A/B confidence-as-weight** against the hard gate, via the existing `intensity` hook:
   pass normalized confidence into `source_cloud` and use it as per-point weights in the
   translation solve (`odometry.py:317-320` — the normal equations take a diagonal weight
   trivially). Weight is hypothesized better than rejection (rejection is terminal), but
   that is a measurement, not an assumption.
3. **Add the missing guard rails regardless of A/B outcome:** a startup/first-frame
   assertion on the confidence distribution (flat-at-~54 ⇒ hard error naming
   `TRANSFORM_LIGHT`), and align the SLAM deprojection range with `depth_max` (or record
   why not).
4. **Depth-bleeding/bilateral filters: not adopted** unless the confidence A/B leaves
   measurable flying-pixel residue — the vendor stage already covers this and a second
   filter needs its own evidence.

### Benchmark plan / kill criterion

Same ensemble protocol and captures as the displacement gate (they can share runs).
Weight mode wins only with a positive paired CI on closure or tracking-loss reduction vs.
the tuned hard gate; otherwise the hard gate stays and #153 closes with "audited:
rejection, tuned, guarded" as the outcome. The guard-rail items ship regardless.

## C. Per-frame time budget (#152)

### Decision

Adopt the *principle* — per-frame cost must not scale unbounded with map size — but the
RTAB-Map mechanism (WM→LTM demotion) is premature while the concrete cost drivers each
have cheaper, targeted answers: BUG-035's collapse shipped a capacity warning; extraction
cost belongs to the mesh-ceiling spec (chunked/incremental extraction); raycast target
windowing is the one live candidate. So: **defer the general demotion machinery; ship a
budget *monitor* first** — per-stage wall time vs. a stated per-frame contract (~33 ms),
logged and surfaced when breached, with `tick_share` from the existing watchdog as the
blocking-cost metric (never summed lateness). Report what exceeds budget; silence is how
BUG-035 stayed invisible for 560 frames.

### Benchmark plan / kill criterion

The monitor is cheap and unkillable (it is observability). The demotion mechanism is
triggered only by evidence from the monitor: if a full 10-minute-scale capture (needs
DC-B re-record) shows per-frame cost growth attributable to a *searchable-set* size (not
extraction, not capacity), design the demotion with an immunized local neighbourhood
then. Killed as a standalone work item if the monitor shows per-frame cost flat after
the mesh-ceiling spec lands — then #152 closes as "principle adopted via monitor;
mechanism unneeded on this map representation."

## Sequencing

A and B share ensemble runs — implement both behind config flags, sweep once. C's
monitor can ship independently. All three are downstream-neutral to the pose-graph spec
except that #149's accepted threshold defines the keyframe rate its node store budgets
against.
