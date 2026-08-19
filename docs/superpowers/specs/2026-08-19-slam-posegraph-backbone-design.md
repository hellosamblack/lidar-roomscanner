# SLAM pose-graph backbone — loop closure, per-node map, graph constraints

**Status:** Specced (priority/later; #110 is priority/next but blocked)
**Date:** 2026-08-19
**Issues:** #110 (drift correction / loop-closure evaluation), #115 (GTSAM factor-graph
backend), #150 (per-node cloud store beside the TSDF), #151 (gravity constraint +
optimize-from-graph-end)
**Companion specs:** relocalization (`2026-08-19-slam-relocalization-design.md`),
integration admission control (`2026-08-19-slam-integration-admission-design.md`),
mesh ceiling (`2026-08-19-slam-mesh-ceiling-design.md`)

## Problem

Frame-to-model ICP closes a single room to **~3%** (0.74 ± 0.19 m over a 23.9 m circuit,
10-run matched ensembles, `docs/roadmap-history.md` §6.D) — that is the number any
pose-graph/loop-closure machinery has to beat. The multi-room evidence so far makes the
case *weaker*, not stronger: DebugCapB1's drift rate (3.6%) is not worse than
single-room, meaning frame-to-model drift scales with path length and does not compound
at room transitions. But the B-series data is unusable for the go/no-go — transport loss
(BUG-049 / #60, still open) explains ~4.6 m of B3's 7.3 m closure (paired bootstrap mean
diff 4.569 m, 95% CI [3.950, 5.168] m), and B2 hides a 628-frame tracking collapse
starting 70 ms after a 2.4 s outage.

Independent of whether loop closure wins on drift, there is a **structural blocker**: our
poses are baked into the `VoxelBlockGrid` at integrate time, so correcting a pose means
de-integrating and re-integrating. RTAB-Map's per-node representation applies a graph
re-optimization as *one mat4 write per node* (`RTABMapApp.cpp:2036-2039`). That — not the
size of the drift — is why loop closure has stayed behind an evidence gate
(`docs/rtabmap-study.md` §1).

## Decision

1. **The pose graph is an offline pass first, not a live 46 Hz backend.** 6.D item 3 as
   written: keyframes from offline tracking, non-adjacent pose-proximity revisit
   candidates, strict-ICP edge verification, global optimize, then re-integrate every raw
   frame against the optimized trajectory. The `researchResults.md` "iSAM2 in a C++
   thread at 46 Hz" architecture is explicitly **not** adopted at this stage — its own §7.4
   admits Python factor construction at 46 Hz is a latency risk, and we have no evidence
   yet that a graph earns its complexity at all. Offline needs no C++ daemon.
2. **Optimizer: GTSAM (#115), scoped to that offline pass.** Chosen over g2o because the
   two graph constraints we want are first-class there: `Pose3AttitudeFactor` (or a
   near-zero-rotational-covariance `PriorFactor<Pose3>`) reproduces the 3-DoF discipline
   the live pipeline already learned the hard way (free 6-DoF rotation loses to the SFLP
   IMU on this sensor — ROADMAP "never 6dof"), and gravity factors are supported (#151).
   IMU preintegration factors are **out of scope** for v1 (accel-derived translation is
   a standing non-input, §6.G survey).
3. **Map correctability: hybrid per-node store (#150), not a rewrite.** Keep the TSDF as
   the live fused surface; *additionally* retain per-keyframe clouds (deprojected,
   decimated, node-frame) with poses, admitted at the keyframe rate from the displacement
   gate (#149 — see the admission-control spec; the measured gate keeps only 15–32% of
   frames, which bounds this store). A loop closure then = optimize graph → rewrite node
   poses → rebuild a fresh TSDF as a batch replay. No interactive de-integration.
4. **Graph conventions decided now, before any UX exists (#151):** gravity direction is a
   per-node constraint (RTAB-Map `Optimizer/GravitySigma = 0.2` as the starting point,
   final value by ensemble), and optimization pins the **last** pose
   (`OptimizeFromGraphEnd = true` semantics) so a correction moves the map under the
   camera, never jumps the camera.
5. **Adopt the wrong-loop rejection check unconditionally** (`RGBD/OptimizeMaxError = 3.0`
   analogue: reject a loop edge if post-optimization residual over per-link sigma exceeds
   the ratio). It is representation-independent and cheap (#154's one portable piece).

## Prerequisites (in order)

1. **#60 (BUG-049) fixed or bypassed** — the Pi 3 bridge (#191) with its pcap tee is the
   in-flight mitigation. Then **re-record DC-B (#138)**: ≥2 multi-room closed-loop takes
   passing `capture_analyze` continuity (`complete`, not merely `clean` — the old takes
   were byte-perfect while losing 2.3–9.4% of frames).
2. **Characterize the SFLP gravity-direction error before assigning it a sigma** (#151):
   the quat is a batch mean with a capture-dependent, sign-varying phase lead
   (+5.1 ± 0.7 ms typical, −3.9 ms on `officeFullScanAug6.bin` — measure per capture with
   `capture_skew`, never assume the recorded number), and the accelerometer sees hand
   motion. A short static + slow-tilt analysis on existing captures is enough.
3. Keyframe store cost measured honestly on `roomSweepFull20260730.bin` and
   `coffeeRoomCircuitNoMnt.bin` (RAM/VRAM per node at the #149 keyframe rate) before
   committing to #150's hybrid.

## Benchmark plan

- **Unit of evidence: matched ensembles**, never single runs (`slam_ensemble`, n=10; a
  3 mm nudge moves closure by 0.37 m). Check `runs_died` / `any_saturated` before quoting.
- **Arms:** baseline (shipped frame-to-model) vs. pose-graph pass, on both single-room
  circuits (`coffeeRoomCircuitNoMnt.bin`, `coffeeRoomCircuitMnt.bin`) and the re-recorded
  DC-B takes. Metric: `summary.horizontal_closure_m` (closure is drift only where the
  operator returned to start — DC-B protocol guarantees it; `roomSweepFull20260730.bin`
  has no bookend and must not be quoted as drift).
- **Acceptance gate is mechanical:** `slam_loop_closure_gate(baseline, loop_closure)` —
  positive paired 95% CI for reduced horizontal closure on **both** circuits, no run
  dies, no increased tracking loss. This is the pre-registered 6.D item-4 gate; the MCP
  tool already exists.
- **Gravity/graph-end sub-experiments:** gravity sigma swept {0.1, 0.2, 0.3} by ensemble;
  optimize-from-graph-end verified by construction (it changes gauge, not accuracy).
- **#150 memory check:** peak RSS/VRAM with and without the node store on both reference
  captures, reported per node and extrapolated to a 10-minute scan.

## Kill criteria

- **Loop closure (#110):** if, on clean-transport DC-B re-records, the paired gate fails
  on either circuit — or multi-room drift rate remains ≈ the single-room ~3% (no
  compounding to correct) — ship `loop_closure.enabled = false` with the reason in the
  manifest, close #110 as *evaluated, does not earn its complexity indoors*, and #115
  (GTSAM) dies with it. The evaluation itself is the deliverable.
- **Per-node store (#150):** killed if the measured per-node cost extrapolates past
  ~25% of host RAM for a 10-minute scan at the chosen keyframe rate, or if the batch
  TSDF rebuild from corrected poses exceeds ~2× a Detailed replay's wall time — at that
  point "re-run Detailed offline with the corrected trajectory" is the same product for
  less machinery.
- **Gravity factor (#151):** killed if the measured SFLP gravity-direction error is so
  small that roll/pitch drift is unobservable over a 3-minute graph (then the factor
  constrains nothing) or so large/motion-contaminated that no sigma in {0.1–0.3} passes
  the ensemble without degrading closure.

## Explicit non-goals

- No live iSAM2 thread, no C++ daemon (that is the native-engine spec's Stage N3, with
  its own gate).
- No relocalization — separate spec; 6.D keeps it out of scope deliberately.
- No appearance-based loop detection — measured dead on this sensor (see the
  relocalization spec's IR spike); revisit candidates come from **pose proximity** only.
