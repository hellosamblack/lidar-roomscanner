# Relocalization and place recognition — recovering from a bad second

**Status:** Specced (priority/later)
**Date:** 2026-08-19
**Issues:** #112 (relocalization), #154 (appearance-based place recognition evaluation)
**Companion spec:** pose-graph backbone (`2026-08-19-slam-posegraph-backbone-design.md`)

## Problem

The ICP escalating retry (BUG-036) survives a bad *frame*, not a bad *second*: a real
tracking kill is terminal unless the operator walks back to already-mapped geometry —
which the DC-C stress captures show works (82–120-frame freezes, every run recovered,
0 died), but only because the capture protocol *is* the walk-back. The gap is real but
operator-maskable (`docs/roadmap-history.md` §6.D). #112 is also the ceiling on the
residual BUG-067 path over-report: only a re-anchor removes fabricated drift the
coherence veto can't hold through.

## Spike result (2026-08-19): appearance-based recognition is dead on this sensor

#154 branch 1 asked whether 54×42 IR/reflectance carries repeatable keypoints. Measured
on real captures (ORB + GFTT, log/percentile normalization, params rescaled for the tiny
image; opencv-python-headless 5.0.0.93 in `host/.venv`; scripts were throwaway):

| condition | ORB matches (med) | RANSAC inlier frac |
|---|---|---|
| sweep, adjacent frames (Δ=1) | 57 | 0.79 |
| **sweep, Δ=30 ≈ 1 s (~83% overlap retained, verified by phase correlation)** | **17** | **0.36** |
| **chance floor: Δ=600 ≈ 20 s, different heading** | **16** | **0.35** |
| static tripod, Δ=30 | 52 of ~178 kp | 0.79 |

After one second of gentle panning, matching is statistically indistinguishable from
matching unrelated frames — and even a rigidly static scene loses ~70% of
correspondences per second to temporal sensor noise alone. 4× upsampling does not help
(Δ=30 gets worse); a fixed-pattern-noise probe (same-coordinate fraction 0.00 at Δ=600)
confirmed FPN is neither faking nor masking the result. **BoW over ToF-IR cannot work.**
This kills #154's branch 1 by measurement, not by argument.

## Decision

1. **Place recognition, where needed, is geometric or borrowed — never IR-appearance.**
   Remaining branches, in order of evaluation cost:
   - **Pose-proximity submap matching** (no descriptor at all): candidate revisits chosen
     by proximity of the current pose estimate to stored keyframe poses, verified by
     strict ICP against the stored node cloud (the `researchResults.md` §8.2 /
     GLIM-style design, and what the pose-graph spec's offline pass already requires).
     This is the primary path — it needs no new signal and its failure mode (drift too
     large for proximity search) is exactly what a bounded search radius exposes.
   - **Scan-context-style depth descriptors** (the LiDAR community's answer): cheap to
     evaluate offline on existing captures with the *same protocol as the IR spike* —
     descriptor distance between revisits vs. unrelated frames on a circuit capture. Only
     pursued if pose-proximity proves insufficient (i.e., real kills where the pose prior
     is too wrong to bound the search).
   - **Borrow the phone** (Phase 7 only): RTAB-Map loop closures computed on textured
     imagery, transferred onto our graph. Offline-only by construction; not a live
     relocalization answer.
2. **Live relocalization v1 = re-anchor, not recognition.** On a detected kill
   (`tracking_stats` already reports `died`/freeze runs): freeze integration, hold the
   SFLP orientation (trustworthy through a kill — it is not ICP-derived), and run the
   escalating ICP against the raycast model from a **zero-velocity re-anchored** pose
   with a widening translation search. This formalizes the walk-back that DC-C proved
   works, and removes the operator from the loop for the common case.
3. **Adopt the wrong-loop / wrong-anchor rejection ratio** (post-optimization residual
   over per-link sigma, RTAB-Map `RGBD/OptimizeMaxError = 3.0` analogue) on any accepted
   re-anchor or revisit edge — representation-independent, and the one #154 item that
   ports regardless of branch.

## Benchmark plan

- **Fixture:** `DebugCapC.bin` (deliberate 2 s tracking kills) — already recorded, known
  freeze windows of 82–120 frames. Ensembles via `slam_ensemble` (n=10).
- **Metrics:** recovery latency (frames from kill to re-lock), recovered-pose error at
  re-lock vs. the pre-kill trajectory extrapolation, fabricated path during the freeze
  (today's baseline: 376–467 lost frames/run), and — regression guard — no change on
  kill-free captures (the re-anchor must be unreachable when tracking is healthy).
- **Scan-context spike (only if triggered):** offline descriptor-separation test on
  `coffeeRoomCircuitNoMnt.bin` — distance distribution of true revisit pairs vs.
  unrelated pairs; report AUC. Same discipline as the IR spike: a control arm of
  unrelated frames, and the check must be able to see wrong answers.
- **Cost note:** a CPU SLAM pass is ~70–125 s per capture (measured 2026-08-19), so a
  10-run ensemble is ~30 min/capture — plan sweeps accordingly.

## Kill criteria

- **Re-anchor v1:** killed if it fires on healthy captures (any activation on the
  circuit ensembles) or if recovered-pose error on DC-C exceeds what the walk-back
  protocol achieves today — automation that recovers worse than the operator is a
  regression.
- **Scan-context branch:** killed if the offline separation test shows revisit vs.
  unrelated AUC < 0.8 — that is the same "chance floor" verdict the IR spike delivered,
  and the honest conclusion becomes: *no in-session place recognition is viable at 54×42;
  relocalization is re-anchor + pose-proximity, full stop.* Close #154 with the two
  negative measurements as the record.
- **Phone-transfer branch:** out of scope here; it lives or dies with the offline-3DGS
  program spec (posed phone captures) and never gates live scanning.

## Explicit non-goals

- No BoW, no visual vocabulary, no learned descriptors on ToF-IR (measured dead, above).
- No live loop-closure search — that stays behind the pose-graph spec's paired-CI gate.
