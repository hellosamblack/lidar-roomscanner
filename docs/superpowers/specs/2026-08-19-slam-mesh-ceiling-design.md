# Lifting the mesh-extraction ceiling

**Status:** Specced (priority/later)
**Date:** 2026-08-19
**Issues:** #111 (lift the mesh-extraction ceiling)
**Companion specs:** native engine (`2026-08-19-slam-native-engine-design.md`, nvblox is
the escalation path), pose-graph backbone (per-node meshing is an alternative answer),
integration admission control (#149 slows block growth upstream)

## Problem

Open3D 0.19's CUDA marching cubes kills the *process* (segfault / illegal memory access →
`terminate()`) above **~260k active blocks** — an absolute count, not a load factor: the
same 273,521 blocks die at 68.4% load in a 400,000-block grid (BUG-053, #64, closed
`status/vendor` — closed as *not fixable by us at this Open3D version*, not as resolved).
`tsdf.py`'s `_MAX_SAFE_EXTRACT_BLOCKS = 250_000` refusal is live. The cost is concrete:
at 5 mm voxels, `DebugCapB1.bin` crosses the ceiling at frame 2625 of 4808 and can never
produce a mesh, which is why the Detailed preset retreated to 10 mm (BUG-054). A
room-sized capture at the sensor's actual resolution is unreachable — the whole point of
Detailed mode.

## Decision

Ordered, each step gating the next:

1. **Re-bisect on the newest Open3D first.** An upstream fix makes this whole spec
   unnecessary. Same method as the original: extract once at a known block count, no
   other extractions in flight.
2. **Build the subprocess bisector as an MCP tool** (named as 6.I's missing tool; the
   companion watchdog already shipped as `slam_stall_profile`). It must run each
   extraction in a **subprocess**, because the failure mode is a segfault/`terminate`
   that takes an in-process harness with it (same class as the OffscreenRenderer
   lesson: some failures abort, you can only make them unreachable). Output: the
   measured ceiling for the installed Open3D, as a number the refusal constant can cite.
3. **Primary fix: chunked extract-and-stitch.** The VBG is block-addressed, so the
   partition is natural; partitions overlap by one block and duplicate vertices are
   welded. Chosen over a different mesher because it keeps Open3D and every downstream
   consumer unchanged, and it makes extraction *incremental* — which also attacks
   BUG-052's per-extraction cost (the 1.11 s GIL-held whole-grid copy era) and #152's
   cost-scaling concern.
4. **Decouple tracking voxel from map voxel** (BUG-054's proposed resolution — track at
   10 mm, reconstruct at 5 mm) rides along once chunked extraction makes 5 mm maps
   extractable at all.
5. **Escalation path, not parallel work:** nvblox (#114) via the native-engine spec, and
   per-node meshing via the pose-graph spec (#150) — each dissolves the single-global-grid
   premise rather than raising its ceiling. Neither starts until chunked extraction has
   a measured verdict.

## Benchmark plan

- **Correctness:** chunked-vs-whole extraction on a below-ceiling capture must be
  vertex-and-triangle equivalent after welding (byte-level indifference not required;
  count and geometry deltas reported — seams framed so a null result can't be explained
  away: assert zero boundary-edge count delta along partition planes, not "looks fine").
- **The headline gate:** `DebugCapB1.bin` at `voxel_size = 0.005` produces a complete
  mesh (today: impossible past frame 2625). Report peak blocks, extraction wall time,
  and `tick_share` via `slam_stall_profile` (blocking cost, not wall time, is the number
  that matters on the live path).
- **Scaling curve:** extraction time vs. block count for whole-grid vs. chunked, past
  the old ceiling, from the bisector tool.
- Ensembles are not needed for the crash gate (deterministic) but any claim about
  Detailed-quality improvement at 5 mm goes through `slam_ensemble` as usual.

## Kill criteria

- **Killed by upstream:** if the newest Open3D extracts cleanly at 400k+ blocks, close
  #111 with the bisect evidence, bump the pinned version, delete the refusal constant.
- **Chunked path killed** if seam welding cannot reach zero boundary-edge delta (visible
  seams in the product are a non-starter), or if incremental chunked extraction at the
  Detailed cadence exceeds the current whole-grid extraction's blocking cost — then the
  answer is a different mesher, and the decision moves to the native-engine spec's nvblox
  stage (or VDBFusion as the CPU fallback, with its PCIe round-trip measured, not
  assumed).
- **Voxel decoupling killed** if 5 mm reconstruction shows no measurable quality gain
  over 10 mm on `splat_compare` against the ground-truth splat — 5 mm is already the one
  setting this repo has measured going backwards; do not ship it on principle.
