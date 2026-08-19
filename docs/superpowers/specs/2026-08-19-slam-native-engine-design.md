# Native engine candidates — small_gicp, nvblox, roomscan-native

**Status:** Specced (priority/later)
**Date:** 2026-08-19
**Issues:** #113 (small_gicp), #114 (nvblox), #116 (roomscan-native C++ core)
**Source report:** `references/software/frameworkExploration/researchResults.md` — used
with care: it is an external LLM-generated viability report, states **no licensing
information**, and sizes every VRAM claim against a **16 GB** RTX 2000 Ada when the
measured card is **8 GiB** (§6.G). Its architecture reasoning is useful; its numbers are
not evidence. Per the standing decision, these libraries are migration *candidates* to
benchmark, not commitments.

## Spike result (2026-08-19): small_gicp measured on real rig frames

`small_gicp 1.0.1` (one 378 KB manylinux wheel, zero install friction; left in
`host/.venv`) vs. the shipped registration, on 100 consecutive real pairs from
`roomSweepFull20260730.bin` (median 2200 pts/frame, identity init, matched knobs
`max_dist=0.05`, 6 iterations; i7-13850HX, 26 logical cores):

| engine (ms/pair) | 1 thread, reg only | 1 thread, + per-frame prep | 8 threads, total |
|---|---|---|---|
| shipped `translation` solve | 9.66 | — (raycast supplies normals) | 9.38 |
| Open3D `6dof` `_reg.icp` | 7.42 | — | **15.78** (slower threaded — tiny-workload thrash) |
| small_gicp GICP | 5.40 | 11.03 (cov est. 5.07) | **2.19** |
| small_gicp point-to-plane | 6.61 | 12.95 | 2.37 |

Agreement: small_gicp GICP vs Open3D 6dof translation delta **1.9 mm median / 5.9 mm
p95** (median inter-frame motion 19.3 mm) — same answer, so this is a speed question,
not a correctness one. All 6-DoF engines differ from the shipped translation solve by
13–15 mm median — that is the *mode* (real rotation leaking into translation under
identity init, BUG-067's mechanism), not the engine.

Three facts that reframe #113:

1. **The shipped `translation` mode does not call Open3D registration at all** — it is a
   hand-rolled hybrid_search + 3×3 numpy normal-equations solve
   (`odometry.py:278-330`). "Replace Open3D ICP" is the wrong frame; the candidate
   replacement target is a 9.4 ms numpy/NNS loop.
2. **small_gicp's Python wheel has no rotation-prior / translation-only mode.** The
   report's own §7.1 concedes DoF masks and IR-intensity costs require building from
   source with a custom pybind wrapper. And this project has *measured* that free 6-DoF
   rotation loses to the SFLP IMU (adaptive experiment: 17 m vs translation's 0.63 m on
   the null capture) — accuracy, not speed, justifies the shipped mode.
3. **The prep cost is real:** GICP needs covariances on both clouds every frame
   (1.1–5 ms), while frame-to-model gets target normals from the raycast for free. And
   the whole ICP stage is ~10 ms of a ~35 ms/frame budget — the ceiling on any win is
   Amdahl-bounded to a few ms unless the rest of the frame moves too.

## Decision

Staged evaluation, each stage with its own gate; **no cutover commitment at any stage**.
The Open3D pipeline remains the shipped engine until a stage's gate passes.

- **Stage N1 (#113): small_gicp inside the real Mapper, cheapest honest form first.**
  Before writing any C++: an A/B where small_gicp runs 6-DoF *initialized* with the SFLP
  rotation (no mask), frame-to-model, against the shipped solve — ensembles on the null
  capture (`imuTranslationError.bin`) and both circuits. The null capture is the
  assassin: every 6-DoF variant so far fabricates path there. Only if sg-with-IMU-init
  survives the null (≤ the shipped mode's ensemble drift) does the C++ work to get true
  3-DoF masks + a pybind wrapper get authorized — that build cost (from-source compile,
  custom wrapper, maintenance of a vendored native dep) is reported as part of the
  recommendation per the dependency rule, never assumed away.
- **Stage N2 (#114): nvblox is the mesh-ceiling escalation, not a parallel adoption.**
  It starts only if the mesh-ceiling spec's chunked-extraction path is killed. Entry
  criteria from the report, verified not assumed: standalone CMake build (CUDA + Eigen +
  glog), custom pybind bridge required, and **no automatic distance-based eviction** — a
  10-minute scan exhausts VRAM unless we build the sliding-window pruning ourselves, on
  an 8 GiB card (not the report's 16). Also weigh the measured alternative that costs
  nothing: the CPU `VoxelBlockGrid` completed the full sweep with 0 lost frames — the
  grid is device-homogeneous, so CPU is already the route to bigger-than-VRAM maps.
- **Stage N3 (#116): roomscan-native exists only as the conjunction gate.** A C++ daemon
  owning ingestion + odometry + graph + meshing is justified only if N1 *and* N2 have
  individually passed *and* profiling (`slam_stall_profile` `tick_share`, `slam_icp_bench
  what="ab"` — never the isolated microbench, which swings 43% between sessions) shows
  Python orchestration itself is the remaining bottleneck. Until then #116 is a
  dependent placeholder, and "the legacy Open3D architecture must be deprecated"
  (report §6) is explicitly **not** adopted as a premise.

## Benchmark plan

- **N1:** `slam_ensemble` (n=10) × {shipped translation, sg-GICP-IMU-init, and — if the
  C++ wrapper is built — sg-3DoF-masked} × {null capture, both circuits}. Primary
  metrics: null-capture fabricated path, circuit `horizontal_closure_m` (paired CI),
  tracking loss/escalations; secondary: real per-frame wall time and `tick_share` inside
  the Mapper (frame-to-model, not the spike's frame-to-frame). Quote `capture_analyze`
  continuity first, as always.
- **N2:** the mesh-ceiling spec's gates apply verbatim (DebugCapB1 at 5 mm, blocking
  cost, seam correctness), plus a VRAM-bounding demonstration on a 10-minute-scale run
  and a written build-burden report.
- **N3:** end-to-end frame budget on the live path vs. the Python orchestration, same
  capture, same ensemble discipline.

## Kill criteria

- **N1 killed** if sg-with-IMU-init fabricates on the null capture (ensemble drift
  materially above the shipped 0.63 m-class baseline), or if — after the mask work — the
  end-to-end per-frame improvement inside the real Mapper is < 2 ms (the spike's 4× on
  a 10 ms stage caps the win; a native dep + custom wrapper is not worth < 2 ms), or if
  drift/closure worsens at all. Close #113 with the spike + ensemble numbers either way —
  the measurement is the deliverable.
- **N2 killed** by the mesh-ceiling spec succeeding upstream (chunked extraction or a
  newer Open3D), or by the build-burden report (sliding-window eviction we must author
  ourselves on 8 GiB) exceeding what the CPU-grid fallback already provides for free.
- **N3 killed** by either prerequisite failing — and independently killed if profiling
  attributes the live bottleneck to anything other than Python orchestration (today's
  evidence says GIL-holding native calls, not Python, are the historical stall source).
