# SLAM compute & transport follow-ups (from the BUG-061 review)

**Status:** active. Items 1–3 are **done and landed**; items 4–7 are open.
**Origin:** the owner's concurrency/GPU review written during the BUG-061 push
(2026-08-02). This file replaces the root-level `BUG-061-handoff.md` scratch log,
whose execution-state half is obsolete now that BUG-061 (`6b7e8fb`) and BUG-062
(`bf3d74c`) have landed. The review's substance is preserved below.

> **Register note:** a concurrent session is mid-way through the plans/specs
> archive move that adds the "Plans and specifications register" table to
> `ROADMAP.md`. This document needs an **Active** row there once that lands.

---

## Scope discipline (from the review, still binding)

The recommendations deliberately keep the user-visible BUG-061 transport fix
separate from deeper SLAM compute changes: **changing ICP or device/host
ownership while fixing transport would make a before/after lag result
impossible to interpret.** Keep that separation for the remaining items too.

## Evidence and confidence

Two execution environments appear in this evidence and must not be conflated:

| Evidence | Result | Confidence / limitation |
|---|---|---|
| Agent venv, Open3D 0.19.0 | `o3d.core.cuda.is_available()` false; `preferred_device()` = `CPU:0`; `[slam].backend` = `local` | Directly measured, but this sandbox is **not** the owner's CUDA rig |
| `slam_stall_profile.py`, first 300 frames of `captures/roomSweepFull20260730.bin`, `CPU:0` | 14.7 s wall, 300 integrated, 20.42 effective fps; `step` p50 19.1 ms, `mesh` p50 55.4 ms, `prep` p50 67.0 ms, `pack` p50 4.8 ms; watchdog starvation 49.1% of wall, worst stall 94.0 ms | A CPU baseline only. The tool runs stages serially to attribute cost; it is not a measurement of live latest-wins drops. **These magnitudes will not transfer to the rig — re-run on CUDA:0 before sizing any work off them.** |
| CUDA at-scale validation on the rig | GPU median 8.85 ms vs CPU 18.94 ms (~2.1×); GPU degradation flat while CPU climbed | `BUGS.md:1507-1513`; re-run after any compute-path change |
| Long-scan validation | Per-frame CUDA memory flat; the leak was throttled extraction, fixed by cache release. CUDA marching cubes unsafe above ~250k active blocks | Measured and guarded; **do not "fix" this by raising `block_count`** |

The sandbox result does **not** disprove that the owner's live web process was on
CUDA:0 — it only means CUDA-specific changes cannot be validated from here. The
rig, a clean CUDA replay, and the device reported by the actual worker remain
authoritative.

## What the current threading actually provides

Useful isolation, but **not** parallel SLAM:

1. `SlamRunner.submit()` runs on the reader thread and forwards into a size-one
   latest-wins slot; `SlamWorker` has one processing thread calling
   `Mapper.step()` serially. A newer frame overwrites a pending one.
2. Mesh extraction runs synchronously inside that same mapper worker every five
   successful integrations. `MeshPrep` then has its own latest-wins thread.
   Preparation can overlap later mapper work; **extraction cannot**.
3. The broadcaster is separate and now receives pose before mesh — but Python
   threads do not defeat a native Open3D call holding the GIL.
4. Detailed/offline SLAM is one background worker processing frames in order.

This is a sound bounded-latency design. **Adding a thread pool around
`Mapper.step()` would be unsafe** without partitioning the mutable TSDF and pose
state, and would destroy frame ordering. The practical lever is to make one
serial step cheaper and to stop extraction/transport blocking the next input.

---

## Item 1 — Finish and verify BUG-061 ✅ DONE (`6b7e8fb`)

MESH moved to a credit-gated `/ws-mesh`; pose sent first; client geometry reuse
and `frustumCulled = false`. Backlog 37 MB → 0; pose age p50 1.1 ms / p95 4.8 ms
on a client that keeps up. **Caveat carried forward:** on the 2 fps llvmpipe
headless client pose age is p95 2.82 s, bounded by its own render rate — the
≤0.15 s contract is demonstrated only on a client that can keep up, and the
owner's real browser was never instrumented, only its socket queues.

## Item 3 — Reconcile live config plumbing ✅ DONE (`bf3d74c`, BUG-062)

Promoted above item 2 because it was a **correctness defect, not an
optimization**, and because item 4 depends on it. The review flagged 4
unforwarded fields; verification found **13**. `SlamConfig.mapper_kwargs()` is
now the single source. See `BUGS.md` → BUG-062.

---

## Item 2 — Counters and stage timing (OPEN)

Instrument before optimizing; preserve a no-instrument baseline where possible
because timing itself perturbs CUDA. Note `slam_stall_profile` already covers
much of the stage timing — **the genuinely missing piece is the input-slot
overwrite counter.**

Expose separately:

- resolved compute device and backend (`local` vs `remote`);
- **submitted frames, processed frames, and input-slot overwrites**;
- mapper step time and its raycast / ICP / integrate components;
- mesh extraction time, mesh-prep time, payload bytes;
- GPU utilization, VRAM used/total, and the measurement scope.

**Do not call an overwritten input a "tracking lost" frame** — it never reached
the mapper and needs its own counter. Likewise, VRAM allocation proves a TSDF
exists on CUDA, not that kernels use the device efficiently.

## Item 4 — Matched CUDA ICP / raycast study (OPEN — now unblocked by BUG-062)

The default `icp_mode` is `translation`, and that path is **only partly GPU
accelerated**: source/target positions and target normals are copied into NumPy;
the residual, condition check and 3×3 solve run on the CPU; each iteration copies
neighbour counts and indices back (`odometry.py:41-89`, `:109-125`). The GPU is
used for the hybrid NN search, but this is not an end-to-end GPU ICP solve.

Run a matched CUDA ensemble comparing:

- `translation` ICP as implemented;
- Open3D tensor `6dof` ICP (`odometry.py:127-142`);
- translation-only alternatives keeping matching and the normal-equation solve on
  device, **if** Open3D's tensor API supports the gather/mask operations without
  hidden synchronizations.

**Measure accuracy, not just `slam_ms`:** lost frames, fitness/RMSE, path,
loop-closure gap and map block count must stay within the validated baseline. A
GPU implementation that is faster but changes translation semantics or worsens
drift is not an acceptable optimization. Per the repo's own lesson, **score
ensembles, not single runs** — a 3 mm perturbation moves closure by 0.37 m.

The GPU-resident translation implementation is **not yet investigated**. Do not
assume `hybrid_search()`, boolean masks, tensor gathers or `np.linalg`
equivalents have identical behaviour and cost on Open3D 0.19 CUDA. Confirm the
API and profile it on the installed CUDA build first.

### Raycast host round-trips (fold into item 4)

`TsdfMap.raycast()` copies vertex/normal/depth to the host, filters valid depth
with NumPy, and rebuilds a point cloud on device (`tsdf.py:357-400`).
`Mapper.step()` then copies positions back **again** merely to count valid points
(`mapper.py:283-287`). The CPU intrinsic/extrinsic requirement is real and
documented — do not "optimize" that by guessing. The available win is after
`ray_cast`: keep results on device, mask and gather there if supported, and
return valid-count metadata so the mapper does not transfer twice. Needs a CUDA
microbenchmark — a nominally device-side op can still force a sync.

## Item 5 — Implement the winning GPU-residency changes (OPEN, gated on item 4)

Add focused tensor/API tests, then rerun long-scan memory and extraction-ceiling
guards.

## Item 6 — Remote service output scheduling (OPEN, only if remote is active)

`serve_client()` is synchronous: receive a frame, run the worker, send pose,
maybe encode and send a whole mesh, then receive the next (`service.py:37-72`,
`wire.py:46-49`). A large `sendall()` can prevent reading newer input — distinct
from the browser `/ws` backlog now fixed. At minimum, service compute should not
call `sendall()` directly. **Do not claim pose-before-mesh ordering solves this:**
it protects the pose for the frame just processed, not the next frame while the
previous mesh transmits. Not implemented or benchmarked; treat as follow-up
unless on-rig verification shows remote SLAM is the active deployment and its
pose age still violates the contract.

## Item 7 — TSDF saturation polling cleanup (OPEN, low priority)

`TsdfMap.integrate()` calls `_check_saturation()` after every integration, which
reads the CUDA hashmap size on each call until the 90% warning fires
(`tsdf.py:231-253`, `:265-290`). BUG-035 measured this at ~5.8 µs/call, ~0.02 s
over a full sweep — **not a leading cost**. Fold into a 25-integration or
metrics-rate check as hygiene during the next compute change.
**Do not remove the warning or the rehash-headroom guard** — the capacity failure
was real; only the cadence is in question.

---

## Rejected — do not reopen without new evidence

`MeshPrep`'s adaptive decimation. It reduced payload by holding Open3D quadric
decimation on the GIL: `prepare_packet` 178 → **2440 ms p50**, starvation
11.9% → **94.3% of wall**. **Do not reactivate it merely because a GPU
utilization gauge reads low.**

## Acceptance gates before claiming "GPU/multithreading fixed"

Required evidence, not assumptions:

- A CUDA-enabled Open3D runtime confirmed on the target machine, with the
  **worker reporting its own device** rather than the host inferring it.
- A matched capture showing processed-frame throughput and overwrite counts at
  the ~28 Hz sensor rate. If frames are intentionally dropped, the UI says so.
- Any new ICP path preserves validated trajectory, tracking-loss, fitness/RMSE
  and map quality within a tolerance **chosen from the existing ensemble, not
  invented after seeing one favourable run**.
- GPU utilization measured during actual SLAM work with no competing workload,
  and the scope labelled device-wide or per-process. `nvmlDeviceGetUtilizationRates`
  is device-wide and **cannot** prove SLAM used the GPU.
- No new device↔host transfer in the hot path; any remaining one has a documented
  Open3D API reason and a measured cost.
- Long replay keeps VRAM bounded, does not cross the extraction ceiling, and
  produces no new GIL-held stall.
- BUG-061's contract still passes: pose p95 ≤ 0.15 s on a client that can keep
  up, bounded mesh backlog, correct World/FPV/Mirror, no Detailed regression.
