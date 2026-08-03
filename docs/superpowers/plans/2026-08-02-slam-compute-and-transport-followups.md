# SLAM compute & transport follow-ups (from the BUG-061 review)

**Status:** active. Items 1–5 and 7 are **done**; item 6 is
**deferred by its own condition** (verified 2026-08-02 — see that section).
**Origin:** the owner's concurrency/GPU review written during the BUG-061 push
(2026-08-02). This file replaces the root-level `BUG-061-handoff.md` scratch log,
whose execution-state half is obsolete now that BUG-061 (`6b7e8fb`) and BUG-062
(`bf3d74c`) have landed. The review's substance is preserved below.

> **Register note:** the plans/specs archive move landed (`22cfad0`), and this
> document is now carried as an **Active** row in `ROADMAP.md` → "Plans and
> specifications register".

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
| ~~Agent venv, Open3D 0.19.0~~ **superseded 2026-08-02** | ~~`o3d.core.cuda.is_available()` false~~ — it reads **True** in `host/.venv`: Open3D 0.19.0, 1 device, RTX 2000 Ada Laptop, 8188 MiB. `[slam].backend` = `local` still holds | The original row was wrong, and it was load-bearing: it is what made item 4 look rig-only. Item 4 ran here on real CUDA:0. Remaining limitation is narrower — this is a *laptop* Ada shared with the owner's server, so carry **ratios** forward, not absolute milliseconds |
| `slam_stall_profile.py`, first 300 frames of `captures/roomSweepFull20260730.bin`, `CPU:0` | 14.7 s wall, 300 integrated, 20.42 effective fps; `step` p50 19.1 ms, `mesh` p50 55.4 ms, `prep` p50 67.0 ms, `pack` p50 4.8 ms; watchdog starvation 49.1% of wall, worst stall 94.0 ms | A CPU baseline only. The tool runs stages serially to attribute cost; it is not a measurement of live latest-wins drops. **These magnitudes will not transfer to the rig — re-run on CUDA:0 before sizing any work off them.** |
| CUDA at-scale validation on the rig | GPU median 8.85 ms vs CPU 18.94 ms (~2.1×); GPU degradation flat while CPU climbed | `BUGS.md:1507-1513`; re-run after any compute-path change |
| Long-scan validation | Per-frame CUDA memory flat; the leak was throttled extraction, fixed by cache release. CUDA marching cubes unsafe above ~250k active blocks | Measured and guarded; **do not "fix" this by raising `block_count`** |

~~The sandbox result does not disprove that the owner's live web process was on
CUDA:0 — it only means CUDA-specific changes cannot be validated from here.~~
**Obsolete:** CUDA-specific changes *can* be validated from here, and item 4 was.
What survives from that paragraph is the discipline, not the constraint: the
device **reported by the actual worker** remains authoritative over any host-side
inference — which is now enforced in code, since `SlamRunner` re-reads
`worker.device` every poll instead of trusting `preferred_device()` once (item 2).

A sharper environment lesson replaced it during item 4: **`nvidia-smi` alone is
not a sufficient environment check on this box.** An 8-pair A/B run was discarded
because a concurrent session's headless Chrome hit 1270% CPU and slowed *both*
arms ~2.3× while the GPU looked idle. Check CPU load and GPU together, and
discard contaminated runs rather than averaging them in.

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

## Item 2 — Counters and stage timing ✅ DONE (2026-08-02)

**Landed.** `SlamWorker`/`RemoteSlamWorker` gained `frames_submitted` /
`frames_processed` / `frames_overwritten` on the size-one latest-wins slot, with
the invariant stated in code: at any instant `submitted == processed +
overwritten + (1 if a frame is in the slot)`. An overwritten frame is **kept
strictly separate from `tracking_lost_count`** — it never reached `Mapper.step`
at all, where a tracking-lost frame reached the mapper and failed to register.
The distinction is carried through to the UI as a separate row, id, CSS class and
colour, and a test asserts the two readouts cannot collide.

`SlamRunner` now re-reads `worker.device` / `worker.backend` **every poll tick**
rather than trusting the host's one-shot `preferred_device()` guess — this is the
acceptance gate's "the worker reports its own device" clause, and it also
propagates the remote service's real device over the wire (`pose_message` gained
an optional `device` field), which the host previously had no way to know.

Stage timings are labelled by what they actually are, not uniformly:
`raycast_ms` and `icp_ms` are real elapsed time (both stages already force a
device→host sync internally, so timing them costs nothing new); **`integrate_ms`
is a dispatch-time lower bound on CUDA**, not kernel-completion time — no
`cuda.synchronize()` was added to the hot path, and both the docstring and the UI
tooltip say to read it as "at least this long", never as "this many ms of GPU
work". `mesh_extract_ms` / `mesh_prep_ms` / `mesh_pack_ms` / `mesh_payload_bytes`
are true wall clock, since `TsdfMap._extract()` always returns host-resident
results.

GPU fields reuse the **already-running** `ResourceSampler` snapshot, so the 30 Hz
SLAM poll adds no NVML call and no device sync of its own. `gpu_util_scope`
carries `pynvml` (per-process) vs `nvml-device` vs `n/a` straight through instead
of assuming, and the UI prints the scope **inline in the label** (`38.2%
(device)`) so a device-wide reading cannot be misread as SLAM's own.

The original brief follows, for the record.

### Original brief (OPEN)

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

## Item 4 — Matched CUDA ICP / raycast study ✅ DONE — see `2026-08-02-cuda-icp-study.md`

**Result:** the GPU-resident translation solve is **rejected** — correct to float
round-off and gate-accepted, but +2.37 ms/frame (+44% of a SLAM step) and it
holds the GIL almost solidly (`tick_share` 0.058, repeatable multi-second
freezes). `6dof` is **rejected on accuracy**: 8.0 ± 3.6 m closure vs the
baseline's 0.67, 3/10 ensemble runs dead. The measurable win runs the other way —
run the ICP nearest-neighbour index on the **host** while the rest stays on CUDA:
bit-identical output over 3177 calls and a full n=10 ensemble, −0.2 to −0.55
ms/frame. The raycast round-trips are only 0.15 ms/frame and the device-resident
replacement measured 0.43 ms/frame *worse*. Full evidence, caveats and the
non-inferiority tolerance are in the study doc; new instrument
`host/tools/slam_icp_bench.py` / MCP `slam_icp_bench`.

**Also correct the evidence table above:** `o3d.core.cuda.is_available()` is now
**True** in `host/.venv` (Open3D 0.19.0, RTX 2000 Ada, 8188 MiB). The "CUDA
cannot be validated from this sandbox" row is obsolete.

The original brief follows, for the record.

### Original brief (OPEN — now unblocked by BUG-062)

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

## Item 5 — Implement the winning GPU-residency changes ✅ DONE

Landed exactly the two changes item 4 recommended and nothing else:

1. **`Mapper.icp_device`** (default `"CPU:0"`, `None` = follow `device`) —
   the ICP nearest-neighbour index runs on the host while the TSDF
   integrate/raycast stay on CUDA. `odometry.register` is unchanged.
   `[slam] icp_device` → `SlamConfig.mapper_kwargs()` → CLI (`--icp-device`) →
   `SlamRunner` → `DetailedSlamPreset`. Ignored by `icp_mode = "6dof"`, whose
   ICP is Open3D's own and must run where its point clouds live.
2. **The redundant recount** at `mapper.py:285` is gone: `TsdfMap.raycast(...,
   with_count=True)` returns the valid-point count it already computed.

Per BUG-062, the field list now lives in **one** place and three re-listings
were removed on the way: `slam/cli.py::_run`, `DetailedSlamPreset.mapper_kwargs`
(now overrides `SlamConfig.mapper_kwargs()`), and the two measurement rigs
(`slam_gpu_memory.py`, `slam_stall_profile.py`) that existed to profile the
*shipped* pipeline and had quietly stopped doing so.

**Measured result — the win did not fully reproduce, and that is the finding.**
New pass `slam_icp_bench --what ab` (interleaved, paired, whole-pipeline).
Output is **bit-identical** on both captures (0.0 m over 7 pairs, identical
block counts, 0 lost, 0 escalations) on genuinely partial-match data. Speed:
**−0.23 ± 0.13 ms (−1.9%)** on `coffeeRoomCircuitNoMnt` but **+0.04 ± 0.10 ms
(+0.3%), i.e. nothing**, on `roomSweepFull` — `register` is reliably ~0.28–0.53
ms faster in both, and raycast + integrate give it all back on the larger map.
Every measurement was taken under a steady 12-core external load; a quiet box
was never available. Full numbers and caveats: the "Item 5 outcome" section of
`2026-08-02-cuda-icp-study.md`.

## Item 6 — Remote service output scheduling ⏸ DEFERRED (condition checked 2026-08-02)

**Remote is not the active deployment, so this item's own gate is not met.**
Verified: `SlamConfig.backend` defaults to `"local"`, the live config at
`/home/sam/roomscan/roomscan.toml` has **no `[slam]` table at all** (so nothing
overrides that default), and the GPU-container service that motivated the remote
backend is already carried in the register's **Deprecated** row, superseded by
in-process local CUDA:0. The analysis below stands and should be acted on **if**
remote SLAM is ever deployed — it was not re-examined, and nothing here is a
claim that `serve_client()` is fine.

One thing did change in remote's favour under item 2: `pose_message()` now
carries the service's own `device`, so a remote worker reports its real device
instead of the host guessing. That is the acceptance gate's clause, not this
item's scheduling problem.

### Original brief (OPEN, only if remote is active)

`serve_client()` is synchronous: receive a frame, run the worker, send pose,
maybe encode and send a whole mesh, then receive the next (`service.py:37-72`,
`wire.py:46-49`). A large `sendall()` can prevent reading newer input — distinct
from the browser `/ws` backlog now fixed. At minimum, service compute should not
call `sendall()` directly. **Do not claim pose-before-mesh ordering solves this:**
it protects the pose for the frame just processed, not the next frame while the
previous mesh transmits. Not implemented or benchmarked; treat as follow-up
unless on-rig verification shows remote SLAM is the active deployment and its
pose age still violates the contract.

## Item 7 — TSDF saturation polling cleanup ✅ DONE (2026-08-02)

`_SATURATION_CHECK_EVERY = 25`, deliberately matching the adjacent
`_HEADROOM_CHECK_EVERY` because both are the same kind of cost (a device-sync
hashmap read). The 90% warning, the rehash-headroom guard, and BUG-053's
`TsdfCapacityError` extraction ceiling are **untouched** — different code paths,
different constants, and their tests still pin them.

**The cost is stated rather than assumed:** the warning can now fire up to
`_SATURATION_CHECK_EVERY - 1` = **24 integrates late**. That is safe only because
the map grows monotonically, so a stride can delay a warning but can never skip
one, and BUG-035's ~30 frames of headroom at 90% absorbs it. A test drives a fake
hashmap whose true size crosses 90% at call 30 — not a multiple of 25 — and
asserts the warning stays silent until the poll at call 50, then fires exactly
once.

### Original brief (OPEN, low priority)

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
