# Item 4 — matched CUDA ICP / raycast study

**Status:** complete (study). **Verdict:** the GPU-resident translation solve is
**not viable** on Open3D 0.19 — it is correct to float round-off, costs
+2.37 ms/frame (+44% of a SLAM step), and holds the GIL almost solidly. The
measurable win runs the other way: move the ICP nearest-neighbour search **off**
the GPU. `6dof`, the only variant that is faster per call, is disqualified on
accuracy (8.0 ± 3.6 m loop closure against the baseline's 0.67).

Executes item 4 of
`docs/superpowers/plans/2026-08-02-slam-compute-and-transport-followups.md`.
Item 5 (landing a winner) is gated on this document.

> **Item 5 landed 2026-08-02 — see the "Item 5 outcome" section at the end
> before quoting any number from §E.** The recommendation held (bit-identical,
> shipped, `[slam] icp_device`), but the **size did not**: re-measured with the
> interleaved A/B it is −1.9% of a SLAM step on one capture and **+0.7%, i.e.
> nothing**, on another. §E's −10% was a quiet-box figure and a quiet box was
> never available again.

New instrument: `host/tools/slam_icp_bench.py` / MCP `slam_icp_bench`
(`--what api|icp|raycast|ensemble`). The shipped `roomscan/slam/odometry.py` is
**unmodified**; the candidate solver lives in the tool and reaches `Mapper` via an
in-memory `register()` shim, so a later before/after stays interpretable.

---

## 0. Correction to the plan's evidence table

The plan records `o3d.core.cuda.is_available()` as **false** in the agent venv and
concludes CUDA changes cannot be validated here. **That is no longer true**, and
this study ran on real CUDA.

Re-confirmed at the start of this session in `host/.venv`:

| | |
|---|---|
| Open3D | 0.19.0 |
| `o3d.core.cuda.is_available()` | **True** |
| `o3d.core.cuda.device_count()` | 1 |
| `nvidia-smi` | NVIDIA RTX 2000 Ada Generation Laptop GPU, 8188 MiB, driver 610.43.02 |
| GPU state before every timed run | 0% utilization, 18 MiB used |

**Scope caveat, stated because the plan demands it:** this is the *agent
sandbox's* GPU, and it is the same physical card the owner's `roomscan-web`
(PID 5925) shares. It is a genuine CUDA device, so a "CUDA cannot be tested
here" result no longer applies — but it is a **laptop RTX 2000 Ada**, and the
absolute millisecond figures should be re-taken on any other card before being
quoted as rig numbers. The *ratios* between variants, which is what the
recommendation rests on, are what this document asks you to carry forward.

Every timed run in this document was bracketed by `nvidia-smi`. The owner's
server sat at 0% utilization / 18 MiB throughout (it was in real-time
point-cloud mode, which allocates no CUDA) and never competed for the GPU, so **no
run was discarded for GPU contention**.

**One run WAS discarded, for CPU contention** — a concurrent agent session's
headless Chrome appeared mid-measurement at 1270% CPU. Details in §F. The lesson
is worth stating up front: **`nvidia-smi` alone is not enough of an environment
check on this box.** Read `/proc/loadavg` and `top` too, because the variant this
study recommends is the CPU-bound one.

---

## A. API confirmation — which Open3D 0.19 tensor ops sync

**A device tensor is not proof that no synchronization happened**, so the probe
does not ask what an op returns. It uses a deep-queue discriminator: enqueue a
long chain of large matmuls (measured drain ≈ 76–91 ms), then time the candidate
op. Open3D uses one ordered stream, so an op that returns while that queue is
still draining is asynchronous, and an op that costs ~the whole drain
synchronized. The op's cost on an **empty** queue is reported alongside, so
"slow" and "synchronizing" cannot be confused.

Measured on CUDA:0, at the real ICP problem size (2268 points = 54×42):

| Op | empty-queue | deep-queue | drain | syncs? |
|---|---|---|---|---|
| elementwise mul | 0.007 ms | 0.01 ms | 76 ms | **no** |
| broadcast `(N,3)*(N,1)` | 0.010 ms | 0.01 ms | 91 ms | **no** |
| `matmul` (3×3 normal eq.) | 0.015 ms | 0.03 ms | 76 ms | **no** |
| row-sum via `(N,3)@(3,1)` | 0.015 ms | 0.02 ms | 91 ms | **no** |
| integer gather `t[idx]` | 0.024 ms | 0.03 ms | 76 ms | **no** |
| `concatenate(axis=1)` | 0.012 ms | 0.02 ms | 91 ms | **no** |
| comparison `gt` / `.to(float)` | 0.019 ms | 0.03 ms | 91 ms | **no** |
| host→device upload | 0.013 ms | 0.02 ms | 76 ms | **no** |
| `sum(dim=0)` | 0.015 ms | **94.3 ms** | 76 ms | **YES** |
| `sum(dim=1)` | 0.014 ms | **97.6 ms** | 76 ms | **YES** |
| boolean mask `t[mask]` | 0.066 ms | **91.6 ms** | 76 ms | **YES** |
| `nonzero()` | 0.045 ms | **91.7 ms** | 76 ms | **YES** |
| `.item()` | 0.029 ms | **92.4 ms** | 76 ms | **YES** |
| `.cpu().numpy()` (any size) | 0.013 ms | **89.4 ms** | 76 ms | **YES** |
| `Tensor.solve` / `inv` / `lstsq` / `svd` | 0.06–0.17 ms | **87.7–89.5 ms** | 76 ms | **YES** |
| `nns.hybrid_search` | 0.067 ms | **88.4 ms** | 76 ms | **YES** |

Reproduce with `slam_icp_bench --what api` (no capture needed).

### The verdict this forces

**The obvious device-resident formulation is the wrong one.** Boolean-masking
the matches, `sum`-ing the normal equations and calling `solve()` on device —
the natural translation of `odometry.py:64-88` — would add **three** host
synchronizations per ICP iteration rather than remove any. `nonzero()`, the only
way to produce a variable-length selection, syncs too. And note
`nns.hybrid_search` **already** syncs, so the shipped path is not sync-free
today either.

A GPU-resident solve is therefore only possible in a **mask-free, reduction-free
form**:

* invalid correspondences are **weighted to zero** rather than removed
  (algebraically identical — a zero row contributes nothing to either
  normal-equation term — and it keeps every tensor at a fixed length);
* the miss index (`hybrid_search` returns `-1`, verified) is clamped by
  multiplying it by the 0/1 count, so the gather is always in range;
* every reduction is expressed as a **matmul**, including the per-row dot
  product (`(n*(p−q)) @ ones3`) and the match count;
* one `(N,4)ᵀ @ (N,6)` product delivers **A, b, the match count and the
  squared-residual sum together**, so the host learns all four scalars from a
  single 24-float download — **one sync per iteration**, on top of the
  `hybrid_search` sync the shipped path already pays;
* the 3×3 solve and condition check stay in numpy, because Open3D's device
  `solve()` costs 0.079 ms against numpy's ~0.01 ms for a 3×3 *and* syncs.

That is variant 3 below. So the answer to (A) is: **yes, the API supports a
GPU-resident translation solve — but only in a shape chosen specifically to
dodge the syncs, and it still cannot avoid two per iteration.**

---

## B. The matched ensemble — speed and blocking cost

`slam_icp_bench --what icp` records every `(source, model, init)` triple a real
replay hands to ICP, then runs all four solvers over **those same triples**, with
the GIL-starvation watchdog running.

`captures/roomSweepFull20260730.bin`, first 1200 frames → **1199 ICP calls**,
CUDA:0, 54×42, `max_dist` 0.05, `max_iter` 6:

| Variant | p50 | p90 | max | mean fitness | mean rmse |
|---|---|---|---|---|---|
| `translation` (shipped, CUDA NNS) | **3.372 ms** | 4.622 | 6.047 | 0.91265 | 0.010734 |
| `6dof` (Open3D tensor ICP) | 2.820 ms | 3.600 | 6.132 | 0.91536 | 0.017467 |
| `gpu_translation` (device-resident) | **5.413 ms** | 6.284 | 7.327 | 0.91265 | 0.010734 |
| `translation_cpu_nns` (shipped solve, host NNS) | **2.771 ms** | 3.810 | 6.940 | 0.91265 | 0.010734 |

**Do not size any work off this table's absolute milliseconds.** Repeating the
identical measurement on `coffeeRoomCircuitNoMnt.bin` in two different sessions
gave `translation_cpu_nns` p50 **2.519 ms** and then **3.594 ms** — a 43% swing
for the same code on the same inputs. The isolated microbenchmark is not stable
enough to size a change; §E uses an interleaved, paired, whole-pipeline A/B
instead. What the table *does* establish reliably is the **ordering**, which
reproduced in every session: `gpu_translation` is always the slowest by a wide
margin.

### GIL blocking cost — and why the obvious metric lied

The watchdog first reported `gpu_translation` at "10.3% starvation", i.e. better
than some stages that ran fine. That number was wrong, and finding out why is
one of this study's results.

**Summing tick lateness under-reports precisely when starvation is total**: if
the measured code holds the GIL continuously, the watchdog thread barely runs,
so there are almost no samples to sum. Measured directly: over a 10.93 s
`gpu_translation` stage the watchdog got **1 tick** where ~2186 were due, and
that single tick was 2998.9 ms late.

The instrument now reports `tick_share` = ticks landed / ticks due, which is the
figure to read. On `coffeeRoomCircuitNoMnt.bin`, 1978 ICP calls:

| Variant | `tick_share` | ticks / due | worst stall | (legacy) starved % |
|---|---|---|---|---|
| `translation` (shipped) | **0.953** | 1470 / 1542 | 2.0 ms | 4.7% |
| `6dof` | **0.989** | 1334 / 1349 | 0.2 ms | 1.1% |
| `gpu_translation` | **0.058** | 127 / 2209 | **3981.6 ms** | 66.7% |
| `translation_cpu_nns` | **0.951** | 1553 / 1632 | 4.2 ms | 4.9% |

`gpu_translation` lets other Python threads run **5.8% as often as they should**,
and produced multi-second whole-process freezes in every session that measured it
(598.7 ms, 1626.8 ms, 2998.9 ms, 3981.6 ms) even though **no single `register()`
call exceeded 9 ms**. For `roomscan-web` that is the BUG-060/BUG-061 failure mode
walking straight back in: a frozen asyncio loop, a starved reader thread, and a
mesh/pose queue that cannot drain.

The cause is the repo's own standing lesson, applied one level down: the shipped
path's expensive calls are numpy and a handful of Open3D calls that release the
GIL, whereas the device-resident variant issues ~60 pybind tensor ops per
`register()` that do not. **A native call's wall time is not its blocking cost**
— and here even the *starvation percentage* was not its blocking cost.

> `host/tools/slam_stall_profile.py` computes starvation the same way and has the
> same blind spot. Its published numbers (BUG-060: 11.9% vs 94.3%) are unaffected
> — both regimes had ample ticks — but it should gain `tick_share` before it is
> pointed at anything that holds the GIL solidly. Left untouched here to keep
> this study's diff to the tool it added.

### Equivalence — and what the check can actually separate

Per-call, on identical inputs, against the shipped solver. This check
discriminates because an algorithmic error (a sign, a mis-read reduction column,
a dropped weight) moves the answer by 1e-3…1e0 m, while float round-off moves it by ~1e-16 m —
five to sixteen orders apart. A trajectory-only comparison **cannot** do this;
SLAM chaos swamps it.

| Capture | Variant | max abs Δt | p99 Δt | max Δfitness | max Δrmse | `ok` disagreements |
|---|---|---|---|---|---|---|
| roomSweepFull (1199 calls) | `gpu_translation` | 3.38e-15 m | 9.22e-16 m | 0.0 | 6.2e-17 m | 0 |
| roomSweepFull (1199 calls) | `translation_cpu_nns` | **0.0 m** | 0.0 m | 0.0 | 0.0 | 0 |
| coffeeRoomCircuitNoMnt (1978 calls) | `gpu_translation` | 7.57e-16 m | 2.59e-16 m | 0.0 | 5.2e-17 m | 0 |
| coffeeRoomCircuitNoMnt (1978 calls) | `translation_cpu_nns` | **0.0 m** | 0.0 m | 0.0 | 0.0 | 0 |

**The check was proved by reintroducing the defect**, on a scene with genuine
unmatched points (fitness 0.598, so the zero-weight path actually runs):

| Injected defect | Δ vs shipped | separated? |
|---|---|---|
| sign of `b` flipped | 89.8 mm, fitness 0.598 off | **yes** |
| wrong stats column read as the match count | 7.8 mm, fitness 0.598 off | **yes** |
| weight dropped from the normal-equations left factor | 3.6 mm | **yes** |
| index clamp removed | 0.0 m | **no — and it is not a defect**: Open3D wraps a −1 index and the zero weight nulls the row, so the clamp is defensive only |

And the trap this repo has fallen into before: on the **full-match** scene the
"weight dropped" defect moves the answer by exactly **0.0 m**. A check run only
on data where everything matches has no power over the weighting at all. Both the
tool's captures and the shipped unit test now include partial-match data, and a
separate test asserts that scene really does contain misses.

`gpu_translation` is the same algorithm to float64 round-off.
`translation_cpu_nns` is **bit-identical** over 3177 calls across both captures —
which is expected rather than lucky: the shipped translation path already
downloads source positions, target positions and target normals and does all its
arithmetic in numpy, so the *only* thing the device selects is which hybrid index
runs the neighbour search, and both returned identical neighbours. (Measured, not
guaranteed: a tie-break difference is possible in principle and did not occur
here.)

### Why device-residency loses

The problem is **2268 points** (54×42), and the solve is a 3×3. One ICP iteration
on device costs ~10 kernel launches at 10–25 µs each; the same iteration in numpy
costs tens of microseconds *total*. Moving it onto the GPU buys nothing at this
size and pays launch overhead six times per call — and, worse, pays it through
pybind calls that hold the GIL, which is what produces the 0.058 `tick_share`
above. The GPU is genuinely useful for the TSDF integrate/raycast; it is not
useful for a 3×3 normal-equations solve over a 54×42 depth image.

Note this is a **problem-size** conclusion, not an Open3D-quality one: at a much
larger point count the launch overhead would amortise. Nothing here says
device-resident ICP is wrong in general; it says it is wrong for *this* sensor.

---

## C. Accuracy — matched perturbation ensembles

`slam_icp_bench --what ensemble` drives `host/tools/slam_ensemble.py::run_ensemble`
(same deterministic perturbations, same metrics, same `died`/`trailing_lost`
caveats) once per variant, paired by perturbation index.

**Capture:** `captures/coffeeRoomCircuitNoMnt.bin` — the closed-loop circuit that
satisfied the 6.D gate, so `horizontal_closure_m` is genuine drift. **n = 10,
CUDA:0, whole capture.**

The shim's own call counter confirms each variant really ran: **19,768** ICP
calls for all three translation modes (1979 frames × 10 perturbations) and
**16,790** for `6dof` — fewer because a lost frame never reaches ICP, which is
itself the first sign of that mode's trouble.

The baseline reproduces the recorded figure exactly: `docs/mcp-server.md` already
documents `slam_ensemble`'s validation pass on this capture as **0.670 ± 0.154 m**,
against the 0.74 ± 0.19 m in `CLAUDE.md`. That the baseline arm lands on a
number this study did not choose is a useful check that nothing in the harness
perturbed the shipped path.

| Variant | horizontal closure (m) | vertical error (m) | path (m) | median step (ms) | died | worst lost | escalations | blocks |
|---|---|---|---|---|---|---|---|---|
| `translation` (baseline) | **0.6698 ± 0.1537** | 0.0968 ± 0.1307 | 23.42 ± 2.18 | **5.357 ± 0.151** | 0/10 | 0 | 0 | 27,973 |
| `gpu_translation` | **0.6698 ± 0.1537** | 0.0968 ± 0.1307 | 23.42 ± 2.18 | **7.723 ± 0.031** | 0/10 | 0 | 0 | 27,973 |
| `translation_cpu_nns` | **0.6698 ± 0.1537** | 0.0968 ± 0.1307 | 23.42 ± 2.18 | **5.323 ± 0.185** | 0/10 | 0 | 0 | 27,973 |
| `6dof` | **8.0158 ± 3.6026** | 3.6722 ± 3.4221 | 37.23 ± 12.36 | 4.543 ± 1.370 | **3/10** | **1480** | 863 | 42,542 |

Gate verdicts (paired bootstrap CI on candidate − baseline closure, tolerance
0.1537 m):

| Variant | mean Δ closure | 95% CI | tracking ok | **accepted** |
|---|---|---|---|---|
| `gpu_translation` | −1.08e-15 m | [−2.6e-15, +4.9e-16] | yes | **yes** |
| `translation_cpu_nns` | **0.0 m** | [0.0, 0.0] | yes | **yes** |
| `6dof` | +7.346 m | [+5.169, +9.498] | **no** | **NO** |

Three things fall out:

1. **`translation_cpu_nns` is bit-identical over the whole capture** — closure,
   vertical error and path length match the baseline to the last digit in all
   ten perturbations. That is expected rather than lucky (see §B) and it makes
   the accuracy question moot for that variant.
2. **`gpu_translation` is numerically equivalent but costs +2.366 ms/frame**
   (5.357 → 7.723 median step, +44%), with a run-to-run sd of only 0.031 ms. The
   cost is not noise.
3. **`6dof` is disqualified on accuracy, decisively.** It is the fastest per ICP
   call (§B) and it produces **8.0 ± 3.6 m** of closure against the baseline's
   0.67, kills 3 runs in 10, loses up to 1480 frames of 1979, and fires 863 ICP
   escalations. This is exactly the case the plan warned about — "faster but
   changes translation semantics" — and it is worth recording that a
   speed-only reading of §B would have picked it.

The gate also **discriminates**, which is worth stating explicitly: it accepted
two variants and rejected one, so a blanket "everything passes" reading is not
available.

### The tolerance, and where its number came from

The gate is **non-inferiority, not improvement.** `slam.validation.paired_loop_gate`
asks whether a change makes closure *better* (one-sided, CI strictly above zero);
an ICP optimization is not supposed to make anything better, it has to be
indistinguishable. So `slam_icp_bench.paired_equivalence` computes a paired
bootstrap 95% CI on (candidate − baseline) `horizontal_closure_m` and requires
the whole CI to lie inside a band.

**The band is one standard deviation of the baseline ensemble's own
`horizontal_closure_m`** — measured in this study, not chosen after the fact,
and not invented. It is the size of the run-to-run chaos already present in the
metric on unchanged code: anything smaller cannot be distinguished from a re-run
of the shipped path, and quoting a tighter tolerance would be inventing
precision the instrument does not have.

**The knob was verified, not assumed.** The shim counts its own calls per mode
and the ensemble asserts the count is non-zero before reporting. A
monkey-patch that silently failed to apply would report "no difference" — which
is also exactly what a correct, equivalent variant reports, so the two must be
separated by construction. (`gpu_translation`'s counter read 100% of the ICP
calls in every ensemble; the counts are in the JSON.)

---

## D. Raycast host round-trips

`slam_icp_bench --what raycast` builds a **real** map (1200 frames of
`roomSweepFull20260730.bin`, 16,184 blocks — well below BUG-053's 250,000
extraction ceiling), then samples 40 real poses × 3 repeats and times each
segment of `TsdfMap.raycast()` (`tsdf.py:357-400`) plus `Mapper.step()`'s second
download (`mapper.py:283-287`).

| Segment | p50 | p90 | max |
|---|---|---|---|
| `ray_cast` kernel itself | **0.605 ms** | 0.866 | 1.020 |
| download vertex + normal + depth (3×) | 0.049 ms | 0.062 | 0.110 |
| numpy `depth > 0` mask | 0.045 ms | 0.058 | 0.114 |
| rebuild point cloud on device | 0.042 ms | 0.058 | 0.104 |
| **shipped total** | **0.742 ms** | 1.010 | 1.301 |
| `Mapper.step()`'s second download, to count points | 0.016 ms | 0.019 | 0.033 |
| **device-resident alternative** (mask + gather on device, count as metadata) | **1.172 ms** | 1.531 | 23.180 |

**The available win is 0.152 ms/frame** — 0.136 ms of round-trip plus 0.016 ms
of redundant recount — against a 0.742 ms stage in which the `ray_cast` kernel
is 82% of the cost. The whole host round-trip is 18% of the stage and ~0.5% of a
SLAM step.

**And the device-resident version does not collect it: it is 0.43 ms/frame
worse.** The plan's own warning is what happened — "a nominally device-side op
can still force a sync". `nonzero()` is the only way to build a variable-length
selection and it synchronizes, and gathering ~2079 rows on device costs more than
memcpy-ing 25 KB over PCIe. This is a **measured negative result**, not an
untried idea.

Two checks on that claim:

* **The two paths select the same points.** `selection_agrees` compares the
  per-sample valid-point count from the numpy mask against the device `nonzero()`
  selection: identical on every sample. Without that, the timing comparison could
  have been between two different computations (a mask-polarity or reshape-order
  slip would be invisible in a stopwatch).
* **The penalty is a ratio, not an absolute.** A repeat at 800 frames on a
  loaded box scaled everything up (shipped 0.742 → 1.214 ms) but the
  device-resident/shipped ratio was **1.58 both times**. The absolute
  milliseconds here move with the machine; the ratio does not.

---

## E. Recommendation for item 5

### Where a SLAM step actually goes (CUDA:0, 1979 frames)

| Stage | p50 | share of step wall |
|---|---|---|
| `Mapper.step` | 5.392 ms | 100% |
| ` └ register` (ICP) | **3.704 ms** | **70.3%** |
| ` └ raycast` | 0.801 ms | 14.0% |
| ` └ integrate` | 0.482 ms | 9.8% |

ICP **is** the dominant term, so it was the right thing to study. The raycast
round-trips (§D) are 0.15 ms of a 5.4 ms step — worth fixing only as hygiene, and
not by the device-resident route, which measured worse.

### Recommendation: land `translation_cpu_nns`, drop `gpu_translation`, never `6dof`

**Do NOT implement the GPU-resident translation solve (the plan's option 3).**
It is correct — bit-equivalent to float round-off, gate-accepted on a matched
n=10 ensemble — and it is worse on every axis that matters: +2.37 ms/frame
(+44% of the whole SLAM step), `tick_share` 0.058, and repeatable multi-second
GIL-held freezes. Recording this as a **closed question**: the API supports it,
the shape that dodges the syncs was found and implemented, and it still loses.
Do not reopen it without a different Open3D.

**Do NOT adopt `6dof` for its speed.** 8.0 ± 3.6 m of closure, 3/10 runs dead.

**Do land the one-line inversion instead: run the ICP nearest-neighbour index on
the HOST while everything else stays on CUDA.** `Mapper` needs an `icp_device`
separate from its compute `device`, defaulting to `"CPU:0"`, plumbed like
`block_count` (`[slam]` key → `SlamConfig.mapper_kwargs()` → CLI → `SlamRunner` →
`DetailedSlamPreset`; per BUG-062 there must be exactly one place that knows the
field list). `odometry.register` already takes `device` and needs no change at
all.

Why it is safe: the shipped `translation` path **already downloads** source
positions, target positions and target normals and does all its arithmetic in
numpy. The device only selects which hybrid index runs the search. Output was
bit-identical over 3177 ICP calls across two captures and over a full 1979-frame
× 10-perturbation ensemble.

Why it is (modestly) worth doing — **interleaved, paired, whole-pipeline A/B**,
which is the only instrument here that survived scrutiny:

| Environment | step Δ (paired) | register Δ | raycast Δ | integrate Δ |
|---|---|---|---|---|
| Quiet box (n=4 pairs) | **−0.554 ± 0.250 ms (−10.1%)** | −0.970 (−25.5%) | +0.280 (+33.6%) | +0.088 (+18.5%) |
| Box under 12.7 cores of external load (n=3 pairs) | **−0.175 ± 0.069 ms (−1.5%)** | −0.485 (−6.0%) | +0.086 (+5.7%) | +0.135 (+11.3%) |

Faster in both regimes, never slower — but the gain is **partly given back** by
raycast and integrate getting slower. Best-supported explanation, directly
observed: with the NN search on the GPU the card sits at 31–35% utilization and
boosts to 2250–2550 MHz; with it on the host, utilization falls to 10% and the SM
clock parks at **1785 MHz**, so the remaining CUDA work runs on a downclocked
card. Under external load both variants ran at 1785 MHz and the penalty shrank to
+5.7%, which supports the clock explanation without proving it is the whole
story — a residual penalty remains at equal clocks, and clocks were not locked
(`nvidia-smi -lgc` needs root here).

**Size the expectation at ≈ −0.2 to −0.55 ms/frame (−1.5% to −10% of a SLAM
step), and require item 5 to re-measure on the target with the interleaved A/B
rather than trusting this figure.** *(Item 5 did, and the top of that range did
not survive: −0.231 ms on one capture, **+0.087 ms — no win — on another**. Read
"Item 5 outcome" instead of this row.)* The effect is small enough that run ordering,
GPU clock state and the box's other tenants all move it by more than the effect
itself. The reason to land it anyway is that it is **free**: bit-identical
output, no new device↔host transfer (it removes one device round trip), no
accuracy risk, and one plumbed parameter.

**Caveat that must ride with it:** it converts GPU wait into CPU work, and
`roomscan-web` runs its asyncio loop, reader thread and broadcaster on the same
CPU. `tick_share` stayed at 0.951 (vs 0.953 shipped), so it does not hold the GIL
any longer — but the loaded-box row above is the honest picture of what it is
worth when CPU is the scarce resource. Make the default overridable rather than
hard-coded.

**Raycast (item 4's folded-in question): change only the recount, if anything.**
Drop `Mapper.step`'s second `positions.cpu().numpy()` (`mapper.py:285`) — it
downloads the whole array to read `.shape[0]`, which `TsdfMap.raycast` already
knows and can return as metadata. That is 0.016 ms/frame and removes a transfer
for free. Leave `tsdf.py:357-400`'s download/mask/re-upload alone: it is 0.136
ms/frame, and the device-resident replacement measured **0.43 ms/frame worse**
because `nonzero()` synchronizes and a 2268-row device gather costs more than a
27 KB memcpy. The CPU-only intrinsic/extrinsic requirement was not touched.

**Also fold in item 7 while in this code** (`_check_saturation` cadence). Do not
touch `MeshPrep`'s decimation (rejected), and do not raise `block_count` in
response to anything here (BUG-053).

---

## F. What could not be measured, and why

**A discarded run, reported rather than hidden.** A second interleaved A/B at
n=8 pairs was thrown away: partway through, a **concurrent agent session's
headless Chrome (SwiftShader, PID 158798, 1270% CPU, 105 min of CPU time)**
appeared and every stage of *both* variants slowed ~2.3× (step 5.3 → 12 ms) with
the GPU at 51 °C and the SM clock unchanged, so it was not thermal. Its paired
delta came out +0.169 ± 1.209 ms — noise. The n=3 "loaded box" row in §E was then
taken deliberately under that same steady load, which is why its standard
deviations are tiny (0.007–0.063 ms): a *steady* competitor is controllable, a
*changing* one is not. **This box has other tenants; check `/proc/loadavg` and
`top` before and after any CPU-sensitive timing here, not just `nvidia-smi`.**

**Not measured, and why:**

* **Live behaviour.** Everything here is offline replay. Nothing was measured
  through `roomscan-web`, because the owner's server (PID 5925) was live with a
  device attached and BUG-060 established that connection count to `:8000` is
  itself a performance variable. The pose-age / mesh-backlog contract from
  BUG-061 was therefore *not* re-verified; item 5 must do that after landing.
* **Any other GPU.** One card, an RTX 2000 Ada Laptop. The GPU-clock mechanism in
  §E is a laptop-DVFS behaviour and may not transfer. Clocks could not be locked
  (`nvidia-smi -lgc` needs root in this container).
* **The multi-second `gpu_translation` freezes were not root-caused.** They
  reproduced in all four sessions that measured the variant (598.7 / 1626.8 /
  2998.9 / 3981.6 ms) and never landed inside a `register()` call (per-call max
  9 ms, inter-call gap max 0.0 ms). Open3D's cached CUDA allocator doing bulk
  maintenance under ~120k small allocations is the obvious suspect and is
  **unproven** — recorded as a hypothesis, in the style BUG-035 asks for. It did
  not need root-causing to reach the decision, since the variant is rejected on
  three independent grounds.
* **`translation_cpu_nns` under a real live server's CPU load.** The §E loaded
  row used a sibling agent's Chrome as the load generator, not `roomscan-web`
  doing live SLAM plus broadcast. Directionally informative, not the real thing.
* **Long-scan memory / extraction-ceiling guards were not re-run.** No variant
  here changes what is integrated (block counts were identical: 27,973 across all
  three translation variants), and every benchmark stayed at 16k–43k blocks,
  far below BUG-053's 250,000 refusal. Item 5 must still re-run them, per the
  plan.
* **Multi-scale ICP / `small_gicp` / any other solver** was out of scope: item 4
  names three candidates and this compares those three plus the host-NNS
  inversion.

**Instrument caveat carried forward.** `slam_stall_profile`'s
`starved_pct_of_wall` shares the blind spot corrected here (§B): it can read
*low* precisely when starvation is total. Its published BUG-060 numbers are safe,
but it should gain `tick_share` before being pointed at GIL-solid code.

---

## Item 5 outcome — as landed, 2026-08-02

**Both recommended changes shipped; nothing else was touched.** The closed
questions above stayed closed: no GPU-resident solve, no `6dof`, no change to
`tsdf.py:357-400`'s download/mask/re-upload, no `MeshPrep` decimation, no
`block_count` change.

1. `Mapper.icp_device` (default `"CPU:0"`, `None` = follow `device`), plumbed
   `[slam] icp_device` → `SlamConfig.mapper_kwargs()` → CLI (`--icp-device`) →
   `SlamRunner` → `DetailedSlamPreset`. `odometry.register` is byte-for-byte
   unchanged, exactly as §E said it would be. It applies to `translation` only:
   `Mapper._register_device` hands `6dof` the compute device, because Open3D's
   own tensor ICP runs where its point clouds live and a mismatched device
   there is a bug, not an optimization.
2. `Mapper.step`'s second `positions.cpu().numpy()` is gone —
   `TsdfMap.raycast(..., with_count=True)` returns the count it already
   computed from its own `depth > 0` mask.

**Three re-listings of the `Mapper` field list were removed on the way**, per
BUG-062's rule that exactly one place may know it: `slam/cli.py::_run` (all
eighteen knobs by hand), `DetailedSlamPreset.mapper_kwargs` (now overrides
`SlamConfig.mapper_kwargs()`), and the two measurement rigs
`slam_gpu_memory.py` / `slam_stall_profile.py` — which existed to profile the
**shipped** pipeline and had quietly stopped doing so. Detailed's only
behavioural delta from that is newly forwarding the four `stationary_*` tuning
values, which are inert because it pins `stationary_hold=False` (asserted, not
assumed).

### Prediction vs. measurement

| | §E predicted | measured (as landed) |
|---|---|---|
| Output equivalence | bit-identical | **bit-identical** — 0.0 m over 11 paired whole-capture replays on two captures, identical block counts, 0 lost, 0 escalations |
| `register` | −0.49 to −0.97 ms | **−0.53 ± 0.09** (coffee, n=4) / **−0.24 ± 0.08** (sweep, n=7) |
| raycast / integrate give-back | +0.09/+0.14 to +0.28/+0.09 ms | **+0.07 / +0.14** (coffee) and **+0.08 / +0.15** (sweep) |
| **whole step** | **−0.2 to −0.55 ms (−1.5% to −10%)** | **−0.231 ± 0.130 ms (−1.9 ± 1.1%)** on `coffeeRoomCircuitNoMnt`; **+0.087 ± 0.121 ms (+0.7 ± 1.0%)** on `roomSweepFull` |
| `tick_share` | 0.951 vs 0.953 shipped | **0.886 vs 0.901** (both arms lower — this box was loaded; see below) |

**The predicted win reproduced on one capture and did not on the other, and
that is the headline.** New pass `slam_icp_bench --what ab` (interleaved,
paired, whole-pipeline, alternating within-pair order), CUDA:0:

| Capture | frames | blocks | pairs | step Δ (paired) | register Δ | raycast Δ | integrate Δ |
|---|---|---|---|---|---|---|---|
| `coffeeRoomCircuitNoMnt.bin` | 1979 | 27,742 | 4 | **−0.231 ± 0.130 ms (−1.9%)**, 4/4 negative | −0.529 ± 0.087 | +0.075 ± 0.023 | +0.142 ± 0.009 |
| `roomSweepFull20260730.bin` | 3525 | 52,161 | 7 (3+4, two sessions) | **+0.087 ± 0.121 ms (+0.7%)** | −0.235 ± 0.082 | +0.078 ± 0.015 | +0.154 ± 0.014 |

So: **`register` is reliably faster on both** (t = −12.2 and −7.6 on the paired
differences), and on the larger map raycast + integrate take all of it back.
On `roomSweepFull` the change is **neutral** — the +0.087 ms mean is 1.9 SEM
from zero over 7 pairs, i.e. not distinguishable from nothing at this
instrument's resolution, and certainly not the −0.2 to −0.55 ms §E sized.

**Why the two captures differ is NOT explained here.** The give-back is the
same size on both (+0.22 ms of raycast+integrate); what changes is the
`register` saving, −0.53 vs −0.24 ms. The larger map is the obvious suspect and
is **unproven** — recorded as an observation, in the style BUG-035 asks for.

### Two corrections to how §E's stage split should be read

* **The stage-level deltas partly measure the sync point moving, not work
  appearing.** With the NN index on CUDA, every `hybrid_search` drains the
  device queue inside `register`; with it on the host, nothing drains it until
  raycast's `.cpu()`. So some of the +0.22 ms that shows up in raycast and
  integrate is cost that used to be billed to `register`. **Only the whole-step
  delta is trustworthy** — which is the same lesson as §B's `tick_share`, one
  level down.
* **The GPU-clock explanation could not be tested here and is not needed for
  this result.** §E attributes the give-back to the card dropping 2250–2550 →
  1785 MHz once the NNS leaves it. On this box **both arms ran at 1785 MHz in
  every one of the 22 timed replays** — the load below pins the clock — so a
  give-back of the same size appeared *at equal clocks*. That does not refute
  §E's mechanism on a quiet box; it does mean the give-back is not only clocks.

### Environment — every number above is from a loaded box

A sibling session's headless Chrome (PID 158798, SwiftShader) sat at a **steady
1205–1208% CPU** — ~12 of 26 cores — for the entire item-5 session, and it holds
two `/ws` connections to the owner's live server. It never varied, so it is the
*controllable* kind of contamination (§F), and the paired interleaved design is
what makes the comparison valid under it. But it inflates every absolute number:
step p50 was **11.5–12.3 ms** here against §E's quiet-box **5.4 ms**, a 2.2×
tax. `loadavg` ran 14–25 throughout; GPU was idle before every run (0%, 18 MiB)
and never contended.

**A quiet-box measurement was not obtainable and is therefore absent, not
estimated.** §E's −0.554 ms quiet-box figure is neither confirmed nor refuted.

### Equivalence — and that the check can separate

Whole-trajectory, per frame, between arms, on real captures: **0.0 m** in all 11
pairs, plus identical path length, identical final block count, identical
tracking-lost and escalation counts. The check has power: `frac_frames_with_misses`
is **0.95 / 0.98** and mean fitness 0.856 / 0.896, i.e. ~10–14% of source points
are genuinely unmatched on nearly every frame, so the correspondence handling is
actually exercised (§B's trap: on a full-match scene a dropped-weight defect
reads 0.0 m).

**Proved by reintroducing a defect.** Making `_translation_icp` stop one
iteration early when its device is CPU — a difference the two arms would
otherwise never expose — moved the check to **0.1034 m** with
`bit_identical: false`, on 300 frames. Restored and re-verified. The same
treatment was applied to each shipped test: dropping `icp_device` from
`mapper_kwargs` fails four tests naming the key; passing `self._device` to
`register` fails `test_icp_device_is_what_step_hands_to_register`; restoring
the second download fails `test_step_does_not_re_download_positions_to_count_them`;
counting pre-mask fails both raycast-count tests.

### Guards

* **Long-scan VRAM boundedness — PASS.** `slam_gpu_memory.py --synthetic
  --frames 1500 --mesh-every 5` on CUDA:0 with the new default (the rig now
  prints `icp_device=CPU:0`, so it is provably the shipped configuration):
  tail growth **0.0123 MiB/frame**, peak 1771 MiB above baseline, 301 cache
  releases, 0 lost frames, step p50 10.7 ms. The 6.G failure was 5.13 MiB/frame
  synthetic and 10.34 MiB/frame on a real sweep, so this is flat.
* **Extraction ceiling — PASS, and untouched by construction.** Every A/B pair
  ended at *identical* block counts in both arms (27,742 and 52,161), far below
  BUG-053's 250,000 refusal; nothing here changes what is integrated. The
  refusal itself still fires (25 `test_slam_tsdf.py` ceiling/capacity/extraction
  tests green), and the 301 mesh extractions in the VRAM guard exercised the
  extraction path end to end under the new default.
* **Suite:** 1505 passed, 1 skipped (the permanent Windows-only skip), from a
  1488/1 baseline — 17 new tests.

### Not measured (carried forward)

* **Live behaviour through `roomscan-web` was not verified**, so BUG-061's
  pose-age / mesh-backlog contract is *still* unre-checked after a compute
  change — §F handed this to item 5 and item 5 could not do it either. The
  owner's server (PID 5925) is live with a device attached and was not to be
  restarted, and no new browser session was to be started (connection count is
  itself a performance variable, BUG-060). This is the one acceptance gate from
  the follow-ups plan that remains open, and it needs an owner-supervised window.
* **A quiet box** — see above.
* **Why the saving differs between captures** — observed, not explained.
* **Any other GPU.** Still one RTX 2000 Ada Laptop; clocks still cannot be
  locked (`nvidia-smi -lgc` needs root).

### Recommendation, restated after measuring

**Keep the default at `"CPU:0"`.** It is free (bit-identical, one fewer device
round-trip, no accuracy risk), it is a measured win on one real capture and a
wash on the other, and it is now a documented knob rather than a hard-coded
choice — `[slam] icp_device = "CUDA:0"` restores the old behaviour for anyone
whose CPU is the scarce resource. But **do not carry §E's −10% forward as the
expected number**: on this hardware, under real load, the honest expectation is
**between −2% and 0% of a SLAM step**, and which end depends on the map size.
