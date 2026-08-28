# Rust migration feasibility report

**Status:** Research / decision record, 2026-08-27. No implementation. Evidence gathered by six
parallel investigations (docs + native-engine spec digest; transport/protocol analysis with a Rust
parser spike; SLAM frame-path measurement on CPU:0 and CUDA:0; web/MCP/splat layer audit; 64-defect
historical classification + 40-invariant audit; Rust ecosystem survey against crates.io on
2026-08-27). Spike code: `host/tools/spikes/rust-rscn/` (parser), `host/tools/spikes/stage_profile.py`
(per-stage SLAM profiler). Raw spike numbers were produced on this box (i7-13850HX, RTX 2000 Ada
8 GiB passed through to the LXC, Open3D 0.19 CUDA — **not** GPU-less for compute; only rendering is
llvmpipe).

The decision weighting requested: (1) compile-time prevention of meaningful bugs, (2) maintainability,
(3) streaming/concurrency reliability, (4) ecosystem compatibility, (5) agentic ergonomics,
(6) performance, (7) deployment.

---

## Executive conclusion

**Recommendation: Stay primarily Python. Do not start a Rust core, and do not start the C++
`roomscan-native` either. Encode the invariants that actually bit this project as Python domain
newtypes under strict Pyright, and keep one small, killable Rust spike on the shelf for the single
boundary where Rust has a non-substitutable advantage (a GIL-free SLAM front-end), to be run only if
the GIL-starvation problem survives the two cheaper fixes below.**

Concrete target architecture (unchanged in language, changed in typing):

- Python remains the application, web, MCP, SLAM orchestration and offline-3DGS layer.
- `roomscan.protocol` / `sensors` / `slam.frames` / `sensor_time` gain a small **typed-domain layer**
  (`McuMicros`, `LsmTicks`, `Instant`-style host clock wrappers, `Millimetres`/`Metres`, `Bearing` vs
  `EulerYaw` vs `Twist`, `Pose[World,Cam]`-style frame-tagged arrays, `Option`-style validated
  `EnvSample`, one `WireVersion` enum, one `IcpMode`/`Display` enum), enforced by **Pyright strict**
  in CI on those modules first.
- Open3D (`VoxelBlockGrid`, raycast, NNS, marching cubes) stays as the shipped mapping engine; the
  vendor C transform stays behind ctypes.
- The existing out-of-process `SlamService` (`slam/service.py`, `remote.py`, `wire.py`) is the
  already-built answer to "native calls hold the GIL" — it needs a credit scheme and stage-timing
  passthrough, not a language.

Why not Rust now, in one paragraph: the measured pipeline already meets budget on the production
device (step p50 12.7 ms / p95 21.8 ms on CUDA:0 against 33.3 ms); Python interpreter overhead in the
hot path is ≤1–2 ms/frame (Amdahl ceiling ≈1.15×); every documented responsiveness failure was a
GIL-holding **C++** call or an unbounded queue, neither of which Rust changes; there is **no Rust TSDF /
voxel-block-grid, no Open3D or nvblox binding, no maintained GICP**, so a Rust core would be a
three-language Python↔Rust↔C++/CUDA system; and of 64 classified host defects, the ~35 % a type
system could catch are almost all clocks/units/frames/optional-argument bugs that Python `NewType` +
frozen dataclasses + Pyright strict catch at near-zero migration cost. Rust's *unique* residual
(no default arguments, exhaustive `match` enforced by the compiler, `Instant`≠`SystemTime` in std,
`!Send` handles) is real but worth perhaps 5–8 of 64 historical bugs — not a rewrite.

---

## What problems are we actually trying to solve?

**Measured problems (evidence in repo):**

| Problem | Evidence | Language-related? |
|---|---|---|
| Event-loop starvation by GIL-holding native calls | `slam_stall_profile` CUDA:0: loop unschedulable 20.1 % of wall, worst stall 102 ms; `prepare_packet` 14.9 %; CPU:0 raycast tick_share 0.21, mesh 0.026 (451 ms freezes). BUG-052 (1.11 s), BUG-060 (mapper fed at 5 fps) | Partly. Cause is Open3D C++ inside CPython. Rust helps only by *not being in CPython*; the same C++ calls would still block a Rust thread — they'd just not block the web loop. A subprocess (already exists) achieves the same. |
| Unbounded / unobserved queues | BUG-061 (37 MB in flight), #217 (10.9 MB write buffer) | No. Architectural; tokio/tungstenite sinks are also unbounded unless someone chooses `n`. |
| Wrong-frame / wrong-unit / wrong-clock defects | BUG-039/048/051/058 (yaw≠bearing, 4 instances, 2 shipped), BUG-050 (`time()−monotonic()`), BUG-055 (shell units as metres), BUG-026 (body cloud drawn as world), BUG-082/083 (angle wrap, 0 Pa sentinel) | **Yes — this is the real correctness case.** Catchable by domain newtypes in Python *or* Rust. |
| Optional-argument / non-exhaustive-forwarding defects | BUG-044 (8 call sites omitting args), BUG-062 (13 of 18 config fields silently dropped), BUG-071 (missing enum case in reverse map), argless-defaults trap | Yes — Rust catches these structurally; Python catches with Pyright + `kw_only` + exhaustive-destructure discipline, less automatically. |
| Frame loss on capture (2.3–9.4 %) | BUG-049/#60 | No. Board-to-file, missing observable. |
| Mesh-extraction ceiling (~260k blocks aborts) | BUG-053 | No. Open3D vendor wall; Rust has no replacement for the grid. |

**Hypothetical advantages that do not apply here:** protocol-parse CPU (measured 0.3–0.4 % of a core),
allocation pressure (~50–70 objects and ~45 KB memcpy per 30 Hz frame), data races on shared state
(every hand-off is already a single Mutex-guarded latest-wins slot — no race-class bug in the history),
use-after-free (none possible in the Python layer; the one native precondition bug, BUG-007, was in
vendor C).

---

## Current performance anatomy

Capture `coffeeRoomCircuitNoMnt.bin`, first 1800 depth frames, shipped `[slam]` config
(`icp_mode=translation`, voxel 0.01, `icp_device=CPU:0`, `block_count=160000`). n=1799.

**Per `Mapper.step` (ms: mean / p50 / p95 / p99)**

| Stage | Lang/lib | GIL | CPU:0 | CUDA:0 |
|---|---|---|---|---|
| decode + vendor transform (reader thread) | Python framing + ctypes C | released in C | 1.29 | 1.52 |
| gate + deproject + source cloud | numpy → O3D tensor | numpy releases; tensor ctor holds | 0.95/1.09/1.40/1.47 | 0.97/1.07/1.50/1.59 |
| IMU prior | numpy scalars | holds (µs) | ~0.05 | ~0.05 |
| raycast | Open3D C++ | **holds** | **23.8/23.5/42.8/44.5** | 2.6/2.2/7.3/9.5 |
| registration (translation ICP) | O3D NNS + numpy 3×3 solve | O3D holds; numpy releases | 7.9/7.7/13.7/23.1 | **8.4/7.8/15.6/33.6** |
| ├ NNS search (≈6 calls) | O3D | holds | 2.4 mean | 2.5 mean |
| ├ NNS index build | O3D | holds | 0.75 | 0.75 |
| └ remainder (3 downloads, uploads, einsum/eigh, Python) | mixed | mixed | ~4.8 | ~5.1 |
| TSDF integrate | Open3D C++ | **holds** | 1.57/1.51/2.08/3.67 | 1.13/1.17/1.42/3.79 |
| other Python (gates, ZUPT, hold, bookkeeping) | Python | holds | 0.20/0.20/0.36/0.40 | 0.19/0.19/0.34/0.40 |
| **total step** | | | **34.5/33.6/55.7/60.3** | **13.3/12.7/21.8/41.1** |

**Off-step, every 5 integrated frames (n=361), mean/p50/p95/max ms**

| | CPU:0 | CUDA:0 |
|---|---|---|
| mesh extract (marching cubes) | 192/144/428/450 | 11.1/10.1/22.1/141 |
| `prepare_packet` (to_legacy, normals, shading, wall split) | 441/339/1180/1329 | 407/345/1085/1298 |
| `pack_mesh` | 26.6/16.6/76.4/113 | 24.4/17.5/73.7/107 |

Final mesh 735–749 k verts → 37.7–38.5 MB MESH payload. Live path throttles prep by bytes
(`session.py:3264-3269`, 12 MB/s governor) so at this size prep runs every ~3.2 s on its own thread.

**GIL / loop availability (shipped `slam_stall_profile`, CUDA:0):** starved 20.1 % of wall, worst
102.5 ms, tick_share 0.799 overall; per stage prep 14.9 % (0.82), pack 2.2 %, step 2.0 %, mesh 1.1 %.
Sub-stage tick_share on CPU:0: raycast 0.21, mesh 0.026, integrate 0.012, icp 0.64.

**Web/transport:** `/ws` POINT_CLOUD ~54 KB/frame at 30 Hz per client; roomscan-web measured at
27.7 % of one core; RSCN parse 17.5–26 µs/frame in Python (0.3–0.4 % of a core at ~155 wire
frames/s); thin render 17 ms/frame GIL-free; JPEG <0.8 ms (TurboJPEG). Pose age on a keeping-up
client p50 1.1 / p95 4.8 ms; browser display latency 58–159 ms.

**Prior in-repo numbers agree:** audit 2026-08-18 total 8.5 ms/frame CUDA; live rig ~7 ms; 6.G p50
6.1→8.7 ms on long maps; small_gicp spike: shipped solve 9.66 ms/pair vs sg-GICP 2.19 ms @ 8 threads.

**Amdahl, per step, CUDA:0 (production device):**

- Registration free: 13.3 → 4.9 ms (2.7×) — but the path already meets budget; the win is p99
  (41 ms) and headroom, not fps. Realistic (small_gicp-class 4× on the stage): 13.3 → ~7 ms.
- All Python orchestration free: measured pure-Python ≤0.2 ms + unmeasured interpreter share inside
  ~5.8 ms of mixed stages. Upper bound 1.8× if *all* of it were interpreter (it is not — einsum,
  eigh, tensor uploads are C). Realistic interpreter cost 1–2 ms → **≤1.15× on CUDA, ≤1.06× on CPU.**
  This is the spec's own N3 kill criterion, and it holds.
- GIL blocking of the web loop: up to 20 % (CUDA) / 36 %+ (CPU) of loop availability and the
  100–450 ms stalls are recoverable by moving Open3D calls out of the interpreter — **by any means**,
  including the existing subprocess service.

---

## Component migration matrix

Legend for recommendation: **KP** keep Python · **RR** rewrite in Rust · **RL** Rust library behind
Python · **CL** existing C/C++ library behind Python · **RD** separate Rust daemon · **DF** defer.

| Subsystem | Now | Frame-budget share | Concurrency | Copies | GIL | Py-eco dep | GPU | FFI | Rust feasibility | Perf gain | Reliability gain | Difficulty | Eco risk | Test/migration | **Rec.** |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| USB CDC / UDP ingestion, reassembly, recording, replay | Python (`sources.py`, `reader.py`), pyserial | ~0.4 % core | reader thread; latest-wins slot | 3 × 14.8 KB/frame | trivial | pyserial, none else | no | no | High (tokio, serialport, mdns-sd all mature) | none observable | low (bugs were address provenance, reorder logic, platform DNS) | medium | low | golden `.bin` replay, byte-exact | **KP** |
| RSCN protocol parse / CRC / stream demux | Python `struct` + `zlib.crc32` | 17.5–26 µs/frame | n/a | 2 copies | holds µs | none | no | ctypes cross-check to C parser | Proven: spike byte-exact on 11 fixtures + 8014-frame capture + corrupted input; 4–25× faster | none observable | low; one more mirror of a 4-way-synced spec | low | golden vectors already exist | **KP** (spike shelved; free front door if a Rust front-end ever exists) |
| Sensor decode (IMU/ENV/RAW meta), clocks, quats, mag cal/fusion | Python + numpy (`sensors.py`, `sensor_time.py`, `imufusion.py`, `magcal.py`) | <0.5 ms | reader thread | small | numpy releases | numpy | no | no | High | none | **high if typed** — this is where 9 of the RD-class bugs live | medium | low | unit tests + captures | **KP + domain newtypes** |
| Vendor ToF transform | C 1.5.0 via ctypes shim | 1.15 ms | reader thread | 1 copy | released | none | no | ctypes | Rust would wrap the same C (bindgen) | none | none | low | low | bit-exact golden | **CL (as is)** |
| Deprojection / cloud construction | numpy (`deproject.py`, `cloud.py`) | 0.35–1.1 ms | worker | f64 grid, f32 mask, tensor upload | numpy releases; O3D ctor holds | numpy, Open3D | tensor upload | no | High (glam) | ≤0.5 ms | none | low | low | numeric golden | **KP** |
| Translation ICP (custom NN + 3×3 solve) | O3D NNS + numpy | 7.8 ms p50 (63 % of CUDA step) | worker | 3 downloads + per-iter uploads | O3D holds | Open3D NNS | optional | no | High for the solve (kiddo + nalgebra ~200 lines); GICP would be self-written or cxx to small_gicp | 2–4× on stage; ≤2.7× step; fps unchanged | none (gates/escalation logic identical) | medium; must preserve gates, cond cap, ZUPT, hold, null behaviour | low for kd-tree; none for GICP | ensemble on null capture + 2 circuits | **DF** — run as the one Rust spike only if E3 below triggers; small_gicp (CL) is the cheaper comparator |
| TSDF integrate / raycast / mesh extract | Open3D `VoxelBlockGrid` CUDA | raycast 2.2, integrate 1.2, mesh 10 ms (CUDA) | worker | grid device-resident; `.cpu()` per extract | **holds** | Open3D | yes | pybind (Open3D) | **None** — no Rust TSDF, no Open3D/nvblox binding; own CUDA kernels via cudarc = new subsystem | n/a | n/a | very high | high | n/a | **CL (Open3D as is)**; nvblox per #114 gates |
| SLAM orchestration (`mapper.py`, `worker.py`, gates, baro, ZUPT) | Python | 0.2 ms | worker thread, slots | trajectory copy/frame | holds µs | numpy | no | no | High | ≤1.15× | **medium if typed** (pose direction, local-frame raycast, lost-pose) | high (preserve chaos-sensitive behaviour) | low | ensemble scoring, null capture | **KP + typed frames/poses** |
| Mesh prep / pack | O3D `to_legacy` + normals (C++) + numpy | 345 ms p50 per mesh, 14.9 % loop starvation | MeshPrep thread | to_legacy + f32 copies | mixed 0.82 | Open3D | no | no | Partial (normals/shading in Rust; to_legacy is O3D) | modest | **high for loop availability — but subprocess gives the same** | medium | low | byte-exact MESH | **KP**, move off-interpreter via existing service |
| `/ws`, `/ws-mesh`, `/ws-thin`, REST, static, mDNS, lifecycle | FastAPI/uvicorn/websockets/zeroconf, 9.3 k lines | ~28 % of one core total | asyncio + per-client tasks + credits | write-buffer copies | loop | fastapi, uvicorn | no | no | High (axum, tokio-tungstenite, mdns-sd) | small | modest (bounded per-client mpsc makes #217-class visible); most bugs were vendor defaults/JS | very high (5.3 k lines of session state) | low | 696 tests, mostly state-object unit tests, non-reusable | **KP** |
| Thin-client render | Open3D Filament offscreen + simplejpeg | 17 ms, GIL-free | singleton render thread | RGB565/JPEG | released | Open3D | no (EGL/llvmpipe) | no | Would need own renderer (wgpu) | none | `!Send` would encode the owning-thread rule (#194) | high | medium | pixel probes | **KP** |
| MCP server (62 tools) | `mcp>=2.0` + Playwright/CDP | offline | subprocess-heavy | n/a | n/a | mcp, playwright, and **every analysis function** | no | no | rmcp 3.1 is mature, but ~40/62 tools wrap Python analysis code | none | none | very high | low | 125 tests | **KP** |
| Offline 3DGS | torch, gsplat, pycolmap, transformers | offline, subprocess | n/a | n/a | n/a | total | yes | no | **None** (no gsplat/COLMAP equivalents) | n/a | n/a | n/a | n/a | n/a | **KP** |
| Remote SLAM service IPC (`wire.py`, `remote.py`) | Python TCP, JSON+raw, config re-sent every frame, no credits | not on this box | send/recv threads | ~36 MB `tobytes()` per mesh each way | n/a | numpy | no | no | High (serde + length-prefixed) — but the fix is design, not language | n/a | **high** — this is the real "native core boundary" and it is currently copy-heavy and uncredited | medium | low | replay through service vs in-process, compare poses | **KP, redesign** |

---

## Rust ecosystem fit (2026-08-27)

**Strong / drop-in:** tokio 1.53, serialport 4.10 + tokio-serial 5.5, tokio-tungstenite 0.30, axum 0.8,
mdns-sd 0.21, zerocopy 0.8 + crc32fast 1.5 (framing), pyo3 0.29 + maturin 1.15 + numpy 0.29
(zero-copy `IntoPyArray`, `Python::detach` GIL release), bindgen 0.72 (vendor C), cxx 1.0 (hand-written
C++ bridge), kiddo 6.1 (kd-tree), nalgebra 0.35 / glam 0.33 / faer 0.24, turbojpeg 1.5, rmcp 3.1
(official MCP SDK, 2026-07-28 spec), tracing/metrics/clap/config, cargo-zigbuild for aarch64.

**Usable with caveats:** cudarc 0.19 (kernels written in CUDA C, NVRTC at runtime — the honest CUDA
path; kernels are not Rust), dlpark 0.9-alpha (DLPack, one maintainer), apex-solver 1.4 / factrs 0.3
(young factor-graph solvers, single-author / lab cadence), opencv 0.100 (heavy build), tch 0.26.

**Missing or unusable:** **no TSDF / voxel-block-grid crate; no Open3D binding; no nvblox binding;
no maintained GICP (fast-gicp bindings 0 stars; small_gicp none); no SfM/COLMAP; no SLAM framework.**
autocxx is unmaintained ("looking for a new maintainer"); Rust-CUDA/`cust` is nightly-only and
experimental; `gtsam-rs` is AGPL and dormant.

Consequence: any Rust core that keeps the current mapping engine is Python ↔ Rust ↔ Open3D-C++/CUDA,
with the Rust layer owning ingestion, decode, deproject, ICP-translation and gates, and calling back
into Open3D **through Python** (there is no other binding) — which puts the GIL right back in the
middle of the frame. The only GIL-free Rust design is "write our own sparse TSDF in CUDA C behind
cudarc" (1–2 k lines of kernels plus raycast, marching cubes, hash resize, VRAM bounding), i.e. the
subsystem the project already decided, correctly, to be extremely skeptical of.

---

## Rust vs C++ `roomscan-native`

The C++ proposal (#116; `researchResults.md` §6–8) is: one C++ library/daemon owning UDP/CDC
ingestion, small_gicp odometry, GTSAM iSAM2, nvblox mapping; pybind11 read-only exposure; DLPack
mesh pointers. The design spec gates it as N3 = N1 (small_gicp survives the null capture and wins
≥2 ms in-Mapper) ∧ N2 (nvblox beats the chunked-extract path and the free CPU grid) ∧ profiling shows
Python orchestration is the bottleneck. Triage (2026-08-11) parks all of it behind #110/#60.

| Criterion | C++ `roomscan-native` | Rust core | This report's read |
|---|---|---|---|
| Performance | Native access to small_gicp (2.2 ms), nvblox, GTSAM; single memory space | Same only via cxx shims or by re-implementing; own kernels via cudarc | Equal ceiling; C++ reaches it with less glue. Neither changes fps (already 30). |
| Library access | Direct: Open3D, small_gicp, nvblox, GTSAM, Eigen, OpenCV | None of those natively; every one is FFI | **C++ wins decisively for this domain.** |
| CUDA | nvcc, first-class | cudarc + CUDA-C kernels (fine), Rust kernels not shippable | C++ simpler; Rust acceptable. |
| FFI complexity | pybind11 (known in repo), one boundary | pyo3 (excellent) + cxx/bindgen to each C++ lib (each hand-written) | Rust = two boundaries per feature. |
| Safety | UB reachable (BUG-007 was uninitialised vendor C); sanitizers catch at runtime | No UB in safe Rust; `unsafe` confined to FFI/launch | Rust wins — but the safety-relevant code here is small (parsers, buffers) and mostly stays in Python anyway. |
| Type-level invariants | Strong typedefs, `std::variant`, `std::expected`, concepts — opt-in, non-exhaustive `switch` unless `-Werror=switch`, default args exist | Newtypes, enums with exhaustive `match`, no default args, `Send`/`Sync`, `!Send` handles | Rust wins on the actual bug classes (BUG-044, 062, 071, 194). |
| Build tooling | CMake + vendored deps + pybind; repo already does this for the C shim | cargo + maturin; clean until a C++ lib is pulled in, then CMake anyway | Rust nicer alone; equal once C++ deps enter. |
| Debugging | gdb/sanitizers, well-known | gdb works; sanitizers via nightly | Equal. |
| Deployment | Shared libs (Open3D/nvblox/CUDA runtime) | Same shared libs + Rust binary | Equal. |
| Maintainability | Two languages (Py+C++) | Three (Py+Rust+C++) unless Rust replaces the C++ libs, which it cannot for TSDF/GICP | **C++ wins** as long as Open3D/nvblox/small_gicp are in the design. |
| Agentic ergonomics | Agents handle C++ moderately (SWE-bench Multilingual C/C++ 28.6 %) | Rust-SWE-bench 21–29 % vs Python 70 %; SWE-bench Multilingual Rust 58 %; failures cluster in module structure and trait resolution, not syntax | Mixed evidence; Rust's diagnostics are richer, but its success rates are benchmark-dependent and no better than C++'s on hard sets. |

**Verdict on this comparison:** if a native core is ever justified, it is justified by library access
(small_gicp, nvblox), and that points at C++ — Rust would add a language without subtracting one.
But the native core's own gates are unmet and its premise (Python orchestration as bottleneck) is
measured false. So the practical conclusion is *neither*, and the ranking is: strict-typed Python now;
C++ `roomscan-native` if N1∧N2 ever pass; Rust only for a self-contained front-end that does not need
the C++ libraries (see E3).

---

## Recommended architecture

```
                 ┌──────────────────────────── Python (roomscan) ────────────────────────────┐
 device ──UDP/CDC──▶ sources/reader ─▶ protocol (typed: McuMicros, WireVersion, EventPayload)  │
                 │        │                                                                   │
                 │        ▼ ctypes                                                            │
                 │   vl53l9-transform-c (C, unchanged)                                        │
                 │        │                                                                   │
                 │        ▼                                                                   │
                 │   sensors / sensor_time / imufusion (typed: LsmTicks, Bearing≠EulerYaw,    │
                 │        │                              Field<Raw|Calibrated>, EnvSample)    │
                 │        ▼ latest-wins slot                                                  │
                 │   slam.mapper / odometry (typed: Pose[World←Cam], Cloud<Local>, Metres)    │
                 │        │           ▲                                                       │
                 │        ▼           │ pose/mesh, credited, stage timings preserved          │
                 │   [in-process SlamWorker]   or   [SlamService subprocess — GIL isolation]  │
                 │        │  Open3D VoxelBlockGrid / NNS / marching cubes (C++/CUDA, as is)   │
                 │        ▼                                                                   │
                 │   MeshPrep ─▶ /ws, /ws-mesh, /ws-thin (FastAPI/uvicorn, bounded by         │
                 │               write-buffer check, as is)                                   │
                 │   roomscan-mcp (62 tools, as is)      roomscan.splat (torch/gsplat, as is) │
                 └───────────────────────────────────────────────────────────────────────────┘
 Shelved, killable:  host/tools/spikes/rust-rscn  (PyO3 front-door parser, byte-exact, 0.7 µs/frame)
```

Stays Python: everything above. Becomes Rust: nothing today. Stays C/C++/CUDA: vendor transform,
Open3D, TurboJPEG, Filament, torch/gsplat.

---

## Expected benefit of the recommended path (typed Python + subprocess isolation)

| Dimension | Expected change | Uncertainty |
|---|---|---|
| Frame time | none (already 12.7 ms p50 on CUDA) | — |
| Latency / p99 | subprocess isolation removes 100–450 ms loop stalls; pose-age p95 stays ≤5 ms | medium — the service IPC must gain credits + zero-copy or it recreates BUG-061 |
| CPU | none | — |
| Memory | none | — |
| Reliability | ~20 of 64 historical bug classes (R+RD+SP, minus Rust-only ones) become Pyright errors on the typed modules; the four-instance yaw family becomes unrepresentable | medium — depends on a single blessed `Bearing.of_axis` constructor and on CI actually running strict mode |
| Concurrency | unchanged (already race-free by construction) | — |
| Deployment | unchanged | — |
| Development velocity | small cost (annotations, strict CI on ~6 modules), then net positive for agents (Pyright diagnostics are a compile step agents already handle well) | low |

For comparison, a Rust front-end (E3) would add: registration 7.8 → ~2–3 ms (stage 3–4×; step
13.3 → ~7–8 ms), GIL-free decode/deproject/ICP, and the R-only bug classes — at the cost of a
three-language build, a second parser to keep in sync, and an FFI seam on the exact chaos-sensitive
path the ensemble tooling exists to protect.

---

## Costs

**Typed-Python path:** ~1–2 weeks of agent time across `protocol.py`, `sensors.py`,
`sensor_time.py`, `imufusion.py`, `slam/frames.py`, `slam/mapper.py`, `slam/odometry.py`; Pyright
strict in CI for those files; refactor of the ~15 call sites that pass raw floats for angles/clocks.
New failure mode: false confidence from a newtype whose constructor is wrong (BUG-058 was exactly the
fix being wrong too) — mitigate with one constructor per domain type and keep the AST guard.

**Rust front-end (if E3 triggers):** maturin in the build, pyo3/numpy version lockstep, a Rust copy
of RSCN + sensor decode + deprojection + ICP + gates (~2–3 k lines) with golden-vector and ensemble
parity tests; dual-stack period where both paths ship behind `[slam] engine=`; ARM64 wheels via zig.
New failure modes: view-across-ring-wrap on zero-copy buffers, GIL re-entry when calling Open3D from
Rust-owned threads, `!Send` Open3D handles, and drift between the Python and Rust decoders.

**C++ `roomscan-native`:** as costed in the spec (from-source small_gicp with 3-DoF masks + pybind,
nvblox CMake/Eigen/glog, VRAM eviction authored by us).

---

## Proof-of-concept program (smallest deciding sequence)

| # | Hypothesis | Implementation | Workload | Metric | Baseline | Pass | Kill | Decision |
|---|---|---|---|---|---|---|---|---|
| **E1** Typed-domain Python catches the historical class | Add `McuMicros`/`LsmTicks`/`HostMono`, `Millimetres`/`Metres`, `Bearing`/`EulerYaw`/`Twist`, `EnvSample`, `WireVersion`, `IcpMode`; Pyright strict on 7 modules | Re-apply the diffs of BUG-039/048/051/058/050/055/026/082/083/044/062/071 as patches | # of the 12 that fail type-check | 0 today | ≥8/12 fail Pyright | <6/12 — newtypes aren't paying; stop | Pass → adopt, extend module list. Fail → Rust's type advantage is also unlikely to pay (same mechanism). |
| **E2** GIL stalls are removable without a language change | Run SLAM through the existing `SlamService` subprocess with (a) config sent once, (b) credit per mesh, (c) stage timings forwarded | `slam_stall_profile` on `coffeeRoomCircuitNoMnt.bin` CUDA:0 + `roomSweepFull20260730.bin` | web-loop `tick_share`, worst stall, pose-age p95 on `/ws` | tick_share 0.799, worst 102 ms, pose-age p95 4.8 ms | tick_share ≥0.97, worst <30 ms, pose-age p95 ≤10 ms | pose-age p95 >20 ms (IPC cost exceeds GIL cost) | Pass → close the GIL problem architecturally; Rust loses its only unique runtime argument. Fail → E3. |
| **E3** A Rust front-end (decode+deproject+translation-ICP+gates, PyO3, kiddo) beats the shipped path without changing behaviour | ~1.5 k lines Rust; Python calls `step_front(frame) -> (pose, gates)`; Open3D raycast/integrate stay in Python | `slam_ensemble` n=10 × {shipped, rust} × {`imuTranslationError.bin` null, both circuits} | null fabricated path, `horizontal_closure_m` paired CI, lost frames, in-Mapper step p50/p99 | shipped: null ≤ baseline drift, closure 0.74 ± 0.19 m, step 12.7/41 ms | closure within CI of shipped, null not worse, step p99 ≤25 ms | any drift/closure regression, or step win <3 ms | Pass → ship behind `engine=rust`, stop there (no TSDF in Rust). Fail → archive; C++ N1 remains the native path. |
| **E4** (only if #113 N1 passes) Rust vs C++ for the native core | none — decided by library access | — | — | — | — | — | If small_gicp/nvblox are in, C++; Rust only if E3 passed and the core is Open3D-only. |

**E1 result (2026-08-27, #223): 8/12 rejected — pass, at the threshold.** Tally, pyright messages and
the corpus in `host/tools/spikes/e1_results.md` + `host/tools/spikes/e1_bug_patches/`. Caught: the four
yaw/bearing/twist bugs, the clock split, the unit split, the ENV sentinel, the missing enum arm.
Not caught: BUG-026 (frames not encoded), BUG-082 (`NewType` arithmetic erases to float), BUG-044/062
(default args / kwarg subsets — the Rust-only class). Strict set so far: `domain.py`, `sensor_time.py`.

E1 and E2 are cheap (days), answer the primary motivation (correctness) and the only measured runtime
problem (GIL) respectively, and both are killable. E3 runs only if E2 fails.

---

## Incremental migration plan

Only E3 leads to any Rust in the tree; it is a single stage that ships independently behind a config
switch, with the Python path retained as default and as the rollback. No further stages are
recommended; a second stage (mapping) has no Rust implementation to migrate to.

---

## What should explicitly stay in Python

Web/session state machine (5.3 k lines, 696 tests), MCP server (62 tools wrapping Python analysis),
offline 3DGS (torch/gsplat/pycolmap), capture analysis tooling, magnetometer calibration and sweep
tooling, SLAM orchestration and gate policy (chaos-sensitive, ensemble-validated), all
`host/tools/` diagnostics.

## What should explicitly not be rewritten

Open3D `VoxelBlockGrid` / raycast / NNS / marching cubes (no replacement exists); vendor
`vl53l9-transform-c` (bit-exact, wrapped either way); TurboJPEG; Filament offscreen renderer;
the RSCN parser (0.3 % of a core; a fourth mirror of the spec is a drift surface, per the repo's own
"mirrored constants drift" lesson).

---

## Risks and unknowns

- CUDA/Open3D binding gap is total: a Rust core either calls Open3D through Python (GIL back in the
  loop) or authors its own TSDF. nvblox/small_gicp/GTSAM are all cxx-bridge work with no community
  bindings.
- pyo3/numpy/maturin minor-version lockstep churns ~2×/year.
- Agent success on Rust is benchmark-dependent (21–58 %); failures cluster in crate/module structure
  and trait resolution — exactly the shape of an FFI-heavy hybrid crate.
- E2's IPC could recreate BUG-061 (uncredited 36 MB meshes each way); it must ship credits and
  stage-timing passthrough or it measures nothing.
- Quiet-box baseline never obtained (step p50 5.4 ms quiet vs 11.5–12.7 loaded); all ms figures here
  carry ~±40 % session-to-session swing (documented in the native spec). Ensembles, not single runs.

---

# Compile-time correctness payoff

Classification of 64 host defects (full table in the investigation transcript; ids reference
`docs/roadmap-history.md` / GitHub): **R** 9 (Rust compiler) · **RD** 15 (Rust domain types) ·
**SP** 4 (strict Python) · **CPP** 0 unique · **T** 16 (test/live probe only) · **A** 20
(algorithmic/architectural). R+RD+SP = 28/64 = 44 % are type-catchable by *some* type system;
of those, ~20 are equally catchable in Python with `NewType`/frozen dataclasses/enums under Pyright
strict; ~8 need Rust specifically (no default args, exhaustive match by default, `Instant`≠`SystemTime`
in std, `!Send`, no uninitialised fields).

| Bug / invariant class | Example in this repo | Python today | Strict Python | Modern C++ | Rust | Worth encoding? |
|---|---|---|---|---|---|---|
| Euler yaw / twist used as a bearing | BUG-039, 048, 051, 058 (4 instances, 2 shipped); AST guard `test_no_new_yaw_twist_consumers` | comment + AST test | `NewType` `Bearing`/`EulerYaw`/`Twist`, no cross `Sub` — **caught** | strong typedef — caught | newtypes — caught | **Yes, #1** |
| Mixed clock domains | BUG-050 (`time()−monotonic()` → 1.78e9 s); `t_us` (MCU) vs stream-13 (LSM ticks) vs host | comments `protocol.py:189`, `sensor_time.py:26` | `HostMono`/`WallClock`/`McuMicros`/`LsmTicks` NewTypes — caught if arithmetic goes through typed helpers | `chrono::steady_clock` vs `system_clock` — caught | `Instant` vs `SystemTime` — caught in std with zero design | **Yes, #2** |
| mm vs m, shell units vs m | BUG-055 (`DEVICE_DIMS`), `depth_scale=1000` at `tsdf.py:201`, `/1000` at `deproject.py:254` | comments | `Millimetres`/`Metres` — caught | `mp-units` — caught | newtype/`uom` — caught | **Yes, #3** |
| Body vs world vs CV vs local-raycast frame | BUG-026 (body cloud drawn as world); `sensors.py:680-691`, `mapper.py:680-691` | comments over bare 3×3 arrays | frame-tagged array wrapper `Cloud[Body]` — caught at call sites | phantom template param — caught | phantom types — caught | **Yes, #4** |
| Pose direction (world←cam vs inverse) | `inv()` at `mapper.py:673,769` is the only marker; no shipped bug yet | comment | `Pose[World,Cam]` with explicit `.inverse()` — caught | same | same | **Yes, #5** (largest unguarded surface) |
| Zero-filled sentinel accepted as a sample | BUG-083 (0.0 Pa → 8.6 m), #93 host half | range check `is_valid_env` | `EnvSample | None` from a validated constructor — caught if consumers must unwrap | `std::optional` — caught | `Option<Pressure>` — caught, exhaustively | **Yes, #6** |
| Angle wrap in subtraction | BUG-082 (359→0 read as motion) | none | `Angle` NewType with wrapping helper — caught only if raw `-` is blocked (Python can't) | operator overload — caught | newtype without `Sub` — caught | Yes (#7) |
| Optional args omitted at call sites | BUG-044 (8 sites), argless-defaults trap | none | Pyright can't flag *omitting* a defaulted arg; `kw_only` + no defaults helps | default args exist | **no default args — caught** | Rust-only; partially via convention |
| Non-exhaustive config forwarding / enum reverse map | BUG-062 (13/18 fields dropped), BUG-071 (`"standby"` missing) | none | Pyright `assert_never` on enums — caught for enums; field forwarding needs `dataclasses.fields()` discipline | `-Werror=switch` — caught for enums | exhaustive `match`, `let S{..}` without `..` — caught | Yes (#8); enum part is cheap in Python |
| Derived alias kept by hand (`ui.mode` vs `ui.display`) | BUG-066, BUG-013 (two bools = 4 states) | none | one enum + property — caught | enum — caught | enum — caught | Yes (#9), Python-sufficient |
| Typestate transitions (Done re-entering worker arm; reset only when stopped) | #186, BUG-056, BUG-091 | `is not None` checks | `Literal` state + narrowing — partial | `std::variant` — partial | typestate/enum — caught | Yes (#10) for the runner state machine; Python enum covers most |
| Owning-thread native handle | #194 (`OffscreenRenderer` aborts cross-thread) | comment + singleton | none | none | `!Send` — caught | Rust-only; one site |
| Uninitialised native field | BUG-007 (vendor C `conf_scaling`) | n/a | n/a | value-init/UBSan — caught | no uninit fields — caught | Vendor code; not ours to encode |
| Source generation guard on per-frame consumers | BUG-090/#101 | runtime check at 9 sites | runtime | runtime | runtime (phantom can't carry a dynamic gen) | No — runtime in any language |
| Unbounded / unobserved queues | BUG-061, #217, BUG-060 | measured via write buffer | no | no | no (tokio sinks unbounded unless sized) | No — architecture |
| Wrong metric / wrong constant / vendor ceiling | BUG-063, 035, 053, 054, 068, 069 | no | no | no | no | No |
| Physical convention (gravity sign, mag dip, det −1) | BUG-059, `GRAVITY_SIGN` unverified | test | no | no | no | No — needs a live observable |
| Vendor/library defaults | BUG-060 deflate, #106 `enableOptionalEffects`, #220 keepalive | no | no | no | no | No |
| Async self-cancel / process lifetime | #219, #221 | guard | no | no | different hazard, not absent | No |

**Top invariants to encode (highest return, in order):** (1) `Bearing` ≠ `EulerYaw` ≠ `Twist` with one
blessed `Bearing.of_axis(Axis[Body])` constructor; (2) three clock newtypes + host `Instant`/wall
split; (3) `Millimetres`/`Metres`; (4) frame-tagged clouds/rotations with composition-checked
multiplication; (5) `Pose[World←Cam]` with explicit inverse at the Open3D boundary; (6) validated
`EnvSample`/`Pressure` replacing zero sentinels; (7) `Angle` with wrapping subtraction; (8) `IcpMode`,
`Display`, `LinkState`, `WireVersion` as single-source enums with exhaustive handling; (9) runner
typestate for `Idle/Building/Done`; (10) `Raw<T>`/`Gated<T>`/`Smoothed<T>` quaternion wrappers where
the choice is load-bearing (`motion.py:113`, `ws_broadcaster.py:670`).

All ten are expressible in Python with `NewType`, frozen dataclasses, `Literal`/`Enum` and Pyright
strict; (1)–(6) are the ones with shipped bugs behind them. Rust makes (7)–(9) *automatic* rather than
disciplined, and adds the owning-thread and no-default-args classes.

### If performance were identical, would Rust still be worth adopting purely for correctness and maintainability?

**Conditional no.** The correctness payoff is real — roughly 28 of 64 historical host defects are
type-catchable — but about 20 of those are catchable by Python domain types under Pyright strict,
which costs a couple of weeks and no new language, toolchain, FFI seam or agent-skill shift. Rust's
unique residual (~8 defects across four mechanisms) is worth having but does not justify a
three-language stack whose middle layer cannot own the mapping engine, cannot bind Open3D, and would
have to keep a second copy of a four-way-synchronised protocol. Maintainability moves the wrong way:
today's system is two languages (Python + vendor C/C++), the Rust design is three.

The condition under which the answer flips to **yes**: if the mapping engine ever leaves Open3D for
something Rust can own end-to-end (own CUDA kernels behind cudarc, or a future Rust TSDF crate), so
that a `roomscan-core` would be *one* GIL-free language from wire to mesh — then Rust's ownership,
exhaustive enums and typestate would cover the whole live path and the three-language objection
disappears. Nothing in the current roadmap (#113/#114/#116 are all C++-library-shaped) makes that
likely, so revisit only if E3 passes *and* the mesh-ceiling work (#114) concludes that a self-authored
mapper is required anyway.

---

## Final decision table

| Area | Current | Candidate | Benefit | Cost | Confidence | Recommendation |
|---|---|---|---|---|---|---|
| Transport/ingestion | Python threads, pyserial, UDP reassembly | Rust/tokio | ~0 (0.4 % core; bugs were logic/platform) | medium | high | Keep Python |
| RSCN protocol | Python `struct`+`zlib` | Rust zerocopy/crc32fast (spike byte-exact, 25×) | ~0 observable; +1 drift surface | low | high | Keep Python; shelve spike |
| Sensor decode / clocks / fusion | Python + numpy | Domain newtypes + Pyright strict | 9 historical bug classes | low | high | **Do now (E1)** |
| Vendor ToF transform | C via ctypes | bindgen wrap | 0 | low | high | Keep as is |
| Deprojection | numpy | Rust/glam | ≤0.5 ms | low | high | Keep Python |
| Translation ICP + gates | O3D NNS + numpy | Rust kiddo/nalgebra PyO3 (E3) or small_gicp (C++) | 2–4× stage, 0 fps; p99 41→~25 ms | medium–high; chaos-sensitive parity | medium | Defer; E3 only if E2 fails |
| TSDF / raycast / mesh | Open3D CUDA | none in Rust; nvblox (C++) per #114 | n/a | very high | high | Do not rewrite |
| SLAM orchestration | Python | typed poses/frames/modes | pose-direction + frame classes | low | high | **Do now (E1)** |
| GIL isolation | in-process worker | existing `SlamService` subprocess + credits (E2) | tick_share 0.80→≥0.97, no 100–450 ms stalls | medium | medium | **Do now (E2)** |
| Web backend | FastAPI/uvicorn | axum | small; bugs were vendor defaults/JS | very high (5.3 k lines) | high | Keep Python |
| Thin client render | Open3D Filament | wgpu | 0 | high | high | Keep |
| MCP | mcp SDK + Playwright | rmcp | 0 (tools wrap Python) | very high | high | Keep Python |
| Offline 3DGS | torch/gsplat/pycolmap | none | n/a | n/a | high | Keep Python |
| C++ `roomscan-native` | proposed (#116) | — | gated on N1∧N2∧profiling; profiling gate fails today | high | high | Do not start; leave gates as written |
| Full Rust host | — | — | negative (3 languages, no mapping engine) | very high | high | Reject |
