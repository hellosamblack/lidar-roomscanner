# E2 — GIL stalls are removable without a language change (Refs #223)

Spec: `docs/superpowers/specs/2026-08-27-rust-migration-feasibility.md` → "Proof-of-concept
program", row E2. Hypothesis: the web-loop GIL-stall problem (tick_share 0.80, worst ~100 ms,
`prepare_packet` 14.9 % of wall) disappears if SLAM runs in the existing out-of-process
`SlamService`, **once the IPC is fixed** — config sent once, credit-gated latest-wins meshes, stage
timings forwarded, raw-bytes arrays. Date 2026-08-27, box: RTX 2000 Ada, 26 cores, Open3D CUDA:0.

## What was built

`host/src/roomscan/slam/{wire,service,remote}.py` — wire **v2** (self-describing, `type`-tagged):

| Fix | Mechanism |
|---|---|
| (a) config once | `hello` on connect carries mapper cfg + frame geometry + mesh format; the service replies `hello(device, icp_device)`. Frames carry no `cfg`. A v1 client (cfg on every frame) is still served. |
| (b) mesh credit | `_MeshCredit`: the service sends a mesh only while the client holds no unacked mesh; a newer mesh **replaces** the pending one (latest-wins, `replaced` counted). The client returns the credit (`mesh_ack`) when the *consumer* takes the mesh out of `latest()`/`latest_packet()`, not on receipt. |
| (c) stage timings | `wire.step_fields()` serialises every numeric `FrameStep` field (raycast/icp/integrate ms, inliers, block gauge, budget flag, `t_device_us`) plus IPC stamps (`t_client_submit/send`, `t_service_recv/done`) and `mesh_extract_ms`; `RemoteSlamWorker` rebuilds the full `FrameStep` and exposes `last_pose_timing`. |
| (d) no gratuitous copies | Framing 8-byte-aligns every blob; `recv_message` reads into one `np.empty` buffer (a zero-filled `bytearray` cost a 36 MB memset **under the GIL**); `decode_message` returns `np.frombuffer` views; packet bytes pass through as a `memoryview`. |
| frame credit (found during measurement) | One frame between send and its pose; later submits overwrite the client slot. Without it the kernel socket queued frames behind every 36 MB mesh send: `submit→service_recv` p95 was **170–207 ms**. |
| packet mode | `mesh_format="packet"`: the service runs `MeshPrep` (prepare_packet + `pack_mesh`, with the same 12 MB/s byte-rate governor as `SlamRunner.poll`) and ships finished MESH bytes; the web loop only forwards them. |
| 16 MB socket buffers + TCP_NODELAY | A whole mesh drains into the kernel instead of blocking the single-threaded service loop; poses are not Nagle-delayed behind it. |

`host/tools/slam_stall_profile.py --mode live` — the threaded `roomscan-web` shape (reader-fed
latest-wins worker, MeshPrep thread, polling broadcaster on the main thread) replayed at the
capture's own cadence; `--backend remote` spawns `python -m roomscan.slam.service` as a subprocess
and drives it through `RemoteSlamWorker`; `--mesh-format arrays|packet`. Reports the broadcaster's
`tick_share` / worst stall, pose-age (submit → pose visible in `poll()`), the IPC breakdown, mesh
cadence/delivery, and the subprocess's CPU/RSS. Exposed via MCP `slam_stall_profile(mode=, backend=,
mesh_format=)`. `FrameStep.t_device_us` (new, echoed by `Mapper.step`) is what lets the rig say
which submitted frame a pose answers on both backends.

## Measurement

`captures/coffeeRoomCircuitNoMnt.bin`, first 1800 depth frames, shipped `[slam]` config, CUDA:0,
real-time cadence (≈60 s wall per live run), every configuration run twice, never concurrently.
Baseline = the shipped serial rig (spec's own instrument) **and** the live rig with the in-process
`SlamWorker` (what `roomscan-web` runs today).

### Headline (web-loop availability)

| config | run | tick_share | worst stall ms | starved % | fps (processed/wall) | overwritten | meshes |
|---|---|---|---|---|---|---|---|
| serial rig (spec baseline) | 1 | 0.799 | 100.9 | 20.1 | 9.7 (serial) | – | 361 |
| serial rig | 2 | 0.800 | 100.6 | 20.0 | 9.5 | – | 361 |
| **live, local** (ships today) | 1 | 0.793 | 107.4 | 20.7 | 27.3 | 148 | 105 |
| live, local | 2 | 0.787 | 96.8 | 21.3 | 27.2 | 152 | 105 |
| live, remote, arrays (prep in-process) | 1 | 0.910 | 120.7 | 9.1 | 26.5 | 193 | 106 |
| live, remote, arrays | 2 | 0.904 | 89.2 | 9.6 | 26.9 | 174 | 109 |
| **live, remote, packet** (prep in service) | 1 | **0.974** | **26.8** | 2.6 | 27.2 | 156 | 107 |
| live, remote, packet | 2 | **0.974** | **24.3** | 2.6 | 27.0 | 166 | 107 |

Serial-rig per-stage (run 2): prep 14.8 % of wall / share 0.82 / worst 100.6 ms; pack 2.2 %;
step 2.0 %; mesh 1.1 % — identical to the spec's numbers, so the instrument is reproducing.

### Pose age

`pose_age` = `submit()` → first `poll()` that returns that frame's pose (**includes** mapper
compute and any slot/credit wait). `transport` = pose_age − that step's own `total_step_ms`.
The IPC columns are the per-hop breakdown from the echoed monotonic stamps (same host clock).

| config | run | pose_age p50 / p95 / p99 ms | transport p50 / p95 ms | step p50 / p95 ms | credit wait p95 | send→svc recv p95 | svc→client p95 | client→poll p95 |
|---|---|---|---|---|---|---|---|---|
| live, local | 1 | 16.3 / 71.3 / 115 | 3.5 / 22.0 | 13.0 / 53.1 | – | – | – | – |
| live, local | 2 | 16.3 / 71.5 / 113 | 3.5 / 21.7 | 13.1 / 54.2 | – | – | – | – |
| remote, arrays | 1 | 16.9 / 81.7 / 116 | 2.9 / 46.1 | 12.7 / 43.3 | 22.6 | 22.8 | 2.6 | 1.9 |
| remote, arrays | 2 | 16.9 / 76.3 / 113 | 2.9 / 42.2 | 12.5 / 42.5 | 22.7 | 24.0 | 3.5 | 1.9 |
| remote, packet | 1 | 16.7 / 73.8 / 115 | 2.6 / 31.3 | 13.1 / 49.6 | 21.2 | 4.8 | 0.6 | 1.9 |
| remote, packet | 2 | 16.5 / 74.0 / 121 | 2.6 / 32.2 | 13.0 / 49.8 | 19.7 | 5.1 | 0.6 | 1.9 |

The spec's baseline "pose age p50 1.1 / p95 4.8 ms" was measured on `/ws` from *pose available →
sent*. The comparable far-side hops here are `service→client` + `client→poll`: **p95 ≈ 2.5 ms**
in packet mode (0.6 + 1.9), p50 ≈ 1.3 ms. On the stricter submit→visible definition the remote
path is **indistinguishable from local** (p95 74 vs 71 ms; p50 identical at 16.5 ms), because both
are dominated by the mapper's own p95 step (~50 ms) plus the latest-wins wait behind it — a
number the local worker shares. The remaining `credit wait` p95 ≈ 20 ms is by design: it is the
frame that would otherwise have queued in the socket, and it turns into `frames_overwritten`
(156–166 remote vs 148–152 local — the same drop rate).

### Cost of the subprocess

| config | service CPU (of one core) | service RSS peak | client CPU | IPC bytes (mesh) |
|---|---|---|---|---|
| live, local | – | – | 631 % | 0 |
| remote, arrays | 231 % | 780–800 MB | 410–428 % | 4.5 GB / 60 s (raw f32 arrays, every mesh) |
| remote, packet | 663 % | 1.07 GB | **4.2 %** | 0.72–0.76 GB / 60 s (packed, governed) |

The web-loop process drops from 6.3 cores to 0.04 of a core in packet mode: everything heavy
moved. The service's 663 % is prep's own numpy/Open3D thread pool (the same work the client used
to do at 631 %). Packet-mode mesh delivery (`t_pub` in the service → consumed by the broadcaster)
p50 4 ms / p95 45–61 ms for 36 MB; cadence p50 220–240 ms, governed to ~3 s once the map is
36 MB, matching the local governor.

## Verdict against the spec's gates

| Gate | Pass | Kill | remote-packet (2 runs) | Result |
|---|---|---|---|---|
| web-loop tick_share | ≥ 0.97 | – | 0.974, 0.974 | **pass** (local 0.79) |
| worst stall | < 30 ms | – | 26.8, 24.3 ms | **pass** (local 107, 97) |
| pose-age p95 on the far side (spec's definition) | ≤ 10 ms | > 20 ms | ≈ 2.5 ms | **pass**; kill not triggered |
| pose-age p95, submit → visible (incl. compute) | – | – | 74 ms vs local 71 ms | no regression; unreachable on either backend while step p95 ≈ 50 ms |

**E2 passes, with two honest qualifications:**

1. **Only the packet variant passes.** Moving *just the mapper* out of process (arrays mode) gets
   tick_share to 0.91 with 90–120 ms stalls, because `prepare_packet`/`pack_mesh` — the biggest
   GIL holder (14.8 % of wall in the serial rig) — is still in the web process, and decoding
   36 MB of arrays into Open3D tensors adds its own GIL time. The GIL problem is closed only when
   every Open3D/numpy-heavy call leaves the interpreter that runs the event loop, which is exactly
   the spec's own "by any means" clause.
2. **The margin on tick_share is thin (0.974 vs 0.97).** The residual 2.6 % starvation in the
   client is now: the 36 MB `recv_into` page-faulting, `list(trajectory)` per pose (O(n) at
   1800 poses), and the JSON header decode. Stable across two runs (0.9741 / 0.9740, worst
   26.8 / 24.3 ms) but not a wide margin; a longer map (the spec also names
   `roomSweepFull20260730.bin`, not run here) would grow the mesh and should be checked before
   this is declared closed for all captures.

Three things the measurement taught that the design did not predict — all were found by the
per-hop breakdown, not by the headline numbers:

- The first remote run had `submit→service_recv` p95 of 170–207 ms with `service→client` under
  4 ms: the socket was a queue, not a pipe. A **frame credit** (one in flight, latest-wins) fixed
  it; a mesh credit alone did not, because frames were the thing piling up.
- `bytearray(36 MB)` zero-fills under the GIL; `np.empty` does not. That plus a `bytes()`
  copy was 5 % starvation and the 65 ms worst stalls of the intermediate packet run.
- The in-service `MeshPrep` needed the same byte-rate governor as `SlamRunner.poll`; without it
  the service prepped 170 meshes instead of 107 and its GIL/CPU load raised the mapper's step p95
  from 50 to 77 ms and the service to 11 cores.

## Not done / follow-ups

- ~~`SlamRunner` still constructs `RemoteSlamWorker` in arrays mode~~ **done, #224** -- see
  "Productised" below.
- ~~Second capture (`roomSweepFull20260730.bin`) not run.~~ **run, #224** -- 0.976 / 0.978.
- The service is single-threaded: a 36 MB `sendall` still blocks its frame loop for the time the
  client takes to drain 20 MB past the 16 MB buffer (~5 ms here). Two sockets (pose / mesh) would
  remove that if it ever shows.

## Reproduce

```
host/tools/spikes/e2_batch.sh serial:1 local:1 remote-arrays:1 remote-packet:1   # JSON in /tmp/e2_*.json
host/.venv/bin/python host/tools/slam_stall_profile.py captures/coffeeRoomCircuitNoMnt.bin \
    --frames 1800 --device CUDA:0 --mode live --backend remote --mesh-format packet
```


## Productised (#224, 2026-08-28)

`SlamRunner` now runs this configuration when `[slam] backend = "remote"` (`remote_mesh_format =
"packet"` by default; `remote_spawn = true` has the web app spawn and supervise the service on
`remote_addr`, loopback only). Save fetches the full-res mesh over the wire (`mesh_request` /
`mesh_full`); the UI's wall toggle reaches the in-service MeshPrep (`set_view`); a dead service is
detected in `poll()` (socket closed without `stop()`), the worker is torn down and the next frame's
rebuild restarts the subprocess -- the CUDA-OOM isolation BUG-053 asked for. Default stays local.
Of the two named residuals, `list(trajectory)` per pose is gone (the append-only list is published
as-is); the 36 MB `recv_into` page-faulting was left alone -- it happens with the GIL released and
the margin below no longer needs it.

### roomSweepFull20260730.bin, ALL 3525 frames, CUDA:0, real-time cadence (117 s wall)

The second capture the spec named (the one E2 did not run). The map ends at ~1.2 M vertices /
70-76 MB per MESH -- twice E2's coffee-room map -- so this is the margin check E2 asked for.

| config | run | tick_share | worst stall ms | starved % | processed / overwritten | fps | web-proc CPU | meshes | final MB |
|---|---|---|---|---|---|---|---|---|---|
| live, local (shipped default) | 1 | 0.736 | 263.0 | 26.4 | 3163 / 362 | 26.9 | 532 % | 90 | 74.5 |
| **live, remote, packet** | 1 | **0.976** | **15.2** | 2.4 | 3003 / 522 | 25.6 | 4.5 % | 87 | 75.8 |
| live, remote, packet (post-rebase, commit under test) | 2 | **0.978** | **23.6** | 2.2 | 3094 / 431 | 26.4 | 4.2 % | 89 | 69.6 |

Pose age (submit -> visible in `poll()`, includes mapper compute): local p50/p95/p99 16.5 / 67.3 /
114 ms; remote-packet 19.3 / 84.7 / 165 ms (run 1), 18.0 / 72.1 / 155 ms (run 2). Transport share
(age minus the step's own compute): local p95 21 ms; remote p95 39 ms, p99 73-98 ms, max 372-408 ms.
The per-hop breakdown puts the remote tail in one place: `send_to_service` p99 50-55 ms / max
369-376 ms with `service_to_client` p95 < 1 ms -- the single-threaded service loop is inside
`sendall()` of a 70 MB mesh while the client drains it past the 16 MB socket buffer. That is E2's
"two sockets (pose / mesh)" follow-up, now measurable and still not needed for the gate; the mapper
step itself is p95 49-62 ms on either backend, so the tail is a p99 phenomenon at this map size.
Service process: ~500-540 % of a core, RSS 1.4-1.7 GB. Mesh cadence p50 0.5 s, governed to ~5-9 s
once the map is 70 MB (12 MB/s rule), delivery p95 149 ms for 70 MB.

The margin is no longer thin: 0.976 / 0.978 on a map twice E2's size, against 0.974 on the small
one; the local baseline on this capture is *worse* than on the coffee room (0.74 vs 0.79, 263 ms
worst stall vs 100), because the stalls scale with the map and the packet path does not.

### Live rig, real browser (second `roomscan-web` on :8010 from the worktree, replaying the same capture, looped)

`host/tools/spikes/e2_web_second_instance.sh` + `e2_live_probe.py` (arms SLAM over `/ws`, then
runs `ws_probe` for 20 s; `slam_age_ms` = server `ts` -> arrival on this host, the far-side pose
age). A headless Chrome tab (MCP `ui_screenshot`, SwiftShader WebGL) was connected on `/ws` +
`/ws-mesh` throughout and had rendered the map (screenshot taken; event log showed
`[slam] worker started, backend=remote device=CUDA:0 mesh_format=packet`, and per loop wrap
`[slam-service] client disconnected; credit stats sent/replaced/acks=(93, 0, 93)` -- every mesh
acked, none replaced).

| window | map at probe | slam msg Hz | pose age p50 / p95 / max ms | meshes in 20 s | MESH bytes | payload |
|---|---|---|---|---|---|---|
| early (369 frames, 3.7 MB) | 74 k verts | 30.0 | 1.3 / 2.0 / 7.6 | 29 | 94 MB | clean |
| grown (2143 -> 2654 frames) | 1.19 M verts, 62 MB | 29.6 | 1.4 / 5.9 / 124.7 | 6 | 345 MB | clean |
| grown, tab resumed from idle (2016 -> 2526 frames) | 1.21 M verts, 62 MB | 29.7 | 1.5 / 7.4 / 153.8 | 5 | 285 MB | clean |

Caveat on the "real browser": the tab had hit the UI's 5-minute idle modal before the two grown-map
windows (the first window is pre-idle; the third was taken right after clicking Resume, so the tab
was active for it). The pose stream stayed at 29.6-30 Hz with p95 <= 7.4 ms in every window; the
100-150 ms maxima coincide with a 62 MB MESH going to two `/ws-mesh` clients. A screenshot after the
last window hung the MCP browser for 30 min (SwiftShader on a 1.2 M-vertex mesh, cf. the
headless-Chrome-burns-cores note) -- that is the probe's cost, not the server's.

Not exercised live: the Save button (refused in replay by design -- `save -> replay SLAM is a
preview`); Save through the remote backend is covered end-to-end by
`test_end_to_end_remote_packet_runner_saves_a_real_map` (real service subprocess, real capture,
PLY written) and by the wire-level `mesh_request` tests. CUDA-OOM restart is covered by the
supervisor test (SIGKILL the child, `ensure()` restarts it) and the runner's dead-worker teardown
test, not by an actual OOM.

Reproduce:

```
E2_CAPTURE=captures/roomSweepFull20260730.bin E2_FRAMES=0 host/tools/spikes/e2_run.sh /tmp/rp.json --mode live --backend remote --mesh-format packet
host/tools/spikes/e2_web_second_instance.sh            # :8010, private HOME, backend=remote
host/.venv/bin/python host/tools/spikes/e2_live_probe.py --restart --arm-wait 78 --seconds 20
```
