# Distributed GPU compute — LAN offload of SLAM and splat workloads

**Status:** Specced (priority/later)
**Date:** 2026-08-19
**Issues:** #117 (distributed GPU compute backend)
**Prior art in-repo:** the 2026-07-13 GPU container service plan+design are **deprecated**
(superseded by in-process local CUDA:0; a legacy remote backend still exists —
`slam/service.py` — documented as legacy support only). Any revival starts by reading why
that was deprecated. Note: `researchResults.md` contains **no** distributed-compute
content — #117 has no basis in that report; this spec is its first design pass.

## Problem / opportunity

Available remote hardware: i9-13900H + RTX 4080 Mobile over gigabit ethernet
(~80–100 MB/s sustained). Local card: RTX 2000 Ada, 8 GiB. Three candidate workloads
(from the issue): (a) Detailed SLAM replay, (b) live ephemeral SLAM, (c) 3DGS splat
training (15–30 min per build; the 2 M-gaussian ceiling OOMs the local 8 GiB card).

## Decision

**Precedence: (c) splat training ≫ (a) Detailed replay ≫ (b) live — and (b) is
presumptively dead.**

- **(c) Splat offload is the only arm with a real, current pain:** the local card OOMs
  at 2 M gaussians while the comparison target (Scaniverse) ships 2.46 M, and a
  15–30 min trainer blocks the box. It is also the easiest shape: batch in, artifact
  out, no latency contract, inputs (frames + poses) copied once. v1 is deliberately
  boring — `rsync` the build inputs, run the trainer remotely via SSH, pull the
  artifact; no gRPC/aiozmq service until the boring version's overhead is measured and
  found wanting. The multi-GPU *parallel* variant (splitting one training run across
  both cards) is out of scope — heterogeneous cards over gigabit is gradient-sync
  territory with no evidence it beats simple full offload.
- **(a) Detailed replay offload** matters only when a replay is both wanted urgently and
  local compute is busy; same boring transport (captures are files). Deferred until (c)
  proves the operational pattern.
- **(b) Live offload is presumptively killed** by three prior lessons, revisitable only
  with contrary measurements: a 30 Hz round-trip on an ordered transport re-creates the
  BUG-061 head-of-line/backpressure problem; an RPC wait on the reader thread is the
  starvation pattern BUG-052/BUG-063 taught us to measure with `tick_share`; and the
  live pipeline meets budget locally today, so there is no pain to justify a network in
  the hot loop. Local-first fallback and "is the remote up" health plumbing — the
  issue's stated infrastructure needs — only exist to serve (b)/(a), so they defer too.

## Benchmark plan (feasibility spike, in order)

1. **Characterize the link + host once:** sustained throughput and RTT between the boxes;
   confirm the 4080 Mobile's actual VRAM (do not assume — every VRAM number in this
   program has burned us once); confirm the remote box's availability pattern with the
   owner (a laptop that sleeps is not a backend — this is a `needs/decision` item, not a
   measurement).
2. **(c) end-to-end:** same capture, same splat config — local build wall time vs.
   (transfer + remote build + artifact return). Also the unblocking case: a >1 M-gaussian
   config the local card cannot run at all; success is "it completes," with quality via
   the standard manifest stats and `splat_compare`.
3. **(a) sizing only if (c) ships:** Detailed replay of `roomSweepFull20260730.bin`,
   local vs. remote wall clock including transfer.

## Kill criteria

- **(c) killed** if remote end-to-end (incl. transfer) fails to beat local wall time by
  ≥1.5× on builds the local card *can* run **and** the local VRAM ceiling stops binding
  (e.g. gsplat/config changes drop the working set) — the unblocking case alone keeps it
  alive as a manual runbook even if the speedup case dies.
- **(a) killed** if (c)'s measured pattern shows transfer + queueing eats the compute
  advantage for capture-sized inputs, or if nobody has actually wanted an urgent replay
  by the time (c) ships (a solution ahead of any demand).
- **(b) stays killed** unless someone produces a measured LAN RTT + a designed
  backpressure story that answers BUG-061 on paper first — and even then it needs a use
  case the local path demonstrably cannot serve.
- **Whole spec parks** if the owner cannot commit the remote box's availability
  (`needs/operator`, `needs/decision`) — infrastructure against a machine that may be
  absent is worse than none.
