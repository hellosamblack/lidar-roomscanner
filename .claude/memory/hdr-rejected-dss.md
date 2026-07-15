---
name: hdr-rejected-dss
description: "HDR exposure-bracketing was considered and rejected — the VL53L9CX's on-chip DSS already does per-zone hardware auto-exposure"
metadata: 
  node_type: memory
  type: project
  originSessionId: 134fa1f3-1646-4a17-9501-df35ab119a97
---

HDR exposure-bracketing (sweep `SET_EXPOSURE_MS`, per-pixel fuse the best-conditioned depth/IR return to
widen dynamic range) was proposed on 2026-07-09 and **rejected — do not re-propose without new information.**

**Why:** it is redundant with the VL53L9CX's on-chip **Dynamic SPAD Selection (DSS)**. Per an ST engineer,
DSS is per-zone hardware auto-gain: before ranging, each zone picks its SPAD collection area (all SPADs for
dull/far targets, down to 1–2 for bright/near), 16 steps/zone, visible in the raw frame's 4-bit/zone DSS map.
The sensor also dual-ranges (two pulse-repetition intervals, radar-aliasing rejection) and returns a
fully-processed, disambiguated depth — we never touch the histogram, so the only host lever is a whole extra
ranging cycle at a different integration time (the slow ~55 ms/step reprofile path). DSS trades collection
*area*; exposure trades integration *time*. Host-side HDR would therefore only add range at DSS's extreme
tails (a retroreflector still saturating at 1–2 SPADs → needs shorter time; a very dark/far target still
starved at all SPADs → needs longer time) — a corner case not worth a subsystem.

**How to apply:** if HDR / exposure-bracketing / "better dynamic range" comes up again, cite DSS and this
decision. The `confidence` (CF32) stream is separately useless as a fusion metric (structurally pinned to
~1.0 by an uninitialized `conf_scaling` divisor — see `docs/deprojector-validation.md`); the usable per-zone
quality signals would be `amplitude` (AF32) and `status` (CU32), which are computed by the transform library
but not yet wired to the host. The one thing that would revive HDR is a firmware `DISABLE_DSS` command. Full
rationale recorded under "Considered and rejected" in `ROADMAP.md`. Related: [[edge-ai-tooling]].
