---
name: hardware-diagnosis-discipline
description: Two hard-won rules for debugging the board stack — consult the authoritative electrical model before theorizing, and never conclude "hardware fault" while a diagnostic binary is flashed
metadata:
  node_type: memory
  type: feedback
  originSessionId: 899dd727-c4aa-44b7-9f8e-2dd3fe19bc5e
---

Two process failures during the 2026-07-10 stacked-I3C debug that cost the owner real time; avoid repeating.

**1. For any bus/stack hardware question, read the authoritative electrical model FIRST — don't theorize
from noisy PDF text extraction.** I proposed wrong root causes twice (parallel-pull-up over-load; then a
physical contact/re-seat issue) by reasoning from `pdftotext` dumps of schematics. Both were refuted the
moment we consulted the real sources via the `stack-electrical` skill: `references/kicad/roomscanner-stack/
roomscanner-stack.net` (netlist — showed both auto-direction translator A-sides on PB8/PB9) and
`references/i2c-i3c-bus-debug-reference.md` §6 (which already named the NXS0108 push-pull mis-latch as the
leading hypothesis — the actual cause). **How to apply:** invoke `stack-electrical` and read the netlist +
debug reference BEFORE forming a hardware hypothesis; cite the net/page, don't guess from OCR.

**Why:** the owner had to stop me and push back ("it's electrically separate") and then hand me the model
before I stopped chasing phantoms. The authoritative docs exist precisely so this doesn't happen.

**2. Never leave a diagnostic (non-streaming) probe binary flashed, and never conclude "hardware fault"
without first reflashing a known-good streaming build.** I flashed the `iks4a1_i3c_probe` build (loops
forever, never calls `tud_connect` → no CDC by design), handed the board back, and then misread the absent
CDC as a dead/disturbed ToF — sending the owner to reseat boards chasing nothing. **How to apply:** after any
probe/diagnostic flash, reflash the shipping streaming build before handing off; and before ever saying
"hardware," confirm with a streaming build + a known-good baseline (e.g. ToF-only) that the fault survives.

See [[firmware-bringup-division-of-labor]] (agentic loop, warm-wedge caution) and
[[lsm6dsv16x-panel-integration]] (the eventual slow-PP-ENTDAA fix).
