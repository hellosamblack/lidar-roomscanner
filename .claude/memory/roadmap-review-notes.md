---
name: roadmap-review-notes
description: Issues and inaccuracies found when reviewing roadmapResearch.md (the 3D-mapping architecture doc)
metadata: 
  node_type: memory
  type: reference
  originSessionId: 75e7b29d-c8db-4ff2-98bc-b97fe95b45f2
---

Review of `roadmapResearch.md` (see [[mapping-pipeline-plan]]). Facts that check out: 250 MHz / 640 KB SRAM; 2268 zones (54×42); payload math (6830 B/frame → 3.27 Mbps @ 60 Hz); no CSI-2 on H5 → I3C mandatory (matches firmware). The 250 MHz is corroborated by the actual PLL config in `Src/main.c` (HSE 8 MHz ×62.5 → 500 MHz VCO ÷2).

Issues to fix before building on it:
1. **Compression is self-contradictory** — payload section says 3.27 Mbps is easily sustainable with ample margin, then Bottleneck-1 mandates Delta-RLE to avoid the 9.2 Mbps ceiling. At ~35% of the ceiling you are not near saturation; on-MCU compression is premature. (Moot if Ethernet is adopted — see [[mapping-pipeline-plan]].)
2. **RTAB-Map "Discard" overstated** — it has an ICP/lidar-odometry mode that needs no visual features; the valid critique is only that its default visual path is mismatched.
3. **`int16_t quat_*` labeled "half-precision float"** — PC parser must reinterpret bits as IEEE binary16, not fixed-point int. Type as uint16/fp16 and document encoding.
4. **accel without gyro is weak for translation** — double-integrated accel drifts; either use a proper filter (needs gyro) or drop it from payload.
5. **Math equations are base64 PNGs with no alt text** — opaque to text tooling; transcribe to LaTeX.
