---
name: workspace-and-repo
description: Project workspace layout and git repo — active dev in roomscanner/; 53L9A1 read-only reference now VENDORED in-repo at firmware/vendor/53L9A1
metadata: 
  node_type: memory
  type: project
  originSessionId: 1e8d59a4-bc3f-49b9-89fb-85ce85c7a712
---

As of 2026-07-07 development moved to a dedicated workspace. Layout under `F:\git\personal\lidar\`:
- **`roomscanner\`** — active dev workspace + the git repo root. Holds `CLAUDE.md`, `ROADMAP.md`, `references\` (roadmapResearch.md, 3D Mapping Architecture Evaluation.md), and (going forward) PC-side visualizer / frame-protocol / new firmware code.
- **`firmware\vendor\53L9A1\`** — ST reference package, treated as **read-only reference** (the existing STM32H563 ToF firmware). Do NOT edit in place. **As of 2026-07-15 it was VENDORED into the repo** at `firmware\vendor\53L9A1\` (was previously the out-of-repo sibling `..\53L9A1\`, which still exists untouched as the canonical ST copy). Both builds reference it via a `PKG_ROOT` CMake var: `firmware/scanner-stream/CMakeLists.txt` (`../vendor/53L9A1`) and `host/transform/CMakeLists.txt` (`../../firmware/vendor/53L9A1`). The reference firmware app `<APP>` is now `firmware\vendor\53L9A1\Projects\NUCLEO-H563ZI\Applications\53L9A1\53L9A1_PostprocessSingle\`. Also on 2026-07-15 `firmware/vendor/lwip` was de-submoduled (was a gitlink, no `.gitmodules`) into plain tracked files, matching how `tinyusb` is vendored — so the repo is now fully self-contained (the huge lidar/ siblings X-CUBE-MEMS1/st-mems-ispu/stm32ai-modelzoo are reference-only, NOT build deps).

**Git:** repo root is `roomscanner\`, remote `origin` = https://github.com/hellosamblack/lidar-roomscanner (default branch `main`).

See [[mapping-pipeline-plan]] for the phase roadmap and [[hardware-stack]] for sensors.
