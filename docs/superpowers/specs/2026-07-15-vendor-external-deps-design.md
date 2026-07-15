# Vendor external dependencies into the repo — design

**Date:** 2026-07-15
**Goal:** Make `roomscanner/` self-contained: no build/runtime dependency resolves to a
path outside the repository. Driven by owner request "there are dependencies that live
outside this repo (notably `F:\git\personal\lidar\53L9A1`) — move them into the repo and
update paths accordingly."

## Scope

Two out-of-repo dependencies were found. The sibling reference/tooling folders in
`F:\git\personal\lidar\` (`X-CUBE-MEMS1` 469 MB, `st-mems-ispu` 177 MB, `stdatalog-pysdk`,
`stm32ai-modelzoo` **11 GB**) are **not** build/runtime dependencies — nothing in the repo's
code or CMake references them — and are explicitly out of scope.

### Dependency 1 — `53L9A1` ST reference package (external, `../../../53L9A1`)

The ST package (`~54 MB`) supplies the vl53l9 transform library, media-object headers,
`vl53l9-common` platform utils, the BSP component, and the ETH HAL driver source. Consumed
by two build files via a `PKG_ROOT` variable:

- `firmware/scanner-stream/CMakeLists.txt` — `PKG_ROOT = ${CMAKE_CURRENT_SOURCE_DIR}/../../../53L9A1`
- `host/transform/CMakeLists.txt` — `PKG_ROOT = ${CMAKE_CURRENT_SOURCE_DIR}/../../../53L9A1`

It is also the "read-only reference firmware" (`<APP>` = `…/53L9A1/Projects/NUCLEO-H563ZI/
Applications/53L9A1/53L9A1_PostprocessSingle/`) referenced throughout `CLAUDE.md`, `ROADMAP.md`,
and skills.

### Dependency 2 — `firmware/vendor/lwip` (git submodule, content external)

`firmware/vendor/lwip` is a git **submodule** (gitlink `160000 6ca936f…`) with **no
`.gitmodules`** config. Its 401 source files are checked out on disk but tracked only as a
commit pointer to an external repo — so the content lives outside this repo's tree. Consumed
by `firmware/scanner-stream/CMakeLists.txt` via glob (`../vendor/lwip/src/...`). tinyusb, by
contrast, is already fully vendored as 195 plain files — lwip should match it.

## Decisions (owner-approved)

1. **Bring the whole `53L9A1` package in** (not just the build-consumed subset) so the
   reference firmware `<APP>`, ST docs, and every existing doc reference remain valid.
2. **Destination: `firmware/vendor/53L9A1/`** — alongside the existing vendored
   `firmware/vendor/{tinyusb,lwip}`, matching the repo's established convention.
3. **De-vendor lwip** into plain tracked files (drop the submodule gitlink; keep files).
4. **Copy, don't move** — leave the external `F:\git\personal\lidar\53L9A1` untouched
   (canonical ST package, may be used by sibling projects). Repo just gains its own copy.

## Plan of record

### A. Vendor 53L9A1
- Copy `F:\git\personal\lidar\53L9A1` → `firmware/vendor/53L9A1/`, **excluding** the stray
  `Projects/NUCLEO-H563ZI/Applications/53L9A1/53L9A1_PostprocessSingle/build/` output tree
  (regenerable; also caught by the existing `build/` gitignore rule).
- No source file inside the vendored subset matches the `*.bin/*.elf/*.o` ignore rules, so
  the `.c/.h` we build from will track normally. Verify with `git status` /
  `git check-ignore` after `git add`.

### B. De-vendor lwip
- `git rm --cached firmware/vendor/lwip` (removes the gitlink; leaves files on disk).
- Remove the nested `.git` marker so the files are plain, then `git add firmware/vendor/lwip`
  to stage all 401 files. (No `.gitmodules` to edit.)

### C. Repoint build files
- `firmware/scanner-stream/CMakeLists.txt`: `PKG_ROOT` → `${CMAKE_CURRENT_SOURCE_DIR}/../vendor/53L9A1`
- `host/transform/CMakeLists.txt`: `PKG_ROOT` → `${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/vendor/53L9A1`
  (update the accompanying path-derivation comment too).

### D. Update live documentation (authoritative, describes current layout)
- `CLAUDE.md` — repo-layout diagram, the reference-package sentence, and the `<APP>` definition.
- `ROADMAP.md` — the `<APP>` path line.
- `docs/engineering-practices.md` — the "`../53L9A1/` is read-only reference" rule.
- `docs/iks4a1-stacking.md` — the `../53L9A1/` platform-layer mention.
- `.claude/skills/firmware-loop/SKILL.md` and `.claude/skills/stack-electrical/SKILL.md`.
- `host/transform/rs_transform_shim.h` — the `../../../53L9A1` comment.

The "read-only reference — do not edit" semantics are **kept**; only the path changes
(now `firmware/vendor/53L9A1/…`).

### E. Leave as-is (historical record — editing would falsify history)
- `.superpowers/sdd/*.diff` and `*-brief.md` — point-in-time review/task snapshots.
- `docs/superpowers/plans/*.md` — dated, already-executed implementation plans.

### F. Verify (evidence before "done")
1. `host/transform` — CMake configure + build the native shim (builds on this PC); confirm
   it resolves sources under `firmware/vendor/53L9A1`.
2. `firmware/scanner-stream` — CMake configure with the arm-none-eabi toolchain; confirm the
   new `PKG_ROOT` resolves (full build if the toolchain is available).
3. `git status` — vendored 53L9A1 tracked, lwip now plain files, no build artifacts staged.

## Non-goals
- No change to what sources the builds compile, no code edits to vendored ST/lwip files.
- Siblings (`X-CUBE-MEMS1`, `st-mems-ispu`, `stdatalog-pysdk`, `stm32ai-modelzoo`) stay external.
- No deletion of the external `53L9A1`.

## Risks
- **Repo size** grows ~54 MB (plus lwip's files becoming tracked). Acceptable per owner.
- **lwip de-vendor**: must delete the nested `.git` so files stage as normal content; verify
  the build glob still matches afterward.
- **ETH HAL**: firmware pulls `stm32h5xx_hal_eth.c(_ex)` from `PKG_ROOT` (not the in-repo
  `scanner-stream/Drivers` HAL copy). The vendored path preserves this — confirm at configure.
