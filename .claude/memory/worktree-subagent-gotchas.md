---
name: worktree-subagent-gotchas
description: Orchestrating subagent-driven / multi-agent work from a git worktree in a background job — spawned subagents default to the MAIN checkout cwd (mis-commit risk); venv lives in main checkout; controller should own commits
metadata: 
  node_type: memory
  type: feedback
  originSessionId: d1be5696-2f58-4f08-a49f-0b0ed498d0ad
---

When running subagent-driven-development (or any spawned Agent) **from a git worktree inside a background
job** in this repo, two environment gotchas bite (learned 2026-07-10 during [[yaw-drift-correction]]):

1. **Spawned subagents default to the MAIN checkout cwd**, not the worktree the controller is in. An
   implementer subagent wrote files to the worktree but ran `git commit` in the MAIN checkout → committed
   the task straight onto `main`. Recovery cost a full cycle (relocate the commit to the worktree branch,
   `git -C <main> reset --mixed` to drop it, delete stray files, preserving the user's uncommitted edits).

**Why:** EnterWorktree changes only the controller session's cwd; fresh subagents launch at the job's
default cwd (the main checkout).

**How to apply:** For SDD/multi-agent work from a worktree here, **the controller does the git commits
itself** — dispatch implementer subagents to only write files + run tests (no git), then commit in the
worktree as controller. If subagents must run git, give them the absolute worktree path for EVERY command
and verify each commit landed on the worktree branch (`git -C <worktree> log`) before proceeding. Never
`reset`/merge `main` casually — the main checkout may hold the user's uncommitted work (use `--mixed`, not
`--hard`, if you must).

2. **The host Python venv lives in the MAIN checkout** (`host/.venv`), not the worktree, and it's an
   editable install of `roomscan` pointing at MAIN's `src`. Running that venv's pytest from a worktree
   imports MAIN's code, not the worktree's. Fixed durably by adding
   `[tool.pytest.ini_options] pythonpath = ["src", "."]` to `host/pyproject.toml` so `pytest` resolves
   `roomscan` from `./src` (and `tools` from `./`) regardless of checkout. Test command that works from a
   worktree: `cd <worktree>/host && "F:/git/personal/lidar/roomscanner/host/.venv/Scripts/python.exe" -m pytest`.

3. **Building the `scanner-stream` firmware from a fresh worktree needs two junctions** (learned
   2026-07-15). A fresh worktree branches from `origin/main` and is missing what the build's relative
   paths expect: (a) `CMakeLists.txt` sets `PKG_ROOT = ${CMAKE_CURRENT_SOURCE_DIR}/../../../53L9A1`, which
   from a worktree resolves to `.claude/worktrees/53L9A1` (nonexistent, since the real ST package lives at
   `F:\git\personal\lidar\53L9A1`, i.e. `roomscanner/../53L9A1`); (b) `firmware/vendor/lwip` is a **git
   submodule / gitlink (mode 160000)** that is NOT populated in a fresh worktree (empty dir), while
   `vendor/tinyusb` IS committed content. Fix for a one-off build: create Windows junctions
   (`cmd /c mklink /J`) `worktrees/53L9A1 → ../../53L9A1` and `<worktree>/firmware/vendor/lwip →
   <main>/firmware/vendor/lwip`, build, then remove the junctions with `cmd /c rmdir <link>` (link-only, never
   `/S` — that would delete through to the target). **Gotcha:** removing the lwip junction leaves the
   gitlink dir *absent*, so the worktree shows a phantom ` D firmware/vendor/lwip` — clear it with
   `git -C <worktree> checkout -- firmware/vendor/lwip`. Toolchain isn't on PATH; prepend the STM32CubeIDE
   2.2.0 `gnu-tools-for-stm32*/tools/bin` (see [[firmware-loop]] / the firmware-loop skill).

Also: this repo's `wrap-up` skill Phase 1 auto-merges the feature branch into `main` — that conflicts with
the background-job no-merge rule and the draft-PR review flow. In a bg job, ship = commit + push + draft
PR; skip the merge.
