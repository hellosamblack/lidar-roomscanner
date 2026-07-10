# Roomscanner Agent Instructions

This document aligns agent behavior, guidance, memories, and skills with those of Claude for the roomscanner project.

## 1. Project Guidance and Instructions
- Always read and follow the instructions in [CLAUDE.md](file:///f:/git/personal/lidar/roomscanner/CLAUDE.md) at the root of the workspace. It is the primary system guidance file for compiling, flashing, capturing, and monitoring STM32 firmware on the NUCLEO-H563ZI.
- Adhere strictly to the coding standards and practices detailed in [engineering-practices.md](file:///f:/git/personal/lidar/roomscanner/docs/engineering-practices.md).

## 2. Project Memory and History
- Read and maintain the memory files located in the [.remember/](file:///f:/git/personal/lidar/roomscanner/.remember) directory:
  - [now.md](file:///f:/git/personal/lidar/roomscanner/.remember/now.md): Tracks the immediate next / current state and findings of the project.
  - [recent.md](file:///f:/git/personal/lidar/roomscanner/.remember/recent.md): Tracks the summary of recent milestones/phases.
  - Daily files (e.g., `today-YYYY-MM-DD.md` matching today's date): Chronologically log task completion, commits made, and insights.
- When finishing a task or landing any work, make sure to update these memory files to document the changes and achievements.

## 3. Superpowers and Implementation Plans
- Refer to [plans/](file:///f:/git/personal/lidar/roomscanner/docs/superpowers/plans) and [specs/](file:///f:/git/personal/lidar/roomscanner/docs/superpowers/specs) under [superpowers/](file:///f:/git/personal/lidar/roomscanner/docs/superpowers) for historical and active design details and implementation plans.
- When implementing a new feature, follow planning mode: create or update the implementation plans within the project structure (or use system-managed artifacts as appropriate) and wait for user approval.

## 4. Status Sync and Milestone Retro
- The unit of "done" is code + the doc deltas it implies.
- When wrapping up a feature, phase, or before opening a PR, use the `status-sync` skill (re-routed from `.claude/skills/status-sync`) to update [ROADMAP.md](file:///f:/git/personal/lidar/roomscanner/ROADMAP.md), [CLAUDE.md](file:///f:/git/personal/lidar/roomscanner/CLAUDE.md), and project memories.
- Run `milestone-retro` at the end of each major milestone.
