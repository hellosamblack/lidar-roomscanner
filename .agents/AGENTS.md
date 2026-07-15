# Roomscanner Agent Instructions

This document aligns agent behavior, guidance, memories, and skills with those of Claude for the roomscanner project.

## 1. Project Guidance and Instructions
- Always read and follow the instructions in [CLAUDE.md](../CLAUDE.md) at the root of the workspace. It is the primary system guidance file for compiling, flashing, capturing, and monitoring STM32 firmware on the NUCLEO-H563ZI.
- Adhere strictly to the coding standards and practices detailed in [engineering-practices.md](../docs/engineering-practices.md).

## 2. Project Memory and History
- Read and maintain the memory files located in the [.remember/](../.remember/) directory:
  - [now.md](../.remember/now.md): Tracks the immediate next / current state and findings of the project.
  - [recent.md](../.remember/recent.md): Tracks the summary of recent milestones/phases.
  - Daily files (e.g., `today-YYYY-MM-DD.md` matching today's date): Chronologically log task completion, commits made, and insights.
- When finishing a task or landing any work, make sure to update these memory files to document the changes and achievements.

## 3. Superpowers and Implementation Plans
- Refer to [plans/](../docs/superpowers/plans) and [specs/](../docs/superpowers/specs) under [superpowers/](../docs/superpowers) for historical and active design details and implementation plans.
- When implementing a new feature, follow planning mode: create or update the implementation plans within the project structure (or use system-managed artifacts as appropriate) and wait for user approval.

## 4. Status Sync and Milestone Retro
- The unit of "done" is code + the doc deltas it implies.
- When wrapping up a feature, phase, or before opening a PR, use the `status-sync` skill (re-routed from `.claude/skills/status-sync`) to update [ROADMAP.md](../ROADMAP.md), [CLAUDE.md](../CLAUDE.md), and project memories.
- Run `milestone-retro` at the end of each major milestone.

## 5. Python Environment and Running Tests
- The Python virtual environment is located at `host/.venv`.
- **Running Pytest**: Always execute `pytest` with the working directory (`Cwd`) set to the `host` directory using the command `.\.venv\Scripts\pytest`. Do NOT run it from the workspace root, as this causes import errors for `tests` and `tools`.
- **Running Python Scripts**: Use `host\.venv\Scripts\python` to execute Python scripts (such as validation and capture tools) from the workspace root or their respective directories.
