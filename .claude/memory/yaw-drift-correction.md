---
name: yaw-drift-correction
description: "Host-side magnetometer yaw-drift correction (9-axis fusion) — MERGED to main (PR #2, merge 33b7796), tests green (225p); STILL needs on-target mag calibration + AXIS_CONVENTION check before it's useful/verified on hardware"
metadata: 
  node_type: memory
  type: project
  originSessionId: d1be5696-2f58-4f08-a49f-0b0ed498d0ad
---

Closes the yaw-drift gap noted in [[lsm6dsv16x-panel-integration]] (SFLP is 6-axis; yaw drifts; mag
was streamed but never fed back). **Host-side only — no firmware/protocol change.** Built via the full
brainstorm→spec→plan→subagent-driven-development flow (2026-07-10).

**Design decisions (from two exploration subagents + owner):**
- The LSM6DSV16X has **no on-chip 9-axis fusion** — SFLP never consumes the mag — so yaw correction MUST
  be a host complementary filter. SFLP already applies gyro-bias internally, so temp-based gyro-bias comp
  is redundant. Dead-reckoning (dt0106) is redundant vs point-cloud ICP. **ASC/MLC/FSM rejected**: need a
  MEMS-Studio `.ucf` blob whose reset header would drop the LSM's I3C address (the no-reset invariant),
  and the rig is tethered so their power/autonomy payoff is nil. All recorded "considered & rejected" in
  the spec, like [[hdr-rejected-dss]].

**What shipped (MERGED to `main` 2026-07-10 via PR #2, merge commit `33b7796`; branch deleted):**
- `host/src/roomscan/magcal.py` — `MagCalibration` (hard/soft-iron `matrix @ (raw-offset)`, JSON) +
  `fit_ellipsoid` (least-squares, rank-deficient guard).
- `host/src/roomscan/sensors.py` — quat yaw/pitch/graft helpers; `absolute_heading()` (**yaw-STRIPPED**
  de-tilt = drift-free reference); `YawFusion` (snap-then-low-pass, yaw-only world-Z graft, gates:
  anomaly/motion-proxy/gimbal); `SensorState.fused_quat()`/`fusion_status()`. `AXIS_CONVENTION` (default
  identity).
- `panel.py` gizmo+compass consume the fused orientation; `config.py` 6 fields; `tools/mag_calibrate.py`
  CLI. Added `[tool.pytest.ini_options] pythonpath=["src","."]` (worktree runs without editable install).
- Full host suite **225 passed / 13 skipped**, TDD.

**KEY BUG the final review caught (don't reintroduce):** de-tilting the mag with the FULL SFLP quat
re-injects its drifting yaw (`fused_yaw = α + Y_sflp − Y_true`) → fusion does nothing. Fix = de-tilt with
a **yaw-stripped** quat (`graft_yaw(quat, -quat_yaw_deg(quat))`). Regression: `test_rejects_sflp_yaw_drift`.

**STILL OPEN — on-target bench steps (need the rig; tests can't cover):**
1. Run `python -m tools.mag_calibrate` (rotate rig) → produce `mag_cal.json`.
2. Verify heading sign; if mirrored, set `AXIS_CONVENTION` in `sensors.py` (residual `−Y_true` sign is
   inherent to the heading definition, resolved here). Confirm gizmo yaw stops drifting on a static hold.

**Env gotcha:** the host venv lives in the MAIN checkout (`host/.venv`), not the worktree. Also:
subagents spawned in this bg job default to the MAIN-checkout cwd — one mis-committed to `main`; had to
relocate to the worktree branch and `reset` main. When orchestrating here, the controller should commit,
not the implementer subagents.
