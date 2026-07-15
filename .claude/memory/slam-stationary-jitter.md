---
name: slam-stationary-jitter
description: "Stationary-device SLAM \"jitter\" is ICP translation noise (not IMU); fixed with a display-only coherence+rotation stationarity hold"
metadata: 
  node_type: memory
  type: project
  originSessionId: 7d93d939-338d-4c18-a5c5-e1fcc31ed98d
---

Owner reported the SLAM model jitters with the device stationary on a tripod, and called it
"IMU jitter". Measured on captures: **the IMU/SFLP orientation is rock-steady when still**
(~0.03-0.08 deg/frame; yaw fusion is a 20 s low-pass and needs a mag cal that isn't set, so it
adds nothing). The shake is the **3-DoF ICP translation** estimate: ~11-45 mm/frame of zero-mean
noise on the 54x42 depth that random-walks the position via `Mapper._t_prev`.

Fix (commit 4e202b4, `slam/motion.py` + `Mapper`): a `StationarityGate` classifies a frame stationary
only when mean rotation, mean step, AND directional **coherence** (‖Σincrements‖/Σ‖increment‖) are all
below ceilings. Coherence separates jitter (random dirs → ~1/√window ≈0.32) from real motion (~1); the
rotation ceiling separates a tripod (~0) from an actively-aimed handheld scan (a fixed magnitude deadband
can't — stationary jitter ~11-45 mm/frame overlaps walking ~35 mm/frame).

CRUCIAL design point: the hold de-jitters the **reported/preview pose ONLY**. The TSDF integration and
tracking prior always use the true ICP pose, so a false hold can never corrupt the reconstruction.
Verified: motion scan start-end gap **0.4050 m → 0.4050 m (byte-identical)**; stationary segment reported
step **13.6 mm → 0.00 mm**. An earlier version that held the *actual* pose regressed the motion gap
0.405→1.41 m from ~41 false holds corrupting the map — DON'T do that; keep it display-only.
Knobs on SlamConfig/Mapper (default on): `stationary_hold/window/coherence/step_ceiling/rot_ceiling`.
Relates to the first-person camera smoothing (`_FOLLOW_SMOOTH` 0.25→0.12) in panel.py.
