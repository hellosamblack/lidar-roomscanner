# AshOffice matched-trajectory golden fixture

`rtabmap_ashoffice_matched.npz` is the small, real-data regression fixture for issue #188.
It contains the 192 optimized RTAB-Map keyframes from
`260816-101003.db` and the roomscanner trajectory resampled at those keyframe times after
the validated 15.56 s angular-speed clock alignment. The source database (49 MB) and full
roomscanner trajectory are deliberately not checked into git.

The fixture pins the independent findings published with the issue: roomscanner/RTAB path
ratio 0.956, rotation ratio 1.012, final-window absolute XYZ error 0.54/0.57/2.57 m, and
the misleading full-overlap Umeyama similarity scale 0.497. It is input data, not a saved
expected report: tests recompute every metric through `compare_matched_trajectories`.

Regenerate only from the paired 2026-08-16 AshOffice artifacts after first verifying the
clock correlation still peaks at 15.56 s. The fixture stores no RGB, depth, audio, location
metadata, or database contents beyond node timestamps and poses.
