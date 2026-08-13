# rtabmap_export_min

Tiny synthetic golden fixture for `host/tests/test_splat_rtabmap.py` (issue #158). NOT a real
capture -- no phone/room imagery, just small procedurally-generated placeholder images.

Matches the layout `rtabmap-export` (`introlab/rtabmap` @ `2e193ee1`, `tools/Export/main.cpp`)
writes for:

```
rtabmap-export --images --poses_camera --poses_format 1 --output_dir <this dir> <name>.db
```

(the exact command documented in `docs/rtabmap-pixel10-capture.md` section 6), with
`--output` / base name `session`:

- `session_rgb/<stamp>.jpg`, `session_depth/<stamp>.png`, `session_confidence/<stamp>.png` --
  `main.cpp:1610-1649`. `<stamp>` is the literal `%f`-formatted double RTAB-Map's DB stores per
  node (`main.cpp:1516`, `dbDriver->getNodeInfo`), the same value that appears as column 1 of
  `session_camera_poses.txt` -- that shared stamp string is the exporter's own cross-artifact
  identifier (`main.cpp:1808`: `cameraStamps[i][node] = stamp`).
- `session_calib/<stamp>.yaml` -- one file per frame (`main.cpp:1655-1667`,
  `CameraModel::save()`, `corelib/src/CameraModel.cpp:404-434`): plain nested-mapping YAML
  (`camera_name`, `image_width`, `image_height`, `camera_matrix{rows,cols,data}`, optionally
  `distortion_coefficients`/`rectification_matrix`/`projection_matrix`, `local_transform{rows,
  cols,data}`) -- NOT an `!!opencv-matrix`-tagged node, since `CameraModel::save()` builds the
  block manually with `fs << "camera_matrix" << "{" ...`.
- `session_camera_poses.txt` -- `--poses_format 1` (RGBD-SLAM/"motion capture" convention):
  `#timestamp x y z qx qy qz qw` header then one row per node, in ascending node-id order
  (`corelib/src/Graph.cpp:86-99, exportPoses()`). The two rows here use frame-local identity
  quaternions on purpose -- association/loading correctness is what this fixture is for; the
  axis-convention math itself (`rtabmap.rtabmap_format1_pose_to_world_T_camera_optical`) has its
  own direct unit tests against hand-derived identity/translation/90-rotation cases in
  `test_splat_rtabmap.py`, independent of this fixture.
- `session_cloud.ply` -- optional assembled-cloud artifact (`--cloud`, `main.cpp:2490`), a
  4-vertex ASCII PLY square, to exercise "optional geometry present" without importing a real
  mesh.

Two frames intentionally carry two different `camera_matrix` values (`fx=50` vs `fx=52,fy=53`),
mirroring the exporter's stated per-frame-autofocus behavior -- the reader must keep both, never
coalesce them.
