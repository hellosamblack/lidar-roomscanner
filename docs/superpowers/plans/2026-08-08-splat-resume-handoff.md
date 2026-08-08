# Splat build resume — running handoff

**Purpose:** survive an agent-context crash without losing the splat build state.
A prior session crashed (ran out of conversation memory) while babysitting a
~45-min build in-context; its `/tmp` log/json were lost. This doc is the durable
record so any session can resume. Plan file:
`/home/sam/.claude/plans/swift-whistling-candle.md`.

## Memory / OOM verdict (CORRECTED 2026-08-08 after a real OOM)
- **8 GB VRAM is the only real ceiling, and 2M does NOT fit.** The synthetic
  `/tmp/vram_probe.py` said 2M peaks at ~2.6 GiB — **it was wrong** (a uniform-cube
  scene under-estimates the real rasterizer's tile-overlap cost). The real Sam Office
  build **OOM'd at n=2M near step 12000**: a stochastic worst-case frame's backward
  needed ~6.7 GiB and a +1.67 GiB alloc blew past the 7.64 GiB card. The per-1000-step
  `vram=` log under-reports (it catches worst-seen-so-far, not the true worst frame).
- **Real n→vram curve (from the dead run):** 535k→1.13, 872k→1.31, 1.42M→1.93,
  2M→2.75 climbing to 3.46 then OOM(~6.7). Worst-case ≈ 2× the logged value.
- **Fix: `--max-gaussians 1300000`** for ALL builds (~5.5 GiB worst-case, ~2 GiB
  margin; the old 1M build completed here, so 1.3M is a safe step up). Same cap on
  builds 1+2 keeps the depth A/B clean. Depth builds also free the depth model off-GPU
  (`train.py`). Web server holds **0** GPU (confirmed), so it isn't the contention.
- **The snowglobe fix is the TUNING (min_opacity/regularizers/cull), not raw count** —
  so 1.3M tuned still far beats the old 1M snowglobe; count is secondary.
- **RAM is bounded** by `max_frames=300` → frame cache ≤ ~5.2 GiB; box has 40 GiB.
- **Fidelity-preserving safeguards in effect:** `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`,
  `cap_max=2M` kept (not lowered), per-1000-step `vram=` logging in `train.py`, and
  **detached execution** (the real crash was the agent, not the build).
- **Live watch:** `grep vram= <log>` — if VRAM trends toward ~7 GiB as N→2M, abort
  and lower `--max-gaussians` (fidelity-preserving fallbacks in order: expandable
  (done), sh_degree 3→2, then a measured cap).

## Build queue (serialized on the GPU — ONE at a time; a 2nd concurrent build OOMs)
| # | Name | Video | Depth | Status |
|---|---|---|---|---|
| 1 | Sam Office | captures/PXL_20260807_044117174.mp4 | off | **← in progress** |
| 2 | Sam Office (depth) | captures/PXL_20260807_044117174.mp4 | `--depth-lambda 0.1` | queued |
| 3 | Sam Office 2 (depth) | captures/PXL_20260808_022604031.mp4 | `--depth-lambda 0.1` | queued |

Depth A/B = build 1 vs build 2 (same video). Builds 2–3 run through the new web
capture-viewer UI (or CLI) after build 1 finishes.

## Build 1 — exact relaunch command
```sh
cd /home/sam/git/personal/lidar-roomscanner/host
export CUDA_HOME=/usr/local/cuda-13.3
export PATH="$PWD/.venv/bin:/usr/local/cuda-13.3/bin:$PATH"
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
nohup .venv/bin/python -m roomscan.splat.cli --results-dir ../results \
  --json ../results/splats/_build_sam-office.json \
  build ../captures/PXL_20260807_044117174.mp4 --name "Sam Office" --force --keep-work \
  > ../results/splats/_build_sam-office.log 2>&1 &
```
- **Log:** `results/splats/_build_sam-office.log` (durable, NOT /tmp)
- **JSON report (commit marker):** `results/splats/_build_sam-office.json` — has
  `"ok": true` when done.
- **Output:** `results/splats/sam-office/{point_cloud.ply,manifest.json}`
- `--keep-work` keeps the COLMAP work dir (a `splat-sam-office-*` temp) so a
  crash-resume skips the ~10-min SfM. Find it: `ls -dt /tmp/splat-sam-office-*`.

## Stage map (what the log shows)
`[frames] …` → `[sfm] extracting/matching/incremental mapping` (~CPU, minutes) →
`[sfm] registered N/M frames` → `[train] step k/15000 … vram=…` (GPU, ~30 min) →
`[train] culled …` → `[train] done … (peak vram … GiB)` → `[splat] done in Ns`.

## If it crashes / how to check status
```sh
pgrep -af roomscan.splat.cli                 # is it alive?
tail -5 results/splats/_build_sam-office.log # where is it
grep -c '"ok": true' results/splats/_build_sam-office.json 2>/dev/null  # done?
```
- **Build process died mid-run:** re-run the relaunch command. With `--keep-work`
  the SfM is redone unless you point `--work-dir` at the kept temp (CLI has
  `--keep-work` but build_splat regenerates the work dir on a fresh invocation, so a
  clean re-run is simplest; SfM on this video is ~10 min).
- **Agent (this session) died, build still alive:** nothing to do — it's detached.
  Reattach by tailing the log; the JSON appears on completion.

## Current position (update me at each milestone)
- 2026-08-08: Phase A done (OOM gate passed, trainer instrumented). Build 1 LAUNCHED
  detached (PID 6401, work `/tmp/splat-sam-office-adgypjue`).
- 2026-08-08: Build 1 **training** — SfM registered 213/287 frames, 36.6k points;
  VRAM flat `~1.13 GiB` through n≈329k @ step 5000 (well under the 2.6 GiB@2M probe).
- 2026-08-08: **Phase C web capture-viewer DONE + verified.** cli `--progress-file` +
  full knobs; sidecar `list_source_videos`/`splat_defaults` + tolerant `list_splats`
  (imported Scaniverse now viewable, count from PLY header); web.py `SplatRunner`
  (subprocess-isolated, single-build lock + pgrep cross-process guard),
  `sanitize_video_name`, `GET /capture_video/{name}`, 4 new `/ws` messages; frontend
  capture viewer + settings form + build banner + resource bars. 491 tests green
  (10 new). Headless UI verified: both mp4s + Scaniverse(imported) list; GPU guard
  refuses a web build while build 1 runs ("GPU busy"). web-protocol.md updated.
  roomscan-web is UP (pid via rig; live device streaming).
- 2026-08-08: **Phase E spec correction DONE** (Pixel/no-LiDAR; depth prior promoted).
- 2026-08-08: **Builds 2–3 QUEUED, detached** — `results/splats/_splat_queue.sh`
  (nohup, PID 23615, log `_splat_queue.log`). It waits for build 1's json, then runs
  build 2 `Sam Office (depth)` (old mp4, `--depth-lambda 0.1`) → build 3
  `Sam Office 2 (depth)` (new mp4, `--depth-lambda 0.1`), one at a time. Per-build
  logs/json at `_build_sam-office-depth.*` and `_build_sam-office-2-depth.*`. The
  queue prints `[queue] ALL BUILDS DONE` at the end.
- **REMAINING:** depth A/B analysis (task #5) — compare builds 1↔2 (same video, depth
  off vs on) on the spec's metrics (median opacity, fraction<0.1, scale/radius
  percentiles). Code is uncommitted (shared tree — attribute hunks before staging).

## Queue status check
```sh
tail -3 results/splats/_splat_queue.log            # queue position
for s in sam-office sam-office-depth sam-office-2-depth; do
  printf "%-22s " "$s"; grep -c '"ok": true' results/splats/_build_$s.json 2>/dev/null || echo 0
done
```
