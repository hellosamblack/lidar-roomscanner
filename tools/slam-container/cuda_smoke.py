"""CUDA path smoke test -- exercises the full SLAM pipeline on CUDA:0 over the
code paths the default translation-mode e2e benchmark does NOT cover, so latent
device/scaling bugs surface here in one shot instead of live.

On the current headless Linux host (RTX 2000 Ada passed through) SLAM runs
in-process on local CUDA:0, so just run it against the host venv:

    host/.venv/bin/python tools/slam-container/cuda_smoke.py

Inside the legacy GPU container ([slam] backend=remote), the host wrapper still
works:

    tools/slam-container/cuda_smoke.ps1     # pipes this into `wslc exec`

Covers, on CUDA:0:
  1. color integrate (reflectance), raycast, BOTH ICP modes (translation +
     6dof), and mesh()/point_cloud() extraction (the marching-cubes path that
     OOMs on-GPU at scale -- must extract on CPU).
  2. sub-phase 6.G: a long run with extraction at the live cadence, asserting a
     DEVICE-MEMORY CEILING. This is the regression guard for the long-scan OOM:
     without `TsdfMap.release_cache_every` the same run grows device memory
     ~5.1 MiB/frame (523 -> 5483 MiB over 1500 frames) and OOMs on an 8 GiB
     card a few hundred frames later.

Exits non-zero on any exception or a breached ceiling.
"""
import sys
import traceback

import numpy as np

from roomscan.slam.gpumem import Nvml
from roomscan.slam.mapper import Mapper
from roomscan.slam.synthscene import SyntheticWalk

W, H = 54, 42

# ---- 6.G memory-ceiling budget ------------------------------------------------
# Steady state for this run is ~520-650 MiB above baseline (the 40k-block VBG is
# ~410 MB of it, allocated up front at construction). 1500 MiB leaves generous
# headroom for driver/allocator variation while still being ~3.5x below where the
# unfixed run sits at the same frame count -- so the guard fires on a real
# regression long before it becomes an OOM, and does not flap.
MEM_CEILING_MIB = 1500.0
# Tail growth must be essentially flat. The fixed run measures 0.005-0.04
# MiB/frame; unfixed is 5.1. Anything above 0.5 is a regression, and the gap
# between 0.04 and 5.1 is wide enough that this threshold needs no tuning.
MAX_GROWTH_MIB_PER_FRAME = 0.5
MEM_FRAMES = 1200
MESH_EVERY = 5              # matches slam/worker.py _MESH_EVERY


def run(mode: str) -> None:
    m = Mapper(W, H, fov_h=55.0, fov_v=42.0, icp_mode=mode, device="CUDA:0")
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    # A tilted plane that translates in depth frame-to-frame: gives ICP a real
    # gradient (not a singular fronto-parallel wall) and adds fresh voxel blocks
    # each frame so the map grows and mesh extraction runs on a non-trivial map.
    for i in range(8):
        depth = (500.0 + 3.0 * xx + 2.0 * yy + 10.0 * i).astype(np.float32)
        refl = ((xx + yy) / (W + H)).astype(np.float32)
        conf = np.full((H, W), 50.0, np.float32)
        m.step(depth, (1.0, 0.0, 0.0, 0.0), 101325.0, reflectance=refl, confidence=conf)
    mesh = m.mesh().cpu()
    pc = m.map_point_cloud().cpu()
    nv = mesh.vertex["positions"].shape[0] if "positions" in mesh.vertex else 0
    npc = pc.point["positions"].shape[0] if "positions" in pc.point else 0
    print(f"mode={mode}: OK 8 steps, mesh_v={nv}, pc_pts={npc}")


def run_memory_ceiling() -> bool:
    """Sub-phase 6.G guard: a long scan with live-cadence extraction must hold a
    device-memory ceiling and a flat tail growth.

    Uses the synthetic walk rather than a recorded capture because the guard
    needs a scan longer than any capture we have, and because it must be
    deterministic run-to-run. Poses are not injected -- `Mapper.step` does its
    own raycast + ICP + integrate, so this measures the production path."""
    nvml = Nvml()
    if not nvml.ok:
        print("6.G memory ceiling: SKIP (libnvidia-ml unavailable, cannot measure)")
        return True

    baseline = nvml.used_bytes()
    walk = SyntheticWalk(W, H)
    m = Mapper(W, H, fov_h=55.0, fov_v=42.0, device="CUDA:0")

    mib, integrated = [], 0
    last_mesh = None            # held, exactly as SlamWorker holds it
    for _ in range(MEM_FRAMES):
        depth, quat = walk.next_frame()
        step = m.step(depth, quat)
        if not step.tracking_lost:
            integrated += 1
            if integrated == 1 or integrated % MESH_EVERY == 0:
                last_mesh = m.mesh()                    # noqa: F841 -- held on purpose
        mib.append((nvml.used_bytes() - baseline) / 2**20)
    nvml.close()

    peak = max(mib)
    half = len(mib) // 2
    x = np.arange(half, len(mib), dtype=np.float64)
    growth = float(np.polyfit(x, np.asarray(mib[half:]), 1)[0])
    releases = m._tsdf.cache_releases

    ok = True
    if peak > MEM_CEILING_MIB:
        print(f"6.G memory ceiling: FAIL peak {peak:.0f} MiB > {MEM_CEILING_MIB:.0f} MiB "
              f"over {MEM_FRAMES} frames ({walk.path_length_m:.0f} m)")
        ok = False
    if growth > MAX_GROWTH_MIB_PER_FRAME:
        print(f"6.G memory ceiling: FAIL tail growth {growth:.3f} MiB/frame > "
              f"{MAX_GROWTH_MIB_PER_FRAME} MiB/frame")
        ok = False
    if releases == 0:
        print("6.G memory ceiling: FAIL no cache releases happened -- "
              "release_cache_every is disabled or the hook is unwired")
        ok = False
    if ok:
        print(f"6.G memory ceiling: OK {MEM_FRAMES} frames ({walk.path_length_m:.0f} m), "
              f"peak {peak:.0f} MiB, tail growth {growth:.3f} MiB/frame, "
              f"{releases} cache releases, {m.tracking_lost_count} lost")
    return ok


def main() -> int:
    import open3d as o3d
    if not o3d.core.cuda.is_available():
        print("CUDA not available in this environment", file=sys.stderr)
        return 2
    for mode in ("translation", "6dof"):
        try:
            run(mode)
        except Exception:
            print(f"mode={mode}: FAIL")
            traceback.print_exc()
            return 1
    try:
        if not run_memory_ceiling():
            return 1
    except Exception:
        print("6.G memory ceiling: FAIL")
        traceback.print_exc()
        return 1
    print("CUDA SMOKE PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
