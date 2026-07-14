"""CUDA path smoke test -- exercises the full SLAM pipeline on CUDA:0 over the
code paths the default translation-mode e2e benchmark does NOT cover, so latent
device/scaling bugs surface here in one shot instead of live. Run INSIDE the
GPU container (it needs a real CUDA device):

    tools/slam-container/cuda_smoke.ps1     # host wrapper: pipes this into `wslc exec`

Covers, on CUDA:0: color integrate (reflectance), raycast, BOTH ICP modes
(translation + 6dof), and mesh()/point_cloud() extraction (the marching-cubes
path that OOMs on-GPU at scale -- must extract on CPU). Exits non-zero on any
exception. This is the regression guard for the "device-parameterized" pipeline
that was never actually run on CUDA until the GPU container existed.
"""
import sys
import traceback

import numpy as np

from roomscan.slam.mapper import Mapper

W, H = 54, 42


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
    print("CUDA SMOKE PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
