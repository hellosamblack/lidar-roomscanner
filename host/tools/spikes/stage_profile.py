"""Per-stage SLAM frame budget on a capture, CPU:0 and CUDA:0 (scratch, /tmp/slam-study)."""
import json, sys, time, threading
import numpy as np
sys.path.insert(0, "/home/sam/git/personal/lidar-roomscanner/host/tools")
from slam_stall_profile import GilWatchdog
import open3d as o3d
from roomscan.slam.cli import _load_frames_maybe_imu
from roomscan.slam.config import SlamConfig
from roomscan.slam.mapper import Mapper
from roomscan.slam import odometry, tsdf as tsdf_mod, mapper as mapper_mod
from roomscan.slam.meshprep import prepare_packet
from roomscan.web import pack_mesh

cap = sys.argv[1]; nframes = int(sys.argv[2]); device = sys.argv[3]
mesh_every = 5
cfg = SlamConfig.load()

t0 = time.perf_counter()
frames, w, h, imu_aux = _load_frames_maybe_imu(cap, nframes, need_imu=True)
t_load = time.perf_counter() - t0
print(f"decode+transform: {len(frames)} frames in {t_load:.2f}s = {1000*t_load/len(frames):.2f} ms/frame (serial, includes file read)", flush=True)

# --- instrument NNS (hybrid_index/hybrid_search) and device<->host copies in register
acc = {"nns_index": 0.0, "nns_search": 0.0, "nns_calls": 0}
_NNS = o3d.core.nns.NearestNeighborSearch
class NNS(_NNS):
    def hybrid_index(self, *a, **k):
        t = time.perf_counter(); r = super().hybrid_index(*a, **k); acc["nns_index"] += time.perf_counter() - t; return r
    def hybrid_search(self, *a, **k):
        t = time.perf_counter(); r = super().hybrid_search(*a, **k); acc["nns_search"] += time.perf_counter() - t; acc["nns_calls"] += 1; return r
odometry.o3d.core.nns.NearestNeighborSearch = NNS

wd = GilWatchdog(); wd.start()
kw = cfg.mapper_kwargs(); kw.update(device=device)
mapper = Mapper(w, h, **kw)

# tag watchdog sub-stages by wrapping the mapper's pieces
_ray = mapper._tsdf.raycast; _integ = mapper._tsdf.integrate
def ray(*a, **k):
    wd.stage = "raycast"; r = _ray(*a, **k); wd.stage = "step_other"; return r
def integ(*a, **k):
    wd.stage = "integrate"; r = _integ(*a, **k); wd.stage = "step_other"; return r
mapper._tsdf.raycast = ray; mapper._tsdf.integrate = integ
_reg = mapper_mod.register_escalating
def reg(*a, **k):
    wd.stage = "icp"; r = _reg(*a, **k); wd.stage = "step_other"; return r
mapper_mod.register_escalating = reg

cols = {k: [] for k in ("total", "tracking", "raycast", "icp", "integrate", "other", "nns_search_per_frame")}
mesh_t, prep_t, pack_t, extract_verts, payload = [], [], [], [], []
integrated = 0; lost = 0; escal0 = 0
wall0 = time.perf_counter()
for i, (depth, refl, conf, quat, pa, t_s) in enumerate(frames):
    imu_raw, off = imu_aux[i] if imu_aux is not None else (None, None)
    nns_before = acc["nns_search"]
    wd.stage = "step_other"
    step = mapper.step(depth, quat, pa, reflectance=refl, confidence=conf, imu_raw=imu_raw, quat_offset_us=off)
    wd.stage = "idle"
    cols["total"].append(step.total_step_ms); cols["tracking"].append(step.t_tracking_ms)
    cols["raycast"].append(step.raycast_ms); cols["icp"].append(step.icp_ms); cols["integrate"].append(step.integrate_ms)
    cols["other"].append(step.total_step_ms - step.t_tracking_ms - step.raycast_ms - step.icp_ms - step.integrate_ms)
    cols["nns_search_per_frame"].append((acc["nns_search"] - nns_before) * 1000)
    if step.tracking_lost:
        lost += 1; continue
    integrated += 1
    if integrated == 1 or integrated % mesh_every == 0:
        wd.stage = "mesh"; t = time.perf_counter(); mesh = mapper.mesh(); mesh_t.append((time.perf_counter() - t) * 1000)
        wd.stage = "prep"; t = time.perf_counter()
        pkt = prepare_packet(mesh, wall_mode="split", glow_origin=None, mesh_seq=integrated, vertex_budget=cfg.live_vertex_budget, decimate=False)
        prep_t.append((time.perf_counter() - t) * 1000)
        wd.stage = "pack"; t = time.perf_counter(); b = pack_mesh(pkt); pack_t.append((time.perf_counter() - t) * 1000)
        wd.stage = "idle"
        extract_verts.append(pkt.source_vertex_count); payload.append(len(b))
wall = time.perf_counter() - wall0
wd.stage = "idle"; wd.stop()

def st(v):
    a = np.asarray(v, float)
    if a.size == 0: return {}
    return {"n": int(a.size), "mean": round(a.mean(), 2), "p50": round(float(np.percentile(a, 50)), 2),
            "p95": round(float(np.percentile(a, 95)), 2), "p99": round(float(np.percentile(a, 99)), 2), "max": round(float(a.max()), 2)}
# step stats over frames that actually ran ICP (skip bootstrap)
ran = np.asarray(cols["icp"]) > 0
rep = {"capture": cap, "device": device, "icp_device": mapper.icp_device, "frames": len(frames), "integrated": integrated, "lost": lost,
       "escalations": mapper.icp_escalations, "zupt": mapper.zupt_count, "held": mapper.held_count,
       "wall_s": round(wall, 2), "loop_fps": round(len(frames) / wall, 1),
       "decode_transform_ms_per_frame": round(1000 * t_load / len(frames), 3),
       "step_ms": {k: st(np.asarray(v)[ran]) for k, v in cols.items()},
       "mesh_extract_ms": st(mesh_t), "prep_ms": st(prep_t), "pack_ms": st(pack_t),
       "final_verts": extract_verts[-1] if extract_verts else 0, "final_payload_mb": round(payload[-1] / 1e6, 2) if payload else 0,
       "nns_index_total_ms_per_icp_frame": round(1000 * acc["nns_index"] / max(1, ran.sum()), 3),
       "nns_search_calls_per_icp_frame": round(acc["nns_calls"] / max(1, ran.sum()), 2),
       "blocks": mapper._tsdf.block_usage(),
       "traj": {"path_m": round(float(sum(np.linalg.norm(np.diff([p[:3, 3] for p in mapper.trajectory], axis=0), axis=1))), 3)}}
wall_by = {"step_other": sum(cols["other"]) / 1000 + sum(cols["tracking"]) / 1000, "raycast": sum(cols["raycast"]) / 1000,
           "icp": sum(cols["icp"]) / 1000, "integrate": sum(cols["integrate"]) / 1000,
           "mesh": sum(mesh_t) / 1000, "prep": sum(prep_t) / 1000, "pack": sum(pack_t) / 1000}
wall_by["idle"] = max(0.0, wall - sum(wall_by.values()))
rep["gil"] = wd.report(wall, wall_by)
print(json.dumps(rep, indent=1))
