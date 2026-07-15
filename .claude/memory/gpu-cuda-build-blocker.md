---
name: gpu-cuda-build-blocker
description: "RESOLVED — native Windows Open3D CUDA build is a dead end (OS too new for CUDA 12.6); GPU runs via the WSL container instead"
metadata: 
  node_type: memory
  type: project
  originSessionId: 7d93d939-338d-4c18-a5c5-e1fcc31ed98d
---

**RESOLUTION (2026-07-13): DON'T build Open3D CUDA on Windows — use the WSL GPU
container already in the repo (`tools/slam-container/`).** CUDA 12.6 (only toolkit
with nvcc here) rejects this machine's Win11 build 26200 as "unsupported OS"
regardless of MSVC/SDK; no full CUDA 13.x is installed. Instead `wslc` (Microsoft's
built-in WSL Container CLI, `C:\Program Files\WSL\wslc`) + `--gpus all` gives the
container the RTX 4080, and the **Open3D 0.19 Linux pip wheel HAS working CUDA**
(`o3d.core.cuda.is_available()`==True in-container). Dockerfile is just
`pip install open3d==0.19.0` — no build. Flow: `tools/slam-container/{build,start,
stop}.ps1`; panel routes via `slam.backend.make_slam_worker` (reads `[slam]
backend="remote"` + `remote_addr` from `%APPDATA%/roomscan/roomscan.toml`, falls
back to local CPU if unreachable). Measured 2.0x speedup (8.94 vs 18.30 ms/frame).
Two CUDA-only bugs fixed (commit 8258f2d): VBG intrinsic/extrinsic must be CPU:0
(tsdf.py `_CPU`); `nns.hybrid_index(max_dist)` radius required on GPU (odometry.py).
CAVEAT: viewer `config.save()` rewrites the toml with only `[viewer]`, clobbering
`[slam]` — re-add if needed. Everything below is HISTORICAL (the dead-end native path).

Building Open3D 0.19 from source with CUDA (for GPU SLAM) on this machine is blocked
by a host-compiler mismatch, not a code problem: the only installed MSVC is **14.51**
(VS BuildTools 18 / "VS 2026"), which is newer than any installed CUDA toolkit
supports.

- **CUDA 13.3** + MSVC 14.51: gets past compiler-id but its bundled `stdgpu` uses
  CUDA-13-removed API (`clockRate`) + needs `/Zc:preprocessor` → source-patch territory.
- **CUDA 12.6** + MSVC 14.51: `host_config.h` hard-rejects >VS2022; `-allow-unsupported-compiler`
  overrides the gate but then `cudafe++` **crashes (ACCESS_VIOLATION)** on 14.51 headers — unfixable by flags.
- **CUDA 12.6** + MSVC **14.39** (installed VS2022 BuildTools, 2026-07-11): passes the version gate
  (MSVC 19.39, _MSC_VER 1939 is in host_config's 1910-1949 range) but nvcc then fails
  `nvcc fatal : Host compiler targets unsupported OS.` — a nvcc.exe-INTERNAL OS check (not host_config.h),
  **identical for SDK 22621 AND 26100**, so it's not the SDK. Root cause: this machine runs **Windows 11
  build 26200** (a 2026 preview), newer than CUDA 12.6 (2024) whitelists. No `-allow-unsupported-compiler`
  equivalent for the OS check.

**DEFINITIVE (2026-07-11): CUDA 12.6 — the only toolkit with a working nvcc here — cannot target this
OS. Real fix = install a FULL CUDA 13.x Toolkit (with the compiler/nvcc component; the present `v13.3` is
runtime-only, bin+lib, no nvcc). CUDA 13.x supports Win11 26200 + MSVC 14.39/14.44.** Then build with cl
14.39 (works: compiled an STL exe fine) and handle stdgpu's CUDA-13 source issues (clockRate removed +
`/Zc:preprocessor`). VS2022 BuildTools install is TOOLSET-ONLY (no vcvars64.bat); set MSVC+SDK env
explicitly or via `Common7\Tools\VsDevCmd.bat -vcvars_ver=14.39`. Note: the Bash tool's `cmd /c` wrapper
hangs on cl — launch builds via the PowerShell tool instead.

**(historical) Fix attempt = MSVC 14.39 (VS 2022 17.9)** — necessary but NOT sufficient (OS check above). Two ways:
1. `winget install Microsoft.VisualStudio.2022.BuildTools ... --add Microsoft.VisualStudio.Component.VC.14.39.17.9.x86.x64 --add ...Windows11SDK.22621` — needs UAC (winget can't elevate from a bg/agent shell → exit 1602 "cancelled"; user must run it in their own session via `! <cmd>`).
2. Portable/no-admin MSVC (portable-msvc.py gist) — downloads the SAME 14.39 toolset + SDK 22621 to `C:\Users\hello\o3dbuild\msvc`; the auto-mode classifier blocked running the external gist without explicit user OK. (Its Windows msi-unpack loop has a `NameError: m` bug — fixed the copy in job tmp: `for m in msi:`.)

Everything else is STAGED and ready: Open3D source at `C:\Open3D` (build tree `C:\Open3D\build`);
standalone cmake 3.31.6 at `C:\Users\hello\o3dbuild\cmake331`; ninja via WinGet Links; two local
Open3D source patches — `3rdparty/stdgpu/stdgpu.cmake` (CUDA-13 thrust/CCCL include fix, harmless on 12.6)
and `cpp/.../filament/FilamentScene.cpp` (skip `srgbColor` on `defaultUnlitTransparency` → kills the
per-frame Filament warning). Build batches in job tmp target arch `89-real` (RTX 4080 Ada), BUILD_GUI=ON,
BUILD_WEBRTC=OFF, `install-pip-package`. Host code already auto-uses CUDA via [[slam-stationary-jitter]]'s
sibling commit (`slam.config.preferred_device()` → CUDA:0 when `o3d.core.cuda.is_available()`).
