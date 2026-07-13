# tools/slam-container

GPU SLAM compute service (Phase 6). Windows captures + renders; this container
runs Mapper on CUDA:0 behind SlamService. See
docs/superpowers/specs/2026-07-13-slam-gpu-container-service-design.md.

- `build.ps1`  — build image roomscan-slam:cuda
- `start.ps1`  — launch detached on 127.0.0.1:5555 (idempotent)
- `stop.ps1`   — stop + remove the container
- logs:  wslc logs roomscan-slam

Enable in roomscan.toml:
    [slam]
    backend = "remote"
    remote_addr = "127.0.0.1:5555"
