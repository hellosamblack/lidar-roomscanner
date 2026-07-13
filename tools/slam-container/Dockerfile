# tools/slam-container/Dockerfile
# GPU SLAM compute service. Open3D 0.19 Linux wheel bundles its CUDA runtime;
# the driver arrives via `wslc run --gpus all`. Only libgl1/libgomp1/libx11-6
# are needed for `import open3d` (verified on this box, 2026-07-13).
FROM python:3.12-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libgomp1 libx11-6 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir "open3d==0.19.0" "numpy>=1.26" "pyserial>=3.5" "psutil>=5.9" "Pillow>=10"

# Install the roomscan package (build context = repo root's host/).
COPY host/ /app/host/
RUN pip install --no-cache-dir --no-deps /app/host

EXPOSE 5555
ENTRYPOINT ["python", "-m", "roomscan.slam.service"]
CMD ["--host", "0.0.0.0", "--port", "5555", "--device", "CUDA:0"]
