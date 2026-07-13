# tools/slam-container/start.ps1
# Launch the GPU SLAM service detached on 127.0.0.1:5555. Idempotent: if a
# container named roomscan-slam already runs, do nothing.
$ErrorActionPreference = "Stop"
$name = "roomscan-slam"
$running = (wslc list 2>$null) -match $name
if ($running) { Write-Host "$name already running."; exit 0 }
# remove any stopped leftover with the same name
wslc remove $name 2>$null | Out-Null
wslc run -d --name $name --gpus all --publish 5555:5555 roomscan-slam:cuda
Write-Host "Started $name. Check: wslc logs $name"
