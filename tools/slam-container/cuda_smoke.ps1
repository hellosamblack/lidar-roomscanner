# Run the CUDA path smoke (tools/slam-container/cuda_smoke.py) INSIDE the running
# roomscan-slam container, which has a real CUDA:0 device. Pipes the script in via
# base64 so no file needs to be baked into the image. Exits with the smoke's code.
# Prereq: the container is running (tools/slam-container/start.ps1).
$ErrorActionPreference = "Stop"
$py = Get-Content -Raw "$PSScriptRoot/cuda_smoke.py"
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($py))
wslc exec roomscan-slam bash -c "echo $b64 | base64 -d | python"
exit $LASTEXITCODE
