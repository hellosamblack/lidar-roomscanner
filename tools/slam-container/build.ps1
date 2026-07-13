# tools/slam-container/build.ps1
# Build roomscan-slam:cuda. Build context is the repo's host/ parent so the
# Dockerfile's `COPY host/` sees the package.
$ErrorActionPreference = "Stop"
$repo = Resolve-Path "$PSScriptRoot/../.."
Write-Host "Building roomscan-slam:cuda from $repo ..."
wslc build -t roomscan-slam:cuda -f "$PSScriptRoot/Dockerfile" "$repo"
Write-Host "Done. Run start.ps1 to launch the detached service."
