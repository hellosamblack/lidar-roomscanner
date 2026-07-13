# tools/slam-container/stop.ps1
$ErrorActionPreference = "SilentlyContinue"
wslc stop roomscan-slam | Out-Null
wslc remove roomscan-slam | Out-Null
Write-Host "Stopped roomscan-slam."
