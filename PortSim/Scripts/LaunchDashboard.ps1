param([int]$Port=4173, [switch]$NoBrowser)
$ErrorActionPreference='Stop'
$repository=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$server=Join-Path $repository 'Dashboard\server.mjs'
$node=Get-Command node -ErrorAction SilentlyContinue
if(-not $node) { throw 'Node.js is required. Install Node.js, then run this script again.' }
$env:PORTSIM_DASHBOARD_PORT="$Port"
Write-Host "PortSim dashboard: http://127.0.0.1:$Port"
Write-Host 'Keep this window open. Ctrl+C stops the dashboard. Run Unreal simulation to receive actor data.'
if(-not $NoBrowser) { Start-Process "http://127.0.0.1:$Port" }
& $node.Source $server
