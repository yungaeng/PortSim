param([string]$Engine = 'C:\Program Files\Epic Games\UE_5.6')
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$project = Join-Path $root 'PortSim.uproject'
$editor = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$log = Join-Path $root "Saved\Logs\VesselCapacity_$PID.log"
$arguments = @("`"$project`"", '/Engine/Maps/Entry', '-game', '-unattended', '-nullrhi',
    '-nosound', '-nosplash', '-PortSimShips=1', '-PortSimCargo1=3000', '-PortSimNoSea',
    '"-ExecCmds=quit"', "`"-abslog=$log`"")
$process = Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle Hidden -PassThru
$null = $process.Handle
if (-not $process.WaitForExit(180000)) { $process.Kill(); throw "Vessel capacity initialization timed out: $log" }
$process.Refresh()
$text = Get-Content -LiteralPath $log -Raw
if ($process.ExitCode -ne 0 -or $text -match 'SITE_LOGISTICS_FAIL:|Fatal error:' -or
    $text -notmatch 'SHIP_ARRIVAL_STOWAGE: vessel=1 applied=3000' -or
    $text -notmatch 'YARD_BOOKING_PLAN: 3000 containers' -or
    $text -notmatch 'SITE_INVENTORY:.*vessel_total=3000,.*site_ship=3000') {
    throw "3000-container vessel initialization or yard booking failed: $log"
}
Write-Output "PASS: 3000 physical vessel containers and supported yard destinations. Log: $log"
