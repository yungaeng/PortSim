param([string]$Engine='C:\Program Files\Epic Games\UE_5.6')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path $PSScriptRoot -Parent
$project=Join-Path $projectRoot 'PortSim.uproject'
$testLog=Join-Path $projectRoot 'Saved\Logs\AGVDynamics_Bench.log'
$editor=Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
& $editor $project -unattended -nullrhi -nosound '-ExecCmds=Automation RunTests PortSim.AGVPhysics.' '-TestExit=Automation Test Queue Empty' "-abslog=$testLog"
$nativeExit=$LASTEXITCODE
$text=Get-Content -LiteralPath $testLog -Raw
if($nativeExit -ne 0 -or $text -notmatch '4 tests performed' -or $text -match 'Test Completed. Result=\{Fail') { throw "AGV physics bench failed. See $testLog" }
$reportPath=Join-Path $projectRoot 'Saved\Tests\AGVDynamics_Report.json'
$report=Get-Content -LiteralPath $reportPath -Raw | ConvertFrom-Json
if(-not $report.valid) { throw 'Invalid benchmark report' }
$report.cases | Select-Object @{n='Case';e={$_.input.id}},braking_distance_m,braking_seconds,peak_slip,peak_ltr,contact_threshold | Format-Table
Write-Output "PASS AGV physics. Web: AGV 물리 시험. Report: $reportPath"
$emergencyPath=Join-Path $projectRoot 'Saved\Tests\AGVEmergency_Report.json'
$emergency=Get-Content -LiteralPath $emergencyPath -Raw | ConvertFrom-Json
if(-not $emergency.valid) { throw 'Invalid emergency report' }
$emergency.cases | Select-Object @{n='Case';e={$_.input.id}},outcome,braking_distance_m,minimum_clearance_m,peak_cargo_shift_m | Format-Table
Write-Output "PASS integrated emergencies. Report: $emergencyPath"
