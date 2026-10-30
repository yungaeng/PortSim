param([string]$Engine='C:\Program Files\Epic Games\UE_5.6',[string]$NativeLogPath,
    [ValidateSet(0,1,2,4,8,16)][int]$PlaybackRate=4,[switch]$Profile,[switch]$ContinuousTraffic)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$project=Join-Path $root 'PortSim.uproject'
$editor=Join-Path $Engine 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
$stamp=Get-Date -Format 'yyyyMMdd_HHmmss'
$log=Join-Path $root "Saved/Logs/ShipArrival_Units_$stamp.log"
if($NativeLogPath) { $log=(Resolve-Path -LiteralPath $NativeLogPath).Path }
else {
$arguments=@("`"$project`"",'-nullrhi','-nosound','-unattended','-nosplash',
    '"-ExecCmds=Automation RunTests PortSim"','"-TestExit=Automation Test Queue Empty"',"`"-abslog=$log`"")
$process=Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle Hidden -PassThru
$null=$process.Handle
if(-not $process.WaitForExit(1200000)) { $process.Kill(); throw "Native arrival tests timed out: $log" }
$process.Refresh(); if($process.ExitCode -ne 0) { throw "Native runner exited with $($process.ExitCode): $log" }
}
$text=Get-Content -LiteralPath $log -Raw
# The installed editor emits three identical Condition-failed self-checks
# before a test session (also in the historical successful native logs).
# Judge the actual PortSim session and its explicit completion results.
$start=$text.IndexOf('Test Started. Name=')
$session=if($start -ge 0) {$text.Substring($start)} else {''}
$performed=[regex]::Match($text,'Automation Test Queue Empty (\d+) tests performed')
if(-not $performed.Success -or [int]$performed.Groups[1].Value -lt 34 -or
    @([regex]::Matches($session,'Test Completed\. Result=\{Success\}[^\r\n]+Path=\{PortSim\.')).Count -ne [int]$performed.Groups[1].Value -or
    $session -match 'Result=\{Fail|LogAutomationTest: Error:|Fatal error:' -or
    $text -notmatch 'Result=\{Success\} Name=\{ArrivalActorsAndAccess\}') {
    Select-String -LiteralPath $log -Pattern 'Error:|Result=\{Fail' | Select-Object -Last 12
    throw "Native arrival validation failed: $log"
}
Write-Output "PASS 34 native tests: $log"
$summary=Join-Path $root "Saved/Tests/ShipArrival_$stamp.json"
New-Item -ItemType Directory -Path (Split-Path $summary -Parent) -Force | Out-Null
$mode=if($ContinuousTraffic) {'Flow'} else {'Normal'}
& (Join-Path $PSScriptRoot 'TestSite.ps1') -Engine $Engine -Mode $mode -PlaybackRate $PlaybackRate -FixedFPS 10 `
    -TimeoutMinutes 120 -CheckStowage -CheckDispatch -Profile:$Profile -ResultSummaryPath $summary
& node (Join-Path $root '../Dashboard/tests/ship-arrival.mjs')
if($LASTEXITCODE -ne 0) { throw 'Arrival dashboard telemetry validation failed' }
Copy-Item -LiteralPath (Join-Path $root 'Saved/Dashboard/state.json') -Destination (
    Join-Path $root "Saved/Tests/ShipArrival_${stamp}_state.json")
Write-Output "SHIP_ARRIVAL_VALIDATION_PASS: $summary"
