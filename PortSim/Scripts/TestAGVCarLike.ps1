param(
    [string]$Engine = 'C:\Program Files\Epic Games\UE_5.6',
    [ValidateRange(1,60)][int]$FixedFPS = 5,
    [ValidateRange(1,30)][int]$TimeoutMinutes = 20,
    [ValidateSet('CarLike','FirstWave')][string]$Mode = 'CarLike'
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$project = Join-Path $projectRoot 'PortSim.uproject'
$editor = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$log = Join-Path $projectRoot 'Saved\Logs\AGVCarLikeFirstWave.log'
$modeArgument = if ($Mode -eq 'FirstWave') { '-PortSimFirstWaveTest' } else { '-PortSimCarLikeTest' }
$passPattern = if ($Mode -eq 'FirstWave') { 'PORTSIM_FIRST_WAVE_PASS:' } else { 'PORTSIM_AGV_CARLIKE_PASS:' }
$arguments = @("`"$project`"",'/Engine/Maps/Entry','-game','-nullrhi','-nosound','-unattended','-nosplash',
    '-benchmark',"-fps=$FixedFPS",$modeArgument,"`"-abslog=$log`"")
if (Test-Path -LiteralPath $log) { Remove-Item -LiteralPath $log -Force }
$process = Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle Hidden -PassThru
$null = $process.Handle
if (-not $process.WaitForExit($TimeoutMinutes*60000)) {
    $process.Kill()
    throw "$Mode AGV test timed out"
}
$process.Refresh()
if (-not (Test-Path -LiteralPath $log)) { throw 'Car-like AGV test created no log' }
$text = Get-Content -LiteralPath $log -Raw
if ($process.ExitCode -ne 0 -or $text -notmatch $passPattern -or
    $text -match 'AGV \d+ used forbidden lateral motion') {
    Select-String -LiteralPath $log -Pattern 'PORTSIM_AGV_CARLIKE|PORTSIM_FIRST_WAVE|SITE_LOGISTICS_FAIL|AGV route|forbidden lateral|BLOCK_CHAIN' |
        Select-Object -Last 20
    throw "$Mode AGV test failed. See $log"
}
Select-String -LiteralPath $log -Pattern $passPattern | Select-Object -Last 1
