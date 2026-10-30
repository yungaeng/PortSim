param(
    [string]$Engine='C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet(1,2,4,8,16)][int]$PlaybackRate=8,
    [ValidateRange(10,600)][int]$Seconds=60,
    [switch]$Render,
    [switch]$EngineStats,
    [switch]$FullPhysics,
    [switch]$NoReplay,
    [string]$LogPath,
    [string]$SummaryPath
)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
if(-not $LogPath) { $LogPath=Join-Path $root ('Saved/Logs/PlaybackPerf_'+(Get-Date -Format 'yyyyMMdd_HHmmss')+'.log') }
$LogPath=[IO.Path]::GetFullPath($LogPath)
$arguments=@(('"'+(Join-Path $root 'PortSim.uproject')+'"'),'/Engine/Maps/Entry',
    '-game','-nosound','-unattended','-nosplash','-PortSimAutoStart',
    "-PortSimPlayback=$PlaybackRate",'-PortSimProfile',"-PortSimProfileSeconds=$Seconds",
    '-PortSimEnvironmentSeed=166290927',('"-abslog='+$LogPath+'"'))
if($Render) { $arguments+=@('-windowed','-ResX=1280','-ResY=720') }
else { $arguments+='-nullrhi' }
if($EngineStats) { $arguments+='-PortSimProfileEngineStats' }
if($FullPhysics) { $arguments+='-PortSimFullPhysics' }
if($NoReplay) { $arguments+='-PortSimNoReplay' }
$process=Start-Process -FilePath (Join-Path $Engine 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe') `
    -ArgumentList $arguments -WindowStyle Hidden -PassThru
$null=$process.Handle
if(-not $process.WaitForExit(($Seconds+180)*1000)) { $process.Kill(); throw "Playback measurement timed out: $LogPath" }
$process.Refresh()
$text=Get-Content -LiteralPath $LogPath -Raw
if($process.ExitCode -ne 0 -or $text -match 'SITE_LOGISTICS_FAIL:|Fatal error:') { throw "Playback measurement failed: $LogPath" }
$samples=[regex]::Matches($text,'PORTSIM_PERF: wall=([\d.]+) processed_sim=([\d.]+) achieved=([\d.]+)x delivered=(\d+)')
if(-not $samples.Count) { throw "No processed simulation measurement: $LogPath" }
$last=$samples[-1]
$result=[ordered]@{
    RequestedRate=$PlaybackRate;Render=[bool]$Render;FullPhysics=[bool]$FullPhysics;BufferedReplay=(-not [bool]$NoReplay);WallSeconds=[double]$last.Groups[1].Value
    ProcessedSimulationSeconds=[double]$last.Groups[2].Value;AchievedRate=[double]$last.Groups[3].Value
    Delivered=[int]$last.Groups[4].Value;Log=$LogPath
}
if($SummaryPath) { $result | ConvertTo-Json | Set-Content -LiteralPath $SummaryPath -Encoding utf8 }
$result | ConvertTo-Json
Select-String -LiteralPath $LogPath -Pattern 'PORTSIM_PERF_SCOPE:' | Select-Object -Last 12 | ForEach-Object {$_.Line}
