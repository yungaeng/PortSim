param(
    [string]$Engine='C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet(1,2,4,8,16)][int]$PlaybackRate=8,
    [ValidateRange(10,3600)][int]$Seconds=60,
    [ValidateRange(1,3)][int]$Ships=3,
    [ValidateRange(1,3000)][int]$CargoPerShip=200,
    [switch]$Render,
    [switch]$EngineStats,
    [switch]$FullPhysics,
    [switch]$SingleThreadedPhysics,
    [switch]$BatchLivePhysics,
    [switch]$NoReplay,
    [switch]$NoSea,
    [switch]$RepeatUnchangedWork,
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
$arguments+="-PortSimShips=$Ships"
for($ship=1;$ship -le $Ships;$ship++) { $arguments+="-PortSimCargo${ship}=$CargoPerShip" }
if($Render) { $arguments+=@('-windowed','-ResX=1280','-ResY=720') }
else { $arguments+='-nullrhi' }
if($EngineStats) { $arguments+='-PortSimProfileEngineStats' }
if($BatchLivePhysics) { $arguments+='-PortSimBatchLivePhysics' }
if($SingleThreadedPhysics) { $arguments+='-SingleThreadedPhysics' }
if($FullPhysics) { $arguments+='-PortSimFullPhysics' }
if($RepeatUnchangedWork) { $arguments+='"-ExecCmds=PortSim.SkipUnchangedUpdates 0"' }
if($NoReplay) { $arguments+='-PortSimNoReplay' }
if($NoSea) { $arguments+='-PortSimNoSea' }
$process=Start-Process -FilePath (Join-Path $Engine 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe') `
    -ArgumentList $arguments -WindowStyle Hidden -PassThru
$null=$process.Handle
$watch=[Diagnostics.Stopwatch]::StartNew()
$memorySamples=@(); $nextMemorySample=0.; $peakWorkingSet=0L
while(-not $process.WaitForExit(1000)) {
    $process.Refresh()
    if($process.HasExited) { break }
    $workingSet=$process.WorkingSet64
    $peakWorkingSet=[Math]::Max($peakWorkingSet,$workingSet)
    if($watch.Elapsed.TotalSeconds -ge $nextMemorySample) {
        $memorySamples += [pscustomobject]@{ProcessSeconds=$watch.Elapsed.TotalSeconds;WorkingSetMB=$workingSet/1MB}
        $nextMemorySample=$watch.Elapsed.TotalSeconds+10
    }
    if($watch.Elapsed.TotalSeconds -gt $Seconds+180) { $process.Kill(); throw "Playback measurement timed out: $LogPath" }
}
$process.Refresh()
$text=Get-Content -LiteralPath $LogPath -Raw
if($process.ExitCode -ne 0 -or $text -match 'SITE_LOGISTICS_FAIL:|Fatal error:') { throw "Playback measurement failed: $LogPath" }
$inventory=[regex]::Matches($text,'SITE_INVENTORY: yard_before=\d+, vessel_total=(\d+), yard_initial=(\d+)')
if(-not $inventory.Count -or [int]$inventory[-1].Groups[1].Value -ne $Ships*$CargoPerShip) {
    throw "Playback workload differs from requested ship/cargo counts: $LogPath"
}
$samples=[regex]::Matches($text,'PORTSIM_PERF: wall=([\d.]+) processed_sim=([\d.]+) achieved=([\d.]+)x delivered=(\d+)')
if(-not $samples.Count) { throw "No processed simulation measurement: $LogPath" }
$last=$samples[-1]
$intervals=@()
$previousWall=0.; $previousSimulation=0.
foreach($sample in $samples) {
    $wall=[double]$sample.Groups[1].Value
    $simulation=[double]$sample.Groups[2].Value
    $duration=$wall-$previousWall
    if($duration -gt 0) {
        $intervals += [pscustomobject]@{WallSeconds=$wall;DurationSeconds=$duration;AchievedRate=($simulation-$previousSimulation)/$duration}
    }
    $previousWall=$wall; $previousSimulation=$simulation
}
# Scopes are inclusive and may overlap; worker output is not game-thread time.
$metricMap=@{}
foreach($metric in [regex]::Matches($text,'PORTSIM_PERF_SCOPE: (\w+) total_ms=([\d.]+) calls=(\d+) mean_ms=([\d.]+)')) {
    $name=$metric.Groups[1].Value
    $metricMap[$name]=[pscustomobject]@{
        Name=$name;TotalMs=[double]$metric.Groups[2].Value;Calls=[long]$metric.Groups[3].Value
        MeanMs=[double]$metric.Groups[4].Value
        MsPerSimulationSecond=[double]$metric.Groups[2].Value/[Math]::Max(.001,[double]$last.Groups[2].Value)
        Thread=if($name -eq 'dashboard_serialize_write') {'worker'} else {'game'}
    }
}
$result=[ordered]@{
    SkipUnchangedUpdates=(-not [bool]$RepeatUnchangedWork)
    ScopeSemantics="Inclusive; do not sum overlapping scopes or worker time";Scopes=@($metricMap.Values | Sort-Object TotalMs -Descending)
    Ships=$Ships;CargoPerShip=$CargoPerShip
    SeaMotion=(-not [bool]$NoSea)
    InitialShipCargo=[int]$inventory[-1].Groups[1].Value;InitialYardCargo=[int]$inventory[-1].Groups[2].Value
    RequestedRate=$PlaybackRate;BatchLivePhysics=[bool]$BatchLivePhysics;Render=[bool]$Render;SingleThreadedPhysics=[bool]$SingleThreadedPhysics;FullPhysics=[bool]$FullPhysics;BufferedReplay=(-not [bool]$NoReplay);WallSeconds=[double]$last.Groups[1].Value
    ProcessedSimulationSeconds=[double]$last.Groups[2].Value;AchievedRate=[double]$last.Groups[3].Value
    Delivered=[int]$last.Groups[4].Value;Log=$LogPath
    MinimumIntervalRate=($intervals.AchievedRate | Measure-Object -Minimum).Minimum
    MaximumIntervalRate=($intervals.AchievedRate | Measure-Object -Maximum).Maximum
    IntervalsBelow95Percent=@($intervals | Where-Object {$_.AchievedRate -lt .95*$PlaybackRate}).Count
    Intervals=$intervals
    PeakWorkingSetMB=$peakWorkingSet/1MB; MemorySamples=$memorySamples
}
if($SummaryPath) { $result | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $SummaryPath -Encoding utf8 }
$result | ConvertTo-Json -Depth 5
Select-String -LiteralPath $LogPath -Pattern 'PORTSIM_PERF_SCOPE:' | Select-Object -Last 12 | ForEach-Object {$_.Line}
