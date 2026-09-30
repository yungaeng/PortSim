param(
    [string]$Engine = 'C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet('All','offset','eccentric','bias','yaw','seat','pose','lock','sensor','stack','agv','collision')][string]$Case = 'All',
    [ValidateRange(1,120)][int]$FixedFPS = 10,
    [ValidateRange(0,2)][double]$SimulationFrameSeconds = 0,
    [ValidateSet('STS','RMG')][string]$Crane = 'STS'
)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$editor = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$cases = if ($Case -eq 'All') { @('offset','eccentric','bias','yaw','seat','pose','lock') } else { @($Case) }
if ($Crane -eq 'RMG' -and $Case -eq 'All') { $cases += @('sensor','stack','agv','collision') }
if ($Crane -eq 'STS' -and $Case -in @('sensor','stack','agv','collision')) { throw 'This case requires -Crane RMG' }
foreach ($name in $cases) {
    $log = Join-Path $root "Saved\Logs\Pickup_${Crane}_$name.log"
    $arguments = @("`"$(Join-Path $root 'PortSim.uproject')`"",'/Engine/Maps/Entry','-game','-nullrhi','-nosound','-unattended','-nosplash','-nop4','-benchmark',"-fps=$FixedFPS",'-PortSimPickupTest',"-PortSimPickupCase=$name","`"-abslog=$log`"")
    if ($name -eq 'seat') { $arguments += "-PortSim${Crane}SeatFault=2" }
    if ($SimulationFrameSeconds -gt 0) { $arguments += ('-PortSimPickupFrameSeconds=' + $SimulationFrameSeconds.ToString([Globalization.CultureInfo]::InvariantCulture)) }
    if ($name -eq 'pose') { $arguments += "-PortSim${Crane}PoseFault" }
    if ($name -eq 'lock') { $arguments += "-PortSim${Crane}LockFault=0" }
    if ($Crane -eq 'RMG') { $arguments += '-PortSimRMGTest' }
    if ($name -eq 'sensor') { $arguments += '-PortSimRMGSensorFault' }
    if ($name -eq 'stack') { $arguments += '-PortSimRMGStackFault' }
    Write-Output "RUN $Crane pickup $name"
    $process = Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle Hidden -PassThru
    $null = $process.Handle
    if (-not $process.WaitForExit(240000)) { $process.Kill(); throw "Pickup $name timed out" }
    $process.Refresh()
    $text = Get-Content -LiteralPath $log -Raw
    if ($process.ExitCode -ne 0 -or $text -match 'PORTSIM_PICKUP_FAIL:' -or $text -notmatch "PORTSIM_PICKUP_PASS: ${name}:") {
        Get-Content -LiteralPath $log -Tail 25
        throw "Pickup $name failed; see $log"
    }
    ($text -split "`n" | Select-String 'PORTSIM_PICKUP_PASS:').Line
}
