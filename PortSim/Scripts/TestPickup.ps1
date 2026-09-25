param(
    [string]$Engine = 'C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet('All','offset','eccentric','bias','yaw','seat','pose','lock')][string]$Case = 'All'
)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$editor = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$cases = if ($Case -eq 'All') { @('offset','eccentric','bias','yaw','seat','pose','lock') } else { @($Case) }
foreach ($name in $cases) {
    $log = Join-Path $root "Saved\Logs\Pickup_$name.log"
    $arguments = @("`"$(Join-Path $root 'PortSim.uproject')`"",'/Engine/Maps/Entry','-game','-nullrhi','-nosound','-unattended','-nosplash','-nop4','-benchmark','-fps=10','-PortSimPickupTest',"-PortSimPickupCase=$name","`"-abslog=$log`"")
    if ($name -eq 'seat') { $arguments += '-PortSimSTSSeatFault=2' }
    if ($name -eq 'pose') { $arguments += '-PortSimSTSPoseFault' }
    if ($name -eq 'lock') { $arguments += '-PortSimSTSLockFault=0' }
    Write-Output "RUN pickup $name"
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
