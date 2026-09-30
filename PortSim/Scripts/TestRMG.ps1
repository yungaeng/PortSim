param(
    [string]$Engine = 'C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet('All','Profile','Pickup')][string]$Mode = 'All'
)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
if ($Mode -in @('All','Profile')) {
    $editor = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
    $log = Join-Path $root 'Saved\Logs\RMG_Profile.log'
    $arguments = @("`"$(Join-Path $root 'PortSim.uproject')`"",'-nullrhi','-nosound','-unattended','-nosplash','-nop4',
        '"-ExecCmds=Automation RunTests PortSim.RMG"','"-TestExit=Automation Test Queue Empty"',"`"-abslog=$log`"")
    $process = Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle Hidden -PassThru
    $null = $process.Handle
    if (-not $process.WaitForExit(240000)) { $process.Kill(); throw 'RMG profile test timed out' }
    $process.Refresh()
    $text = Get-Content -LiteralPath $log -Raw
    if ($process.ExitCode -ne 0 -or $text -match 'Result=\{Fail' -or $text -notmatch 'Result=\{Success\}.*PortSim.RMG.ReferenceAndLimits') {
        throw "RMG profile test failed. See $log"
    }
    Write-Output 'PASS RMG reference and limits'
}
if ($Mode -in @('All','Pickup')) { & (Join-Path $PSScriptRoot 'TestPickup.ps1') -Engine $Engine -Crane RMG }
