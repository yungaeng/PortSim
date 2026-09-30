param(
    [string]$Engine = 'C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet(8,16)][int[]]$Playback = @(8,16),
    [switch]$FullUnload,
    [switch]$Render,
    [ValidateRange(1,480)][int]$TimeoutMinutes = 120
)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$editor = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
foreach ($rate in $Playback) {
    $mode = if ($FullUnload) { 'FullUnload' } else { 'Site' }
    $log = Join-Path $root "Saved\Logs\Playback_${mode}_${rate}x.log"
    $arguments = @("`"$(Join-Path $root 'PortSim.uproject')`"", '/Engine/Maps/Entry',
        '-game', '-nosound', '-unattended', '-nosplash', "-PortSimPlayback=$rate", "`"-abslog=$log`"")
    $arguments += if ($FullUnload) { '-PortSimFullUnloadTest' } else { '-PortSimSiteTest' }
    $arguments += if ($Render) { @('-RenderOffscreen','-dx11') } else { '-nullrhi' }
    Write-Output "RUN $mode ${rate}x"
    $process = Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle Hidden -PassThru
    $null = $process.Handle
    if (-not $process.WaitForExit($TimeoutMinutes * 60000)) {
        $process.Kill()
        throw "Playback ${rate}x exceeded the wall-clock limit. See $log"
    }
    $process.Refresh()
    $result = Get-Content -LiteralPath $log -Raw
    # Unreal can return zero even after a failed in-game assertion.
    if ($process.ExitCode -ne 0 -or $result -match 'PORTSIM_SITE_FAIL:' -or $result -notmatch 'PORTSIM_SITE_PASS:') {
        Select-String -LiteralPath $log -Pattern 'FAIL|Fatal' | Select-Object -Last 10
        throw "Playback ${rate}x failed. See $log"
    }
    Select-String -LiteralPath $log -Pattern 'PORTSIM_SITE_PASS:|RECEIVING_METRICS:|DISPATCH_METRICS:'
}
