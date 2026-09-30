param(
    [string]$Engine = 'C:\Program Files\Epic Games\UE_5.6',
    [ValidateRange(2,60)][int]$FixedFPS = 2
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$editor = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$log = Join-Path $root 'Saved\Logs\FullUnloadTest.log'
& $editor (Join-Path $root 'PortSim.uproject') /Engine/Maps/Entry -game -nullrhi -nosound -unattended -nosplash -benchmark "-fps=$FixedFPS" -PortSimFullUnloadTest "-abslog=$log"
if ($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $log -Pattern 'PORTSIM_SITE_PASS:.*600 complete' -Quiet)) {
    throw "Full unloading did not pass. See $log"
}
Select-String -LiteralPath $log -Pattern 'RECEIVING_METRICS:|DISPATCH_METRICS:|PORTSIM_SITE_PASS:'
$rmgs = Select-String -LiteralPath $log -Pattern 'SITE_DELIVERED:.*-> RMG(\d+);' |
    ForEach-Object { $_.Matches[0].Groups[1].Value } | Sort-Object -Unique
Write-Output "RMGs that physically placed cargo: $($rmgs.Count) / 46"
