param([string]$Engine = 'C:\Program Files\Epic Games\UE_5.6')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$repositoryRoot = Split-Path $projectRoot -Parent
# MSVC response-file paths can be corrupted by non-ASCII project directories.
# A junction changes the spelling only; sources and build products stay in this repository.
$hasher = [Security.Cryptography.SHA256]::Create()
try { $digest = [BitConverter]::ToString($hasher.ComputeHash([Text.Encoding]::UTF8.GetBytes($repositoryRoot))).Replace('-','').Substring(0,12) }
finally { $hasher.Dispose() }
$aliasRoot = Join-Path ([IO.Path]::GetTempPath()) ('PortSimBuild-' + $digest)
if (Test-Path -LiteralPath $aliasRoot) {
    $aliasItem = Get-Item -LiteralPath $aliasRoot
    if ($aliasItem.LinkType -ne 'Junction' -or [IO.Path]::GetFullPath([string]$aliasItem.Target) -ne $repositoryRoot) {
        throw "Build alias already exists with a different target: $aliasRoot"
    }
} else {
    New-Item -ItemType Junction -Path $aliasRoot -Target $repositoryRoot | Out-Null
}
$project = Join-Path $aliasRoot 'PortSim\PortSim.uproject'
& (Join-Path $Engine 'Engine\Build\BatchFiles\Build.bat') PortSimEditor Win64 Development "-Project=$project" -WaitMutex -NoHotReloadFromIDE -NoUBA -NoUBTMakefiles
if ($LASTEXITCODE -ne 0) { throw "Unreal build failed with exit code $LASTEXITCODE" }
