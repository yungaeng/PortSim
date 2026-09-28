param(
    [ValidateRange(1,65535)][int]$Port=4173,
    [switch]$NoBrowser,
    [switch]$Background,
    [string]$NodePath
)
$ErrorActionPreference='Stop'
$repository=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$server=Join-Path $repository 'Dashboard\server.mjs'
$url="http://127.0.0.1:$Port"
function Get-DashboardHealth {
    try { Invoke-RestMethod "$url/api/health" -TimeoutSec 1 } catch { $null }
}
$health=Get-DashboardHealth
if ($health) {
    if ($health.application -ne 'PortSim Observatory' -or $health.repository -ne $repository) {
        throw "Port $Port is serving another workspace. Choose -Port with an unused port."
    }
    Write-Host "PortSim dashboard already running: $url"
    if(-not $NoBrowser) { Start-Process $url }
    return
}
if (-not $NodePath) {
    $command=Get-Command node -ErrorAction SilentlyContinue
    if($command) { $NodePath=$command.Source }
    else {
        $candidates=@(
            (Join-Path $env:ProgramFiles 'nodejs\node.exe'),
            (Join-Path $env:LOCALAPPDATA 'Programs\nodejs\node.exe'),
            (Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe')
        )
        $NodePath=$candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    }
}
if(-not $NodePath -or -not (Test-Path -LiteralPath $NodePath)) {
    throw 'Node.js was not found. Install Node.js or supply -NodePath with the path to node.exe.'
}
$previousPort=$env:PORTSIM_DASHBOARD_PORT
try {
    $env:PORTSIM_DASHBOARD_PORT="$Port"
    if($Background) {
        $logDir=Join-Path $repository 'PortSim\Saved\Logs'
        New-Item -ItemType Directory -Path $logDir -Force | Out-Null
        $process=Start-Process -FilePath $NodePath -ArgumentList @("`"$server`"") -WorkingDirectory $repository -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logDir "Dashboard-$Port.log") -RedirectStandardError (Join-Path $logDir "Dashboard-$Port-error.log")
        for($attempt=0;$attempt -lt 40;$attempt++) {
            if($process.HasExited) { throw "Dashboard exited. See $logDir\Dashboard-$Port-error.log" }
            $health=Get-DashboardHealth
            if($health) { break }
            Start-Sleep -Milliseconds 250
        }
        if(-not $health -or $health.application -ne 'PortSim Observatory' -or $health.repository -ne $repository) {
            if(-not $process.HasExited) { $process.Kill() }
            throw "Dashboard did not start correctly. See $logDir\Dashboard-$Port-error.log"
        }
        Write-Host "PortSim dashboard ready: $url (PID $($process.Id))"
        if(-not $NoBrowser) { Start-Process $url }
    } else {
        Write-Host "PortSim dashboard: $url"
        Write-Host 'Keep this window open. Ctrl+C stops the dashboard. Run Unreal simulation to receive actor data.'
        if(-not $NoBrowser) { Start-Process $url }
        & $NodePath $server
        if($LASTEXITCODE -ne 0) { throw "Dashboard exited with code $LASTEXITCODE" }
    }
} finally { $env:PORTSIM_DASHBOARD_PORT=$previousPort }
