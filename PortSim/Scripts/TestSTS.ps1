param(
    [string]$Engine = 'C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet('All','Faults','RegressionTail','FleetReset','Profile','Terminal','Smoke','SensorFault','LockFault','Overload')][string]$Mode = 'All'
)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$project = Join-Path $root 'PortSim.uproject'
$editor = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$logDir = Join-Path $root 'Saved\Logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$cases = if ($Mode -eq 'All') { @('Profile','Terminal','Smoke','SensorFault','LockFault','Overload') }
    elseif ($Mode -eq 'RegressionTail') { @('FleetReset','Smoke','SensorFault','LockFault','Overload') }
    elseif ($Mode -eq 'Faults') { @('SensorFault','LockFault','Overload') } else { @($Mode) }
foreach ($case in $cases) {
    $caseStarted = Get-Date
    $log = Join-Path $logDir "STS_$case.log"
    $arguments = @("`"$project`"", '-nullrhi','-nosound','-unattended','-nosplash','-nop4',"`"-abslog=$log`"")
    $expectedFailure = $null
    if ($case -eq 'Profile') {
        $arguments += @('"-ExecCmds=Automation RunTests PortSim.STS"','"-TestExit=Automation Test Queue Empty"')
        $passPattern = 'Result=\{Success\}.*PortSim.STS.ReferenceAndInterlocks'
    } else {
        $arguments += @('/Engine/Maps/Entry','-game','-benchmark','-fps=20')
        if ($case -eq 'Smoke') { $arguments += '-PortSimSmokeTest'; $passPattern = 'PORTSIM_SMOKE_PASS:' }
        else { $arguments += '-PortSimTerminalTest'; $passPattern = 'PORTSIM_TERMINAL_PASS:' }
        if ($case -eq 'FleetReset') { $arguments += '-PortSimFleetResetTest' }
        if ($case -eq 'SensorFault') { $arguments += '-PortSimSTSSensorFault'; $expectedFailure = 'Required STS sensor observation invalid/stale' }
        if ($case -eq 'LockFault') { $arguments += '-PortSimSTSLockFault=0'; $expectedFailure = 'Twist lock alignment failed' }
        if ($case -eq 'Overload') {
            $testDir = Join-Path $root 'Saved\Tests\STS'
            New-Item -ItemType Directory -Path $testDir -Force | Out-Null
            $settings = Get-Content -LiteralPath (Join-Path $root 'Config\STS_Simulation.json') -Raw | ConvertFrom-Json
            $settings.default_container_mass_kg = 100000
            $settingsPath = Join-Path $testDir 'overload.json'
            $settings | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath $settingsPath -Encoding utf8
            $arguments += "`"-PortSimSTSSettings=$settingsPath`""
            $expectedFailure = 'Payload exceeds resolved STS reference capacity'
        }
    }
    Write-Output "RUN $case"
    $process = Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle Hidden -PassThru
    $null = $process.Handle
    if (-not $process.WaitForExit(1200000)) { $process.Kill(); throw "$case timed out after 20 minutes" }
    $process.Refresh()
    if (-not (Test-Path -LiteralPath $log)) { throw "$case created no log" }
    $text = Get-Content -LiteralPath $log -Raw
    if ($expectedFailure) {
        # UE's orderly Windows shutdown can return 0 despite RequestExitWithStatus(false, 1).
        # Require our exact fault result and requested failure status; reject crashes and any completed transfer.
        if ($process.ExitCode -notin @(0,1) -or
            $text -notmatch 'FPlatformMisc::RequestExitWithStatus\(0, 1,' -or
            $text -notmatch ('PORTSIM_TERMINAL_FAIL:.*' + [Regex]::Escape($expectedFailure)) -or $text -match 'FLEET_JOB ') {
            throw "$case did not reject the injected fault before transfer (process exit $($process.ExitCode)). See $log"
        }
    } elseif ($process.ExitCode -ne 0 -or $text -notmatch $passPattern) {
        Get-Content -LiteralPath $log -Tail 30
        throw "$case failed. See $log"
    }
    if ($case -eq 'Profile' -and ($text -match 'Result=\{Fail' -or $text -notmatch 'Result=\{Success\}.*PortSim.STS.SuspensionAndDrives' -or $text -notmatch 'Result=\{Success\}.*PortSim.STS.SensorPickup')) {
        throw "STS dynamics automation failed or did not run. See $log"
    }
    if ($case -eq 'Terminal') {
        $completedBatches = 0
        $reports = Get-ChildItem -LiteralPath (Join-Path $root 'Saved\Results') -Filter 'Terminal_*.csv' |
            Where-Object { $_.Name -notlike '*_stages.csv' -and $_.LastWriteTime -ge $caseStarted }
        foreach ($report in $reports) {
            $rows = @(Import-Csv -LiteralPath $report.FullName)
            if ($rows.Count -ne 24) { continue }
            $completedBatches++
            $cumulative = 0.0
            foreach ($row in $rows) {
                $parts = [double]$row.STSActiveSeconds + [double]$row.FleetPrepareSeconds + [double]$row.FleetDeliverySeconds + [double]$row.PausedSeconds
                $cumulative += [double]$row.SimulationSeconds
                if ([Math]::Abs($parts - [double]$row.SimulationSeconds) -gt 0.0001 -or
                    [Math]::Abs($cumulative - [double]$row.BatchElapsedSeconds) -gt 0.0001 -or
                    [double]$row.STSHandoverAtSeconds -gt [double]$row.FinalPlacementAtSeconds -or
                    [double]$row.FinalPlacementAtSeconds -gt [double]$row.BatchElapsedSeconds) {
                    throw "Simulation timing/event accounting failed: $($report.Name), $($row.ContainerID)"
                }
            }
            $base = [IO.Path]::Combine($report.DirectoryName, $report.BaseName)
            $snapshot = Get-Content -LiteralPath ($base + '_profile.json') -Raw | ConvertFrom-Json
            if ($snapshot.reference.geometry.rail_gauge.value -ne 30.48 -or $snapshot.simulation_assumptions.kind -ne 'simulation_assumptions_not_manufacturer_data') {
                throw "Missing applied reference/assumption snapshot: $($report.Name)"
            }
            Write-Output "TIMING $($rows[0].Direction): 24 jobs, final placement $($rows[-1].FinalPlacementAtSeconds) simulation seconds"
        }
        if ($completedBatches -ne 2) { throw 'Expected one full unload and one full load report' }
    }
    Write-Output "PASS $case : $log"
}
