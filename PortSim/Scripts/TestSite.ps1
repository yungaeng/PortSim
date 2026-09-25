param([string]$Engine = 'C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet('All','Normal','Faults','SensorFault','LockFault','Overload','AGVFault')][string]$Mode = 'All',
    [ValidateRange(5,60)][int]$FixedFPS = 10)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$project = Join-Path $root 'PortSim.uproject'
$editor = Join-Path $Engine 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$cases = if ($Mode -eq 'All') { @('Normal','SensorFault','LockFault','Overload','AGVFault') }
    elseif ($Mode -eq 'Faults') { @('SensorFault','LockFault','Overload','AGVFault') } else { @($Mode) }
foreach ($case in $cases) {
    $started = Get-Date
    $testLog = Join-Path $root "Saved\Logs\SiteTest_$case.log"
    $arguments = @("`"$project`"", '/Engine/Maps/Entry','-game','-nullrhi','-nosound','-unattended','-nosplash','-benchmark',"-fps=$FixedFPS",'-PortSimSiteTest',"`"-abslog=$testLog`"")
    $expected = $null
    if ($case -eq 'SensorFault') { $arguments += '-PortSimSTSSensorFault'; $expected = 'Required STS sensor observation invalid/stale' }
    if ($case -eq 'LockFault') { $arguments += '-PortSimSTSLockFault=0'; $expected = 'Twist lock alignment failed' }
    if ($case -eq 'AGVFault') { $arguments += '-PortSimSTSAGVFault'; $expected = 'AGV alignment lost during STS handover' }
    if ($case -eq 'Overload') {
        $settings = Get-Content -LiteralPath (Join-Path $root 'Config\STS_Simulation.json') -Raw | ConvertFrom-Json
        $settings.default_container_mass_kg = 100000
        $dir = Join-Path $root 'Saved\Tests\STS'
        New-Item -ItemType Directory -Path $dir -Force | Out-Null
        $path = Join-Path $dir 'site_overload.json'
        $settings | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath $path -Encoding utf8
        $arguments += "`"-PortSimSTSSettings=$path`""
        $expected = 'Payload exceeds resolved STS reference capacity'
    }
    Write-Output "RUN Site $case"
    $process = Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle Hidden -PassThru
    $null = $process.Handle
    $timeoutMs = if ($case -eq 'Normal') { 1800000 } else { 900000 }
    if (-not $process.WaitForExit($timeoutMs)) { $process.Kill(); throw "Site $case timed out" }
    $process.Refresh()
    $text = Get-Content -LiteralPath $testLog -Raw
    if ($expected) {
        if ($process.ExitCode -notin @(0,1) -or $text -notmatch ('PORTSIM_SITE_FAIL:.*' + [Regex]::Escape($expected)) -or
            $text -notmatch 'FPlatformMisc::RequestExitWithStatus\(0, 1,' -or $text -match 'SITE_HANDOVER:') {
            throw "Site $case did not reject the fault before handover. See $testLog"
        }
    } else {
        if ($process.ExitCode -ne 0 -or $text -notmatch 'PORTSIM_SITE_PASS:') {
            Select-String -LiteralPath $testLog -Pattern 'FAIL|WORKING_CRANE|Fatal' | Select-Object -Last 12
            throw "Site test failed. See $testLog"
        }
        $completed = 0
        $reports = Get-ChildItem -LiteralPath (Join-Path $root 'Saved\Results') -Filter 'Site_*.csv' | Where-Object { $_.LastWriteTime -ge $started }
        foreach ($report in $reports) {
            $rows = @(Import-Csv -LiteralPath $report.FullName)
            if ($rows.Count -ne 120) { continue }
            $completed++
            if (@($rows.STSLane | Sort-Object -Unique).Count -ne 9) { throw 'Not all nine STSs completed shipments' }
            if (@($rows.AGVID | Sort-Object -Unique).Count -ne 60) { throw 'Not all sixty AGVs completed shipments' }
            if (@($rows | Where-Object { [int]$_.STSLane -lt 1 -or [int]$_.STSLane -gt 9 -or [int]$_.RMGID -lt 1 -or [int]$_.RMGID -gt 46 }).Count) { throw 'Invalid crane ID in shipment report' }
            foreach ($row in $rows) {
                $duration = [double]$row.FinalPlacementAtSeconds - [double]$row.StartedAtSeconds
                if ([Math]::Abs($duration - [double]$row.ShipmentSeconds) -gt .0001 -or
                    [double]$row.STSHandoverAtSeconds -lt [double]$row.StartedAtSeconds -or
                    [double]$row.FinalPlacementAtSeconds -lt [double]$row.STSHandoverAtSeconds -or
                    [double]$row.STSSeconds -gt [double]$row.ShipmentSeconds -or [double]$row.PayloadKg -ne 12000) {
                    throw "Site timing/mass mismatch: $($row.ContainerID)"
                }
            }
            $snapshot = Get-Content -LiteralPath ($report.FullName.Replace('.csv','_profile.json')) -Raw | ConvertFrom-Json
            if ($snapshot.sts_profile.reference.geometry.rail_gauge.value -ne 30.48) { throw 'Site reference snapshot missing' }
            Write-Output "TIMING Site: 9 STSs, 120 shipments; final placement $($rows[-1].FinalPlacementAtSeconds) seconds"
        }
        if ($completed -ne 1) { throw 'Expected one completed 120-shipment report' }
    }
    Write-Output "PASS Site $case : $testLog"
}
