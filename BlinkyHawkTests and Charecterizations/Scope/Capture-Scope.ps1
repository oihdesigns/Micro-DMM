<#
.SYNOPSIS
  Capture every displayed channel from a Siglent SDS800X HD over LAN into one CSV, with notes and
  multiple test conditions (e.g. "button pressed" / "button released") side by side.

.EXAMPLE
  .\Capture-Scope.ps1
  .\Capture-Scope.ps1 -MaxPoints 0            # full record, no decimation
  .\Capture-Scope.ps1 -Channels C1,C3 -Ip 192.168.10.2
#>
param(
    [string]$Ip = '192.168.10.2',
    [int]$Port = 5025,
    [string]$OutDir = (Join-Path $PSScriptRoot 'Captures'),
    # Default: whatever channels are turned on at the moment of each capture.
    [string[]]$Channels,
    # Decimate on the scope when a record is longer than this (Excel tops out at 1,048,576 rows). 0 = full record.
    [int]$MaxPoints = 1000000
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'ScopeLib.ps1')

# ---------------------------------------------------------------------------------------------------------

Write-Host "Connecting to $Ip`:$Port ..."
$sc = New-Object SiglentScpi($Ip, $Port, 10000)
$origMode = $null
try {
    $idn = $sc.Query('*IDN?')
    $origMode = $sc.Query(':TRIG:MODE?')
    $wasRunning = $sc.Query(':TRIG:STAT?') -ne 'Stop'
    Write-Host "Connected: $idn" -ForegroundColor Green
    Write-Host "Channels on: $((Get-ActiveChannels $sc) -join ', ')"
    Write-Host ''

    $note = Read-Host 'Session note (what is being measured, e.g. "33 ohm / 0.33uF RC filter")'
    $started = Get-Date
    $safe = ($note -replace '[^\w\-. ]', '' -replace '\s+', '_').Trim('_')
    if ($safe.Length -gt 50) { $safe = $safe.Substring(0, 50) }
    $name = $started.ToString('yyyy-MM-dd_HHmmss') + $(if ($safe) { "_$safe" } else { '' }) + '.csv'
    New-Item -ItemType Directory -Force $OutDir | Out-Null
    $path = Join-Path $OutDir $name
    $session = @{ Idn = $idn; Note = $note; Started = $started.ToString('yyyy-MM-dd HH:mm:ss') }
    $conds = New-Object System.Collections.ArrayList

    while ($true) {
        Write-Host ''
        Write-Host "--- Condition $($conds.Count + 1) ---" -ForegroundColor Cyan
        $label = Read-Host 'Condition label (e.g. "button pressed"; Enter alone = finish)'
        if (-not $label) { break }
        $cnote = Read-Host 'Condition note (optional)'

        while ($true) {
            $mode = Read-Host 'Capture: [Enter] = grab what is on screen now, [S] = arm single trigger and wait, [X] = skip'
            if ($mode -match '^[xX]') { $cap = $null; break }
            if ($mode -match '^[sS]') {
                if (-not (Wait-SingleTrigger $sc)) { Write-Host '  Cancelled.'; continue }
            } else {
                $sc.Write(':TRIG:STOP'); Wait-Stopped $sc
            }

            $tdiv = [double]$sc.Query(':TIM:SCAL?')
            $trig = Get-TriggerDesc $sc
            $chs = @(Get-ActiveChannels $sc)
            if (-not $chs) { Write-Host '  No channels are on.' -ForegroundColor Red; continue }
            try { $waves = foreach ($ch in $chs) { Write-Host "  Reading $ch..."; Read-Channel $sc $ch $tdiv } }
            catch [InvalidOperationException] {
                Write-Host "  $($_.Exception.Message)" -ForegroundColor Red
                $sc.Write(":TRIG:MODE $origMode"); $sc.Write(':TRIG:RUN'); continue
            }
            foreach ($w in $waves) {
                Write-Host ("  {0}: {1} pts, min {2}, max {3}, mean {4}" -f $w.Channel, $w.Volts.Length,
                    (Num ([Linq.Enumerable]::Min($w.Volts)) 'V'), (Num ([Linq.Enumerable]::Max($w.Volts)) 'V'),
                    (Num ([Linq.Enumerable]::Average($w.Volts)) 'V'))
            }
            $cap = [pscustomobject]@{ Label = $label; Note = $cnote; Time = (Get-Date).ToString('HH:mm:ss'); TDiv = $tdiv; Trigger = $trig; Waves = @($waves) }

            $ans = Read-Host 'Keep this capture? [Y] = keep, [R] = retake, [D] = discard'
            if ($ans -match '^[rR]') { $sc.Write(":TRIG:MODE $origMode"); $sc.Write(':TRIG:RUN'); continue }
            if ($ans -match '^[dD]') { $cap = $null }
            break
        }

        # Let the scope run live again while the next condition is set up.
        $sc.Write(":TRIG:MODE $origMode"); if ($wasRunning) { $sc.Write(':TRIG:RUN') }

        if ($cap) {
            [void]$conds.Add($cap)
            Write-SessionCsv $path $session $conds
            Write-Host "  Saved ($($conds.Count) condition(s)) -> $path" -ForegroundColor Green
        }
    }

    Write-Host ''
    if ($conds.Count) { Write-Host "Done. $($conds.Count) condition(s) in $path" -ForegroundColor Green }
    else { Write-Host 'Nothing captured; no file written.' }
}
finally {
    if ($origMode) { try { $sc.Write(":TRIG:MODE $origMode"); if ($wasRunning) { $sc.Write(':TRIG:RUN') } } catch {} }
    $sc.Dispose()
}
