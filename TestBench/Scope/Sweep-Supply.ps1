<#
.SYNOPSIS
  Step a Nordic PPK2 (source mode) through a list of supply voltages and capture every displayed Siglent
  channel at each step. One condition per voltage, same CSV format as Capture-Scope.ps1, then plots it.

.EXAMPLE
  .\Sweep-Supply.ps1 -StartMv 3000 -StopMv 4200 -StepMv 100 -Note "VBatt sweep, button open"
  .\Sweep-Supply.ps1 -Voltages 3200,3600,4100 -Grab          # no trigger wait, grab the screen at each step
  .\Sweep-Supply.ps1 -StartMv 4200 -StopMv 3000 -StepMv 50   # descending works too
#>
param(
    [int]$StartMv,
    [int]$StopMv,
    [int]$StepMv = 100,
    # Explicit list instead of Start/Stop/Step.
    [int[]]$Voltages,
    # Wait after each voltage change before capturing.
    [int]$SettleMs = 1000,
    # Default: arm a single trigger at each step. -Grab: stop and take what is on screen instead.
    [switch]$Grab,
    [double]$TriggerTimeoutS = 5,
    [string]$Note,
    # Default: DUT power is switched off after the sweep.
    [switch]$LeaveOn,
    # Default: find the PPK2 by USB ID.
    [string]$PpkPort,
    [string]$Ip = '192.168.10.2',
    [int]$Port = 5025,
    [string]$OutDir = (Join-Path $PSScriptRoot 'Captures'),
    [string[]]$Channels,
    [int]$MaxPoints = 1000000
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'ScopeLib.ps1')

# --- PPK2 (USB CDC, single-byte opcodes; see Nordic pc-nrfconnect-ppk) -------------------------------------

function Read-PpkMeta($sp) {
    $sp.DiscardInBuffer()
    $sp.Write([byte[]](0x19), 0, 1)                            # GET_META_DATA
    $buf = ''; $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($buf -notmatch 'END' -and $sw.ElapsedMilliseconds -lt 2000) {
        Start-Sleep -Milliseconds 50; $buf += $sp.ReadExisting()
    }
    if ($buf -notmatch 'END') { return $null }
    $meta = @{}
    foreach ($line in $buf -split '\r?\n') { if ($line -match '^\s*([^:]+):\s*(.+?)\s*$') { $meta[$Matches[1]] = $Matches[2] } }
    $meta
}

function Open-Ppk2([string]$portName) {
    $candidates = if ($portName) { @($portName) } else {
        Get-CimInstance Win32_PnPEntity -Filter "PNPDeviceID LIKE 'USB\\VID_1915&PID_C00A%'" |
            Where-Object { $_.Name -match '\((COM\d+)\)' } | ForEach-Object { $Matches[1] }
    }
    if (-not $candidates) { throw 'No PPK2 found (USB VID 1915 / PID C00A). Is it plugged in?' }
    $errors = @()
    foreach ($c in $candidates) {
        $sp = New-Object IO.Ports.SerialPort($c, 115200)
        $sp.DtrEnable = $true; $sp.ReadTimeout = 500; $sp.WriteTimeout = 500
        try { $sp.Open() } catch { $errors += "${c}: in use by another program (close the Power Profiler app?)"; continue }
        $meta = Read-PpkMeta $sp
        if ($meta) { return [pscustomobject]@{ Port = $sp; Name = $c; Meta = $meta } }
        $sp.Close(); $errors += "${c}: no metadata response"
    }
    throw "Could not talk to the PPK2:`n  " + ($errors -join "`n  ")
}

function Send-Ppk($ppk, [byte[]]$bytes) { $ppk.Port.Write($bytes, 0, $bytes.Length) }
function Set-PpkSourceMode($ppk) { Send-Ppk $ppk @(0x11, 2) }             # SET_POWER_MODE: 1 = ampere, 2 = source
function Set-PpkVoltage($ppk, [int]$mv) { Send-Ppk $ppk @(0x0D, ($mv -shr 8), ($mv -band 0xFF)) }   # REGULATOR_SET
function Set-PpkDutPower($ppk, [bool]$on) { Send-Ppk $ppk @(0x0C, [byte]$on) }                     # DEVICE_RUNNING_SET

# --- scope ---------------------------------------------------------------------------------------------------

# Arm a single acquisition. $false on timeout (no trigger); throws if the user presses Q/Esc.
function Wait-TriggerOrTimeout($sc, [double]$timeoutS) {
    $sc.Write(':TRIG:MODE SING')
    $null = $sc.Query('*OPC?')
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sc.Query(':TRIG:STAT?') -ne 'Stop') {
        $key = $null
        try { if ([Console]::KeyAvailable) { $key = [Console]::ReadKey($true).Key } } catch {}
        if ($key -eq 'Escape' -or $key -eq 'Q') { $sc.Write(':TRIG:STOP'); throw 'Sweep cancelled.' }
        if ($sw.Elapsed.TotalSeconds -gt $timeoutS) { $sc.Write(':TRIG:STOP'); return $false }
        Start-Sleep -Milliseconds 50
    }
    $true
}

# ---------------------------------------------------------------------------------------------------------

function Read-Default([string]$prompt, $default) { $a = Read-Host "$prompt [$default]"; if ($a) { $a } else { $default } }

# Started with no sweep given (e.g. double-clicked Sweep-Supply.cmd): ask for the settings.
if (-not $Voltages -and -not $PSBoundParameters.ContainsKey('StartMv') -and -not $PSBoundParameters.ContainsKey('StopMv')) {
    $StartMv = [int](Read-Default 'Start voltage (mV)' 3000)
    $StopMv = [int](Read-Default 'Stop voltage (mV)' 4200)
    $StepMv = [int](Read-Default 'Step (mV)' $StepMv)
    $SettleMs = [int](Read-Default 'Settle time after each step (ms)' $SettleMs)
    $Grab = (Read-Default 'Capture: [T] = wait for trigger, [G] = grab screen' 'T') -match '^[gG]'
    $LeaveOn = (Read-Default 'After the sweep: [O] = DUT power off, [L] = leave on' 'O') -match '^[lL]'
    if (-not $Note) { $Note = Read-Host 'Sweep note (optional)' }
}

if (-not $Voltages) {
    if (-not $StartMv -or -not $StopMv) {
        throw 'Give -StartMv and -StopMv (and optionally -StepMv), or -Voltages.'
    }
    if ($StepMv -le 0) { throw '-StepMv must be positive.' }
    $dir = if ($StopMv -ge $StartMv) { 1 } else { -1 }
    $Voltages = for ($v = $StartMv; ($StopMv - $v) * $dir -ge 0; $v += $dir * $StepMv) { $v }
}
$bad = $Voltages | Where-Object { $_ -lt 800 -or $_ -gt 5000 }
if ($bad) { throw "PPK2 source range is 800-5000 mV; out of range: $($bad -join ', ')" }

$ppk = Open-Ppk2 $PpkPort
Write-Host "PPK2 on $($ppk.Name): HW $($ppk.Meta['HW']), currently $($ppk.Meta['VDD']) mV" -ForegroundColor Green

Write-Host "Connecting to scope $Ip`:$Port ..."
$sc = New-Object SiglentScpi($Ip, $Port, 10000)
$origMode = $null
try {
    $idn = $sc.Query('*IDN?')
    $origMode = $sc.Query(':TRIG:MODE?')
    $wasRunning = $sc.Query(':TRIG:STAT?') -ne 'Stop'
    Write-Host "Connected: $idn" -ForegroundColor Green
    Write-Host "Channels on: $((Get-ActiveChannels $sc) -join ', ')"
    Write-Host "Sweep: $($Voltages -join ', ') mV; settle $SettleMs ms; $(if ($Grab) { 'grab screen' } else { "single trigger ($(Get-TriggerDesc $sc)), ${TriggerTimeoutS}s timeout" })"
    Write-Host 'Press Q or Esc while waiting for a trigger to stop the sweep.'

    $started = Get-Date
    $sweepDesc = "PPK2 source-mode sweep $($Voltages[0])-$($Voltages[-1]) mV ($($Voltages.Count) steps), settle $SettleMs ms"
    $safe = ($Note -replace '[^\w\-. ]', '' -replace '\s+', '_').Trim('_')
    if ($safe.Length -gt 40) { $safe = $safe.Substring(0, 40) }
    $name = $started.ToString('yyyy-MM-dd_HHmmss') + '_SupplySweep' + $(if ($safe) { "_$safe" } else { '' }) + '.csv'
    New-Item -ItemType Directory -Force $OutDir | Out-Null
    $path = Join-Path $OutDir $name
    $baseNote = if ($Note) { "$Note | $sweepDesc" } else { $sweepDesc }
    $session = @{ Idn = $idn; Note = $baseNote; Started = $started.ToString('yyyy-MM-dd HH:mm:ss') }
    $conds = New-Object System.Collections.ArrayList
    $missed = @()

    Set-PpkSourceMode $ppk
    Set-PpkVoltage $ppk $Voltages[0]
    Set-PpkDutPower $ppk $true

    $prev = $null
    foreach ($mv in $Voltages) {
        Write-Host ''
        Write-Host "--- $mv mV ---" -ForegroundColor Cyan
        Set-PpkVoltage $ppk $mv
        # Acquire live while the supply settles, so a grab shows this voltage and not the previous stopped frame.
        if ($Grab) { $sc.Write(":TRIG:MODE $origMode"); $sc.Write(':TRIG:RUN') }
        Start-Sleep -Milliseconds $SettleMs

        if ($Grab) { $sc.Write(':TRIG:STOP'); Wait-Stopped $sc }
        elseif (-not (Wait-TriggerOrTimeout $sc $TriggerTimeoutS)) {
            Write-Host "  No trigger within $TriggerTimeoutS s - skipped." -ForegroundColor Red
            $missed += $mv
            continue
        }

        $tdiv = [double]$sc.Query(':TIM:SCAL?')
        $chs = @(Get-ActiveChannels $sc)
        if (-not $chs) { throw 'No scope channels are on.' }
        try { $waves = foreach ($ch in $chs) { Read-Channel $sc $ch $tdiv } }
        catch [InvalidOperationException] {
            Write-Host "  $($_.Exception.Message) Skipped." -ForegroundColor Red
            $missed += $mv
            continue
        }
        foreach ($w in $waves) {
            Write-Host ("  {0}: min {1}, max {2}, mean {3}" -f $w.Channel,
                (Num ([Linq.Enumerable]::Min($w.Volts)) 'V'), (Num ([Linq.Enumerable]::Max($w.Volts)) 'V'),
                (Num ([Linq.Enumerable]::Average($w.Volts)) 'V'))
        }
        # Live data never repeats sample for sample; an exact repeat means the scope did not acquire (no trigger in Normal mode).
        $stale = $prev -and $prev.Count -eq $waves.Count -and -not (0..($waves.Count - 1) | Where-Object {
            -not [Collections.StructuralComparisons]::StructuralEqualityComparer.Equals($prev[$_].Volts, $waves[$_].Volts) })
        if ($stale) { Write-Host '  WARNING: identical to the previous step - the scope did not acquire new data.' -ForegroundColor Red }
        $prev = @($waves)
        [void]$conds.Add([pscustomobject]@{
            Label = "${mv}mV"; Note = "PPK2 supply $mv mV" + $(if ($stale) { ' (STALE: same data as previous step)' } else { '' }); Time = (Get-Date).ToString('HH:mm:ss')
            TDiv = $tdiv; Trigger = $(if ($Grab) { 'screen grab' } else { Get-TriggerDesc $sc }); Waves = @($waves)
        })
        $session.Note = $baseNote + $(if ($missed) { " | no trigger at: $($missed -join ', ') mV" } else { '' })
        Write-SessionCsv $path $session $conds
    }

    Write-Host ''
    if ($missed) { Write-Host "No trigger at: $($missed -join ', ') mV" -ForegroundColor Yellow }
    if ($conds.Count) {
        # Record misses that came after the last saved step too.
        $session.Note = $baseNote + $(if ($missed) { " | no trigger at: $($missed -join ', ') mV" } else { '' })
        Write-SessionCsv $path $session $conds
        Write-Host "Done. $($conds.Count) step(s) in $path" -ForegroundColor Green
        & (Join-Path $PSScriptRoot 'Plot-Capture.ps1') $path
    } else { Write-Host 'Nothing captured; no file written.' }
}
finally {
    if ($origMode) { try { $sc.Write(":TRIG:MODE $origMode"); if ($wasRunning) { $sc.Write(':TRIG:RUN') } } catch {} }
    $sc.Dispose()
    if (-not $LeaveOn) { try { Set-PpkDutPower $ppk $false; Write-Host 'PPK2 DUT power off.' } catch {} }
    $ppk.Port.Close()
}
