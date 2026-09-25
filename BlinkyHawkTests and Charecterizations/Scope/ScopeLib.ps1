# Shared Siglent helpers for Capture-Scope.ps1 and Sweep-Supply.ps1 (dot-source this).
# Read-Channel and Get-ActiveChannels read $MaxPoints and $Channels from the calling script.

Add-Type -Path (Join-Path $PSScriptRoot 'SiglentScpi.cs')
$inv = [Globalization.CultureInfo]::InvariantCulture

function Csv([string]$s) { if ($s -match '[",\r\n]') { '"' + $s.Replace('"', '""') + '"' } else { $s } }
function Num([double]$x, [string]$unit) { $x.ToString('G4', $inv) + $unit }

function Get-ActiveChannels($sc) {
    if ($Channels) { return $Channels | ForEach-Object { $_.ToUpper() } }
    1..4 | Where-Object { $sc.Query(":CHAN$($_):SWIT?") -eq 'ON' } | ForEach-Object { "C$_" }
}

function Wait-Stopped($sc, [int]$timeoutMs = 3000) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sc.Query(':TRIG:STAT?') -ne 'Stop') {
        if ($sw.ElapsedMilliseconds -gt $timeoutMs) { throw 'Scope did not stop.' }
        Start-Sleep -Milliseconds 50
    }
}

# Arm a single acquisition and wait for the trigger. Returns $false if the user cancelled with Q/Esc.
function Wait-SingleTrigger($sc) {
    $sc.Write(':TRIG:MODE SING')
    $null = $sc.Query('*OPC?')
    Write-Host '  Armed - waiting for trigger (Q or Esc to cancel)...' -ForegroundColor Yellow
    while ($sc.Query(':TRIG:STAT?') -ne 'Stop') {
        $key = $null
        try { if ([Console]::KeyAvailable) { $key = [Console]::ReadKey($true).Key } } catch {}  # no console (input redirected)
        if ($key -eq 'Escape' -or $key -eq 'Q') { $sc.Write(':TRIG:STOP'); return $false }
        Start-Sleep -Milliseconds 50
    }
    $true
}

function Read-Channel($sc, [string]$ch, [double]$tdiv) {
    $sc.Write(":WAV:SOUR $ch"); $sc.Write(':WAV:WIDT WORD'); $sc.Write(':WAV:INT 1')
    $sc.Write(':WAV:STAR 0'); $sc.Write(':WAV:POIN 0')
    $pre = $sc.QueryBlock(':WAV:PRE?')
    $count = [BitConverter]::ToInt32($pre, 0x74)
    if ($count -le 0) { throw [InvalidOperationException]::new("No waveform on $ch yet - let the scope trigger at least once, then capture again.") }
    $probe = [BitConverter]::ToSingle($pre, 0x148)
    $vdiv = [BitConverter]::ToSingle($pre, 0x9c) * $probe
    $offs = [BitConverter]::ToSingle($pre, 0xa0) * $probe
    $code = [BitConverter]::ToSingle($pre, 0xa4)
    # float32 in the descriptor; round-trip through text so 2E-06 stays 2E-06 instead of 1.99999995E-06
    $dt = [double]([BitConverter]::ToSingle($pre, 0xb0).ToString('R', $inv))
    $delay = [BitConverter]::ToDouble($pre, 0xb4)
    $maxp = [int][double]$sc.Query(':WAV:MAXP?')

    $chunks = New-Object 'System.Collections.Generic.List[byte[]]'
    $sparse = 1
    if ($MaxPoints -gt 0 -and $count -gt $MaxPoints) {
        $sparse = [int][Math]::Ceiling($count / $MaxPoints)
        $sc.Write(":WAV:INT $sparse")
        $chunks.Add($sc.QueryBlock(':WAV:DATA?'))
        $sc.Write(':WAV:INT 1')
    } else {
        for ($start = 0; $start -lt $count; $start += $maxp) {
            $sc.Write(":WAV:STAR $start"); $sc.Write(":WAV:POIN $([Math]::Min($maxp, $count - $start))")
            $chunks.Add($sc.QueryBlock(':WAV:DATA?'))
        }
        $sc.Write(':WAV:STAR 0'); $sc.Write(':WAV:POIN 0')
    }
    $volts = [SiglentScpi]::DecodeWord($chunks, $vdiv, $offs, $code)
    # The descriptor keeps the old point count, but the data is empty when the scope was stopped before it triggered.
    if ($volts.Length -eq 0) { throw [InvalidOperationException]::new("No waveform data on $ch - the scope did not trigger since it was last started.") }
    # Trigger is t = 0: the record spans 10 divisions, and the timebase delay shifts it right.
    $t0 = $delay - 5 * $tdiv
    [pscustomobject]@{
        Channel = $ch; Volts = $volts; T0 = $t0; Dt = $dt * $sparse; Sparse = $sparse
        VDiv = $vdiv; Offset = $offs; Probe = $probe
    }
}

function Write-SessionCsv($path, $session, $conds) {
    $L = New-Object 'System.Collections.Generic.List[string]'
    $L.Add('# Siglent capture,' + (Csv $session.Idn))
    $L.Add('# Session started,' + $session.Started)
    $L.Add('# Session note,' + (Csv $session.Note))
    $L.Add('# Time is seconds relative to the trigger point')
    $n = 0
    foreach ($c in $conds) {
        $n++
        $L.Add("# Condition $n," + (Csv $c.Label) + ',captured ' + $c.Time + ',' + (Csv $c.Note))
        $L.Add("#   timebase $(Num $c.TDiv 's')/div, sample interval $(Num $c.Waves[0].Dt 's'), $($c.Waves[0].Volts.Length) points" +
               $(if ($c.Waves[0].Sparse -gt 1) { " (every $($c.Waves[0].Sparse)th sample)" } else { '' }) +
               ", trigger $($c.Trigger)")
        foreach ($w in $c.Waves) {
            $L.Add("#   $($w.Channel): $(Num $w.VDiv 'V')/div, offset $(Num $w.Offset 'V'), probe $($w.Probe)x")
        }
    }

    # One shared time column when every capture has the same time axis, otherwise one per condition.
    $ref = $conds[0].Waves[0]
    $shared = -not ($conds | ForEach-Object { $_.Waves } | Where-Object {
        $_.Volts.Length -ne $ref.Volts.Length -or [Math]::Abs($_.T0 - $ref.T0) -gt 1e-12 -or [Math]::Abs($_.Dt - $ref.Dt) -gt 1e-15 })

    $hdr = New-Object 'System.Collections.Generic.List[string]'
    $cols = New-Object 'System.Collections.Generic.List[double[]]'
    $isTime = New-Object 'System.Collections.Generic.List[bool]'
    if ($shared) { $hdr.Add('Time (s)'); $cols.Add([SiglentScpi]::MakeTime($ref.T0, $ref.Dt, $ref.Volts.Length)); $isTime.Add($true) }
    foreach ($c in $conds) {
        if (-not $shared) {
            $w0 = $c.Waves[0]
            $hdr.Add((Csv "$($c.Label) Time (s)")); $cols.Add([SiglentScpi]::MakeTime($w0.T0, $w0.Dt, $w0.Volts.Length)); $isTime.Add($true)
        }
        foreach ($w in $c.Waves) { $hdr.Add((Csv "$($c.Label) $($w.Channel) (V)")); $cols.Add($w.Volts); $isTime.Add($false) }
    }
    $L.Add($hdr -join ',')

    [IO.File]::WriteAllLines($path, $L, (New-Object Text.UTF8Encoding($true)))
    [SiglentScpi]::AppendColumns($path, $cols.ToArray(), $isTime.ToArray())
}


function Get-TriggerDesc($sc) {
    $type = $sc.Query(':TRIG:TYPE?')
    if ($type -ne 'EDGE') { return $type }
    "$($sc.Query(':TRIG:EDGE:SOUR?')) $($sc.Query(':TRIG:EDGE:SLOP?')) at $(Num ([double]$sc.Query(':TRIG:EDGE:LEV?')) 'V')"
}
