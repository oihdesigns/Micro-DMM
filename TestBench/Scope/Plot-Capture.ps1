<#
.SYNOPSIS
  Turn a Capture-Scope CSV into a self-contained interactive plot (.html next to the CSV) and open it.

.EXAMPLE
  .\Plot-Capture.ps1                       # newest CSV in .\Captures
  .\Plot-Capture.ps1 .\Captures\foo.csv
  .\Plot-Capture.ps1 -All                  # (re)build a plot for every CSV in .\Captures
#>
param(
    [string[]]$CsvPath,
    [switch]$All,
    [switch]$NoOpen
)

$ErrorActionPreference = 'Stop'
$captures = Join-Path $PSScriptRoot 'Captures'
$lib = Join-Path $PSScriptRoot 'lib'

if ($All) { $CsvPath = Get-ChildItem $captures -Filter *.csv | ForEach-Object FullName; $NoOpen = $true }
elseif (-not $CsvPath) {
    $newest = Get-ChildItem $captures -Filter *.csv -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $newest) { throw "No CSV files in $captures" }
    $CsvPath = $newest.FullName
}

$template = [IO.File]::ReadAllText((Join-Path $lib 'viewer.html'))
$js = [IO.File]::ReadAllText((Join-Path $lib 'uPlot.iife.min.js'))
$css = [IO.File]::ReadAllText((Join-Path $lib 'uPlot.min.css'))
$page = $template.Replace('/*@@UPLOT_CSS@@*/', $css).Replace('/*@@UPLOT_JS@@*/', $js)

foreach ($p in $CsvPath) {
    $csv = (Resolve-Path $p).Path
    $name = [IO.Path]::GetFileName($csv)
    # Keep the CSV from closing the <script> block it is embedded in.
    $data = [IO.File]::ReadAllText($csv) -replace '(?i)</script', '<\/script'
    $html = $page.Replace('@@NAME@@', [Net.WebUtility]::HtmlEncode($name)).Replace('@@CSV@@', $data)
    $out = [IO.Path]::ChangeExtension($csv, '.html')
    [IO.File]::WriteAllText($out, $html, (New-Object Text.UTF8Encoding($false)))
    Write-Host "Plot written: $out"
    if (-not $NoOpen) { Start-Process $out }
}
