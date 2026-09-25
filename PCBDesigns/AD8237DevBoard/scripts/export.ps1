$ErrorActionPreference = 'Stop'
$designRoot = Split-Path $PSScriptRoot -Parent
Set-Location -LiteralPath $designRoot
$cli = 'C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'
$py = 'C:/Program Files/KiCad/10.0/bin/python.exe'
$env:KICAD_CONFIG_HOME = Join-Path $designRoot 'tmp/kicad-config'
$env:KICAD10_3DMODEL_DIR = 'C:/Program Files/KiCad/10.0/share/kicad/3dmodels'
function Invoke-KiCad {
    & $cli @args
    if ($LASTEXITCODE -ne 0) { throw "KiCad export/check failed: $args" }
}
New-Item -ItemType Directory -Force -Path 'docs','tmp','manufacturing/gerbers','manufacturing/stencil' | Out-Null
Invoke-KiCad sch erc --exit-code-violations --format json --output docs/ERC.json AD8237DevBoard.kicad_sch
Invoke-KiCad pcb drc --refill-zones --save-board --schematic-parity --exit-code-violations --format json --output docs/DRC.json AD8237DevBoard.kicad_pcb
Invoke-KiCad sch export netlist --format kicadxml --output docs/netlist.xml AD8237DevBoard.kicad_sch
& $py scripts/verify_design.py
if ($LASTEXITCODE -ne 0) { throw 'Circuit verification failed' }
Invoke-KiCad sch export pdf --output docs/AD8237DevBoard-schematic.pdf AD8237DevBoard.kicad_sch
Invoke-KiCad sch export svg --output docs AD8237DevBoard.kicad_sch
Invoke-KiCad pcb export gerbers --layers 'F.Cu,B.Cu,F.Mask,B.Mask,F.SilkS,B.SilkS,Edge.Cuts' --subtract-soldermask --output manufacturing/gerbers/ AD8237DevBoard.kicad_pcb
Invoke-KiCad pcb export drill --format excellon --excellon-units mm --excellon-separate-th --generate-report --report-path manufacturing/drill-report.txt --output manufacturing/gerbers/ AD8237DevBoard.kicad_pcb
Invoke-KiCad pcb export pos --side front --format csv --units mm --smd-only --exclude-dnp --output manufacturing/placement.csv AD8237DevBoard.kicad_pcb
# Export a default-population stencil from a temporary copy with DNP parts removed.
& $py -c "import pcbnew as k; b=k.LoadBoard('AD8237DevBoard.kicad_pcb'); [b.Remove(f) for f in list(b.GetFootprints()) if f.GetAttributes() & k.FP_DNP]; k.SaveBoard('tmp/AD8237DevBoard-stencil.kicad_pcb',b)"
if ($LASTEXITCODE -ne 0) { throw 'Stencil preparation failed' }
Invoke-KiCad pcb export gerbers --layers F.Paste --output manufacturing/stencil/ tmp/AD8237DevBoard-stencil.kicad_pcb
Invoke-KiCad pcb export svg --layers 'F.Cu,F.SilkS,Edge.Cuts' --page-size-mode 2 --mode-single --output docs/PCB-routing-top.svg AD8237DevBoard.kicad_pcb
Invoke-KiCad pcb export svg --layers 'B.Cu,B.SilkS,Edge.Cuts' --page-size-mode 2 --mode-single --output docs/PCB-routing-bottom.svg AD8237DevBoard.kicad_pcb
Invoke-KiCad pcb export svg --layers 'F.Fab,Edge.Cuts' --crossout-DNP-footprints-on-fab-layers --page-size-mode 2 --mode-single --output docs/PCB-assembly.svg AD8237DevBoard.kicad_pcb
Invoke-KiCad pcb render --output docs/PCB-top.png --width 1800 --height 1440 --side top --background opaque --quality high AD8237DevBoard.kicad_pcb
Invoke-KiCad pcb render --output docs/PCB-bottom.png --width 1800 --height 1440 --side bottom --background opaque --quality high AD8237DevBoard.kicad_pcb
Compress-Archive -Path 'manufacturing/gerbers/*' -DestinationPath 'manufacturing/AD8237DevBoard-RevC-Gerbers.zip' -Force
Write-Output 'Checks and fabrication exports complete.'
