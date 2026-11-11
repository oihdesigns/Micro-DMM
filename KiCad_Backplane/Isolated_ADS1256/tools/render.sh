#!/bin/sh
# render.sh <out.png> [layers] [dpi]
K="/c/Users/Nick/AppData/Local/Programs/Backplane-KiCad/10.99.0-bp.6/bin/kicad-cli.exe"
L=${2:-F.Cu,F.SilkS,F.CrtYd,Edge.Cuts,F.Fab}
"$K" pcb export svg --mode-single --page-size-mode 2 --exclude-drawing-sheet -l "$L" -o build/pcb_render.svg Isolated_ADS1256.kicad_pcb 2>&1 | grep -v -i "fontconfig\|^Plot\|Done"
py -c "
import fitz,sys
d=fitz.open('build/pcb_render.svg'); p=d[0]
p.get_pixmap(dpi=int(sys.argv[2])).save(sys.argv[1]); print(p.rect)" "$1" "${3:-150}"
