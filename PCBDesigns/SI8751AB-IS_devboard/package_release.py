"""Package the checked revision C CAD and prototype manufacturing outputs."""
from pathlib import Path
import json,csv,zipfile,hashlib,re
import pcbnew as k
ROOT=Path(__file__).resolve().parent
NAME='SI8751AB-IS_devboard'
b=k.LoadBoard(str(ROOT/(NAME+'.kicad_pcb')))
erc=(ROOT/'validation/revC_erc.rpt').read_text()
drc=(ROOT/'validation/revC_drc.rpt').read_text()
assert 'Errors 0  Warnings 0' in erc
assert all(s in drc for s in ['Found 0 DRC violations','Found 0 unconnected pads','Found 0 Footprint errors'])
parts=json.loads((ROOT/'circuit.json').read_text())
assert len(parts)==37
assert all(n!='+5V' for p in parts for n in p['nets'].values())
fps={f.GetReference():f for f in b.GetFootprints()}
assert fps['U6'].GetValue()=='ACS725LLCTR-05AB-T'
assert fps['U6'].FindPadByNumber('8').GetNetname()=='/+3V3'
assert fps['U6'].FindPadByNumber('7').GetNetname()=='/IOUT_ADC'
assert fps['J1'].FindPadByNumber('7').GetNetname().startswith('unconnected-')
assert not {'R15','R16'} & set(fps)
assert all('0805' in p['footprint'] for p in parts if p['reference'].startswith(('R','C')))
pth=[p for f in b.GetFootprints() for p in f.Pads() if p.GetAttribute()==k.PAD_ATTRIB_PTH]
assert all(p.GetLayerSet().Contains(k.F_Mask) and p.GetLayerSet().Contains(k.B_Mask) for p in pth)
summary={'revision':'C','date':'2026-09-05','fitted_components':len(parts),'all_resistors_capacitors_0805':True,'erc_errors':0,'erc_warnings':0,'drc_violations':0,'unconnected_items':0,'schematic_parity_issues':0,'pth_pads_with_both_mask_openings':len(pth),'actual_vias':sum(isinstance(t,k.PCB_VIA) for t in b.GetTracks()),'actual_track_segments':sum(not isinstance(t,k.PCB_VIA) for t in b.GetTracks()),'hardware_tested':False,'mains_certified':False}
(ROOT/'validation/revC_summary.json').write_text(json.dumps(summary,indent=2))
with (ROOT/'BOM_inventory.csv').open(encoding='utf-8-sig',newline='') as f: rows=list(csv.DictReader(f))
group={}
for r in rows:
    key=r['manufacturer_part_number'];g=group.setdefault(key,dict(manufacturer_part_number=key,quantity=0,references=[],stocked_alternative=r['stocked_alternative'],inventory_match=r['inventory_match'],historical_quantity=r['historical_quantity']))
    g['quantity']+=1;g['references'].append(r['reference'])
with (ROOT/'BOM_grouped.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(next(iter(group.values()))));writer.writeheader()
    for g in group.values():g['references']=', '.join(g['references']);writer.writerow(g)
fab=ROOT/'fabrication/revC'
assert all(any(fab.glob('*'+ext)) for ext in ['.gtl','.gbl','.gts','.gbs','.gto','.gm1','.drl'])
(fab/'FABRICATION_NOTES.txt').write_text('REVISION C — BENCH PROTOTYPE\n110 x 90 mm; 2 copper layers, 35 um (1 oz) copper, 1.6 mm FR-4.\nGerber X2; Excellon metric absolute coordinates; separate PTH/NPTH drill files.\nNo internal milled slot. Four 3.2 mm mounting holes.\nAll resistors/capacitors 0805. All fitted components on the top side.\nConfirm TO-220 lead fit in 1.1 mm drills and terminal-header fit in 1.6 mm drills.\nSolder-mask openings on both sides of every plated component pad.\nNo hardware, surge, thermal, EMC or mains-safety qualification has been completed.\nSee project README and BOM for assembly, supply and load restrictions.\n',encoding='utf-8')
with zipfile.ZipFile(ROOT/(NAME+'_revC_fabrication.zip'),'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(fab.iterdir()):
        if p.is_file():z.write(p,p.name)
files=[ROOT/(NAME+ext) for ext in ['.kicad_pro','.kicad_sch','.kicad_pcb','.kicad_dru','.pdf']]
files += [ROOT/p for p in ['README.md','BOM.csv','BOM_inventory.csv','BOM_grouped.csv','circuit.json','Devboard.kicad_sym','sym-lib-table','fp-lib-table','build_design.py','build_pcb.py','make_rules.py','match_bom.py','package_release.py']]
files += [ROOT/'Devboard.pretty'/(name+'.kicad_mod') for name in sorted({p['footprint'].split(':')[-1] for p in parts})]
files += [p for p in (ROOT/'validation').glob('revC_*') if p.name!='revC_manifest.json']+list(fab.iterdir())
manifest={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}
(ROOT/'validation/revC_manifest.json').write_text(json.dumps(manifest,indent=2))
with zipfile.ZipFile(ROOT/(NAME+'_revC_project.zip'),'w',zipfile.ZIP_DEFLATED) as z:
    for p in files:
        if p.is_file():z.write(p,p.relative_to(ROOT))
    if ROOT/'validation/revC_manifest.json' not in files:z.write(ROOT/'validation/revC_manifest.json','validation/revC_manifest.json')
print(json.dumps(summary,indent=2))
