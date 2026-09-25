"""Generate the editable AD8237 dev-board schematic, local libraries and BOM.

Run with Python 3. This file is the component/net source for build_pcb.py.
"""
from pathlib import Path
import json, uuid, csv, shutil

ROOT = Path(__file__).resolve().parents[1]
NAME = 'AD8237DevBoard'
NS = uuid.UUID('89d69edb-af28-46c2-b4c0-fb6a3720d6bc')
def uid(s): return str(uuid.uuid5(NS, s))
SHEET = uid('sheet')
KICAD = Path('C:/Program Files/KiCad/10.0/share/kicad')
RFP='Resistor_SMD:R_0805_2012Metric'
CFP='Capacitor_SMD:C_0805_2012Metric'
H2='Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical'
H3='Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical'
H6='Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical'
TP='TestPoint:TestPoint_Pad_D1.5mm'
HOLE='MountingHole:MountingHole_3.2mm_M3'
parts=[]
def add(ref,value,kind,fp,nets,sch,pcb,dnp=False,notes=''):
    parts.append(dict(ref=ref,value=value,kind=kind,source_fp=fp,fp='AD8237DevBoard:'+fp.split(':')[1],nets={str(k):v for k,v in enumerate(nets,1)},sch=sch,pcb=pcb,dnp=dnp,notes=notes,uuid=uid(ref)))

from revb_parts import populate
populate(add)
# Arrange all seven dividers together, retaining original references and UUIDs.
for upper,lower,x in [('R21','R22',238.76),('R23','R24',261.62),('R4','R3',284.48),('R25','R26',307.34),('R27','R28',330.2),('R6','R5',353.06),('R12','R11',375.92)]:
    for p in parts:
        if p['ref'] in (upper,lower):p['sch']=[x,190.5 if p['ref']==upper else 213.36,270]

def esc(x): return str(x).replace('\\','\\\\').replace('"','\\"').replace('\n','\\n')
def effects(size=1.27,extra=''): return f'(effects (font (size {size} {size})) {extra})'
def prop(k,v,x=0,y=0,hide=False,size=1.27,angle=0):
    return f'(property "{esc(k)}" "{esc(v)}" (at {x} {y} {angle}) {effects(size,"(hide yes)" if hide else "")})'
def pin(n,name,x,y,angle,typ='passive',length=2.54):
    return dict(n=str(n),name=name,x=x,y=y,a=angle,typ=typ,length=length)
pins={
 'R':[pin(1,'~',-5.08,0,0),pin(2,'~',5.08,0,180)],
 'C':[pin(1,'~',0,5.08,270,length=4.318),pin(2,'~',0,-5.08,90,length=4.318)],
 'AD8237':[pin(1,'BW',-15.24,-10.16,0,'input',5.08),pin(2,'+IN',-15.24,5.08,0,'input',5.08),pin(3,'-IN',-15.24,-5.08,0,'input',5.08),pin(4,'-VS',0,-20.32,90,'power_in',7.62),pin(5,'+VS',0,20.32,270,'power_in',7.62),pin(6,'REF',15.24,-7.62,180,'input',5.08),pin(7,'FB',15.24,0,180,'input',5.08),pin(8,'OUT',15.24,7.62,180,'output',5.08)],
 'TP':[pin(1,'~',0,0,90,length=2.54)],'Hole':[],
 'Flag':[pin(1,'pwr',0,0,90,'power_out',0)]}
for n in [2,3,4]: pins['Conn'+str(n)]=[pin(i,str(i),-7.62,(n-1)*2.54/2-(i-1)*2.54,0) for i in range(1,n+1)]
pins['Gain']=sum(([pin(2*i+1,'FB',-10.16,5.08-5.08*i,0),pin(2*i+2,['1','10.09','101'][i],10.16,5.08-5.08*i,180)] for i in range(3)),[])
pins['RefSelect']=sum(([pin(2*i+1,'REF',-10.16,5.08-5.08*i,0),pin(2*i+2,['GND','MID','EXT'][i],10.16,5.08-5.08*i,180)] for i in range(3)),[])
pins['TMUX1104']=[pin(1,'A0',-20.32,-12.7,0,'input',7.62),pin(2,'S1',-20.32,10.16,0,length=7.62),pin(3,'GND',0,-22.86,90,'power_in',5.08),pin(4,'S3',-20.32,0,0,length=7.62),pin(5,'EN',20.32,-12.7,180,'input',7.62),pin(6,'VDD',0,22.86,270,'power_in',5.08),pin(7,'S4',-20.32,-5.08,0,length=7.62),pin(8,'D',20.32,5.08,180,length=7.62),pin(9,'S2',-20.32,5.08,0,length=7.62),pin(10,'A1',-20.32,-17.78,0,'input',7.62)]
pins['TMUX1108']=[pin(1,'A0',20.32,-7.62,180,'input',7.62),pin(2,'EN',20.32,-22.86,180,'input',7.62),pin(3,'VSS',-5.08,-27.94,90,'power_in',5.08),pin(8,'D',20.32,10.16,180,length=7.62),pin(13,'VDD',0,25.4,270,'power_in',5.08),pin(14,'GND',5.08,-27.94,90,'power_in',5.08),pin(15,'A2',20.32,-17.78,180,'input',7.62),pin(16,'A1',20.32,-12.7,180,'input',7.62)]+[pin(n,'S'+str(i+1),-20.32,17.78-5.08*i,0,length=7.62) for i,n in enumerate([4,5,6,7,12,11,10,9])]
pins['TCA9536']=[pin(1,'P0',15.24,7.62,180,'bidirectional',5.08),pin(2,'P1',15.24,2.54,180,'bidirectional',5.08),pin(3,'P2',15.24,-2.54,180,'bidirectional',5.08),pin(4,'GND',0,-17.78,90,'power_in',5.08),pin(5,'P3/INT',15.24,-7.62,180,'bidirectional',5.08),pin(6,'SCL',-15.24,-5.08,0,'input',5.08),pin(7,'SDA',-15.24,5.08,0,'bidirectional',5.08),pin(8,'VCC',0,17.78,270,'power_in',5.08)]
pins['OPA333']=[pin(1,'OUT',15.24,0,180,'output',5.08),pin(2,'V-',0,-15.24,90,'power_in',5.08),pin(3,'+IN',-15.24,5.08,0,'input',5.08),pin(4,'-IN',-15.24,-5.08,0,'input',5.08),pin(5,'V+',0,15.24,270,'power_in',5.08)]
datasheets={'AD8237':'https://www.analog.com/media/en/technical-documentation/data-sheets/ad8237.pdf','TMUX1104':'https://www.ti.com/lit/ds/symlink/tmux1104.pdf','TCA9536':'https://www.ti.com/lit/ds/symlink/tca9536.pdf','OPA333':'https://www.ti.com/lit/ds/symlink/opa333.pdf'}
datasheets['TMUX1108']='https://www.ti.com/lit/ds/symlink/tmux1108.pdf'

def line(points,width=.254): return '(polyline (pts '+' '.join(f'(xy {x} {y})' for x,y in points)+f') (stroke (width {width}) (type default)) (fill (type none)))'
def rect(x1,y1,x2,y2):return f'(rectangle (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.254) (type default)) (fill (type background)))'
def symdef(kind,fullname):
    g=[]
    if kind=='R':g=[rect(-2.54,1.016,2.54,-1.016)]
    if kind=='C':g=[line([(-2.54,.762),(2.54,.762)]),line([(-2.54,-.762),(2.54,-.762)])]
    if kind in ['AD8237','TCA9536']:g=[rect(-10.16,12.7,10.16,-12.7)]
    if kind=='TMUX1104':g=[rect(-12.7,17.78,12.7,-17.78)]
    if kind=='TMUX1108':g=[rect(-12.7,20.32,12.7,-25.4)]
    if kind=='OPA333':g=[line([(-10.16,10.16),(-10.16,-10.16),(10.16,0),(-10.16,10.16)])]
    if kind.startswith('Conn'):g=[rect(-5.08,6.35,0,-6.35)]
    if kind in ['Gain','RefSelect']:g=[rect(-7.62,8.89,7.62,-8.89)]
    if kind in ['TP','Hole']:g=[f'(circle (center 0 {3.3 if kind=="TP" else 0}) (radius {0.76 if kind=="TP" else 2.54}) (stroke (width 0.254) (type default)) (fill (type none)))']
    if kind=='Flag':g=[line([(0,0),(0,2.54),(-1.27,2.54),(0,3.81),(1.27,2.54),(0,2.54)])]
    pstrings=[]
    for p in pins[kind]:pstrings.append(f'(pin {p["typ"]} line (at {p["x"]} {p["y"]} {p["a"]}) (length {p["length"]}) (name "{p["name"]}" {effects(1.016)}) (number "{p["n"]}" {effects(1.016)}))')
    hide_names=kind in ['R','C','TP','Hole','Flag']
    return f'(symbol "{fullname}" (pin_names (offset 0.508) {"(hide yes)" if hide_names else ""}) (exclude_from_sim no) (in_bom yes) (on_board yes) '+prop('Reference',{'R':'R','C':'C','AD8237':'U','Hole':'H','TP':'TP','Flag':'#FLG'}.get(kind,'J'),0,5.08)+prop('Value',kind,0,-5.08)+prop('Footprint','',hide=True)+prop('Datasheet','',hide=True)+f'(symbol "{kind}_0_1" '+''.join(g)+f') (symbol "{kind}_1_1" '+''.join(pstrings)+'))'

def generate():
    # Rotate vertical resistors so pin 1 is at the top of each divider.
    for p in parts:
        p['sch'][0]=round(round(p['sch'][0]/1.27)*1.27,5)
        p['sch'][1]=round(round(p['sch'][1]/1.27)*1.27,5)
        if p['kind']=='R' and p['sch'][2]==90:p['sch'][2]=270
    for d in ['AD8237DevBoard.pretty','docs','manufacturing','tmp']:(ROOT/d).mkdir(exist_ok=True)
    kinds=list(pins)
    symbols={k:symdef(k,'AD8237DevBoard:'+k) for k in kinds}
    (ROOT/(NAME+'.kicad_sym')).write_text('(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor") '+'\n'.join(symdef(k,k) for k in kinds)+')',encoding='utf-8')
    (ROOT/'sym-lib-table').write_text('(sym_lib_table (version 7) (lib (name "AD8237DevBoard") (type "KiCad") (uri "${KIPRJMOD}/AD8237DevBoard.kicad_sym") (options "") (descr "Project symbols; AD8237 datasheet Rev. 0")))\n')
    (ROOT/'fp-lib-table').write_text('(fp_lib_table (version 7) (lib (name "AD8237DevBoard") (type "KiCad") (uri "${KIPRJMOD}/AD8237DevBoard.pretty") (options "") (descr "Project-local copies of official KiCad footprints")))\n')
    for p in parts:
        lib,f=p['source_fp'].split(':')
        shutil.copyfile(KICAD/'footprints'/(lib+'.pretty')/(f+'.kicad_mod'),ROOT/'AD8237DevBoard.pretty'/(f+'.kicad_mod'))
    out=[f'(kicad_sch (version 20250114) (generator "eeschema") (uuid "{SHEET}") (paper "A3") (title_block (title "AD8237 Sensor Development Board") (date "2026-09-20") (rev "C") (company "Micro-DMM") (comment 1 "Eight I2C gains | Buffered midsupply | 3.3 / 5 V | 60 x 50 mm | Prototype")) (lib_symbols '+''.join(symbols.values())+')']
    def wire(a,b):out.append(f'(wire (pts (xy {a[0]:g} {a[1]:g}) (xy {b[0]:g} {b[1]:g})) (stroke (width 0) (type default)) (uuid "{uuid.uuid4()}"))')
    def label(net,xy,ang=0):out.append(f'(label "{net}" (at {xy[0]:g} {xy[1]:g} {ang}) {effects(1.016,"(justify left bottom)")} (uuid "{uuid.uuid4()}"))')
    def text(t,x,y,size=1.27):out.append(f'(text "{esc(t)}" (at {x} {y} 0) {effects(size,"(justify left top)")} (uuid "{uuid.uuid4()}"))')
    import math
    def coord(x,y,p,a):
        c=round(math.cos(math.radians(a)));s=round(math.sin(math.radians(a)))
        return x+p[0]*c-p[1]*s,y-p[0]*s-p[1]*c
    for p in parts:
        x,y,a=p['sch'];kind=p['kind']
        if kind in ['AD8237','TCA9536','OPA333']:rx,ry=x+12.7,y-19.05;vx,vy=x+12.7,y-16.51
        elif kind in ['TMUX1104','TMUX1108']:rx,ry=x+15.24,y-24.13;vx,vy=x+15.24,y-21.59
        elif kind in ['Gain','RefSelect']:rx,ry=x,y-12.7;vx,vy=x,y+12.7
        elif kind in ['C'] or (kind=='R' and a in [90,270]):rx,ry=x+8.89,y-1.905;vx,vy=x+8.89,y+.635
        elif kind=='TP':rx,ry=x,y-7.62;vx,vy=x,y+5.08
        elif kind.startswith('Conn'):rx,ry=x,y-10.16;vx,vy=x,y-7.62
        else:rx,ry=x,y-6.35;vx,vy=x,y-3.81
        out.append(f'(symbol (lib_id "AD8237DevBoard:{kind}") (at {x} {y} {a}) (unit 1) (in_bom yes) (on_board yes) (dnp {"yes" if p["dnp"] else "no"}) (uuid "{p["uuid"]}") '+prop('Reference',p['ref'],rx,ry,size=1.016,angle=90 if a in [90,270] else 0)+prop('Value',p['value'],vx,vy,hide=kind=='TP',size=1.016,angle=90 if a in [90,270] else 0)+prop('Footprint',p['fp'],x,y,True)+prop('Datasheet',datasheets.get(kind,''),x,y,True)+prop('Assembly',p['notes'],x,y,True)+''.join(f'(pin "{pin_["n"]}" (uuid "{uid(p["ref"]+"pin"+pin_["n"])}"))' for pin_ in pins[kind])+f'(instances (project "{NAME}" (path "/{SHEET}" (reference "{p["ref"]}") (unit 1)))))')
        for pp in pins[kind]:
            xy=coord(x,y,(pp['x'],pp['y']),a)
            if p['nets'][pp['n']] is None:
                out.append(f'(no_connect (at {xy[0]} {xy[1]}) (uuid \"{uuid.uuid4()}\"))')
                continue
            # Every pin has an explicit wired net label. Main input and gain branches are joined below.
            theta=math.radians(pp['a']+a)
            end=(round(xy[0]-6.35*math.cos(theta),5),round(xy[1]+6.35*math.sin(theta),5))
            wire(xy,end)
            if not (p['ref'] in ['R3','R5','R11','R16','R22','R24','R26','R28'] and pp['n']=='1'):label(p['nets'][pp['n']],end,0)
    # Actual wires in the central signal path and gain divider branches.
    for y in [121.92,132.08]:wire((90.17,y),(105.41,y))
    # Divider wire stubs meet exactly at 201.93 mm (and 219.71 mm for midpoint).
    for i,(net,x) in enumerate([('VS',63.5),('GND',88.9),('VIO',180.34)]):
        y=27.94;ref='#FLG0'+str(i+1)
        out.append(f'(symbol (lib_id "AD8237DevBoard:Flag") (at {x} {y} 0) (unit 1) (in_bom no) (on_board no) (dnp no) (uuid "{uid(ref)}") '+prop('Reference',ref,x,y,True)+prop('Value','PWR_FLAG',x,y-5.08,True)+f'(pin "1" (uuid "{uid(ref+"pin")}")) (instances (project "{NAME}" (path "/{SHEET}" (reference "{ref}") (unit 1)))))')
        wire((x,y),(x,y+2.54));label(net,(x,y+2.54),0)
    text('AD8237 | EIGHT I2C GAINS | REV C',20.32,15.24,2.286)
    text('ANALOG POWER: 3.3 V nominal / 5 V option',20.32,22.86)
    text('JP1 2-3: VIO (default); 1-2: external J1',91.44,68.58,1.016)
    text('I2C LOGIC DOMAIN - power VIO at the host I/O voltage',223.52,22.86)
    text('0x41 | P0=A0, P1=A1, P2=A2 | P3 unused',248.92,89.54,1.016)
    text('ANALOG SIGNAL PATH',20.32,93.98)
    text('HIGH-IMPEDANCE FEEDBACK MULTIPLEXER',248.92,95.25)
    text('JP2: use LOW for all eight gains and startup.',157.48,161.29,1.016)
    text('OPTIONAL INPUT FILTERS / BIAS RETURNS (DNP)',20.32,151.13)
    text('C7: Figure 76 HF compensation',160.02,192.4,1.016)
    text('GAIN DIVIDERS (0.1%)',246.38,172.72)
    text('BUFFERED HALF-SUPPLY REFERENCE',20.32,191.77)
    text('VMID = VS/2: 1.65 V at 3.3 V; 2.5 V at 5 V',81.28,201.93,1.016)
    text('JP3: ONE shunt\n1-2 GND; 3-4 MID (default); 5-6 EXT',170.18,239.39,1.016)
    text('I2C init: reg 0x01=0x00 BEFORE reg 0x03=0xF0; then reg 0x50=0x40.\nGain codes 0..7: 1 / 2.5 / 5.02 / 10.09 / 25.0481 / 49.7805 / 101 / 1001.\n4.7k pull-downs select unity before initialization. Unused P3 is output LOW.\nUse LOW bandwidth for gains 1, 2.5 and 5.02. See README for timing.',238.76,233.68,1.016)
    text('TEST POINTS',20.32,252.73)
    text('VOUT_RAW = VREF + G x (VIN+ - VIN-). REF source drives ALL divider returns.\nC3-C6 and R9/R10 are DNP. C7 is FITTED. Change supply/reference jumpers with power off.\nKeep sensor, reference and output voltages within analog supply rails. Host VIO and analog VS may differ.\nPower-up: unity, LOW bandwidth, MID reference. Wait 50 ms after power-up; discard samples after gain changes.',20.32,278.13,1.016)
    out.append('(embedded_fonts no))')
    (ROOT/(NAME+'.kicad_sch')).write_text('\n'.join(out),encoding='utf-8')
    project=json.loads((KICAD/'template/kicad.kicad_pro').read_text())
    project['meta']['filename']=NAME+'.kicad_pro'
    project['board']['design_settings']['rules']={'min_clearance':0.15,'min_track_width':0.2,'min_via_diameter':0.6,'min_through_hole_diameter':0.3,'min_hole_clearance':0.25,'min_hole_to_hole':0.25,'min_copper_edge_clearance':0.3,'min_silk_clearance':0.1,'min_text_height':0.8,'min_text_thickness':0.12}
    project['net_settings']={'meta':{'version':4},'classes':[{'name':'Default','clearance':0.2,'track_width':0.25,'via_diameter':0.6,'via_drill':0.3,'microvia_diameter':0.3,'microvia_drill':0.1,'diff_pair_width':0.2,'diff_pair_gap':0.25,'diff_pair_via_gap':0.25,'pcb_color':'rgba(0, 0, 0, 0.000)','schematic_color':'rgba(0, 0, 0, 0.000)','wire_width':6,'bus_width':12,'line_style':0}]}
    # Existing PCB design rules and UI settings belong to the edited project.
    if not (ROOT/(NAME+'.kicad_pro')).exists():
        (ROOT/(NAME+'.kicad_pro')).write_text(json.dumps(project,indent=2)+'\n')
    (ROOT/'scripts/components.json').write_text(json.dumps(parts,indent=2)+'\n')
    with (ROOT/'manufacturing/BOM.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['Reference','Value','Footprint','Fit','Specification / assembly note'])
        for p in parts:w.writerow([p['ref'],p['value'],p['fp'],'PCB feature' if p['kind'] in ['TP','Hole'] else ('DNP' if p['dnp'] else 'YES'),p['notes']])
        w.writerow(['SH1-SH4','2.54 mm jumper shunt','Accessory','YES','Four shunts: JP1 2-3 (VIO), JP2 1-2 (LOW), JP3 3-4 (MID), JP4 1-2 (pull-ups enabled)'])
    print('Created schematic, libraries, project and BOM:',len(parts),'components')

if __name__=='__main__':generate()
