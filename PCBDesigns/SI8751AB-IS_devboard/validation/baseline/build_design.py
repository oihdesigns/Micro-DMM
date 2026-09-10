"""Generate revision A of the 120Vac / 170Vdc, <1A bench development circuit."""
from pathlib import Path
import re, json, uuid, csv

ROOT = Path(__file__).resolve().parent
LIB = Path('C:/Program Files/KiCad/10.0/share/kicad/symbols')
NAME = 'SI8751AB-IS_devboard'
def uid(key): return str(uuid.uuid5(uuid.NAMESPACE_URL, NAME+'/'+key))
def q(s): return json.dumps(str(s))
def fx(size=1.27, extra=''): return f'(effects (font (size {size} {size})) {extra})'
symbols, pins, parts, drawing = {}, {}, [], []

def extract(lib, name):
    data=(LIB/(lib+'.kicad_sym')).read_text(encoding='utf-8')
    start=data.index('(symbol '+q(name)+'\n')
    depth=0; quoted=False; escape=False
    for i in range(start,len(data)):
        c=data[i]
        if quoted:
            if escape: escape=False
            elif c=='\\': escape=True
            elif c=='"': quoted=False
        elif c=='"': quoted=True
        elif c=='(': depth+=1
        elif c==')':
            depth-=1
            if not depth: return data[start:i+1]

def block(name, left, right, width=12.7):
    entries=[]; p={}
    for side, seq in [(-1,left),(1,right)]:
        for i,(num,label,typ) in enumerate(seq):
            y=7.62-i*5.08
            x=side*(width+5.08); p[str(num)]=(x,y,0 if side<0 else 180)
            entries.append(f'(pin {typ} line (at {x} {y} {p[str(num)][2]}) (length 5.08) (name {q(label)} {fx()}) (number {q(num)} {fx()}))')
    low=min(x[1] for x in p.values())-5.08
    symbols[name]=f'''(symbol {q(name)} (pin_names (offset 1.016)) (in_bom yes) (on_board yes)
    (property "Reference" "U" (at 0 15.24 0) {fx()})
    (property "Value" {q(name)} (at 0 12.7 0) {fx()})
    (symbol {q(name+'_0_1')} (rectangle (start {-width} 10.16) (end {width} {low}) (stroke (width 0.254) (type default)) (fill (type background))))
    (symbol {q(name+'_1_1')} {''.join(entries)}))'''
    pins[name]=p

block('SI8751AB-IS',[(1,'VDD','power_in'),(2,'TT','passive'),(3,'IN','input'),(4,'GND','power_in')],[(8,'GATE','output'),(7,'MCAP1','input'),(6,'MCAP2','input'),(5,'SOURCE','passive')])
block('AMC0330RDWVR',[(1,'VDD1','power_in'),(2,'INP','input'),(3,'SNSN','input'),(4,'GND1','power_in')],[(8,'VDD2','power_in'),(7,'OUT','output'),(6,'REFIN','input'),(5,'GND2','power_in')])
block('SCT01F03S05',[(2,'+VIN','power_in'),(1,'-VIN','power_in')],[(4,'+VOUT','power_out'),(3,'-VOUT','power_out')])
block('TPS70933DBVR',[(1,'IN','power_in'),(3,'EN','input'),(2,'GND','power_in')],[(5,'OUT','power_out'),(4,'NC','no_connect')])
for lib,name in [('Device','R'),('Device','C'),('Transistor_FET','Q_NMOS_GDS'),('Connector_Generic','Conn_01x02'),('Connector_Generic','Conn_01x05'),('power','PWR_FLAG')]:
    symbols[name]=extract(lib,name)
    # Parse pin geometry from the imported, non-inherited symbol.
    found=re.findall(r'\(pin\s+\w+\s+\w+\s+\(at\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\).*?\(number\s+"([^"]+)"',symbols[name],re.S)
    pins[name]={n:(float(x),float(y),float(a)) for x,y,a,n in found}

def wire(a,b):
    drawing.append(f'(wire (pts (xy {a[0]} {a[1]}) (xy {b[0]} {b[1]})) (stroke (width 0) (type default)) (uuid {q(uid("wire"+str(a)+str(b)))}))')
def label(net,x,y,angle=0):
    just='(justify right bottom)' if angle==180 else '(justify left bottom)'
    drawing.append(f'(label {q(net)} (at {x} {y} 0) {fx(1.016,just)} (uuid {q(uid("label"+net+str(x)+str(y)))}))')
def note(text,x,y,size=1.524):
    drawing.append(f'(text {q(text)} (at {x} {y} 0) {fx(size,"(justify left top)")} (uuid {q(uid("text"+text))}))')

def part(ref,typ,value,x,y,nets,footprint='',datasheet='',purpose='',dnp=False):
    x=round(round(x/1.27)*1.27,4); y=round(round(y/1.27)*1.27,4)
    source_footprint=footprint
    if footprint: footprint='Devboard:'+footprint.split(':')[-1]
    sid=uid(ref); text=[]
    is_passive=typ in ['R','C']
    px=x+3.81 if is_passive else x
    py=y-1.27 if is_passive else y-15.24
    just='(justify left)' if is_passive else ''
    for k,v,yy,hide in [('Reference',ref,py,False),('Value',value,py+2.54,False),('Footprint',footprint,y,True),('Datasheet',datasheet,y,True),('Purpose',purpose,y,True)]:
        text.append(f'(property {q(k)} {q(v)} (at {px} {yy} 0) {fx(1.27,just+(" (hide yes)" if hide else ""))})')
    for n in pins[typ]: text.append(f'(pin {q(n)} (uuid {q(uid(ref+"pin"+n))}))')
    drawing.append(f'''(symbol (lib_id {q('Devboard:'+typ)}) (at {x} {y} 0) (unit 1) (in_bom {'no' if typ=='PWR_FLAG' else 'yes'}) (on_board {'no' if typ=='PWR_FLAG' else 'yes'}) (dnp {'yes' if dnp else 'no'}) (uuid {q(sid)})
    {''.join(text)} (instances (project {q(NAME)} (path {q('/'+uid('root'))} (reference {q(ref)}) (unit 1)))))''')
    for n,(dx,dy,angle) in pins[typ].items():
        a=(round(x+dx,4),round(y-dy,4)); net=nets.get(n)
        if net is None:
            drawing.append(f'(no_connect (at {a[0]} {a[1]}) (uuid {q(uid(ref+"NC"+n))}))'); continue
        if angle==0: b=(a[0]-5.08,a[1])
        elif angle==180: b=(a[0]+5.08,a[1])
        elif angle==90: b=(a[0],a[1]+5.08)
        else: b=(a[0],a[1]-5.08)
        wire(a,b); label(net,*b,180 if angle==0 else 0)
    if typ!='PWR_FLAG': parts.append(dict(reference=ref,symbol=typ,value=value,footprint=footprint,source_footprint=source_footprint,datasheet=datasheet,purpose=purpose,dnp=dnp,nets=nets,uuid=sid))

RFP='Resistor_SMD:R_0805_2012Metric'
CFP='Capacitor_SMD:C_0805_2012Metric'
def rc(ref,val,x,y,n1,n2,purpose='',dnp=False):
    typ=ref[0]; part(ref,typ,val,x,y,{'1':n1,'2':n2},RFP if typ=='R' else CFP,purpose=purpose,dnp=dnp)
si='https://www.skyworksinc.com/-/media/SkyWorks/SL/documents/public/data-sheets/Si8751-2.pdf'
amc='https://www.ti.com/lit/ds/symlink/amc0330r.pdf'
xp='https://www.xppower.com/storage/portals/0/pdfs/SF_SCT01F.pdf'

note('AC / DC SOLID-STATE SWITCH + ISOLATED VOLTAGE SENSE',15,15,2.54)
note('REV A - 120 Vac / 170 Vdc, <1 A - RESISTIVE BENCH LOADS - EXTERNAL FUSE REQUIRED',15,23,1.524)
note('1. LOGIC CONTROL - 3.3 V supply, active-high EN, both drivers always commanded together',15,30)
part('J1','Conn_01x05','CONTROL / ADC',35,60,{'1':'+3V3','2':'GND_LOGIC','3':'EN','4':'VOUT_ADC','5':'GND_LOGIC'},'Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical')
rc('R1','100R',65,60,'EN','EN_A','Series input damping')
rc('R2','100k',65,90,'EN_A','GND_LOGIC','Default OFF')
rc('R3','100R',65,125,'EN','EN_B','Series input damping')
rc('R4','100k',65,155,'EN_B','GND_LOGIC','Default OFF')
for i,y in [(1,60),(2,125)]:
    part(f'U{i}','SI8751AB-IS','SI8751AB-IS',125,y,{'1':'+3V3','2':f'TT_{i}','3':'EN_A' if i==1 else 'EN_B','4':'GND_LOGIC','5':'SOURCE_FLOAT','6':None,'7':f'MCAP_{i}','8':f'GATE_{i}'},'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',si,'One isolated gate driver per MOSFET')
    rc(f'C{i}','100n / 16V',95,y+27.94,'+3V3','GND_LOGIC','Place beside driver VDD / GND')
    rc(f'R{i+4}','10k',125,y+27.94,f'TT_{i}','GND_LOGIC','TT=10k: 9V minimum unloaded gate output; qualify actual VGS under load')
    rc(f'C{i+2}','10p / 1kV',175,y-2.54,'RAIL_IN' if i==1 else 'RAIL_OUT',f'MCAP_{i}','0805 C0G 1kV; qualify dV/dt <=600V/us for 6mA MCAP limit')
    part(f'Q{i}','Q_NMOS_GDS','STP12N60M2',235,y,{'1':f'GATE_{i}','2':'RAIL_IN' if i==1 else 'RAIL_OUT','3':'SOURCE_FLOAT'},'Package_TO_SOT_THT:TO-220-3_Vertical','https://www.st.com/resource/en/datasheet/stp12n60m2.pdf','600V NMOS; RDSon specified at 10V; qualify VGS and dissipation on prototype')
note('2. BIDIRECTIONAL POWER PATH',265,30)
note('Q1/Q2 sources share SOURCE_FLOAT.\nOpposed body diodes block either polarity when OFF.\nGates are separate; driver outputs are not paralleled.\nSOURCE_FLOAT must NOT connect to GND_LOAD_RETURN.',265,40,1.27)
part('J2','Conn_01x02','SOURCE',290,85,{'1':'RAIL_IN','2':'GND_LOAD_RETURN'},'TerminalBlock_Wuerth:Wuerth_691311400102_P7.62mm',purpose='Wuerth 691311400102, 7.62mm; feed through external appropriately rated <=1A fuse')
part('J3','Conn_01x02','LOAD',350,85,{'1':'RAIL_OUT','2':'GND_LOAD_RETURN'},'TerminalBlock_Wuerth:Wuerth_691311400102_P7.62mm',purpose='Switched line / unswitched return; resistive loads initially')
note('No gate-source bleed resistor: the Si8751 has a high\nsource impedance. A conventional 10k/100k pull-down\nwould collapse its generated gate voltage.\nMiller clamp connects through the drain capacitor.',265,103,1.27)
note('Start with resistive loads and <=1 on/off cycle per second.\nInductive loads need separately sized surge suppression.\nSCT01F isolation is functional; no mains safety certification.\nMOSFET tabs are live and at different potentials.',265,127,1.27)

note('3. ISOLATED MEASUREMENT POWER - converter return is GND_LOAD_RETURN',15,176)
part('U3','SCT01F03S05','SCT01F03S05',85,210,{'1':'GND_LOGIC','2':'+3V3','3':'GND_LOAD_RETURN','4':'ISO_5V_RAW'},'Devboard:SCT01F_SIP4',datasheet=xp,purpose='Unregulated isolated output; mechanical drawing bottom view converted to top view')
part('U4','TPS70933DBVR','TPS70933DBVR',165,210,{'1':'ISO_5V_RAW','2':'GND_LOAD_RETURN','3':None,'4':None,'5':'ISO_3V3'},'Package_TO_SOT_SMD:SOT-23-5','https://www.ti.com/lit/ds/symlink/tps709.pdf','EN intentionally floating per datasheet; never tie EN to raw converter output')
rc('C5','10u / 16V',30,211,'+3V3','GND_LOGIC','Converter input bulk')
rc('C6','100n / 16V',30,247,'+3V3','GND_LOGIC','Converter input bypass')
rc('C7','1u / 25V',85,247,'ISO_5V_RAW','GND_LOAD_RETURN','Converter output and regulator input bypass')
rc('C8','4.7u / 16V',165,247,'ISO_3V3','GND_LOAD_RETURN','X7R; >=2.2uF effective at 3.3V for regulator stability')
note('4. SWITCHED OUTPUT MEASUREMENT',220,155)
for i,x in enumerate([245,285,325]):
    rc(f'R{10+i}','390k / 0.1%',x,177,'RAIL_OUT' if i==0 else f'DIV_{i}',f'DIV_{i+1}' if i<2 else 'SENSE_IN','0805, >=150V working, >=0.125W, <=25ppm/K')
rc('R13','4.99k / 0.1%',370,177,'SENSE_IN','GND_LOAD_RETURN','0805 <=25ppm/K; divider factor 235.469; input 0.721Vpk at 120Vac')
part('U5','AMC0330RDWVR','AMC0330RDWVR',290,224,{'1':'ISO_3V3','2':'SENSE_IN','3':'GND_LOAD_RETURN','4':'GND_LOAD_RETURN','5':'GND_LOGIC','6':'+3V3','7':'VOUT_ADC','8':'+3V3'},'Package_SO:SOIC-8_7.5x5.85mm_P1.27mm',amc,'SNSN Kelvin connected to divider bottom / GND1; ADC reference must equal +3V3')
rc('C9','100n / 16V',220,247,'ISO_3V3','GND_LOAD_RETURN','AMC VDD1 local bypass')
rc('C10','1u / 16V',250,247,'ISO_3V3','GND_LOAD_RETURN','AMC VDD1 bulk')
rc('C11','100n / 16V',330,243,'+3V3','GND_LOGIC','AMC VDD2 local bypass')
rc('C12','1u / 16V',365,243,'+3V3','GND_LOGIC','AMC VDD2 bulk')
rc('C13','100n / 16V',370,218,'+3V3','GND_LOGIC','AMC REFIN local bypass')
rc('C14','100p / 50V',220,211,'SENSE_IN','GND_LOAD_RETURN','C0G input filter; bandwidth set after divider values')
note('OUT = VREF/2 at zero input AND when isolated supply is missing. Zero output indication is not proof of a de-energized rail.',15,278,1.27)
for i,(net,x) in enumerate([('+3V3',35),('GND_LOGIC',48)]):
    part(f'#FLG0{i+1}','PWR_FLAG','PWR_FLAG',x,163,{'1':net})

library='(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor")\n'+'\n'.join(symbols.values())+'\n)'
(ROOT/'Devboard.kicad_sym').write_text(library,encoding='utf-8')
(ROOT/'sym-lib-table').write_text('(sym_lib_table (version 7) (lib (name "Devboard") (type "KiCad") (uri "${KIPRJMOD}/Devboard.kicad_sym") (options "") (descr "Project-local symbols verified against manufacturer pin tables")))\n',encoding='utf-8')
embedded=[]
for name,body in symbols.items(): embedded.append(body.replace('(symbol '+q(name),'(symbol '+q('Devboard:'+name),1))
sch=f'''(kicad_sch (version 20250114) (generator "eeschema") (uuid {q(uid('root'))}) (paper "A3")
 (title_block (title "Isolated AC/DC switch development board") (date "2026-09-04") (rev "A-PROTOTYPE") (comment 1 "120Vac / 170Vdc, <1A, resistive loads; bench validation required"))
 (lib_symbols {''.join(embedded)})
 {''.join(drawing)} (sheet_instances (path "/" (page "1"))))'''
(ROOT/(NAME+'.kicad_sch')).write_text(sch,encoding='utf-8')
(ROOT/(NAME+'.kicad_pro')).write_text(json.dumps({'meta':{'filename':NAME+'.kicad_pro','version':1},'net_settings':{'classes':[{'name':'Default','clearance':0.25,'track_width':0.25,'via_diameter':0.7,'via_drill':0.3}],'meta':{'version':3}}},indent=2),encoding='utf-8')
(ROOT/'circuit.json').write_text(json.dumps(parts,indent=2),encoding='utf-8')
with (ROOT/'BOM.csv').open('w',newline='',encoding='utf-8-sig') as f:
    w=csv.DictWriter(f,fieldnames=['reference','value','footprint','purpose','datasheet']);w.writeheader()
    for p in parts: w.writerow({k:p[k] for k in w.fieldnames})
print(f'Wrote {len(parts)} components, editable schematic and project-local symbol library.')
