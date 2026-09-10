"""Generate revision C of the 120Vac / 170Vdc, <1A bench development circuit."""
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
block('AMC3330DWER',[(1,'DCDC_OUT','power_out'),(2,'DCDC_HGND','passive'),(3,'HLDO_IN','power_in'),(4,'NC','passive'),(5,'HLDO_OUT','power_out'),(6,'INP','input'),(7,'INN','input'),(8,'HGND','passive')],[(16,'DCDC_IN','power_in'),(15,'DCDC_GND','power_in'),(14,'DIAG','open_collector'),(13,'LDO_OUT','power_out'),(12,'VDD','power_in'),(11,'OUTP','output'),(10,'OUTN','output'),(9,'GND','power_in')],width=17.78)
block('ACS725LLCTR-05AB-T',[(1,'IP+','passive'),(2,'IP+','passive'),(3,'IP-','passive'),(4,'IP-','passive')],[(8,'VCC','power_in'),(7,'VIOUT','output'),(6,'FILTER','passive'),(5,'GND','power_in')],width=17.78)
for lib,name in [('Device','R'),('Device','C'),('Transistor_FET','Q_NMOS_GDS'),('Connector_Generic','Conn_01x02'),('Connector_Generic','Conn_01x10'),('power','PWR_FLAG')]:
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
        if typ=='PWR_FLAG':hide=True
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
amc='https://www.ti.com/lit/ds/symlink/amc3330.pdf'
xp='https://www.xppower.com/storage/portals/0/pdfs/SF_SCT01F.pdf'

note('AC / DC SOLID-STATE SWITCH + ISOLATED VOLTAGE AND CURRENT SENSE',15,15,2.54)
note('REV C - 120 Vac / 170 Vdc, <1 A - RESISTIVE BENCH LOADS - EXTERNAL FUSE REQUIRED',15,23,1.524)
note('1. LOGIC CONTROL - 3.3 V supply, active-high EN, both drivers always commanded together',15,30)
part('J1','Conn_01x10','CONTROL / ADC',40,65,{'1':'+3V3','2':'GND_LOGIC','3':'EN','4':'VOUT_P','5':'VOUT_N','6':'GND_LOGIC','7':None,'8':'IOUT_ADC','9':'DIAG_N','10':'GND_LOGIC'},'Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical',purpose='Rev C: 3.3V only; pin7 NC (formerly 5V); current output is no longer divided')
rc('R1','100R',65,60,'EN','EN_A','Series input damping')
rc('R2','75k / 0.1%',65,101,'EN_A','GND_LOGIC','Default OFF; stocked ERA-6AEB753V')
rc('R3','100R',65,125,'EN','EN_B','Series input damping')
rc('R4','75k / 0.1%',65,155,'EN_B','GND_LOGIC','Default OFF; stocked ERA-6AEB753V')
for i,y in [(1,60),(2,125)]:
    part(f'U{i}','SI8751AB-IS','SI8751AB-IS',125,y,{'1':'+3V3','2':f'TT_{i}','3':'EN_A' if i==1 else 'EN_B','4':'GND_LOGIC','5':'SOURCE_FLOAT','6':None,'7':f'MCAP_{i}','8':f'GATE_{i}'},'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',si,'One isolated gate driver per MOSFET')
    rc(f'C{i}','100n / 25V',95,y+27.94,'+3V3','GND_LOGIC','Place beside driver VDD / GND')
    rc(f'R{i+4}','10k',125,y+27.94,f'TT_{i}','GND_LOGIC','TT=10k: 9V minimum unloaded gate output; qualify actual VGS under load')
    rc(f'C{i+2}','10p / 1kV',175,y-2.54,'RAIL_IN' if i==1 else 'SWITCHED_PRE',f'MCAP_{i}','0805 C0G 1kV; qualify dV/dt <=600V/us for 6mA MCAP limit')
    part(f'Q{i}','Q_NMOS_GDS','STP12N60M2',235,y,{'1':f'GATE_{i}','2':'RAIL_IN' if i==1 else 'SWITCHED_PRE','3':'SOURCE_FLOAT'},'Package_TO_SOT_THT:TO-220-3_Vertical','https://www.st.com/resource/en/datasheet/stp12n60m2.pdf','600V NMOS; RDSon specified at 10V; qualify VGS and dissipation on prototype')
note('2. BIDIRECTIONAL POWER PATH',265,30)
note('Q1/Q2 sources share SOURCE_FLOAT.\nOpposed body diodes block either polarity when OFF.\nGates are separate; driver outputs are not paralleled.\nSOURCE_FLOAT must NOT connect to GND_LOAD_RETURN.',265,40,1.27)
part('J2','Conn_01x02','SOURCE',290,85,{'1':'RAIL_IN','2':'GND_LOAD_RETURN'},'TerminalBlock_Wuerth:Wuerth_691311400102_P7.62mm',purpose='Wuerth 691311400102, 7.62mm; feed through external appropriately rated <=1A fuse')
part('J3','Conn_01x02','LOAD',350,85,{'1':'RAIL_OUT','2':'GND_LOAD_RETURN'},'TerminalBlock_Wuerth:Wuerth_691311400102_P7.62mm',purpose='Switched line / unswitched return; resistive loads initially')
note('No gate-source bleed resistor: the Si8751 has a high\nsource impedance. A conventional 10k/100k pull-down\nwould collapse its generated gate voltage.\nMiller clamp connects through the drain capacitor.',265,103,1.27)
note('Start with resistive loads and <=1 on/off cycle per second.\nInductive loads need separately sized surge suppression.\nBoard is a bench prototype; no mains safety certification.\nMOSFET tabs are live and at different potentials.',265,127,1.27)

note('3. ISOLATED CURRENT SENSOR - positive current from Q2 to J3',380,165)
part('U6','ACS725LLCTR-05AB-T','ACS725LLCTR-05AB-T',445,200,{'1':'SWITCHED_PRE','2':'SWITCHED_PRE','3':'RAIL_OUT','4':'RAIL_OUT','5':'GND_LOGIC','6':'I_FILTER','7':'IOUT_ADC','8':'+3V3'},'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm','https://www.allegromicro.com/-/media/files/datasheets/acs725-datasheet.ashx','3.3V supply; 264mV/A nominal; zero-current output VCC/2; isolated series current conductor')
rc('C15','100n / 25V',385,245,'+3V3','GND_LOGIC','ACS VCC bypass')
rc('C16','1u / 50V',425,245,'+3V3','GND_LOGIC','ACS VCC bulk')
rc('C17','100n / 25V',470,245,'I_FILTER','GND_LOGIC','Internal 1.8k plus 100n: approximately 884Hz pole')
rc('C18','1n / 50V',525,267,'IOUT_ADC','GND_LOGIC','Direct output load; total output capacitance <=10nF and DC load >=4.7k; qualify host ADC settling')
note('At VCC=3.3V: IOUT_ADC = 1.65V + 0.264V/A * I(A).\nR15/R16 divider removed. Calibrate zero and gain.\nC17=100n gives approximately 884Hz filter bandwidth.',380,292,1.27)
note('4. SELF-POWERED ISOLATED VOLTAGE MEASUREMENT',15,185)
for i,x in enumerate([40,90,140]):
    rc(f'R{10+i}','1M / 0.1%',x,217,'RAIL_OUT' if i==0 else f'DIV_{i}',f'DIV_{i+1}' if i<2 else 'SENSE_IN','RG2012P-105-B-T5; 0805, 150V limiting element voltage, 0.125W')
rc('R13','15k / 0.1%',190,217,'SENSE_IN','GND_LOAD_RETURN','RG2012P-153-B-T5; divider factor 201; input 0.8443Vpk at 120Vac')
part('U5','AMC3330DWER','AMC3330DWER',185,270,{'1':'AMC_HRAW','2':'GND_LOAD_RETURN','3':'AMC_HRAW','4':'GND_LOAD_RETURN','5':'AMC_HLDO','6':'SENSE_IN','7':'GND_LOAD_RETURN','8':'GND_LOAD_RETURN','9':'GND_LOGIC','10':'VOUT_N','11':'VOUT_P','12':'+3V3','13':'AMC_LLDO','14':'DIAG_N','15':'GND_LOGIC','16':'AMC_LLDO'},'Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm',amc,'Integrated isolated DC/DC; fixed differential gain 2; stocked AMC3330QDWERQ1 is an assembly alternative')
rc('C5','1n / 50V',40,327,'AMC_HRAW','GND_LOAD_RETURN','DCDC_OUT pin1 / DCDC_HGND pin2 HF bypass')
rc('C6','1u / 50V',85,327,'AMC_HRAW','GND_LOAD_RETURN','DCDC_OUT bulk; 1u effective target')
rc('C19','100n / 25V',355,327,'AMC_HRAW','GND_LOAD_RETURN','HLDO_IN local bypass at pin3; no ferrite beads fitted')
rc('C7','1n / 50V',130,327,'AMC_HLDO','GND_LOAD_RETURN','HLDO_OUT HF bypass')
rc('C8','100n / 25V',175,327,'AMC_HLDO','GND_LOAD_RETURN','HLDO_OUT bypass; no external load')
rc('C11','1n / 50V',220,327,'+3V3','GND_LOGIC','VDD HF bypass')
rc('C12','1u / 50V',265,327,'+3V3','GND_LOGIC','VDD bulk; 1u effective target')
rc('C13','100n / 25V',310,327,'AMC_LLDO','GND_LOGIC','DCDC_IN pin16 to DCDC_GND pin15; no external load')
rc('C14','1n / 50V',75,270,'SENSE_IN','GND_LOAD_RETURN','Input filter: approximately 10.7kHz; return at AMC INN')
rc('R14','10k / 0.1%',275,270,'+3V3','DIAG_N','Active-low open-drain diagnostic pullup; LOW means invalid measurement')
note('Vrail = 100.5 * (VOUT_P - VOUT_N), with voltages in volts.\nOUTP/OUTN require a differential ADC or external difference amplifier.\nLoss of measurement power is not proof of a de-energized rail.',15,360,1.27)
note('U3/U4 and C9/C10 retired from revision A. No external isolated converter required.\nJ1 pinout changed: pin5 is now OUTN. Supply +3.3V at pin1; pin7 is now NC. Do not apply 5V.\nAll resistors and capacitors use 0805 packages. See BOM for inventory matches.',15,383,1.27)
for i,(net,x) in enumerate([('+3V3',35),('GND_LOGIC',48)]):
    part(f'#FLG0{i+1}','PWR_FLAG','PWR_FLAG',x,173,{'1':net})

library='(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor")\n'+'\n'.join(symbols.values())+'\n)'
(ROOT/'Devboard.kicad_sym').write_text(library,encoding='utf-8')
(ROOT/'sym-lib-table').write_text('(sym_lib_table (version 7) (lib (name "Devboard") (type "KiCad") (uri "${KIPRJMOD}/Devboard.kicad_sym") (options "") (descr "Project-local symbols verified against manufacturer pin tables")))\n',encoding='utf-8')
embedded=[]
for name,body in symbols.items(): embedded.append(body.replace('(symbol '+q(name),'(symbol '+q('Devboard:'+name),1))
sch=f'''(kicad_sch (version 20250114) (generator "eeschema") (uuid {q(uid('root'))}) (paper "A2")
 (title_block (title "Isolated AC/DC switch development board") (date "2026-09-05") (rev "C-PROTOTYPE") (comment 1 "120Vac / 170Vdc, <1A, resistive loads; bench validation required"))
 (lib_symbols {''.join(embedded)})
 {''.join(drawing)} (sheet_instances (path "/" (page "1"))))'''
(ROOT/(NAME+'.kicad_sch')).write_text(sch,encoding='utf-8')
if not (ROOT/(NAME+'.kicad_pro')).exists():
    (ROOT/(NAME+'.kicad_pro')).write_text(json.dumps({'meta':{'filename':NAME+'.kicad_pro','version':1},'net_settings':{'classes':[{'name':'Default','clearance':0.25,'track_width':0.25,'via_diameter':0.7,'via_drill':0.3}],'meta':{'version':3}}},indent=2),encoding='utf-8')
(ROOT/'circuit.json').write_text(json.dumps(parts,indent=2),encoding='utf-8')
with (ROOT/'BOM.csv').open('w',newline='',encoding='utf-8-sig') as f:
    w=csv.DictWriter(f,fieldnames=['reference','value','footprint','purpose','datasheet']);w.writeheader()
    for p in parts: w.writerow({k:p[k] for k in w.fieldnames})
print(f'Wrote {len(parts)} components, editable schematic and project-local symbol library.')
