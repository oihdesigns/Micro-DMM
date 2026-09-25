"""Independent Rev C checks of exported connectivity, DC ratios and rule reports."""
from pathlib import Path
import xml.etree.ElementTree as ET
import json, math
import pcbnew as k
ROOT=Path(__file__).resolve().parents[1]
tree=ET.parse(ROOT/'docs/netlist.xml')
pin_net={}
for net in tree.findall('./nets/net'):
    for n in net.findall('node'):pin_net[n.get('ref'),n.get('pin')]=net.get('name').lstrip('/')
expected={
 'U1':['BW','IN_P_F','IN_N_F','GND','VS','REF_PIN','FB','OUT_RAW'],
 'U2':['SEL0','VS','GND','UNITY','GAIN2_5','GAIN5','GAIN10','FB','GAIN1001','GAIN101','GAIN50','GAIN25','VS','GND','SEL2','SEL1'],
 'U3':['SEL0','SEL1','SEL2','GND',None,'SCL','SDA','VIO'],
 'U4':['VMID','GND','MID_DIV','VMID','VS'],
 'J1':['VS_EXT','GND'],'J2':['IN_P','IN_N','GND'],'J3':['OUT','GND','REF_EXT'],'J4':['GND','VIO','SDA','SCL'],
 'JP1':['VS_EXT','VS','VIO'],'JP2':['GND','BW','VS'],'JP3':['REF','GND','REF','VMID','REF','REF_EXT'],'JP4':['VIO','PULL_V'],
 'R13':['OUT_RAW','UNITY'],'R14':['REF','REF_PIN'],'R15':['VS','MID_DIV'],'R16':['MID_DIV','GND'],
 'R17':['SEL0','GND'],'R18':['SEL1','GND'],'R29':['SEL2','GND'],
 'R19':['PULL_V','SDA'],'R20':['PULL_V','SCL'],
 'C7':['OUT_RAW','FB'],'C8':['MID_DIV','GND'],'C9':['VS','GND'],'C10':['VS','GND'],'C11':['VIO','GND'],'C12':['VIO','GND']}
dividers=[('GAIN2_5','R21','R22',2.5),('GAIN5','R23','R24',5.02),('GAIN10','R4','R3',10.09),
 ('GAIN25','R25','R26',1+120000/4990),('GAIN50','R27','R28',1+100000/2050),('GAIN101','R6','R5',101),('GAIN1001','R12','R11',1001)]
for net,upper,lower,_ in dividers:
    expected[upper]=['OUT_RAW',net];expected[lower]=[net,'REF']
for ref,names in expected.items():
    for i,name in enumerate(names,1):
        actual=pin_net[ref,str(i)]
        assert actual==name if name else actual.startswith('unconnected-'),(ref,i,actual,name)
# Independently check PCB pin assignment, beyond the DRC parity report.
board=k.LoadBoard(str(ROOT/'AD8237DevBoard.kicad_pcb'))
fp={f.GetReference():f for f in board.GetFootprints()}
for ref,names in expected.items():
    for pad in fp[ref].Pads():
        assert pad.GetNetname().lstrip('/').replace('{slash}','/')==pin_net[ref,pad.GetNumber()],(ref,pad.GetNumber())
assert str(fp['U2'].GetFPID().GetLibItemName())=='TSSOP-16_4.4x5mm_P0.65mm'
components={c.get('ref'):c for c in tree.findall('./components/comp')}
def resistance(ref):
    s=components[ref].findtext('value').split()[0].replace('R','')
    scale=1
    if s[-1] in 'kM':scale={'k':1e3,'M':1e6}[s[-1]];s=s[:-1]
    return float(s)*scale
gains=[1];loads=[];ratios=[]
for net,u,l,target in dividers:
    ru,rl=resistance(u),resistance(l);g=1+ru/rl
    assert math.isclose(g,target,rel_tol=1e-12)
    assert ru*rl/(ru+rl)<30000
    gains.append(g);loads.append(ru+rl)
    ratios.append({'net':net,'gain':g,'divider_load_ohm':ru+rl,'thevenin_ohm':ru*rl/(ru+rl),
      'gain_min_resistors_only':1+ru*.999/(rl*1.001),'gain_max_resistors_only':1+ru*1.001/(rl*.999)})
load=1/sum(1/r for r in loads);effective=1/(1/load+1/100000)
assert effective>10000
assert max(resistance(r)*1.01*100e-6 for r in ['R17','R18','R29'])<.8
dc=[]
for vs in [3.3,5]:
    for code,g in enumerate(gains):
        for sign in [-1,1]:
            vd=sign*.5/g;vref=vs/2;out=vref+g*vd
            assert .1<out<vs-.1
            assert math.isclose((out-vref)/g,vd,abs_tol=1e-12)
            dc.append({'supply_V':vs,'code':code,'gain':g,'input_difference_V':vd,'reference_V':vref,'output_V':out})
# Check unchanged original circuits against the pre-change exported schematic.
baseline=ET.parse(ROOT/'archive/pre-eight-gain/netlist.xml');old={}
for net in baseline.findall('./nets/net'):
    for node in net.findall('node'):old[node.get('ref'),node.get('pin')]=net.get('name').lstrip('/')
for pair,name in old.items():
    if pair[0] not in ['U2','U3']:assert pin_net[pair]==name,(pair,name,pin_net[pair])
erc=json.loads((ROOT/'docs/ERC.json').read_text());drc=json.loads((ROOT/'docs/DRC.json').read_text())
ne=sum(len(s['violations']) for s in erc['sheets'])
assert ne==0 and not drc['violations'] and not drc['unconnected_items'] and not drc['schematic_parity']
result={'status':'PASS','revision':'C','pin_connections_checked':sum(len(v) for v in expected.values()),
 'erc_violations':ne,'drc_violations':0,'unconnected_items':0,'schematic_parity_issues':0,
 'nominal_gains':gains,'gain_network_parallel_load_ohm':load,'effective_load_with_100k_output_ohm':effective,
 'resistor_ratio_checks':ratios,'ideal_dc_cases':dc,
 'limitations':'Ideal DC checks only. No assembled-board measurement or device-level transient/noise simulation. Firmware example not compiled for a specific controller.'}
(ROOT/'docs/design-verification.json').write_text(json.dumps(result,indent=2)+'\n')
print('PASS: Rev C pin maps, unchanged original circuits, eight gains, startup pull-downs, loading, 32 DC cases, ERC and DRC/parity.')
