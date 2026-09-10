from pathlib import Path
ROOT=Path(__file__).resolve().parent
def names(item,nn):return '('+' || '.join(f"{item}.NetName == '/{n}'" for n in nn)+')'
hi=['RAIL_IN','SWITCHED_PRE','RAIL_OUT','SOURCE_FLOAT','GATE_1','GATE_2','MCAP_1','MCAP_2','DIV_1','DIV_2']
flo=['SOURCE_FLOAT','GATE_1','GATE_2','MCAP_1','MCAP_2']
logic=['+3V3','GND_LOGIC','EN','EN_A','EN_B','TT_1','TT_2','VOUT_P','VOUT_N','AMC_LLDO','DIAG_N','I_FILTER','IOUT_ADC']
field=hi+['GND_LOAD_RETURN','AMC_HRAW','AMC_HLDO','SENSE_IN']
rules=['(version 1)']
def rule(title,cond,value):rules.append(f'(rule "{title}" (condition "{cond}") (constraint clearance (min {value}mm)))')
rule('Field voltage nodes',names('A',hi)+' || '+names('B',hi),.8)
rule('Low voltage floating gate domain',names('A',flo)+' && '+names('B',flo),.3)
cross='(('+names('A',logic)+' && '+names('B',field)+') || ('+names('B',logic)+' && '+names('A',field)+'))'
rule('Control to live field copper',cross,2.5)
rule('ACS conductor equipotential terminals',names('A',['SWITCHED_PRE','RAIL_OUT'])+' && '+names('B',['SWITCHED_PRE','RAIL_OUT']),.3)
# Unconnected pins still need normal fabrication spacing; no signal is applied.
rule('Unused pins',"A.NetName == 'unconnected-*' || B.NetName == 'unconnected-*'",.25)
(ROOT/'SI8751AB-IS_devboard.kicad_dru').write_text('\n'.join(rules)+'\n')
