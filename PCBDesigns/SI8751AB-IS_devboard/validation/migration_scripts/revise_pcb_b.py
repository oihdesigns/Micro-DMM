from pathlib import Path
p=Path('build_pcb.py');s=p.read_text()
start=s.index('# Converter drawing');end=s.index('for p in parts:',start);s=s[:start]+s[end:]
s=s.replace("pad.SetSize(v(1.65,1.905));ls=pad.GetLayerSet();ls.AddLayer(k.F_Mask);ls.AddLayer(k.B_Mask);pad.SetLayerSet(ls)","pad.SetSize(v(1.65,1.905))")
s=s.replace("        if pad.GetAttribute()==k.PAD_ATTRIB_PTH:\n            ls=pad.GetLayerSet();ls.AddLayer(k.F_Mask);ls.AddLayer(k.B_Mask);pad.SetLayerSet(ls)\n",'')
start=s.index('pos={');end=s.index('footprints={}',start)
s=s[:start]+'''pos={
'J1':(29,40,0), 'U1':(64,38,0),'U2':(64,58,0),'U5':(64,90,180),'U6':(64,73,180),
'Q1':(86,38,0),'Q2':(86,58,180),'J2':(114,36,90),'J3':(114,68,90),
'R1':(43,43,0),'R2':(51,44,90),'R3':(43,57,0),'R4':(51,62,90),'R5':(56,38,90),'R6':(56,58,90),
'C1':(58,33,90),'C2':(58,53,90),'C3':(76,34,0),'C4':(76,55,0),
'C5':(72.7,94.5,90),'C6':(77,94.5,90),'C7':(72.7,89.4,90),'C8':(78,89.4,90),
'C11':(55.5,89.4,90),'C12':(50.5,89.4,90),'C13':(55.5,94.5,90),'C19':(82,94.5,90),
'R10':(104,85,0),'R11':(98,85,0),'R12':(92,85,0),'R13':(73,84,90),'C14':(78,84,90),
'R14':(50,96,90),'C15':(58,78,90),'C16':(53,78,90),'C17':(56,70,90),
'R15':(48,70,0),'R16':(42,75,90),'C18':(37,75,90)
}
'''+s[end:]
s=s.replace("text('REV A  |", "text('REV B  |")
s=s.replace("text('3V3  GND  EN  OUT  GND',38,64,.9)","\nfor i,label in enumerate(['1  3V3','2  GND','3  EN','4  VP','5  VN','6  GND','7  5V','8  IOUT','9  DIAG','10 GND']):text(label,35,40+2.54*i,.75)")
s=s.replace("text('3.3V INPUT ONLY',42,99,.9)","text('3V3 + 5V INPUTS',41,102,.9)")
s=s.replace("[(23,32),(43,52),(63,66),(80,83),(97,101)]","[(23,32),(43,52),(63,68),(78,83),(97,101)]")
start=s.index('# Through-board milling slot');end=s.index('# Reserve four mounting holes',start);s=s[:start]+s[end:]
s=s.replace("power={'RAIL_IN','RAIL_OUT','SOURCE_FLOAT','GND_LOAD_RETURN'}","power={'RAIL_IN','SWITCHED_PRE','RAIL_OUT','SOURCE_FLOAT','GND_LOAD_RETURN'}")
s=s.replace("primary={'+3V3','GND_LOGIC','EN','EN_A','EN_B','TT_1','TT_2','VOUT_ADC'}","primary={'+3V3','+5V','GND_LOGIC','EN','EN_A','EN_B','TT_1','TT_2','VOUT_P','VOUT_N','AMC_LLDO','DIAG_N','I_RAW','I_FILTER','IOUT_ADC'}")
s=s.replace("hi={'RAIL_IN','RAIL_OUT'","hi={'RAIL_IN','SWITCHED_PRE','RAIL_OUT'")
s=s.replace("    if a==b:return 0","    if a==b:return 0\n    if {a,b}=={'SWITCHED_PRE','RAIL_OUT'}:return .3")
start=s.index('    # Only the converter pin escapes');end=s.index('    for p in allpads:',start);s=s[:start]+s[end:]
s=s.replace("            if (n in primary)!=(other in primary) and min(a[0],b[0])<68 and 72<=a[1]<=76 and 72<=b[1]<=76:gap=.8\n",'')
s=s.replace("order=['MCAP_1','MCAP_2','GATE_1','GATE_2','SOURCE_FLOAT','RAIL_IN','RAIL_OUT','GND_LOAD_RETURN','DIV_1','DIV_2','SENSE_IN','ISO_5V_RAW','ISO_3V3','EN_A','EN_B','TT_1','TT_2','VOUT_ADC','EN','+3V3','GND_LOGIC']", "order=['MCAP_1','MCAP_2','GATE_1','GATE_2','SOURCE_FLOAT','RAIL_IN','SWITCHED_PRE','RAIL_OUT','DIV_1','DIV_2','SENSE_IN','AMC_HRAW','AMC_HLDO','AMC_LLDO','GND_LOAD_RETURN','I_RAW','I_FILTER','IOUT_ADC','EN_A','EN_B','TT_1','TT_2','DIAG_N','VOUT_P','VOUT_N','EN','+5V','+3V3','GND_LOGIC']")
s=s.replace(",('U4','3','EN'),('U4','4','NC')",'')
p.write_text(s)
p=Path('make_rules.py');s=p.read_text().replace("hi=['RAIL_IN','RAIL_OUT'","hi=['RAIL_IN','SWITCHED_PRE','RAIL_OUT'")
s=s.replace("logic=['+3V3','GND_LOGIC','EN','EN_A','EN_B','TT_1','TT_2','VOUT_ADC']","logic=['+3V3','+5V','GND_LOGIC','EN','EN_A','EN_B','TT_1','TT_2','VOUT_P','VOUT_N','AMC_LLDO','DIAG_N','I_RAW','I_FILTER','IOUT_ADC']")
s=s.replace("'ISO_3V3','ISO_5V_RAW'","'AMC_HRAW','AMC_HLDO'")
start=s.index('# Converter is');end=s.index('# Unconnected pins',start)
s=s[:start]+"rule('ACS conductor equipotential terminals',names('A',['SWITCHED_PRE','RAIL_OUT'])+' && '+names('B',['SWITCHED_PRE','RAIL_OUT']),.3)\n"+s[end:]
p.write_text(s)
