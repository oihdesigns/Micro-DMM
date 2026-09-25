"""Rev C component data. PCB coordinates are initial placement hints only.

The edited KiCad board is authoritative; do not regenerate it from these hints.
"""
def populate(add):
    R='Resistor_SMD:R_0805_2012Metric'; C='Capacitor_SMD:C_0805_2012Metric'
    H=lambda n:f'Connector_PinHeader_2.54mm:PinHeader_1x0{n}_P2.54mm_Vertical'
    H6='Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical'
    def a(ref,v,kind,fp,n,sch,pcb,dnp=False,note=''):add(ref,v,kind,fp,n,sch,pcb,dnp,note)
    a('U1','AD8237ARMZ','AD8237','Package_SO:MSOP-8_3x3mm_P0.65mm',['BW','IN_P_F','IN_N_F','GND','VS','REF_PIN','FB','OUT_RAW'],[127,127,0],[122,121,0],note='Analog Devices AD8237ARMZ, RM-8')
    a('U2','TMUX1108PWR','TMUX1108','Package_SO:TSSOP-16_4.4x5mm_P0.65mm',['SEL0','VS','GND','UNITY','GAIN2_5','GAIN5','GAIN10','FB','GAIN1001','GAIN101','GAIN50','GAIN25','VS','GND','SEL2','SEL1'],[279.4,130.81,0],[133,119.5,0],note='TI TMUX1108PWR, PW TSSOP-16; EN tied high; VSS tied GND; fail-safe logic')
    a('U3','TCA9536DGKR','TCA9536','Package_SO:VSSOP-8_3x3mm_P0.65mm',['SEL0','SEL1','SEL2','GND',None,'SCL','SDA','VIO'],[279.4,54.61,0],[149,113.5,0],note='TI TCA9536DGKR; I2C 7-bit address 0x41; P0-P2 select gain; P3 unused')
    a('U4','OPA333AIDBVR','OPA333','Package_TO_SOT_SMD:SOT-23-5',['VMID','GND','MID_DIV','VMID','VS'],[119.38,223.52,0],[119,140,0],note='TI OPA333AIDBVR, SOT-23-5; unity-gain midpoint buffer')
    a('J1','EXT ANALOG POWER','Conn2',H(2),['VS_EXT','GND'],[35.56,45.72,0],[105,109.5,0],note='External regulated analog supply input; select JP1 EXT')
    a('J2','SENSOR','Conn3',H(3),['IN_P','IN_N','GND'],[35.56,127,0],[105,118,0],note='1 IN+, 2 IN-, 3 GND')
    a('J3','OUTPUT / EXT REF','Conn3',H(3),['OUT','GND','REF_EXT'],[218.44,119.38,0],[156,129,0],note='1 OUT after R8, 2 GND, 3 external reference input')
    a('J4','I2C / LOGIC POWER','Conn4',H(4),['GND','VIO','SDA','SCL'],[236.22,48.26,0],[143,106,90],note='1 GND, 2 VIO input (host 3.3 V or 5 V), 3 SDA, 4 SCL; match VIO to host')
    a('JP1','ANALOG SUPPLY','Conn3',H(3),['VS_EXT','VS','VIO'],[119.38,45.72,0],[110,106,90],note='ONE shunt: 2-3 default powers analog from VIO; 1-2 selects external J1 supply')
    a('JP2','BW: LOW / HIGH','Conn3',H(3),['GND','BW','VS'],[194.31,150,0],[122,108,90],note='1-2 LOW default, valid for every gain. 2-3 HIGH requires gain >=10, including at startup')
    a('JP3','REFERENCE SELECT','RefSelect',H6,['REF','GND','REF','VMID','REF','REF_EXT'],[195.58,223.52,0],[139,140,0],note='ONE shunt: 1-2 GND; 3-4 MID default; 5-6 external J3.3')
    a('JP4','I2C PULL-UPS','Conn2',H(2),['VIO','PULL_V'],[391.16,78.74,0],[154,113,90],note='Fit shunt to enable 4.7k pull-ups to VIO. Remove when host provides sufficient pull-ups')
    rs=[
      ('R1','100R 0.1%',['IN_P','IN_P_F'],[78.74,121.92,0],[115.5,118.8,0]),
      ('R2','100R 0.1%',['IN_N','IN_N_F'],[78.74,132.08,0],[115.5,123.2,0]),
      ('R3','10k 0.1%',['GAIN10','REF'],[259.08,213.36,270],[141,121,0]),
      ('R4','90.9k 0.1%',['OUT_RAW','GAIN10'],[259.08,190.5,270],[141,117.5,0]),
      ('R5','1k 0.1%',['GAIN101','REF'],[302.26,213.36,270],[142,128.5,0]),
      ('R6','100k 0.1%',['OUT_RAW','GAIN101'],[302.26,190.5,270],[142,125,0]),
      ('R7','100k',['BW','GND'],[156.21,149.86,270],[120,112,0]),
      ('R8','100R',['OUT_RAW','OUT'],[177.8,119.38,0],[150,128.5,0]),
      ('R9','1M (DNP)',['IN_P_F','REF'],[106.68,171.45,270],[110,133,0]),
      ('R10','1M (DNP)',['IN_N_F','REF'],[137.16,171.45,270],[115,133,0]),
      ('R11','1k 0.1%',['GAIN1001','REF'],[345.44,213.36,270],[149,136,0]),
      ('R12','1M 0.1%',['OUT_RAW','GAIN1001'],[345.44,190.5,270],[149,132.5,0]),
      ('R13','2k',['OUT_RAW','UNITY'],[345.44,119.38,0],[137,113.5,0]),
      ('R14','200R',['REF','REF_PIN'],[160.02,246.38,0],[126,128.5,0]),
      ('R15','10k 0.1%',['VS','MID_DIV'],[40.64,208.28,270],[111,137,0]),
      ('R16','10k 0.1%',['MID_DIV','GND'],[40.64,231.14,270],[111,142,0]),
      ('R17','4.7k',['SEL0','GND'],[330.2,48.26,270],[141,111,0]),
      ('R18','4.7k',['SEL1','GND'],[358.14,48.26,270],[141,108,0]),
      ('R19','4.7k',['PULL_V','SDA'],[330.2,80.01,270],[155,120,0]),
      ('R20','4.7k',['PULL_V','SCL'],[358.14,80.01,270],[155,124,0]),
      ('R21','30k 0.1%',['OUT_RAW','GAIN2_5'],[238.76,190.5,270],[130.5,132,0]),
      ('R22','20k 0.1%',['GAIN2_5','REF'],[238.76,213.36,270],[135.5,132,0]),
      ('R23','40.2k 0.1%',['OUT_RAW','GAIN5'],[261.62,190.5,270],[130.5,135,0]),
      ('R24','10k 0.1%',['GAIN5','REF'],[261.62,213.36,270],[135.5,135,0]),
      ('R25','120k 0.1%',['OUT_RAW','GAIN25'],[307.34,190.5,270],[130.5,138,0]),
      ('R26','4.99k 0.1%',['GAIN25','REF'],[307.34,213.36,270],[135.5,138,0]),
      ('R27','100k 0.1%',['OUT_RAW','GAIN50'],[330.2,190.5,270],[129.5,145,0]),
      ('R28','2.05k 0.1%',['GAIN50','REF'],[330.2,213.36,270],[134.5,145,0]),
      ('R29','4.7k',['SEL2','GND'],[386.08,48.26,270],[143,114.5,0])]
    for ref,v,n,s,p in rs:
        note='Thin film, 0.1%, <=25 ppm/C' if '0.1%' in v else '1%, 0805'
        if ref in ['R9','R10']:note='Optional sensor DC bias return to selected REF; not fitted'
        if ref in ['R17','R18','R29']:note='1% pull-down: <=0.475 V with expander maximum 100 uA internal pull-up; default unity'
        if ref=='R13':note='Unity sense series resistor, as in AD8237 Figure 76'
        if ref=='R14':note='REF input bias compensation; divider returns use REF before this resistor'
        a(ref,v,'R',R,n,s,p,ref in ['R9','R10'],note)
    cs=[
      ('C1','10uF / 16V',['VS','GND'],[76.2,50.8,0],[112,110.5,0]),
      ('C2','100nF / 16V',['VS','GND'],[152.4,50.8,0],[125,125,0]),
      ('C3','10nF (DNP)',['IN_P_F','IN_N_F'],[30.48,171.45,0],[110,121.5,270]),
      ('C4','1nF (DNP)',['IN_P_F','GND'],[55.88,171.45,0],[109,128,0]),
      ('C5','1nF (DNP)',['IN_N_F','GND'],[81.28,171.45,0],[114,128.5,0]),
      ('C6','10nF (DNP)',['OUT','GND'],[218.44,144.78,0],[151,139.5,0]),
      ('C7','470pF C0G',['OUT_RAW','FB'],[177.8,177.8,0],[125.5,117,0]),
      ('C8','1uF / 16V',['MID_DIV','GND'],[78.74,220.98,0],[115,140,0]),
      ('C9','100nF / 16V',['VS','GND'],[152.4,220.98,0],[121,145,0]),
      ('C10','100nF / 16V',['VS','GND'],[381,144.78,0],[135.5,123,0]),
      ('C11','100nF / 16V',['VIO','GND'],[312.42,73.66,0],[153,116.5,0]),
      ('C12','1uF / 16V',['VIO','GND'],[190.5,50.8,0],[155,109.5,90])]
    for ref,v,n,s,p in cs:
        note='X7R, 16 V; local supply bypass'
        if ref in ['C3','C4','C5','C6']:note='DNP; optional C0G filter; match C4/C5 1% if populated'
        if ref=='C7':note='C0G 5%, >=16 V; high-frequency feedback compensation, AD8237 Figure 76'
        if ref=='C8':note='X7R 16 V; midpoint divider filter, nominal 5 ms time constant'
        a(ref,v,'C',C,n,s,p,ref in ['C3','C4','C5','C6'],note)
    for i,(net,xy) in enumerate([('VS',(124,113)),('GND',(107,137)),('IN_P_F',(120,131)),('IN_N_F',(120,135)),('OUT_RAW',(134,109)),('FB',(135,128)),('REF',(129,141)),('VMID',(124,141))],1):
        a('TP'+str(i),net,'TP','TestPoint:TestPoint_Pad_D1.5mm',[net],[27.94+(i-1)*25.4,264.16,0],[*xy,0],note='Exposed 1.5 mm probe pad')
    for i,xy in enumerate([(103.5,103.5),(156.5,103.5),(103.5,146.5),(156.5,146.5)],1):
        a('H'+str(i),'M3 NPTH','Hole','MountingHole:MountingHole_3.2mm_M3',[],[241.3+(i-1)*15.24,259.08,0],[*xy,0],note='3.2 mm non-plated mounting hole')
