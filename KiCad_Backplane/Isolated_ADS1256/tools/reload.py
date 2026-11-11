"""Replace the open board with a given file and reopen it (no save).  usage: reload.py <source.kicad_pcb>"""
import sys, shutil, kipc
DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'Isolated_ADS1256.kicad_pcb'}
PATH = 'C:/Users/Nick/Documents/GitHub/Micro-DMM/KiCad_Backplane/Isolated_ADS1256/Isolated_ADS1256.kicad_pcb'
c = kipc.Client()
try:
    c.call('kiapi.common.commands.CloseDocument', {'document': DOC})
except kipc.ApiError as e:
    print('close:', e)
if sys.argv[1] != '-':
    shutil.copyfile(sys.argv[1], PATH)
c.call('kiapi.common.commands.OpenDocument', {'type': 'DOCTYPE_PCB', 'path': PATH})
print('reopened', sys.argv[1])
