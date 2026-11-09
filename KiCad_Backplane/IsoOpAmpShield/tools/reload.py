"""Save the open board, replace it with a given file, reopen.  usage: reload.py <source.kicad_pcb|-> (- = just reopen)"""
import sys, shutil, kipc
DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'IsoOpAmpShield.kicad_pcb'}
PATH = 'C:/Users/Nick/Documents/GitHub/Micro-DMM/KiCad_Backplane/IsoOpAmpShield/IsoOpAmpShield.kicad_pcb'
c = kipc.Client()
try:
    c.call('kiapi.common.commands.SaveDocument', {'document': DOC})
    c.call('kiapi.common.commands.CloseDocument', {'document': DOC})
except kipc.ApiError as e:
    print('close:', e)
if sys.argv[1] != '-':
    shutil.copyfile(sys.argv[1], PATH)
c.call('kiapi.common.commands.OpenDocument', {'type': 'DOCTYPE_PCB', 'path': PATH})
print('reopened', sys.argv[1])
