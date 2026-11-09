"""Close the (saved, clean) board on the IPC server and reopen it from disk WITHOUT saving."""
import kipc
DOC = {'type': 'DOCTYPE_PCB', 'board_filename': 'IsoOpAmpShield.kicad_pcb'}
c = kipc.Client()
c.call('kiapi.common.commands.CloseDocument', {'document': DOC})
c.call('kiapi.common.commands.OpenDocument', {'type': 'DOCTYPE_PCB', 'path': 'C:/Users/Nick/Documents/GitHub/Micro-DMM/KiCad_Backplane/IsoOpAmpShield/IsoOpAmpShield.kicad_pcb'})
print('reopened from disk')
