"""Print message/enum definitions from the Backplane runtime's api.desc.  usage: protodesc.py NAME_SUBSTRING..."""
import sys
from google.protobuf import descriptor_pb2

DESC = 'C:/Users/Nick/AppData/Local/Programs/Backplane-KiCad/10.99.0-bp.6/share/backplane/api.desc'
fds = descriptor_pb2.FileDescriptorSet()
fds.ParseFromString(open(DESC, 'rb').read())
want = [w.lower() for w in sys.argv[1:]]
T = descriptor_pb2.FieldDescriptorProto
for f in fds.file:
    for m in f.message_type:
        full = f.package + '.' + m.name
        if any(w in full.lower() for w in want):
            print('MSG', full)
            for fl in m.field:
                print('   ', fl.number, fl.name, fl.type_name or T.Type.Name(fl.type), 'repeated' if fl.label == 3 else '')
    for e in f.enum_type:
        full = f.package + '.' + e.name
        if any(w in full.lower() for w in want):
            print('ENUM', full, [v.name for v in e.value])
