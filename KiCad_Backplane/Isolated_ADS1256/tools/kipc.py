"""Minimal KiCad IPC client (nng + protobuf built from the Backplane runtime's api.desc).

    from kipc import Client
    c = Client(SOCKET)
    r = c.call('kiapi.common.commands.GetItems', {...json...})
"""
import os
import pynng
from google.protobuf import descriptor_pb2, descriptor_pool, json_format, message_factory
from google.protobuf import any_pb2

DESC = os.environ.get('KIPC_DESC', 'C:/Users/Nick/AppData/Local/Programs/Backplane-KiCad/10.99.0-bp.6/share/backplane/api.desc')
SOCKET = os.environ.get('KIPC_SOCKET', 'C:/Users/Nick/AppData/Local/Temp/isoads1256_kicad_api.sock')

def _has(name):
    try:
        POOL.FindFileByName(name)
        return True
    except KeyError:
        return False


_fds = descriptor_pb2.FileDescriptorSet()
_fds.ParseFromString(open(DESC, 'rb').read())
from google.protobuf import timestamp_pb2, empty_pb2, struct_pb2, wrappers_pb2, field_mask_pb2, duration_pb2  # noqa: register WKTs
POOL = descriptor_pool.Default()
_pending = [f for f in _fds.file]
_added = set()
while _pending:
    progressed = False
    for f in list(_pending):
        try:
            POOL.FindFileByName(f.name)
            _added.add(f.name)
            _pending.remove(f)
            progressed = True
            continue
        except KeyError:
            pass
        if all(d in _added or _has(d) for d in f.dependency):
            POOL.AddSerializedFile(f.SerializeToString())
            _added.add(f.name)
            _pending.remove(f)
            progressed = True
    if not progressed:
        raise RuntimeError('cannot resolve proto deps: %s' % [f.name for f in _pending])


def cls(name):
    return message_factory.GetMessageClass(POOL.FindMessageTypeByName(name))


class ApiError(Exception):
    pass


class Client:
    def __init__(self, socket=SOCKET, name='com.oihdesigns.isolatedads1256', timeout_ms=600000):
        self.sock = pynng.Req0(dial='ipc://' + socket, recv_timeout=timeout_ms, send_timeout=10000)
        self.name = name
        self.token = ''

    def call(self, typ, msg=None, resp_json=True):
        Req = cls('kiapi.common.ApiRequest')
        Resp = cls('kiapi.common.ApiResponse')
        inner = cls(typ)()
        if isinstance(msg, dict):
            json_format.ParseDict(msg, inner)
        elif msg is not None:
            inner = msg
        req = Req()
        req.header.kicad_token = self.token
        req.header.client_name = self.name
        req.message.Pack(inner)
        self.sock.send(req.SerializeToString())
        resp = Resp()
        resp.ParseFromString(self.sock.recv())
        if resp.header.kicad_token:
            self.token = resp.header.kicad_token
        st = resp.status
        code = POOL.FindEnumTypeByName('kiapi.common.ApiStatusCode').values_by_number[st.status].name
        if code != 'AS_OK':
            raise ApiError('%s: %s' % (code, st.error_message))
        tname = resp.message.TypeName()
        if not tname:
            return {}
        out = cls(tname)()
        resp.message.Unpack(out)
        if not resp_json:
            return out
        return json_format.MessageToDict(out, preserving_proto_field_name=True)

    def unpack(self, anyd):
        """Turn a JSON Any dict ({'@type': ..., ...}) into a python dict (already is) -- helper for GetItems."""
        return anyd
