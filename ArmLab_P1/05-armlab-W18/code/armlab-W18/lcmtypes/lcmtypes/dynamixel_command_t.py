"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class dynamixel_command_t(object):
    __slots__ = ["utime", "position_radians", "speed", "max_torque"]

    def __init__(self):
        self.utime = 0
        self.position_radians = 0.0
        self.speed = 0.0
        self.max_torque = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(dynamixel_command_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qddd", self.utime, self.position_radians, self.speed, self.max_torque))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != dynamixel_command_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return dynamixel_command_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = dynamixel_command_t()
        self.utime, self.position_radians, self.speed, self.max_torque = struct.unpack(">qddd", buf.read(32))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if dynamixel_command_t in parents: return 0
        tmphash = (0x94bff3111878405e) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if dynamixel_command_t._packed_fingerprint is None:
            dynamixel_command_t._packed_fingerprint = struct.pack(">Q", dynamixel_command_t._get_hash_recursive([]))
        return dynamixel_command_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

