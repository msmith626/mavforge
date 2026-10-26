"""Decoder for ArduPilot's packed parameter file, ``@PARAM/param.pck`` over MAVFTP.

See https://mavlink.io/en/services/parameter_ftp.html. Pure decoding over an already
fully-reassembled file (MAVFTP chunking/retries are a transport concern, handled in
``protocol/ftp.py``); no I/O here. Unlike PARAM_VALUE's "C cast" float encoding,
integer entries here are stored as genuine binary ints, so there is no
``precision_limited`` concern for values decoded by this module.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from mavforge.domain.errors import ProtocolError
from mavforge.domain.models import ParamType, ParamValue
from mavforge.domain.values import normalize

_MAGIC_NO_DEFAULTS = 0x671B
_MAGIC_WITH_DEFAULTS = 0x671C
_MAX_NAME_LEN = 16

# ArduPilot's internal type codes for this format - not MAV_PARAM_TYPE.
_TYPES: dict[int, tuple[ParamType, int, str]] = {
    1: (ParamType.INT8, 1, "<b"),
    2: (ParamType.INT16, 2, "<h"),
    3: (ParamType.INT32, 4, "<i"),
    4: (ParamType.REAL32, 4, "<f"),
}


@dataclass
class ParamPack:
    total_params: int
    """The component's total parameter count, from the file header."""
    params: list[ParamValue]


def decode(data: bytes) -> ParamPack:
    """Decode a fully-reassembled param.pck file. Raises ``ProtocolError`` on anything
    that doesn't match the spec - this is vehicle-supplied binary data, never trusted
    blindly."""
    if len(data) < 6:
        raise ProtocolError(f"param.pck too short ({len(data)} bytes; needs a 6-byte header)")
    magic, num_params, total_params = struct.unpack_from("<HHH", data, 0)
    if magic not in (_MAGIC_NO_DEFAULTS, _MAGIC_WITH_DEFAULTS):
        raise ProtocolError(f"param.pck has an unrecognised magic 0x{magic:04x}")
    params: list[ParamValue] = []
    offset = 6
    name = ""
    while len(params) < num_params:
        if offset >= len(data):
            raise ProtocolError(f"param.pck ended after {len(params)} of {num_params} entries")
        if data[offset] == 0:  # a zero *first byte* is padding, skip one at a time
            offset += 1
            continue
        if offset + 1 >= len(data):
            raise ProtocolError("param.pck ended mid-entry (name header)")
        type_byte, name_byte = data[offset], data[offset + 1]
        type_code, flags = type_byte & 0x0F, type_byte >> 4
        if type_code not in _TYPES:
            raise ProtocolError(f"param.pck entry has an unknown type code {type_code}")
        ptype, type_len, fmt = _TYPES[type_code]
        common_len, name_len = name_byte & 0x0F, (name_byte >> 4) + 1
        if common_len + name_len > _MAX_NAME_LEN:
            raise ProtocolError(
                f"param.pck entry's name exceeds {_MAX_NAME_LEN} characters "
                f"(common_len={common_len}, name_len={name_len})"
            )
        if common_len > len(name):
            raise ProtocolError(
                "param.pck entry's common_len exceeds the previous entry's name length"
            )
        name_start = offset + 2
        name_end = name_start + name_len
        value_end = name_end + type_len
        end = value_end + (type_len if flags & 1 else 0)  # flags bit 0: a default follows
        if end > len(data):
            raise ProtocolError("param.pck ended mid-entry (value or default)")
        try:
            suffix = data[name_start:name_end].decode("ascii")
        except UnicodeDecodeError as exc:
            raise ProtocolError("param.pck entry name is not ASCII") from exc
        name = name[:common_len] + suffix
        (raw,) = struct.unpack_from(fmt, data, name_end)
        params.append(
            ParamValue(name=name, value=normalize(float(raw), ptype), type=ptype, index=len(params))
        )
        offset = end
    return ParamPack(total_params=total_params, params=params)
