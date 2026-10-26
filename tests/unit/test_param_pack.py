"""Decoder for @PARAM/param.pck (see https://mavlink.io/en/services/parameter_ftp.html).

Byte sequences are built by hand from the spec, independent of the decoder's own logic.
"""

import struct

import pytest

from mavforge.domain.errors import ProtocolError
from mavforge.domain.models import ParamType
from mavforge.protocol import param_pack

_TYPE_CODES = {ParamType.INT8: 1, ParamType.INT16: 2, ParamType.INT32: 3, ParamType.REAL32: 4}
_STRUCT_FMT = {
    ParamType.INT8: "<b",
    ParamType.INT16: "<h",
    ParamType.INT32: "<i",
    ParamType.REAL32: "<f",
}


def _entry(
    ptype: ParamType,
    suffix: str,
    value: float | int,
    *,
    common_len: int = 0,
    default: float | int | None = None,
) -> bytes:
    """One entry, exactly per spec. ``suffix`` is the non-shared part of the name, after
    ``common_len`` characters of the previous entry's name are assumed shared."""
    flags = 1 if default is not None else 0
    type_byte = _TYPE_CODES[ptype] | (flags << 4)
    name_byte = common_len | ((len(suffix) - 1) << 4)
    out = (
        bytes([type_byte, name_byte])
        + suffix.encode("ascii")
        + struct.pack(_STRUCT_FMT[ptype], value)
    )
    if default is not None:
        out += struct.pack(_STRUCT_FMT[ptype], default)
    return out


def _pck(num_params: int, total_params: int, *entries: bytes, with_defaults: bool = False) -> bytes:
    magic = 0x671C if with_defaults else 0x671B
    return struct.pack("<HHH", magic, num_params, total_params) + b"".join(entries)


def test_decodes_simple_entries() -> None:
    data = _pck(
        2, 2, _entry(ParamType.INT8, "FRAME_CLASS", 1), _entry(ParamType.INT8, "FRAME_TYPE", 0)
    )
    result = param_pack.decode(data)
    assert result.total_params == 2
    assert [(p.name, p.value, p.type) for p in result.params] == [
        ("FRAME_CLASS", 1, ParamType.INT8),
        ("FRAME_TYPE", 0, ParamType.INT8),
    ]
    assert result.defaults == {}  # plain param.pck: no entry ever carries a default
    assert [p.index for p in result.params] == [0, 1]


def test_name_compression() -> None:
    # "FRAME_TYPE" shares "FRAME_" (6 chars) with the previous "FRAME_CLASS".
    data = _pck(
        2,
        2,
        _entry(ParamType.INT8, "FRAME_CLASS", 1),
        _entry(ParamType.INT8, "TYPE", 0, common_len=6),
    )
    result = param_pack.decode(data)
    assert [p.name for p in result.params] == ["FRAME_CLASS", "FRAME_TYPE"]


def test_all_four_types() -> None:
    data = _pck(
        4,
        4,
        _entry(ParamType.INT8, "A", -5),
        _entry(ParamType.INT16, "B", 12345, common_len=0),
        _entry(ParamType.INT32, "C", 70000, common_len=0),
        _entry(ParamType.REAL32, "D", 3.5, common_len=0),
    )
    result = param_pack.decode(data)
    assert [(p.name, p.value, p.type) for p in result.params] == [
        ("A", -5, ParamType.INT8),
        ("B", 12345, ParamType.INT16),
        ("C", 70000, ParamType.INT32),
        ("D", 3.5, ParamType.REAL32),
    ]


def test_padding_bytes_between_entries_are_skipped() -> None:
    data = _pck(
        2, 2, _entry(ParamType.INT8, "A", 1) + b"\x00\x00\x00", _entry(ParamType.INT8, "B", 2)
    )
    result = param_pack.decode(data)
    assert [p.name for p in result.params] == ["A", "B"]


def test_flagged_entries_expose_their_default() -> None:
    # withdefaults=1: a default value follows each flagged entry. Must also skip past
    # the default bytes correctly for *unflagged* entries after it, or they misparse.
    data = _pck(
        2,
        2,
        _entry(ParamType.INT8, "A", 5, default=0),
        _entry(ParamType.INT8, "B", 9),
        with_defaults=True,
    )
    result = param_pack.decode(data)
    assert [(p.name, p.value) for p in result.params] == [("A", 5), ("B", 9)]
    assert result.defaults == {"A": 0}  # B carried no default: it already equals it


def test_default_decoding_follows_the_entry_s_own_type() -> None:
    data = _pck(
        2,
        2,
        _entry(ParamType.REAL32, "ARSPD_FBW_MIN", 18.5, default=16.0),
        _entry(ParamType.INT32, "BATT_CAPACITY", 4000, default=3300),
        with_defaults=True,
    )
    result = param_pack.decode(data)
    assert result.defaults == {"ARSPD_FBW_MIN": 16.0, "BATT_CAPACITY": 3300}


def test_rejects_unknown_magic() -> None:
    data = struct.pack("<HHH", 0x1234, 0, 0)
    with pytest.raises(ProtocolError, match="magic"):
        param_pack.decode(data)


def test_rejects_unknown_type_code() -> None:
    data = struct.pack("<HHH", 0x671B, 1, 1) + bytes([0x09, 0x00]) + b"A" + b"\x00"
    with pytest.raises(ProtocolError, match="type code"):
        param_pack.decode(data)


def test_rejects_name_over_16_chars() -> None:
    # First entry uses all 16 slots; a second entry sharing 15 of them plus a 2-char
    # suffix would make a 17-character name, which the spec forbids.
    data = _pck(
        2,
        2,
        _entry(ParamType.INT8, "A" * 16, 1),
        _entry(ParamType.INT8, "BB", 2, common_len=15),
    )
    with pytest.raises(ProtocolError, match="exceeds"):
        param_pack.decode(data)


def test_rejects_common_len_exceeding_previous_name() -> None:
    data = _pck(1, 1, _entry(ParamType.INT8, "AB", 1, common_len=5))
    with pytest.raises(ProtocolError, match="common_len"):
        param_pack.decode(data)


def test_rejects_truncated_header() -> None:
    with pytest.raises(ProtocolError, match="too short"):
        param_pack.decode(b"\x1b\x67\x00")


def test_rejects_truncated_entry() -> None:
    data = _pck(1, 1, _entry(ParamType.INT8, "AB", 1))[:-1]
    with pytest.raises(ProtocolError):
        param_pack.decode(data)


def test_rejects_non_ascii_name() -> None:
    data = struct.pack("<HHH", 0x671B, 1, 1) + bytes([0x01, 0x00, 0x80]) + b"\x00"
    with pytest.raises(ProtocolError, match="ASCII"):
        param_pack.decode(data)
