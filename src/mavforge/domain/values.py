"""Parameter value arithmetic for MAVLink's float32 transport.

Every ArduPilot parameter travels as a float32, so a value read as ``0.001`` arrives as
``0.0010000000474974513``. Values are normalised to the shortest decimal that maps to
the same float32, and compared in float32, so backups stay readable and diffs do not
report differences that the vehicle cannot represent.
"""

from __future__ import annotations

import math
import struct

from mavforge.domain.models import ParamType


def _f32_bits(value: float) -> bytes:
    return struct.pack("<f", value)


def to_float32(value: float) -> float:
    """``value`` rounded to the nearest float32 (non-finite values pass through)."""
    if not math.isfinite(value):
        return value
    try:
        return struct.unpack("<f", _f32_bits(value))[0]
    except OverflowError:
        return math.copysign(math.inf, value)


def shortest_float32(value: float) -> float:
    """The shortest decimal that represents the same float32 as ``value``."""
    f32 = to_float32(value)
    if not math.isfinite(f32):
        return f32
    bits = _f32_bits(f32)
    for digits in range(1, 10):
        candidate = float(f"{f32:.{digits}g}")
        if _f32_bits(candidate) == bits:
            return candidate
    return f32


def normalize(value: float, ptype: ParamType | None) -> float | int:
    """Canonical form of a parameter value: int for integer types, else shortest float32."""
    if ptype is not None and ptype.is_integer:
        return round(value)
    return shortest_float32(value)


def values_equal(a: float, b: float, ptype: ParamType | None = None) -> bool:
    """Equality as the vehicle sees it: exact for integer types, float32 otherwise."""
    if ptype is not None and ptype.is_integer:
        return round(a) == round(b)
    return to_float32(a) == to_float32(b)


def format_value(value: float | int) -> str:
    """Text for files and terminals: integral values without a decimal point."""
    number = float(value)
    if math.isfinite(number) and number.is_integer() and abs(number) < 1e15:
        return str(int(number))
    return repr(number)
