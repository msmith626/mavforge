import math

import pytest

from mavforge.domain.models import ParamType
from mavforge.domain.values import (
    format_value,
    normalize,
    shortest_float32,
    to_float32,
    values_equal,
)


@pytest.mark.parametrize("decimal", [0.001, -12.5, 0.75, 0.3, 1e-7, 3.4e38, 149.165237])
def test_shortest_float32_recovers_the_decimal_a_user_typed(decimal: float) -> None:
    assert shortest_float32(to_float32(decimal)) == pytest.approx(decimal, rel=1e-7)
    assert len(repr(shortest_float32(to_float32(decimal)))) <= len(repr(decimal)) + 1


def test_shortest_float32_examples() -> None:
    assert to_float32(0.001) == 0.0010000000474974513
    assert shortest_float32(0.0010000000474974513) == 0.001
    assert shortest_float32(0.3) == 0.3


def test_normalize_by_type() -> None:
    assert normalize(33554432.0, ParamType.INT32) == 33554432
    assert isinstance(normalize(1.0, ParamType.INT8), int)
    assert normalize(0.0010000000474974513, ParamType.REAL32) == 0.001
    assert normalize(0.0010000000474974513, None) == 0.001


def test_values_equal_in_float32() -> None:
    assert values_equal(0.001, 0.0010000000474974513)
    assert values_equal(0.1, 0.10000000149011612, ParamType.REAL32)
    assert not values_equal(0.1, 0.1001)
    assert values_equal(33554433, 33554432)  # indistinguishable on the wire
    assert not values_equal(33554433, 33554432, ParamType.INT32)


def test_format_value() -> None:
    assert format_value(1) == "1"
    assert format_value(4.0) == "4"
    assert format_value(0.001) == "0.001"
    assert format_value(-12.5) == "-12.5"
    assert format_value(math.inf) == "inf"
