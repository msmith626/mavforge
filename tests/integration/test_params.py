"""Parameter reads through the full stack against the fake vehicle."""

import pytest

import mavforge
from mavforge.domain.errors import InvalidInputError, ParamNotFoundError
from mavforge.domain.models import ParamType
from mavforge.domain.outcomes import OperationStatus
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle, default_copter_params


def test_list_returns_every_parameter_sorted(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.list()
    names = [p.name for p in result.params]
    assert result.status is OperationStatus.SUCCEEDED
    assert names == sorted(default_copter_params())
    assert result.total_on_vehicle == result.received == len(names)
    assert result.missing_indices == []


def test_list_recovers_dropped_parameters(make_fake: FakeFactory) -> None:
    fake = make_fake(drop_param_indices_once={0, 3, 10})
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.list()
    assert result.status is OperationStatus.SUCCEEDED
    assert result.received == len(default_copter_params())


def test_list_glob_is_case_insensitive(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.list("rc1_*")
    assert [p.name for p in result.params] == ["RC1_MAX", "RC1_MIN", "RC1_TRIM"]
    assert result.received == len(default_copter_params())


def test_get_decodes_types_and_flags_precision_loss(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        frame = vehicle.params.get("frame_class").param
        offs = vehicle.params.get("COMPASS_OFS_Y").param
        serial = vehicle.params.get("BRD_SERIAL_NUM").param
    assert (frame.name, frame.value, frame.type) == ("FRAME_CLASS", 1, ParamType.INT8)
    assert isinstance(frame.value, int)
    assert offs.value == pytest.approx(-12.5)
    assert offs.type is ParamType.REAL32
    assert serial.precision_limited
    assert serial.value == 33554432  # 33554433 does not survive float32 encoding


def test_get_unknown_parameter_raises_not_found(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle, pytest.raises(ParamNotFoundError):
        vehicle.params.get("NO_SUCH_PARAM")


def test_get_rejects_malformed_names_without_contacting_vehicle(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle, pytest.raises(InvalidInputError):
        vehicle.params.get("BAD;NAME")
    assert "PARAM_REQUEST_READ" not in fake.received_types()
