"""Comparing the vehicle's live parameters against the firmware's own shipped
defaults (``params diff-defaults`` / MAVFTP ``param.pck?withdefaults=1``)."""

import pytest

import mavforge
from mavforge.domain.errors import UnsupportedError
from mavforge.domain.models import ParamType
from mavforge.domain.outcomes import OperationStatus
from mavforge.testing.fake_vehicle import FakeFactory, default_copter_params


def test_finds_changed_params_and_counts_the_rest_unchanged(make_fake: FakeFactory) -> None:
    fake = make_fake(firmware_defaults={"BATT_CAPACITY": 3400})  # live value is 3300
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.diff_defaults()
    assert result.status is OperationStatus.SUCCEEDED
    assert result.reference == "firmware defaults"
    assert [(c.name, c.reference, c.current) for c in result.changed] == [
        ("BATT_CAPACITY", 3400, 3300)
    ]
    assert result.unchanged == len(default_copter_params()) - 1
    assert result.only_in_reference == []
    assert result.only_in_current == []


def test_nothing_recorded_as_different_means_no_changes(make_fake: FakeFactory) -> None:
    fake = make_fake()  # no firmware_defaults configured at all
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.diff_defaults()
    assert result.changed == []
    assert result.unchanged == len(default_copter_params())


def test_changed_value_equal_to_default_is_not_reported(make_fake: FakeFactory) -> None:
    # Real ArduPilot only attaches a default when it differs from the live value;
    # listing a name with the same value it already has must behave identically to
    # not listing it at all.
    fake = make_fake(firmware_defaults={"BATT_CAPACITY": 3300})  # equals the live value
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.diff_defaults()
    assert result.changed == []


def test_volatile_changes_are_marked(make_fake: FakeFactory) -> None:
    fake = make_fake(
        params=default_copter_params() | {"STAT_BOOTCNT": (3, ParamType.INT32)},
        firmware_defaults={"STAT_BOOTCNT": 0},
    )
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.diff_defaults()
    (change,) = result.changed
    assert change.name == "STAT_BOOTCNT"
    assert change.volatile is True


def test_without_ftp_support_raises_unsupported(make_fake: FakeFactory) -> None:
    fake = make_fake(ftp_enabled=False)
    with (
        mavforge.connect(fake.connection_string) as vehicle,
        pytest.raises(UnsupportedError, match="MAVFTP"),
    ):
        vehicle.params.diff_defaults()
