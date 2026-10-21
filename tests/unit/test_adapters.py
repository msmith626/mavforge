import pytest

from mavforge.domain.errors import InvalidInputError
from mavforge.platforms.ardupilot.adapter import ArduPilotAdapter
from mavforge.platforms.base import select_adapter
from mavforge.platforms.generic import GenericAdapter
from mavforge.transport.dialect import mavlink


def _hb(autopilot: int, mav_type: int = mavlink.MAV_TYPE_QUADROTOR, mode: int = 5):
    return mavlink.MAVLink_heartbeat_message(
        mav_type, autopilot, mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 3, 3
    )


def test_selects_ardupilot_adapter_from_heartbeat() -> None:
    assert isinstance(select_adapter(_hb(mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)), ArduPilotAdapter)
    assert isinstance(select_adapter(_hb(mavlink.MAV_AUTOPILOT_PX4)), GenericAdapter)


def test_ardupilot_mode_and_family() -> None:
    adapter = ArduPilotAdapter()
    assert adapter.mode_name(_hb(mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, mode=5)) == "LOITER"
    plane = _hb(mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, mavlink.MAV_TYPE_FIXED_WING, mode=10)
    assert adapter.mode_name(plane) == "AUTO"
    assert adapter.vehicle_family(mavlink.MAV_TYPE_FIXED_WING) == "plane"
    assert adapter.vehicle_family(mavlink.MAV_TYPE_HELICOPTER) == "heli"


def test_param_name_normalisation() -> None:
    assert ArduPilotAdapter().normalize_param_name(" compass_use ") == "COMPASS_USE"
    assert GenericAdapter().normalize_param_name("sys_autostart") == "sys_autostart"
    for bad in ("", "A" * 17, "BAD-NAME", "X;Y"):
        with pytest.raises(InvalidInputError):
            ArduPilotAdapter().normalize_param_name(bad)
