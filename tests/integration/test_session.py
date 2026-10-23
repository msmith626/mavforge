"""Connection, vehicle selection and identification against the fake vehicle."""

import pytest

import mavforge
from mavforge.domain.errors import NoVehicleError
from mavforge.domain.models import FirmwareReleaseType
from mavforge.testing.fake_vehicle import FakeFactory
from mavforge.transport.dialect import mavlink


def test_identifies_ardupilot_copter(make_fake: FakeFactory) -> None:
    fake = make_fake(firmware=(4, 6, 3, 128), git_hash="1234abcd")
    with mavforge.connect(fake.connection_string) as vehicle:
        ident = vehicle.identity
    assert (ident.system_id, ident.component_id) == (1, 1)
    assert ident.platform == "ardupilot"
    assert ident.vehicle_family == "copter"
    assert ident.firmware is not None
    assert str(ident.firmware) == "4.6.3-beta"
    assert ident.firmware.release_type is FirmwareReleaseType.BETA
    assert ident.firmware_git_hash == "1234abcd"
    assert "FTP" in ident.protocol_capabilities
    assert ident.simulated is True


def test_real_vehicle_is_not_reported_as_simulated(make_fake: FakeFactory) -> None:
    fake = make_fake(simulated=False)
    with mavforge.connect(fake.connection_string) as vehicle:
        assert vehicle.identity.simulated is False


def test_ignores_gcs_heartbeats_and_reports_other_vehicles(make_fake: FakeFactory) -> None:
    fake = make_fake(gcs_heartbeat_sysids=[255], other_autopilot_sysids=[7])
    with mavforge.connect(fake.connection_string) as vehicle:
        assert vehicle.identity.system_id == 1
        assert vehicle.identity.other_systems_seen == [7]


def test_target_system_pins_vehicle(make_fake: FakeFactory) -> None:
    fake = make_fake(other_autopilot_sysids=[7])
    with mavforge.connect(fake.connection_string, target_system=7) as vehicle:
        assert vehicle.identity.system_id == 7
        assert 1 in vehicle.identity.other_systems_seen


def test_missing_target_system_raises_no_vehicle(make_fake: FakeFactory) -> None:
    fake = make_fake()
    with pytest.raises(NoVehicleError) as info:
        mavforge.connect(fake.connection_string, target_system=42, timeout=0.8)
    assert info.value.details["autopilot_systems_seen"] == [1]


def test_silent_link_raises_no_vehicle(make_fake: FakeFactory) -> None:
    fake = make_fake(silent=True)
    with pytest.raises(NoVehicleError):
        mavforge.connect(fake.connection_string, timeout=0.5)


def test_unknown_autopilot_uses_generic_adapter(make_fake: FakeFactory) -> None:
    fake = make_fake(autopilot=mavlink.MAV_AUTOPILOT_PX4)
    with mavforge.connect(fake.connection_string) as vehicle:
        assert vehicle.identity.platform == "generic"
        assert vehicle.identity.simulated is None
        assert vehicle.status().mode is None


def test_tcp_link_loss_is_detected_not_silently_ignored(
    make_fake: FakeFactory, capfd: pytest.CaptureFixture[str]
) -> None:
    """Regression: pymavlink's TCP EOF handling printed to stdout forever and never
    reported the loss. The connection must close and waiters must get an error."""
    import time

    from mavforge.domain.errors import ConnectionFailedError

    fake = make_fake()
    with mavforge.connect(fake.connection_string) as vehicle:
        fake.stop()
        deadline = time.monotonic() + 3
        while vehicle.is_connected and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not vehicle.is_connected
        with pytest.raises(ConnectionFailedError):
            vehicle.params.get("FRAME_CLASS")
    assert "EOF on TCP socket" not in capfd.readouterr().out
