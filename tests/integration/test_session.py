"""Connection, vehicle selection and identification against the fake vehicle."""

import pytest

import mavforge
from mavforge.domain.errors import NotPermittedError, NoVehicleError
from mavforge.domain.models import FirmwareReleaseType
from mavforge.policy.permissions import Policy
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


def test_falls_back_to_a_non_autopilot1_component(make_fake: FakeFactory) -> None:
    """wait_for_vehicle prefers MAV_COMP_ID_AUTOPILOT1, but a vehicle that only ever
    announces a different component must still be usable, not time out."""
    fake = make_fake(component_id=2)
    with mavforge.connect(fake.connection_string) as vehicle:
        assert vehicle.identity.component_id == 2
        assert vehicle.identity.system_id == 1


def test_gcs_and_multiple_other_vehicles_together(make_fake: FakeFactory) -> None:
    """The two existing scenarios (GCS heartbeats ignored; other vehicles reported)
    combined, and with more than one 'other' system, not just one."""
    fake = make_fake(gcs_heartbeat_sysids=[255, 190], other_autopilot_sysids=[7, 9])
    with mavforge.connect(fake.connection_string) as vehicle:
        assert vehicle.identity.system_id == 1  # the GCSes are never candidates
        assert sorted(vehicle.identity.other_systems_seen) == [7, 9]


def test_a_plan_made_for_one_vehicle_is_refused_against_another_on_the_same_link(
    make_fake: FakeFactory,
) -> None:
    """Risk register #2: several real vehicles on one link. A plan made for the one
    selection picked must not be applicable just because some vehicle answers - it
    must be *that* vehicle, confirmed by identity, not merely 'a' vehicle on this link.

    allow_real_vehicle=True here isn't about real hardware (this is still entirely the
    fake vehicle): the 'other' system is a lightweight stub that can't answer
    AUTOPILOT_VERSION, so it comes back as not-confirmed-simulated, and the real-vehicle
    gate would otherwise fire first and mask the identity check this test is actually
    about. That gate has its own dedicated coverage elsewhere.
    """
    fake = make_fake(other_autopilot_sysids=[7])
    rw = Policy.configure(allow_real_vehicle=True)
    with mavforge.connect(fake.connection_string, target_system=1, policy=rw) as vehicle:
        plan = vehicle.params.plan({"RC1_TRIM": 1490})
        mavforge.approve_plan(plan)
    with (
        mavforge.connect(fake.connection_string, target_system=7, policy=rw) as vehicle,
        pytest.raises(NotPermittedError, match="different vehicle"),
    ):
        vehicle.params.apply(plan)


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
