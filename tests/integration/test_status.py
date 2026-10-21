"""Status snapshots against the fake vehicle."""

import mavforge
from mavforge.domain.outcomes import OperationStatus
from mavforge.testing.fake_vehicle import FakeFactory
from mavforge.transport.dialect import mavlink


def test_status_reports_mode_sensors_battery_and_gps(make_fake: FakeFactory) -> None:
    fake = make_fake(custom_mode=5, statustext_on_connect=["PreArm: Need 3D Fix"])
    with mavforge.connect(fake.connection_string) as vehicle:
        report = vehicle.status()
    assert report.status is OperationStatus.SUCCEEDED
    assert report.mode == "LOITER"
    assert report.armed is False
    assert report.prearm_checks_passing is True
    assert {s.sensor for s in report.sensors} >= {"3D_GYRO", "3D_ACCEL", "3D_MAG", "GPS"}
    assert report.battery is not None
    assert report.battery.voltage_v == 12.6
    assert report.gps is not None
    assert report.gps.fix_type == "3D_FIX"
    assert [t.text for t in report.recent_statustext] == ["PreArm: Need 3D Fix"]


def test_status_reports_unhealthy_sensor_and_failing_prearm(make_fake: FakeFactory) -> None:
    fake = make_fake(
        sensors_unhealthy=mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG, prearm_passing=False, armed=True
    )
    with mavforge.connect(fake.connection_string) as vehicle:
        report = vehicle.status()
    mag = next(s for s in report.sensors if s.sensor == "3D_MAG")
    assert mag.enabled
    assert not mag.healthy
    assert report.prearm_checks_passing is False
    assert report.armed is True


def test_status_is_partial_when_a_message_never_arrives(make_fake: FakeFactory) -> None:
    fake = make_fake(unanswered_message_ids={mavlink.MAVLINK_MSG_ID_GPS_RAW_INT})
    with mavforge.connect(fake.connection_string) as vehicle:
        report = vehicle.status()
    assert report.status is OperationStatus.PARTIAL
    assert report.unavailable == ["GPS_RAW_INT"]
    assert report.gps is None
