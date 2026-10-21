"""Read-only operations must never send anything that could change vehicle state."""

import mavforge
from mavforge.testing.fake_vehicle import FakeVehicle
from mavforge.transport.dialect import mavlink

READ_ONLY_COMMANDS = {
    mavlink.MAV_CMD_REQUEST_MESSAGE,
    mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
}
READ_ONLY_MESSAGES = {"COMMAND_LONG", "PARAM_REQUEST_LIST", "PARAM_REQUEST_READ"}


def test_read_operations_send_only_requests(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        vehicle.info()
        vehicle.status()
        vehicle.params.list()
        vehicle.params.get("FRAME_CLASS")
    assert fake.received_types() <= READ_ONLY_MESSAGES
    commands = {
        m.command for m in fake.received if isinstance(m, mavlink.MAVLink_command_long_message)
    }
    assert commands <= READ_ONLY_COMMANDS
    # MAVForge never heartbeats (see transport.connection), so it cannot mask a GCS failsafe
    assert "HEARTBEAT" not in fake.received_types()
