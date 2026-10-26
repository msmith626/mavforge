"""MAVLink FTP client against the fake vehicle's FTP server."""

import pytest

import mavforge
from mavforge.domain.errors import OperationTimeoutError, ProtocolError
from mavforge.protocol import ftp, param_pack
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle, default_copter_params
from mavforge.transport.dialect import mavlink


def _param_pck_messages(fake: FakeVehicle) -> list[mavlink.MAVLink_file_transfer_protocol_message]:
    return [
        m for m in fake.received if isinstance(m, mavlink.MAVLink_file_transfer_protocol_message)
    ]


def test_fetch_file_downloads_and_decodes_param_pck(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        data = ftp.fetch_file(
            vehicle.session.connection, *vehicle.session.target, b"@PARAM/param.pck"
        )
    pack = param_pack.decode(data)
    live = dict(default_copter_params())
    assert pack.total_params == len(live)
    assert {p.name: p.value for p in pack.params} == {
        name: value for name, (value, _) in live.items()
    }


def test_fetch_file_needs_more_than_one_read_chunk(fake: FakeVehicle) -> None:
    # The default parameter set encodes to well over 239 bytes (one ReadFile's worth),
    # so a correct client must issue multiple sequential reads, not just one.
    with mavforge.connect(fake.connection_string) as vehicle:
        ftp.fetch_file(vehicle.session.connection, *vehicle.session.target, b"@PARAM/param.pck")
    reads = [
        m
        for m in _param_pck_messages(fake)
        if ftp.decode_frame(bytes(m.payload)).opcode == ftp.OP_READ_FILE
    ]
    assert len(reads) > 1


def test_fetch_file_closes_the_session(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        ftp.fetch_file(vehicle.session.connection, *vehicle.session.target, b"@PARAM/param.pck")
    opcodes = [ftp.decode_frame(bytes(m.payload)).opcode for m in _param_pck_messages(fake)]
    assert ftp.OP_TERMINATE_SESSION in opcodes


def test_fetch_file_unknown_path_is_a_protocol_error(fake: FakeVehicle) -> None:
    with (
        mavforge.connect(fake.connection_string) as vehicle,
        pytest.raises(ProtocolError, match="OpenFileRO"),
    ):
        ftp.fetch_file(vehicle.session.connection, *vehicle.session.target, b"@MISSION/nope.dat")


def test_fetch_file_server_side_failure_is_a_protocol_error(make_fake: FakeFactory) -> None:
    fake = make_fake(ftp_param_pck_fails=True)
    with (
        mavforge.connect(fake.connection_string) as vehicle,
        pytest.raises(ProtocolError, match="OpenFileRO"),
    ):
        ftp.fetch_file(vehicle.session.connection, *vehicle.session.target, b"@PARAM/param.pck")


def test_fetch_file_times_out_cleanly_when_the_server_never_replies(
    make_fake: FakeFactory,
) -> None:
    fake = make_fake(ftp_enabled=False)
    with (
        mavforge.connect(fake.connection_string) as vehicle,
        pytest.raises(OperationTimeoutError),
    ):
        ftp.fetch_file(vehicle.session.connection, *vehicle.session.target, b"@PARAM/param.pck")
