"""MAVLink FTP param.pck download against real ArduCopter SITL. Read-only on the
vehicle. This is the authoritative correctness check for this binary format: the fake
vehicle's FTP server is a test double, built from the same spec this decodes against,
but real firmware is the thing that actually has to agree with it.
"""

from __future__ import annotations

import pytest

import mavforge
from mavforge.protocol import ftp, param_pack
from mavforge.protocol import params as param_protocol

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(240)]

BOOT_TIMEOUT = 60.0


def test_real_firmware_reports_ftp_capability(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        assert "FTP" in vehicle.identity.protocol_capabilities


def test_param_pck_matches_param_request_list(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        session = vehicle.session
        data = ftp.fetch_file(session.connection, *session.target, b"@PARAM/param.pck")
        via_list = param_protocol.fetch_all(session.connection, *session.target)
        is_volatile = session.adapter.is_volatile_param

    pack = param_pack.decode(data)
    assert pack.total_params > 500  # same ballpark as test_sitl_backup.py's own check

    via_ftp = {p.name: p.value for p in pack.params}
    via_list_values = {p.name: p.value for p in via_list.params.values()}
    assert via_ftp.keys() == via_list_values.keys()

    # Two reads, seconds apart, over two different protocols: anything that differs
    # must be something the firmware changes on its own, not a decoding mistake.
    differing = [n for n in via_ftp if via_ftp[n] != via_list_values[n]]
    assert all(is_volatile(n) for n in differing), differing


def test_backup_against_sitl_uses_ftp_not_the_fallback(
    sitl_connection: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Confirms the integration, not just the two pieces separately: params.backup()
    against real firmware actually takes the FTP path and never falls back, proven by
    making the fallback explode if it's ever called."""

    def fallback_must_not_be_called(*args: object, **kwargs: object) -> None:
        raise AssertionError(
            "fetch_all_params fell back to PARAM_REQUEST_LIST against "
            "real SITL, which reports the FTP capability"
        )

    monkeypatch.setattr(param_protocol, "fetch_all", fallback_must_not_be_called)
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        backup = vehicle.params.backup()
    assert backup.param_count > 500
