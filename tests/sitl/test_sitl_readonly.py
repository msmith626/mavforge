"""Read-only workflows against real ArduCopter SITL (not a fake).

These verify MAVForge against actual ArduPilot firmware behaviour. They make no
changes to the simulated vehicle.
"""

from __future__ import annotations

import json

import pytest
from typer.testing import CliRunner

import mavforge
from mavforge.cli.app import app
from mavforge.domain.errors import ParamNotFoundError
from mavforge.domain.models import ParamType
from mavforge.domain.outcomes import OperationStatus

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(180)]

BOOT_TIMEOUT = 60.0


def test_identifies_arducopter_sitl(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        ident = vehicle.identity
    assert ident.autopilot == "ARDUPILOTMEGA"
    assert ident.platform == "ardupilot"
    assert ident.vehicle_family == "copter"
    assert ident.simulated is True
    assert ident.firmware is not None
    assert (ident.firmware.major, ident.firmware.minor) >= (4, 0)
    assert "PARAM_FLOAT" in ident.protocol_capabilities
    assert "FTP" in ident.protocol_capabilities


def test_status_snapshot(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        report = vehicle.status()
    assert report.status is OperationStatus.SUCCEEDED
    assert report.armed is False
    assert report.mode == "STABILIZE"
    assert {"3D_GYRO", "3D_ACCEL", "3D_MAG"} <= {s.sensor for s in report.sensors}
    assert report.battery is not None


def test_full_parameter_download(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        result = vehicle.params.list()
    assert result.status is OperationStatus.SUCCEEDED
    assert result.total_on_vehicle is not None
    assert result.total_on_vehicle > 500
    assert result.received == result.total_on_vehicle == len(result.params)
    assert result.missing_indices == []
    names = {p.name for p in result.params}
    assert {"FRAME_CLASS", "COMPASS_USE"} <= names
    # ArduPilot 4.7 renamed SYSID_THISMAV to MAV_SYSID; names are firmware-specific.
    assert "MAV_SYSID" in names or "SYSID_THISMAV" in names


def test_param_get_types_and_defaults(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        sysid = vehicle.params.get("mav_sysid").param  # SYSID_THISMAV before 4.7
        frame_type = vehicle.params.get("FRAME_TYPE").param
    assert sysid.value == 1
    assert sysid.type.is_integer
    assert frame_type.value == 0  # from copter.parm defaults
    assert frame_type.type in {ParamType.INT8, ParamType.INT16}


def test_unknown_parameter_is_reported_as_not_found(sitl_connection: str) -> None:
    """ArduPilot >= 4.7 answers PARAM_REQUEST_READ for unknown names with PARAM_ERROR."""
    with (
        mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle,
        pytest.raises(ParamNotFoundError),
    ):
        vehicle.params.get("NO_SUCH_PARAM")


def test_cli_against_sitl(sitl_connection: str) -> None:
    result = CliRunner().invoke(
        app,
        [
            "-c",
            sitl_connection,
            "--timeout",
            str(BOOT_TIMEOUT),
            "--json",
            "params",
            "get",
            "FRAME_CLASS",
        ],
    )
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout)["param"]["value"] == 1


def test_mcp_stdio_against_sitl(sitl_connection: str) -> None:
    import os
    import sys

    import anyio
    from mcp import Client, StdioServerParameters

    params = StdioServerParameters(
        command=sys.executable,
        args=["-m", "mavforge.mcp", "-c", sitl_connection, "--timeout", str(BOOT_TIMEOUT)],
        env={**os.environ},
    )

    async def run() -> tuple[object, object]:
        async with Client(params) as client:
            info = await client.call_tool("vehicle_info", {})
            param = await client.call_tool("params_get", {"name": "FRAME_CLASS"})
            return info, param

    info, param = anyio.run(run)
    assert info.structured_content["vehicle"]["simulated"] is True  # type: ignore[attr-defined]
    assert param.structured_content["param"]["value"] == 1  # type: ignore[attr-defined]
