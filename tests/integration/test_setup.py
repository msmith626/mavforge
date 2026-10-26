"""setup_status end to end: API, CLI and MCP against the fake vehicle."""

from __future__ import annotations

import json
from typing import Any

import anyio
from mcp import Client
from typer.testing import CliRunner

import mavforge
from mavforge.cli.app import app
from mavforge.domain.models import ParamType, SetupStepStatus
from mavforge.domain.outcomes import OperationStatus
from mavforge.mcp.server import VehicleProvider, build_server
from mavforge.policy.permissions import Policy
from mavforge.services.session import ConnectOptions
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle, default_copter_params
from mavforge.testing.scenarios import unconfigured_plane_params
from mavforge.transport.dialect import mavlink

runner = CliRunner()


def _call_mcp(connection: str, tool: str) -> Any:
    options = ConnectOptions(connection=connection, heartbeat_timeout=1.0)
    policy = Policy.read_only()
    provider = VehicleProvider(options, policy)
    server = build_server(provider, policy)

    async def run() -> Any:
        async with Client(server) as client:
            return await client.call_tool(tool, {})

    try:
        return anyio.run(run)
    finally:
        provider.close()


UNVERIFIABLE_FROM_PARAMS = ("orientation", "gyro", "level", "ardupilot.flight_modes")
"""Always unknown, even on a fully configured vehicle: see services/setup.py."""


def test_fully_configured_vehicle_is_done_except_what_cant_be_known(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        report = vehicle.setup_status()
    by_step = {s.step: s.status for s in report.steps}
    for step in UNVERIFIABLE_FROM_PARAMS:
        assert by_step[step] is SetupStepStatus.UNKNOWN, step
    for step, status in by_step.items():
        if step not in UNVERIFIABLE_FROM_PARAMS:
            assert status is SetupStepStatus.DONE, step
    assert report.next_step is not None
    assert report.next_step.step == "orientation"  # first in sequence order
    assert report.status is OperationStatus.SUCCEEDED


def test_unconfigured_plane_shows_needed_and_recommended_steps(make_fake: FakeFactory) -> None:
    fake = make_fake(
        mav_type=mavlink.MAV_TYPE_FIXED_WING,
        params=unconfigured_plane_params(),
        prearm_checks_enabled=False,
    )
    with mavforge.connect(fake.connection_string) as vehicle:
        report = vehicle.setup_status()
    by_step = {s.step: s.status for s in report.steps}
    assert by_step["ardupilot.compass_cal"] is SetupStepStatus.NEEDED
    assert by_step["ardupilot.battery_failsafe"] is SetupStepStatus.RECOMMENDED
    assert by_step["ardupilot.frame_class"] is SetupStepStatus.NOT_APPLICABLE  # plane


def test_cli_setup_shows_the_checklist_and_next_step(fake: FakeVehicle) -> None:
    result = runner.invoke(
        app, ["-c", fake.connection_string, "setup"], env={"MAVFORGE_CONNECTION": None}
    )
    assert result.exit_code == 0, result.output
    assert "[done] Accelerometer calibration" in result.stdout
    assert "[unknown] Board orientation" in result.stdout
    assert "mavforge calibrate orientation" in result.stdout
    assert "Next: Board orientation" in result.stdout


def test_cli_setup_json(fake: FakeVehicle) -> None:
    result = runner.invoke(
        app,
        ["-c", fake.connection_string, "--json", "setup"],
        env={"MAVFORGE_CONNECTION": None},
    )
    assert result.exit_code == 0, result.output
    data = json.loads(result.stdout)
    assert data["next_step"]["step"] == "orientation"
    steps = {s["step"]: s["status"] for s in data["steps"]}
    assert steps["ardupilot.accel_cal"] == "done"


def test_mcp_setup_status_tool(fake: FakeVehicle) -> None:
    result = _call_mcp(fake.connection_string, "setup_status")
    assert not result.is_error
    steps = {s["step"]: s["status"] for s in result.structured_content["steps"]}
    assert steps["ardupilot.accel_cal"] == "done"
    assert steps["orientation"] == "unknown"
    assert result.structured_content["next_step"]["step"] == "orientation"


def test_fltmode_ch_disabled_is_suspected_not_done(make_fake: FakeFactory) -> None:
    fake = make_fake(params=default_copter_params() | {"FLTMODE_CH": (0, ParamType.INT8)})
    with mavforge.connect(fake.connection_string) as vehicle:
        report = vehicle.setup_status()
    step = next(s for s in report.steps if s.step == "ardupilot.flight_modes")
    assert step.status is SetupStepStatus.RECOMMENDED
    assert step.run == "mavforge calibrate flight-modes FLTMODEn=VALUE ..."


def test_fltmode_ch_enabled_is_unknown_not_done(fake: FakeVehicle) -> None:
    # Found on real hardware (2026-10-26): a non-zero FLTMODE_CH doesn't mean the
    # channel carries a real signal. The default fixture's FLTMODE_CH=5 is non-zero,
    # so this must stay unknown rather than a false 'done'.
    with mavforge.connect(fake.connection_string) as vehicle:
        report = vehicle.setup_status()
    step = next(s for s in report.steps if s.step == "ardupilot.flight_modes")
    assert step.status is SetupStepStatus.UNKNOWN
    assert step.finding is not None
    assert step.finding.kind.value == "passed"  # the underlying diagnose finding
    assert step.run == "mavforge calibrate flight-modes FLTMODEn=VALUE ..."
