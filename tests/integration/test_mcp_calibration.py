"""MCP calibration tools: registration, approval, polling and the accel exclusion."""

import time
from typing import Any

import anyio
import pytest
from mcp import Client
from mcp_types import ElicitResult

import mavforge
from mavforge.mcp.server import VehicleProvider, build_server
from mavforge.policy.permissions import Permission, Policy
from mavforge.services.session import ConnectOptions
from mavforge.testing.fake_calibration import FakeCalibrationConfig
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle
from mavforge.transport.dialect import mavlink

CAL_TOOLS = {"calibration_request", "calibration_start", "calibration_status", "calibration_cancel"}
CAL = Policy(granted=frozenset({Permission.READ, Permission.CALIBRATE}))


def run(fake: FakeVehicle, policy: Policy, steps: Any, elicitation: Any = None) -> Any:
    provider = VehicleProvider(ConnectOptions(connection=fake.connection_string), policy)
    server = build_server(provider, policy)

    async def go() -> Any:
        kwargs: dict[str, Any] = {"elicitation_callback": elicitation} if elicitation else {}
        async with Client(server, **kwargs) as client:
            return await steps(client)

    try:
        return anyio.run(go)
    finally:
        provider.close()


async def accept(ctx: Any, params: Any) -> ElicitResult:
    return ElicitResult(action="accept", content={"approve": True})


async def _poll(client: Client, op: str) -> Any:
    for _ in range(200):
        st = (await client.call_tool("calibration_status", {"operation_id": op})).structured_content
        if st["state"] in {"succeeded", "failed", "cancelled", "unknown"}:
            return st
        await anyio.sleep(0.05)
    raise AssertionError("calibration did not finish")


def test_tools_registered_only_with_allow_calibrate(fake: FakeVehicle) -> None:
    async def names(client: Client) -> set[str]:
        return {t.name for t in (await client.list_tools()).tools}

    assert not run(fake, Policy.read_only(), names) & CAL_TOOLS
    assert "calibration_procedures" in run(fake, Policy.read_only(), names)
    assert run(fake, CAL, names) >= CAL_TOOLS


def test_compass_calibration_end_to_end_over_mcp(make_fake: FakeFactory) -> None:
    fake = make_fake(calibration=FakeCalibrationConfig(mag_duration_s=0.5))
    shown: list[str] = []

    async def approve(ctx: Any, params: Any) -> ElicitResult:
        shown.append(params.message)
        return ElicitResult(action="accept", content={"approve": True})

    async def steps(client: Client) -> Any:
        req = (
            await client.call_tool("calibration_request", {"kind": "compass"})
        ).structured_content
        started = await client.call_tool("calibration_start", {"request_id": req["plan_id"]})
        assert not started.is_error, started.content
        return await _poll(client, started.structured_content["operation_id"])

    final = run(fake, CAL, steps, approve)
    assert final["state"] == "succeeded"
    assert final["result"]["reboot_required"] is True
    assert "rotate" in shown[0].lower() or "Rotate" in shown[0]


def test_accel_is_refused_over_mcp(fake: FakeVehicle) -> None:
    async def steps(client: Client) -> Any:
        return await client.call_tool("calibration_request", {"kind": "accel"})

    result = run(fake, CAL, steps, accept)
    assert result.is_error
    assert "mavforge calibrate accel" in result.content[0].text


@pytest.mark.parametrize("kind", ["radio", "orientation"])
def test_person_guided_checks_are_refused_over_mcp(fake: FakeVehicle, kind: str) -> None:
    async def steps(client: Client) -> Any:
        return await client.call_tool("calibration_request", {"kind": kind})

    result = run(fake, CAL, steps, accept)
    assert result.is_error
    assert f"mavforge calibrate {kind}" in result.content[0].text


def test_accel_request_made_elsewhere_is_still_refused_over_mcp(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        request = vehicle.calibration.request("accel")
    mavforge.approve_plan(request)

    async def steps(client: Client) -> Any:
        return await client.call_tool("calibration_start", {"request_id": request.plan_id})

    result = run(fake, CAL, steps, accept)
    assert result.is_error  # enforced in the service, not only by the tool list
    assert "needs the person at the vehicle" in result.content[0].text


def test_no_elicitation_requires_terminal_approval(fake: FakeVehicle) -> None:
    async def steps(client: Client) -> Any:
        req = (await client.call_tool("calibration_request", {"kind": "gyro"})).structured_content
        first = await client.call_tool("calibration_start", {"request_id": req["plan_id"]})
        forged = await client.call_tool(
            "calibration_start", {"request_id": req["plan_id"], "approval": {"approve": True}}
        )
        mavforge.approve_plan(req["plan_id"], approved_by="cli-prompt")  # the human
        second = await client.call_tool("calibration_start", {"request_id": req["plan_id"]})
        return first, forged, second, await _poll(client, second.structured_content["operation_id"])

    first, forged, second, final = run(fake, CAL, steps)
    assert first.is_error
    assert "approval_required" in first.content[0].text
    assert forged.is_error
    assert not second.is_error
    assert final["state"] == "succeeded"


def test_cancel_over_mcp(make_fake: FakeFactory) -> None:
    fake = make_fake(calibration=FakeCalibrationConfig(mag_duration_s=30))

    async def steps(client: Client) -> Any:
        req = (
            await client.call_tool("calibration_request", {"kind": "compass"})
        ).structured_content
        started = await client.call_tool("calibration_start", {"request_id": req["plan_id"]})
        await anyio.sleep(0.5)
        op = started.structured_content["operation_id"]
        return await client.call_tool("calibration_cancel", {"operation_id": op})

    cancelled = run(fake, CAL, steps, accept).structured_content
    assert cancelled["state"] == "cancelled"
    time.sleep(0.2)
    assert any(
        isinstance(m, mavlink.MAVLink_command_long_message)
        and m.command == mavlink.MAV_CMD_DO_CANCEL_MAG_CAL
        for m in fake.received
    )
