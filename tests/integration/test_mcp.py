"""MCP server tools, in-process and over a real stdio subprocess."""

import json
import os
import sys
from typing import Any

import anyio
import pytest
from mcp import Client, StdioServerParameters

from mavforge.domain.models import ParamType
from mavforge.mcp.server import VehicleProvider, build_server
from mavforge.policy.permissions import Policy
from mavforge.services.session import ConnectOptions
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle, default_copter_params

TOOLS = {
    "mavforge_capabilities",
    "vehicle_info",
    "vehicle_status",
    "params_list",
    "params_get",
    "params_backup",
    "params_backups",
    "params_diff",
    "diagnose",
    "setup_status",
    "calibration_procedures",
}
CREATES_FILES = {"params_backup"}


def _server(connection: str | None) -> tuple[Any, VehicleProvider]:
    policy = Policy.read_only()
    options = ConnectOptions(connection=connection, heartbeat_timeout=1.0) if connection else None
    provider = VehicleProvider(options, policy)
    return build_server(provider, policy), provider


def call(connection: str | None, tool: str, args: dict[str, Any] | None = None) -> Any:
    server, provider = _server(connection)

    async def run() -> Any:
        async with Client(server) as client:
            return await client.call_tool(tool, args or {})

    try:
        return anyio.run(run)
    finally:
        provider.close()


def test_tools_are_listed_with_read_only_annotations_and_schemas() -> None:
    server, provider = _server(None)

    async def run() -> Any:
        async with Client(server) as client:
            return await client.list_tools()

    tools = anyio.run(run).tools
    provider.close()
    assert {t.name for t in tools} == TOOLS
    for tool in tools:
        assert tool.annotations is not None
        assert tool.annotations.read_only_hint is (tool.name not in CREATES_FILES)
        assert tool.annotations.destructive_hint is False
        assert tool.output_schema is not None
        assert "status" in tool.output_schema["properties"]
        assert tool.description


def test_vehicle_info_structured_output(fake: FakeVehicle) -> None:
    result = call(fake.connection_string, "vehicle_info")
    assert not result.is_error
    assert result.structured_content["vehicle"]["platform"] == "ardupilot"


def test_params_get_structured_output(fake: FakeVehicle) -> None:
    result = call(fake.connection_string, "params_get", {"name": "compass_use"})
    assert result.structured_content["param"] == {
        "name": "COMPASS_USE",
        "value": 1,
        "type": "INT8",
        "index": 11,
        "precision_limited": False,
    }


def test_errors_are_structured_tool_errors(fake: FakeVehicle) -> None:
    result = call(fake.connection_string, "params_get", {"name": "NOPE"})
    assert result.is_error
    text = result.content[0].text
    payload = json.loads(text[text.index("{") :])
    assert payload["error"] == "param_not_found"


def test_without_connection_vehicle_tools_explain_operator_action() -> None:
    result = call(None, "vehicle_status")
    assert result.is_error
    assert "operator" in result.content[0].text
    caps = call(None, "mavforge_capabilities")
    assert not caps.is_error


def test_reconnects_after_link_loss(make_fake: FakeFactory) -> None:
    fake = make_fake()
    server, provider = _server(fake.connection_string)

    async def run() -> tuple[Any, Any]:
        async with Client(server) as client:
            first = await client.call_tool("vehicle_info", {})
            provider.get().session.connection.close()  # simulate the link dropping
            second = await client.call_tool("vehicle_info", {})
            return first, second

    try:
        first, second = anyio.run(run)
    finally:
        provider.close()
    assert not first.is_error
    assert not second.is_error


@pytest.mark.timeout(60)
def test_stdio_subprocess_end_to_end(fake: FakeVehicle) -> None:
    params = StdioServerParameters(
        command=sys.executable,
        args=["-m", "mavforge.mcp", "--connection", fake.connection_string],
        env={**os.environ},
    )

    async def run() -> Any:
        async with Client(params) as client:
            return await client.call_tool("params_list", {"pattern": "FRAME_*"})

    result = anyio.run(run)
    assert not result.is_error
    names = [p["name"] for p in result.structured_content["params"]]
    assert names == ["FRAME_CLASS", "FRAME_TYPE"]


def test_backup_then_diff_by_id(fake: FakeVehicle) -> None:
    server, provider = _server(fake.connection_string)

    async def run() -> tuple[Any, Any, Any]:
        async with Client(server) as client:
            backup = await client.call_tool("params_backup", {})
            backup_id = backup.structured_content["backup_id"]
            listed = await client.call_tool("params_backups", {})
            diff = await client.call_tool("params_diff", {"backup_id": backup_id})
            return backup, listed, diff

    try:
        backup, listed, diff = anyio.run(run)
    finally:
        provider.close()
    assert backup.structured_content["complete"] is True
    assert [b["backup_id"] for b in listed.structured_content["backups"]] == [
        backup.structured_content["backup_id"]
    ]
    assert diff.structured_content["changed"] == []


@pytest.mark.parametrize("backup_id", ["../../etc/passwd", "/etc/passwd", "x/../y.json"])
def test_agents_cannot_diff_arbitrary_paths(fake: FakeVehicle, backup_id: str) -> None:
    result = call(fake.connection_string, "params_diff", {"backup_id": backup_id})
    assert result.is_error
    assert "invalid_input" in result.content[0].text
    assert "PARAM_REQUEST_LIST" not in fake.received_types()


def test_diagnose_tool(make_fake: FakeFactory) -> None:
    fake = make_fake(prearm_failures=["Need 3D Fix"])
    result = call(fake.connection_string, "diagnose")
    assert not result.is_error
    findings = result.structured_content["findings"]
    assert findings[0]["kind"] == "fault"
    assert findings[0]["vehicle_text"] == "PreArm: Need 3D Fix"


def test_diagnose_tool_propose_fixes(make_fake: FakeFactory) -> None:
    fake = make_fake(params=default_copter_params() | {"LOG_BACKEND_TYPE": (0, ParamType.INT8)})
    off = call(fake.connection_string, "diagnose")
    logging_off = next(
        f for f in off.structured_content["findings"] if f["check"] == "ardupilot.logging"
    )
    assert logging_off["proposed_changes"] == {"LOG_BACKEND_TYPE": 1}
    assert logging_off["proposed_plan"] is None

    on = call(fake.connection_string, "diagnose", {"propose_fixes": True})
    logging_on = next(
        f for f in on.structured_content["findings"] if f["check"] == "ardupilot.logging"
    )
    assert logging_on["proposed_plan"]["changes"][0]["requested"] == 1


def test_diagnose_tool_with_baseline(fake: FakeVehicle) -> None:
    server, provider = _server(fake.connection_string)

    async def run() -> tuple[Any, Any]:
        async with Client(server) as client:
            backup = await client.call_tool("params_backup", {})
            backup_id = backup.structured_content["backup_id"]
            report = await client.call_tool("diagnose", {"baseline_backup_id": backup_id})
            return backup, report

    try:
        _, report = anyio.run(run)
    finally:
        provider.close()
    assert not report.is_error
    baseline = report.structured_content["baseline"]
    assert baseline is not None
    assert baseline["changed"] == []
    assert baseline["current"] == "vehicle"


def test_diagnose_without_baseline_omits_it(fake: FakeVehicle) -> None:
    result = call(fake.connection_string, "diagnose")
    assert result.structured_content["baseline"] is None


@pytest.mark.parametrize("backup_id", ["../../etc/passwd", "/etc/passwd", "x/../y.json"])
def test_agents_cannot_use_diagnose_baseline_for_arbitrary_paths(
    fake: FakeVehicle, backup_id: str
) -> None:
    result = call(fake.connection_string, "diagnose", {"baseline_backup_id": backup_id})
    assert result.is_error
    assert "invalid_input" in result.content[0].text
