"""Interface parity: the API, CLI (--json) and MCP return the same results.

Each interface opens its own connection to the same fake vehicle. Fields that
legitimately differ per connection (link statistics, message timestamps) are removed
before comparison.
"""

import json
from collections.abc import Callable
from pathlib import Path
from typing import Any

import anyio
import pytest
from mcp import Client
from typer.testing import CliRunner

import mavforge
from mavforge.cli.app import app
from mavforge.domain.errors import NotPermittedError
from mavforge.mcp.server import VehicleProvider, build_server
from mavforge.policy.permissions import Policy
from mavforge.services.session import ConnectOptions
from mavforge.testing.fake_vehicle import FakeVehicle

VOLATILE = {"link", "received_at"}


def _strip(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: _strip(v) for k, v in value.items() if k not in VOLATILE}  # type: ignore[misc]
    if isinstance(value, list):
        return [_strip(v) for v in value]  # type: ignore[misc]
    return value


def via_api(fake: FakeVehicle, op: Callable[[mavforge.Vehicle], Any]) -> Any:
    with mavforge.connect(fake.connection_string) as vehicle:
        return json.loads(op(vehicle).model_dump_json())


def via_cli(fake: FakeVehicle, *args: str) -> Any:
    result = CliRunner().invoke(app, ["-c", fake.connection_string, "--json", *args])
    return json.loads(result.stdout)


def via_mcp(fake: FakeVehicle, tool: str, arguments: dict[str, Any]) -> Any:
    policy = Policy.read_only()
    provider = VehicleProvider(ConnectOptions(connection=fake.connection_string), policy)
    server = build_server(provider, policy)

    async def run() -> Any:
        async with Client(server) as client:
            return await client.call_tool(tool, arguments)

    try:
        return anyio.run(run).structured_content
    finally:
        provider.close()


CASES: list[tuple[str, Callable[[mavforge.Vehicle], Any], list[str], str, dict[str, Any]]] = [
    ("info", lambda v: v.info(), ["info"], "vehicle_info", {}),
    ("status", lambda v: v.status(), ["status"], "vehicle_status", {}),
    (
        "params_list",
        lambda v: v.params.list("INS_*"),
        ["params", "list", "INS_*"],
        "params_list",
        {"pattern": "INS_*"},
    ),
    (
        "params_get",
        lambda v: v.params.get("LOG_BITMASK"),
        ["params", "get", "LOG_BITMASK"],
        "params_get",
        {"name": "LOG_BITMASK"},
    ),
    ("diagnose", lambda v: v.diagnose(), ["diagnose"], "diagnose", {}),
]


@pytest.mark.parametrize(
    ("name", "api_op", "cli_args", "tool", "tool_args"), CASES, ids=[c[0] for c in CASES]
)
def test_interfaces_return_identical_results(
    fake: FakeVehicle,
    name: str,
    api_op: Callable[[mavforge.Vehicle], Any],
    cli_args: list[str],
    tool: str,
    tool_args: dict[str, Any],
) -> None:
    api = _strip(via_api(fake, api_op))
    cli = _strip(via_cli(fake, *cli_args))
    mcp = _strip(via_mcp(fake, tool, tool_args))
    assert api == cli == mcp
    assert api["status"] == "succeeded"


def test_capabilities_parity() -> None:
    api = json.loads(mavforge.capabilities().model_dump_json())
    cli = json.loads(CliRunner().invoke(app, ["--json", "capabilities"]).stdout)
    mcp = via_mcp_no_vehicle("mavforge_capabilities")
    assert api == cli == mcp


def via_mcp_no_vehicle(tool: str) -> Any:
    policy = Policy.read_only()
    provider = VehicleProvider(None, policy)

    async def run() -> Any:
        async with Client(build_server(provider, policy)) as client:
            return await client.call_tool(tool, {})

    return anyio.run(run).structured_content


def test_every_interface_writes_the_same_audit_records(
    fake: FakeVehicle, isolated_state_dir: Path
) -> None:
    via_api(fake, lambda v: v.params.get("FRAME_CLASS"))
    via_cli(fake, "params", "get", "FRAME_CLASS")
    via_mcp(fake, "params_get", {"name": "FRAME_CLASS"})
    entries = [
        json.loads(line) for line in (isolated_state_dir / "audit.jsonl").read_text().splitlines()
    ]
    assert [(e["interface"], e["operation"], e["param"], e["status"]) for e in entries] == [
        (interface, "params_get", "FRAME_CLASS", "succeeded") for interface in ("api", "cli", "mcp")
    ]


def test_policy_is_enforced_below_the_interfaces(fake: FakeVehicle) -> None:
    """A policy that withholds a permission blocks the API and MCP alike."""
    deny_all = Policy(granted=frozenset())
    with (
        mavforge.connect(fake.connection_string, policy=deny_all) as vehicle,
        pytest.raises(NotPermittedError),
    ):
        vehicle.status()

    provider = VehicleProvider(ConnectOptions(connection=fake.connection_string), deny_all)

    async def run() -> Any:
        async with Client(build_server(provider, deny_all)) as client:
            return await client.call_tool("vehicle_status", {})

    try:
        result = anyio.run(run)
    finally:
        provider.close()
    assert result.is_error
    assert "not_permitted" in result.content[0].text


def test_backup_and_diff_parity(fake: FakeVehicle, isolated_state_dir: Path) -> None:
    """Backups made through each interface hold the same data; diffs agree."""
    api = via_api(fake, lambda v: v.params.backup())
    cli = via_cli(fake, "params", "backup")
    mcp = via_mcp(fake, "params_backup", {})
    ids = [api["backup_id"], cli["backup_id"], mcp["backup_id"]]
    assert len(set(ids)) == 3  # same-second default names get -2, -3 suffixes
    contents = [json.loads(Path(r["path"]).read_text()) for r in (api, cli, mcp)]
    for content in contents:
        content.pop("created_at")
    assert contents[0] == contents[1] == contents[2]

    diffs = [
        _strip(via_api(fake, lambda v: v.params.diff(ids[0]))),
        _strip(via_cli(fake, "params", "diff", ids[0])),
        _strip(via_mcp(fake, "params_diff", {"backup_id": ids[0]})),
    ]
    assert diffs[0] == diffs[1] == diffs[2]
    assert diffs[0]["changed"] == []
