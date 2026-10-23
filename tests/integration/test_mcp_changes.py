"""MCP parameter-change tools: registration, approval paths and enforcement."""

from typing import Any

import anyio
import pytest
from mcp import Client
from mcp_types import ElicitResult

import mavforge
from mavforge.mcp.server import VehicleProvider, build_server
from mavforge.policy.permissions import Policy
from mavforge.services.session import ConnectOptions
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle

WRITE_TOOLS = {"params_plan", "params_plan_restore", "params_plans", "params_apply"}


def server_for(fake: FakeVehicle, policy: Policy) -> tuple[Any, VehicleProvider]:
    provider = VehicleProvider(ConnectOptions(connection=fake.connection_string), policy)
    return build_server(provider, policy), provider


def run(fake: FakeVehicle, policy: Policy, steps: Any, elicitation: Any = None) -> Any:
    server, provider = server_for(fake, policy)

    async def go() -> Any:
        kwargs = {"elicitation_callback": elicitation} if elicitation else {}
        async with Client(server, **kwargs) as client:
            return await steps(client)

    try:
        return anyio.run(go)
    finally:
        provider.close()


def writes(fake: FakeVehicle) -> int:
    return sum(m.get_type() == "PARAM_SET" for m in fake.received)


def test_write_tools_absent_without_allow_configure(fake: FakeVehicle) -> None:
    async def steps(client: Client) -> Any:
        return {t.name for t in (await client.list_tools()).tools}

    names = run(fake, Policy.read_only(), steps)
    assert not names & WRITE_TOOLS


def test_apply_is_marked_destructive(fake: FakeVehicle) -> None:
    async def steps(client: Client) -> Any:
        return {t.name: t for t in (await client.list_tools()).tools}

    tools = run(fake, Policy.configure(allow_real_vehicle=False), steps)
    assert tools.keys() >= WRITE_TOOLS
    assert tools["params_apply"].annotations.destructive_hint is True
    assert "approval" not in tools["params_apply"].input_schema["properties"]  # hidden from agents


async def _plan_and_apply(client: Client) -> tuple[Any, Any]:
    plan = await client.call_tool("params_plan", {"changes": {"RC1_TRIM": 1490}})
    plan_id = plan.structured_content["plan_id"]
    return plan, await client.call_tool("params_apply", {"plan_id": plan_id})


def test_human_approves_via_elicitation(fake: FakeVehicle) -> None:
    shown: list[str] = []

    async def accept(ctx: Any, params: Any) -> ElicitResult:
        shown.append(params.message)
        return ElicitResult(action="accept", content={"approve": True})

    _, applied = run(fake, Policy.configure(allow_real_vehicle=False), _plan_and_apply, accept)
    assert not applied.is_error, applied.content
    assert applied.structured_content["approved_by"] == "mcp-elicitation"
    assert applied.structured_content["changes"][0]["outcome"] == "verified"
    assert "RC1_TRIM: 1500 -> 1490" in shown[0]


@pytest.mark.parametrize(
    "answer", [{"action": "decline"}, {"action": "accept", "content": {"approve": False}}]
)
def test_human_declines(fake: FakeVehicle, answer: dict[str, Any]) -> None:
    async def respond(ctx: Any, params: Any) -> ElicitResult:
        return ElicitResult(**answer)

    _, applied = run(fake, Policy.configure(allow_real_vehicle=False), _plan_and_apply, respond)
    assert applied.is_error
    assert writes(fake) == 0


def test_without_elicitation_human_approves_in_terminal(fake: FakeVehicle) -> None:
    async def steps(client: Client) -> Any:
        plan, first = await _plan_and_apply(client)
        plan_id = plan.structured_content["plan_id"]
        mavforge.approve_plan(plan_id, approved_by="cli-prompt")  # the human, out of band
        second = await client.call_tool("params_apply", {"plan_id": plan_id})
        return first, second

    first, second = run(fake, Policy.configure(allow_real_vehicle=False), steps)
    assert first.is_error
    assert "approval_required" in first.content[0].text
    assert "mavforge approve" in first.content[0].text
    assert not second.is_error
    assert second.structured_content["approved_by"] == "cli-prompt"


def test_agent_cannot_forge_approval(fake: FakeVehicle) -> None:
    async def steps(client: Client) -> Any:
        plan = await client.call_tool("params_plan", {"changes": {"RC1_TRIM": 1490}})
        return await client.call_tool(
            "params_apply",
            {"plan_id": plan.structured_content["plan_id"], "approval": {"approve": True}},
        )

    result = run(fake, Policy.configure(allow_real_vehicle=False), steps)
    assert result.is_error
    assert writes(fake) == 0


def test_real_vehicle_requires_operator_flag(make_fake: FakeFactory) -> None:
    fake = make_fake(simulated=False)

    async def accept(ctx: Any, params: Any) -> ElicitResult:
        return ElicitResult(action="accept", content={"approve": True})

    _, applied = run(fake, Policy.configure(allow_real_vehicle=False), _plan_and_apply, accept)
    assert applied.is_error
    assert "not_permitted" in applied.content[0].text
    assert writes(fake) == 0
