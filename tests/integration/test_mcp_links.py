"""MCP link discovery, human link choice and uid-bound reconnect, against fake vehicles."""

import json
from typing import Any

import anyio
from mcp import Client
from mcp_types import ElicitResult

from mavforge.domain.models import VehicleIdentity
from mavforge.mcp.links import DiscoveryConfig
from mavforge.mcp.server import VehicleProvider, build_server
from mavforge.policy.permissions import Policy
from mavforge.services.discovery import probe_link
from mavforge.services.session import ConnectOptions
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle
from mavforge.transport.connstr import ConnectionSpec

LINK_TOOLS = {"vehicle_discover", "vehicle_connect"}


def provider_for(connection: str | None, *also: str) -> VehicleProvider:
    options = ConnectOptions(connection=connection) if connection else None
    return VehicleProvider(
        options,
        Policy.read_only(),
        discovery=DiscoveryConfig(udp_port=None, serial=False, also=also),
    )


def run(provider: VehicleProvider, steps: Any, elicitation: Any = None) -> Any:
    server = build_server(provider, Policy.read_only())

    async def go() -> Any:
        kwargs = {"elicitation_callback": elicitation} if elicitation else {}
        async with Client(server, **kwargs) as client:
            return await steps(client)

    try:
        return anyio.run(go)
    finally:
        provider.close()


def pick_first(shown: list[Any]) -> Any:
    async def respond(ctx: Any, params: Any) -> ElicitResult:
        shown.append(params)
        first = params.requested_schema["properties"]["link"]["enum"][0]
        return ElicitResult(action="accept", content={"link": first})

    return respond


def error(result: Any) -> dict[str, Any]:
    assert result.is_error
    text = result.content[0].text
    return json.loads(text[text.index("{") :])


def test_link_tools_only_with_discovery(fake: FakeVehicle) -> None:
    async def steps(client: Client) -> Any:
        return {t.name: t for t in (await client.list_tools()).tools}

    plain = run(
        VehicleProvider(ConnectOptions(connection=fake.connection_string), Policy.read_only()),
        steps,
    )
    assert not plain.keys() & LINK_TOOLS
    tools = run(provider_for(None, fake.connection_string), steps)
    assert tools.keys() >= LINK_TOOLS
    assert "choice" not in tools["vehicle_connect"].input_schema["properties"]  # hidden
    assert set(tools["vehicle_connect"].input_schema["properties"]) == {"discovery_id"}


def test_user_picks_a_discovered_link(fake: FakeVehicle) -> None:
    shown: list[Any] = []

    async def steps(client: Client) -> Any:
        before = await client.call_tool("vehicle_info", {})
        found = await client.call_tool("vehicle_discover", {})
        connected = await client.call_tool(
            "vehicle_connect", {"discovery_id": found.structured_content["discovery_id"]}
        )
        after = await client.call_tool("vehicle_info", {})
        return before, found, connected, after

    before, found, connected, after = run(
        provider_for(None, fake.connection_string), steps, pick_first(shown)
    )
    assert "vehicle_discover" in error(before)["message"]
    assert found.structured_content["selectable"] == 1
    assert not connected.is_error, connected.content
    assert connected.structured_content["link"]["connection"] == fake.connection_string
    assert after.structured_content["vehicle"]["uid"] == "0123456789abcdef"
    assert len(shown) == 1
    assert fake.connection_string in shown[0].requested_schema["properties"]["link"]["enum"][0]


def test_declined_choice_leaves_server_unconnected(fake: FakeVehicle) -> None:
    async def decline(ctx: Any, params: Any) -> ElicitResult:
        return ElicitResult(action="decline")

    async def steps(client: Client) -> Any:
        found = await client.call_tool("vehicle_discover", {})
        connected = await client.call_tool(
            "vehicle_connect", {"discovery_id": found.structured_content["discovery_id"]}
        )
        return connected, await client.call_tool("vehicle_info", {})

    connected, info = run(provider_for(None, fake.connection_string), steps, decline)
    assert connected.is_error
    assert info.is_error


def test_without_elicitation_the_operator_is_told_what_to_configure(fake: FakeVehicle) -> None:
    async def steps(client: Client) -> Any:
        found = await client.call_tool("vehicle_discover", {})
        return await client.call_tool(
            "vehicle_connect", {"discovery_id": found.structured_content["discovery_id"]}
        )

    err = error(run(provider_for(None, fake.connection_string), steps))
    assert err["error"] == "link_choice_unavailable"
    assert fake.connection_string in err["message"]


def test_unknown_discovery_id_is_refused(fake: FakeVehicle) -> None:
    async def steps(client: Client) -> Any:
        return await client.call_tool("vehicle_connect", {"discovery_id": "disc-nope"})

    err = error(run(provider_for(None, fake.connection_string), steps, pick_first([])))
    assert "vehicle_discover again" in err["message"]


def test_discovery_never_probes_the_link_in_use(make_fake: FakeFactory) -> None:
    current, other = make_fake(), make_fake(uid=0xAAAA)

    async def steps(client: Client) -> Any:
        await client.call_tool("vehicle_info", {})
        return await client.call_tool("vehicle_discover", {})

    found = run(
        provider_for(current.connection_string, current.connection_string, other.connection_string),
        steps,
    ).structured_content
    assert [c["connection"] for c in found["candidates"]] == [other.connection_string]
    assert found["current_connection"] == current.connection_string
    assert found["current_uid"] == "0123456789abcdef"


def test_switch_refused_while_an_operation_runs(make_fake: FakeFactory) -> None:
    current, other = make_fake(), make_fake()
    provider = provider_for(current.connection_string, other.connection_string)

    async def steps(client: Client) -> Any:
        await client.call_tool("vehicle_info", {})
        found = await client.call_tool("vehicle_discover", {})
        provider.enter()  # models another tool call still in progress
        try:
            return await client.call_tool(
                "vehicle_connect", {"discovery_id": found.structured_content["discovery_id"]}
            )
        finally:
            provider.leave()

    assert "still running" in error(run(provider, steps, pick_first([])))["message"]


def test_lost_link_reconnects_to_the_same_board_elsewhere(make_fake: FakeFactory) -> None:
    usb, wifi = make_fake(), make_fake()  # same default uid: one board, two links
    provider = provider_for(usb.connection_string, wifi.connection_string)

    async def steps(client: Client) -> Any:
        first = await client.call_tool("vehicle_info", {})
        usb.stop()
        result = await client.call_tool("vehicle_info", {})
        # The first call after a loss may report it before the next one reconnects.
        for _ in range(2):
            if not result.is_error:
                break
            result = await client.call_tool("vehicle_info", {})
        return first, result

    first, later = run(provider, steps)
    assert first.structured_content["link"]["connection"] == usb.connection_string
    assert not later.is_error, later.content
    assert later.structured_content["link"]["connection"] == wifi.connection_string


def test_lost_link_does_not_switch_to_a_different_board(make_fake: FakeFactory) -> None:
    usb, stranger = make_fake(), make_fake(uid=0xBBBB)
    provider = provider_for(usb.connection_string, stranger.connection_string)

    async def steps(client: Client) -> Any:
        await client.call_tool("vehicle_info", {})
        usb.stop()
        results = [await client.call_tool("vehicle_info", {}) for _ in range(2)]
        return results[-1]

    err = error(run(provider, steps))
    assert "did not find the same board" in err["message"]
    assert stranger.received == [] or all(
        m.get_type() != "PARAM_REQUEST_LIST" for m in stranger.received
    )


def test_a_different_board_on_the_approved_link_is_refused(fake: FakeVehicle) -> None:
    provider = provider_for(fake.connection_string, fake.connection_string)

    async def steps(client: Client) -> Any:
        await client.call_tool("vehicle_info", {})
        provider._approved_uid = "ffffffffffffffff"  # as if another board was approved
        provider.drop()
        return await client.call_tool("vehicle_info", {})

    assert "different board" in error(run(provider, steps))["message"]


def test_reconnect_tries_a_link_whose_probe_lost_the_uid(make_fake: FakeFactory) -> None:
    """Lossy WiFi often drops AUTOPILOT_VERSION during the probe; the full connect that
    follows may still get the uid, and is only used if it matches."""
    usb, wifi = make_fake(), make_fake()

    def lossy(spec: ConnectionSpec, timeout: float) -> VehicleIdentity:
        identity = probe_link(spec, timeout)
        return identity.model_copy(update={"uid": None})

    provider = VehicleProvider(
        ConnectOptions(connection=usb.connection_string),
        Policy.read_only(),
        discovery=DiscoveryConfig(udp_port=None, serial=False, also=(wifi.connection_string,)),
        prober=lossy,
    )

    async def steps(client: Client) -> Any:
        await client.call_tool("vehicle_info", {})
        usb.stop()
        result = await client.call_tool("vehicle_info", {})
        if result.is_error:
            result = await client.call_tool("vehicle_info", {})
        return result

    later = run(provider, steps)
    assert not later.is_error, later.content
    assert later.structured_content["link"]["connection"] == wifi.connection_string


def test_first_connect_failure_points_at_discovery(make_fake: FakeFactory) -> None:
    """Started with a link that is already gone (USB unplugged) and no board approved yet:
    no silent switch, but the error says how the user can choose another link."""
    gone, wifi = make_fake(), make_fake()
    gone.stop()

    async def steps(client: Client) -> Any:
        return await client.call_tool("vehicle_info", {})

    err = error(run(provider_for(gone.connection_string, wifi.connection_string), steps))
    assert "vehicle_discover" in err["message"]
    assert wifi.received == []  # nothing else was probed or connected to
