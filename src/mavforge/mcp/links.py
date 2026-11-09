"""Human-selected vehicle links for the MCP server.

The agent may ask the server to look for vehicles (``vehicle_discover``) and to switch
to one (``vehicle_connect``), but it never supplies a connection string and cannot make
the choice: the server finds the candidates itself, and the human picks one through MCP
elicitation from a list rendered only from validated fields (connection strings,
MAVLink enum names, version numbers, hex uids), never from free-form OS or vehicle text.

Discoveries are stored by id so the choice offered to the human is identical on every
round of the elicitation protocol, however long the probe took or whatever changed since.
"""

from __future__ import annotations

import secrets
import threading
import time
from dataclasses import dataclass, field
from typing import Annotated

from pydantic import AfterValidator, BaseModel, ConfigDict, Field, JsonValue, create_model

from mavforge.domain.errors import InvalidInputError
from mavforge.domain.models import LinkCandidate, ProbeOutcome
from mavforge.domain.outcomes import OperationStatus
from mavforge.services.discovery import DEFAULT_UDP_PORT

DISCOVERY_TTL = 600.0
"""Seconds a discovery stays selectable; after that, ask the agent to discover again."""
_MAX_STORED = 8


@dataclass(frozen=True)
class DiscoveryConfig:
    """What the server may probe, fixed by the operator at startup."""

    udp_port: int | None = DEFAULT_UDP_PORT
    serial: bool = True
    also: tuple[str, ...] = ()


class LinkDiscovery(BaseModel):
    """Result of ``vehicle_discover``. Pass ``discovery_id`` to ``vehicle_connect``."""

    model_config = ConfigDict(frozen=True)

    status: OperationStatus = OperationStatus.SUCCEEDED
    discovery_id: str
    current_connection: str | None = Field(
        description="The link the server is using now (not probed), if any."
    )
    current_uid: str | None = Field(description="Board uid of the vehicle the user approved.")
    candidates: list[LinkCandidate]
    skipped: list[str] = Field(default_factory=list)
    selectable: int = Field(description="How many candidates have a vehicle the user can pick.")


class LinkChoice(BaseModel):
    """Form shown to the human by the MCP client (never filled in by the agent)."""

    link: str = Field(description="Vehicle link to connect to")


class LinkChoiceUnavailable(BaseModel):
    reason: str


@dataclass
class _Stored:
    created: float
    result: LinkDiscovery
    labels: dict[str, LinkCandidate] = field(default_factory=dict)


def _label(c: LinkCandidate, approved_uid: str | None) -> str:
    v = c.vehicle
    assert v is not None
    kind = {"serial": "USB/serial", "udp": "UDP (WiFi)", "operator": "configured"}[c.source]
    fw = f" {v.firmware.major}.{v.firmware.minor}.{v.firmware.patch}" if v.firmware else ""
    uid = f", uid {v.uid[:12]}" if v.uid else ", no uid"
    same = " [same board as before]" if approved_uid and v.uid == approved_uid else ""
    what = f"{v.vehicle_family or v.mav_type}{fw} sysid {v.system_id}"
    return f"{c.connection} ({kind}): {what}{uid}{same}"


class DiscoveryStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._items: dict[str, _Stored] = {}

    def put(self, result: LinkDiscovery, approved_uid: str | None) -> None:
        labels: dict[str, LinkCandidate] = {}
        for c in result.candidates:
            if c.outcome is ProbeOutcome.VEHICLE and c.vehicle is not None:
                labels[_label(c, approved_uid)] = c
        with self._lock:
            now = time.monotonic()
            self._items = {k: v for k, v in self._items.items() if now - v.created < DISCOVERY_TTL}
            while len(self._items) >= _MAX_STORED:
                self._items.pop(next(iter(self._items)))
            self._items[result.discovery_id] = _Stored(now, result, labels)

    def get(self, discovery_id: str) -> _Stored:
        with self._lock:
            item = self._items.get(discovery_id)
        if item is None or time.monotonic() - item.created >= DISCOVERY_TTL:
            raise InvalidInputError(
                f"Unknown or expired discovery {discovery_id!r}; call vehicle_discover again"
            )
        return item


def new_discovery_id() -> str:
    return f"disc-{secrets.token_hex(4)}"


def choice_schema(labels: list[str]) -> type[LinkChoice]:
    """A LinkChoice whose ``link`` must be one of exactly these labels (a pick-list).

    The enum is written into the JSON schema explicitly: pydantic renders a one-value
    ``Literal`` as ``const``, which clients don't show as a choice. The validator keeps
    the server from accepting anything else the client might send back.
    """
    allowed = frozenset(labels)

    def one_of(value: str) -> str:
        if value not in allowed:
            raise ValueError("not one of the offered links")
        return value

    return create_model(
        "LinkChoice",
        __base__=LinkChoice,
        link=(
            Annotated[str, AfterValidator(one_of)],
            Field(
                description="Vehicle link to connect to",
                json_schema_extra={"enum": list[JsonValue](labels)},
            ),
        ),
    )


def choice_message(stored: _Stored) -> str:
    r = stored.result
    lines = ["MAVForge: choose the vehicle link to use (the agent cannot choose for you)."]
    if r.current_connection:
        lines.append(f"Currently using: {r.current_connection}")
    for c in stored.labels.values():
        if c.vehicle is not None and c.vehicle.simulated is not True:
            lines.append(f"! REAL VEHICLE on {c.connection}")
        if r.current_uid and c.vehicle is not None and c.vehicle.uid != r.current_uid:
            lines.append(f"! {c.connection} is a DIFFERENT board than the one approved before")
    return "\n".join(lines)
