"""The ``PlatformAdapter`` interface and adapter selection.

Autopilot-specific behaviour (mode names, parameter naming, simulator detection, and
which MAVForge operations are supported) lives in adapters, not in ``if autopilot ==``
checks spread across services.
"""

from __future__ import annotations

import re
from abc import ABC, abstractmethod
from collections.abc import Sequence
from enum import StrEnum
from typing import TYPE_CHECKING

from mavforge.domain.errors import InvalidInputError
from mavforge.transport.dialect import mavlink

if TYPE_CHECKING:
    from mavforge.platforms.rules import ParamRule

_PARAM_NAME_RE = re.compile(r"^[A-Za-z0-9_]{1,16}$")


class Capability(StrEnum):
    """MAVForge operations an adapter may declare support for."""

    IDENTIFY = "identify"
    STATUS = "status"
    PARAM_READ = "param_read"


class PlatformAdapter(ABC):
    name: str
    """Short identifier, e.g. 'ardupilot'."""
    capabilities: frozenset[Capability]
    simulation_probe_message: int | None = None
    """A message only a simulator sends; if it arrives on request, the vehicle is simulated."""

    @classmethod
    @abstractmethod
    def matches(cls, heartbeat: mavlink.MAVLink_heartbeat_message) -> bool: ...

    @abstractmethod
    def vehicle_family(self, mav_type: int) -> str | None: ...

    @abstractmethod
    def mode_name(self, heartbeat: mavlink.MAVLink_heartbeat_message) -> str | None: ...

    def param_rules(self) -> Sequence[ParamRule]:
        """Platform-specific configuration rules used by diagnostics."""
        return ()

    def is_volatile_param(self, name: str) -> bool:
        """True for parameters the firmware changes on its own (counters, boot-time
        calibration), which a diff should flag rather than treat as configuration."""
        return False

    def normalize_param_name(self, name: str) -> str:
        name = name.strip()
        if not _PARAM_NAME_RE.match(name):
            raise InvalidInputError(
                f"Invalid parameter name {name!r}: expected 1-16 letters, digits or underscores"
            )
        return name


def select_adapter(heartbeat: mavlink.MAVLink_heartbeat_message) -> PlatformAdapter:
    from mavforge.platforms.ardupilot.adapter import ArduPilotAdapter
    from mavforge.platforms.generic import GenericAdapter

    for adapter_cls in (ArduPilotAdapter,):
        if adapter_cls.matches(heartbeat):
            return adapter_cls()
    return GenericAdapter()
