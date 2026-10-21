"""Fallback adapter for autopilots MAVForge has no specific support for.

It offers only protocol-generic, read-only operations. Nothing here is claimed to be
tested against a non-ArduPilot autopilot.
"""

from __future__ import annotations

from mavforge.platforms.base import Capability, PlatformAdapter
from mavforge.transport.dialect import mavlink


class GenericAdapter(PlatformAdapter):
    name = "generic"
    capabilities = frozenset({Capability.IDENTIFY, Capability.STATUS, Capability.PARAM_READ})

    @classmethod
    def matches(cls, heartbeat: mavlink.MAVLink_heartbeat_message) -> bool:
        return True

    def vehicle_family(self, mav_type: int) -> str | None:
        return None

    def mode_name(self, heartbeat: mavlink.MAVLink_heartbeat_message) -> str | None:
        return None
