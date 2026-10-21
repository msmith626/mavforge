"""ArduPilot platform adapter."""

from __future__ import annotations

from mavforge.platforms.base import Capability, PlatformAdapter
from mavforge.transport.dialect import mavlink, mavutil

_FAMILY_TYPES: dict[str, list[str]] = {
    "copter": [
        "QUADROTOR",
        "HEXAROTOR",
        "OCTOROTOR",
        "TRICOPTER",
        "COAXIAL",
        "DECAROTOR",
        "DODECAROTOR",
    ],
    "heli": ["HELICOPTER"],
    "plane": ["FIXED_WING", "VTOL_DUOROTOR", "VTOL_QUADROTOR", "VTOL_TILTROTOR"],
    "rover": ["GROUND_ROVER", "SURFACE_BOAT"],
    "sub": ["SUBMARINE"],
    "tracker": ["ANTENNA_TRACKER"],
    "blimp": ["AIRSHIP"],
}

# Resolved by name so a dialect lacking a newer MAV_TYPE does not break import.
_FAMILIES: dict[int, str] = {
    value: family
    for family, names in _FAMILY_TYPES.items()
    for name in names
    if (value := getattr(mavlink, f"MAV_TYPE_{name}", None)) is not None
}


class ArduPilotAdapter(PlatformAdapter):
    name = "ardupilot"
    capabilities = frozenset({Capability.IDENTIFY, Capability.STATUS, Capability.PARAM_READ})
    simulation_probe_message = mavlink.MAVLINK_MSG_ID_SIMSTATE
    """ArduPilot only sends SIMSTATE from simulator builds (AP_SIM_ENABLED)."""

    @classmethod
    def matches(cls, heartbeat: mavlink.MAVLink_heartbeat_message) -> bool:
        return heartbeat.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA

    def vehicle_family(self, mav_type: int) -> str | None:
        return _FAMILIES.get(mav_type)

    def mode_name(self, heartbeat: mavlink.MAVLink_heartbeat_message) -> str | None:
        if not heartbeat.base_mode & mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            return None
        mapping = mavutil.mode_mapping_bynumber(heartbeat.type)
        if not mapping:
            return None
        return mapping.get(heartbeat.custom_mode, f"MODE_{heartbeat.custom_mode}")

    def normalize_param_name(self, name: str) -> str:
        # ArduPilot parameter names are upper case by convention.
        return super().normalize_param_name(name).upper()
