"""The single place where pymavlink's process-global dialect state is configured.

pymavlink selects its dialect from environment variables at import time and may later
switch protocol versions by mutating ``os.environ``. MAVForge pins MAVLink 2 with the
``ardupilotmega`` dialect (a superset of ``common``) here, once, before anything else
imports pymavlink. Import ``mavlink`` and ``mavutil`` from this module only.
"""

from __future__ import annotations

import os

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

mavutil.set_dialect("ardupilotmega")

Message = mavlink.MAVLink_message


def enum_name(enum: str, value: int, *, strip_prefix: bool = True) -> str:
    """Return the dialect name for an enum value, e.g. ``MAV_TYPE`` 2 -> ``QUADROTOR``.

    Unknown values are rendered as ``<ENUM>_<value>`` rather than raising, because
    vehicles may report values newer than our dialect.
    """
    entries = mavlink.enums.get(enum, {})
    entry = entries.get(value)
    if entry is None:
        return f"{enum}_{value}" if not strip_prefix else f"UNKNOWN_{value}"
    name: str = entry.name
    prefix = f"{enum}_"
    if strip_prefix and name.startswith(prefix):
        return name[len(prefix) :]
    return name


__all__ = ["Message", "enum_name", "mavlink", "mavutil"]
