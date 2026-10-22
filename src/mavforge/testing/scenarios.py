"""Ready-made fake-vehicle configurations for tests and demos."""

from __future__ import annotations

from mavforge.domain.models import ParamType
from mavforge.testing.fake_vehicle import ParamSpec, default_copter_params


def unconfigured_plane_params() -> dict[str, ParamSpec]:
    """Modelled on the ArduPlane 4.6.2 bench board in docs/research.md (hardware notes):
    pre-arm checks off, compass uncalibrated, no battery failsafe action."""
    params = default_copter_params()
    for name in ("ARMING_SKIPCHK", "FS_THR_ENABLE"):
        del params[name]
    params |= {
        "ARMING_CHECK": (0, ParamType.INT32),
        "THR_FAILSAFE": (1, ParamType.INT8),
        "COMPASS_OFS_X": (0.0, ParamType.REAL32),
        "COMPASS_OFS_Y": (0.0, ParamType.REAL32),
        "COMPASS_OFS_Z": (0.0, ParamType.REAL32),
        "BATT_FS_LOW_ACT": (0, ParamType.INT8),
        "BATT_FS_CRT_ACT": (0, ParamType.INT8),
    }
    return params
