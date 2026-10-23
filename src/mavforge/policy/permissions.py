"""Permission tiers and the policy object every operation is checked against.

The default policy is read-only. There is deliberately no permission tier for arming,
motor tests or flight: MAVForge is a configuration toolkit, and those actions belong
to a ground-control station operated by a human.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from mavforge.domain.errors import NotPermittedError


class Permission(StrEnum):
    READ = "read"
    """Observe the vehicle. No vehicle state is changed."""
    CONFIGURE = "configure"
    """Change parameters through reviewed, approved and verified plans."""
    CALIBRATE = "calibrate"
    """Run approved calibration workflows (they rewrite calibration parameters)."""
    MAINTENANCE = "maintenance"
    """Reboot and similar maintenance actions. Not yet implemented."""


@dataclass(frozen=True)
class Policy:
    granted: frozenset[Permission] = field(default_factory=lambda: frozenset({Permission.READ}))
    allow_real_vehicle: bool = False
    """Allow modifying operations on vehicles not identified as simulators. Off by default:
    an operator must opt in explicitly before anything changes real hardware."""

    @classmethod
    def read_only(cls) -> Policy:
        return cls()

    @classmethod
    def configure(cls, *, allow_real_vehicle: bool, calibrate: bool = False) -> Policy:
        granted = {Permission.READ, Permission.CONFIGURE}
        if calibrate:
            granted.add(Permission.CALIBRATE)
        return cls(granted=frozenset(granted), allow_real_vehicle=allow_real_vehicle)

    @classmethod
    def calibrate(cls, *, allow_real_vehicle: bool) -> Policy:
        return cls(
            granted=frozenset({Permission.READ, Permission.CALIBRATE}),
            allow_real_vehicle=allow_real_vehicle,
        )

    def require_vehicle_writable(self, simulated: bool | None, operation: str) -> None:
        if simulated is not True and not self.allow_real_vehicle:
            raise NotPermittedError(
                f"Operation {operation!r} would modify a real (non-simulated) vehicle, which "
                "the active policy does not allow",
                details={"operation": operation, "simulated": simulated},
            )

    def allows(self, permission: Permission) -> bool:
        return permission in self.granted

    def require(self, permission: Permission, operation: str) -> None:
        if not self.allows(permission):
            raise NotPermittedError(
                f"Operation {operation!r} requires the {permission.value!r} permission, "
                "which the active policy does not grant",
                details={"operation": operation, "required": permission.value},
            )
