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
    """Change parameters (with review, approval and verification). Not yet implemented."""
    CALIBRATE = "calibrate"
    """Run calibration workflows. Not yet implemented."""
    MAINTENANCE = "maintenance"
    """Reboot and similar maintenance actions. Not yet implemented."""


@dataclass(frozen=True)
class Policy:
    granted: frozenset[Permission] = field(default_factory=lambda: frozenset({Permission.READ}))

    @classmethod
    def read_only(cls) -> Policy:
        return cls()

    def allows(self, permission: Permission) -> bool:
        return permission in self.granted

    def require(self, permission: Permission, operation: str) -> None:
        if not self.allows(permission):
            raise NotPermittedError(
                f"Operation {operation!r} requires the {permission.value!r} permission, "
                "which the active policy does not grant",
                details={"operation": operation, "required": permission.value},
            )
