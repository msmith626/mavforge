"""Vehicle identification and MAVForge capability listing."""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version

from mavforge.domain.models import CapabilitiesResult, InfoResult, OperationInfo
from mavforge.domain.outcomes import OperationStatus
from mavforge.policy.permissions import Policy
from mavforge.services.catalog import CATALOG, INFO, run_operation
from mavforge.services.session import Session

SUPPORTED_PLATFORMS = ["ardupilot"]


def mavforge_version() -> str:
    try:
        return version("mavforge")
    except PackageNotFoundError:  # running from a source tree without installation
        return "unknown"


def info(session: Session) -> InfoResult:
    """Return the identity captured at connect time plus current link statistics."""
    return run_operation(
        session,
        INFO,
        lambda: InfoResult(
            status=OperationStatus.SUCCEEDED, vehicle=session.identity, link=session.link_stats()
        ),
    )


def capabilities(policy: Policy | None = None) -> CapabilitiesResult:
    """What this MAVForge build can do. Needs no vehicle connection."""
    policy = policy or Policy.read_only()
    return CapabilitiesResult(
        status=OperationStatus.SUCCEEDED,
        mavforge_version=mavforge_version(),
        operations=[
            OperationInfo(
                name=op.name,
                summary=op.summary,
                permission=op.permission.value,
                modifies_vehicle=op.modifies_vehicle,
                available=op.available,
            )
            for op in CATALOG
        ],
        platforms=SUPPORTED_PLATFORMS,
        granted_permissions=sorted(p.value for p in policy.granted),
    )
