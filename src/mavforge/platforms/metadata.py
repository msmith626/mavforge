"""Platform-neutral parameter metadata types (ranges, allowed values, flags)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ParamMeta:
    name: str
    display_name: str | None = None
    description: str | None = None
    units: str | None = None
    range: tuple[float, float] | None = None
    values: dict[float, str] = field(default_factory=dict[float, str])
    bitmask: dict[int, str] = field(default_factory=dict[int, str])
    reboot_required: bool = False
    read_only: bool = False
    volatile: bool = False


@dataclass(frozen=True)
class MetadataSet:
    source: str
    """URL the definitions came from."""
    version_matched: bool
    """True if the definitions are for the vehicle's exact firmware release."""
    params: dict[str, ParamMeta]

    def get(self, name: str) -> ParamMeta | None:
        return self.params.get(name)
