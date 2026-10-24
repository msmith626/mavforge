"""Infrastructure for parameter-based diagnostic rules.

A rule is a function of the vehicle identity and its parameters that returns findings.
Rules read parameters through ``ParamView``; asking for a parameter the firmware does
not have raises ``MissingParams``, which ``run_rule`` turns into a NOT_CHECKED finding
instead of guessing.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass

from mavforge.domain.models import Finding, FindingKind, FindingSource, VehicleIdentity


class MissingParams(Exception):  # noqa: N818 - control flow, not an error condition
    def __init__(self, names: list[str]) -> None:
        super().__init__(", ".join(names))
        self.names = names


class ParamView:
    def __init__(self, values: Mapping[str, float | int]) -> None:
        self._values = values

    def has(self, name: str) -> bool:
        return name in self._values

    def get(self, name: str) -> float | int | None:
        return self._values.get(name)

    def require(self, *names: str) -> list[float | int]:
        missing = [n for n in names if n not in self._values]
        if missing:
            raise MissingParams(missing)
        return [self._values[n] for n in names]

    def first_present(self, *names: str) -> str:
        """The first of several firmware-specific names that exists (for renamed params)."""
        for name in names:
            if name in self._values:
                return name
        raise MissingParams(list(names))


@dataclass(frozen=True)
class ParamRule:
    check: str
    title: str
    """What the check looks at; used for NOT_CHECKED findings."""
    evaluate: Callable[[VehicleIdentity, ParamView], list[Finding]]


def finding(
    rule: ParamRule | str,
    kind: FindingKind,
    title: str,
    detail: str,
    *,
    evidence: dict[str, float | int | str | bool | None] | None = None,
    recommendation: str | None = None,
    source: FindingSource = FindingSource.PARAMETERS,
    proposed_changes: dict[str, float | int] | None = None,
) -> Finding:
    return Finding(
        check=rule.check if isinstance(rule, ParamRule) else rule,
        kind=kind,
        title=title,
        detail=detail,
        source=source,
        evidence=evidence or {},
        recommendation=recommendation,
        proposed_changes=proposed_changes,
    )


def run_rule(rule: ParamRule, identity: VehicleIdentity, params: ParamView) -> list[Finding]:
    try:
        return rule.evaluate(identity, params)
    except MissingParams as exc:
        return [
            finding(
                rule,
                FindingKind.NOT_CHECKED,
                f"{rule.title}: not checked",
                f"Parameter(s) not present on this vehicle/firmware: {', '.join(exc.names)}.",
            )
        ]
