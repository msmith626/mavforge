"""Diff against firmware defaults (param.pck?withdefaults=1) against real ArduCopter
SITL. Read-only on the vehicle."""

from __future__ import annotations

import pytest

import mavforge
from mavforge.domain.outcomes import OperationStatus

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(240)]

BOOT_TIMEOUT = 60.0


def test_diff_against_defaults_sitl(sitl_connection: str) -> None:
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        result = vehicle.params.diff_defaults()
        backup = vehicle.params.backup()
    assert result.status is OperationStatus.SUCCEEDED
    assert result.reference == "firmware defaults"
    # SITL boots with Tools/autotest/default_params/copter.parm, which deliberately
    # overrides some stock defaults for simulation (frame type, sim backends, ...):
    # there must be at least some genuine differences to report.
    assert len(result.changed) > 0
    assert result.unchanged + len(result.changed) == backup.param_count
    # Every changed entry's "current" value must match what a plain backup sees -
    # confirms param.pck?withdefaults=1's live values agree with the plain download.
    live = {p.name: p.value for p in mavforge.load_backup(backup.path).params}
    assert all(c.current == live[c.name] for c in result.changed)
