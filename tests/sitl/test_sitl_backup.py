"""Parameter backup and diff against real ArduCopter SITL. Read-only on the vehicle."""

from __future__ import annotations

from pathlib import Path

import pytest

import mavforge
from mavforge.domain.outcomes import OperationStatus

pytestmark = [pytest.mark.sitl, pytest.mark.timeout(240)]

BOOT_TIMEOUT = 60.0


def test_backup_then_diff_shows_no_configuration_changes(
    sitl_connection: str, tmp_path: Path
) -> None:
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        backup = vehicle.params.backup(tmp_path / "sitl.json", param_file=tmp_path / "sitl.param")
        diff = vehicle.params.diff(tmp_path / "sitl.json")
    assert backup.status is OperationStatus.SUCCEEDED
    assert backup.param_count > 500
    assert diff.status is OperationStatus.SUCCEEDED
    assert diff.warnings == []
    assert diff.only_in_reference == []
    assert diff.only_in_current == []
    # Anything that changed between two reads seconds apart must be volatile.
    assert [c.name for c in diff.changed if not c.volatile] == []


def test_exported_param_file_matches_vehicle(sitl_connection: str, tmp_path: Path) -> None:
    with mavforge.connect(sitl_connection, timeout=BOOT_TIMEOUT) as vehicle:
        vehicle.params.backup(tmp_path / "b.json", param_file=tmp_path / "b.param")
    result = mavforge.diff_backups(tmp_path / "b.json", tmp_path / "b.param")
    assert result.changed == []
    assert result.only_in_reference == []
    assert result.only_in_current == []
