"""Parameter backup and diff against the fake vehicle."""

import json
import stat
from pathlib import Path

import pytest

import mavforge
from mavforge.domain.errors import InvalidInputError
from mavforge.domain.models import ParamType
from mavforge.domain.outcomes import OperationStatus
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle


def test_backup_records_identity_and_is_owner_only(
    fake: FakeVehicle, isolated_state_dir: Path
) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.backup()
    path = Path(result.path)
    assert result.status is OperationStatus.SUCCEEDED
    assert path.parent == isolated_state_dir / "backups"
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert stat.S_IMODE(path.parent.stat().st_mode) == 0o700
    data = json.loads(path.read_text())
    assert data["format"] == "mavforge.param-backup"
    assert data["vehicle"]["firmware"]["major"] == 4
    assert data["complete"] is True
    assert len(data["params"]) == result.param_count == len(fake.config.params)
    listed = mavforge.list_backups()
    assert [b.backup_id for b in listed.backups] == [result.backup_id]


def test_backup_never_overwrites(fake: FakeVehicle, tmp_path: Path) -> None:
    target = tmp_path / "plane.json"
    target.write_text("precious")
    with mavforge.connect(fake.connection_string) as vehicle, pytest.raises(InvalidInputError):
        vehicle.params.backup(target)
    assert target.read_text() == "precious"


def test_param_file_export_matches_backup(fake: FakeVehicle, tmp_path: Path) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.backup(tmp_path / "b.json", param_file=tmp_path / "b.param")
    exported = mavforge.load_backup(tmp_path / "b.param")
    saved = mavforge.load_backup(result.path)
    assert {p.name: p.value for p in exported.params} == {p.name: p.value for p in saved.params}
    assert "\nCOMPASS_OFS_Y    -12.5\n" in (tmp_path / "b.param").read_text()


def test_diff_detects_changes_and_flags_volatile(make_fake: FakeFactory) -> None:
    fake = make_fake()
    fake.set_param("STAT_BOOTCNT", 7, ParamType.INT16)
    with mavforge.connect(fake.connection_string) as vehicle:
        baseline = vehicle.params.backup()
        assert vehicle.params.diff(baseline.backup_id).changed == []

        fake.set_param("ARMING_SKIPCHK", 4, ParamType.INT32)
        fake.set_param("STAT_BOOTCNT", 8, ParamType.INT16)
        fake.remove_param("RC1_TRIM")
        fake.set_param("NEW_PARAM", 1, ParamType.INT8)
        result = vehicle.params.diff(baseline.backup_id)

    assert result.status is OperationStatus.SUCCEEDED
    assert [(c.name, c.reference, c.current, c.volatile) for c in result.changed] == [
        ("ARMING_SKIPCHK", 0, 4, False),
        ("STAT_BOOTCNT", 7, 8, True),
    ]
    assert [p.name for p in result.only_in_reference] == ["RC1_TRIM"]
    assert [p.name for p in result.only_in_current] == ["NEW_PARAM"]
    assert result.warnings == []


def test_diff_against_other_firmware_warns(make_fake: FakeFactory, tmp_path: Path) -> None:
    old = make_fake(firmware=(4, 6, 2, 255))
    new = make_fake(firmware=(4, 7, 2, 255))
    with mavforge.connect(old.connection_string) as vehicle:
        vehicle.params.backup(tmp_path / "old.json")
    with mavforge.connect(new.connection_string) as vehicle:
        result = vehicle.params.diff(tmp_path / "old.json")
    assert any("4.6.2 vs 4.7.2" in w for w in result.warnings)


def test_diff_against_plain_param_file(fake: FakeVehicle, tmp_path: Path) -> None:
    reference = tmp_path / "template.param"
    reference.write_text("FRAME_CLASS 2\nCOMPASS_OFS_X,5\nNOT_ON_VEHICLE 1\n")
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.diff(reference)
    assert [(c.name, c.reference, c.current) for c in result.changed] == [("FRAME_CLASS", 2, 1)]
    assert [p.name for p in result.only_in_reference] == ["NOT_ON_VEHICLE"]
    assert len(result.only_in_current) == len(fake.config.params) - 2
    assert any("no vehicle identity" in w for w in result.warnings)


def test_incomplete_backup_is_partial_and_marked(make_fake: FakeFactory) -> None:
    fake = make_fake(withheld_param_indices={4})
    with mavforge.connect(fake.connection_string) as vehicle:
        result = vehicle.params.backup()
        diff = vehicle.params.diff(result.backup_id)
    assert result.status is OperationStatus.PARTIAL
    assert result.complete is False
    assert mavforge.load_backup(result.path).missing_indices == [4]
    assert diff.status is OperationStatus.PARTIAL


def test_offline_diff_of_two_backups(make_fake: FakeFactory, tmp_path: Path) -> None:
    fake = make_fake()
    with mavforge.connect(fake.connection_string) as vehicle:
        vehicle.params.backup(tmp_path / "a.json")
        fake.set_param("FRAME_TYPE", 3, ParamType.INT8)
        vehicle.params.backup(tmp_path / "b.json")
    result = mavforge.diff_backups(tmp_path / "a.json", tmp_path / "b.json")
    assert [c.name for c in result.changed] == ["FRAME_TYPE"]
    assert result.vehicle is None


def test_invalid_backup_files_are_rejected_and_listed(isolated_state_dir: Path) -> None:
    backups = isolated_state_dir / "backups"
    backups.mkdir(parents=True)
    (backups / "broken.json").write_text('{"format": "something-else"}')
    with pytest.raises(InvalidInputError, match="not a valid MAVForge backup"):
        mavforge.load_backup(backups / "broken.json")
    assert mavforge.list_backups().unreadable == ["broken.json"]


def test_back_to_back_default_backups_get_distinct_names(fake: FakeVehicle) -> None:
    with mavforge.connect(fake.connection_string) as vehicle:
        first = vehicle.params.backup()
        second = vehicle.params.backup()
    assert first.backup_id != second.backup_id
    assert Path(first.path).exists()
    assert Path(second.path).exists()
