from datetime import UTC, datetime

import pytest

from mavforge.domain.errors import InvalidInputError
from mavforge.domain.models import (
    BackupParam,
    FirmwareVersion,
    ParamBackup,
    ParamType,
    VehicleIdentity,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.services.backup import parse_param_file, render_param_file, resolve_backup_id
from mavforge.services.diff import compare, identity_warnings


def ident(fw: tuple[int, int, int] = (4, 6, 2), uid: str = "aa", sysid: int = 1) -> VehicleIdentity:
    return VehicleIdentity(
        system_id=sysid,
        component_id=1,
        autopilot="ARDUPILOTMEGA",
        mav_type="FIXED_WING",
        vehicle_family="plane",
        platform="ardupilot",
        firmware=FirmwareVersion.from_packed((fw[0] << 24) | (fw[1] << 16) | (fw[2] << 8) | 255),
        uid=uid,
    )


def backup(
    params: dict[str, float], vehicle: VehicleIdentity | None = None, complete: bool = True
) -> ParamBackup:
    return ParamBackup(
        created_at=datetime(2026, 10, 21, tzinfo=UTC),
        mavforge_version="test",
        vehicle=vehicle if vehicle is not None else ident(),
        source="test",
        complete=complete,
        missing_indices=[] if complete else [3],
        params=[
            BackupParam(
                name=n, value=v, type=ParamType.REAL32 if isinstance(v, float) else ParamType.INT16
            )
            for n, v in params.items()
        ],
    )


def run(ref: ParamBackup, cur: ParamBackup):
    return compare(
        ref,
        cur,
        reference_label="ref",
        current_label="cur",
        is_volatile=lambda n: n.startswith("STAT_"),
    )


def test_compare_classifies_parameters() -> None:
    result = run(
        backup({"A": 1, "B": 0.5, "OLD": 3, "STAT_BOOTCNT": 4}),
        backup({"A": 2, "B": 0.5, "NEW": 7, "STAT_BOOTCNT": 5}),
    )
    assert result.status is OperationStatus.SUCCEEDED
    assert [(c.name, c.reference, c.current, c.volatile) for c in result.changed] == [
        ("A", 1, 2, False),
        ("STAT_BOOTCNT", 4, 5, True),
    ]
    assert [p.name for p in result.only_in_reference] == ["OLD"]
    assert [p.name for p in result.only_in_current] == ["NEW"]
    assert result.unchanged == 1
    assert result.warnings == []


def test_float32_noise_is_not_a_difference() -> None:
    result = run(backup({"X": 0.001}), backup({"X": 0.0010000000474974513}))
    assert result.changed == []


def test_incomplete_side_makes_result_partial() -> None:
    result = run(backup({"A": 1}), backup({"A": 1}, complete=False))
    assert result.status is OperationStatus.PARTIAL
    assert any("incomplete" in w for w in result.warnings)


@pytest.mark.parametrize(
    ("current", "expected"),
    [
        (ident(), []),
        (ident(fw=(4, 6, 3)), ["patch versions differ"]),
        (ident(fw=(4, 7, 2)), ["may have been renamed"]),
        (ident(uid="bb"), ["board UID differs"]),
        (ident(sysid=2), ["system IDs"]),
    ],
)
def test_identity_warnings(current: VehicleIdentity, expected: list[str]) -> None:
    warnings = identity_warnings(ident(), current)
    assert len(warnings) == len(expected)
    for warning, fragment in zip(warnings, expected, strict=True):
        assert fragment in warning


def test_missing_identity_is_warned() -> None:
    assert "no vehicle identity" in identity_warnings(None, ident())[0]


def test_param_file_round_trip() -> None:
    original = backup({"ARMING_CHECK": 1, "INS_ACCOFFS_X": 0.001, "COMPASS_OFS_Y": -12.5})
    parsed = parse_param_file(render_param_file(original), "x.param")
    assert parsed.vehicle is None
    assert {p.name: p.value for p in parsed.params} == {
        "ARMING_CHECK": 1,
        "COMPASS_OFS_Y": -12.5,
        "INS_ACCOFFS_X": 0.001,
    }


def test_param_file_accepts_common_dialects() -> None:
    text = "# comment\nFOO,1\nBAR 2.5  # trailing comment\n\nBAZ\t-3\n"
    parsed = parse_param_file(text, "x.param")
    assert {p.name: p.value for p in parsed.params} == {"FOO": 1, "BAR": 2.5, "BAZ": -3}


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("FOO\n", ":1:"),
        ("OK 1\nBAD-NAME 2\n", ":2:"),
        ("FOO abc\n", "invalid value"),
        ("# only\n", "no parameters"),
    ],
)
def test_param_file_errors_name_the_line(text: str, message: str) -> None:
    with pytest.raises(InvalidInputError, match=message):
        parse_param_file(text, "x.param")


@pytest.mark.parametrize("bad", ["../x.json", "/etc/passwd", "a/b.json", ".hidden.json", "x y", ""])
def test_backup_ids_cannot_escape_the_backup_directory(bad: str) -> None:
    with pytest.raises(InvalidInputError):
        resolve_backup_id(bad)


def test_backup_id_extension_is_optional() -> None:
    assert resolve_backup_id("20261021T000000Z-sys1-plane-4.6.2").name.endswith("4.6.2.json")
