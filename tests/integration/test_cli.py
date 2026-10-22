"""CLI behaviour: output formats, exit codes and error reporting."""

import json
from pathlib import Path

from typer.testing import CliRunner

from mavforge.cli.app import app
from mavforge.domain.errors import ExitCode
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle
from mavforge.transport.dialect import mavlink

runner = CliRunner()


def invoke(*args: str) -> tuple[int, str, str]:
    result = runner.invoke(app, list(args), env={"MAVFORGE_CONNECTION": None})
    return result.exit_code, result.stdout, result.stderr


def test_info_json(fake: FakeVehicle) -> None:
    code, stdout, _ = invoke("-c", fake.connection_string, "--json", "info")
    assert code == ExitCode.OK
    data = json.loads(stdout)
    assert data["status"] == "succeeded"
    assert data["vehicle"]["firmware"]["major"] == 4


def test_params_get_human_output(fake: FakeVehicle) -> None:
    code, stdout, _ = invoke("-c", fake.connection_string, "params", "get", "rc1_trim")
    assert code == ExitCode.OK
    assert "RC1_TRIM = 1500" in stdout


def test_partial_result_exit_code(make_fake: FakeFactory) -> None:
    fake = make_fake(unanswered_message_ids={mavlink.MAVLINK_MSG_ID_SYS_STATUS})
    code, stdout, _ = invoke("-c", fake.connection_string, "--json", "status")
    assert code == ExitCode.PARTIAL
    assert json.loads(stdout)["unavailable"] == ["SYS_STATUS"]


def test_missing_connection_is_usage_error() -> None:
    code, _, stderr = invoke("info")
    assert code == ExitCode.USAGE
    assert "MAVFORGE_CONNECTION" in stderr


def test_connection_from_environment(fake: FakeVehicle) -> None:
    result = runner.invoke(
        app, ["--json", "info"], env={"MAVFORGE_CONNECTION": fake.connection_string}
    )
    assert result.exit_code == ExitCode.OK


def test_rejected_connection_string(fake: FakeVehicle) -> None:
    code, stdout, _ = invoke("-c", "/bin/sh", "--json", "info")
    assert code == ExitCode.USAGE
    assert json.loads(stdout)["error"] == "invalid_input"


def test_no_vehicle_exit_code(make_fake: FakeFactory) -> None:
    fake = make_fake(silent=True)
    code, _, stderr = invoke("-c", fake.connection_string, "--timeout", "0.5", "info")
    assert code == ExitCode.CONNECTION
    assert "No heartbeat" in stderr


def test_param_not_found_json_error(fake: FakeVehicle) -> None:
    code, stdout, _ = invoke("-c", fake.connection_string, "--json", "params", "get", "NOPE")
    assert code == ExitCode.FAILED
    assert json.loads(stdout)["error"] == "param_not_found"


def test_vehicle_text_cannot_inject_markup_or_escapes(make_fake: FakeFactory) -> None:
    fake = make_fake(statustext_on_connect=["PreArm: [red]x\x1b[2J"])
    code, stdout, _ = invoke("-c", fake.connection_string, "status")
    assert code == ExitCode.OK
    assert "PreArm: [red]x?[2J" in stdout
    assert "\x1b" not in stdout


def test_capabilities_needs_no_connection() -> None:
    code, stdout, _ = invoke("--json", "capabilities")
    assert code == ExitCode.OK
    ops = {op["name"]: op for op in json.loads(stdout)["operations"]}
    assert ops["params_get"]["available"] is True
    assert ops["calibrate"]["available"] is False


def test_backup_and_diff_human_output(make_fake: FakeFactory, tmp_path: Path) -> None:
    from mavforge.domain.models import ParamType

    fake = make_fake()
    fake.set_param("STAT_BOOTCNT", 1, ParamType.INT16)
    baseline = tmp_path / "base.json"
    code, stdout, _ = invoke("-c", fake.connection_string, "params", "backup", "-o", str(baseline))
    assert code == ExitCode.OK
    assert f"to {baseline}" in stdout.replace("\n", "")

    fake.set_param("FRAME_TYPE", 3, ParamType.INT8)
    fake.set_param("STAT_BOOTCNT", 2, ParamType.INT16)
    code, stdout, _ = invoke("-c", fake.connection_string, "params", "diff", str(baseline))
    assert code == ExitCode.OK
    assert "FRAME_TYPE" in stdout
    assert "STAT_BOOTCNT" not in stdout
    assert "1 volatile change(s) hidden" in stdout

    code, stdout, _ = invoke("-c", fake.connection_string, "params", "diff", "--all", str(baseline))
    assert "STAT_BOOTCNT" in stdout


def test_offline_diff_needs_no_connection(fake: FakeVehicle, tmp_path: Path) -> None:
    invoke("-c", fake.connection_string, "params", "backup", "-o", str(tmp_path / "a.json"))
    (tmp_path / "b.param").write_text("FRAME_CLASS 2\n")
    code, stdout, stderr = invoke(
        "--json", "params", "diff", str(tmp_path / "b.param"), "--against", str(tmp_path / "a.json")
    )
    assert code == ExitCode.OK, stderr
    data = json.loads(stdout)
    assert [c["name"] for c in data["changed"]] == ["FRAME_CLASS"]


def test_backups_list_and_bad_reference(fake: FakeVehicle) -> None:
    invoke("-c", fake.connection_string, "params", "backup")
    code, stdout, _ = invoke("--json", "params", "backups")
    assert code == ExitCode.OK
    assert len(json.loads(stdout)["backups"]) == 1
    code, _, stderr = invoke("-c", fake.connection_string, "params", "diff", "no-such-backup")
    assert code == ExitCode.USAGE
    assert "No such file" in stderr
    code, _, stderr = invoke("-c", fake.connection_string, "params", "diff", "../escape")
    assert code == ExitCode.USAGE
    assert "Invalid backup id" in stderr
