"""CLI behaviour: output formats, exit codes and error reporting."""

import json

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
