"""CLI behaviour: output formats, exit codes and error reporting."""

import io
import json
import threading
import time
from datetime import UTC, datetime
from pathlib import Path

import pytest
import typer.testing
from typer.testing import CliRunner, Result

from mavforge.cli.app import app
from mavforge.domain.errors import ExitCode
from mavforge.domain.models import ParamType
from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle, default_copter_params
from mavforge.testing.scenarios import unconfigured_plane_params
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
    assert ops["calibrate"]["available"] is True
    assert ops["calibrate"]["modifies_vehicle"] is True
    assert ops["calibrate"]["permission"] == "calibrate"


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


def test_diagnose_output_and_fail_on(make_fake: FakeFactory) -> None:
    fake = make_fake(
        mav_type=mavlink.MAV_TYPE_FIXED_WING,
        params=unconfigured_plane_params(),
        prearm_checks_enabled=False,
    )
    base = ("-c", fake.connection_string, "diagnose", "--prearm-wait", "0.5")
    code, stdout, _ = invoke(*base)
    assert code == ExitCode.OK
    assert "[FAULT] Primary compass is not calibrated" in stdout
    assert "[WARNING] All pre-arm checks are disabled" in stdout
    assert "Run compass calibration" in stdout
    assert "[passed]" not in stdout
    assert "airworthy" in stdout

    assert invoke(*base, "--fail-on", "fault")[0] == ExitCode.FAILED
    assert invoke(*base, "--fail-on", "warning")[0] == ExitCode.FAILED
    assert invoke(*base, "--fail-on", "bogus")[0] == ExitCode.USAGE
    code, stdout, _ = invoke(*base, "--show-passed")
    assert "[passed] Accelerometer calibration present" in stdout


def test_diagnose_propose_fixes(make_fake: FakeFactory) -> None:
    fake = make_fake(params=default_copter_params() | {"LOG_BACKEND_TYPE": (0, ParamType.INT8)})
    base = ("-c", fake.connection_string, "diagnose", "--prearm-wait", "0.5")
    code, stdout, _ = invoke(*base)
    assert code == ExitCode.OK
    assert "[WARNING] Onboard logging is disabled" in stdout
    assert "proposes a fix" not in stdout  # not built unless asked

    code, stdout, _ = invoke(*base, "--propose-fixes")
    assert code == ExitCode.OK
    assert "proposes a fix: `mavforge params apply " in stdout


def test_diagnose_fail_on_passes_for_clean_vehicle(fake: FakeVehicle) -> None:
    code, _, _ = invoke(
        "-c", fake.connection_string, "diagnose", "--prearm-wait", "0.5", "--fail-on", "warning"
    )
    assert code == ExitCode.OK


def test_params_set_prompts_and_writes_only_when_confirmed(fake: FakeVehicle) -> None:
    args = ["-c", fake.connection_string, "params", "set", "RC1_TRIM=1490"]
    declined = runner.invoke(app, args, input="n\n", env={"MAVFORGE_CONNECTION": None})
    assert declined.exit_code == ExitCode.FAILED
    assert "not approved" in declined.stderr
    assert "RC1_TRIM" in declined.stderr  # the plan was shown before asking
    accepted = runner.invoke(app, args, input="y\n", env={"MAVFORGE_CONNECTION": None})
    assert accepted.exit_code == ExitCode.OK, accepted.output
    assert "verified" in accepted.stdout
    assert "params restore" in accepted.stdout  # tells the user how to undo


def test_plan_then_approve_then_apply(fake: FakeVehicle) -> None:
    code, stdout, _ = invoke(
        "-c", fake.connection_string, "--json", "params", "plan", "RC1_TRIM=1490"
    )
    assert code == ExitCode.OK
    plan_id = json.loads(stdout)["plan_id"]
    assert invoke("approve", plan_id, "--yes")[0] == ExitCode.OK
    code, stdout, _ = invoke("--json", "params", "plans")
    assert json.loads(stdout)["plans"][0]["approved"] is True
    code, stdout, _ = invoke("-c", fake.connection_string, "--json", "params", "apply", plan_id)
    assert code == ExitCode.OK
    assert json.loads(stdout)["changes"][0]["outcome"] == "verified"


def test_blocked_plan_exits_without_writing(fake: FakeVehicle) -> None:
    code, _, stderr = invoke("-c", fake.connection_string, "params", "set", "MAV_SYSID=7", "--yes")
    assert code == ExitCode.FAILED
    assert "blocking issues" in stderr
    assert not any(m.get_type() == "PARAM_SET" for m in fake.received)


def test_restore_via_cli(fake: FakeVehicle) -> None:
    code, stdout, _ = invoke(
        "-c", fake.connection_string, "--json", "params", "set", "RC1_TRIM=1490", "-y"
    )
    backup = json.loads(stdout)["pre_change_backup"]
    code, stdout, _ = invoke(
        "-c", fake.connection_string, "--json", "params", "restore", backup, "-y"
    )
    assert code == ExitCode.OK
    assert json.loads(stdout)["changes"][0] == {
        "name": "RC1_TRIM",
        "before": 1490,
        "requested": 1500,
        "after": 1500,
        "outcome": "verified",
        "detail": "Verified by read back",
    }


def test_calibrate_gyro_cli(fake: FakeVehicle) -> None:
    code, stdout, stderr = invoke("-c", fake.connection_string, "calibrate", "gyro", "--yes")
    assert code == ExitCode.OK, stderr
    assert "Before you start" in stderr  # the procedure is shown first
    assert "gyro calibration: succeeded" in stdout
    assert "INS_GYROFFS_X" in stdout


def test_calibrate_accel_cli_prompts_for_each_pose(fake: FakeVehicle) -> None:
    result = runner.invoke(
        app,
        ["-c", fake.connection_string, "calibrate", "accel"],
        input="y\n" + "\n" * 6,
        env={"MAVFORGE_CONNECTION": None},
    )
    assert result.exit_code == ExitCode.OK, result.output
    assert result.stderr.count("Press Enter when the vehicle is in position") == 6
    assert "on its BACK" in result.stderr
    assert "accel calibration: succeeded" in result.stdout


def test_calibrate_declined_sends_nothing(fake: FakeVehicle) -> None:
    result = runner.invoke(
        app,
        ["-c", fake.connection_string, "calibrate", "level"],
        input="n\n",
        env={"MAVFORGE_CONNECTION": None},
    )
    assert result.exit_code == ExitCode.FAILED
    assert not any(
        isinstance(m, mavlink.MAVLink_command_long_message)
        and m.command == mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
        for m in fake.received
    )


def test_calibrate_list_needs_no_vehicle() -> None:
    code, stdout, _ = invoke("--json", "calibrate", "list")
    assert code == ExitCode.OK
    kinds = [p["kind"] for p in json.loads(stdout)["procedures"]]
    assert kinds == ["gyro", "level", "compass", "accel", "radio", "orientation"]


class _GatedInput(io.BytesIO):
    """stdin that hands out one line per read, the first only once ``gate`` is set."""

    def __init__(self, data: bytes, gate: threading.Event) -> None:
        super().__init__(data)
        self.gate = gate

    def read(self, size: int | None = -1) -> bytes:
        if size == 0:
            return b""
        self.gate.wait(15)
        return super().readline()

    read1 = read  # type: ignore[assignment]


def _person_at_transmitter(fake: FakeVehicle, gate: threading.Event) -> None:
    """Wait until MAVForge reads RC input, sweep the sticks, centre them, then confirm."""

    def rc_requests() -> int:
        return sum(
            isinstance(m, mavlink.MAVLink_command_long_message)
            and m.command == mavlink.MAV_CMD_REQUEST_MESSAGE
            and int(m.param1) == mavlink.MAVLINK_MSG_ID_RC_CHANNELS
            for m in fake.received
        )

    deadline = time.monotonic() + 10
    while rc_requests() < 3 and time.monotonic() < deadline:
        time.sleep(0.05)
    for frame in ((1080, 1078, 1075, 1084), (1915, 1921, 1926, 1913)):
        fake.set_rc(*frame)
        time.sleep(0.3)
    fake.set_rc(1510, 1513, 1076, 1490)
    gate.set()


def _calibrate_radio(
    monkeypatch: pytest.MonkeyPatch,
    fake: FakeVehicle,
    *options: str,
    data: bytes,
    json_out: bool = False,
) -> Result:
    gate = threading.Event()
    stdin = _GatedInput(data, gate)

    # Typer's runner only takes str/bytes input; give it a stdin that waits for the person
    def input_stream(_input: object, _charset: str) -> _GatedInput:
        return stdin

    monkeypatch.setattr(typer.testing, "make_input_stream", input_stream)
    person = threading.Thread(target=_person_at_transmitter, args=(fake, gate), daemon=True)
    person.start()
    result = runner.invoke(
        app,
        [
            "-c",
            fake.connection_string,
            *(["--json"] if json_out else []),
            "calibrate",
            "radio",
            "--offline",
            *options,
        ],
        env={"MAVFORGE_CONNECTION": None},
    )
    person.join(5)
    return result


def test_calibrate_radio_cli_sweeps_then_applies_after_approval(
    monkeypatch: pytest.MonkeyPatch, fake: FakeVehicle
) -> None:
    result = _calibrate_radio(monkeypatch, fake, data=b"\n\ny\n")
    assert result.exit_code == ExitCode.OK, result.output
    assert "moved: RC1 RC2 RC3 RC4" in result.stderr
    # live progress must not land on the prompt's line
    assert "Press Enter when done (Ctrl-C to cancel).\n" in result.stderr
    assert "cancel):" not in result.stderr
    assert "Write 12 parameter(s)" in result.stderr
    assert "verified" in result.stdout
    assert fake.config.params["RC3_MAX"][0] == 1926


def test_calibrate_radio_plan_only_json_writes_nothing(
    monkeypatch: pytest.MonkeyPatch, fake: FakeVehicle
) -> None:
    result = _calibrate_radio(monkeypatch, fake, "--plan-only", data=b"\n\n", json_out=True)
    assert result.exit_code == ExitCode.OK, result.output
    data = json.loads(result.stdout)
    assert data["result"]["status"] == "succeeded"
    assert len(data["result"]["plan"]["changes"]) == 12
    assert not any(isinstance(m, mavlink.MAVLink_param_set_message) for m in fake.received)


class _PosingInput(io.BytesIO):
    """stdin for the orientation check: before each Enter, put the fake in the next pose."""

    def __init__(self, fake: FakeVehicle, readings: list[tuple[float, float, float]]) -> None:
        super().__init__(b"\n" * len(readings) + b"y\n")
        self.fake = fake
        self.readings = readings

    def read(self, size: int | None = -1) -> bytes:
        if size == 0:
            return b""
        if self.readings:
            self.fake.set_accel(*self.readings.pop(0))
            time.sleep(0.3)
        return super().readline()

    read1 = read  # type: ignore[assignment]


def test_calibrate_orientation_cli_proposes_then_applies(
    monkeypatch: pytest.MonkeyPatch, fake: FakeVehicle
) -> None:
    s, c = 574.0, 819.0  # 35 degrees
    # board arrow toward the right wing, AHRS_ORIENTATION 0: reading = (a_y, -a_x, a_z)
    stdin = _PosingInput(fake, [(0, 0, -1000), (0, -s, -c), (-s, 0, -c)])

    def input_stream(_input: object, _charset: str) -> _PosingInput:
        return stdin

    monkeypatch.setattr(typer.testing, "make_input_stream", input_stream)
    result = runner.invoke(
        app,
        ["-c", fake.connection_string, "calibrate", "orientation", "--offline"],
        env={"MAVFORGE_CONNECTION": None},
    )
    assert result.exit_code == ExitCode.OK, result.output
    # nose up on this board looks like the right side going down
    assert "the autopilot sees: nose up 0°, right side down 35°" in result.stderr
    assert "orientation check: succeeded" in result.stderr
    assert "AHRS_ORIENTATION 2 (Yaw90), not 0 (None)" in result.stderr
    assert "Write 1 parameter(s)" in result.stderr
    assert fake.config.params["AHRS_ORIENTATION"][0] == 2


def test_live_tilt_prints_only_on_a_real_change() -> None:
    from mavforge.cli.app import _progress_changed, _progress_line  # pyright: ignore[reportPrivateUsage]
    from mavforge.domain.models import (
        CalibrationKind,
        CalibrationState,
        CalibrationStatus,
        Tilt,
        VehicleIdentity,
    )

    def status(pitch: float, roll: float) -> CalibrationStatus:
        now = datetime.now(UTC)
        return CalibrationStatus(
            operation_id="x",
            kind=CalibrationKind.ORIENTATION,
            vehicle=VehicleIdentity.model_construct(),
            state=CalibrationState.WAITING_FOR_USER,
            started_at=now,
            updated_at=now,
            tilt=Tilt(pitch_deg=pitch, roll_deg=roll),
        )

    last, _ = _progress_line(status(7.4, 5.0))
    for pitch, roll in ((7.6, 5.2), (8.1, 4.4), (6.9, 6.0)):  # jitter at a rounding edge
        st = status(pitch, roll)
        assert not _progress_changed(st, _progress_line(st)[0], last)
    st = status(12.5, 5.0)
    assert _progress_changed(st, _progress_line(st)[0], last)
