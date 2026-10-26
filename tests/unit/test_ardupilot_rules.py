"""ArduPilot diagnostic rules (see docs/research.md, 'Diagnostics rules')."""

from typing import Any

import pytest

from mavforge.domain.models import FindingKind, VehicleIdentity
from mavforge.platforms.ardupilot import rules
from mavforge.platforms.rules import ParamRule, ParamView, run_rule


def ident(family: str) -> VehicleIdentity:
    return VehicleIdentity(
        system_id=1,
        component_id=1,
        autopilot="ARDUPILOTMEGA",
        mav_type="FIXED_WING" if family == "plane" else "QUADROTOR",
        vehicle_family=family,
        platform="ardupilot",
    )


def kinds(rule: ParamRule, params: dict[str, Any], family: str = "plane") -> list[FindingKind]:
    return [f.kind for f in run_rule(rule, ident(family), ParamView(params))]


def titles(rule: ParamRule, params: dict[str, Any], family: str = "plane") -> list[str]:
    return [f.title for f in run_rule(rule, ident(family), ParamView(params))]


@pytest.mark.parametrize(
    ("params", "expected"),
    [
        ({"ARMING_CHECK": 0}, FindingKind.WARNING),
        ({"ARMING_CHECK": 1}, FindingKind.PASSED),
        ({"ARMING_CHECK": 0b1011110}, FindingKind.WARNING),  # 4.6: some bits only
        ({"ARMING_SKIPCHK": 0}, FindingKind.PASSED),  # 4.7: nothing skipped
        ({"ARMING_SKIPCHK": 1 << 2}, FindingKind.WARNING),
    ],
)
def test_arming_checks(params: dict[str, Any], expected: FindingKind) -> None:
    assert kinds(rules.ARMING, params) == [expected]


def test_arming_skipchk_names_skipped_checks() -> None:
    finding = run_rule(
        rules.ARMING, ident("copter"), ParamView({"ARMING_SKIPCHK": (1 << 2) | (1 << 8)})
    )[0]
    assert "Compass" in finding.detail
    assert "Battery Level" in finding.detail


ACCEL_OK = {
    "INS_ACC_ID": 3408138,
    "INS_ACCOFFS_X": 0.68,
    "INS_ACCOFFS_Y": 0.02,
    "INS_ACCOFFS_Z": -0.13,
    "INS_ACCSCAL_X": 1.0,
    "INS_ACCSCAL_Y": 1.0,
    "INS_ACCSCAL_Z": 1.0,
}


def test_accel_calibrated_passes_and_absent_instances_are_skipped() -> None:
    assert kinds(rules.ACCEL, ACCEL_OK) == [FindingKind.PASSED]


def test_accel_zero_offsets_is_a_fault() -> None:
    params = ACCEL_OK | {"INS_ACCOFFS_X": 0, "INS_ACCOFFS_Y": 0, "INS_ACCOFFS_Z": 0}
    assert kinds(rules.ACCEL, params) == [FindingKind.FAULT]


def test_accel_second_imu_checked_when_present() -> None:
    params = ACCEL_OK | {
        "INS_ACC2_ID": 1234,
        **{f"INS_ACC2OFFS_{a}": 0 for a in "XYZ"},
        **{f"INS_ACC2SCAL_{a}": 1 for a in "XYZ"},
    }
    assert titles(rules.ACCEL, params) == ["Accelerometer 2 is not calibrated"]


def test_accel_without_saved_ids_is_not_checked() -> None:
    assert kinds(rules.ACCEL, ACCEL_OK | {"INS_ACC_ID": 0}) == [FindingKind.NOT_CHECKED]


COMPASS_ZERO = {
    "COMPASS_USE": 1,
    "COMPASS_DEV_ID": 855297,
    "COMPASS_DEV_ID2": 0,
    "COMPASS_OFS_X": 0,
    "COMPASS_OFS_Y": 0,
    "COMPASS_OFS_Z": 0,
}


@pytest.mark.parametrize(
    ("family", "learn", "expected"),
    [
        ("plane", 0, FindingKind.FAULT),  # the user's ArduPlane 4.6.2 bench case
        ("plane", 3, FindingKind.SUSPECTED),  # in-flight learning: firmware skips the check
        ("copter", 3, FindingKind.FAULT),  # copter always requires configured compasses
    ],
)
def test_uncalibrated_primary_compass(family: str, learn: int, expected: FindingKind) -> None:
    assert kinds(rules.COMPASS, COMPASS_ZERO | {"COMPASS_LEARN": learn}, family) == [expected]


def test_compass_unused_is_not_checked_and_calibrated_passes() -> None:
    assert kinds(rules.COMPASS, COMPASS_ZERO | {"COMPASS_LEARN": 0, "COMPASS_USE": 0}) == [
        FindingKind.NOT_CHECKED
    ]
    calibrated = COMPASS_ZERO | {"COMPASS_LEARN": 0, "COMPASS_OFS_X": 12.0}
    assert kinds(rules.COMPASS, calibrated) == [FindingKind.PASSED]


def rc(**channels: tuple[int, int, int]) -> dict[str, Any]:
    params: dict[str, Any] = {
        "RCMAP_ROLL": 1,
        "RCMAP_PITCH": 2,
        "RCMAP_THROTTLE": 3,
        "RCMAP_YAW": 4,
    }
    for ch in range(1, 5):
        lo, hi, trim = channels.get(f"rc{ch}", (1102, 1895, 1498))
        params |= {f"RC{ch}_MIN": lo, f"RC{ch}_MAX": hi, f"RC{ch}_TRIM": trim}
    return params


def test_rc_rules() -> None:
    assert kinds(rules.RC, rc()) == [FindingKind.PASSED]
    assert kinds(rules.RC, rc(rc2=(1600, 1900, 1500))) == [FindingKind.FAULT]
    defaults = {f"rc{c}": (1100, 1900, 1500) for c in range(1, 5)}
    assert kinds(rules.RC, rc(**defaults)) == [FindingKind.SUSPECTED]


def test_rc_rule_follows_rcmap() -> None:
    params = rc() | {"RCMAP_THROTTLE": 5, "RC5_MIN": 1900, "RC5_MAX": 1100, "RC5_TRIM": 1500}
    assert titles(rules.RC, params) == ["RC5 (throttle) range is inconsistent"]


@pytest.mark.parametrize(
    ("params", "expected"),
    [
        ({"THR_FAILSAFE": 0}, FindingKind.WARNING),
        ({"THR_FAILSAFE": 2}, FindingKind.WARNING),
        ({"THR_FAILSAFE": 1}, FindingKind.PASSED),
        ({"FS_THR_ENABLE": 0}, FindingKind.WARNING),
        ({"FS_THR_ENABLE": 1}, FindingKind.PASSED),
        ({}, FindingKind.NOT_CHECKED),
    ],
)
def test_throttle_failsafe(params: dict[str, Any], expected: FindingKind) -> None:
    assert kinds(rules.THROTTLE_FS, params) == [expected]


@pytest.mark.parametrize(
    ("params", "expected"),
    [
        ({"BATT_MONITOR": 0}, FindingKind.WARNING),
        ({"BATT_MONITOR": 4, "BATT_FS_LOW_ACT": 0, "BATT_FS_CRT_ACT": 0}, FindingKind.WARNING),
        # Action set, but no threshold either -> this session's own bench bug.
        ({"BATT_MONITOR": 4, "BATT_FS_LOW_ACT": 1, "BATT_FS_CRT_ACT": 0}, FindingKind.WARNING),
        (
            {
                "BATT_MONITOR": 4,
                "BATT_FS_LOW_ACT": 1,
                "BATT_FS_CRT_ACT": 0,
                "BATT_LOW_VOLT": 14.0,
            },
            FindingKind.PASSED,
        ),
        # The _MAH threshold counts just as well as voltage.
        (
            {
                "BATT_MONITOR": 4,
                "BATT_FS_LOW_ACT": 1,
                "BATT_FS_CRT_ACT": 0,
                "BATT_LOW_MAH": 300,
            },
            FindingKind.PASSED,
        ),
    ],
)
def test_battery(params: dict[str, Any], expected: FindingKind) -> None:
    assert kinds(rules.BATTERY, params) == [expected]


def test_battery_no_action_does_not_propose_a_specific_one() -> None:
    # No safe default action exists (RTL vs Land vs ... is a judgment call), unlike the
    # thresholdless case below, which also doesn't propose a value (needs cell count).
    result = run_rule(
        rules.BATTERY,
        ident("plane"),
        ParamView({"BATT_MONITOR": 4, "BATT_FS_LOW_ACT": 0, "BATT_FS_CRT_ACT": 0}),
    )
    assert result[0].proposed_changes is None


def test_battery_no_threshold_names_the_action_and_does_not_propose_a_voltage() -> None:
    result = run_rule(
        rules.BATTERY,
        ident("plane"),
        ParamView({"BATT_MONITOR": 4, "BATT_FS_LOW_ACT": 1, "BATT_FS_CRT_ACT": 2}),
    )
    titles_seen = [f.title for f in result]
    assert "BATT_FS_LOW_ACT has no threshold set" in titles_seen
    assert "BATT_FS_CRT_ACT has no threshold set" in titles_seen
    assert all(f.proposed_changes is None for f in result)


@pytest.mark.parametrize(
    ("params", "expected"),
    [
        # Inverted voltage thresholds: ArduPilot's own arming check refuses this.
        ({"BATT_LOW_VOLT": 13.2, "BATT_CRT_VOLT": 14.0}, FindingKind.FAULT),  # low < crit
        ({"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 14.0}, FindingKind.FAULT),  # low == crit
        ({"BATT_LOW_VOLT": 14.0, "BATT_CRT_VOLT": 13.2}, FindingKind.PASSED),  # correctly ordered
        # Same check for the _MAH pair.
        ({"BATT_LOW_MAH": 300, "BATT_CRT_MAH": 500}, FindingKind.FAULT),
        ({"BATT_LOW_MAH": 500, "BATT_CRT_MAH": 300}, FindingKind.PASSED),
        # Firmware only compares a pair when *both* sides are set - one-sided is a
        # different (missing-threshold) problem, already covered above.
        ({"BATT_LOW_VOLT": 13.2, "BATT_CRT_VOLT": 0}, FindingKind.WARNING),
    ],
)
def test_battery_failsafe_threshold_inversion(
    params: dict[str, Any], expected: FindingKind
) -> None:
    base = {"BATT_MONITOR": 4, "BATT_FS_LOW_ACT": 1, "BATT_FS_CRT_ACT": 2}
    assert kinds(rules.BATTERY, base | params) == [expected]


def test_battery_failsafe_inversion_is_checked_even_with_no_action_configured() -> None:
    # Firmware's own check doesn't care whether an action is set, only the raw
    # threshold values - so this fires even where "no action" would otherwise apply.
    result = run_rule(
        rules.BATTERY,
        ident("plane"),
        ParamView(
            {
                "BATT_MONITOR": 4,
                "BATT_FS_LOW_ACT": 0,
                "BATT_FS_CRT_ACT": 0,
                "BATT_LOW_VOLT": 13.0,
                "BATT_CRT_VOLT": 14.0,
            }
        ),
    )
    assert [f.kind for f in result] == [FindingKind.FAULT]


def test_gps_logging_and_frame() -> None:
    assert kinds(rules.GPS, {"GPS1_TYPE": 0}) == [FindingKind.WARNING]
    assert kinds(rules.GPS, {"GPS_TYPE": 1}) == [FindingKind.PASSED]  # pre-4.6 name
    assert kinds(rules.LOGGING, {"LOG_BACKEND_TYPE": 0}) == [FindingKind.WARNING]
    assert kinds(rules.FRAME, {"FRAME_CLASS": 0}, "copter") == [FindingKind.FAULT]
    assert kinds(rules.FRAME, {}, "plane") == []  # not applicable to planes


@pytest.mark.parametrize(
    ("params", "expected"),
    [
        ({"ARSPD_TYPE": 0, "ARSPD_USE": 0}, FindingKind.PASSED),  # no sensor, not relied on
        ({"ARSPD_TYPE": 2, "ARSPD_USE": 1}, FindingKind.PASSED),  # sensor fitted and used
        ({"ARSPD_TYPE": 0, "ARSPD_USE": 1}, FindingKind.WARNING),  # estimate used as if real
        ({"ARSPD_TYPE": 0, "ARSPD_USE": 2}, FindingKind.WARNING),  # same, throttle=0 variant
    ],
)
def test_airspeed_setup(params: dict[str, Any], expected: FindingKind) -> None:
    assert kinds(rules.AIRSPEED, params) == [expected]


def test_airspeed_setup_not_applicable_to_copters() -> None:
    assert kinds(rules.AIRSPEED, {}, "copter") == []  # no p.require() call at all


def test_airspeed_setup_proposes_disabling_use_not_a_sensor_choice() -> None:
    # Unlike a battery threshold, this fix needs no further judgment call: it's a
    # complete, self-contained reversion to a safe state, same as logging's default.
    result = run_rule(rules.AIRSPEED, ident("plane"), ParamView({"ARSPD_TYPE": 0, "ARSPD_USE": 1}))
    assert result[0].proposed_changes == {"ARSPD_USE": 0}


@pytest.mark.parametrize(
    ("params", "expected"),
    [
        ({"FLTMODE_CH": 0}, FindingKind.SUSPECTED),  # disabled: never set up
        ({"FLTMODE_CH": 5}, FindingKind.PASSED),  # a channel is read (not: the 6 values are right)
        ({}, FindingKind.NOT_CHECKED),
    ],
)
def test_flight_modes(params: dict[str, Any], expected: FindingKind) -> None:
    assert kinds(rules.FLIGHT_MODES, params) == [expected]


def test_logging_proposes_a_safe_default() -> None:
    # No judgment call here (unlike battery failsafe actions): enabling logging has no
    # tradeoff, so it's the one rule allowed to propose a specific value.
    result = run_rule(rules.LOGGING, ident("plane"), ParamView({"LOG_BACKEND_TYPE": 0}))
    assert result[0].proposed_changes == {"LOG_BACKEND_TYPE": 1}


def test_missing_parameters_become_not_checked() -> None:
    result = run_rule(rules.BATTERY, ident("plane"), ParamView({"BATT_MONITOR": 4}))
    assert [f.kind for f in result] == [FindingKind.NOT_CHECKED]
    assert "BATT_FS_LOW_ACT" in result[0].detail


def test_compass_offsets_too_high_mirrors_firmware_limit() -> None:
    calibrated = COMPASS_ZERO | {
        "COMPASS_LEARN": 0,
        "COMPASS_OFS_X": 406.8,
        "COMPASS_OFS_Y": -213.3,
    }
    assert kinds(rules.COMPASS, calibrated | {"COMPASS_OFFS_MAX": 1800}) == [FindingKind.PASSED]
    too_high = calibrated | {"COMPASS_OFS_X": 1900.0, "COMPASS_OFFS_MAX": 1800}
    assert titles(rules.COMPASS, too_high) == ["Compass 1 offsets too high"]
