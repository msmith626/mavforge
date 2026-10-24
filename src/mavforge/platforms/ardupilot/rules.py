"""ArduPilot configuration rules for diagnostics.

Each rule mirrors firmware behaviour or published parameter metadata, cited in
docs/research.md ("Diagnostics rules"). FAULT is used only where ArduPilot's own
pre-arm logic would reject the configuration; inferences are SUSPECTED. These rules
matter most when the vehicle's own checks are disabled (ARMING_CHECK=0).
"""

from __future__ import annotations

import math

from mavforge.domain.models import Finding, FindingKind, VehicleIdentity
from mavforge.platforms.rules import ParamRule, ParamView, finding

# Bit positions from apm.pdef.xml: ARMING_CHECK (Plane 4.6.2), ARMING_SKIPCHK (Copter 4.7.1).
_ARMING_BITS = {
    1: "Barometer",
    2: "Compass",
    3: "GPS lock",
    4: "INS",
    5: "Parameters",
    6: "RC Channels",
    7: "Board voltage",
    8: "Battery Level",
    9: "Airspeed",
    10: "Logging Available",
    11: "Hardware safety switch",
    12: "GPS Configuration",
    13: "System",
    14: "Mission",
    15: "Rangefinder",
    16: "Camera",
    17: "AuxAuth",
    18: "VisualOdometry",
    19: "FFT",
}

# Families whose firmware always requires calibrated compass offsets (AP_Arming.cpp).
_COMPASS_ALWAYS_REQUIRED = {"copter", "heli", "blimp"}
_COMPASS_LEARN_INFLIGHT = 3  # AP_Compass.h: learn_offsets_enabled() == LEARN_INFLIGHT

_RC_DEFAULTS = (1100, 1900, 1500)  # RCn_MIN, RCn_MAX, RCn_TRIM firmware defaults


def _bits(value: float) -> list[str]:
    v = int(value)
    return [name for bit, name in _ARMING_BITS.items() if v & (1 << bit)]


def _arming_checks(identity: VehicleIdentity, p: ParamView) -> list[Finding]:
    name = p.first_present("ARMING_SKIPCHK", "ARMING_CHECK")
    (value,) = p.require(name)
    evidence: dict[str, float | int | str | bool | None] = {name: value}
    advice = "Re-enable all pre-arm checks before flight unless you have a specific reason."
    if name == "ARMING_SKIPCHK":  # ArduPilot 4.7+: bits are checks to *skip*
        skipped = _bits(value)
        if int(value) == 0:
            return [
                finding(
                    ARMING,
                    FindingKind.PASSED,
                    "All pre-arm checks enabled",
                    "ARMING_SKIPCHK=0.",
                    evidence=evidence,
                )
            ]
        return [
            finding(
                ARMING,
                FindingKind.WARNING,
                "Some pre-arm checks are skipped",
                f"ARMING_SKIPCHK skips: {', '.join(skipped) or f'unknown bits ({int(value)})'}.",
                evidence=evidence,
                recommendation=advice + " Set ARMING_SKIPCHK=0.",
            )
        ]
    if int(value) == 0:
        return [
            finding(
                ARMING,
                FindingKind.WARNING,
                "All pre-arm checks are disabled",
                "ARMING_CHECK=0: the firmware will arm without checking sensors, "
                "calibration, RC, GPS or battery. Faults in this report would otherwise "
                "have blocked arming.",
                evidence=evidence,
                recommendation=advice + " Set ARMING_CHECK=1 (All).",
            )
        ]
    if int(value) & 1:
        return [
            finding(
                ARMING,
                FindingKind.PASSED,
                "All pre-arm checks enabled",
                "ARMING_CHECK includes All.",
                evidence=evidence,
            )
        ]
    disabled = [n for bit, n in _ARMING_BITS.items() if not int(value) & (1 << bit)]
    return [
        finding(
            ARMING,
            FindingKind.WARNING,
            "Some pre-arm checks are disabled",
            f"ARMING_CHECK does not include: {', '.join(disabled)}.",
            evidence=evidence,
            recommendation=advice + " Set ARMING_CHECK=1 (All).",
        )
    ]


def _accel_cal(identity: VehicleIdentity, p: ParamView) -> list[Finding]:
    p.require("INS_ACC_ID")
    findings: list[Finding] = []
    instances = [("", 1), ("2", 2), ("3", 3)]
    checked = 0
    for suffix, number in instances:
        id_name = f"INS_ACC{suffix}_ID"
        if not p.has(id_name) or p.get(id_name) == 0:
            continue  # no accelerometer saved for this instance
        offs = p.require(*(f"INS_ACC{suffix}OFFS_{a}" for a in "XYZ"))
        scal = p.require(*(f"INS_ACC{suffix}SCAL_{a}" for a in "XYZ"))
        checked += 1
        evidence: dict[str, float | int | str | bool | None] = {
            id_name: p.get(id_name),
            **{f"INS_ACC{suffix}OFFS_{a}": v for a, v in zip("XYZ", offs, strict=True)},
        }
        if all(v == 0 for v in offs) or all(v == 0 for v in scal):
            findings.append(
                finding(
                    ACCEL,
                    FindingKind.FAULT,
                    f"Accelerometer {number} is not calibrated",
                    "Offsets are exactly zero (or scale is zero). ArduPilot's INS pre-arm "
                    "check rejects this as '3D Accel calibration needed'.",
                    evidence=evidence,
                    recommendation="Run accelerometer calibration (6-position).",
                )
            )
    if checked == 0:
        return [
            finding(
                ACCEL,
                FindingKind.NOT_CHECKED,
                "Accelerometer calibration: not checked",
                "No accelerometer IDs are saved (INS_ACC_ID=0); the board may not have "
                "booted with its IMU detected.",
            )
        ]
    if not findings:
        findings.append(
            finding(
                ACCEL,
                FindingKind.PASSED,
                "Accelerometer calibration present",
                f"{checked} accelerometer(s) have non-zero offsets and scale.",
            )
        )
    return findings


def _compass_cal(identity: VehicleIdentity, p: ParamView) -> list[Finding]:
    use, learn = p.require("COMPASS_USE", "COMPASS_LEARN")
    if use == 0:
        return [
            finding(
                COMPASS,
                FindingKind.NOT_CHECKED,
                "Compass calibration: not checked",
                "COMPASS_USE=0: the primary compass is not used for yaw.",
                evidence={"COMPASS_USE": use},
            )
        ]
    findings: list[Finding] = []
    required = (
        identity.vehicle_family in _COMPASS_ALWAYS_REQUIRED or learn != _COMPASS_LEARN_INFLIGHT
    )
    for suffix, number in (("", 1), ("2", 2), ("3", 3)):
        dev_name = f"COMPASS_DEV_ID{suffix}"
        if not p.has(dev_name) or p.get(dev_name) == 0:
            continue
        offs = p.require(*(f"COMPASS_OFS{suffix}_{a}" for a in "XYZ"))
        length = math.hypot(*offs)
        limit = p.get("COMPASS_OFFS_MAX")
        if limit is not None and length > limit:
            findings.append(
                finding(
                    COMPASS,
                    FindingKind.FAULT,
                    f"Compass {number} offsets too high",
                    f"Offset length {length:.0f} exceeds COMPASS_OFFS_MAX={limit:g}; "
                    "ArduPilot's pre-arm check reports 'Compass offsets too high'. Usually "
                    "magnetic interference near the compass or a bad calibration.",
                    evidence={dev_name: p.get(dev_name), "COMPASS_OFFS_MAX": limit},
                    recommendation="Move the compass away from metal and power wiring, "
                    "then recalibrate.",
                )
            )
            continue
        if length != 0:
            continue
        evidence: dict[str, float | int | str | bool | None] = {
            dev_name: p.get(dev_name),
            "COMPASS_LEARN": learn,
            **{f"COMPASS_OFS{suffix}_{a}": v for a, v in zip("XYZ", offs, strict=True)},
        }
        if number == 1 and required:
            findings.append(
                finding(
                    COMPASS,
                    FindingKind.FAULT,
                    "Primary compass is not calibrated",
                    "The compass has a device ID but all offsets are zero, and offsets are "
                    "not being learned in flight (COMPASS_LEARN != 3). ArduPilot's compass "
                    "pre-arm check rejects an unconfigured compass.",
                    evidence=evidence,
                    recommendation="Run compass calibration (onboard calibration).",
                )
            )
        else:
            reason = (
                "COMPASS_LEARN=3 means offsets are learned in flight"
                if number == 1
                else "this compass may not be in use"
            )
            findings.append(
                finding(
                    COMPASS,
                    FindingKind.SUSPECTED,
                    f"Compass {number} has no calibration offsets",
                    f"Device ID present but offsets are zero; {reason}.",
                    evidence=evidence,
                    recommendation="Calibrate the compass, or disable it if unused.",
                )
            )
    if not findings:
        findings.append(
            finding(
                COMPASS,
                FindingKind.PASSED,
                "Compass calibration present",
                "Every detected compass has non-zero offsets.",
            )
        )
    return findings


def _rc_cal(identity: VehicleIdentity, p: ParamView) -> list[Finding]:
    roles = ("ROLL", "PITCH", "THROTTLE", "YAW")
    channels = [int(c) for c in p.require(*(f"RCMAP_{r}" for r in roles))]
    findings: list[Finding] = []
    defaults = 0
    for role, ch in zip(roles, channels, strict=True):
        lo, hi, trim = p.require(f"RC{ch}_MIN", f"RC{ch}_MAX", f"RC{ch}_TRIM")
        evidence: dict[str, float | int | str | bool | None] = {
            f"RC{ch}_MIN": lo,
            f"RC{ch}_MAX": hi,
            f"RC{ch}_TRIM": trim,
        }
        if lo > trim or hi < trim:
            findings.append(
                finding(
                    RC,
                    FindingKind.FAULT,
                    f"RC{ch} ({role.lower()}) range is inconsistent",
                    "RCn_MIN is above RCn_TRIM or RCn_MAX is below it; ArduPilot's RC "
                    "pre-arm check rejects this when the receiver is connected.",
                    evidence=evidence,
                    recommendation="Redo radio calibration.",
                )
            )
        if (lo, hi, trim) == _RC_DEFAULTS:
            defaults += 1
    if defaults == len(roles):
        findings.append(
            finding(
                RC,
                FindingKind.SUSPECTED,
                "Radio calibration was probably never done",
                "Roll, pitch, throttle and yaw channels all still have the firmware "
                "defaults (MIN 1100, MAX 1900, TRIM 1500).",
                recommendation="Run radio calibration with the transmitter you will fly with.",
            )
        )
    if not findings:
        findings.append(
            finding(
                RC,
                FindingKind.PASSED,
                "Radio calibration present",
                "Primary channels have calibrated, consistent ranges.",
            )
        )
    return findings


def _throttle_failsafe(identity: VehicleIdentity, p: ParamView) -> list[Finding]:
    name = p.first_present("THR_FAILSAFE", "FS_THR_ENABLE")
    (value,) = p.require(name)
    evidence: dict[str, float | int | str | bool | None] = {name: value}
    if value == 0:
        return [
            finding(
                THROTTLE_FS,
                FindingKind.WARNING,
                "RC/throttle failsafe is disabled",
                f"{name}=0: losing the RC link will not trigger a failsafe action.",
                evidence=evidence,
                recommendation=f"Enable {name} and set the failsafe PWM threshold "
                "for your receiver.",
            )
        ]
    if name == "THR_FAILSAFE" and value == 2:
        return [
            finding(
                THROTTLE_FS,
                FindingKind.WARNING,
                "RC failsafe detects loss but takes no action",
                "THR_FAILSAFE=2 (EnabledNoFailsafe).",
                evidence=evidence,
            )
        ]
    return [
        finding(
            THROTTLE_FS,
            FindingKind.PASSED,
            "RC/throttle failsafe enabled",
            f"{name}={value}.",
            evidence=evidence,
        )
    ]


def _battery_failsafe(identity: VehicleIdentity, p: ParamView) -> list[Finding]:
    (monitor,) = p.require("BATT_MONITOR")
    if monitor == 0:
        return [
            finding(
                BATTERY,
                FindingKind.WARNING,
                "No battery monitoring",
                "BATT_MONITOR=0: voltage/capacity are not measured, so low-battery "
                "failsafes cannot trigger.",
                evidence={"BATT_MONITOR": monitor},
                recommendation="Configure BATT_MONITOR for your power module.",
            )
        ]
    low, crit = p.require("BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT")
    evidence: dict[str, float | int | str | bool | None] = {
        "BATT_MONITOR": monitor,
        "BATT_FS_LOW_ACT": low,
        "BATT_FS_CRT_ACT": crit,
    }
    if low == 0 and crit == 0:
        return [
            finding(
                BATTERY,
                FindingKind.WARNING,
                "No battery failsafe action",
                "A battery monitor is configured but both low and critical failsafe "
                "actions are 0 (no action / warn only).",
                evidence=evidence,
                recommendation="Set BATT_FS_LOW_ACT / BATT_FS_CRT_ACT and their thresholds. "
                "MAVForge does not propose specific actions here: which one is right (RTL, "
                "Land, ...) is a judgment call for the pilot, same as flight-mode assignment.",
            )
        ]
    # An action can be configured but still unable to ever trigger, if its threshold was
    # never set: BATT_LOW_VOLT/BATT_CRT_VOLT (or the _MAH equivalents) default to 0, and a
    # configured action with a 0 threshold is exactly as inert as no action at all. Found by
    # hand on this project's own bench vehicle before this check existed (docs/roadmap.md,
    # M8/M9). No specific voltage is proposed: that depends on the pack's cell count, which
    # MAVForge has no way to know, unlike radio/orientation's proposals (which come from live
    # sensor readings, not a guess about hardware MAVForge never asked about).
    no_threshold = []
    for act, act_name, volt_name, mah_name in (
        (low, "BATT_FS_LOW_ACT", "BATT_LOW_VOLT", "BATT_LOW_MAH"),
        (crit, "BATT_FS_CRT_ACT", "BATT_CRT_VOLT", "BATT_CRT_MAH"),
    ):
        if act == 0:
            continue
        volt, mah = p.get(volt_name), p.get(mah_name)
        if not volt and not mah:
            no_threshold.append((act_name, act, volt_name, mah_name, volt, mah))
    if no_threshold:
        return [
            finding(
                BATTERY,
                FindingKind.WARNING,
                f"{act_name} has no threshold set",
                f"{act_name}={act} but both {volt_name} and {mah_name} are 0 (or not "
                "present on this firmware): this action can never trigger. MAVForge does "
                "not propose a value here: the right voltage depends on the pack's cell "
                "count, which MAVForge has no way to know.",
                evidence=evidence | {volt_name: volt, mah_name: mah},
                recommendation=f"Set {volt_name} (or {mah_name}) for your battery's cell count.",
            )
            for act_name, act, volt_name, mah_name, volt, mah in no_threshold
        ]
    return [
        finding(
            BATTERY,
            FindingKind.PASSED,
            "Battery failsafe configured",
            "A low or critical battery action is set, with a threshold that can trigger it.",
            evidence=evidence,
        )
    ]


def _gps_configured(identity: VehicleIdentity, p: ParamView) -> list[Finding]:
    name = p.first_present("GPS1_TYPE", "GPS_TYPE")
    (value,) = p.require(name)
    if value == 0:
        return [
            finding(
                GPS,
                FindingKind.WARNING,
                "No GPS configured",
                f"{name}=0: position-dependent modes and failsafes (RTL) will not work.",
                evidence={name: value},
            )
        ]
    return [
        finding(
            GPS, FindingKind.PASSED, "GPS configured", f"{name}={value}.", evidence={name: value}
        )
    ]


def _logging(identity: VehicleIdentity, p: ParamView) -> list[Finding]:
    (value,) = p.require("LOG_BACKEND_TYPE")
    if value == 0:
        return [
            finding(
                LOGGING,
                FindingKind.WARNING,
                "Onboard logging is disabled",
                "LOG_BACKEND_TYPE=0: no flight logs will be available for analysis.",
                evidence={"LOG_BACKEND_TYPE": value},
                recommendation="Set LOG_BACKEND_TYPE=1 (File).",
                proposed_changes={"LOG_BACKEND_TYPE": 1},
            )
        ]
    return [
        finding(
            LOGGING,
            FindingKind.PASSED,
            "Logging enabled",
            f"LOG_BACKEND_TYPE={value}.",
            evidence={"LOG_BACKEND_TYPE": value},
        )
    ]


def _frame_class(identity: VehicleIdentity, p: ParamView) -> list[Finding]:
    if identity.vehicle_family not in {"copter", "heli"}:
        return []
    (value,) = p.require("FRAME_CLASS")
    if value == 0:
        return [
            finding(
                FRAME,
                FindingKind.FAULT,
                "Frame class is not set",
                "FRAME_CLASS=0 (Undefined): ArduCopter cannot set up its motor outputs.",
                evidence={"FRAME_CLASS": value},
                recommendation="Set FRAME_CLASS and FRAME_TYPE for your airframe, then reboot.",
            )
        ]
    return [
        finding(
            FRAME,
            FindingKind.PASSED,
            "Frame class set",
            f"FRAME_CLASS={value}.",
            evidence={"FRAME_CLASS": value},
        )
    ]


ARMING = ParamRule("ardupilot.arming_checks", "Pre-arm check configuration", _arming_checks)
ACCEL = ParamRule("ardupilot.accel_cal", "Accelerometer calibration", _accel_cal)
COMPASS = ParamRule("ardupilot.compass_cal", "Compass calibration", _compass_cal)
RC = ParamRule("ardupilot.rc_cal", "Radio calibration", _rc_cal)
THROTTLE_FS = ParamRule("ardupilot.throttle_failsafe", "RC/throttle failsafe", _throttle_failsafe)
BATTERY = ParamRule(
    "ardupilot.battery_failsafe", "Battery monitoring and failsafe", _battery_failsafe
)
GPS = ParamRule("ardupilot.gps_configured", "GPS configuration", _gps_configured)
LOGGING = ParamRule("ardupilot.logging", "Onboard logging", _logging)
FRAME = ParamRule("ardupilot.frame_class", "Frame class", _frame_class)

RULES: list[ParamRule] = [ARMING, ACCEL, COMPASS, RC, THROTTLE_FS, BATTERY, GPS, LOGGING, FRAME]
