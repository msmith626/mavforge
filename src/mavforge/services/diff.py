"""Parameter comparison against a baseline (backup or .param file).

A diff is only as meaningful as the match between the two sides, so every result
carries warnings when the vehicle, board or firmware differ, or when either side is
incomplete. Values are compared as the vehicle stores them (float32 / integer).
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from mavforge.domain.errors import UnsupportedError
from mavforge.domain.models import (
    BackupParam,
    ParamBackup,
    ParamChange,
    ParamDiffResult,
    ParamPresence,
    VehicleIdentity,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.domain.values import values_equal
from mavforge.platforms.ardupilot.adapter import ArduPilotAdapter
from mavforge.platforms.base import PlatformAdapter
from mavforge.platforms.generic import GenericAdapter
from mavforge.protocol import ftp, param_pack
from mavforge.services.backup import build_backup, describe_vehicle, load_backup
from mavforge.services.catalog import PARAMS_DIFF, PARAMS_DIFF_DEFAULTS, run_operation
from mavforge.services.session import Session


def identity_warnings(
    reference: VehicleIdentity | None, current: VehicleIdentity | None
) -> list[str]:
    if reference is None or current is None:
        side = "baseline" if reference is None else "current side"
        return [
            f"The {side} has no vehicle identity (plain .param file); MAVForge cannot "
            "confirm both sides belong to the same vehicle type or firmware."
        ]
    warnings: list[str] = []
    if (reference.autopilot, reference.vehicle_family) != (
        current.autopilot,
        current.vehicle_family,
    ):
        warnings.append(
            f"Different vehicle kinds: baseline is {describe_vehicle(reference)}, "
            f"current is {describe_vehicle(current)}."
        )
    ref_fw, cur_fw = reference.firmware, current.firmware
    if ref_fw is None or cur_fw is None:
        warnings.append("Firmware version unknown on one side; parameter names may differ.")
    elif (ref_fw.major, ref_fw.minor) != (cur_fw.major, cur_fw.minor):
        warnings.append(
            f"Firmware differs ({ref_fw} vs {cur_fw}): parameters may have been renamed, "
            "added or changed meaning between these versions."
        )
    elif str(ref_fw) != str(cur_fw):
        warnings.append(f"Firmware patch versions differ ({ref_fw} vs {cur_fw}).")
    if reference.uid and current.uid and reference.uid != current.uid:
        warnings.append("Different flight-controller board (board UID differs).")
    if reference.system_id != current.system_id:
        warnings.append(
            f"Different MAVLink system IDs ({reference.system_id} vs {current.system_id})."
        )
    return warnings


def _adapter_for(*identities: VehicleIdentity | None) -> PlatformAdapter:
    if any(i is not None and i.platform == ArduPilotAdapter.name for i in identities):
        return ArduPilotAdapter()
    return GenericAdapter()


def compare(
    reference: ParamBackup,
    current: ParamBackup,
    *,
    reference_label: str,
    current_label: str,
    is_volatile: Callable[[str], bool],
    vehicle: VehicleIdentity | None = None,
) -> ParamDiffResult:
    """Pure comparison of two parameter sets."""
    ref = {p.name: p for p in reference.params}
    cur = {p.name: p for p in current.params}
    changed: list[ParamChange] = []
    unchanged = 0
    for name in sorted(ref.keys() & cur.keys()):
        r, c = ref[name], cur[name]
        ptype = c.type or r.type
        if values_equal(r.value, c.value, ptype):
            unchanged += 1
        else:
            changed.append(
                ParamChange(
                    name=name,
                    reference=r.value,
                    current=c.value,
                    type=ptype,
                    volatile=is_volatile(name),
                )
            )

    def presence(params: dict[str, BackupParam], names: set[str]) -> list[ParamPresence]:
        return [
            ParamPresence(name=n, value=params[n].value, volatile=is_volatile(n))
            for n in sorted(names)
        ]

    warnings = identity_warnings(reference.vehicle, current.vehicle)
    for label, side in ((reference_label, reference), (current_label, current)):
        if not side.complete:
            warnings.append(
                f"{label} is incomplete ({len(side.missing_indices)} parameters missing); "
                "parameters reported as absent may simply not have arrived."
            )
    complete = reference.complete and current.complete
    return ParamDiffResult(
        status=OperationStatus.SUCCEEDED if complete else OperationStatus.PARTIAL,
        reference=reference_label,
        current=current_label,
        vehicle=vehicle,
        changed=changed,
        only_in_reference=presence(ref, ref.keys() - cur.keys()),
        only_in_current=presence(cur, cur.keys() - ref.keys()),
        unchanged=unchanged,
        warnings=warnings,
    )


def diff_vehicle(session: Session, reference_path: Path) -> ParamDiffResult:
    """Compare the live vehicle against a backup or .param file."""
    reference = load_backup(reference_path)

    def run() -> ParamDiffResult:
        current = build_backup(session)
        return compare(
            reference,
            current,
            reference_label=reference_path.name,
            current_label="vehicle",
            is_volatile=session.adapter.is_volatile_param,
            vehicle=session.identity,
        )

    return run_operation(session, PARAMS_DIFF, run, reference=reference_path.name)


def diff_against_defaults(session: Session) -> ParamDiffResult:
    """Compare the live vehicle against the firmware's own shipped defaults.

    Needs MAVFTP: there is no PARAM_REQUEST_LIST equivalent of "what is this
    parameter's default", so this raises ``UnsupportedError`` rather than falling
    back, unlike ``fetch_all_params()``'s silent fallback for the plain download.

    One request answers the whole comparison: ``@PARAM/param.pck?withdefaults=1``
    carries every current value, and - per ArduPilot's own encoder
    (``AP_Filesystem_Param.cpp::pack_param()``) - a default only for entries where it
    differs from the current value. A name absent from that set already equals its
    default; nothing further needs fetching or comparing.
    """

    def run() -> ParamDiffResult:
        if "FTP" not in session.identity.protocol_capabilities:
            raise UnsupportedError(
                "This vehicle does not report MAVFTP support, which this comparison "
                "needs (there is no PARAM_REQUEST_LIST equivalent of 'what is the "
                "default for this parameter')."
            )
        data = ftp.fetch_file(
            session.connection, *session.target, b"@PARAM/param.pck?withdefaults=1"
        )
        pack = param_pack.decode(data)
        values = {p.name: p for p in pack.params}
        is_volatile = session.adapter.is_volatile_param
        changed = [
            ParamChange(
                name=name,
                reference=default,
                current=values[name].value,
                type=values[name].type,
                volatile=is_volatile(name),
            )
            for name, default in pack.defaults.items()
        ]
        changed.sort(key=lambda c: c.name)
        return ParamDiffResult(
            status=OperationStatus.SUCCEEDED,
            reference="firmware defaults",
            current="vehicle",
            vehicle=session.identity,
            changed=changed,
            only_in_reference=[],
            only_in_current=[],
            unchanged=len(pack.params) - len(changed),
            warnings=[],
        )

    return run_operation(session, PARAMS_DIFF_DEFAULTS, run)


def diff_files(reference_path: Path, current_path: Path) -> ParamDiffResult:
    """Compare two saved files. No vehicle is contacted."""
    reference, current = load_backup(reference_path), load_backup(current_path)
    adapter = _adapter_for(reference.vehicle, current.vehicle)
    return compare(
        reference,
        current,
        reference_label=reference_path.name,
        current_label=current_path.name,
        is_volatile=adapter.is_volatile_param,
    )
