"""Parameter backups: create, store, list and load.

A backup is a JSON ``ParamBackup`` that records the vehicle identity and firmware next
to every value, so later comparisons and (future) restores can check they are talking
about the same vehicle. Files are created owner-only and are never overwritten.

Plain ``.param`` files (``NAME VALUE`` or ``NAME,VALUE`` per line, ``#`` comments), as
written by Mission Planner, MAVProxy and ArduPilot tools, can be exported and loaded as
references, but they carry no identity.
"""

from __future__ import annotations

import os
import re
from datetime import UTC, datetime
from pathlib import Path

from pydantic import ValidationError

from mavforge.domain.errors import InvalidInputError, OperationTimeoutError, ProtocolError
from mavforge.domain.models import (
    BackupInfo,
    BackupListResult,
    BackupParam,
    BackupResult,
    ParamBackup,
    VehicleIdentity,
)
from mavforge.domain.outcomes import OperationStatus
from mavforge.domain.values import format_value
from mavforge.policy.audit import default_state_dir
from mavforge.protocol import ftp, param_pack
from mavforge.protocol import params as param_protocol
from mavforge.services.catalog import PARAMS_BACKUP, run_operation
from mavforge.services.info import mavforge_version
from mavforge.services.session import Session

MAX_FILE_BYTES = 16 * 1024 * 1024
_BACKUP_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}\.json$")
_PARAM_NAME_RE = re.compile(r"^[A-Za-z0-9_]{1,16}$")


def backup_dir() -> Path:
    return default_state_dir() / "backups"


def describe_vehicle(identity: VehicleIdentity | None) -> str:
    if identity is None:
        return "unknown vehicle (no identity)"
    firmware = f" {identity.firmware}" if identity.firmware else ""
    family = f" ({identity.vehicle_family})" if identity.vehicle_family else ""
    return f"system {identity.system_id} {identity.autopilot} {identity.mav_type}{family}{firmware}"


def default_backup_stem(identity: VehicleIdentity, created: datetime) -> str:
    kind = identity.vehicle_family or identity.mav_type.lower()
    firmware = str(identity.firmware) if identity.firmware else "unknown"
    return f"{created:%Y%m%dT%H%M%SZ}-sys{identity.system_id}-{kind}-{firmware}"


def write_new_default(stem: str, text: str) -> Path:
    """Write to the backup directory under ``stem``, adding -2, -3... if taken."""
    for n in range(1, 100):
        path = backup_dir() / (f"{stem}.json" if n == 1 else f"{stem}-{n}.json")
        try:
            _write_exclusive(path, text)
        except InvalidInputError:
            continue
        return path
    raise InvalidInputError(f"Too many backups named {stem}* in {backup_dir()}")


def _write_exclusive(path: Path, text: str) -> None:
    """Create ``path`` with mode 0600; refuse to overwrite an existing file."""
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        raise InvalidInputError(
            f"{path} already exists; MAVForge does not overwrite backups"
        ) from None
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write(text)


def render_param_file(backup: ParamBackup) -> str:
    """``NAME VALUE`` lines, readable by MAVProxy, Mission Planner and ArduPilot."""
    lines = [
        "# MAVForge parameter backup",
        f"# vehicle: {describe_vehicle(backup.vehicle)}",
        f"# created: {backup.created_at.isoformat()}",
        f"# complete: {'yes' if backup.complete else 'NO - some parameters are missing'}",
    ]
    lines += [f"{p.name:<16} {format_value(p.value)}" for p in backup.params]
    return "\n".join(lines) + "\n"


def fetch_all_params(session: Session) -> param_protocol.ParamDownload:
    """Download the full parameter set, preferring MAVFTP's packed ``param.pck`` (far
    fewer round trips than one PARAM_VALUE per parameter) when the vehicle reports FTP
    support. Falls back to PARAM_REQUEST_LIST on any FTP-specific failure - a missing
    virtual file, a firmware that only claims the capability, or anything else going
    wrong with that specific transfer. A dropped link fails the same way either way,
    so that isn't caught here - there is nothing a fallback could do about it.
    """
    if "FTP" in session.identity.protocol_capabilities:
        try:
            data = ftp.fetch_file(session.connection, *session.target, b"@PARAM/param.pck")
            pack = param_pack.decode(data)
            return param_protocol.ParamDownload(
                params=dict(enumerate(pack.params)), total=pack.total_params
            )
        except (ProtocolError, OperationTimeoutError):
            pass  # fall back below
    return param_protocol.fetch_all(session.connection, *session.target)


def build_backup(session: Session) -> ParamBackup:
    download = fetch_all_params(session)
    if not download.params:
        raise OperationTimeoutError("The vehicle sent no parameters")
    return ParamBackup(
        created_at=datetime.now(UTC),
        mavforge_version=mavforge_version(),
        vehicle=session.identity,
        source=session.connection.description,
        complete=download.complete,
        total_on_vehicle=download.total,
        missing_indices=download.missing,
        params=[
            BackupParam(
                name=p.name, value=p.value, type=p.type, precision_limited=p.precision_limited
            )
            for p in sorted(download.params.values(), key=lambda p: p.name)
        ],
    )


def create_backup(
    session: Session, path: Path | None = None, param_file: Path | None = None
) -> BackupResult:
    """Download all parameters and save them, with identity, as a new JSON backup."""

    def run() -> BackupResult:
        backup = build_backup(session)
        text = backup.model_dump_json(indent=2) + "\n"
        if path is None:
            target = write_new_default(
                default_backup_stem(session.identity, backup.created_at), text
            )
        else:
            target = path
            _write_exclusive(target, text)
        if param_file is not None:
            _write_exclusive(param_file, render_param_file(backup))
        return BackupResult(
            status=OperationStatus.SUCCEEDED if backup.complete else OperationStatus.PARTIAL,
            vehicle=session.identity,
            backup_id=target.name,
            path=str(target),
            param_file=str(param_file) if param_file else None,
            param_count=len(backup.params),
            complete=backup.complete,
        )

    return run_operation(
        session,
        PARAMS_BACKUP,
        run,
        path=str(path) if path else None,
        param_file=str(param_file) if param_file else None,
    )


# -- loading ------------------------------------------------------------------------


def _read_limited(path: Path) -> str:
    try:
        size = path.stat().st_size
        if size > MAX_FILE_BYTES:
            raise InvalidInputError(f"{path} is too large ({size} bytes) to be a parameter file")
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        raise InvalidInputError(f"No such file: {path}") from None
    except (OSError, UnicodeDecodeError) as exc:
        raise InvalidInputError(f"Cannot read {path}: {exc}") from exc


def parse_param_file(text: str, source: str) -> ParamBackup:
    """Parse a plain ``.param`` file into an identity-less backup."""
    params: dict[str, BackupParam] = {}
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = [p for p in re.split(r"[\s,]+", line) if p]
        if len(parts) != 2 or not _PARAM_NAME_RE.match(parts[0]):
            raise InvalidInputError(f"{source}:{lineno}: expected 'NAME VALUE', got {raw!r}")
        try:
            value = float(parts[1])
        except ValueError:
            raise InvalidInputError(f"{source}:{lineno}: invalid value {parts[1]!r}") from None
        params[parts[0]] = BackupParam(
            name=parts[0], value=int(value) if value.is_integer() else value
        )
    if not params:
        raise InvalidInputError(f"{source} contains no parameters")
    return ParamBackup(
        created_at=datetime.fromtimestamp(0, UTC),
        mavforge_version=mavforge_version(),
        vehicle=None,
        source=source,
        complete=True,
        params=sorted(params.values(), key=lambda p: p.name),
    )


def load_backup(path: Path) -> ParamBackup:
    """Load a MAVForge JSON backup or a plain ``.param`` file."""
    text = _read_limited(path)
    if path.suffix.lower() == ".json" or text.lstrip().startswith("{"):
        try:
            return ParamBackup.model_validate_json(text)
        except ValidationError as exc:
            raise InvalidInputError(
                f"{path} is not a valid MAVForge backup (format/schema mismatch)",
                details={"errors": exc.errors(include_url=False, include_input=False)[:5]},
            ) from None
    return parse_param_file(text, path.name)


def resolve_backup_id(backup_id: str) -> Path:
    """Map a backup id (file name in the backup directory) to a path, rejecting traversal."""
    if not backup_id.endswith(".json"):
        backup_id += ".json"
    if not _BACKUP_ID_RE.match(backup_id):
        raise InvalidInputError(
            f"Invalid backup id {backup_id!r}: use a file name from params_backups"
        )
    return backup_dir() / backup_id


def resolve_reference(reference: str | Path) -> Path:
    """A filesystem path if it exists, otherwise a backup id. For human interfaces only."""
    path = Path(reference)
    if path.exists():
        return path
    return resolve_backup_id(str(reference))


def list_backups() -> BackupListResult:
    directory = backup_dir()
    infos: list[BackupInfo] = []
    unreadable: list[str] = []
    for path in sorted(directory.glob("*.json")) if directory.is_dir() else []:
        try:
            backup = load_backup(path)
        except InvalidInputError:
            unreadable.append(path.name)
            continue
        infos.append(
            BackupInfo(
                backup_id=path.name,
                path=str(path),
                created_at=backup.created_at,
                system_id=backup.vehicle.system_id if backup.vehicle else None,
                vehicle=describe_vehicle(backup.vehicle),
                param_count=len(backup.params),
                complete=backup.complete,
            )
        )
    return BackupListResult(
        status=OperationStatus.SUCCEEDED,
        directory=str(directory),
        backups=infos,
        unreadable=unreadable,
    )
