"""Parameter inspection services (read-only in this milestone)."""

from __future__ import annotations

from fnmatch import fnmatchcase

from mavforge.domain.errors import InvalidInputError, OperationTimeoutError
from mavforge.domain.models import ParamGetResult, ParamListResult
from mavforge.domain.outcomes import OperationStatus
from mavforge.protocol import params as param_protocol
from mavforge.services.catalog import PARAMS_GET, PARAMS_LIST, run_operation
from mavforge.services.session import Session


def _check_pattern(pattern: str) -> str:
    pattern = pattern.strip()
    if not pattern or len(pattern) > 64:
        raise InvalidInputError("Parameter pattern must be 1-64 characters")
    return pattern


def list_params(session: Session, pattern: str | None = None) -> ParamListResult:
    """Download every parameter; filter by a case-insensitive glob such as ``COMPASS_*``."""
    if pattern is not None:
        pattern = _check_pattern(pattern)

    def run() -> ParamListResult:
        download = param_protocol.fetch_all(session.connection, *session.target)
        if not download.params:
            raise OperationTimeoutError("The vehicle sent no parameters")
        params = sorted(download.params.values(), key=lambda p: p.name)
        if pattern is not None:
            upper = pattern.upper()
            params = [p for p in params if fnmatchcase(p.name.upper(), upper)]
        return ParamListResult(
            status=OperationStatus.SUCCEEDED if download.complete else OperationStatus.PARTIAL,
            vehicle=session.identity,
            pattern=pattern,
            params=params,
            total_on_vehicle=download.total,
            received=len(download.params),
            missing_indices=download.missing,
        )

    return run_operation(session, PARAMS_LIST, run, pattern=pattern)


def get_param(session: Session, name: str) -> ParamGetResult:
    name = session.adapter.normalize_param_name(name)

    def run() -> ParamGetResult:
        value = param_protocol.read_param(
            session.connection,
            *session.target,
            name,
            timeout=session.options.request_timeout,
        )
        return ParamGetResult(
            status=OperationStatus.SUCCEEDED, vehicle=session.identity, param=value
        )

    return run_operation(session, PARAMS_GET, run, param=name)
