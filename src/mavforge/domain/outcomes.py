"""Structured operation outcomes.

MAVForge never collapses "we sent it" into "it worked". Every result carries an
``OperationStatus`` that distinguishes verified success from partial completion,
rejection, timeouts and states we cannot determine.
"""

from __future__ import annotations

from enum import StrEnum

from mavforge.domain.errors import ExitCode


class OperationStatus(StrEnum):
    SUCCEEDED = "succeeded"
    """Completed and verified with the best evidence available."""
    PARTIAL = "partial"
    """Some, but not all, of the requested work completed; see the result details."""
    FAILED = "failed"
    """The vehicle or MAVForge reported a definite failure."""
    TIMED_OUT = "timed_out"
    """No conclusive response arrived in time."""
    CANCELLED = "cancelled"
    """Cancelled before completion."""
    UNKNOWN = "unknown"
    """The vehicle-side outcome cannot be determined (e.g. link lost mid-operation)."""
    REJECTED = "rejected"
    """The vehicle refused the request (denied or temporarily rejected)."""
    UNSUPPORTED = "unsupported"
    """The vehicle or platform does not support the request."""
    IN_PROGRESS = "in_progress"
    """Accepted and still running."""


_EXIT_CODES: dict[OperationStatus, ExitCode] = {
    OperationStatus.SUCCEEDED: ExitCode.OK,
    OperationStatus.PARTIAL: ExitCode.PARTIAL,
    OperationStatus.FAILED: ExitCode.FAILED,
    OperationStatus.TIMED_OUT: ExitCode.TIMEOUT,
    OperationStatus.CANCELLED: ExitCode.FAILED,
    OperationStatus.UNKNOWN: ExitCode.TIMEOUT,
    OperationStatus.REJECTED: ExitCode.FAILED,
    OperationStatus.UNSUPPORTED: ExitCode.UNSUPPORTED,
    OperationStatus.IN_PROGRESS: ExitCode.OK,
}


def exit_code_for(status: OperationStatus) -> ExitCode:
    return _EXIT_CODES[status]
