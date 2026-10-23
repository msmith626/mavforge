"""Calibration workflow framework: explicit state machines run as background operations.

A workflow never improvises: it follows the firmware's documented sequence, tells the
person at the vehicle exactly what to do (``instruction``), waits for them to confirm
physical steps when the protocol needs it, and decides success only from the vehicle's
terminal signal plus a parameter read-back.
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from datetime import UTC, datetime
from typing import Any

from mavforge.domain.errors import InvalidInputError
from mavforge.domain.models import (
    CalibrationKind,
    CalibrationProcedure,
    CalibrationResult,
    CalibrationState,
    CalibrationStatus,
    CompassCalProgress,
    VehicleIdentity,
)
from mavforge.protocol import params as param_protocol
from mavforge.services.session import Session


class CalibrationCancelled(Exception):  # noqa: N818 - control flow
    """Raised inside a workflow when the user cancels."""


class WorkflowContext:
    """Live, thread-safe status of one calibration, plus its control channels."""

    def __init__(self, session: Session, operation_id: str, kind: CalibrationKind) -> None:
        self.session = session
        self.operation_id = operation_id
        self.kind = kind
        self.started_at = datetime.now(UTC)
        self._lock = threading.Lock()
        self._fields: dict[str, Any] = {"state": CalibrationState.STARTING}
        self._updated_at = self.started_at
        self.cancel_requested = threading.Event()
        self._confirmed = threading.Event()

    # -- status --------------------------------------------------------------------

    def update(self, **fields: Any) -> None:
        with self._lock:
            self._fields.update(fields)
            self._updated_at = datetime.now(UTC)
            if fields.get("awaiting_confirmation"):
                self._confirmed.clear()

    def snapshot(self) -> CalibrationStatus:
        messages = [
            t
            for t in self.session.connection.statustexts(self.session.identity.system_id)
            if t.received_at >= self.started_at
        ]
        with self._lock:
            fields = dict(self._fields)
            updated = self._updated_at
        return CalibrationStatus(
            operation_id=self.operation_id,
            kind=self.kind,
            vehicle=self.session.identity,
            started_at=self.started_at,
            updated_at=updated,
            vehicle_messages=messages[-30:],
            **fields,
        )

    @property
    def state(self) -> CalibrationState:
        with self._lock:
            return self._fields["state"]

    def set_compasses(self, compasses: list[CompassCalProgress]) -> None:
        self.update(compasses=compasses)

    # -- human interaction -------------------------------------------------------------

    def wait_for_confirmation(self, step: str, instruction: str, timeout: float) -> bool:
        """Ask the person to perform a physical step and confirm it. False on timeout."""
        self.update(
            state=CalibrationState.WAITING_FOR_USER,
            instruction=instruction,
            awaiting_confirmation=step,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check_cancelled()
            if self._confirmed.wait(0.1):
                return True
        return False

    def confirm(self, step: str) -> None:
        """Record the person's confirmation. Atomic, so a step cannot be confirmed twice."""
        with self._lock:
            awaiting = self._fields.get("awaiting_confirmation")
            if not awaiting:
                raise InvalidInputError("This calibration is not waiting for a confirmation")
            if step != awaiting:
                raise InvalidInputError(f"Expected confirmation of {awaiting!r}, got {step!r}")
            self._fields.update(
                awaiting_confirmation=None,
                state=CalibrationState.RUNNING,
                instruction="Hold the vehicle still while it samples this position.",
            )
            self._updated_at = datetime.now(UTC)
            self._confirmed.set()

    # -- cancellation ---------------------------------------------------------------

    def check_cancelled(self) -> None:
        if self.cancel_requested.is_set():
            raise CalibrationCancelled

    def sleep(self, seconds: float) -> None:
        if self.cancel_requested.wait(seconds):
            raise CalibrationCancelled

    # -- helpers --------------------------------------------------------------------

    def read_params(self, names: list[str]) -> dict[str, float | int]:
        out: dict[str, float | int] = {}
        for name in names:
            try:
                out[name] = param_protocol.read_param(
                    self.session.connection, *self.session.target, name
                ).value
            except Exception:  # noqa: S112 - absent on this board; evidence is best-effort
                continue
        return out


class Workflow(ABC):
    kind: CalibrationKind
    via_mcp: bool = True
    """False when the protocol needs the person at the vehicle to confirm steps."""

    @abstractmethod
    def procedure(self) -> CalibrationProcedure: ...

    def check_supported(self, identity: VehicleIdentity) -> str | None:
        """None if supported, otherwise the reason it is not."""
        return None

    @abstractmethod
    def run(self, ctx: WorkflowContext) -> CalibrationResult: ...

    def on_cancel(self, ctx: WorkflowContext) -> list[str]:
        """Stop the vehicle-side calibration if possible; return warnings for the user."""
        return []
