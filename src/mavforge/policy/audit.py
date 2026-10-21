"""Append-only JSON-lines audit log.

Every operation records who asked (interface), what was asked, which vehicle answered
and how it ended. Entries never include credentials; connection strings are redacted.
The log file is created with owner-only permissions.
"""

from __future__ import annotations

import json
import os
import re
import threading
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

_CREDENTIALS_RE = re.compile(r"//[^/@\s]+@")


def redact(text: str) -> str:
    """Strip ``user:password@`` style credentials from URLs and connection strings."""
    return _CREDENTIALS_RE.sub("//***@", text)


def default_state_dir() -> Path:
    if env := os.environ.get("MAVFORGE_STATE_DIR"):
        return Path(env)
    base = os.environ.get("XDG_STATE_HOME") or str(Path.home() / ".local" / "state")
    return Path(base) / "mavforge"


class AuditLog:
    def __init__(self, path: Path | None) -> None:
        """``path=None`` disables auditing (only permitted for read-only use)."""
        self.path = path
        self._lock = threading.Lock()

    @classmethod
    def default(cls) -> AuditLog:
        return cls(default_state_dir() / "audit.jsonl")

    @classmethod
    def disabled(cls) -> AuditLog:
        return cls(None)

    @property
    def enabled(self) -> bool:
        return self.path is not None

    def record(self, event: str, **fields: Any) -> None:
        """Append one entry. Raises ``OSError`` if the log cannot be written."""
        if self.path is None:
            return
        entry = {"ts": datetime.now(UTC).isoformat(), "event": event, "pid": os.getpid(), **fields}
        line = redact(json.dumps(entry, default=str, sort_keys=True)) + "\n"
        with self._lock:
            self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            fd = os.open(self.path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
            with os.fdopen(fd, "a", encoding="utf-8") as fh:
                fh.write(line)
