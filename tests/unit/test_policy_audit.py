import json
import stat
from pathlib import Path

import pytest

from mavforge.domain.errors import NotPermittedError
from mavforge.policy.audit import AuditLog, redact
from mavforge.policy.permissions import Permission, Policy


def test_default_policy_is_read_only() -> None:
    policy = Policy.read_only()
    policy.require(Permission.READ, "status")
    for permission in (Permission.CONFIGURE, Permission.CALIBRATE, Permission.MAINTENANCE):
        assert not policy.allows(permission)
        with pytest.raises(NotPermittedError) as info:
            policy.require(permission, "something")
        assert info.value.details["required"] == permission.value


def test_audit_log_appends_json_lines_with_owner_only_permissions(tmp_path: Path) -> None:
    path = tmp_path / "nested" / "audit.jsonl"
    log = AuditLog(path)
    log.record("operation", operation="info", connection="ws://user:secret@host:1")
    log.record("operation", operation="status")
    lines = [json.loads(line) for line in path.read_text().splitlines()]
    assert [e["operation"] for e in lines] == ["info", "status"]
    assert "secret" not in path.read_text()
    assert stat.S_IMODE(path.stat().st_mode) == 0o600


def test_disabled_audit_log_writes_nothing(tmp_path: Path) -> None:
    AuditLog.disabled().record("operation", operation="info")
    assert list(tmp_path.iterdir()) == []


def test_redact() -> None:
    assert redact("tcp://alice:pw@10.0.0.1:5760") == "tcp://***@10.0.0.1:5760"
    assert redact("tcp:127.0.0.1:5760") == "tcp:127.0.0.1:5760"
