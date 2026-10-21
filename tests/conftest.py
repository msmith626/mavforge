from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest

from mavforge.testing.fake_vehicle import FakeFactory, FakeVehicle, FakeVehicleConfig


@pytest.fixture(autouse=True)
def isolated_state_dir(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Keep audit logs and other per-user state out of the developer's home directory."""
    state = tmp_path / "state"
    monkeypatch.setenv("MAVFORGE_STATE_DIR", str(state))
    return state


@pytest.fixture
def make_fake() -> Iterator[FakeFactory]:
    fakes: list[FakeVehicle] = []

    def factory(**config: Any) -> FakeVehicle:
        fake = FakeVehicle(FakeVehicleConfig(**config))
        fakes.append(fake)
        return fake

    yield factory
    for fake in fakes:
        fake.stop()


@pytest.fixture
def fake(make_fake: FakeFactory) -> FakeVehicle:
    return make_fake()
