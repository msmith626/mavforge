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


SAMPLE_METADATA = (Path(__file__).parent / "fixtures" / "apm.pdef.sample.xml").read_bytes()


@pytest.fixture(autouse=True)
def no_network_metadata(request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch) -> None:
    """Serve the sample apm.pdef.xml instead of downloading (except SITL/hardware tests)."""
    if request.node.get_closest_marker("sitl") or request.node.get_closest_marker("hardware"):
        return
    from mavforge.platforms.ardupilot import metadata

    def sample_fetcher(url: str) -> bytes:
        return SAMPLE_METADATA

    monkeypatch.setattr(metadata, "default_fetcher", sample_fetcher)


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
