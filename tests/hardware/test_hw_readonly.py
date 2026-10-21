"""Read-only smoke tests against a real ArduPilot flight controller."""

from __future__ import annotations

import pytest

import mavforge
from mavforge.domain.outcomes import OperationStatus

pytestmark = [pytest.mark.hardware, pytest.mark.timeout(300)]


def test_identify_real_flight_controller(hw_connection: str) -> None:
    with mavforge.connect(hw_connection, timeout=15) as vehicle:
        ident = vehicle.identity
        print(f"\n{ident.model_dump_json(indent=2)}")
    assert ident.platform == "ardupilot"
    assert ident.firmware is not None
    assert ident.simulated is False


def test_status_and_full_param_download(hw_connection: str) -> None:
    with mavforge.connect(hw_connection, timeout=15) as vehicle:
        report = vehicle.status()
        params = vehicle.params.list()
    assert report.status in {OperationStatus.SUCCEEDED, OperationStatus.PARTIAL}
    assert report.armed is False, "Disarm the vehicle before running hardware tests"
    assert params.status is OperationStatus.SUCCEEDED, params.missing_indices
    assert params.received == params.total_on_vehicle
