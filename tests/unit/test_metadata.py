from pathlib import Path

import pytest
from defusedxml import EntitiesForbidden

from mavforge.domain.models import FirmwareVersion, VehicleIdentity
from mavforge.platforms.ardupilot.metadata import (
    candidate_urls,
    default_fetcher,
    load_metadata,
    parse_pdef_xml,
)

SAMPLE = (Path(__file__).parents[1] / "fixtures" / "apm.pdef.sample.xml").read_bytes()


def plane(release: int = 255, family: str = "plane") -> VehicleIdentity:
    return VehicleIdentity(
        system_id=1,
        component_id=1,
        autopilot="ARDUPILOTMEGA",
        mav_type="FIXED_WING",
        vehicle_family=family,
        platform="ardupilot",
        firmware=FirmwareVersion.from_packed((4 << 24) | (6 << 16) | (2 << 8) | release),
    )


def test_parse_fields() -> None:
    meta = parse_pdef_xml(SAMPLE)
    assert meta["THR_FAILSAFE"].values == {
        0.0: "Disabled",
        1.0: "Enabled",
        2.0: "EnabledNoFailsafe",
    }
    assert meta["ARMING_CHECK"].bitmask == {0: "All", 1: "Barometer", 2: "Compass", 3: "GPS lock"}
    assert meta["RC1_MIN"].range == (800.0, 2200.0)
    assert meta["RC1_MIN"].units == "PWM"
    assert meta["SERIAL1_PROTOCOL"].reboot_required
    assert meta["INS_ACC_ID"].read_only
    assert meta["BARO1_GND_PRESS"].volatile


def test_hostile_xml_is_rejected() -> None:
    bomb = (
        b'<?xml version="1.0"?><!DOCTYPE x [<!ENTITY a "aaaa"><!ENTITY b "&a;&a;&a;">]><x>&b;</x>'
    )
    with pytest.raises(EntitiesForbidden):
        parse_pdef_xml(bomb)


def test_candidate_urls() -> None:
    urls = candidate_urls(plane())
    assert urls[0] == (
        "https://autotest.ardupilot.org/Parameters/versioned/Plane/stable-4.6.2/apm.pdef.xml",
        True,
        "Plane-4.6.2",
    )
    assert urls[1][0].endswith("/Parameters/ArduPlane/apm.pdef.xml")
    assert [u for _, u, _ in candidate_urls(plane(release=128))] == [False]  # beta: latest only
    assert candidate_urls(plane(family="unknown")) == []


def test_load_prefers_versioned_and_caches(isolated_state_dir: Path) -> None:
    calls: list[str] = []

    def fetch(url: str) -> bytes:
        calls.append(url)
        return SAMPLE

    meta, note = load_metadata(plane(), fetcher=fetch)
    assert meta is not None
    assert meta.version_matched
    assert "matches" in note
    again, _ = load_metadata(plane(), fetcher=fetch, offline=True)
    assert again is not None
    assert len(calls) == 1


def test_falls_back_to_latest_with_mismatch_flag(isolated_state_dir: Path) -> None:
    def fetch(url: str) -> bytes:
        if "versioned" in url:
            raise OSError("404")
        return SAMPLE

    meta, note = load_metadata(plane(), fetcher=fetch)
    assert meta is not None
    assert not meta.version_matched
    assert "may differ" in note


def test_offline_without_cache_reports_why(isolated_state_dir: Path) -> None:
    meta, note = load_metadata(plane(), offline=True, fetcher=lambda url: SAMPLE)
    assert meta is None
    assert "Offline" in note


def test_fetcher_refuses_plain_http() -> None:
    with pytest.raises(ValueError, match="non-HTTPS"):
        default_fetcher("http://autotest.ardupilot.org/x.xml")
