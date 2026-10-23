"""ArduPilot parameter metadata (apm.pdef.xml): ranges, allowed values, reboot flags.

ArduPilot publishes machine-readable parameter definitions generated from its source:

- ``https://autotest.ardupilot.org/Parameters/versioned/<Vehicle>/stable-X.Y.Z/apm.pdef.xml``
  for official releases, and
- ``https://autotest.ardupilot.org/Parameters/<ArduVehicle>/apm.pdef.xml`` for the
  latest development build.

The versioned file matching the vehicle's firmware is preferred. If none exists the
latest file is used and the result is marked ``version_matched=False``, because names
and ranges change between releases. Files are downloaded on first use, cached under the
MAVForge state directory and parsed with defusedxml (the content comes from the network).
Nothing is fetched when ``offline`` is set.
"""

from __future__ import annotations

import logging
import time
import urllib.error
import urllib.request
from collections.abc import Callable
from pathlib import Path

from defusedxml import ElementTree

from mavforge.domain.models import FirmwareReleaseType, VehicleIdentity
from mavforge.platforms.metadata import MetadataSet, ParamMeta
from mavforge.policy.audit import default_state_dir

log = logging.getLogger(__name__)

BASE_URL = "https://autotest.ardupilot.org/Parameters"
MAX_BYTES = 32 * 1024 * 1024
LATEST_MAX_AGE_S = 7 * 24 * 3600
FETCH_TIMEOUT_S = 30.0

# vehicle_family -> (versioned directory, latest directory)
_DIRECTORIES: dict[str, tuple[str | None, str]] = {
    "copter": ("Copter", "ArduCopter"),
    "heli": (None, "Heli"),
    "plane": ("Plane", "ArduPlane"),
    "rover": ("Rover", "Rover"),
    "sub": ("Sub", "ArduSub"),
    "tracker": ("Tracker", "AntennaTracker"),
    "blimp": (None, "Blimp"),
}

Fetcher = Callable[[str], bytes]


def _flag(fields: dict[str, str], name: str) -> bool:
    return fields.get(name, "").strip().lower() == "true"


def parse_pdef_xml(data: bytes) -> dict[str, ParamMeta]:
    root = ElementTree.fromstring(data)
    params: dict[str, ParamMeta] = {}
    for p in root.iter("param"):
        raw_name = p.get("name") or ""
        name = raw_name.split(":")[-1]
        if not name:
            continue
        fields = {f.get("name") or "": (f.text or "").strip() for f in p.findall("field")}
        rng: tuple[float, float] | None = None
        if parts := fields.get("Range", "").split():
            try:
                rng = (float(parts[0]), float(parts[1]))
            except (ValueError, IndexError):
                rng = None
        values: dict[float, str] = {}
        for v in p.iter("value"):
            try:
                values[float(v.get("code") or "")] = (v.text or "").strip()
            except ValueError:
                continue
        bitmask: dict[int, str] = {}
        for item in fields.get("Bitmask", "").split(","):
            bit, _, label = item.partition(":")
            if bit.strip().isdigit():
                bitmask[int(bit)] = label.strip()
        params.setdefault(
            name,
            ParamMeta(
                name=name,
                display_name=p.get("humanName"),
                description=p.get("documentation"),
                units=fields.get("Units") or None,
                range=rng,
                values=values,
                bitmask=bitmask,
                reboot_required=_flag(fields, "RebootRequired"),
                read_only=_flag(fields, "ReadOnly"),
                volatile=_flag(fields, "Volatile"),
            ),
        )
    return params


def candidate_urls(identity: VehicleIdentity) -> list[tuple[str, bool, str]]:
    """(url, version_matched, cache key) in order of preference."""
    dirs = _DIRECTORIES.get(identity.vehicle_family or "")
    if dirs is None:
        return []
    versioned, latest = dirs
    out: list[tuple[str, bool, str]] = []
    fw = identity.firmware
    if versioned and fw is not None and fw.release_type is FirmwareReleaseType.OFFICIAL:
        version = f"{fw.major}.{fw.minor}.{fw.patch}"
        out.append(
            (
                f"{BASE_URL}/versioned/{versioned}/stable-{version}/apm.pdef.xml",
                True,
                f"{versioned}-{version}",
            )
        )
    out.append((f"{BASE_URL}/{latest}/apm.pdef.xml", False, f"{latest}-latest"))
    return out


def default_fetcher(url: str) -> bytes:
    if not url.startswith("https://"):
        raise ValueError(f"Refusing non-HTTPS metadata URL {url}")
    request = urllib.request.Request(url, headers={"User-Agent": "mavforge"})  # noqa: S310
    with urllib.request.urlopen(request, timeout=FETCH_TIMEOUT_S) as response:  # noqa: S310
        data = response.read(MAX_BYTES + 1)
    if len(data) > MAX_BYTES:
        raise ValueError(f"Metadata at {url} exceeds {MAX_BYTES} bytes")
    return data


def cache_dir() -> Path:
    return default_state_dir() / "metadata"


def load_metadata(
    identity: VehicleIdentity,
    *,
    offline: bool = False,
    fetcher: Fetcher | None = None,
) -> tuple[MetadataSet | None, str]:
    """Metadata for this vehicle, plus a human-readable note about where it came from."""
    fetch = fetcher or default_fetcher
    candidates = candidate_urls(identity)
    if not candidates:
        return None, f"No ArduPilot metadata is published for {identity.mav_type}."
    errors: list[str] = []
    for url, matched, key in candidates:
        path = cache_dir() / f"{key}.xml"
        fresh = path.exists() and (matched or time.time() - path.stat().st_mtime < LATEST_MAX_AGE_S)
        data: bytes | None = path.read_bytes() if fresh else None
        if data is None and not offline:
            try:
                data = fetch(url)
                path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                path.write_bytes(data)
            except (urllib.error.URLError, OSError, ValueError) as exc:
                errors.append(f"{url}: {exc}")
                log.info("metadata fetch failed: %s: %s", url, exc)
        if data is None and path.exists():
            data = path.read_bytes()  # stale latest is better than nothing
        if data is None:
            continue
        try:
            params = parse_pdef_xml(data)
        except Exception as exc:  # malformed or hostile XML
            errors.append(f"{url}: unparseable ({exc})")
            continue
        note = (
            "matches the vehicle firmware"
            if matched
            else (
                "latest development definitions; the vehicle's exact release was not available, "
                "so names and ranges may differ"
            )
        )
        return MetadataSet(source=url, version_matched=matched, params=params), note
    if offline:
        return None, "Offline: no cached ArduPilot parameter metadata for this firmware."
    return None, "Could not obtain ArduPilot parameter metadata: " + "; ".join(errors)
