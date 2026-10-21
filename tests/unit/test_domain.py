import pytest

from mavforge.domain.errors import (
    ExitCode,
    NotPermittedError,
    NoVehicleError,
    OperationTimeoutError,
)
from mavforge.domain.models import FirmwareReleaseType, FirmwareVersion, ParamType
from mavforge.domain.outcomes import OperationStatus, exit_code_for


def test_firmware_version_decodes_packed_official_release() -> None:
    fw = FirmwareVersion.from_packed((4 << 24) | (7 << 16) | (1 << 8) | 255)
    assert (fw.major, fw.minor, fw.patch) == (4, 7, 1)
    assert fw.release_type is FirmwareReleaseType.OFFICIAL
    assert str(fw) == "4.7.1"


@pytest.mark.parametrize(
    ("type_byte", "expected", "suffix"),
    [
        (0, FirmwareReleaseType.DEV, "-dev"),
        (64, FirmwareReleaseType.ALPHA, "-alpha"),
        (128, FirmwareReleaseType.BETA, "-beta"),
        (192, FirmwareReleaseType.RC, "-rc"),
        (17, FirmwareReleaseType.UNKNOWN, "-unknown"),
    ],
)
def test_firmware_version_release_types(
    type_byte: int, expected: FirmwareReleaseType, suffix: str
) -> None:
    fw = FirmwareVersion.from_packed((4 << 24) | (6 << 16) | (2 << 8) | type_byte)
    assert fw.release_type is expected
    assert str(fw) == f"4.6.2{suffix}"


def test_every_status_has_an_exit_code() -> None:
    for status in OperationStatus:
        assert isinstance(exit_code_for(status), ExitCode)
    assert exit_code_for(OperationStatus.SUCCEEDED) is ExitCode.OK
    assert exit_code_for(OperationStatus.PARTIAL) is ExitCode.PARTIAL
    assert exit_code_for(OperationStatus.UNKNOWN) is ExitCode.TIMEOUT


def test_errors_map_to_exit_codes_and_serialize() -> None:
    assert NoVehicleError("x").exit_code is ExitCode.CONNECTION
    assert NotPermittedError("x").exit_code is ExitCode.NOT_PERMITTED
    err = OperationTimeoutError("no reply", details={"waited_s": 3})
    assert err.to_dict() == {"error": "timeout", "message": "no reply", "details": {"waited_s": 3}}


def test_param_type_integer_flag() -> None:
    assert ParamType.INT32.is_integer
    assert not ParamType.REAL32.is_integer
