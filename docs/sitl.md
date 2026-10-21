# Testing with ArduPilot SITL

MAVForge's `sitl` tests run against **real ArduPilot firmware** compiled for the host
(Software-In-The-Loop). They cover behaviour the fake vehicle can only approximate.

## Docker image

`docker/sitl/Dockerfile` packages ArduPilot's official prebuilt, statically linked
ArduCopter SITL binary:

- **Version:** stable 4.7.2, commit `fb773567`.
- **Defaults:** the matching `Tools/autotest/default_params/copter.parm`.
- **Integrity:** both files are pinned by SHA-256.
- **Build:** nothing is compiled.

```sh
docker build -t mavforge-sitl:4.7.2 docker/sitl
docker run --rm -p 127.0.0.1:5760:5760 mavforge-sitl:4.7.2
uv run mavforge -c tcp:127.0.0.1:5760 --timeout 30 status
```

SITL waits for the first TCP client before it boots. Give the first command a generous
`--timeout`.

## Running the tests

```sh
uv run pytest -m sitl
```

The session fixture (`tests/sitl/conftest.py`) does the following:

1. Builds the image if it's missing.
2. Starts a container on a free localhost port, with a wiped EEPROM.
3. Runs the tests.
4. Removes the container.

To use a SITL you started yourself (e.g. `sim_vehicle.py`, or another vehicle type),
point the tests at it:

```sh
MAVFORGE_SITL_CONNECTION=tcp:127.0.0.1:5760 uv run pytest -m sitl
```

The current SITL tests are read-only. Future write and calibration tests will run
against the throwaway container only.

## Updating the pinned version

1. Read `firmware-version.txt` and `git-version.txt` from
   `https://firmware.ardupilot.org/Copter/stable-X.Y.Z/SITL_x86_64_linux_gnu/`.
2. Download the binary, and `copter.parm` at that commit. Record their SHA-256 values.
3. Update the `ARG`s in the Dockerfile, the tag in `tests/sitl/conftest.py`, and
   `.github/workflows/ci.yml`.
4. Run `pytest -m sitl`, and record any behaviour changes in `docs/research.md`. For
   example, 4.7 renamed `SYSID_THISMAV` to `MAV_SYSID`.
