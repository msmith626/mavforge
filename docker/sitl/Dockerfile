# ArduCopter SITL for MAVForge integration tests.
#
# Runs ArduPilot's official prebuilt (statically linked) SITL binary; nothing is
# compiled. The binary and the default parameter file are pinned by version, commit
# and SHA-256. To move to a new release, update the ARGs below (see docs/sitl.md).
#
# ArduPilot is GPL-3.0-or-later; see https://github.com/ArduPilot/ardupilot.
FROM debian:bookworm-slim

ARG ARDUPILOT_VERSION=4.7.2
ARG ARDUPILOT_COMMIT=fb773567896014da007bca58358704d4a0d8516f
ARG SITL_SHA256=caed35e4cb796ec06cac40b09566a18251771486b1cbac5e2bf16b7a5ed5045d
ARG PARM_SHA256=5e01345b45d1c6190b28bece5638bbdd4cf1cce35e05bbbf480ab24d2b51aa0e

RUN apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates curl \
 && mkdir -p /opt/ardupilot \
 && curl -fsSL -o /opt/ardupilot/arducopter \
      "https://firmware.ardupilot.org/Copter/stable-${ARDUPILOT_VERSION}/SITL_x86_64_linux_gnu/arducopter" \
 && curl -fsSL -o /opt/ardupilot/copter.parm \
      "https://raw.githubusercontent.com/ArduPilot/ardupilot/${ARDUPILOT_COMMIT}/Tools/autotest/default_params/copter.parm" \
 && echo "${SITL_SHA256}  /opt/ardupilot/arducopter" | sha256sum -c - \
 && echo "${PARM_SHA256}  /opt/ardupilot/copter.parm" | sha256sum -c - \
 && chmod 0755 /opt/ardupilot/arducopter \
 && apt-get purge -y curl && apt-get autoremove -y && rm -rf /var/lib/apt/lists/* \
 && useradd --system --create-home --home-dir /sitl sitl

LABEL org.opencontainers.image.title="mavforge-sitl" \
      org.opencontainers.image.description="ArduCopter ${ARDUPILOT_VERSION} SITL for MAVForge tests" \
      org.ardupilot.version="${ARDUPILOT_VERSION}" \
      org.ardupilot.commit="${ARDUPILOT_COMMIT}"

USER sitl
WORKDIR /sitl
# SERIAL0 is a TCP server on 5760. SITL waits for a client before it starts booting.
EXPOSE 5760
ENTRYPOINT ["/opt/ardupilot/arducopter"]
CMD ["--model", "quad", "--speedup", "1", "-I0", "-w", \
     "--home", "-35.363262,149.165237,584,353", \
     "--defaults", "/opt/ardupilot/copter.parm"]
