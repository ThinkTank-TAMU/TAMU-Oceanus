# ArduSub SITL (Software-In-The-Loop) for Team Oceanus GNC
# Builds the real ArduSub firmware so the simulated vehicle behaves exactly
# like the Navigator + BlueOS stack will on hardware.
#
# Build takes ~20-40 min and ~3 GB the first time (one-time cost, cached after).
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
# Which ArduSub release to build. Override with:
#   docker compose build --build-arg ARDUSUB_REF=Sub-4.5
ARG ARDUSUB_REF=Sub-4.5

RUN apt-get update && apt-get install -y --no-install-recommends \
        git sudo lsb-release tzdata ca-certificates netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

# ArduPilot's installer refuses to run as root, so build as a non-root user
# with passwordless sudo (the installer uses sudo internally for apt).
RUN useradd -m -s /bin/bash sub \
    && echo "sub ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/sub
USER sub

WORKDIR /home/sub
RUN git clone --depth 1 -b ${ARDUSUB_REF} https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/sub/ardupilot
RUN git submodule update --init --recursive --depth 1

# Docker RUN shells aren't login shells, so $USER/$HOME are unset — the installer
# needs them (e.g. `usermod -aG dialout $USER`, pip --user into $HOME/.local).
ENV HOME=/home/sub
ENV USER=sub

# ArduPilot's own prereq installer (pulls MAVProxy, empy, build deps, etc.)
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Patch the SITL submarine model with Team Oceanus' ROV parameters so the
# simulated dynamics approximate OUR vehicle, not the stock BlueROV2.
COPY --chown=sub:sub apply_oceanus_model.sh /home/sub/apply_oceanus_model.sh
RUN bash /home/sub/apply_oceanus_model.sh libraries/SITL/SIM_Submarine.h

# Make SIM_WIND_SPD/DIR act as a water current (stock SITL doesn't apply it to
# the sub) so we can test ConOps current-compensation / drift.
COPY --chown=sub:sub apply_current_model.sh /home/sub/apply_current_model.sh
RUN bash /home/sub/apply_current_model.sh libraries/SITL/SIM_Submarine.cpp

# GNC Practice Project 1 (training only): a 4-thruster frame in FRAME_CONFIG 7
# (Custom) plus the matching SITL physics. Applied only when built with
# PRACTICE_FRAME=1 (the `ardusub-sitl-practice4` compose service); the default
# image stays stock. See practice/P1-4-thruster-frame.md.
ARG PRACTICE_FRAME=0
COPY --chown=sub:sub apply_practice_frame.sh /home/sub/apply_practice_frame.sh
RUN if [ "$PRACTICE_FRAME" = "1" ]; then bash /home/sub/apply_practice_frame.sh .; fi

# Configure + build the ArduSub SITL binary
RUN . ~/.profile && ./waf configure --board sitl && ./waf sub

ENV PATH="/home/sub/ardupilot/Tools/autotest:/home/sub/.local/bin:${PATH}"

COPY --chown=sub:sub sim_entry.sh /home/sub/sim_entry.sh
RUN chmod +x /home/sub/sim_entry.sh

#  5780 -> pymavlink / GNC scripts      5781 -> QGroundControl
EXPOSE 5780 5781
ENTRYPOINT ["/home/sub/sim_entry.sh"]
