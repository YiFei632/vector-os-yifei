# Isaac Sim Docker image for Vector OS Nano
#
# Base: NVIDIA Isaac Sim 5.1.0 (Ubuntu 24.04, Python 3.11)
# Adds: ROS2 Jazzy, Isaac Lab 2.3.2, pinned Unitree G1 runtime, Vector bridges
#
# Build:
#   docker build --network host -t vector-isaac-sim:latest docker/isaac-sim/
#
# Run via docker-compose:
#   docker compose -f docker/isaac-sim/docker-compose.yaml up

ARG ISAAC_SIM_TAG=5.1.0
FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_TAG}

# Isaac Sim 5.1 runs rootless (uid 1234) by default.
# Switch to root for package installation, then switch back.
USER root

# ── System locale ──────────────────────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 Jazzy ─────────────────────────────────────────────────────────────────
# Isaac Sim 5.1 ships on Ubuntu 24.04 (Noble), so Jazzy is the matching ROS2.
# This means container and host both run Jazzy — no DDS version mismatch!
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    build-essential \
    cmake \
    git \
    git-lfs \
    gnupg2 \
    libssl-dev \
    lsb-release \
    software-properties-common \
    unzip \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       http://packages.ros.org/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-msgs \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── CycloneDDS config ──────────────────────────────────────────────────────────
COPY cyclonedds.xml /opt/cyclonedds.xml
COPY unitree-cyclonedds.xml /opt/unitree-cyclonedds.xml

# ── Isaac Lab (supported 5.1/2.3.x runtime pair) ───────────────────────────────
ENV ISAACSIM_PATH=/isaac-sim
ENV ISAACSIM_PYTHON_EXE=/isaac-sim/python.sh
RUN cd /opt && git clone --depth 1 --branch v2.3.2 https://github.com/isaac-sim/IsaacLab.git \
    && ln -s /isaac-sim /opt/IsaacLab/_isaac_sim \
    && test -x /opt/IsaacLab/_isaac_sim/python.sh \
    && cd /opt/IsaacLab && ./isaaclab.sh --install \
    && echo "Isaac Lab installed"

# ── Unitree SDK2 + official Isaac Lab integration ─────────────────────────────
# The simulator repository, SDK, teleimager submodule and CycloneDDS core are
# pinned. Assets and the >1 GB locomotion policy are deliberately NOT baked into
# the image; they must be mounted and pass g1_isaac_runtime.py preflight.
ARG CYCLONEDDS_REF=0.10.5
ARG UNITREE_SDK2_REF=65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5
ARG UNITREE_SIM_REF=e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc

RUN git clone --depth 1 --branch ${CYCLONEDDS_REF} \
        https://github.com/eclipse-cyclonedds/cyclonedds.git /opt/cyclonedds \
    && cmake -S /opt/cyclonedds -B /opt/cyclonedds/build \
        -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds/install \
        -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF \
    && cmake --build /opt/cyclonedds/build --target install -j2

ENV CYCLONEDDS_HOME=/opt/cyclonedds/install

RUN mkdir -p /opt/unitree_sdk2_python \
    && cd /opt/unitree_sdk2_python \
    && git init \
    && git remote add origin https://github.com/unitreerobotics/unitree_sdk2_python.git \
    && git fetch --depth 1 origin ${UNITREE_SDK2_REF} \
    && git checkout --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${UNITREE_SDK2_REF}" \
    && /isaac-sim/python.sh -m pip install --no-cache-dir /opt/unitree_sdk2_python

# unitree_sdk2_python passes an inline CycloneDDS configuration to Domain(),
# which takes precedence over CYCLONEDDS_URI. Bind the pinned simulator's DDS
# participant explicitly to loopback so rt/lowcmd cannot reach a real G1.
RUN mkdir -p /opt/unitree_sim_isaaclab \
    && cd /opt/unitree_sim_isaaclab \
    && git init \
    && git remote add origin https://github.com/unitreerobotics/unitree_sim_isaaclab.git \
    && git fetch --depth 1 origin ${UNITREE_SIM_REF} \
    && git checkout --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${UNITREE_SIM_REF}" \
    && git submodule update --init --depth 1 \
    && sed -i 's/ChannelFactoryInitialize(1)/ChannelFactoryInitialize(1, "lo")/' \
        dds/dds_master.py \
    && grep -Fq 'ChannelFactoryInitialize(1, "lo")' dds/dds_master.py \
    && git rev-parse HEAD > .vector-pinned-revision \
    && /isaac-sim/python.sh -m pip install --no-cache-dir -r requirements.txt \
    && /isaac-sim/python.sh -m pip install --no-cache-dir -e teleimager \
    && sed -i 's/^type:.*/type: isaacsim/' teleimager/cam_config_server.yaml \
    && sed -i 's/^image_shape:.*/image_shape: [480, 640]/' teleimager/cam_config_server.yaml

# ── Python bridge dependencies ─────────────────────────────────────────────────
COPY bridge/requirements.txt /tmp/bridge_requirements.txt
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -r /tmp/bridge_requirements.txt

# ── Bridge scripts ─────────────────────────────────────────────────────────────
RUN mkdir -p /vector/bridge
COPY bridge/ /vector/bridge/
RUN chmod +x /vector/bridge/*.py

# ── Custom lidar config + RL policies ─────────────────────────────────────────
RUN mkdir -p /vector/bridge/lidar_configs /vector/bridge/policies
COPY bridge/lidar_configs/ /vector/bridge/lidar_configs/
COPY bridge/policies/ /vector/bridge/policies/

# ── Environment ────────────────────────────────────────────────────────────────
ENV ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=/opt/cyclonedds.xml \
    ISAAC_ROBOT_TYPE=go2 \
    UNITREE_SIM_ROOT=/opt/unitree_sim_isaaclab \
    UNITREE_DDS_DOMAIN_ID=1 \
    UNITREE_DDS_INTERFACE=lo \
    UNITREE_CYCLONEDDS_URI=file:///opt/unitree-cyclonedds.xml \
    BRIDGE_SCRIPT=/vector/bridge/isaac_sim_physics.py

# ── Entrypoint ─────────────────────────────────────────────────────────────────
COPY docker-entrypoint.sh /vector/docker-entrypoint.sh
RUN chmod +x /vector/docker-entrypoint.sh

# Run as root for volume write access and asset downloads.
# Isaac Sim 5.1 defaults to uid 1234 but needs write access to
# cache dirs and asset download paths.

ENTRYPOINT ["/vector/docker-entrypoint.sh"]
CMD []
