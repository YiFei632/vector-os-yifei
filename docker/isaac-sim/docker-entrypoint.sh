#!/bin/bash
# Docker entrypoint for Vector Isaac Sim 5.1 bridge.
#
# Robot plugin architecture:
#   Go2: Isaac Core physics process + ROS2 state-file bridge (legacy default)
#   G1:  official Unitree Isaac Lab runtime + isolated DDS/file gateway + ROS2
#
# Why: Isaac Sim 5.1 uses Python 3.11 internally, but ROS2 Jazzy needs 3.12.
# The rclpy C extensions are version-locked and cannot cross Python versions.

set -eo pipefail

export ACCEPT_EULA=Y
export PRIVACY_CONSENT=Y

# State exchange directory (tmpfs for speed)
export ISAAC_STATE_DIR="/tmp/isaac_state"
mkdir -p "${ISAAC_STATE_DIR}"
rm -f \
    "${ISAAC_STATE_DIR}/ready" \
    "${ISAAC_STATE_DIR}/robot_type" \
    "${ISAAC_STATE_DIR}/robot_manifest.json" \
    "${ISAAC_STATE_DIR}/g1_command.json" \
    "${ISAAC_STATE_DIR}/g1_state.json"

case "${ISAAC_ROBOT_TYPE:-go2}" in
    go2)
        export ISAAC_ROBOT_TYPE="go2"
        ;;
    g1|g129|g1_29dof_dex3|g1-29dof-dex3)
        export ISAAC_ROBOT_TYPE="g1_29dof_dex3"
        ;;
    *)
        echo "[entrypoint] ERROR: unsupported ISAAC_ROBOT_TYPE='${ISAAC_ROBOT_TYPE}'"
        exit 2
        ;;
esac

SIM_PID=""
DDS_PID=""
ROS2_PID=""

cleanup() {
    for pid in "${SIM_PID}" "${DDS_PID}" "${ROS2_PID}"; do
        if [ -n "${pid}" ]; then
            kill "${pid}" 2>/dev/null || true
        fi
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "${ISAAC_ROBOT_TYPE}" = "go2" ]; then
    echo "[entrypoint] Starting Go2 Isaac Core plugin..."
    echo "go2" > "${ISAAC_STATE_DIR}/robot_type"
    /isaac-sim/python.sh /vector/bridge/isaac_sim_physics.py "$@" &
    SIM_PID=$!
    STARTUP_TIMEOUT_SEC="${ISAAC_STARTUP_TIMEOUT_SEC:-120}"
else
    # Unitree upstream uses the same topic names as a physical G1.  Override
    # CycloneDDS only for the two Unitree processes; the ROS2 process below
    # retains /opt/cyclonedds.xml for host ROS discovery.
    UNITREE_DDS_CONFIG="${UNITREE_CYCLONEDDS_URI:-file:///opt/unitree-cyclonedds.xml}"
    echo "[entrypoint] Starting official G1 29-DOF + dual Dex3 Isaac Lab plugin..."
    CYCLONEDDS_URI="${UNITREE_DDS_CONFIG}" \
        /isaac-sim/python.sh /vector/bridge/g1_isaac_runtime.py &
    SIM_PID=$!
    CYCLONEDDS_URI="${UNITREE_DDS_CONFIG}" \
        /isaac-sim/python.sh /vector/bridge/g1_dds_bridge.py &
    DDS_PID=$!
    STARTUP_TIMEOUT_SEC="${G1_STARTUP_TIMEOUT_SEC:-300}"
fi

# Wait for the selected plugin to publish a validated ready flag.
echo "[entrypoint] Waiting for ${ISAAC_ROBOT_TYPE} to initialize..."
for i in $(seq 1 "${STARTUP_TIMEOUT_SEC}"); do
    if [ -f "${ISAAC_STATE_DIR}/ready" ]; then
        echo "[entrypoint] ${ISAAC_ROBOT_TYPE} ready after ${i}s"
        break
    fi
    if ! kill -0 "${SIM_PID}" 2>/dev/null; then
        echo "[entrypoint] ERROR: simulation plugin exited during startup"
        exit 1
    fi
    if [ -n "${DDS_PID}" ] && ! kill -0 "${DDS_PID}" 2>/dev/null; then
        echo "[entrypoint] ERROR: G1 DDS gateway exited during startup"
        exit 1
    fi
    sleep 1
done

if [ ! -f "${ISAAC_STATE_DIR}/ready" ]; then
    echo "[entrypoint] ERROR: ${ISAAC_ROBOT_TYPE} failed to start within ${STARTUP_TIMEOUT_SEC}s"
    exit 1
fi

# Start ROS2 publisher (system Python 3.12)
echo "[entrypoint] Starting ROS2 publisher..."
source /opt/ros/jazzy/setup.bash
python3 /vector/bridge/ros2_publisher.py &
ROS2_PID=$!

echo "[entrypoint] Runtime ready (sim=${SIM_PID}, dds=${DDS_PID:-n/a}, ros2=${ROS2_PID})"

# Any component exit tears down the plugin; silently running a placeholder or a
# publisher without physics would make controller success reports dishonest.
PIDS=("${SIM_PID}" "${ROS2_PID}")
if [ -n "${DDS_PID}" ]; then
    PIDS+=("${DDS_PID}")
fi
if wait -n "${PIDS[@]}"; then
    EXIT_CODE=0
else
    EXIT_CODE=$?
fi

exit $EXIT_CODE
