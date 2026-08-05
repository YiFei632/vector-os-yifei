#!/bin/bash
# Launch Isaac Sim Docker container for Vector OS Nano.
#
# Usage:
#   ./scripts/launch_isaac.sh [--gui] [--scene flat|room|apartment]
#
# Options:
#   --gui               Enable the Isaac Sim viewport window (requires X11)
#   --scene <name>      Scene to load: flat (default), room, or apartment
#
# Environment overrides (set before calling this script):
#   ISAAC_ROBOT_TYPE    go2 (default) or g1_29dof_dex3
#   G1_ASSET_DIR        Complete official Unitree IsaacLab assets directory (G1)
#   G1_POLICY_PATH      Policy path relative to UNITREE_SIM_ROOT (G1)
#   ISAAC_PHYSICS_HZ    Physics step rate in Hz (default: 200)
#   ISAAC_LAUNCH_TIMEOUT_SEC  Outer health wait override
#   OMNI_SERVER         Nucleus server URL (e.g. omniverse://localhost)
#   VECTOR_LOG_DIR      Host path for Isaac Sim logs (default: /tmp/vector_isaac_logs)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker/isaac-sim/docker-compose.yaml"

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
ISAAC_HEADLESS="true"
ISAAC_SCENE="flat"
ISAAC_ROBOT_TYPE="${ISAAC_ROBOT_TYPE:-go2}"

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --gui)
            ISAAC_HEADLESS="false"
            shift
            ;;
        --scene)
            if [[ -z "${2-}" ]]; then
                echo "ERROR: --scene requires an argument (flat|room|apartment)" >&2
                exit 1
            fi
            ISAAC_SCENE="$2"
            shift 2
            ;;
        -h|--help)
            sed -n '2,20p' "$0"
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

# Validate scene name
case "${ISAAC_SCENE}" in
    flat|room|apartment) ;;
    *)
        echo "ERROR: Invalid scene '${ISAAC_SCENE}'. Choose flat, room, or apartment." >&2
        exit 1
        ;;
esac

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------

echo "[launch_isaac] Pre-flight checks..."

# Docker
if ! command -v docker &>/dev/null; then
    echo "ERROR: docker not found. Install Docker Engine first." >&2
    exit 1
fi

if ! docker info &>/dev/null; then
    echo "ERROR: Docker daemon is not running. Start it with: sudo systemctl start docker" >&2
    exit 1
fi

# docker compose (v2 plugin)
if ! docker compose version &>/dev/null; then
    echo "ERROR: docker compose v2 not found. Install the Docker Compose plugin." >&2
    exit 1
fi

# NVIDIA GPU + container toolkit
if ! command -v nvidia-smi &>/dev/null; then
    echo "ERROR: nvidia-smi not found. Install NVIDIA drivers." >&2
    exit 1
fi

if ! docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi &>/dev/null; then
    echo "ERROR: nvidia-container-toolkit not configured or GPU not accessible." >&2
    echo "       Install with: sudo apt install nvidia-container-toolkit" >&2
    echo "       Then restart Docker: sudo systemctl restart docker" >&2
    exit 1
fi

# X11 display check (only needed for GUI mode)
if [[ "${ISAAC_HEADLESS}" == "false" ]]; then
    if [[ -z "${DISPLAY-}" ]]; then
        echo "ERROR: --gui mode requires DISPLAY to be set (X11 session)." >&2
        exit 1
    fi
    if ! xhost &>/dev/null; then
        echo "WARNING: xhost command not found; X11 forwarding may not work." >&2
    else
        # Allow Docker root user to connect to the local X display
        xhost +local:root &>/dev/null || true
    fi
fi

# Log directory
VECTOR_LOG_DIR="${VECTOR_LOG_DIR:-/tmp/vector_isaac_logs}"
mkdir -p "${VECTOR_LOG_DIR}"

# A G1 launch must use the complete, externally supplied official asset bundle.
# Fail here instead of spending several minutes building/starting a container
# whose runtime preflight is guaranteed to reject the placeholder directory.
case "${ISAAC_ROBOT_TYPE}" in
    g1|g129|g1_29dof_dex3|g1-29dof-dex3)
        G1_ASSET_DIR_VALUE="${G1_ASSET_DIR:-}"
        if [[ -z "${G1_ASSET_DIR_VALUE}" || ! -d "${G1_ASSET_DIR_VALUE}" ]]; then
            echo "ERROR: G1 requires G1_ASSET_DIR to name the complete official unitree_sim_isaaclab/assets directory." >&2
            exit 1
        fi
        G1_ASSET_DIR_VALUE="$(cd "${G1_ASSET_DIR_VALUE}" && pwd)"
        G1_POLICY_PATH_VALUE="${G1_POLICY_PATH:-assets/model/policy.onnx}"
        if [[ "${G1_POLICY_PATH_VALUE}" != assets/* || "${G1_POLICY_PATH_VALUE}" == *".."* ]]; then
            echo "ERROR: G1_POLICY_PATH must be a safe path below assets/ (default: assets/model/policy.onnx)." >&2
            exit 1
        fi
        G1_POLICY_ASSET_REL="${G1_POLICY_PATH_VALUE#assets/}"
        G1_REQUIRED_ASSETS=(
            "robots/g1-29dof_wholebody_dex3/g1_29dof_with_dex3_rev_1_0.usd"
            "objects/small_warehouse/small_warehouse_digital_twin.usd"
            "objects/PackingTable_2/PackingTable.usd"
            "objects/PackingTable/PackingTable.usd"
            "${G1_POLICY_ASSET_REL}"
        )
        for relative_path in "${G1_REQUIRED_ASSETS[@]}"; do
            if [[ ! -s "${G1_ASSET_DIR_VALUE}/${relative_path}" ]]; then
                echo "ERROR: G1 official runtime asset is missing or empty: ${G1_ASSET_DIR_VALUE}/${relative_path}" >&2
                echo "       The supplied URDF/meshes/MJCF are sufficient for MuJoCo and Pinocchio, but the pinned Isaac whole-body task additionally requires its official USD bundle and policy." >&2
                exit 1
            fi
        done
        export G1_ASSET_DIR="${G1_ASSET_DIR_VALUE}"
        export G1_POLICY_PATH="${G1_POLICY_PATH_VALUE}"
        ;;
esac

# ---------------------------------------------------------------------------
# Build image if not present or stale
# ---------------------------------------------------------------------------
IMAGE_TAG="vector-isaac-sim:latest"

if ! docker image inspect "${IMAGE_TAG}" &>/dev/null; then
    echo "[launch_isaac] Image '${IMAGE_TAG}' not found. Building..."
    docker build \
        -t "${IMAGE_TAG}" \
        --progress=plain \
        "${SCRIPT_DIR}/docker/isaac-sim/"
else
    echo "[launch_isaac] Using existing image '${IMAGE_TAG}'. (Re-build with: docker build -t ${IMAGE_TAG} docker/isaac-sim/)"
fi

# ---------------------------------------------------------------------------
# Launch
# ---------------------------------------------------------------------------
export ISAAC_HEADLESS
export ISAAC_SCENE
export ISAAC_ROBOT_TYPE
export VECTOR_LOG_DIR
export DISPLAY="${DISPLAY:-:0}"

echo ""
echo "[launch_isaac] Starting Isaac Sim container..."
echo "  Scene   : ${ISAAC_SCENE}"
echo "  Robot   : ${ISAAC_ROBOT_TYPE}"
echo "  Headless: ${ISAAC_HEADLESS}"
echo "  Logs    : ${VECTOR_LOG_DIR}"
echo ""

docker compose -f "${COMPOSE_FILE}" up -d

# ---------------------------------------------------------------------------
# Wait for health check. G1's official IsaacLab task has a longer container-side
# startup budget than the legacy Go2 plugin, so the outer wait must not expire
# first. ISAAC_LAUNCH_TIMEOUT_SEC can override either default.
# ---------------------------------------------------------------------------
CONTAINER="vector-isaac-sim"
case "${ISAAC_ROBOT_TYPE}" in
    g1|g129|g1_29dof_dex3|g1-29dof-dex3)
        MAX_WAIT="${ISAAC_LAUNCH_TIMEOUT_SEC:-360}"
        ;;
    *)
        MAX_WAIT="${ISAAC_LAUNCH_TIMEOUT_SEC:-180}"
        ;;
esac
INTERVAL=10
elapsed=0

echo "[launch_isaac] Waiting for bridge to become healthy (max ${MAX_WAIT}s)..."
echo "  Isaac Sim shader compilation can take 2-3 minutes on first launch."

while true; do
    status="$(docker inspect --format='{{.State.Health.Status}}' "${CONTAINER}" 2>/dev/null || echo "missing")"

    case "${status}" in
        healthy)
            echo "[launch_isaac] Container is healthy."
            break
            ;;
        unhealthy)
            echo "ERROR: Container reported unhealthy. Check logs:" >&2
            docker logs --tail 50 "${CONTAINER}" >&2
            exit 1
            ;;
        missing)
            echo "ERROR: Container '${CONTAINER}' not found." >&2
            exit 1
            ;;
        *)
            # starting or none (no health check result yet)
            if [[ ${elapsed} -ge ${MAX_WAIT} ]]; then
                echo "ERROR: Timed out waiting for health check after ${MAX_WAIT}s." >&2
                echo "       Container logs:" >&2
                docker logs --tail 80 "${CONTAINER}" >&2
                exit 1
            fi
            echo "  [${elapsed}s/${MAX_WAIT}s] Status: ${status} — waiting..."
            sleep ${INTERVAL}
            elapsed=$((elapsed + INTERVAL))
            ;;
    esac
done

# ---------------------------------------------------------------------------
# Print connection info
# ---------------------------------------------------------------------------
echo ""
echo "[launch_isaac] Isaac Sim bridge is running."
echo ""
echo "  ROS2 topics (on host):"
echo "    ros2 topic list"
echo "    ros2 topic hz /state_estimation"
if [[ "${ISAAC_ROBOT_TYPE}" == g1* || "${ISAAC_ROBOT_TYPE}" == "g129" ]]; then
    echo "    ros2 topic hz /joint_states"
else
    echo "    ros2 topic hz /registered_scan"
fi
echo ""
echo "  Container logs:"
echo "    docker logs -f ${CONTAINER}"
echo ""
echo "  Stop the simulation:"
echo "    ./scripts/stop_isaac.sh"
echo ""
