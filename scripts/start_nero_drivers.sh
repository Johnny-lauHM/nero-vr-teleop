#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_ROOT="${NEROX_RUNTIME_ROOT:-${ROOT}/.runtime}"
ROS_SETUP="${NEROX_ROS_SETUP:-/opt/ros/humble/setup.bash}"
AGX_OVERLAY="${NEROX_AGX_OVERLAY:-${RUNTIME_ROOT}/install/agx/setup.bash}"
PYTHON="${NEROX_PYTHON:-python3}"
MODE=readonly

case "${1:-}" in
  --readonly|"") MODE=readonly ;;
  --active) MODE=active ;;
  *) echo "Usage: $0 [--readonly|--active]" >&2; exit 2 ;;
esac

if [[ ! -f "${AGX_OVERLAY}" ]]; then
  echo "Local AGX overlay not built. Run ${ROOT}/scripts/build.sh" >&2
  exit 1
fi
if ! command -v "${PYTHON}" >/dev/null 2>&1; then
  echo "Python not found: ${PYTHON} (set NEROX_PYTHON)" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${AGX_OVERLAY}"
set -u

export PYTHONPATH="${ROOT}/src:${ROOT}/vendor/pyAgxArm${PYTHONPATH:+:${PYTHONPATH}}"
export ROS_DOMAIN_ID="${NEROX_ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${NEROX_ROS_LOCALHOST_ONLY:-1}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOG_DIR="${NEROX_ROS_LOG_DIR:-${RUNTIME_ROOT}/log}"
mkdir -p "${ROS_LOG_DIR}"

AUTO_ENABLE=false
CONTROL_ENABLED=false
ARM_A_CAN="${NEROX_ARM_A_CAN:-can_arm_a}"
ARM_B_CAN="${NEROX_ARM_B_CAN:-can_arm_b}"
PAYLOAD_ENABLED="${NEROX_PAYLOAD_COMPENSATION_ENABLED:-true}"
PAYLOAD_LEVEL="${NEROX_PAYLOAD_LEVEL:-half}"
PAYLOAD_MASS_KG="${NEROX_PAYLOAD_MASS_KG:-1.010}"
PAYLOAD_REQUIRE_ACK="${NEROX_PAYLOAD_REQUIRE_ACK:-false}"
PAYLOAD_READY_TIMEOUT_S="${NEROX_PAYLOAD_READY_TIMEOUT_S:-15}"
DRIVER_READY_FILE="${NEROX_DRIVER_READY_FILE:-}"
if [[ -n "${DRIVER_READY_FILE}" && "${DRIVER_READY_FILE}" != /* ]]; then
  echo "NEROX_DRIVER_READY_FILE must be an absolute path" >&2
  exit 2
fi
if [[ "${MODE}" == active ]]; then
  AUTO_ENABLE=true
  echo "[nerox-driver] ACTIVE: enabling both NERO arms; command gates stay closed until payload setup"
else
  echo "[nerox-driver] READONLY: feedback only; motor auto-enable and control are disabled"
fi

PIDS=()
SHUTTING_DOWN=false

ros_service_bool() {
  local service_name="$1"
  local value="$2"
  timeout 3 ros2 service call "${service_name}" \
    std_srvs/srv/SetBool "{data: ${value}}" >/dev/null 2>&1
}

cleanup() {
  if [[ "${SHUTTING_DOWN}" == true ]]; then
    return
  fi
  SHUTTING_DOWN=true
  trap - EXIT INT TERM
  set +e
  if [[ "${MODE}" == active ]]; then
    echo "[nerox-driver] closing command gates and disabling both NERO arms"
    CLEANUP_SERVICE_PIDS=()
    ros_service_bool /arm_a/control_enable false &
    CLEANUP_SERVICE_PIDS+=("$!")
    ros_service_bool /arm_b/control_enable false &
    CLEANUP_SERVICE_PIDS+=("$!")
    ros_service_bool /arm_a/enable_agx_arm false &
    CLEANUP_SERVICE_PIDS+=("$!")
    ros_service_bool /arm_b/enable_agx_arm false &
    CLEANUP_SERVICE_PIDS+=("$!")
    for pid in "${CLEANUP_SERVICE_PIDS[@]}"; do
      wait "${pid}" 2>/dev/null || true
    done
  fi
  for pid in "${PIDS[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for _attempt in {1..10}; do
    any_alive=false
    for pid in "${PIDS[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        any_alive=true
      fi
    done
    if [[ "${any_alive}" == false ]]; then
      break
    fi
    sleep 0.2
  done
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      echo "[nerox-driver] forcing stale driver process ${pid} to exit" >&2
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
  wait "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

drivers_alive() {
  local pid
  for pid in "${PIDS[@]}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 1
    fi
  done
}

wait_for_feedback() {
  local topic="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if ! drivers_alive; then
      echo "[nerox-driver] a NERO driver exited while waiting for ${topic}" >&2
      return 1
    fi
    # ros2 topic echo exits immediately while discovery cannot yet determine
    # the type. Retry that normal startup race until one real sample arrives.
    if timeout 2 ros2 topic echo --no-daemon "${topic}" --once >/dev/null 2>&1; then
      echo "[nerox-driver] feedback ready: ${topic}"
      return 0
    fi
    sleep 0.2
  done
  echo "[nerox-driver] timed out waiting for ${topic}" >&2
  return 1
}

"${PYTHON}" -m agx_arm_ctrl.agx_arm_ctrl_single_node --ros-args \
  -r __ns:=/arm_a \
  -p can_port:="${ARM_A_CAN}" -p arm_type:=nero -p effector_type:=none \
  -p auto_enable:="${AUTO_ENABLE}" -p control_enabled:="${CONTROL_ENABLED}" \
  -p fast_mode:=true -p speed_percent:=10 &
PIDS+=("$!")

"${PYTHON}" -m agx_arm_ctrl.agx_arm_ctrl_single_node --ros-args \
  -r __ns:=/arm_b \
  -p can_port:="${ARM_B_CAN}" -p arm_type:=nero -p effector_type:=none \
  -p auto_enable:="${AUTO_ENABLE}" -p control_enabled:="${CONTROL_ENABLED}" \
  -p fast_mode:=true -p speed_percent:=10 &
PIDS+=("$!")

echo "[nerox-driver] waiting for NERO feedback"
wait_for_feedback /arm_a/feedback/joint_states "${PAYLOAD_READY_TIMEOUT_S}"
wait_for_feedback /arm_b/feedback/joint_states "${PAYLOAD_READY_TIMEOUT_S}"

if [[ "${MODE}" == active ]]; then
  if [[ ! "${PAYLOAD_ENABLED,,}" =~ ^(0|false|no|off)$ ]]; then
    PAYLOAD_ARGS=(
      --can-port "${ARM_A_CAN}"
      --can-port "${ARM_B_CAN}"
      --payload-level "${PAYLOAD_LEVEL}"
      --payload-mass-kg "${PAYLOAD_MASS_KG}"
    )
    if [[ "${PAYLOAD_REQUIRE_ACK,,}" =~ ^(1|true|yes|on)$ ]]; then
      PAYLOAD_ARGS+=(--require-ack)
    fi
    "${PYTHON}" -m nerox_openarmx_teleop.payload "${PAYLOAD_ARGS[@]}"
  else
    echo "[nerox-driver] payload compensation disabled by NEROX_PAYLOAD_COMPENSATION_ENABLED"
  fi

  ros_service_bool /arm_a/control_enable true
  ros_service_bool /arm_b/control_enable true
  echo "[nerox-driver] both external command gates are open"
else
  echo "[nerox-driver] READONLY: payload configuration is not transmitted"
fi

if [[ -n "${DRIVER_READY_FILE}" ]]; then
  (umask 077; : >"${DRIVER_READY_FILE}")
fi

wait -n "${PIDS[@]}"
