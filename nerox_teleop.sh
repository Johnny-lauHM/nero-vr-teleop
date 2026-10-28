#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_ROOT="${NEROX_RUNTIME_ROOT:-${ROOT}/.runtime}"
ROS_SETUP="${NEROX_ROS_SETUP:-/opt/ros/humble/setup.bash}"
AGX_OVERLAY="${NEROX_AGX_OVERLAY:-${RUNTIME_ROOT}/install/agx/setup.bash}"
QUEST_OVERLAY="${NEROX_QUEST_OVERLAY:-${RUNTIME_ROOT}/install/quest/local_setup.bash}"
PYTHON="${NEROX_PYTHON:-python3}"
MODE=shadow
INPUT_BACKEND=openarmx_udp
FORWARD_ARGS=()

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --check)
      MODE=check
      shift
      ;;
    --shadow)
      MODE=shadow
      shift
      ;;
    --execute)
      MODE=execute
      shift
      ;;
    --input-backend)
      if [[ "$#" -lt 2 ]]; then
        echo "--input-backend requires a value" >&2
        exit 2
      fi
      INPUT_BACKEND="$2"
      FORWARD_ARGS+=("$1" "$2")
      shift 2
      ;;
    *)
      FORWARD_ARGS+=("$1")
      shift
      ;;
  esac
done
if [[ "${INPUT_BACKEND}" != openarmx_udp && "${INPUT_BACKEND}" != xrobotoolkit ]]; then
  echo "unsupported input backend: ${INPUT_BACKEND}" >&2
  exit 2
fi

if ! command -v "${PYTHON}" >/dev/null 2>&1; then
  echo "Python not found: ${PYTHON} (set NEROX_PYTHON)" >&2
  exit 1
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
if [[ -f "${AGX_OVERLAY}" ]]; then
  # shellcheck disable=SC1090
  source "${AGX_OVERLAY}"
elif [[ "${MODE}" != check ]]; then
  echo "Local AGX overlay not built: ${AGX_OVERLAY}" >&2
  echo "Run: ${ROOT}/scripts/build.sh" >&2
  exit 1
fi
if [[ -f "${QUEST_OVERLAY}" ]]; then
  # shellcheck disable=SC1090
  source "${QUEST_OVERLAY}"
elif [[ "${MODE}" != check && "${INPUT_BACKEND}" == openarmx_udp ]]; then
  echo "Official OpenArmX Quest bridge is not built: ${QUEST_OVERLAY}" >&2
  echo "Run: ${ROOT}/scripts/build.sh" >&2
  exit 1
fi
set -u

export PYTHONPATH="${ROOT}/src:${ROOT}/vendor/openarmx_python:${ROOT}/vendor/pyAgxArm${PYTHONPATH:+:${PYTHONPATH}}"
export ROS_DOMAIN_ID="${NEROX_ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${NEROX_ROS_LOCALHOST_ONLY:-1}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOG_DIR="${NEROX_ROS_LOG_DIR:-${RUNTIME_ROOT}/log}"
mkdir -p "${ROS_LOG_DIR}"

echo "[nerox] mode=${MODE} ik=pinocchio input=${INPUT_BACKEND} robot=dual-nero"
cd "${ROOT}"
exec "${PYTHON}" -m nerox_openarmx_teleop.node \
  "--${MODE}" \
  "${FORWARD_ARGS[@]}"
