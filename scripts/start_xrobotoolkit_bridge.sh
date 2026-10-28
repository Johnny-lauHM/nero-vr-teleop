#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_ROOT="${NEROX_RUNTIME_ROOT:-${ROOT}/.runtime}"
ROS_SETUP="${NEROX_ROS_SETUP:-/opt/ros/humble/setup.bash}"
PYTHON="${NEROX_PYTHON:-python3}"
XR_SDK_LIB="${XR_SDK_LIB:-/opt/apps/roboticsservice/SDK/x64}"

if ! command -v "${PYTHON}" >/dev/null 2>&1; then
  echo "Python not found: ${PYTHON} (set NEROX_PYTHON)" >&2
  exit 1
fi
if [[ ! -f "${XR_SDK_LIB}/libPXREARobotSDK.so" ]]; then
  echo "XRoboToolkit runtime library is missing: ${XR_SDK_LIB}/libPXREARobotSDK.so" >&2
  echo "Install XRoboToolkit PC Service or set XR_SDK_LIB" >&2
  exit 1
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

case ":${LD_LIBRARY_PATH:-}:" in
  *":${XR_SDK_LIB}:"*) ;;
  *) export LD_LIBRARY_PATH="${XR_SDK_LIB}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
esac
export PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
export ROS_DOMAIN_ID="${NEROX_ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${NEROX_ROS_LOCALHOST_ONLY:-1}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOG_DIR="${NEROX_ROS_LOG_DIR:-${RUNTIME_ROOT}/log}"
mkdir -p "${ROS_LOG_DIR}"

echo "[xrt-bridge] XRoboToolkit PC Service -> OpenArmX-compatible ROS 2 topics"
exec "${PYTHON}" -m nerox_openarmx_teleop.xrobotoolkit_bridge \
  --sdk-library-path "${XR_SDK_LIB}/libPXREARobotSDK.so" \
  --native-log-path "${RUNTIME_ROOT}/log/xrobotoolkit_native.log" \
  "$@"
