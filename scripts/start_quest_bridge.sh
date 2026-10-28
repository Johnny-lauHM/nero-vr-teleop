#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_ROOT="${NEROX_RUNTIME_ROOT:-${ROOT}/.runtime}"
ROS_SETUP="${NEROX_ROS_SETUP:-/opt/ros/humble/setup.bash}"
QUEST_OVERLAY="${NEROX_QUEST_OVERLAY:-${RUNTIME_ROOT}/install/quest/setup.bash}"
BRIDGE_PACKAGE_PREFIX="${RUNTIME_ROOT}/install/quest/openarmx_teleop_bridge_vr"
BRIDGE_EXECUTABLE_DEFAULT="${BRIDGE_PACKAGE_PREFIX}/lib"
BRIDGE_EXECUTABLE_DEFAULT+="/openarmx_teleop_bridge_vr/openarmx_teleop_bridge_vr_node"
BRIDGE_EXECUTABLE="${NEROX_QUEST_BRIDGE_EXECUTABLE:-${BRIDGE_EXECUTABLE_DEFAULT}}"
LISTEN_ADDRESS="${NEROX_QUEST_LISTEN_ADDRESS:-0.0.0.0}"
LISTEN_PORT="${NEROX_QUEST_LISTEN_PORT:-5100}"

if [[ ! -f "${QUEST_OVERLAY}" ]]; then
  echo "Official OpenArmX Quest bridge is not built. Run ${ROOT}/scripts/build.sh" >&2
  exit 1
fi
if [[ ! -x "${BRIDGE_EXECUTABLE}" ]]; then
  echo "Official OpenArmX Quest bridge executable not found: ${BRIDGE_EXECUTABLE}" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${QUEST_OVERLAY}"
set -u

export ROS_DOMAIN_ID="${NEROX_ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${NEROX_ROS_LOCALHOST_ONLY:-1}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOG_DIR="${NEROX_ROS_LOG_DIR:-${RUNTIME_ROOT}/log}"
mkdir -p "${ROS_LOG_DIR}"

echo "[quest-bridge] official OpenArmX UDP ${LISTEN_ADDRESS}:${LISTEN_PORT} -> ROS 2"
exec "${BRIDGE_EXECUTABLE}" \
  --ros-args \
  -p listen_address:="${LISTEN_ADDRESS}" \
  -p listen_port:="${LISTEN_PORT}" \
  -p publish_tf:=false
