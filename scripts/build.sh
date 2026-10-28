#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_ROOT="${NEROX_RUNTIME_ROOT:-${ROOT}/.runtime}"
ROS_SETUP="${NEROX_ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
AGX_SOURCE="${ROOT}/vendor/agx_arm_ros/src"
AGX_DESCRIPTION_SOURCE="${ROOT}/third_party/agx_arm_ros/src/agx_arm_description"
QUEST_SOURCE="${ROOT}/vendor/openarmx_teleop_bridge_vr"
if [[ ! -d "${AGX_SOURCE}" ]]; then
  echo "Vendored Nerox AGX control source is missing" >&2
  exit 1
fi
if [[ ! -d "${AGX_DESCRIPTION_SOURCE}" ]]; then
  echo "agx_arm_description is missing; run: git submodule update --init --recursive" >&2
  exit 1
fi
if [[ ! -d "${QUEST_SOURCE}" ]]; then
  echo "Official OpenArmX Quest bridge source is missing" >&2
  exit 1
fi

python3 "${ROOT}/tools/build_nerox_openarmx_model.py"

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

# The interactive machine has a base Conda Python ahead of /usr/bin.  ROS 2
# Humble's ament tooling and Debian catkin_pkg belong to the system Python.
export PATH="/usr/bin:/bin:${PATH}"

cd "${ROOT}"
mkdir -p "${RUNTIME_ROOT}/log"
colcon --log-base "${RUNTIME_ROOT}/log/agx" build \
  --base-paths "${AGX_SOURCE}" "${AGX_DESCRIPTION_SOURCE}" \
  --packages-select agx_arm_msgs agx_arm_description agx_arm_ctrl \
  --build-base "${RUNTIME_ROOT}/build/agx" \
  --install-base "${RUNTIME_ROOT}/install/agx" \
  --symlink-install \
  --cmake-args \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPython3_EXECUTABLE=/usr/bin/python3

colcon --log-base "${RUNTIME_ROOT}/log/quest" build \
  --base-paths "${QUEST_SOURCE}" \
  --packages-select openarmx_teleop_bridge_vr \
  --build-base "${RUNTIME_ROOT}/build/quest" \
  --install-base "${RUNTIME_ROOT}/install/quest" \
  --symlink-install \
  --cmake-args \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPython3_EXECUTABLE=/usr/bin/python3

echo "Built local AGX NERO driver overlay: ${RUNTIME_ROOT}/install/agx/setup.bash"
echo "Built official OpenArmX Quest bridge: ${RUNTIME_ROOT}/install/quest/setup.bash"
