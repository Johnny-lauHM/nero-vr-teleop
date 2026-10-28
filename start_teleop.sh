#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE=shadow
INPUT_BACKEND=openarmx_udp
TELEOP_ARGS=()

while [[ "$#" -gt 0 ]]; do
  case "$1" in
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
      shift 2
      ;;
    --check)
      echo "Use ./nerox_teleop.sh --check for the offline check" >&2
      exit 2
      ;;
    *)
      TELEOP_ARGS+=("$1")
      shift
      ;;
  esac
done
if [[ "${INPUT_BACKEND}" != openarmx_udp && "${INPUT_BACKEND}" != xrobotoolkit ]]; then
  echo "unsupported input backend: ${INPUT_BACKEND}" >&2
  exit 2
fi

SESSION_DIR="$(mktemp -d /tmp/nerox-teleop.XXXXXX)"
DRIVER_READY_FILE="${SESSION_DIR}/driver.ready"
BRIDGE_PID=""
DRIVER_PID=""
TELEOP_PID=""
SHUTTING_DOWN=false

terminate_pid() {
  local pid="$1"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  if [[ "${SHUTTING_DOWN}" == true ]]; then
    return
  fi
  SHUTTING_DOWN=true
  trap - EXIT INT TERM
  set +e
  echo "[nerox-session] stopping teleop, NERO drivers, and ${INPUT_BACKEND} input"
  terminate_pid "${TELEOP_PID}"
  terminate_pid "${DRIVER_PID}"
  terminate_pid "${BRIDGE_PID}"

  for _attempt in {1..40}; do
    any_alive=false
    for pid in "${TELEOP_PID}" "${DRIVER_PID}" "${BRIDGE_PID}"; do
      if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        any_alive=true
      fi
    done
    if [[ "${any_alive}" == false ]]; then
      break
    fi
    sleep 0.2
  done
  for pid in "${TELEOP_PID}" "${DRIVER_PID}" "${BRIDGE_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      echo "[nerox-session] forcing stale process ${pid} to exit" >&2
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
  local wait_pids=()
  for pid in "${TELEOP_PID}" "${DRIVER_PID}" "${BRIDGE_PID}"; do
    if [[ -n "${pid}" ]]; then
      wait_pids+=("${pid}")
    fi
  done
  if [[ "${#wait_pids[@]}" -gt 0 ]]; then
    wait "${wait_pids[@]}" 2>/dev/null || true
  fi

  case "${SESSION_DIR}" in
    /tmp/nerox-teleop.*)
      unlink "${DRIVER_READY_FILE}" 2>/dev/null || true
      rmdir "${SESSION_DIR}" 2>/dev/null || true
      ;;
  esac
  echo "[nerox-session] stopped"
}
trap cleanup EXIT INT TERM

if [[ "${INPUT_BACKEND}" == openarmx_udp ]]; then
  echo "[nerox-session] starting Quest bridge"
  "${ROOT}/scripts/start_quest_bridge.sh" &
  BRIDGE_PID="$!"
else
  echo "[nerox-session] starting XRoboToolkit bridge"
  "${ROOT}/scripts/start_xrobotoolkit_bridge.sh" &
  BRIDGE_PID="$!"
fi

if [[ "${MODE}" == execute ]]; then
  DRIVER_MODE=--active
  echo "[nerox-session] starting ACTIVE dual-NERO drivers"
else
  DRIVER_MODE=--readonly
  echo "[nerox-session] starting READONLY dual-NERO drivers"
fi
NEROX_DRIVER_READY_FILE="${DRIVER_READY_FILE}" \
  "${ROOT}/scripts/start_nero_drivers.sh" "${DRIVER_MODE}" &
DRIVER_PID="$!"

echo "[nerox-session] waiting for both NERO feedback streams and driver readiness"
while [[ ! -f "${DRIVER_READY_FILE}" ]]; do
  if [[ -n "${BRIDGE_PID}" ]] && ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    echo "[nerox-session] ${INPUT_BACKEND} input bridge exited during startup" >&2
    exit 1
  fi
  if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
    echo "[nerox-session] NERO driver launcher exited during startup" >&2
    exit 1
  fi
  sleep 0.1
done

echo "[nerox-session] starting OpenArmX teleop mode=${MODE}"
"${ROOT}/nerox_teleop.sh" "--${MODE}" \
  --input-backend "${INPUT_BACKEND}" "${TELEOP_ARGS[@]}" &
TELEOP_PID="$!"

set +e
SESSION_PIDS=("${DRIVER_PID}" "${TELEOP_PID}")
if [[ -n "${BRIDGE_PID}" ]]; then
  SESSION_PIDS+=("${BRIDGE_PID}")
fi
wait -n "${SESSION_PIDS[@]}"
STATUS="$?"
set -e
exit "${STATUS}"
