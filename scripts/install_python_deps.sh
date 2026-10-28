#!/usr/bin/env bash
# Install Python runtime dependencies for the selected interpreter.
# openarmx-arm-driver (CC BY-NC-SA 4.0, binary wheel) is installed into
# vendor/openarmx_python so it stays out of site-packages and out of git.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${NEROX_PYTHON:-python3}"

if ! command -v "${PYTHON}" >/dev/null 2>&1; then
  echo "Python not found: ${PYTHON} (set NEROX_PYTHON)" >&2
  exit 1
fi
if [[ ! -f "${ROOT}/vendor/pyAgxArm/setup.py" && ! -f "${ROOT}/vendor/pyAgxArm/pyproject.toml" ]]; then
  echo "vendored pyAgxArm is missing: ${ROOT}/vendor/pyAgxArm" >&2
  exit 1
fi

"${PYTHON}" -m pip install -r "${ROOT}/requirements.txt"
"${PYTHON}" -m pip install --no-deps --target "${ROOT}/vendor/openarmx_python" \
  -r "${ROOT}/requirements-openarmx.txt"
echo "Installed Python dependencies for ${PYTHON}"
echo "XRoboToolkit input additionally needs xrobotoolkit_sdk:"
echo "  https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind"
