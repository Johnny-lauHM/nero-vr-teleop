#!/usr/bin/env bash
# Download the official OpenArmX Quest APK (CC BY-NC-SA 4.0) at a pinned
# upstream commit, verify its checksum, and install it over ADB.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APK_COMMIT=27808afdb01dd49e04d5c298c5666c6c91d5d2dc
APK_SHA256=c74dfae5bf60af0aa3d9dbd41bc86e32dd642c0081f0a75656442eb545f79596
APK_URL="https://github.com/openarmx/openarmx_teleop_vr_apk/raw/${APK_COMMIT}/openarmx-vr-quest.apk"
APK="${ROOT}/vendor/openarmx-vr-quest.apk"

if ! command -v adb >/dev/null 2>&1; then
  echo "adb is not installed" >&2
  exit 1
fi
if [[ ! -f "${APK}" ]]; then
  echo "Downloading official OpenArmX Quest APK"
  curl -fL --retry 3 -o "${APK}.part" "${APK_URL}"
  mv "${APK}.part" "${APK}"
fi
if ! echo "${APK_SHA256}  ${APK}" | sha256sum --check --status; then
  echo "APK checksum mismatch: ${APK}" >&2
  exit 1
fi

adb devices
exec adb install -r "${APK}"
