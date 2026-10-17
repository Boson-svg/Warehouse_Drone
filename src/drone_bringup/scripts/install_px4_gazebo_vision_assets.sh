#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${PX4_ROOT:-/home/bosen/PX4-Autopilot}"
SOURCE_DIR="${SCRIPT_DIR}/../sim/px4_models/iris_opt_flow_mid360"
TARGET_DIR="${PX4_ROOT}/Tools/sitl_gazebo/models/iris_opt_flow_mid360"
BACKUP_SUFFIX=".bak.drone_ws_vision"

if [[ ! -d "${PX4_ROOT}" ]]; then
  echo "PX4 root not found: ${PX4_ROOT}" >&2
  exit 1
fi

mkdir -p "${TARGET_DIR}"

for name in "iris_opt_flow_mid360.sdf" "model.config"; do
  target="${TARGET_DIR}/${name}"
  source="${SOURCE_DIR}/${name}"
  backup="${target}${BACKUP_SUFFIX}"

  if [[ -f "${target}" && ! -f "${backup}" ]]; then
    cp "${target}" "${backup}"
    echo "backup created: ${backup}"
  fi

  install -m 0644 "${source}" "${target}"
  echo "installed: ${target}"
done

echo "PX4 Gazebo vision assets installed."
