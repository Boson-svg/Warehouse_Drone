#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${PX4_ROOT:-/home/bosen/PX4-Autopilot}"
WORLD_PATH="${SCRIPT_DIR}/../sim/worlds/warehouse_vision.world"
MODEL_PATH="${SCRIPT_DIR}/../sim/models"

if [[ ! -f "${WORLD_PATH}" ]]; then
  echo "World file not found: ${WORLD_PATH}" >&2
  exit 1
fi

if [[ ! -d "${PX4_ROOT}" ]]; then
  echo "PX4 root not found: ${PX4_ROOT}" >&2
  exit 1
fi

export PX4_SITL_WORLD="${WORLD_PATH}"
export GAZEBO_MODEL_PATH="${MODEL_PATH}:${GAZEBO_MODEL_PATH:-}"
export PX4_GAZEBO_X="${PX4_GAZEBO_X:-0.0}"
export PX4_GAZEBO_Y="${PX4_GAZEBO_Y:-0.0}"
export PX4_GAZEBO_Z="${PX4_GAZEBO_Z:-0.83}"
export PX4_GAZEBO_YAW="${PX4_GAZEBO_YAW:-0.0}"

cd "${PX4_ROOT}"
set +u
source "/opt/ros/humble/setup.bash"
set -u
make px4_sitl gazebo_iris_opt_flow_mid360
