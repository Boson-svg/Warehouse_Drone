#!/usr/bin/env bash
# hover_simulate.sh [hold_seconds]
#
# Simulates the mission control hover signal for qr_vision_node without the
# ground station: publishes /mission/hover_active=true, holds for a duration,
# then publishes false (which triggers the final barcode capture).
#
# Usage (while qr_vision_node is running):
#   source /opt/ros/humble/setup.bash
#   source ~/warehouse_ws/install/setup.bash
#   bash hover_simulate.sh 20
set -e

HOLD="${1:-20}"

echo "[hover_simulate] publish hover_active=true  (hold ${HOLD}s)"
ros2 topic pub --qos-durability transient_local -1 \
  /mission/hover_active std_msgs/msg/Bool "{data: true}"

echo "[hover_simulate] holding ${HOLD}s ..."
sleep "${HOLD}"

echo "[hover_simulate] publish hover_active=false (triggers final capture)"
ros2 topic pub --qos-durability transient_local -1 \
  /mission/hover_active std_msgs/msg/Bool "{data: false}"

echo "[hover_simulate] done"
