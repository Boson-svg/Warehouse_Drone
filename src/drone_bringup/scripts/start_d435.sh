#!/usr/bin/env bash
set -Eeuo pipefail

# Wait for the D435i to finish USB enumeration after boot before opening it.
sleep 10

# ROS 2 setup scripts may read optional variables (AMENT_TRACE_SETUP_FILES)
# before defining them; with set -u that aborts sourcing, so relax it here.
set +u
source /opt/ros/humble/setup.bash
source ~/warehouse_ws/install/setup.bash
set -u

MEDIAMTX_BIN=${MEDIAMTX_BIN:-/home/sunrise/mediamtx/mediamtx}
RTSP_PORT=${RTSP_PORT:-8554}
PARAMS_FILE=/home/sunrise/warehouse_ws/src/drone_perception/config/d435_direct_detection.yaml

mediamtx_pid=""
runtime_mediamtx_config=""

cleanup()
{
  trap - EXIT INT TERM
  if [[ -n "$mediamtx_pid" ]]; then
    kill "$mediamtx_pid" 2>/dev/null || true
    wait "$mediamtx_pid" 2>/dev/null || true
  fi
  if [[ -n "$runtime_mediamtx_config" ]]; then
    rm -f "$runtime_mediamtx_config"
  fi
}
trap cleanup EXIT INT TERM

# MediaMTX serves the RTSP publish target for qr_vision_node's video-stream
# worker (FFmpeg -c:v copy to rtsp://127.0.0.1:8554/d435i).
[[ -x "$MEDIAMTX_BIN" ]] || { echo "MediaMTX not executable: $MEDIAMTX_BIN" >&2; exit 1; }

if ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${RTSP_PORT}$"; then
  echo "TCP port ${RTSP_PORT} already in use; stop the existing RTSP server first" >&2
  exit 1
fi

runtime_mediamtx_config=$(mktemp /tmp/d435i-mediamtx.XXXXXX.yml)
printf '%s\n' \
  'logLevel: info' \
  "rtspAddress: :${RTSP_PORT}" \
  'rtspTransports: [udp, tcp]' \
  '' \
  'paths:' \
  '  d435i:' \
  '    source: publisher' > "$runtime_mediamtx_config"

"$MEDIAMTX_BIN" "$runtime_mediamtx_config" &
mediamtx_pid=$!

for _ in $(seq 1 50); do
  if ! kill -0 "$mediamtx_pid" 2>/dev/null; then
    wait "$mediamtx_pid" || true
    echo "MediaMTX exited during startup" >&2
    exit 1
  fi
  if ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${RTSP_PORT}$"; then
    break
  fi
  sleep 0.1
done
if ! ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE "(^|:)${RTSP_PORT}$"; then
  echo "MediaMTX did not open TCP port ${RTSP_PORT}" >&2
  exit 1
fi

echo "MediaMTX up on :${RTSP_PORT}; starting qr_vision_node (detection + video stream)"

node_exit=0

# Direct-capture chain (stages A-D + video stream): qr_vision_node opens the
# D435i via librealsense (YUYV 640x480@30), preprocesses through Nano2D into
# 640x640 NV12 for the BPU, runs QR/OCR/capture, and its video-stream worker
# encodes the same frame to H.264/RTSP via MediaMTX. systemd sends SIGTERM to
# the whole service cgroup, so the node, FFmpeg and MediaMTX all shut down
# together; the script then reports the node exit code back to systemd.
ros2 run drone_perception qr_vision_node \
  --ros-args \
  --params-file "$PARAMS_FILE" || node_exit=$?
exit "$node_exit"

# --- rollback: old realsense2_camera_node path (do not delete) ---
# ros2 launch realsense2_camera rs_launch.py \
#   enable_color:=true \
#   rgb_camera.color_profile:=640,480,30 \
#   enable_depth:=false \
#   enable_rgbd:=false \
#   enable_sync:=false \
#   align_depth.enable:=false
