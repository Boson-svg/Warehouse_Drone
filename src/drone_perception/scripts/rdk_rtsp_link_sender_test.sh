#!/usr/bin/env bash
# rdk_rtsp_link_sender_test.sh - repeatable RDK X5 D435i H.264/RTSP sender test.
#
# Part of the RDK X5 -> OrangePi 5 Ultra video-link validation (T1-T3 / T5 / T8).
# It runs the standalone V4L2 sender + MediaMTX for a bounded duration, writes a
# timestamped log bundle, and can optionally capture pidstat/vmstat baselines.
#
# The D435I_start.service must be stopped first (the sender and qr_vision_node
# must not both hold the D435i). After the test the service can be restarted.
#
# Usage:
#   rdk_rtsp_link_sender_test.sh [BITRATE_KBPS] [DURATION_SEC] [OUT_DIR]
#     BITRATE_KBPS   H.264 CBR bitrate (default: 1500)
#     DURATION_SEC   sender run duration (default: 120)
#     OUT_DIR        log bundle dir (default: /tmp/rtsp_link_test)
#
# Environment overrides (same as rdk_d435i_mediamtx_sender.sh):
#   WIDTH HEIGHT FPS GOP RTSP_PORT RTSP_PATH
#
# NOTE: the current phone-hotspot WiFi (2.4 GHz, OrangePi ~-78 dBm) sustains
# only ~2 Mbit/s. 3000 kbit/s cannot reach 29 fps; 1500 kbit/s does. Use 1500
# for acceptance on this network and re-test 3000 on a 5 GHz/wired link.
set -Eeuo pipefail

BITRATE_KBPS="${1:-1500}"
DURATION_SEC="${2:-120}"
OUT_DIR="${3:-/tmp/rtsp_link_test}"
MEDIAMTX_BIN="${MEDIAMTX_BIN:-/home/sunrise/mediamtx/mediamtx}"

LOG_FILE="$OUT_DIR/rdk_sender.log"
PIDSTAT_FILE="$OUT_DIR/rdk_sender_pidstat.log"
VMSTAT_FILE="$OUT_DIR/system_vmstat.log"
SENDER_PID_FILE="$OUT_DIR/sender.pid"

if ! command -v ss >/dev/null; then
  echo "ss not found (iproute2 required)" >&2
  exit 127
fi

mkdir -p "$OUT_DIR"
: > "$LOG_FILE"
echo "[test] start: $(date -u +%FT%TZ) bitrate=${BITRATE_KBPS} duration=${DURATION_SEC}s" | tee -a "$LOG_FILE"

# The sender must be the only D435i opener; the service must be stopped.
if systemctl is-active --quiet D435I_start.service 2>/dev/null; then
  echo "D435I_start.service is active; stop it first (sender would contend for the camera)" >&2
  exit 1
fi

# Fail fast if another RTSP server is already on the port.
if ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE ":${RTSP_PORT:-8554}$"; then
  echo "TCP port ${RTSP_PORT:-8554} already in use" >&2
  exit 1
fi

SENDER_SCRIPT=~/warehouse_ws/install/drone_perception/lib/drone_perception/start_rdk_d435i_sender.sh
if [[ ! -x "$SENDER_SCRIPT" ]]; then
  echo "sender script not found: $SENDER_SCRIPT (build drone_perception first)" >&2
  exit 1
fi

cleanup()
{
  trap - EXIT INT TERM
  if [[ -f "$SENDER_PID_FILE" ]]; then
    kill "$(cat "$SENDER_PID_FILE")" 2>/dev/null || true
  fi
  # MediaMTX + ffmpeg are children of the sender script chain; kill leftovers.
  fuser -k 8554/tcp 2>/dev/null || true
  fuser -k /dev/d435i_color 2>/dev/null || true
  pkill -x ffmpeg 2>/dev/null || true
  echo "[test] cleaned up; 8554: $(ss -lnt | grep -c 8554)" >> "$LOG_FILE"
}
trap cleanup EXIT INT TERM

# Optional baseline monitors (background).
if command -v pidstat >/dev/null; then
  pidstat -dur 5 > "$PIDSTAT_FILE" 2>&1 &
fi
if command -v vmstat >/dev/null; then
  vmstat 5 > "$VMSTAT_FILE" 2>&1 &
fi

echo "[test] starting sender (${BITRATE_KBPS} kbit/s, ${DURATION_SEC}s)..." | tee -a "$LOG_FILE"
env BITRATE_KBPS="$BITRATE_KBPS" timeout "${DURATION_SEC}s" "$SENDER_SCRIPT" >> "$LOG_FILE" 2>&1 &
echo $! > "$SENDER_PID_FILE"
SENDER_RUN_PID=$!

# Wait for RTSP port and for the pipeline to publish.
for _ in $(seq 1 60); do
  if ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE ":8554$"; then
    break
  fi
  if ! kill -0 "$SENDER_RUN_PID" 2>/dev/null; then
    echo "[test] sender exited early" >&2
    exit 1
  fi
  sleep 0.5
done

if ss -lnt 2>/dev/null | awk '{print $4}' | grep -qE ":8554$"; then
  echo "[test] RTSP up on :8554, publishing ${WIDTH:-640}x${HEIGHT:-480}@${FPS:-30} H.264 ${BITRATE_KBPS} kbit/s" | tee -a "$LOG_FILE"
else
  echo "[test] RTSP did not come up in time" >&2
  exit 1
fi

wait "$SENDER_RUN_PID"
echo "[test] sender run finished: $(date -u +%FT%TZ)" | tee -a "$LOG_FILE"

# Kill background monitors.
for p in $(jobs -p); do
  kill "$p" 2>/dev/null || true
done

echo "[test] log bundle: $OUT_DIR" | tee -a "$LOG_FILE"
