#!/usr/bin/env bash
# orangepi_rtsp_mpp_probe_run.sh - repeatable OrangePi 5 Ultra MPP decode-probe run.
#
# Part of the RDK X5 -> OrangePi video-link validation (T5-T8). Wraps the
# orangepi_rtsp_mpp_probe with baseline monitoring and a timestamped log bundle.
#
# Usage:
#   orangepi_rtsp_mpp_probe_run.sh [PROTOCOL] [DURATION_SEC] [URL] [FORMAT] [OUT_DIR]
#     PROTOCOL      udp|tcp (default: udp)
#     DURATION_SEC  run duration (default: 60)
#     URL           RTSP URL (default: rtsp://192.168.3.114:8554/d435i)
#     FORMAT        nv12|rgb|bgr|auto (default: nv12)
#     OUT_DIR       log bundle dir (default: /tmp/rtsp_probe_test)
#
# The probe binary is resolved from, in order: $ORANGEPI_RTSP_MPP_PROBE, the
# ROS package install, or /tmp/orangepi_rtsp_mpp_probe (standalone g++ build).
set -Eeuo pipefail

PROTOCOL="${1:-udp}"
DURATION_SEC="${2:-60}"
URL="${3:-rtsp://192.168.3.114:8554/d435i}"
FORMAT="${4:-nv12}"
OUT_DIR="${5:-/tmp/rtsp_probe_test}"

PROBE_BIN="${ORANGEPI_RTSP_MPP_PROBE:-}"
if [[ -z "$PROBE_BIN" ]] && command -v ros2 >/dev/null 2>&1; then
  pkg_prefix=$(ros2 pkg prefix drone_perception 2>/dev/null || true)
  if [[ -n "$pkg_prefix" ]]; then
    PROBE_BIN="$pkg_prefix/lib/drone_perception/orangepi_rtsp_mpp_probe"
  fi
fi
if [[ -z "$PROBE_BIN" || ! -x "$PROBE_BIN" ]]; then
  PROBE_BIN=/tmp/orangepi_rtsp_mpp_probe
fi
if [[ ! -x "$PROBE_BIN" ]]; then
  echo "probe binary not found: $PROBE_BIN (build standalone: g++ \$(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0) src/orangepi_rtsp_mpp_probe.cpp -o \$PROBE_BIN)" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
PROBE_LOG="$OUT_DIR/probe.log"
PIDSTAT_LOG="$OUT_DIR/probe_pidstat.log"
VMSTAT_LOG="$OUT_DIR/system_vmstat.log"

echo "[run] start: $(date -u +%FT%TZ) protocol=${PROTOCOL} duration=${DURATION_SEC}s format=${FORMAT} url=${URL}" | tee "$PROBE_LOG"

if command -v pidstat >/dev/null; then
  pidstat -dur 5 > "$PIDSTAT_LOG" 2>&1 &
fi
if command -v vmstat >/dev/null; then
  vmstat 5 > "$VMSTAT_LOG" 2>&1 &
fi

"$PROBE_BIN" \
  --url "$URL" \
  --protocol "$PROTOCOL" \
  --latency-ms "$([ "$PROTOCOL" = tcp ] && echo 100 || echo 50)" \
  --format "$FORMAT" \
  --duration-sec "$DURATION_SEC" \
  --stats-period-sec 10 2>&1 | tee -a "$PROBE_LOG"

for p in $(jobs -p); do
  kill "$p" 2>/dev/null || true
done
echo "[run] finished: $(date -u +%FT%TZ) bundle=$OUT_DIR" | tee -a "$PROBE_LOG"
