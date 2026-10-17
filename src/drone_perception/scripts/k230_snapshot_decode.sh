#!/usr/bin/env bash
# ==============================================================================
# K230 Snapshot Decoder — 订阅 ROS 2 图像话题，解码 JPEG 并保存为本地文件
#
# 环境变量：
#   K230_SNAPSHOT_TOPIC    - 订阅的 ROS 2 话题，默认 /drone/image
#   K230_SNAPSHOT_OUT_DIR  - 保存目录，默认 ~/Desktop/k230_snapshots
#
# 输出文件名格式: {barcode}_{timestamp_sec}_{nanosec}_{序号}.jpg
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPIC_NAME="${K230_SNAPSHOT_TOPIC:-/drone/image}"
OUT_DIR="${K230_SNAPSHOT_OUT_DIR:-${HOME}/Desktop/k230_snapshots}"

# 向上查找 ROS 2 工作空间根目录（包含 install/setup.bash）
WORKSPACE_ROOT="${SCRIPT_DIR}"
while [ "${WORKSPACE_ROOT}" != "/" ] && [ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]; do
  WORKSPACE_ROOT="$(dirname "${WORKSPACE_ROOT}")"
done

mkdir -p "${OUT_DIR}"

# Source ROS 2 Humble 环境 (如果存在)
if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "/opt/ros/humble/setup.bash"
  set -u
fi

# Source 本地工作空间环境
if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "${WORKSPACE_ROOT}/install/setup.bash"
  set -u
else
  echo "Workspace setup not found: ${WORKSPACE_ROOT}/install/setup.bash" >&2
  exit 1
fi

export K230_SNAPSHOT_TOPIC="${TOPIC_NAME}"
export K230_SNAPSHOT_OUT_DIR="${OUT_DIR}"

python3 -u <<'PY'
#
# 内嵌 Python 脚本：订阅 BarcodeCapture 消息，解码 JPEG 并保存
#
import os
import re

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_msgs.msg import BarcodeCapture

TOPIC_NAME = os.environ.get("K230_SNAPSHOT_TOPIC", "/drone/image")
OUT_DIR = os.environ.get("K230_SNAPSHOT_OUT_DIR", os.path.expanduser("~/Desktop/k230_snapshots"))


def sanitize(text: str) -> str:
    """将帧名/条码清理为安全的文件名字符"""
    text = text.strip()
    if not text:
        return "snapshot"
    text = re.sub(r"[^A-Za-z0-9._-]+", "_", text)
    return text.strip("._-") or "snapshot"


class SnapshotDecoder(Node):
    def __init__(self) -> None:
        super().__init__("k230_snapshot_decoder")
        self._count = 0
        # best_effort 与发布端保持一致
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._sub = self.create_subscription(
            BarcodeCapture, TOPIC_NAME, self._on_frame, qos
        )
        self.get_logger().info(f"listening on {TOPIC_NAME}, saving to {OUT_DIR}")

    def _make_path(self, msg: BarcodeCapture) -> str:
        """生成输出文件路径，包含条码名、时间戳和序号"""
        self._count += 1
        barcode = sanitize(msg.barcode)
        filename = (
            f"{barcode}_{int(msg.stamp.sec)}_{int(msg.stamp.nanosec)}_{self._count:04d}.jpg"
        )
        return os.path.join(OUT_DIR, filename)

    def _on_frame(self, msg: BarcodeCapture) -> None:
        """收到一帧：跳过非 JPEG，尝试 OpenCV 解码，失败则写入原始字节"""
        if msg.image_format and msg.image_format.lower() not in ("jpeg", "jpg"):
            self.get_logger().warn(f"skip non-jpeg image_format={msg.image_format!r}")
            return

        raw = bytes(msg.image_data)
        if not raw:
            self.get_logger().warn("empty image_data")
            return

        path = self._make_path(msg)

        # 尝试用 OpenCV 解码为彩色图像并保存
        np_buffer = np.frombuffer(raw, dtype=np.uint8)
        image = cv2.imdecode(np_buffer, cv2.IMREAD_COLOR)

        if image is not None:
            if cv2.imwrite(path, image):
                self.get_logger().info(f"saved {path}")
                return
            self.get_logger().error(f"failed to write {path}")
            return

        # OpenCV 解码失败，回退：直接写入原始 JPEG 字节
        with open(path, "wb") as handle:
            handle.write(raw)
        self.get_logger().warn(f"decode failed, raw bytes written to {path}")


def main() -> None:
    rclpy.init()
    node = SnapshotDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
