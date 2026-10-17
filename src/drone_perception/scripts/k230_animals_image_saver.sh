#!/usr/bin/env bash
# Save BarcodeCapture images from /drone/image to ~/Desktop/animals_detect_image.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOPIC_NAME="${K230_ANIMALS_IMAGE_TOPIC:-/drone/image}"
OUT_DIR="${K230_ANIMALS_IMAGE_OUT_DIR:-${HOME}/Desktop/animals_detect_image}"

WORKSPACE_ROOT="${SCRIPT_DIR}"
while [ "${WORKSPACE_ROOT}" != "/" ] && [ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]; do
  WORKSPACE_ROOT="$(dirname "${WORKSPACE_ROOT}")"
done

mkdir -p "${OUT_DIR}"

if [ -f "/opt/ros/humble/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/humble/setup.bash"
  set -u
fi

if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/install/setup.bash"
  set -u
else
  echo "Workspace setup not found: ${WORKSPACE_ROOT}/install/setup.bash" >&2
  exit 1
fi

export K230_ANIMALS_IMAGE_TOPIC="${TOPIC_NAME}"
export K230_ANIMALS_IMAGE_OUT_DIR="${OUT_DIR}"

python3 -u <<'PY'
import os
import re

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_msgs.msg import BarcodeCapture

TOPIC_NAME = os.environ.get("K230_ANIMALS_IMAGE_TOPIC", "/drone/image")
OUT_DIR = os.environ.get(
    "K230_ANIMALS_IMAGE_OUT_DIR",
    os.path.expanduser("~/Desktop/animals_detect_image"),
)


def sanitize(text: str) -> str:
    text = (text or "").strip()
    if not text:
        return "动物"
    text = re.sub(r"[^\w._-]+", "_", text)
    return text.strip("._-") or "动物"


def image_extension(image_format: str) -> str:
    fmt = (image_format or "jpeg").strip().lower()
    if fmt in ("jpeg", "jpg"):
        return "jpg"
    if fmt == "png":
        return "png"
    return "bin"


def looks_like_jpeg(raw: bytes) -> bool:
    return len(raw) >= 4 and raw[0] == 0xFF and raw[1] == 0xD8 and raw[-2] == 0xFF and raw[-1] == 0xD9


class AnimalsImageSaver(Node):
    def __init__(self) -> None:
        super().__init__("k230_animals_image_saver")
        self._count = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._sub = self.create_subscription(
            BarcodeCapture,
            TOPIC_NAME,
            self._on_image,
            qos,
        )
        self.get_logger().info(f"listening on {TOPIC_NAME}, saving to {OUT_DIR}")

    def _make_path(self, msg: BarcodeCapture) -> str:
        self._count += 1
        name = sanitize(msg.barcode)
        ext = image_extension(msg.image_format)
        filename = f"{name}.{ext}"
        return os.path.join(OUT_DIR, filename)

    def _on_image(self, msg: BarcodeCapture) -> None:
        raw = bytes(msg.image_data)
        if not raw:
            self.get_logger().warn("skip empty image_data")
            return

        if msg.image_format.lower() in ("jpeg", "jpg") and not looks_like_jpeg(raw):
            self.get_logger().warn(
                f"jpeg marker check failed: barcode={msg.barcode!r} bytes={len(raw)}"
            )

        path = self._make_path(msg)
        with open(path, "wb") as handle:
            handle.write(raw)

        self.get_logger().info(
            f"saved {path} bytes={len(raw)} format={msg.image_format!r} barcode={msg.barcode!r}"
        )


def main() -> None:
    rclpy.init()
    node = AnimalsImageSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
