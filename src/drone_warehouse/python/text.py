#!/usr/bin/env python3
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node

from drone_msgs.msg import BarcodeCapture


class BarcodeImagePublisher(Node):
    def __init__(self, image_path: str, barcode: str, image_format: str):
        super().__init__('barcode_image_test_publisher')
        self.publisher_ = self.create_publisher(
            BarcodeCapture,
            '/drone/barcode_capture',
            10
        )

        self.image_path = Path(image_path)
        self.barcode = barcode
        self.image_format = image_format.lower().strip('.')

        self.timer = self.create_timer(1.0, self.publish_once)
        self.published = False

    def publish_once(self):
        if self.published:
            return

        if not self.image_path.exists():
            self.get_logger().error(f'图片不存在: {self.image_path}')
            rclpy.shutdown()
            return

        image_bytes = self.image_path.read_bytes()

        if not image_bytes:
            self.get_logger().error('图片文件为空')
            rclpy.shutdown()
            return

        msg = BarcodeCapture()
        msg.barcode = self.barcode
        msg.image_data = list(image_bytes)
        msg.image_format = self.image_format
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'已发布图片: barcode={msg.barcode}, '
            f'format={msg.image_format}, bytes={len(image_bytes)}'
        )

        self.published = True
        self.destroy_timer(self.timer)
        self.create_timer(0.5, self.shutdown_later)

    def shutdown_later(self):
        self.get_logger().info('测试消息发送完成，准备退出')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True, help='图片路径')
    parser.add_argument('--barcode', default='PKG001|SKU001|A-1-1', help='条码文本')
    parser.add_argument('--format', default='jpg', help='图片格式，如 jpg/png')
    args = parser.parse_args()

    rclpy.init()
    node = BarcodeImagePublisher(
        image_path=args.image,
        barcode=args.barcode,
        image_format=args.format,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()