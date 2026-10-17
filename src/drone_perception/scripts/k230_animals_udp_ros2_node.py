#!/usr/bin/env python3
"""Receive K230 animal detections over UDP and publish ROS 2 topics."""

import argparse
import json
import socket
import threading
from typing import Any, Dict

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String


class K230AnimalsUdpNode(Node):
    def __init__(self, listen_host: str, listen_port: int) -> None:
        super().__init__("k230_animals_udp_node")
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.stop_event = threading.Event()
        self.detect_pub = self.create_publisher(String, "/k230/animals/detect", 10)
        self.center_pub = self.create_publisher(Point, "/k230/animals/center", 10)
        self.get_logger().info(f"starting UDP listener on {self.listen_host}:{self.listen_port}")
        self.server_thread = threading.Thread(target=self._serve, daemon=True)
        self.server_thread.start()

    def destroy_node(self) -> bool:
        self.stop_event.set()
        return super().destroy_node()

    def _serve(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.listen_host, self.listen_port))
            sock.settimeout(0.5)
            self.get_logger().info(f"listening UDP on {self.listen_host}:{self.listen_port}")
            while rclpy.ok() and not self.stop_event.is_set():
                try:
                    data, addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self.stop_event.is_set():
                        self.get_logger().error(f"UDP receive failed: {exc}")
                    continue
                self._handle_packet(data, addr)

    def _handle_packet(self, data: bytes, addr: tuple[str, int]) -> None:
        try:
            raw = data.decode("utf-8").strip()
        except UnicodeDecodeError as exc:
            self.get_logger().warn(f"bad utf-8 from {addr}: {exc}")
            return
        if not raw:
            return

        try:
            payload = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"bad json from {addr}: {exc}: {raw[:120]}")
            return

        msg = String()
        msg.data = raw
        self.detect_pub.publish(msg)

        if not bool(payload.get("valid", False)):
            return
        self._publish_center(payload)

    def _publish_center(self, payload: Dict[str, Any]) -> None:
        try:
            cx = float(payload["cx"])
            cy = float(payload["cy"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(f"valid payload missing center: {payload}")
            return

        center = Point()
        center.x = cx
        center.y = cy
        center.z = 0.0
        self.center_pub.publish(center)

        label = payload.get("label", "unknown")
        score = payload.get("score", 0.0)
        norm_x = payload.get("norm_x", 0.0)
        norm_y = payload.get("norm_y", 0.0)
        self.get_logger().info(
            f"detect label={label} score={score} center=({cx:.1f},{cy:.1f}) norm=({norm_x},{norm_y})"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=60001)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print(f"Starting K230 animals UDP ROS2 node on {args.listen_host}:{args.listen_port}", flush=True)
    rclpy.init()
    node = K230AnimalsUdpNode(args.listen_host, args.listen_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
