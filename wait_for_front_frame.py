#!/usr/bin/env python3
"""Wait for a real front cube-face frame before starting a ROS bag."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main() -> int:
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = Node('x3_record_frame_check')
    received = False

    def on_image(message: Image) -> None:
        nonlocal received
        if message.width > 0 and message.height > 0 and message.data:
            print(
                f'Front image ready: {message.width}x{message.height}, '
                f'stamp {message.header.stamp.sec}.{message.header.stamp.nanosec:09d}',
                flush=True,
            )
            received = True

    subscription = node.create_subscription(
        Image, '/cubemap/front/image', on_image, qos_profile_sensor_data
    )
    deadline = time.monotonic() + timeout
    try:
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if not received:
        print(f'No front image received within {timeout:g} seconds.', file=sys.stderr)
    return 0 if received else 1


if __name__ == '__main__':
    raise SystemExit(main())
