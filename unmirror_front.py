#!/usr/bin/env python3
"""Correct the mirrored front images in bags recorded before the X3 fix."""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class UnmirrorFront(Node):
    def __init__(self):
        super().__init__('unmirror_front')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, '/cubemap/front/image', qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Image,
            '/cubemap/front/image/mirrored',
            self.callback,
            qos_profile_sensor_data,
        )

    def callback(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        corrected = self.bridge.cv2_to_imgmsg(cv2.flip(image, 1), encoding='bgr8')
        corrected.header = message.header
        self.publisher.publish(corrected)


def main():
    rclpy.init()
    node = UnmirrorFront()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
