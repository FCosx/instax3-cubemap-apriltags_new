#!/usr/bin/env python3
"""Give each face's tag frames unique names and connect nominal camera axes."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


FACE_YAW_DEGREES = {'right': 90, 'back': 180, 'left': -90}
FACES = ('front', 'right', 'back', 'left')


class FourFaceTF(Node):
    def __init__(self):
        super().__init__('x3_four_face_tf')
        self.pub = self.create_publisher(TFMessage, '/tf', 10)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for face, degrees in FACE_YAW_DEGREES.items():
            angle = math.radians(degrees) / 2.0
            transform = TransformStamped()
            transform.header.frame_id = 'cubemap_front'
            transform.child_frame_id = f'cubemap_{face}'
            transform.transform.rotation.y = math.sin(angle)
            transform.transform.rotation.w = math.cos(angle)
            transforms.append(transform)
        self.static.sendTransform(transforms)
        self.raw_tf_subscriptions = []
        for face in FACES:
            subscription = self.create_subscription(
                TFMessage, f'/apriltag/{face}/tf_raw',
                lambda message, name=face: self.on_tf(name, message), 10,
            )
            self.raw_tf_subscriptions.append(subscription)

    def on_tf(self, face, message):
        if not message.transforms:
            return
        output = TFMessage()
        for transform in message.transforms:
            # The same physical tag may appear in multiple faces at once.
            # Keep each observation as its own TF child to avoid competing
            # parents and preserve the detector's original camera frame.
            transform.child_frame_id = f'{face}_{transform.child_frame_id}'
            output.transforms.append(transform)
        self.pub.publish(output)


def main():
    rclpy.init()
    node = FourFaceTF()
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
