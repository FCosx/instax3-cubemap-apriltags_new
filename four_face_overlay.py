#!/usr/bin/env python3
"""Annotate four AprilTag feeds, publish a 2x2 view, and log each face."""

from __future__ import annotations

import csv
import sys
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


FACES = ('front', 'right', 'back', 'left')


def stamp_key(message):
    return message.header.stamp.sec, message.header.stamp.nanosec


class FourFaceOverlay(Node):
    def __init__(self, output_prefix: Path):
        super().__init__('x3_four_face_overlay')
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.images = {face: OrderedDict() for face in FACES}
        self.latest = {}
        self.last_mosaic_time = 0.0
        self.mosaic_count = 0
        self.files = {}
        self.writers = {}
        self.face_publishers = {}
        for face in FACES:
            path = Path(f'{output_prefix}_{face}_detections.csv')
            path.parent.mkdir(parents=True, exist_ok=True)
            self.files[face] = path.open('w', newline='')
            self.writers[face] = csv.writer(self.files[face])
            self.writers[face].writerow([
                'stamp_sec', 'stamp_nanosec', 'detected', 'id',
                'decision_margin', 'centre_x', 'centre_y',
            ])
            self.face_publishers[face] = self.create_publisher(
                Image, f'/apriltag/{face}/image_annotated', 10,
            )
            self.create_subscription(
                Image, f'/cubemap/{face}/image',
                lambda message, name=face: self.on_image(name, message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                AprilTagDetectionArray, f'/apriltag/{face}/detections',
                lambda message, name=face: self.on_detections(name, message), 10,
            )
        self.mosaic_publisher = self.create_publisher(
            Image, '/apriltag/four_views/image_annotated', 10,
        )

    def on_image(self, face, message):
        queue = self.images[face]
        queue[stamp_key(message)] = message
        while len(queue) > 100:
            queue.popitem(last=False)

    def on_detections(self, face, message):
        sec, nanosec = stamp_key(message)
        if not message.detections:
            self.writers[face].writerow([
                sec, nanosec, 0, '', '', '', '',
            ])
        for detection in message.detections:
            self.writers[face].writerow([
                sec, nanosec, 1, detection.id,
                detection.decision_margin, detection.centre.x,
                detection.centre.y,
            ])
        self.files[face].flush()

        image_message = self.images[face].pop((sec, nanosec), None)
        if image_message is None:
            return
        image = self.bridge.imgmsg_to_cv2(image_message, 'bgr8').copy()
        for detection in message.detections:
            corners = [(round(p.x), round(p.y)) for p in detection.corners]
            for index in range(4):
                cv2.line(image, corners[index], corners[(index + 1) % 4],
                         (0, 255, 0), 2)
            centre = (round(detection.centre.x), round(detection.centre.y))
            cv2.circle(image, centre, 4, (0, 0, 255), -1)
            cv2.putText(image, f'{detection.family}:{detection.id}',
                        corners[0], cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)
        output = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        output.header = image_message.header
        self.face_publishers[face].publish(output)
        self.latest[face] = image
        now = time.monotonic()
        if len(self.latest) == len(FACES) and now - self.last_mosaic_time >= 0.1:
            self.publish_mosaic(image_message.header)
            self.last_mosaic_time = now

    def publish_mosaic(self, header):
        height = min(image.shape[0] for image in self.latest.values())
        width = min(image.shape[1] for image in self.latest.values())
        tiles = []
        for face in FACES:
            tile = cv2.resize(self.latest[face], (width, height),
                              interpolation=cv2.INTER_AREA).copy()
            cv2.rectangle(tile, (0, 0), (92, 28), (0, 0, 0), -1)
            cv2.putText(tile, face.upper(), (6, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            tiles.append(tile)
        mosaic = np.vstack((np.hstack(tiles[:2]), np.hstack(tiles[2:])))
        output = self.bridge.cv2_to_imgmsg(mosaic, 'bgr8')
        output.header.stamp = header.stamp
        output.header.frame_id = 'cubemap_front'
        self.mosaic_publisher.publish(output)
        self.mosaic_count += 1
        if self.mosaic_count == 1:
            self.get_logger().info(
                'Publishing four annotated views on '
                '/apriltag/four_views/image_annotated'
            )

    def destroy_node(self):
        self.get_logger().info(f'Published {self.mosaic_count} four-view images')
        for file in self.files.values():
            file.close()
        return super().destroy_node()


def main():
    if len(sys.argv) < 2:
        raise SystemExit('Usage: four_face_overlay.py OUTPUT_PREFIX [--ros-args ...]')
    output_prefix = Path(sys.argv[1]).expanduser()
    rclpy.init(args=[sys.argv[0], *sys.argv[2:]])
    node = FourFaceOverlay(output_prefix)
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
