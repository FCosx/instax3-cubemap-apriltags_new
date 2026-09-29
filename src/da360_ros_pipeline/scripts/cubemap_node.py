#!/usr/bin/env python3
"""Show and publish four horizontal cubemap faces from a ROS ERP image."""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from da360_ros_pipeline.cubemap_projection import build_face_map, remap_face


FACE_NAMES = ('front', 'right', 'back', 'left')
# Output face name -> source direction in the original ERP projection.
# This keeps the public topic names stable while applying the requested
# physical face remapping:
# right -> front, left -> back, back -> left, front -> right.
FACE_SOURCE_NAMES = {
    'front': 'right',
    'right': 'front',
    'back': 'left',
    'left': 'back',
}


class RosCubemapView(Node):
    def __init__(self, topic: str, face_size: int, show: bool, max_fps: int) -> None:
        super().__init__('da360_cubemap')
        # Four small remaps are faster and more predictable with one worker;
        # this also leaves CPU headroom for DA360 post-processing.
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.face_size = face_size
        self.show = show
        self.max_fps = max(0, int(max_fps))
        self._min_interval = 1.0 / self.max_fps if self.max_fps else 0.0
        self._last_process_time = 0.0
        self.maps = {
            name: build_face_map(face_size, FACE_SOURCE_NAMES[name])
            for name in FACE_NAMES
        }

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.face_publishers = {
            name: self.create_publisher(Image, f'/cubemap/{name}/image', qos)
            for name in FACE_NAMES
        }
        self.info_publishers = {
            name: self.create_publisher(CameraInfo, f'/cubemap/{name}/camera_info', qos)
            for name in FACE_NAMES
        }
        self.mosaic_publisher = self.create_publisher(
            Image,
            '/cubemap/horizontal/image',
            qos,
        )
        self.compressed_mosaic_publisher = self.create_publisher(
            CompressedImage,
            '/cubemap/horizontal/image/compressed',
            qos,
        )
        self.compressed_input = topic.endswith('/compressed')
        self.subscription = self.create_subscription(
            CompressedImage if self.compressed_input else Image,
            topic,
            self.image_callback,
            qos,
        )
        self.get_logger().info(
            f'Listening on {topic}; publishing four {face_size}x{face_size} '
            f'cubemap faces; gui={show}; max_fps={self.max_fps or "unlimited"}'
        )

    def image_callback(self, message: Image | CompressedImage) -> None:
        face_subscribers = {
            name: publisher.get_subscription_count() > 0
            for name, publisher in self.face_publishers.items()
        }
        mosaic_subscribers = self.mosaic_publisher.get_subscription_count() > 0
        compressed_mosaic_subscribers = (
            self.compressed_mosaic_publisher.get_subscription_count() > 0
        )
        need_mosaic = self.show or mosaic_subscribers or compressed_mosaic_subscribers
        needed_faces = {
            name for name, subscribed in face_subscribers.items() if subscribed
        }
        if need_mosaic:
            needed_faces.update(FACE_NAMES)
        if not needed_faces:
            return

        now = time.monotonic()
        if self._min_interval and now - self._last_process_time < self._min_interval:
            return
        self._last_process_time = now

        try:
            if self.compressed_input:
                encoded = np.frombuffer(message.data, dtype=np.uint8)
                erp_bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if erp_bgr is None:
                    raise ValueError('JPEG decode returned no image')
            else:
                erp_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except (CvBridgeError, ValueError) as exc:
            self.get_logger().error(f'ROS image conversion failed: {exc}')
            return

        faces: dict[str, np.ndarray] = {}
        for name in FACE_NAMES:
            if name not in needed_faces:
                continue
            map_x, map_y = self.maps[name]
            face = remap_face(
                erp_bgr,
                map_x,
                map_y,
                interpolation=cv2.INTER_LINEAR,
                border_mode=cv2.BORDER_WRAP,
            )
            # The X3 SDK panorama is horizontally mirrored. Restore the
            # pinhole view before publishing images with CameraInfo.
            face = cv2.flip(face, 1)
            faces[name] = face

            if face_subscribers[name]:
                output = self.bridge.cv2_to_imgmsg(face, encoding='bgr8')
                output.header = message.header
                output.header.frame_id = f'cubemap_{name}'
                info = CameraInfo()
                info.header = output.header
                info.width = self.face_size
                info.height = self.face_size
                # A cube face is a 90-degree pinhole view of the panorama.
                focal = self.face_size / 2.0
                center = (self.face_size - 1) / 2.0
                info.distortion_model = 'plumb_bob'
                info.d = [0.0] * 5
                info.k = [focal, 0.0, center,
                          0.0, focal, center,
                          0.0, 0.0, 1.0]
                info.r = [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
                info.p = [focal, 0.0, center, 0.0,
                          0.0, focal, center, 0.0,
                          0.0, 0.0, 1.0, 0.0]
                self.info_publishers[name].publish(info)
                self.face_publishers[name].publish(output)

        if need_mosaic:
            mosaic = np.vstack((
                np.hstack((faces['front'], faces['right'])),
                np.hstack((faces['back'], faces['left'])),
            ))
            for label, x, y in (
                ('FRONT', 8, 24),
                ('RIGHT', self.face_size + 8, 24),
                ('BACK', 8, self.face_size + 24),
                ('LEFT', self.face_size + 8, self.face_size + 24),
            ):
                cv2.putText(
                    mosaic,
                    label,
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            if mosaic_subscribers:
                mosaic_message = self.bridge.cv2_to_imgmsg(mosaic, encoding='bgr8')
                mosaic_message.header = message.header
                mosaic_message.header.frame_id = 'cubemap_horizontal'
                self.mosaic_publisher.publish(mosaic_message)
            if compressed_mosaic_subscribers:
                success, encoded = cv2.imencode(
                    '.jpg', mosaic, [cv2.IMWRITE_JPEG_QUALITY, 95]
                )
                if success:
                    compressed = CompressedImage()
                    compressed.header = message.header
                    compressed.header.frame_id = 'cubemap_horizontal'
                    compressed.format = 'jpeg'
                    compressed.data = encoded.tobytes()
                    self.compressed_mosaic_publisher.publish(compressed)

        if self.show:
            cv2.imshow('Cubemap: front | right / back | left', mosaic)
            cv2.waitKey(1)

    def destroy_node(self) -> bool:
        if self.show:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/equirectangular/image')
    parser.add_argument('--face-size', type=int, default=360)
    parser.add_argument('--max-fps', type=int, default=15)
    parser.add_argument('--gui', choices=('true', 'false'), default='true')
    parser.add_argument('--no-gui', action='store_true')
    args = parser.parse_args(remove_ros_args()[1:])
    if args.face_size <= 0:
        parser.error('--face-size must be a positive integer')
    if args.max_fps < 0:
        parser.error('--max-fps must be zero or a positive integer')

    show = args.gui == 'true' and not args.no_gui
    rclpy.init()
    node = RosCubemapView(args.topic, args.face_size, show, args.max_fps)
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
