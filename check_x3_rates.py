#!/usr/bin/env python3
"""Measure camera and AprilTag callback rates with one ROS 2 subscriber node."""

from __future__ import annotations

import sys
import time

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


FACES = ('front', 'right', 'back', 'left')


class RateMonitor(Node):
    def __init__(self):
        super().__init__('x3_rate_monitor')
        self.arrivals = {
            face: {'camera': [], 'detector': []} for face in FACES
        }
        self.tagged_frames = {face: 0 for face in FACES}
        self.subscriptions_owned = []
        for face in FACES:
            self.subscriptions_owned.append(self.create_subscription(
                Image, f'/cubemap/{face}/image',
                lambda _message, name=face: self.on_camera(name),
                qos_profile_sensor_data,
            ))
            self.subscriptions_owned.append(self.create_subscription(
                AprilTagDetectionArray, f'/apriltag/{face}/detections',
                lambda message, name=face: self.on_detector(name, message),
                qos_profile_sensor_data,
            ))

    def on_camera(self, face):
        self.arrivals[face]['camera'].append(time.monotonic_ns())

    def on_detector(self, face, message):
        self.arrivals[face]['detector'].append(time.monotonic_ns())
        if message.detections:
            self.tagged_frames[face] += 1


def rate(arrivals):
    if len(arrivals) < 2 or arrivals[-1] == arrivals[0]:
        return None
    return (len(arrivals) - 1) * 1e9 / (arrivals[-1] - arrivals[0])


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: check_x3_rates.py SECONDS')
    seconds = float(sys.argv[1])
    if seconds <= 0:
        raise SystemExit('SECONDS must be greater than zero')
    rclpy.init()
    node = RateMonitor()
    try:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        print(f'{"Face":<7} {"Camera":>13} {"Detector":>13} {"Tag frames":>12}')
        for face in FACES:
            camera = node.arrivals[face]['camera']
            detector = node.arrivals[face]['detector']
            camera_hz = rate(camera)
            detector_hz = rate(detector)
            camera_text = f'{camera_hz:.2f} Hz ({len(camera)})' if camera_hz else f'-- ({len(camera)})'
            detector_text = f'{detector_hz:.2f} Hz ({len(detector)})' if detector_hz else f'-- ({len(detector)})'
            print(f'{face:<7} {camera_text:>13} {detector_text:>13} {node.tagged_frames[face]:>12}')
        if not any(node.arrivals[face]['camera'] for face in FACES):
            print('No camera frames arrived. Start the four-view record or replay script first.', file=sys.stderr)
            result = 1
        else:
            result = 0
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
