#!/usr/bin/env bash
# Check the external prerequisites and build both ROS packages once.
set -Eeuo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ros_setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
[[ -r "$ros_setup" ]] || { echo "ROS 2 setup not found: $ros_setup" >&2; exit 1; }

for command in cmake pkg-config gnome-terminal rviz2; do
  command -v "$command" >/dev/null || {
    echo "Missing command: $command" >&2
    exit 1
  }
done

set +u
source "$ros_setup"
set -u
for package in apriltag_ros apriltag_msgs cv_bridge; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || {
    echo "Missing ROS package: $package" >&2
    exit 1
  }
done
python3 -c 'import cv2, numpy, rclpy, rosbag2_py, yaml' || {
  echo 'Missing Python ROS/OpenCV dependencies' >&2
  exit 1
}
pkg-config --exists libavcodec libavformat libavutil libswscale || {
  echo 'Missing FFmpeg development libraries (libavcodec, libavformat, libavutil, libswscale)' >&2
  exit 1
}

export INSTA360_SDK_DIR="$("$root/find_x3_sdk.sh")"
echo "CameraSDK: $INSTA360_SDK_DIR"
"$root/build.sh"
echo 'Setup complete. Use ./start_x3_four_rviz.sh record BAG_PATH 0.06'
