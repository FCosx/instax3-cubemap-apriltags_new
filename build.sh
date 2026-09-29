#!/usr/bin/env bash
set -Eeuo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
sdk_dir="$(./find_x3_sdk.sh)"
export INSTA360_SDK_DIR="$sdk_dir"

set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u

cmake -S src/insta360_ros_driver -B build/insta360_ros_driver \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/install" \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DINSTA360_SDK_DIR="$sdk_dir"
cmake --build build/insta360_ros_driver -j 2
cmake --install build/insta360_ros_driver

export AMENT_PREFIX_PATH="$PWD/install:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$PWD/install:${CMAKE_PREFIX_PATH:-}"
cmake -S src/da360_ros_pipeline -B build/da360_ros_pipeline \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/install" \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build build/da360_ros_pipeline -j 2
cmake --install build/da360_ros_pipeline
