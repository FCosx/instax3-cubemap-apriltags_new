#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
DRIVER_SETUP="${ROOT_DIR}/install/share/insta360_ros_driver/local_setup.bash"
PIPELINE_SETUP="${ROOT_DIR}/install/share/da360_ros_pipeline/local_setup.bash"
SDK_DIR="${INSTA360_SDK_DIR:-}"
LAUNCH_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./start_x3_cubemap.sh [--sdk-dir PATH] [launch arguments]

Starts the bundled Insta360 ROS driver, ERP conversion, and cubemap publisher.

Options:
  --sdk-dir PATH  External CameraSDK root.
  -h, --help      Show this help.

Examples:
  ./start_x3_cubemap.sh --sdk-dir /path/to/CameraSDK
  INSTA360_SDK_DIR=/path/to/CameraSDK ./start_x3_cubemap.sh cubemap_gui:=false
EOF
}

die() {
  echo "[x3-cubemap] ERROR: $*" >&2
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sdk-dir)
      [[ $# -ge 2 ]] || die "--sdk-dir requires a path"
      SDK_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ -n "${X3_CUBEMAP_IMAGE_ENCODING:-}" ]]; then
  [[ "$X3_CUBEMAP_IMAGE_ENCODING" == bgr8 ||
     "$X3_CUBEMAP_IMAGE_ENCODING" == mono8 ]] ||
    die "X3_CUBEMAP_IMAGE_ENCODING must be bgr8 or mono8"
  LAUNCH_ARGS+=("cubemap_image_encoding:=$X3_CUBEMAP_IMAGE_ENCODING")
fi

[[ -r "$ROS_SETUP" ]] || die "ROS 2 setup not found: $ROS_SETUP"
[[ -r "$DRIVER_SETUP" && -r "$PIPELINE_SETUP" ]] || die "workspace is not built; run ./build.sh first"
if [[ -z "$SDK_DIR" ]]; then
  SDK_DIR="$("$ROOT_DIR/find_x3_sdk.sh")" || die "CameraSDK not found"
fi

if ! SDK_DIR="$(realpath -e -- "$SDK_DIR")"; then
  die "SDK directory does not exist: $SDK_DIR"
fi

[[ -r "$SDK_DIR/include/camera/camera.h" ]] || die "missing SDK header: include/camera/camera.h"
[[ -r "$SDK_DIR/include/stream/stream_delegate.h" ]] || die "missing SDK header: include/stream/stream_delegate.h"
[[ -r "$SDK_DIR/lib/libCameraSDK.so" ]] || die "missing SDK library: lib/libCameraSDK.so"

# ROS setup files use optional variables without initializing them.
source "$ROS_SETUP"
source "$DRIVER_SETUP"
source "$PIPELINE_SETUP"
set -u
export INSTA360_SDK_DIR="$SDK_DIR"
export LD_LIBRARY_PATH="${SDK_DIR}/lib:${SDK_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

echo "[x3-cubemap] CameraSDK: $SDK_DIR"
echo "[x3-cubemap] Starting X3 ROS driver, ERP, and cubemap nodes"
exec ros2 launch da360_ros_pipeline x3_cubemap.launch.py "${LAUNCH_ARGS[@]}"
