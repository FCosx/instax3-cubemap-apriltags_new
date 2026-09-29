#!/usr/bin/env bash
set -Eeuo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
bag_path="${1:-}"
if [[ -z "$bag_path" ]]; then
  echo "Usage: $0 /absolute/path/to/new_bag [X3 launch arguments...]" >&2
  exit 2
fi
shift
[[ "$bag_path" = /* ]] || { echo "Bag path must be absolute" >&2; exit 2; }
[[ ! -e "$bag_path" ]] || { echo "Bag path already exists: $bag_path" >&2; exit 2; }
mkdir -p -- "$(dirname -- "$bag_path")"

set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "$root/install/share/insta360_ros_driver/local_setup.bash"
source "$root/install/share/da360_ros_pipeline/local_setup.bash"
set -u

camera_log="${bag_path}.camera.log"
# Give the launch and its child nodes their own process group so a failed
# recording cannot leave the camera driver holding the USB interface.
setsid "$root/start_x3_cubemap.sh" "$@" >"$camera_log" 2>&1 &
camera_pid=$!
cleanup() {
  kill -TERM -- "-$camera_pid" 2>/dev/null || true
  for ((attempt=0; attempt<30; attempt++)); do
    kill -0 -- "-$camera_pid" 2>/dev/null || break
    sleep 0.2
  done
  kill -KILL -- "-$camera_pid" 2>/dev/null || true
  wait "$camera_pid" 2>/dev/null || true
}
trap cleanup EXIT

# A typed subscriber avoids a startup race in the ros2 topic echo CLI where
# the topic name is discovered before its message type.
if ! python3 "$root/wait_for_front_frame.py" 30; then
  echo "No front camera frame arrived. See $camera_log" >&2
  exit 1
fi

echo "Recording SQLite ROS 2 bag (.db3) to $bag_path. Press Ctrl+C to finish."
ros2 bag record -s sqlite3 -o "$bag_path" --topics \
  /dual_fisheye/image/compressed \
  /equirectangular/image/compressed \
  /cubemap/front/image /cubemap/front/camera_info \
  /cubemap/right/image /cubemap/right/camera_info \
  /cubemap/back/image /cubemap/back/camera_info \
  /cubemap/left/image /cubemap/left/camera_info \
  /cubemap/horizontal/image/compressed \
  /imu/data_raw
