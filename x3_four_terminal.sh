#!/usr/bin/env bash
# One terminal in the four-face record or replay workflow.
set -Eeuo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
step="${1:-}"
shift || true
mode="${X3_FOUR_MODE:-replay}"
export ROS_DOMAIN_ID="${X3_FOUR_DOMAIN_ID:-71}"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "$root/install/share/insta360_ros_driver/local_setup.bash"
source "$root/install/share/da360_ros_pipeline/local_setup.bash"
set -u

case "$step" in
  record)
    [[ $# == 1 ]] || exit 2
    exec "$root/record_x3_bag.sh" "$1"
    ;;
  play)
    [[ $# == 1 || ( $# == 2 && "$2" == --unmirror ) ]] || exit 2
    [[ -f "$1/metadata.yaml" ]] || { echo "Bag not found: $1" >&2; exit 2; }
    if [[ $# == 2 ]]; then
      exec ros2 bag play "$1" --loop --clock --remap \
        /cubemap/front/image:=/cubemap/front/image/mirrored
    fi
    exec ros2 bag play "$1" --loop --clock
    ;;
  unmirror)
    [[ $# == 0 ]] || exit 2
    exec python3 "$root/unmirror_front.py" --ros-args \
      -p use_sim_time:=true
    ;;
  detect)
    [[ $# == 2 ]] || exit 2
    face="$1"
    spec="$2"
    case "$face" in front|right|back|left) ;; *) exit 2 ;; esac
    if [[ -f "$spec" ]]; then
      params=(--params-file "$spec")
    elif [[ "$spec" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
      params=(-p family:=36h11 -p size:="$spec")
    else
      echo "Use a positive tag size in metres or an existing YAML file: $spec" >&2
      exit 2
    fi
    sim_time=false
    [[ "$mode" == replay ]] && sim_time=true
    exec ros2 run apriltag_ros apriltag_node --ros-args \
      -r __node:="apriltag_${face}" \
      -r image_rect:="/cubemap/${face}/image" \
      -r camera_info:="/cubemap/${face}/camera_info" \
      -r detections:="/apriltag/${face}/detections" \
      -r /tf:="/apriltag/${face}/tf_raw" \
      "${params[@]}" \
      -p pose_estimation_method:=pnp -p qos_profile:=sensor_data \
      -p detector.decimate:=1.0 -p max_hamming:=1 \
      -p use_sim_time:="$sim_time"
    ;;
  overlay)
    [[ $# == 1 ]] || exit 2
    sim_time=false
    [[ "$mode" == replay ]] && sim_time=true
    exec python3 "$root/four_face_overlay.py" "$1" --ros-args \
      -p use_sim_time:="$sim_time"
    ;;
  tf)
    [[ $# == 0 ]] || exit 2
    exec python3 "$root/four_face_tf.py"
    ;;
  rviz)
    [[ $# == 0 ]] || exit 2
    sim_time=false
    [[ "$mode" == replay ]] && sim_time=true
    exec rviz2 -d "$root/config/x3_four_tags.rviz" --ros-args \
      -p use_sim_time:="$sim_time"
    ;;
  *)
    echo "Usage: $0 {record|play|unmirror|detect|overlay|tf|rviz} ..." >&2
    exit 2
    ;;
esac
