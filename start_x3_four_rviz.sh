#!/usr/bin/env bash
# Open one terminal for recording/replay, four detectors, overlays, TF, RViz.
set -Eeuo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
mode="${1:-}"
bag_path="${2:-}"
tag_spec="${3:-0.06}"
option="${4:-}"

usage() {
  cat >&2 <<'EOF'
Usage:
  start_x3_four_rviz.sh record /absolute/path/to/new_bag [TAG_SIZE_METRES|TAG_CONFIG.yaml]
  start_x3_four_rviz.sh replay /absolute/path/to/bag [TAG_SIZE_METRES|TAG_CONFIG.yaml] [--unmirror]
EOF
  exit 2
}

[[ "$mode" == record || "$mode" == replay ]] || usage
[[ -r "$root/install/share/insta360_ros_driver/local_setup.bash" &&
   -r "$root/install/share/da360_ros_pipeline/local_setup.bash" ]] || {
  echo "Project is not built. Run $root/setup_x3.sh first." >&2
  exit 1
}
[[ "$bag_path" = /* ]] || usage
[[ "$tag_spec" =~ ^[0-9]+([.][0-9]+)?$ || -f "$tag_spec" ]] || usage
if [[ -f "$tag_spec" ]]; then
  tag_spec="$(realpath -- "$tag_spec")"
fi
[[ -z "$option" || ( "$mode" == replay && "$option" == --unmirror ) ]] || usage
[[ $# -le 4 ]] || usage
if [[ "$mode" == record ]]; then
  [[ ! -e "$bag_path" ]] || { echo "Bag already exists: $bag_path" >&2; exit 2; }
else
  [[ -f "$bag_path/metadata.yaml" ]] || { echo "Bag not found: $bag_path" >&2; exit 2; }
fi

domain="${X3_FOUR_DOMAIN_ID:-}"
if [[ -z "$domain" ]]; then
  [[ "$mode" == record ]] && domain=0 || domain=71
fi
[[ "$domain" =~ ^[0-9]+$ ]] || { echo "Invalid ROS domain: $domain" >&2; exit 2; }
command -v gnome-terminal >/dev/null || { echo "gnome-terminal is required" >&2; exit 1; }
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || { echo "No graphical display" >&2; exit 1; }

exec 9>"/tmp/x3_four_rviz_${domain}.lock"
flock -n 9 || { echo "A four-face workflow is already using domain $domain" >&2; exit 1; }

terminal_pids=()
open_terminal() {
  local title="$1"
  shift
  gnome-terminal --wait --window --title="$title" -- \
    env X3_FOUR_DOMAIN_ID="$domain" X3_FOUR_MODE="$mode" \
    "$root/x3_four_terminal.sh" "$@" &
  terminal_pids+=("$!")
  sleep 0.4
}

echo "Four-face $mode on ROS domain $domain"
if [[ "$mode" == record ]]; then
  open_terminal "X3 recording" record "$bag_path"
else
  if [[ "$option" == --unmirror ]]; then
    open_terminal "X3 bag loop" play "$bag_path" --unmirror
    open_terminal "X3 front mirror correction" unmirror
  else
    open_terminal "X3 bag loop" play "$bag_path"
  fi
fi
for face in front right back left; do
  open_terminal "X3 AprilTag $face" detect "$face" "$tag_spec"
done
open_terminal "X3 four annotated views" overlay "${bag_path}_four"
open_terminal "X3 four-face TF" tf
open_terminal "X3 four-face RViz" rviz

echo "Four-face views and TF are running. Close the terminal windows to stop."
for terminal_pid in "${terminal_pids[@]}"; do
  wait "$terminal_pid" || true
done
