#!/usr/bin/env bash
# Measure all four camera and detector topic rates on the matching ROS domain.
set -Eeuo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
mode="${1:-}"
seconds="${2:-15}"
[[ $# -le 2 && ( "$mode" == record || "$mode" == replay ) ]] || {
  echo "Usage: $0 {record|replay} [SECONDS]" >&2
  exit 2
}
[[ "$seconds" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
  echo "SECONDS must be a positive number" >&2
  exit 2
}

domain="${X3_FOUR_DOMAIN_ID:-}"
if [[ -z "$domain" ]]; then
  [[ "$mode" == record ]] && domain=0 || domain=71
fi
[[ "$domain" =~ ^[0-9]+$ ]] || { echo "Invalid ROS domain: $domain" >&2; exit 2; }
export ROS_DOMAIN_ID="$domain"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
echo "Measuring four X3 views on ROS domain $domain for $seconds seconds..."
exec python3 "$root/check_x3_rates.py" "$seconds"
