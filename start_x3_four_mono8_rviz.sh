#!/usr/bin/env bash
# Run the four-face workflow with live grayscale cubemap faces when recording.
set -Eeuo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ "${1:-}" == record ]]; then
  export X3_CUBEMAP_IMAGE_ENCODING=mono8
fi
exec "$root/start_x3_four_rviz.sh" "$@"
