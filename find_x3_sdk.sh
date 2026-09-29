#!/usr/bin/env bash
# Print the external CameraSDK root. This file is called by setup and launch scripts.
set -Eeuo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

valid_sdk() {
  [[ -r "$1/include/camera/camera.h" &&
     -r "$1/include/stream/stream_delegate.h" &&
     -r "$1/lib/libCameraSDK.so" ]]
}

if [[ -n "${INSTA360_SDK_DIR:-}" ]]; then
  if valid_sdk "$INSTA360_SDK_DIR"; then
    realpath -e -- "$INSTA360_SDK_DIR"
    exit 0
  fi
  echo "INSTA360_SDK_DIR is set but is not a CameraSDK root: $INSTA360_SDK_DIR" >&2
  exit 1
fi

for candidate in "$root/../sdk" "$root/../sdk/"* "$root/../Linux_CameraSDK"* "$root/../Linux_CameraSDK"*/*; do
  if valid_sdk "$candidate"; then
    realpath -e -- "$candidate"
    exit 0
  fi
done

cat >&2 <<'EOF'
Insta360 CameraSDK was not found. Extract it outside this repository, for
example into ../sdk/CameraSDK-<version>/, or set INSTA360_SDK_DIR to the
directory containing include/camera/camera.h and lib/libCameraSDK.so.
The proprietary SDK and local recordings are not distributed in this repo.
EOF
exit 1
