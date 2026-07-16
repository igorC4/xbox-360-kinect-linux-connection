#!/usr/bin/env bash
#
# run_rtabmap.sh — run the RTAB-Map GUI built by build.sh, with Freenect
# (Kinect) support. libfreenect itself is resolved from the system install
# (/usr/local, see setup.sh); only rtabmap's own libs (built in-place, not
# installed) need LD_LIBRARY_PATH.
#
# Usage:
#   ./run_rtabmap.sh [rtabmap args...]
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN_DIR="$HERE/rtabmap/build/bin"
RTABMAP_BIN="$BIN_DIR/rtabmap"

if [ ! -x "$RTABMAP_BIN" ]; then
    echo "ERROR: $RTABMAP_BIN not found. Run ./build.sh first." >&2
    exit 1
fi

export LD_LIBRARY_PATH="$BIN_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export DISPLAY="${DISPLAY:-:1}"

exec "$RTABMAP_BIN" "$@"
