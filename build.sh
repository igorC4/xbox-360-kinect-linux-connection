#!/usr/bin/env bash
#
# build.sh — configure + build RTAB-Map (rtabmap/) with Freenect (libfreenect)
# support, against the libfreenect installed by setup.sh (system-wide, under
# /usr/local). Idempotent: safe to re-run, only rebuilds what changed.
#
# Usage:
#   ./build.sh
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RTABMAP_DIR="$HERE/rtabmap"
BUILD_DIR="$RTABMAP_DIR/build"

echo "==> [1/3] checking libfreenect is installed (run ./setup.sh first if not)"
if [ ! -f /usr/local/include/libfreenect/libfreenect.h ] || [ ! -e /usr/local/lib/libfreenect.so ]; then
    echo "    ERROR: libfreenect not found under /usr/local. Run ./setup.sh first." >&2
    exit 1
fi

echo "==> [2/3] configuring rtabmap (cmake, WITH_FREENECT=ON)"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release -DWITH_FREENECT=ON | tee cmake_configure.log
if ! grep -q "With Freenect *= YES" cmake_configure.log; then
    echo "    ERROR: cmake did not pick up Freenect support, see cmake_configure.log" >&2
    exit 1
fi

echo "==> [3/3] building rtabmap ($(nproc) jobs)"
make -j"$(nproc)"

echo
echo "==> DONE. Binaries are in $BUILD_DIR/bin"
echo "   * Run:  ./run_rtabmap.sh"
