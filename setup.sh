#!/usr/bin/env bash
#
# setup.sh — reproduce the full Kinect-1473 streaming setup on a fresh Ubuntu box.
#
# Idempotent-ish: safe to re-run. Tested on Ubuntu 22.04 (x86_64), libfreenect
# master @ 09a1f09. Read README.md for the *why* behind each step.
#
# Usage:
#   ./setup.sh
# then log out / back in once (for the plugdev/video group change), and:
#   unset PYTHONPATH && ./venv/bin/python stream_server.py   # http://localhost:8080
#
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# Pin the libfreenect commit we validated. Set to "" to track master instead.
FREENECT_COMMIT=""

echo "==> [1/8] system build dependencies (sudo)"
sudo apt-get update
sudo apt-get install -y git cmake build-essential pkg-config \
    libusb-1.0-0-dev freeglut3-dev libxmu-dev libxi-dev python3-venv

echo "==> [2/8] clone libfreenect"
if [ -d libfreenect ] && git -C libfreenect rev-parse --git-dir >/dev/null 2>&1; then
    echo "    libfreenect already present (submodule or clone), skipping clone"
else
    git clone https://github.com/OpenKinect/libfreenect.git
fi
( cd libfreenect
  git fetch --all
  if [ -n "$FREENECT_COMMIT" ]; then git checkout "$FREENECT_COMMIT"; fi )

echo "==> [3/8] apply the model-1473 camera-only sync patch"
# The c_sync wrapper claims MOTOR|CAMERA; on the 1473 the motor claim fails
# fatally (LIBUSB_ERROR_IO) because the motor lives behind audio firmware.
# We select CAMERA only. Apply only if not already applied.
if ! grep -q "model 1473 patch" libfreenect/wrappers/c_sync/libfreenect_sync.c; then
    git -C libfreenect apply "$HERE/patches/0001-1473-sync-camera-only.patch"
    echo "    patch applied"
else
    echo "    patch already present, skipping"
fi

echo "==> [4/8] udev rules (MODE=0666 -> no root needed) + user groups (sudo)"
sudo cp libfreenect/platform/linux/udev/51-kinect.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG plugdev,video "$USER" || true

echo "==> [5/8] python venv with numpy<2 (avoids the numpy-2 ABI clash with the cython binding)"
python3 -m venv venv
unset PYTHONPATH                         # keep ROS/system numpy off the path
./venv/bin/pip -q install --upgrade pip
./venv/bin/pip -q install "numpy<2" "cython>=3" setuptools pillow pyusb
VENV="$HERE/venv"

echo "==> [6/8] build + install the C libraries (sudo make install)"
( cd libfreenect
  rm -rf build && mkdir build && cd build
  cmake .. -DBUILD_EXAMPLES=ON -DBUILD_PYTHON3=OFF -DBUILD_CV=OFF
  make -j"$(nproc)"
  sudo make install
  sudo ldconfig )

echo "==> [7/8] build the python binding against the venv (numpy<2 + venv cython)"
# Build in a separate dir using the VENV python+cython so build-time and runtime
# numpy match. The resulting binding has an RPATH into build-venv/lib, so the
# patched sync lib must live there too — which it does, since we build it here.
( cd libfreenect
  rm -rf build-venv && mkdir build-venv && cd build-venv
  PATH="$VENV/bin:$PATH" cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON3=ON -DBUILD_CV=OFF \
      -DPython3_EXECUTABLE="$VENV/bin/python"
  PATH="$VENV/bin:$PATH" make -j"$(nproc)"
  SO="$(find . -name 'freenect*.so' | head -1)"
  SITE_PKG="$("$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
  cp "$SO" "$SITE_PKG/freenect.so" )

echo "==> [8/8] fetch the motor/audio firmware (audios.bin) for tilt/LED control"
# Extracted from an official Microsoft Xbox 360 system update (~116 MB download).
# Not redistributed in this repo. Skips if already present.
if [ ! -f firmware/audios.bin ]; then
    mkdir -p firmware
    ( cd firmware && python3 "$HERE/libfreenect/src/fwfetcher.py" audios.bin ) \
        || echo "    (firmware fetch failed — motor control will be unavailable until "\
                "you run libfreenect/src/fwfetcher.py; camera streaming is unaffected)"
fi

echo
echo "==> DONE."
echo "   * Log out and back in once (plugdev/video group change)."
echo "   * Connect the Kinect WITH its 12V power (see README hardware section)."
echo "   * Verify:  unset PYTHONPATH && ./venv/bin/python -c 'import freenect; print(\"ok\")'"
echo "   * Stream:  unset PYTHONPATH && ./venv/bin/python stream_server.py  -> http://localhost:8080"
echo "   * Motor :  unset PYTHONPATH && ./venv/bin/python kinect_motor.py --sweep"
