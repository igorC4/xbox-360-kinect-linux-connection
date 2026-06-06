# Kinect 1473 — Worklog

## 2026-06-06 15:30
- Goal: get an Xbox 360 Kinect (model **1473**) streaming on Ubuntu 22.04 laptop.
- **Initial state:** only `045e:02c2` (internal hub/"motor") enumerated; camera
  `02ae` + audio `02ad` absent. Diagnosed as the 12 V power-rail requirement
  (hub runs on 5 V USB; camera/IR/audio need separate 12 V). Confirmed via sysfs
  (hub class 09, maxchild 2, empty downstream ports) + `kinect_usb_watch.py`
  (ms-resolution USB watcher built for this).
- User had power **soldered directly**; first replug showed hub flapping, still no
  camera. After **resolder**, all three devices enumerated: motor `02c2`,
  audio `02ad`, camera `02ae` (1-3.2). 12 V was the whole problem.
- Built **libfreenect from source** (master, 1473 support) + udev rules (MODE 0666).
- Python bindings: fought a numpy 2.x-vs-1.x ABI clash (ROS Humble + system +
  .local numpys). Resolved with a dedicated **venv built against numpy<2** and the
  venv's Cython 3.2.5.
- 1473 quirks handled: LED-set `LIBUSB_ERROR_IO` is benign; patched c_sync wrapper
  to select **CAMERA only** (motor claim is fatal on 1473). Rebuilt sync lib in
  both `build/` and `build-venv/` (RPATH).
- **Result: streaming works.** `capture_frames.py` saves RGB+depth; `stream_server.py`
  serves live MJPEG (RGB | colorized depth) at http://localhost:8080. Verified real
  frames (1280x480). Depth sparse only because scene is inside the ~50 cm dead zone.
- TODO/optional: mic array (needs audios.bin firmware upload); metric depth via
  DEPTH_MM/REGISTERED; v4l2loopback if a /dev/video device is ever wanted.

## 2026-06-06 15:50
- Transient: Kinect dropped off USB bus mid-stream (iso packet loss / 12 V margin);
  non-resilient server died. Made stream_server.py self-healing (reset sync +
  reopen on dead device) and ran it under a while-true restart supervisor. Back at
  ~30 fps; verified full RGBD frames (two people, clean depth map).
- Wrote setup.sh (one-shot reproducible install), extracted the 1473 patch to
  patches/0001-1473-sync-camera-only.patch, wrote detailed README (hardware +
  Linux replication), added docs/sample_rgbd.jpg.
- git init + initial commit 9791097 (libfreenect/ and venv/ gitignored).
