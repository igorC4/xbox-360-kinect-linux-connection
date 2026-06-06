#!/usr/bin/env python3
"""
capture_frames.py — headless depth+RGB capture from an Xbox 360 Kinect (model 1473)
via libfreenect's synchronous API. No display required.

Saves into ./captures/:
    rgb_NN.png        8-bit color
    depth_NN.png      16-bit depth (raw 11-bit values, 0..2047; 2047 = no reading)
    depth_vis_NN.png  8-bit normalized depth for quick eyeballing
    depth_NN.npy      raw numpy uint16 depth (for real processing)
And prints per-frame stats so it's useful even with no image viewer.

Run with the project venv:
    ./venv/bin/python capture_frames.py --frames 5
"""
import argparse
import os
import time

import numpy as np
import freenect
from PIL import Image

OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")


def grab_depth():
    # sync_get_depth -> (HxW uint16 array, timestamp). FORMAT_11BIT: 0..2047
    depth, ts = freenect.sync_get_depth(0, freenect.DEPTH_11BIT)
    return depth, ts


def grab_video():
    # sync_get_video -> (HxWx3 uint8 RGB, timestamp)
    rgb, ts = freenect.sync_get_video(0, freenect.VIDEO_RGB)
    return rgb, ts


def depth_to_vis(depth):
    d = depth.astype(np.float32)
    valid = d < 2047
    if valid.any():
        lo, hi = d[valid].min(), d[valid].max()
        norm = np.zeros_like(d)
        if hi > lo:
            norm[valid] = (d[valid] - lo) / (hi - lo) * 255.0
        return norm.astype(np.uint8)
    return np.zeros_like(d, dtype=np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=5, help="number of frame pairs to grab")
    ap.add_argument("--warmup", type=int, default=10, help="frames to discard while sensor settles")
    ap.add_argument("--delay", type=float, default=0.2, help="seconds between saved frames")
    args = ap.parse_args()

    os.makedirs(OUTDIR, exist_ok=True)
    print(f"[capture] warming up ({args.warmup} frames)...")
    for _ in range(args.warmup):
        grab_depth()
        grab_video()

    for i in range(args.frames):
        depth, dts = grab_depth()
        rgb, vts = grab_video()

        valid = depth < 2047
        nvalid = int(valid.sum())
        pct = 100.0 * nvalid / depth.size
        dmin = int(depth[valid].min()) if nvalid else -1
        dmax = int(depth[valid].max()) if nvalid else -1
        dmean = float(depth[valid].mean()) if nvalid else -1

        Image.fromarray(rgb, "RGB").save(os.path.join(OUTDIR, f"rgb_{i:02d}.png"))
        Image.fromarray(depth.astype(np.uint16)).save(os.path.join(OUTDIR, f"depth_{i:02d}.png"))
        Image.fromarray(depth_to_vis(depth)).save(os.path.join(OUTDIR, f"depth_vis_{i:02d}.png"))
        np.save(os.path.join(OUTDIR, f"depth_{i:02d}.npy"), depth.astype(np.uint16))

        print(f"[frame {i:02d}] rgb={rgb.shape} depth={depth.shape} "
              f"valid={pct:5.1f}%  depth(raw 11-bit) min/mean/max={dmin}/{dmean:.0f}/{dmax}")
        time.sleep(args.delay)

    freenect.sync_stop()
    print(f"[capture] done -> {OUTDIR}")


if __name__ == "__main__":
    main()
