#!/usr/bin/env python3
"""
stream_server.py — live MJPEG stream of the Kinect (model 1473) in a browser.

RGB (left) + colorized depth (right), side by side, served over HTTP.
Open http://localhost:8080 in any browser.

A single background thread pulls frames from libfreenect (sync camera-only API,
patched for the 1473) and encodes the latest combined frame as JPEG; all HTTP
clients share that latest frame, so multiple viewers are fine.

Run with the project venv:
    ./venv/bin/python stream_server.py            # default 0.0.0.0:8080
    ./venv/bin/python stream_server.py --port 9000
"""
import argparse
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import freenect
from PIL import Image

# ---- shared latest frame ----
_latest = {"jpeg": None, "ts": 0.0, "fps": 0.0}
_cond = threading.Condition()
_running = True

# Compact "turbo"-ish colormap control points (R,G,B), interpolated across 0..1.
_CMAP = np.array([
    [48, 18, 59], [70, 134, 251], [27, 229, 181],
    [165, 254, 60], [251, 178, 45], [220, 51, 17], [122, 4, 3],
], dtype=np.float32)


def colorize_depth(depth):
    """uint16 11-bit depth (0..2047, 2047=invalid) -> HxWx3 uint8 color image."""
    d = depth.astype(np.float32)
    valid = d < 2047
    out = np.zeros((*d.shape, 3), dtype=np.uint8)
    if valid.any():
        lo, hi = d[valid].min(), d[valid].max()
        if hi <= lo:
            hi = lo + 1
        norm = np.clip((d - lo) / (hi - lo), 0, 1)
        # interpolate through the colormap control points
        pos = norm * (len(_CMAP) - 1)
        i0 = np.floor(pos).astype(int)
        i1 = np.clip(i0 + 1, 0, len(_CMAP) - 1)
        frac = (pos - i0)[..., None]
        rgb = _CMAP[i0] * (1 - frac) + _CMAP[i1] * frac
        out = rgb.astype(np.uint8)
        out[~valid] = 0  # invalid -> black
    return out


def _get_pair():
    """Return (rgb, depth) or None. freenect.sync_get_* return None when the
    device is dead/absent; treat that as a failure to trigger recovery."""
    d = freenect.sync_get_depth(0, freenect.DEPTH_11BIT)
    v = freenect.sync_get_video(0, freenect.VIDEO_RGB)
    if d is None or v is None:
        return None
    return v[0], d[0]


def capture_loop(warmup):
    global _running
    fails = 0
    for _ in range(warmup):
        try:
            _get_pair()
        except Exception:
            pass
    last = time.time()
    fps = 0.0
    while _running:
        try:
            pair = _get_pair()
        except Exception:
            pair = None

        if pair is None:
            # Device dead / disappeared. Reset the sync engine so the next
            # get_* call re-opens the (possibly re-enumerated) device, and
            # back off so we don't busy-spin while it's unplugged.
            fails += 1
            try:
                freenect.sync_stop()
            except Exception:
                pass
            if fails == 1 or fails % 20 == 0:
                print(f"[stream] device unavailable (x{fails}), retrying...", flush=True)
            with _cond:
                _latest["fps"] = 0.0
            time.sleep(0.5)
            continue

        if fails:
            print(f"[stream] device recovered after {fails} retries", flush=True)
            fails = 0
        rgb, depth = pair
        combined = np.hstack([rgb, colorize_depth(depth)])  # 480 x 1280 x 3
        buf = io.BytesIO()
        Image.fromarray(combined, "RGB").save(buf, format="JPEG", quality=80)
        now = time.time()
        dt = now - last
        last = now
        if dt > 0:
            fps = 0.9 * fps + 0.1 * (1.0 / dt)
        with _cond:
            _latest["jpeg"] = buf.getvalue()
            _latest["ts"] = now
            _latest["fps"] = fps
            _cond.notify_all()
    freenect.sync_stop()


PAGE = b"""<!doctype html><html><head><title>Kinect 1473 live</title>
<style>body{background:#111;color:#ddd;font-family:sans-serif;text-align:center;margin:0;padding:12px}
img{max-width:100%;height:auto;border:1px solid #333}h2{font-weight:400}</style></head>
<body><h2>Kinect 1473 &mdash; RGB (left) | Depth (right)</h2>
<img src="/stream"><p style="color:#777">live MJPEG &bull; refresh if it stalls</p></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # quiet

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_ts = 0.0
            try:
                while _running:
                    with _cond:
                        _cond.wait(timeout=5)
                        if _latest["jpeg"] is None or _latest["ts"] == last_ts:
                            continue
                        frame = _latest["jpeg"]
                        last_ts = _latest["ts"]
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                return
            return
        self.send_error(404)


def main():
    global _running
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--warmup", type=int, default=10)
    args = ap.parse_args()

    t = threading.Thread(target=capture_loop, args=(args.warmup,), daemon=True)
    t.start()

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[stream] serving on http://localhost:{args.port}  (Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        _running = False
        with _cond:
            _cond.notify_all()
        srv.shutdown()
        print("\n[stream] stopped")


if __name__ == "__main__":
    main()
