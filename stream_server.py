#!/usr/bin/env python3
"""
stream_server.py — live MJPEG stream of the Kinect (model 1473) + actuator controls.

In a browser at http://localhost:8080 you get:
  * RGB (left) + colorized depth (right), ~30 fps, MJPEG.
  * A vertical TILT slider (-31..31 deg) driving the motor.
  * LED buttons (off / blink / green / red).
  * A live accelerometer / tilt-angle readout.

Camera frames come from libfreenect (sync, camera-only, patched for 1473).
Motor/LED/accel go through the audio device via pyusb (see kinect_motor.py):
the firmware is uploaded on first use and the controller self-heals if the
audio device gets reset (e.g. when the camera stream re-opens).

Run with the project venv:
    unset PYTHONPATH && ./venv/bin/python stream_server.py
"""
import argparse
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np
import freenect
from PIL import Image

import usb.core
import usb.util
import kinect_motor as km

# ---- shared latest frame ----
_latest = {"jpeg": None, "ts": 0.0, "fps": 0.0}
_cond = threading.Condition()
_running = True

_CMAP = np.array([
    [48, 18, 59], [70, 134, 251], [27, 229, 181],
    [165, 254, 60], [251, 178, 45], [220, 51, 17], [122, 4, 3],
], dtype=np.float32)


def colorize_depth(depth):
    d = depth.astype(np.float32)
    valid = d < 2047
    out = np.zeros((*d.shape, 3), dtype=np.uint8)
    if valid.any():
        lo, hi = d[valid].min(), d[valid].max()
        if hi <= lo:
            hi = lo + 1
        norm = np.clip((d - lo) / (hi - lo), 0, 1)
        pos = norm * (len(_CMAP) - 1)
        i0 = np.floor(pos).astype(int)
        i1 = np.clip(i0 + 1, 0, len(_CMAP) - 1)
        frac = (pos - i0)[..., None]
        rgb = _CMAP[i0] * (1 - frac) + _CMAP[i1] * frac
        out = rgb.astype(np.uint8)
        out[~valid] = 0
    return out


# ===================== motor controller (thread-safe, self-healing) ============
class MotorController:
    def __init__(self):
        self.lock = threading.Lock()
        self.motor = None

    def _ensure(self):
        if self.motor is None:
            dev = km.ensure_running(verbose=False)
            self.motor = km.Motor(dev)

    def _reset(self):
        try:
            usb.util.dispose_resources(self.motor.dev)
        except Exception:
            pass
        self.motor = None

    def _do(self, fn):
        with self.lock:
            last = None
            for attempt in (1, 2):
                try:
                    self._ensure()
                    return fn(self.motor)
                except usb.core.USBError as e:
                    last = e
                    self._reset()
            raise last

    def tilt(self, deg):
        return self._do(lambda m: m.set_tilt(deg))

    def led(self, state):
        return self._do(lambda m: m.set_led(state))

    def accel(self):
        return self._do(lambda m: m.read_accel())


MOTOR = MotorController()


def capture_loop(warmup):
    global _running
    fails = 0
    for _ in range(warmup):
        try:
            freenect.sync_get_depth(0, freenect.DEPTH_11BIT)
            freenect.sync_get_video(0, freenect.VIDEO_RGB)
        except Exception:
            pass
    last = time.time()
    fps = 0.0
    while _running:
        try:
            d = freenect.sync_get_depth(0, freenect.DEPTH_11BIT)
            v = freenect.sync_get_video(0, freenect.VIDEO_RGB)
            pair = None if (d is None or v is None) else (v[0], d[0])
        except Exception:
            pair = None
        if pair is None:
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
        combined = np.hstack([rgb, colorize_depth(depth)])
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


PAGE = b"""<!doctype html><html><head><meta charset=utf-8><title>Kinect 1473</title>
<style>
 body{background:#0f1115;color:#dde;font-family:system-ui,sans-serif;margin:0;padding:14px}
 h2{font-weight:500;margin:0 0 10px}
 .wrap{display:flex;gap:18px;align-items:flex-start;flex-wrap:wrap}
 img{max-width:100%;height:auto;border:1px solid #2a2e38;border-radius:6px}
 .panel{display:flex;gap:24px;background:#171a21;border:1px solid #2a2e38;border-radius:8px;padding:16px}
 .col{display:flex;flex-direction:column;align-items:center;gap:8px}
 .tilt input[type=range]{writing-mode:vertical-lr;direction:rtl;width:32px;height:280px;accent-color:#4f9dfb}
 .lbl{color:#8a93a6;font-size:13px}
 .val{font-size:22px;font-variant-numeric:tabular-nums}
 button{background:#222733;color:#dde;border:1px solid #38404f;border-radius:6px;padding:7px 12px;cursor:pointer;font-size:14px}
 button:hover{background:#2c3340}
 .leds button.red{border-color:#c33}.leds button.green{border-color:#3a3}
 #accel{font-family:ui-monospace,monospace;font-size:13px;color:#9fb;white-space:pre;line-height:1.5}
</style></head><body>
<h2>Kinect 1473 &mdash; RGB | Depth + actuator controls</h2>
<div class=wrap>
  <img src="/stream" alt="stream">
  <div class=panel>
    <div class="col tilt">
      <div class=lbl>TILT</div>
      <input type=range min=-31 max=31 value=0 step=1 id=tilt
             oninput="tv.textContent=this.value+'\\u00b0'" onchange="setTilt(this.value)">
      <div class=val id=tv>0&deg;</div>
      <button onclick="document.getElementById('tilt').value=0;tv.textContent='0\\u00b0';setTilt(0)">Center</button>
    </div>
    <div class="col leds">
      <div class=lbl>LED</div>
      <button class=green onclick="setLed('green')">Green</button>
      <button onclick="setLed('blink')">Blink</button>
      <button class=red onclick="setLed('red')">Red</button>
      <button onclick="setLed('off')">Off</button>
    </div>
    <div class=col>
      <div class=lbl>ACCELEROMETER</div>
      <div id=accel>connecting...</div>
    </div>
  </div>
</div>
<script>
const tv=document.getElementById('tv');
function setTilt(v){fetch('/tilt?deg='+v,{method:'POST'}).catch(()=>{});}
function setLed(s){fetch('/led?state='+s,{method:'POST'}).catch(()=>{});}
async function pollAccel(){
  try{
    const r=await fetch('/accel'); const j=await r.json();
    if(j.error){document.getElementById('accel').textContent='(motor not ready)\\n'+j.error;}
    else{const a=j.accel_raw;
      document.getElementById('accel').textContent=
        `tilt : ${j.tilt_deg}\\u00b0\\nx: ${a[0]}\\ny: ${a[1]}\\nz: ${a[2]}`;}
  }catch(e){}
}
setInterval(pollAccel,1000); pollAccel();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        import json
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        try:
            if u.path == "/tilt":
                deg = int(q.get("deg", ["0"])[0])
                d = MOTOR.tilt(deg)
                return self._json({"ok": True, "tilt": d})
            if u.path == "/led":
                state = q.get("state", ["green"])[0]
                if state not in km.LED:
                    return self._json({"error": "bad state"}, 400)
                MOTOR.led(state)
                return self._json({"ok": True, "led": state})
        except Exception as e:
            return self._json({"error": str(e)}, 500)
        self.send_error(404)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/" or u.path.startswith("/index"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return
        if u.path == "/accel":
            try:
                return self._json(MOTOR.accel())
            except Exception as e:
                return self._json({"error": str(e)}, 500)
        if u.path == "/stream":
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
