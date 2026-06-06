# Streaming an Xbox 360 Kinect (model 1473) on Linux

Live **RGB + depth** streaming **and full actuator control** (tilt motor, status
LED, accelerometer) of an Xbox 360 **Kinect model 1473** on Ubuntu 22.04, via
[libfreenect](https://github.com/OpenKinect/libfreenect) for the camera and
direct USB (pyusb) for the motor. Includes a browser UI (live MJPEG + a vertical
tilt slider + LED buttons + accel readout), a headless frame grabber, a
millisecond USB enumeration watcher used to debug the hardware, and a one-shot
setup script.

Output is RGB on the left and colorized depth on the right (blue = near, red =
far) at ~30 fps. Run `stream_server.py` and open the browser to see it live and
drive the motor; or use `kinect_motor.py` from the CLI.

---

## TL;DR — replicate on another PC

```bash
git clone <this-repo> kinect-connection-linux && cd kinect-connection-linux
./setup.sh                 # installs deps, builds libfreenect (patched), makes a venv
# log out / back in once (group change), connect the Kinect WITH 12 V power, then:
unset PYTHONPATH
./venv/bin/python stream_server.py     # open http://localhost:8080
```

If only `045e:02c2` shows up in `lsusb` and the camera never appears, **it's a
power problem, not software** — read the hardware section.

---

## 1. How the Kinect 360 works (hardware side)

### 1.1 What's inside model 1473
The Kinect is not one USB device — it's an **internal 2-port USB 2.0 hub** with
several functions hanging off it. On the bus you should eventually see **three**
Microsoft (`045e`) devices:

| USB ID      | Function           | Notes                                              |
|-------------|--------------------|----------------------------------------------------|
| `045e:02c2` | NUI Motor **/ hub**| Class 09 — this is actually the internal USB hub.  |
| `045e:02ad` | NUI Audio          | Mic array. Reports `© 2011 Microsoft`. Re-enumerates periodically. |
| `045e:02ae` | NUI Camera         | RGB + IR/depth. This is what we stream.            |

Model **1473** is a later revision of the Xbox 360 Kinect that moved toward the
"Kinect for Windows" protocol. The practical consequence for us: **the tilt
motor, status LED, and accelerometer are controlled through the audio device**,
not a standalone motor endpoint — and the audio device must have its firmware
(`audios.bin`) uploaded first. We do exactly that (see §2.7). (Depth + RGB need
**no** firmware upload; the motor does.)

> Model 1414 is the *original* 360 Kinect and behaves slightly differently
> (real motor subdevice, no audio-firmware dependency). The 1473 is the one this
> repo targets.

### 1.2 The power requirement — THE thing that bites everyone
The Kinect's proprietary connector carries **two power domains**:

- **5 V** from the USB data line — powers **only the internal hub + motor logic**.
- **A separate 12 V (~1.08 A) rail** — powers the **camera, IR laser projector,
  depth sensor, and microphone array**.

A normal PC USB port supplies only the 5 V line. The Xbox 360 console supplied
the 12 V through its special port. So on a PC **you must provide 12 V yourself**.

**Symptom when 12 V is missing/marginal:** `lsusb` shows *only* `045e:02c2`
(the hub, drawing ~2 mA). The camera (`02ae`) and audio (`02ad`) **never
enumerate** — the hub's two downstream ports stay empty. No driver can fix this;
the sensors are physically unpowered.

You get 12 V one of two ways:
1. **The official "Kinect 360 USB AC adapter"** — a Y-cable: the proprietary
   plug splits into a standard USB-A (data + 5 V) and a 12 V barrel from a wall
   wart. Plug-and-play, no soldering.
2. **Solder the 12 V directly** (what this machine uses). Cut into the
   proprietary cable: the four USB lines (V+5, D−, D+, GND) go to a USB-A plug,
   and the **separate 12 V pair** goes to a 12 V / ≥1.5 A supply, correct
   polarity. **Verify with a multimeter that 12 V holds steady under load** — a
   cold joint or a supply that sags when the IR projector fires produces a feed
   that streams briefly then drops off the bus (see Troubleshooting).

### 1.3 Verifying the hardware
```bash
lsusb | grep 045e          # want THREE devices: 02c2, 02ad, 02ae
lsusb -t                   # the 02c2 hub should have children on its 2 ports
ls /sys/bus/usb/devices/1-3.*   # 1-3.1 + 1-3.2 = audio + camera enumerated
```
If you only ever see `02c2`, fix power before touching software. To watch
enumeration live (catches devices that appear for only milliseconds), use the
included watcher:
```bash
python3 kinect_usb_watch.py        # millisecond-stamped APPEAR/GONE/CHANGE log
```

---

## 2. How we make it work (Linux side)

### 2.1 libfreenect
We build [libfreenect](https://github.com/OpenKinect/libfreenect) from source
(master, commit `09a1f09`) rather than the apt package, because master has the
model-1473/K4W detection that disables the unusable motor subdevice and opens
the camera cleanly. Standard CMake build → installs to `/usr/local`.

### 2.2 udev rules — no root needed
`platform/linux/udev/51-kinect.rules` sets `MODE="0666"` on all the `045e`
Kinect IDs, so any user can open the device without `sudo`. We also add the user
to the `plugdev` and `video` groups (takes effect after re-login).

### 2.3 Two model-1473 quirks you WILL hit
1. **`Failed to set the LED of K4W or 1473 device: LIBUSB_ERROR_IO`** prints on
   every open. **Harmless** — the 1473's LED/motor is behind the audio firmware,
   which we don't load. Streaming works regardless. (Even `freenect-camtest`
   prints this and streams fine.)

2. **The synchronous wrapper fatally fails to open the device.** libfreenect's
   `c_sync` wrapper (`freenect_sync_get_depth/_video`, which the Python binding
   uses) claims `FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA`. On the 1473 the
   **motor claim fails fatally** → `Could not open device: LIBUSB_ERROR_IO` →
   Python sees `None`. Fix: select **CAMERA only**.
   See `patches/0001-1473-sync-camera-only.patch`:
   ```c
   // wrappers/c_sync/libfreenect_sync.c, init_thread()
   - freenect_select_subdevices(ctx, MOTOR | CAMERA);
   + freenect_select_subdevices(ctx, CAMERA);   // 1473: motor claim is fatal
   ```
   `freenect-camtest` works out of the box because it already selects
   `FREENECT_DEVICE_CAMERA` only — that's the tell that pointed us here.

### 2.4 The numpy ABI trap (Python binding)
The Cython binding (`wrappers/python/freenect.pyx`) `cimport`s numpy, so its
compiled `.so` is tied to a numpy ABI. This machine has **three** numpys (ROS
Humble, system `/usr/lib`, and `~/.local`), and numpy 2.0 changed the
`PyArray_Descr` struct size (96 → 88 bytes). Mixing build-time and run-time
numpy versions yields:
```
ValueError: numpy.dtype size changed ... Expected 96 from C header, got 88 from PyObject
```
**Fix:** a dedicated **venv built against `numpy<2`** (a module built on numpy 1.x
runs against both 1.x and 2.x), using the **venv's own Cython**. Always
`unset PYTHONPATH` before using it so the ROS/system numpys stay off `sys.path`.

### 2.5 The RPATH gotcha (why we build twice)
The venv binding is linked with an **RPATH pointing into its own build tree**
(`build-venv/lib/libfreenect_sync.so.0`), *not* `/usr/local/lib`. So patching and
reinstalling the system lib is not enough — the patched `libfreenect_sync` must
be compiled **in `build-venv/` too**. `setup.sh` applies the patch to the source
*before* either build, so both the `build/` (C tools) and `build-venv/` (Python)
copies are patched automatically.

### 2.6 Data formats
- **Depth:** default here is `DEPTH_11BIT` → `uint16` 0..2047, where **2047 =
  no/invalid reading**. Range ≈ **0.5 m (dead zone) to ~4 m**; closer than ~50 cm
  reads invalid. For **metric** output use `freenect.DEPTH_MM` or
  `DEPTH_REGISTERED` (depth aligned to the RGB camera).
- **RGB:** `VIDEO_RGB` → `uint8` 640×480×3.

### 2.7 Driving the tilt motor / LED / accelerometer (model-1473)
This was the hard part, and `kinect_motor.py` implements it from scratch over
**pyusb**, deliberately bypassing libfreenect. Why and how:

**The motor lives behind the audio device.** On the 1473 there is no usable motor
endpoint; `src/tilt.c` sends bulk commands to the **audio device** (`045e:02ad`)
on endpoints `0x01` (OUT) / `0x81` (IN). But the audio device boots in a
**bootloader state** (1 USB interface) that only understands a *firmware-upload*
protocol. Until `audios.bin` is uploaded, motor/LED/accel commands fail with
`LIBUSB_ERROR_IO`. After upload + launch it re-enumerates in **running state**
(≥2 interfaces) and accepts motor commands.

**Why not let libfreenect do it?** libfreenect's open path calls
`fnusb_keep_alive_led()` which `libusb_open`s the audio device and immediately
`libusb_reset_device()`s it; on this machine that wedges the audio device with
`LIBUSB_ERROR_IO`, so its built-in upload never completes. pyusb opens the same
device cleanly, so we drive it directly.

**Getting the firmware (`audios.bin`).** It's extracted from an official
Microsoft Xbox 360 system update by libfreenect's fetcher (not redistributed in
this repo — `firmware/` is git-ignored):
```bash
unset PYTHONPATH
mkdir -p firmware && cd firmware
python3 ../libfreenect/src/fwfetcher.py audios.bin   # downloads ~116 MB from xbox.com
cd ..
cp firmware/audios.bin ~/.libfreenect/               # optional: a search location
```
`kinect_motor.py` looks for it in `$KINECT_FW`, `./firmware/audios.bin`,
`./audios.bin`, then `~/.libfreenect/audios.bin`.

**The protocols we reimplement** (all little-endian; magic `0x06022009`, replies
carry magic `0x0a6fe000`):
- *Firmware upload* (`src/loader.c`): per 0x4000-byte page send a
  `bootloader_command{tag, bytes, cmd=0x03, addr}` then the page in ≤512-byte
  bulk writes, read a 12-byte ack; finish with `cmd=0x04` at the entry address to
  launch. The device then re-enumerates from 1→2 interfaces.
- *Motor / LED / accel* (`src/tilt.c`): `cmd=0x803b` tilt (arg2 = degrees,
  −31..31), `cmd=0x10` LED (arg2: 1=off, 2=blink-green, 3=green, 4=red),
  `cmd=0x8032` reads a 104-byte reply containing accelerometer x/y/z and tilt.

`kinect_motor.py` auto-detects bootloader vs running state, flashes if needed,
waits for re-enumeration, then issues commands. `stream_server.py` wraps it in a
thread-safe, self-healing controller (the camera-open path resets the audio
device, so the controller re-flashes on demand if a motor command hits
`LIBUSB_ERROR_IO`). **Firmware lives in volatile RAM**, so a power-cycle (or a
USB reset) reverts the audio device to bootloader — it's simply re-flashed on the
next motor command.

---

## 3. Usage

Always: `cd` into the repo and `unset PYTHONPATH` first (keeps ROS/system numpy
off the path), and use the venv's python.

```bash
unset PYTHONPATH

# Live browser stream + actuator controls (RGB | depth, tilt slider, LED, accel):
./venv/bin/python stream_server.py            # http://localhost:8080
./venv/bin/python stream_server.py --port 9000

# CLI motor control (auto-uploads firmware on first use):
./venv/bin/python kinect_motor.py --sweep     # LED walk + tilt sweep demo
./venv/bin/python kinect_motor.py --tilt 20   # tilt to +20 deg (-31..31)
./venv/bin/python kinect_motor.py --led red   # off | blink | green | red
./venv/bin/python kinect_motor.py --accel     # read accelerometer + tilt angle
./venv/bin/python kinect_motor.py --upload-only   # just flash audios.bin

# Headless still capture -> ./captures/ (rgb png, 16-bit depth png, depth vis, raw npy):
./venv/bin/python capture_frames.py --frames 5

# Pure-C sanity check (no Python/numpy), prints frame + packet stats:
freenect-camtest
freenect-glview            # GUI viewer, needs a display (DISPLAY=:0)
```

### Browser controls
At `http://localhost:8080` the page shows the live RGB|depth stream plus a
control panel: a **vertical tilt slider** (−31..31°, applied on release), **LED**
buttons (Green / Blink / Red / Off), and a live **accelerometer / tilt** readout
(polled once a second). The first tilt/LED action uploads the audio firmware
(~a few seconds) and may briefly blip the camera stream.

HTTP API (handy for scripting): `POST /tilt?deg=N`, `POST /led?state=red`,
`GET /accel` (JSON).

### Run the stream as a resilient background service
The Kinect can transiently drop off the bus. To keep the stream up no matter
what, run it under a restart supervisor (this is how it's running on this box):
```bash
unset PYTHONPATH
nohup bash -c 'while true; do ./venv/bin/python stream_server.py; echo restart; sleep 2; done' \
    >/tmp/kinect_stream.log 2>&1 &
```
`stream_server.py` *also* self-heals at the application level: if the device
disappears it resets the libfreenect sync engine and reopens automatically.

---

## 4. Files in this repo

| Path | What it is |
|------|------------|
| `setup.sh` | One-shot reproducible install (deps → patch → build → venv). |
| `stream_server.py` | Live MJPEG server + actuator controls (tilt/LED/accel), self-healing. |
| `kinect_motor.py` | Tilt motor / LED / accelerometer driver over pyusb (incl. firmware upload). |
| `capture_frames.py` | Headless still capture to `captures/` (png + raw npy). |
| `kinect_usb_watch.py` | Millisecond USB enumeration watcher (hardware debugging). |
| `patches/0001-1473-sync-camera-only.patch` | The camera-only sync patch. |
| `WORKLOG.md` | Chronological log of how this was brought up. |
| `libfreenect/`, `venv/`, `firmware/` | Build tree, python env, firmware — **git-ignored**, recreated by `setup.sh` / `fwfetcher.py`. |

---

## 5. Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| Only `045e:02c2` in `lsusb`, no camera/audio | **No/weak 12 V.** Check the adapter/solder; measure 12 V under load. |
| Camera appears then `USB device disappeared` / `camera marked dead` mid-stream | USB isochronous bandwidth or **12 V sag** under IR load. Move Kinect to its own USB controller/port (not shared with the webcam); verify the solder joint. The server auto-restarts. |
| `Invalid magic ffff` / `Lost N packets` spam | Normal-ish iso packet loss; libfreenect resyncs. Heavy loss → bandwidth issue, as above. Streaming **only** depth or RGB halves the load. |
| `Could not open device: LIBUSB_ERROR_IO` from Python | The 1473 motor-claim issue — apply the camera-only patch and rebuild **build-venv** (see §2.3/§2.5). |
| `numpy.dtype size changed ... Expected 96 got 88` | numpy ABI mismatch — use the venv (`numpy<2`) and `unset PYTHONPATH` (see §2.4). |
| `Failed to set the LED of K4W or 1473 device` | Harmless, ignore (see §2.3). |
| Audio device's USB devnum keeps changing | The 1473 audio re-enumerates periodically (bootloader state); normal — it's flashed on motor use. |
| Motor commands do nothing / `LIBUSB_ERROR_IO` from `kinect_motor.py` | Audio device in bootloader state with no firmware. Ensure `audios.bin` is findable (see §2.7); the script re-flashes automatically. Stop `stream_server.py` if running standalone CLI. |
| `audios.bin not found` | Run `libfreenect/src/fwfetcher.py` to fetch it (see §2.7). |
| Motor worked, then stopped after replug/power-cycle | Firmware is in volatile RAM; it re-flashes on the next motor command. |
| `ModuleNotFoundError: freenect` | Use the venv python and `unset PYTHONPATH`; binding lives in `venv/lib/python3.10/site-packages/freenect.so`. |

---

## 6. Not done here (future work)
- **Microphone array** — the audio firmware is uploaded (we use it for the
  motor), but the 4-channel audio *streaming* path (CEMD data + iso endpoints)
  isn't wired up.
- **Metric / registered depth** — switch capture format to `DEPTH_MM` /
  `DEPTH_REGISTERED`.
- **`/dev/video` device** — pipe frames through `v4l2loopback` so generic apps
  (OBS, ffplay, browsers) see the Kinect as a normal webcam.

## What works
RGB + depth streaming (~30 fps), tilt motor (±31°), status LED, and
accelerometer — all controllable from the browser UI or CLI, camera and motor
simultaneously.

## Environment this was validated on
Ubuntu 22.04.5 (kernel 6.8), x86_64, Python 3.10, libfreenect master `09a1f09`,
numpy 1.26.4, Cython 3.2.5, pyusb 1.x, Kinect model **1473** with 12 V soldered
directly.
