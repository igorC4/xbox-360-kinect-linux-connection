#!/usr/bin/env python3
"""
kinect_motor.py — drive the Xbox 360 Kinect (model 1473) tilt motor + LED and
read the accelerometer, by talking directly to the audio device (045e:02ad)
with pyusb.

On the 1473 the motor/LED/accel live behind the AUDIO device, which boots in a
"bootloader" state (1 interface) that only understands a firmware-upload
protocol. Once audios.bin is uploaded and launched, the device re-enumerates in
"running" state (>=2 interfaces) and accepts motor/LED/accel commands on bulk
endpoints 0x01 (OUT) / 0x81 (IN).

We do BOTH steps here, replicating libfreenect's src/loader.c (upload) and
src/tilt.c (motor protocol) — but via pyusb, because libfreenect's own open
sequence (keep_alive_led + libusb_reset_device) wedges the audio device on this
machine with LIBUSB_ERROR_IO.

Firmware file search order: $KINECT_FW, ./firmware/audios.bin, ./audios.bin,
~/.libfreenect/audios.bin. Get it with libfreenect/src/fwfetcher.py.

Device must be free (stop stream_server.py first).

Usage:
    ./venv/bin/python kinect_motor.py --sweep            # LED walk + tilt sweep
    ./venv/bin/python kinect_motor.py --tilt 20
    ./venv/bin/python kinect_motor.py --led red
    ./venv/bin/python kinect_motor.py --accel
    ./venv/bin/python kinect_motor.py --upload-only      # just flash firmware
"""
import argparse
import os
import struct
import time

import usb.core
import usb.util

VID, PID = 0x045E, 0x02AD
CMD_MAGIC = 0x06022009
ACK_MAGIC = 0x0A6FE000
EP_OUT, EP_IN = 0x01, 0x81
PAGE = 0x4000
LED = {"off": 1, "blink": 2, "green": 3, "red": 4}


def find_fw():
    cands = [os.environ.get("KINECT_FW"),
             "firmware/audios.bin", "audios.bin",
             os.path.expanduser("~/.libfreenect/audios.bin")]
    for c in cands:
        if c and os.path.isfile(c):
            return c
    raise SystemExit("audios.bin not found — run libfreenect/src/fwfetcher.py")


def find_audio():
    return usb.core.find(idVendor=VID, idProduct=PID)


def num_interfaces(dev):
    return dev.get_active_configuration().bNumInterfaces


class Bootloader:
    """Implements libfreenect src/loader.c firmware upload over bulk 0x01/0x81."""
    def __init__(self, dev):
        self.dev = dev
        self.tag = 0
        usb.util.claim_interface(dev, 0)

    def _reply(self):
        r = bytes(self.dev.read(EP_IN, 512, timeout=1000))
        if len(r) < 12:
            raise RuntimeError(f"short bootloader reply ({len(r)} bytes)")
        magic, tag, status = struct.unpack("<III", r[:12])
        if magic != ACK_MAGIC:
            raise RuntimeError(f"bad bootloader reply magic 0x{magic:08x}")
        if tag != self.tag:
            raise RuntimeError(f"bootloader reply tag {tag} != {self.tag}")
        if status != 0:
            print(f"  [warn] bootloader status nonzero: {status}")

    def upload(self, fw):
        magic, _vmin, _vmaj, _vrel, _vpat, base, size, entry = struct.unpack(
            "<IHHHHIII", fw[:24])
        if magic != 0xCA77F00D:
            raise SystemExit(f"bad firmware magic 0x{magic:08X}")
        print(f"[fw] uploading {size} bytes, base=0x{base:08X} entry=0x{entry:08X}")
        addr, sent = base, 0
        idx = 0
        while sent < size:
            n = min(PAGE, size - sent)
            cmd = struct.pack("<IIIIII", CMD_MAGIC, self.tag, n, 0x03, addr, 0)
            self.dev.write(EP_OUT, cmd, timeout=1000)
            off = 0
            while off < n:
                chunk = min(512, n - off)
                self.dev.write(EP_OUT, fw[idx + off: idx + off + chunk], timeout=1000)
                off += chunk
            self._reply()
            addr += n
            idx += n
            sent += n
            self.tag += 1
            if sent % (PAGE * 4) == 0 or sent == size:
                print(f"[fw]   {sent}/{size} bytes")
        # launch
        cmd = struct.pack("<IIIIII", CMD_MAGIC, self.tag, 0, 0x04, entry, 0)
        self.dev.write(EP_OUT, cmd, timeout=1000)
        self._reply()
        self.tag += 1
        print("[fw] launched — device will re-enumerate")
        usb.util.dispose_resources(self.dev)


class Motor:
    """Implements libfreenect src/tilt.c alt (audio) motor protocol."""
    def __init__(self, dev):
        self.dev = dev
        self.tag = 1
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except Exception:
            pass
        usb.util.claim_interface(dev, 0)

    def _cmd(self, cmd, arg1=0, arg2=0, send_len=20):
        pkt = struct.pack("<IIIIi", CMD_MAGIC, self.tag, arg1, cmd, arg2)[:send_len]
        self.dev.write(EP_OUT, pkt, timeout=500)
        self.tag += 1

    def _ack(self):
        r = bytes(self.dev.read(EP_IN, 512, timeout=500))
        if len(r) >= 12:
            magic, _tag, status = struct.unpack("<III", r[:12])
            if magic != ACK_MAGIC:
                print(f"  [warn] motor ack magic 0x{magic:08x}")
        return r

    def set_tilt(self, deg):
        deg = max(-31, min(31, int(deg)))
        self._cmd(0x803B, 0, deg, 20)
        self._ack()
        return deg

    def set_led(self, state):
        self._cmd(0x10, 0, LED[state], 20)
        self._ack()

    def read_accel(self):
        self._cmd(0x8032, 0x68, 0, 16)
        data = bytes(self.dev.read(EP_IN, 256, timeout=500))
        self._ack()
        x, y, z, tilt = struct.unpack("<iiii", data[16:32])
        return {"accel_raw": (x, y, z), "tilt_deg": tilt}


def ensure_running(verbose=True):
    """Return an audio device in running state, flashing firmware if needed."""
    dev = find_audio()
    if dev is None:
        raise SystemExit("045e:02ad not found — powered & enumerated? check lsusb")
    if num_interfaces(dev) >= 2:
        if verbose:
            print("[motor] audio device already running (firmware loaded)")
        return dev
    if verbose:
        print("[motor] audio device in BOOTLOADER state — uploading firmware")
    fw = open(find_fw(), "rb").read()
    Bootloader(dev).upload(fw)
    # wait for re-enumeration into running state
    for _ in range(40):
        time.sleep(0.25)
        d = find_audio()
        try:
            if d is not None and num_interfaces(d) >= 2:
                if verbose:
                    print("[motor] device re-enumerated in running state")
                time.sleep(0.5)
                return d
        except usb.core.USBError:
            pass
    raise SystemExit("device did not re-enumerate into running state after upload")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilt", type=int)
    ap.add_argument("--led", choices=list(LED))
    ap.add_argument("--accel", action="store_true")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--upload-only", action="store_true")
    args = ap.parse_args()

    dev = ensure_running()
    if args.upload_only:
        print("[motor] firmware ready."); return
    m = Motor(dev)
    print("[motor] motor/LED/accel ready")
    try:
        if args.led:
            m.set_led(args.led); print(f"[led ] {args.led}")
        if args.tilt is not None:
            d = m.set_tilt(args.tilt); print(f"[tilt] commanded {d:+d}°")
            time.sleep(2.5); print("       ", m.read_accel())
        if args.accel:
            print("[accel]", m.read_accel())
        if args.sweep:
            for name in ("red", "green", "blink", "off", "green"):
                m.set_led(name); print(f"[led ] {name}"); time.sleep(0.6)
            for target in (0, 25, -25, 0):
                m.set_tilt(target); print(f"[tilt] commanded {target:+d}°")
                time.sleep(3.0); print("       ", m.read_accel())
        if not any([args.led, args.tilt is not None, args.accel, args.sweep]):
            print("[accel]", m.read_accel(), "(no action; try --sweep)")
    finally:
        usb.util.dispose_resources(m.dev)


if __name__ == "__main__":
    main()
