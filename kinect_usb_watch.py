#!/usr/bin/env python3
"""
kinect_usb_watch.py — millisecond-resolution USB watcher for catching a Kinect
(or anything) that enumerates briefly and then drops off the bus.

Two independent sources are merged into one log:
  [UDEV]  lines from `udevadm monitor --kernel --udev --property` — the kernel's
          own event stream, carries a monotonic [secs.usecs] stamp; this is the
          most reliable way to catch a device that appears for only a few ms.
  [POLL]  a tight ~5 ms loop over /sys/bus/usb/devices reading idVendor/idProduct;
          reports APPEAR / GONE / CHANGE transitions for watched devices.

Every line is prefixed with a wall-clock timestamp at millisecond precision.

Usage:
    python3 kinect_usb_watch.py                 # watch 045e (Microsoft/Kinect) only
    python3 kinect_usb_watch.py --all           # watch every USB device
    python3 kinect_usb_watch.py --vid 045e 046d # watch specific vendor ids
    python3 kinect_usb_watch.py --interval 0.002# 2 ms poll
Logs to stdout AND to kinect_usb_watch.log in the cwd.
Stop with Ctrl-C.
"""
import argparse
import glob
import os
import subprocess
import sys
import threading
import time
from datetime import datetime

SYS = "/sys/bus/usb/devices"
LOGFILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "kinect_usb_watch.log")

_lock = threading.Lock()
_logf = open(LOGFILE, "a", buffering=1)


def ts():
    # wall-clock with millisecond precision
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def emit(tag, msg):
    line = f"{ts()} [{tag}] {msg}"
    with _lock:
        print(line, flush=True)
        _logf.write(line + "\n")


def read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except Exception:
        return None


def snapshot(vids, watch_all):
    """Return {devpath: (vid, pid, product, bDeviceClass, busnum, devnum, maxpower)}."""
    out = {}
    for d in glob.glob(f"{SYS}/*/"):
        vid = read(os.path.join(d, "idVendor"))
        if vid is None:
            continue  # not a real device node (interfaces, hubs roots handled separately)
        if not watch_all and vid not in vids:
            continue
        pid = read(os.path.join(d, "idProduct"))
        out[d.rstrip("/")] = (
            vid,
            pid,
            read(os.path.join(d, "product")) or "?",
            read(os.path.join(d, "bDeviceClass")),
            read(os.path.join(d, "busnum")),
            read(os.path.join(d, "devnum")),
            read(os.path.join(d, "bMaxPower")),
        )
    return out


def describe(info):
    vid, pid, product, cls, bus, dev, pwr = info
    return f"{vid}:{pid} cls={cls} bus={bus} dev={dev} maxpwr={pwr} '{product}'"


def poller(vids, watch_all, interval):
    prev = snapshot(vids, watch_all)
    emit("POLL", f"baseline: {len(prev)} watched device(s)")
    for path, info in sorted(prev.items()):
        emit("POLL", f"  present {os.path.basename(path)}  {describe(info)}")
    while True:
        time.sleep(interval)
        cur = snapshot(vids, watch_all)
        if cur == prev:
            continue
        # appeared
        for path in cur.keys() - prev.keys():
            emit("POLL", f"APPEAR  {os.path.basename(path)}  {describe(cur[path])}")
        # gone
        for path in prev.keys() - cur.keys():
            emit("POLL", f"GONE    {os.path.basename(path)}  {describe(prev[path])}")
        # changed (e.g. devnum bump = re-enumeration)
        for path in cur.keys() & prev.keys():
            if cur[path] != prev[path]:
                emit("POLL", f"CHANGE  {os.path.basename(path)}  {describe(prev[path])} -> {describe(cur[path])}")
        prev = cur


def udev_monitor(vids, watch_all):
    cmd = ["stdbuf", "-oL", "udevadm", "monitor", "--kernel", "--udev", "--property",
           "--subsystem-match=usb"]
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, bufsize=1)
    except FileNotFoundError:
        emit("UDEV", "udevadm not found — running with POLL only")
        return
    block = []
    capture = False
    for raw in p.stdout:
        line = raw.rstrip("\n")
        if not line.strip():
            block = []
            capture = False
            continue
        # Event header line, e.g. "KERNEL[12345.678] add /devices/.../usb1 (usb)"
        if (line.startswith("KERNEL[") or line.startswith("UDEV[")):
            block = [line]
            capture = True
            # decide relevance after we read a few props, but always show add/remove headers
            continue
        if capture:
            block.append(line)
            # filter to watched vendors unless --all
            if line.startswith("ID_VENDOR_ID=") or line.startswith("PRODUCT="):
                pass
        # emit interesting property lines inline with the header context
        if capture and (line.startswith("ACTION=") or line.startswith("PRODUCT=")
                        or line.startswith("ID_VENDOR_ID=") or line.startswith("ID_MODEL=")
                        or line.startswith("DEVPATH=") or line.startswith("ID_MODEL_ID=")
                        or line.startswith("DEVNUM=") or line.startswith("BUSNUM=")):
            hdr = block[0] if block else ""
            relevant = watch_all
            joined = "\n".join(block)
            if not relevant:
                for v in vids:
                    if f"ID_VENDOR_ID={v}" in joined or f"PRODUCT={v.lstrip('0')}/" in joined or f"/{v}/" in joined:
                        relevant = True
                        break
            if relevant:
                emit("UDEV", f"{hdr.split(']')[0]}] {line}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vid", nargs="+", default=["045e"], help="vendor ids to watch (hex, no 0x)")
    ap.add_argument("--all", action="store_true", help="watch every USB device")
    ap.add_argument("--interval", type=float, default=0.005, help="poll interval seconds (default 5ms)")
    args = ap.parse_args()
    vids = [v.lower() for v in args.vid]

    emit("INFO", "=" * 70)
    emit("INFO", f"kinect_usb_watch start | watch={'ALL' if args.all else vids} "
                 f"| poll={args.interval*1000:.0f}ms | log={LOGFILE}")
    emit("INFO", "Replug / power-cycle the Kinect now. Ctrl-C to stop.")

    t = threading.Thread(target=poller, args=(vids, args.all, args.interval), daemon=True)
    t.start()
    try:
        udev_monitor(vids, args.all)
    except KeyboardInterrupt:
        pass
    finally:
        emit("INFO", "stopped")


if __name__ == "__main__":
    main()
