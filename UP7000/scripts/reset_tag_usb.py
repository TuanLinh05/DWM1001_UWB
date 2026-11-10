#!/usr/bin/env python3
"""Recover a stuck SEGGER VCOM without resetting PX4 or flashing the TAG.

Run with sudo. Logging pauses briefly and resumes in a new session if the
service was active. Select --serial when more than one J-Link is connected.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import time


def find_jlink(root: Path, serial: str | None = None) -> Path:
    matches = []
    for vendor in root.glob("*/idVendor"):
        device = vendor.parent
        try:
            if vendor.read_text().strip() != "1366":
                continue
            if "J-Link" not in (device / "product").read_text():
                continue
            if serial is not None and (device / "serial").read_text().strip() != serial:
                continue
            matches.append(device)
        except OSError:
            continue
    if len(matches) != 1:
        raise RuntimeError(f"Expected exactly one matching SEGGER J-Link, found {len(matches)}; use --serial")
    return matches[0]


def main() -> None:
    import fcntl  # Linux only; discovery can be tested on the development PC.

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="SEGGER USB serial number")
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error("Run with sudo to reset the USB device and pause the logger service")
    device = find_jlink(Path("/sys/bus/usb/devices"), args.serial)
    service = "uwb-px4-logger"
    active = subprocess.run(["systemctl", "is-active", "--quiet", service]).returncode == 0
    print(f"Resetting only {device.name}: {(device / 'product').read_text().strip()}, "
          f"serial {(device / 'serial').read_text().strip()}", flush=True)
    if active:
        subprocess.run(["systemctl", "stop", service], check=True)
    try:
        # Refuse to reset if another application still owns this TAG's tty.
        for tty in device.glob("**/tty/tty*"):
            users = subprocess.run(["fuser", f"/dev/{tty.name}"], capture_output=True, text=True)
            if users.stdout.strip():
                raise RuntimeError(f"/dev/{tty.name} is still in use by PID(s) {users.stdout.strip()}")
        bus = int((device / "busnum").read_text())
        number = int((device / "devnum").read_text())
        fd = os.open(f"/dev/bus/usb/{bus:03}/{number:03}", os.O_WRONLY)
        try:
            fcntl.ioctl(fd, 0x5514, 0)  # USBDEVFS_RESET
        finally:
            os.close(fd)
        time.sleep(2)
    finally:
        if active:
            subprocess.run(["systemctl", "start", service], check=True)
    print("USB reset complete; logger restarted" if active else "USB reset complete; logger left stopped")


if __name__ == "__main__":
    main()
