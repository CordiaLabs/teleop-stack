#!/usr/bin/env python3
"""Enumerate USB serial ports + camera indices; write config/hardware_ports.yaml."""

from __future__ import annotations

import glob
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "config" / "hardware_ports.yaml"


def list_serial_ports() -> list[str]:
    patterns = [
        "/dev/tty.usbserial*",
        "/dev/tty.usbmodem*",
        "/dev/tty.usb*",
        "/dev/cu.usbserial*",
        "/dev/cu.usbmodem*",
    ]
    found: list[str] = []
    for pat in patterns:
        found.extend(glob.glob(pat))
    # Prefer cu.* on macOS for write; keep unique sorted
    return sorted(set(found))


def classify_ports(ports: list[str]) -> dict[str, list[str]]:
    waveshare, esp32, other = [], [], []
    for p in ports:
        low = p.lower()
        if "usbmodem" in low:
            esp32.append(p)
        elif "usbserial" in low or "wchusbserial" in low:
            waveshare.append(p)
        else:
            other.append(p)
    return {"waveshare_like": waveshare, "esp32_like": esp32, "other": other}


def probe_cameras(max_index: int = 8) -> list[dict]:
    import cv2

    cams = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            cams.append({"index": i, "width": w, "height": h})
            cap.release()
    return cams


def main() -> int:
    if CFG.exists():
        with open(CFG) as f:
            cfg = yaml.safe_load(f) or {}
    else:
        cfg = {}

    ports = list_serial_ports()
    classified = classify_ports(ports)
    print("Serial ports:")
    for k, v in classified.items():
        print(f"  {k}: {v or '(none)'}")

    cams = []
    try:
        cams = probe_cameras()
    except Exception as e:
        print(f"Camera probe failed (permission?): {e}", file=sys.stderr)
    print("Cameras:")
    if cams:
        for c in cams:
            print(f"  index={c['index']} {c['width']}x{c['height']}")
    else:
        print("  (none open — grant Camera permission / plug UVC)")

    arms = cfg.setdefault("arms", {})
    left = arms.setdefault("left", {})
    right = arms.setdefault("right", {})
    ws = classified["waveshare_like"]
    if len(ws) >= 1 and not left.get("port"):
        left["port"] = ws[0]
    if len(ws) >= 2 and not right.get("port"):
        right["port"] = ws[1]

    tactile = cfg.setdefault("tactile", {})
    if classified["esp32_like"] and not tactile.get("port"):
        tactile["port"] = classified["esp32_like"][0]

    cameras = cfg.setdefault("cameras", {})
    overhead = cameras.setdefault("overhead", {})
    if cams and overhead.get("index") is None:
        # Prefer non-zero if FaceTime is 0 and a UVC is plugged
        overhead["index"] = cams[-1]["index"] if len(cams) > 1 else cams[0]["index"]

    cfg["_enumeration"] = {
        "serial": classified,
        "cameras": cams,
    }

    with open(CFG, "w") as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)
    print(f"\nWrote {CFG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
