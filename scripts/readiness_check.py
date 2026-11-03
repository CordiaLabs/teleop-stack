#!/usr/bin/env python3
"""Dry readiness: ports, optional motor ping, camera open, force-proxy path."""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hardware.arms import DualArmBus, ping_ports  # noqa: E402
from hardware.force_proxy import estimate_force_score  # noqa: E402
from hardware.tactile import TactileStreamer  # noqa: E402


def main() -> int:
    cfg_path = ROOT / "config" / "hardware_ports.yaml"
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f) or {}

    print("=== Cordia V0 readiness ===")
    print(f"config: {cfg_path}")

    # Force proxy math smoke
    score = estimate_force_score(pos_error=20, current_ma=300)
    print(f"force_proxy smoke score={score} (expect >0)")

    tactile = TactileStreamer(cfg_path)
    tstat = tactile.start()
    print(f"tactile: {tstat}")
    tactile.stop()
    if not tstat.get("available"):
        print("  → FSR pending; teleop will log gripper current/Δθ instead")

    arms = DualArmBus(cfg_path)
    ports = [p for p in (arms.left.port, arms.right.port) if p]
    if not ports:
        print("arms: no ports configured — run scripts/find_devices.py with boards plugged")
    else:
        print(f"arms: pinging {ports} @ {arms.baudrate} baud…")
        ping = ping_ports(ports, arms.baudrate)
        for port, ids in ping.items():
            print(f"  {port}: IDs {ids or '(none)'}")
        print("connecting via lerobot FeetechMotorsBus (if available)…")
        status = arms.connect()
        for side, st in status.items():
            print(f"  {side}: {st}")
            if st.get("ok"):
                side_obj = arms.left if side == "left" else arms.right
                pos = arms.read_positions(side_obj)
                force = arms.read_force(side_obj)
                print(f"    positions={pos}")
                print(f"    force={force}")
        arms.disconnect()

    cams = (cfg.get("cameras") or {})
    print(f"cameras config: {cams}")
    try:
        import cv2

        idx = (cams.get("overhead") or {}).get("index")
        if idx is not None:
            cap = cv2.VideoCapture(int(idx))
            ok = cap.isOpened()
            print(f"  overhead index={idx} open={ok}")
            if ok:
                ret, frame = cap.read()
                print(f"  frame ok={ret} shape={None if frame is None else frame.shape}")
                cap.release()
    except Exception as e:
        print(f"  camera check error: {e}")

    print("=== done ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
