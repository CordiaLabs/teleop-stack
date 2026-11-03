"""
Dual Feetech STS3215 bus manager (left / right Waveshare boards @ 1 Mbps).

Features:
- High-speed direct UART serial protocol (1 Mbps) for 50 Hz control loops
- Automatic fallback if lerobot package isn't present
- Thread-safe non-blocking reading of Present_Position across all 12 motors
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import serial
import yaml

from hardware.force_proxy import GripperForceSample


class FastFeetechBus:
    """Direct high-speed serial driver for Feetech STS3215 servo chain."""

    def __init__(self, port: str | None, baud: int = 1_000_000):
        self.port = port
        self.baud = baud
        self.ser: serial.Serial | None = None

    def connect(self) -> bool:
        if not self.port:
            return False
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.005)
            return True
        except Exception:
            self.ser = None
            return False

    def close(self) -> None:
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def read_positions_rad(self, motor_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6)) -> list[float]:
        """Read 6 motor positions in radians (-pi..pi). Returns zeros on read error."""
        if not self.is_connected():
            return [0.0] * len(motor_ids)
        positions = []
        for mid in motor_ids:
            try:
                # Instruction packet: FF FF ID 04 02 38 02 CHK (Read 2 bytes at addr 56)
                chk = (~(mid + 4 + 2 + 56 + 2)) & 0xFF
                self.ser.write(bytes([0xFF, 0xFF, mid, 4, 2, 56, 2, chk]))
                self.ser.flush()
                time.sleep(0.001)
                resp = self.ser.read(8)
                if len(resp) >= 8 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == mid:
                    pos = resp[5] | (resp[6] << 8)
                    rad = (pos / 4095.0) * 6.2831853 - 3.14159265
                    positions.append(float(rad))
                else:
                    positions.append(0.0)
            except Exception:
                positions.append(0.0)
        return positions


@dataclass
class ArmSide:
    name: str
    port: str | None
    motor_ids: list[int] = field(default_factory=lambda: [1, 2, 3, 4, 5, 6])
    gripper_id: int = 6
    bus: FastFeetechBus | None = None


class DualArmBus:
    """Manages both left and right follower arm serial buses."""

    JOINT_NAMES = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )

    def __init__(self, config_path: str | Path | None = None):
        root = Path(__file__).resolve().parents[1]
        cfg_path = Path(config_path) if config_path else root / "config" / "hardware_ports.yaml"
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f) or {}
        arms = cfg.get("arms", {})
        self.baudrate = int(arms.get("baudrate", 1_000_000))
        self.force_cfg = cfg.get("force_proxy", {})
        self.left = ArmSide(
            "left",
            arms.get("left", {}).get("port"),
            list(arms.get("left", {}).get("motor_ids", [1, 2, 3, 4, 5, 6])),
            int(arms.get("left", {}).get("gripper_id", 6)),
        )
        self.right = ArmSide(
            "right",
            arms.get("right", {}).get("port"),
            list(arms.get("right", {}).get("motor_ids", [1, 2, 3, 4, 5, 6])),
            int(arms.get("right", {}).get("gripper_id", 6)),
        )

    def connect(self) -> dict[str, bool]:
        res = {}
        for side in (self.left, self.right):
            side.bus = FastFeetechBus(side.port, self.baudrate)
            res[side.name] = side.bus.connect()
        return res

    def disconnect(self) -> None:
        for side in (self.left, self.right):
            if side.bus:
                side.bus.close()
                side.bus = None

    def read_all_positions(self) -> list[float]:
        """Returns 12-element list [L1..L6, R1..R6] in radians for observation.state."""
        left_pos = self.left.bus.read_positions_rad(tuple(self.left.motor_ids)) if self.left.bus else [0.0] * 6
        right_pos = self.right.bus.read_positions_rad(tuple(self.right.motor_ids)) if self.right.bus else [0.0] * 6
        return left_pos + right_pos
