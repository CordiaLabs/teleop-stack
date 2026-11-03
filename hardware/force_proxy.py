"""Contact-force proxies from Feetech STS3215 gripper telemetry (pre-FSR).

Uses Present_Current (addr 69) and |Goal_Position − Present_Position|.
When RP-S40-ST + ESP32 arrive, keep these as secondary features and swap the
primary contact score to FSR voltage in hardware/tactile.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol


class MotorBus(Protocol):
    def read(self, data_name: str, motor: str, *, normalize: bool = True) -> Any: ...


@dataclass
class GripperForceSample:
    servo_id: int
    present_position: int
    goal_position: int
    pos_error: int
    current_ma: float
    load_raw: int
    contact_force_score: float
    contact_state: bool


def _signed_current_ma(raw: int) -> float:
    """Feetech Present_Current is often a signed 16-bit in ~mA units."""
    if raw > 0x7FFF:
        raw -= 0x10000
    return float(abs(raw))


def estimate_force_score(pos_error: int, current_ma: float) -> float:
    """Heuristic 0–100 contact score. Tune with linkage mechanical advantage."""
    return float(min(100.0, (pos_error * 0.5) + (current_ma * 0.1)))


def read_gripper_force_proxy(
    bus: MotorBus,
    motor_name: str = "gripper",
    *,
    servo_id: int = 6,
    pos_error_threshold: int = 15,
    current_ma_threshold: float = 250.0,
) -> GripperForceSample:
    present = int(bus.read("Present_Position", motor_name, normalize=False))
    goal = int(bus.read("Goal_Position", motor_name, normalize=False))
    try:
        current_raw = int(bus.read("Present_Current", motor_name, normalize=False))
    except Exception:
        current_raw = 0
    try:
        load_raw = int(bus.read("Present_Load", motor_name, normalize=False))
    except Exception:
        load_raw = 0

    pos_error = abs(goal - present)
    current_ma = _signed_current_ma(current_raw)
    score = estimate_force_score(pos_error, current_ma)
    contact = pos_error > pos_error_threshold and current_ma > current_ma_threshold

    return GripperForceSample(
        servo_id=servo_id,
        present_position=present,
        goal_position=goal,
        pos_error=pos_error,
        current_ma=current_ma,
        load_raw=load_raw,
        contact_force_score=round(score, 2),
        contact_state=contact,
    )


def read_gripper_force_scservo(
    port_handler: Any,
    packet_handler: Any,
    servo_id: int = 6,
    *,
    pos_error_threshold: int = 15,
    current_ma_threshold: float = 250.0,
) -> GripperForceSample | None:
    """Direct scservo_sdk path (no lerobot MotorsBus)."""
    ADDR_GOAL = 42
    ADDR_PRESENT = 56
    ADDR_LOAD = 60
    ADDR_CURRENT = 69

    present, result, _ = packet_handler.read2ByteTxRx(port_handler, servo_id, ADDR_PRESENT)
    if result != 0:
        return None
    goal, result, _ = packet_handler.read2ByteTxRx(port_handler, servo_id, ADDR_GOAL)
    if result != 0:
        return None
    current_raw, result, _ = packet_handler.read2ByteTxRx(port_handler, servo_id, ADDR_CURRENT)
    if result != 0:
        current_raw = 0
    load_raw, result, _ = packet_handler.read2ByteTxRx(port_handler, servo_id, ADDR_LOAD)
    if result != 0:
        load_raw = 0

    pos_error = abs(int(goal) - int(present))
    current_ma = _signed_current_ma(int(current_raw))
    score = estimate_force_score(pos_error, current_ma)
    contact = pos_error > pos_error_threshold and current_ma > current_ma_threshold

    return GripperForceSample(
        servo_id=servo_id,
        present_position=int(present),
        goal_position=int(goal),
        pos_error=pos_error,
        current_ma=current_ma,
        load_raw=int(load_raw),
        contact_force_score=round(score, 2),
        contact_state=contact,
    )
