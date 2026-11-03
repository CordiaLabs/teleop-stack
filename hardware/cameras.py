"""Non-blocking OpenCV camera threads with UID-pinned 3-camera policy."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)

# Standard known hardware cameras & native rates
CAMERA_DEFAULTS = {
    "overhead": {
        "unique_id": "0x11000005a39230",
        "name": "USB2.0_CAM1",
        "width": 1280,
        "height": 720,
        "native_fps": 24.4,
    },
    "wrist_left": {
        "unique_id": "0x14200005a39230",
        "name": "USB2.0_CAM1",
        "width": 1280,
        "height": 720,
        "native_fps": 25.0,
    },
    "wrist_right": {
        "unique_id": "0x11200000c456366",
        "name": "Innomaker-U20CAM-1080p-S1",
        "width": 1280,
        "height": 720,
        "native_fps": 29.4,
    },
}

REQUIRED_ROLES = ["overhead", "wrist_left", "wrist_right"]


@dataclass
class CameraFrame:
    index: int
    t_us: int
    frame: np.ndarray
    fps: float


class CameraWorker:
    def __init__(self, index: int, width: int = 1280, height: int = 720, name: str = "cam"):
        self.index = index
        self.width = width
        self.height = height
        self.name = name
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._t_us = 0
        self._fps = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.opened = False
        self.error: str | None = None

    def start(self) -> bool:
        import cv2

        cap = cv2.VideoCapture(self.index)
        if not cap.isOpened():
            self.error = f"cannot open camera index {self.index}"
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.opened = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(cap,), name=self.name, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.5)
            self._thread = None
        self.opened = False

    def latest(self) -> CameraFrame | None:
        with self._lock:
            if self._frame is None:
                return None
            return CameraFrame(self.index, self._t_us, self._frame.copy(), self._fps)

    def _run(self, cap: Any) -> None:
        n = 0
        t0 = time.perf_counter()
        try:
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.01)
                    continue
                now = time.time_ns() // 1000
                n += 1
                elapsed = time.perf_counter() - t0
                fps = n / elapsed if elapsed > 0 else 0.0
                with self._lock:
                    self._frame = frame
                    self._t_us = now
                    self._fps = fps
        finally:
            cap.release()


class MultiCameraReader:
    """Manages simultaneous capture for all required cameras (overhead, wrist_left, wrist_right).

    Enforces strict UID verification and native FPS tracking.
    """

    def __init__(
        self,
        cam_cfgs: dict[str, dict] | None = None,
        required_roles: list[str] | None = None,
    ):
        self.cfgs = cam_cfgs or {}
        self.required_roles = required_roles or REQUIRED_ROLES
        self._workers: dict[str, CameraWorker] = {}
        self._specs: dict[str, dict[str, Any]] = {}

        # Merge configs with CAMERA_DEFAULTS
        for role in self.required_roles:
            default = dict(CAMERA_DEFAULTS.get(role, {}))
            user_cfg = self.cfgs.get(role) or {}
            default.update(user_cfg)
            self._specs[role] = default

    def check_readiness(self) -> dict[str, Any]:
        """Check hardware presence via system_profiler unique_id resolution."""
        try:
            from hardware.robot_cameras import resolve_roles
            live_roles = resolve_roles(force=True)
        except Exception as e:
            logger.warning("Failed to resolve live camera roles: %s", e)
            live_roles = {}

        report: dict[str, Any] = {
            "all_present": True,
            "roles": {},
            "missing": [],
        }

        for role in self.required_roles:
            spec = self._specs[role]
            expected_uid = spec.get("unique_id", "")
            live = live_roles.get(role)
            if live and (not expected_uid or live.get("unique_id") == expected_uid):
                report["roles"][role] = {
                    "present": True,
                    "index": live.get("index"),
                    "name": live.get("name"),
                    "unique_id": live.get("unique_id"),
                    "native_fps": spec.get("native_fps", 30.0),
                    "resolution": (spec.get("width", 1280), spec.get("height", 720)),
                }
            else:
                report["all_present"] = False
                report["missing"].append(role)
                report["roles"][role] = {
                    "present": False,
                    "index": None,
                    "name": spec.get("name", "Unknown"),
                    "unique_id": expected_uid,
                    "native_fps": spec.get("native_fps", 30.0),
                    "resolution": (spec.get("width", 1280), spec.get("height", 720)),
                }

        return report

    def start(self, allow_missing: bool = False) -> dict[str, bool]:
        readiness = self.check_readiness()
        if not readiness["all_present"] and not allow_missing:
            logger.error("Missing required cameras: %s", readiness["missing"])

        status = {}
        for role, info in readiness["roles"].items():
            if not info["present"]:
                status[role] = False
                continue
            idx = info["index"]
            spec = self._specs[role]
            worker = CameraWorker(
                index=idx,
                width=spec.get("width", 1280),
                height=spec.get("height", 720),
                name=f"cam-{role}",
            )
            ok = worker.start()
            if ok:
                self._workers[role] = worker
            status[role] = ok

        return status

    def stop(self) -> None:
        for worker in self._workers.values():
            worker.stop()
        self._workers.clear()

    def latest_frames(self) -> dict[str, np.ndarray | None]:
        frames = {}
        for role in self.required_roles:
            worker = self._workers.get(role)
            if worker:
                f = worker.latest()
                frames[role] = f.frame if f else None
            else:
                frames[role] = None
        return frames

    def get_native_fps(self) -> dict[str, float]:
        fps_map = {}
        for role in self.required_roles:
            worker = self._workers.get(role)
            if worker and worker.opened:
                f = worker.latest()
                if f and f.fps > 0:
                    fps_map[role] = round(f.fps, 1)
                    continue
            fps_map[role] = self._specs[role].get("native_fps", 30.0)
        return fps_map

    def get_camera_specs(self) -> dict[str, dict[str, Any]]:
        return dict(self._specs)
