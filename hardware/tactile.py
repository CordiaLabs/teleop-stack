"""
Tactile sensor reader — RP-S40-ST FSRs via ESP32 UDP broadcast node.

Four channels: left_jaw_a, left_jaw_b, right_jaw_a, right_jaw_b.
One sensor per jaw face, two per gripper, two grippers total.

Data stream:
  Receives 20-byte UDP packets on port 5006 from the tactile ESP32 bridge:
  uint32_t seq + float left_fixed + float left_moving + float right_fixed + float right_moving

Slip detection:
  Do NOT threshold on absolute ADC values. FSR sensors creep and hysteresis,
  so sustained grip drifts. Slip shows up as:
    - jaw-to-jaw differential collapsing fast (both jaws lose contact together)
    - short-window variance spiking (noisy contact break)
  Use latest_delta() for baseline-relative readings and slip_score() for
  the differential+variance signal. Log both in your dataset.
"""

from __future__ import annotations

import collections
import socket
import struct
import threading
import time
from typing import NamedTuple

# Packet layout: uint32_t seq (+ optional uint32_t t_us) + float[4] adc
_PACKET_FMT_20 = "<I4f"
_PACKET_SIZE_20 = struct.calcsize(_PACKET_FMT_20)  # 20 bytes (legacy)

_PACKET_FMT_24 = "<2I4f"
_PACKET_SIZE_24 = struct.calcsize(_PACKET_FMT_24)  # 24 bytes (with hardware t_us)


class TactileSample(NamedTuple):
    left_jaw_a: float = 0.0
    left_jaw_b: float = 0.0
    right_jaw_a: float = 0.0
    right_jaw_b: float = 0.0
    timestamp: float = 0.0
    seq: int = 0
    t_us: int = 0

    def as_list(self) -> list[float]:
        return [self.left_jaw_a, self.left_jaw_b, self.right_jaw_a, self.right_jaw_b]


class SlipScore(NamedTuple):
    """Per-gripper slip indicators derived from jaw differential + rolling variance."""
    left_differential: float    # |left_jaw_a - left_jaw_b|, drops on slip
    right_differential: float   # |right_jaw_a - right_jaw_b|, drops on slip
    left_variance: float        # rolling variance of left pair, spikes on slip
    right_variance: float       # rolling variance of right pair, spikes on slip


class TactileReader:
    """
    Non-blocking UDP tactile reader with episode-level baseline subtraction.

    Workflow per episode:
        reader.capture_baseline()   # call just before recording starts
        ...
        delta = reader.latest_delta()   # baseline-subtracted [4] list
        slip  = reader.slip_score()     # SlipScore for provenance

    Baseline is reset to zeros by default (meaning raw readings pass through
    unchanged) so callers that skip capture_baseline() still get sensible output.
    """

    _VARIANCE_WINDOW = 40   # 0.2 s at 200 Hz

    def __init__(
        self,
        udp_port: int = 5006,
        bind_ip: str = "0.0.0.0",
        poll_hz: int = 200,
        port: str | None = None,  # Backward compatibility parameter
        baud: int = 115200,       # Backward compatibility parameter
    ):
        self._udp_port = udp_port
        self._bind_ip = bind_ip
        self._poll_hz = poll_hz

        self._latest = TactileSample(timestamp=time.time())
        self._baseline = TactileSample()
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None

        # Stats & sequence tracking
        self._rx_count = 0
        self._drop_count = 0
        self._last_seq: int | None = None
        self._gap_log: list[dict] = []

        # Rolling buffers for variance
        self._left_buf: collections.deque[float] = collections.deque(
            maxlen=self._VARIANCE_WINDOW
        )
        self._right_buf: collections.deque[float] = collections.deque(
            maxlen=self._VARIANCE_WINDOW
        )

    def start(self) -> None:
        self._running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._bind_ip, self._udp_port))
        self._sock.settimeout(0.5)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

    def latest(self) -> TactileSample:
        """Raw ADC reading (not baseline-subtracted)."""
        with self._lock:
            return self._latest

    def capture_baseline(self, n_samples: int = 25) -> TactileSample:
        """
        Average n_samples readings and store as baseline for this episode.
        Call this just before recording starts, while sensors are unloaded.
        """
        sums = [0.0, 0.0, 0.0, 0.0]
        collected = 0
        deadline = time.time() + 1.0
        while collected < n_samples and time.time() < deadline:
            with self._lock:
                s = self._latest
            sums[0] += s.left_jaw_a
            sums[1] += s.left_jaw_b
            sums[2] += s.right_jaw_a
            sums[3] += s.right_jaw_b
            collected += 1
            time.sleep(0.005)

        n = max(1, collected)
        baseline = TactileSample(
            left_jaw_a=sums[0] / n,
            left_jaw_b=sums[1] / n,
            right_jaw_a=sums[2] / n,
            right_jaw_b=sums[3] / n,
            timestamp=time.time(),
        )
        with self._lock:
            self._baseline = baseline
            self._left_buf.clear()
            self._right_buf.clear()
        return baseline

    def reset_baseline(self) -> None:
        """Reset baseline to zeros (pass-through mode)."""
        with self._lock:
            self._baseline = TactileSample()
            self._left_buf.clear()
            self._right_buf.clear()

    def latest_delta(self) -> list[float]:
        """
        Returns [left_jaw_a, left_jaw_b, right_jaw_a, right_jaw_b] baseline-subtracted.
        This is written to observation.tactile in the dataset.
        """
        with self._lock:
            s = self._latest
            b = self._baseline
        return [
            s.left_jaw_a - b.left_jaw_a,
            s.left_jaw_b - b.left_jaw_b,
            s.right_jaw_a - b.right_jaw_a,
            s.right_jaw_b - b.right_jaw_b,
        ]

    def slip_score(self) -> SlipScore:
        """
        Returns slip detection signal derived from jaw differential + rolling variance.
        A slip event looks like: differential collapsing AND variance spiking.
        """
        with self._lock:
            s = self._latest
            b = self._baseline
            left_vals = list(self._left_buf)
            right_vals = list(self._right_buf)

        la = s.left_jaw_a - b.left_jaw_a
        lb = s.left_jaw_b - b.left_jaw_b
        ra = s.right_jaw_a - b.right_jaw_a
        rb = s.right_jaw_b - b.right_jaw_b

        left_diff = abs(la - lb)
        right_diff = abs(ra - rb)

        def _var(vals: list[float]) -> float:
            if len(vals) < 2:
                return 0.0
            mean = sum(vals) / len(vals)
            return sum((v - mean) ** 2 for v in vals) / len(vals)

        return SlipScore(
            left_differential=left_diff,
            right_differential=right_diff,
            left_variance=_var(left_vals),
            right_variance=_var(right_vals),
        )

    def stats(self) -> dict:
        with self._lock:
            return {
                "rx_count": self._rx_count,
                "drop_count": self._drop_count,
                "drop_rate": self._drop_count / max(1, self._rx_count + self._drop_count),
            }

    def flush_gap_log(self) -> list[dict]:
        with self._lock:
            log = list(self._gap_log)
            self._gap_log = []
            return log

    def _run(self) -> None:
        """Receive UDP packets from Tactile ESP32 on port 5006."""
        while self._running:
            try:
                data, _ = self._sock.recvfrom(64)
                now = time.time()
                if len(data) == _PACKET_SIZE_24:
                    seq, t_us, left_fixed, left_moving, right_fixed, right_moving = struct.unpack(_PACKET_FMT_24, data)
                elif len(data) == _PACKET_SIZE_20:
                    seq, left_fixed, left_moving, right_fixed, right_moving = struct.unpack(_PACKET_FMT_20, data)
                    t_us = int(now * 1e6)
                else:
                    continue

                left_fixed = float(left_fixed)
                left_moving = float(left_moving)
                right_fixed = float(right_fixed)
                right_moving = float(right_moving)

                with self._lock:
                    if self._last_seq is not None:
                        gap = seq - self._last_seq - 1
                        if gap > 0:
                            self._drop_count += gap
                            self._gap_log.append({
                                "seq_expected": self._last_seq + 1,
                                "seq_got": seq,
                                "gap_size": gap,
                                "ts": now,
                                "t_us": t_us,
                            })
                    self._last_seq = seq
                    self._rx_count += 1
                    self._latest = TactileSample(
                        left_jaw_a=left_fixed,
                        left_jaw_b=left_moving,
                        right_jaw_a=right_fixed,
                        right_jaw_b=right_moving,
                        timestamp=now,
                        seq=seq,
                        t_us=t_us,
                    )
                    self._left_buf.append((left_fixed + left_moving) / 2.0)
                    self._right_buf.append((right_fixed + right_moving) / 2.0)
            except socket.timeout:
                continue
            except Exception:
                continue
