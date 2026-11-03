"""Session logger → parquet (joints + force proxy + optional FSR + camera meta)."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


@dataclass
class TeleopRow:
    t_us: int
    side: str
    joints: dict[str, int] = field(default_factory=dict)
    pos_error: int | None = None
    current_ma: float | None = None
    contact_force_score: float | None = None
    contact_state: bool | None = None
    fsr_left: float | None = None
    fsr_right: float | None = None
    cam_overhead_fps: float | None = None


class SessionLogger:
    def __init__(self, data_dir: str | Path | None = None):
        root = Path(__file__).resolve().parents[1]
        self.data_dir = Path(data_dir) if data_dir else root / "data"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.path = self.data_dir / f"teleop_session_{stamp}.parquet"
        self._rows: list[dict[str, Any]] = []

    def log(self, row: TeleopRow) -> None:
        d = asdict(row)
        joints = d.pop("joints") or {}
        for k, v in joints.items():
            d[f"joint_{k}"] = v
        self._rows.append(d)

    def flush(self) -> Path | None:
        if not self._rows:
            return None
        try:
            import pandas as pd
        except ImportError:
            # Fallback JSON lines
            import json

            jp = self.path.with_suffix(".jsonl")
            with open(jp, "w") as f:
                for r in self._rows:
                    f.write(json.dumps(r) + "\n")
            return jp
        df = pd.DataFrame(self._rows)
        df.to_parquet(self.path, index=False)
        return self.path
