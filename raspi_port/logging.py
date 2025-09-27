"""Logging helpers for the Raspberry Pi Micro-DMM port."""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable

from .data_model import MeasurementState


class PiLogger:
    """Persist CSV logs on the Raspberry Pi filesystem."""

    def __init__(self, base_dir: Path | None = None) -> None:
        self.base_dir = base_dir or Path.home() / "micro_dmm_logs"
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.manual_path = self.base_dir / "data.csv"
        self.auto_path = self.base_dir / "data_autologged.csv"
        self.keyboard_path = self.base_dir / "typed_entries.txt"

    def _write_rows(self, path: Path, header: str, end_time: float, rows: Iterable[Iterable[float]]) -> None:
        with path.open("a", newline="") as fp:
            writer = csv.writer(fp)
            writer.writerow([header, f"{end_time:.2f}"])
            for row in rows:
                writer.writerow([f"{value:.3f}" for value in row])

    def write_manual_log(self, state: MeasurementState) -> None:
        rows = state.manual_log.as_rows(include_current=state.current_enabled)
        self._write_rows(self.manual_path, "DataLog End Time", state.log_end_time, rows)

    def write_auto_log(self, state: MeasurementState) -> None:
        rows = state.auto_log.as_rows(include_current=state.current_enabled, start_time=state.auto_log_start_time)
        end_time = state.auto_log.timestamps[0] if state.auto_log.sample_count else 0.0
        self._write_rows(self.auto_path, "DataLogged End Time", end_time, rows)

    def emit_keyboard_line(self, text: str) -> None:
        with self.keyboard_path.open("a", encoding="utf-8") as fp:
            fp.write(text + "\n")
        print(text)

