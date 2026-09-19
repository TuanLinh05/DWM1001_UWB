"""Non-blocking session recorder for UWB telemetry."""

from __future__ import annotations

import csv
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import queue
import re
import threading
import time
from typing import TextIO

from host_range_filter import (
    GATE_MARGIN_DOWN_MM,
    GATE_MARGIN_UP_MM,
    HOST_FILTER_VERSION,
    MAX_RADIAL_SPEED_MM_S,
    POSITION_GAIN,
    QUIET_RESIDUAL_MM,
    STALE_RESET_MS,
    VELOCITY_DAMPING,
    VELOCITY_GAIN,
    WEAK_FPP_DBM,
    WEAK_POSITION_GAIN_SCALE,
)
from telemetry_protocol import (
    InfoMessage,
    ParserCounters,
    RangeMessage,
    StatsMessage,
    status_names,
)


FORMAT_VERSION = 1
QUEUE_CAPACITY = 20_000
_STOP = object()


def _now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return cleaned.strip("._") or "UART"


class SessionRecorder:
    """Write telemetry on a worker thread so serial/UI processing stays responsive."""

    def __init__(self, base_directory: Path, source: str, baud: int) -> None:
        self.source = source
        self.baud = baud
        self.started_iso = _now_iso()
        self.started_monotonic = time.monotonic()
        self.error: str | None = None
        self.queue_drops = 0
        self.range_frames = 0
        self.range_samples = 0
        self.info_frames = 0
        self.stats_frames = 0
        self.uart_snapshots = 0
        self.raw_frames = 0
        self.raw_bytes = 0
        self.event_lines = 0
        self._accepting = True
        self._queue: queue.Queue = queue.Queue(maxsize=QUEUE_CAPACITY)

        base_directory = Path(base_directory).expanduser().resolve()
        base_directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = f"{stamp}_{_safe_name(source)}"
        self.session_directory = base_directory / prefix
        suffix = 1
        while self.session_directory.exists():
            self.session_directory = base_directory / f"{prefix}_{suffix:02d}"
            suffix += 1
        self.session_directory.mkdir()

        self._range_file = self._open_text("range.csv")
        self._stats_file = self._open_text("stats.csv")
        self._uart_file = self._open_text("uart.csv")
        self._info_file = self._open_text("info.jsonl")
        self._event_file = self._open_text("events.log")
        self._raw_file = (self.session_directory / "raw_telemetry.bin").open("wb")
        self._range_writer = csv.writer(self._range_file)
        self._stats_writer = csv.writer(self._stats_file)
        self._uart_writer = csv.writer(self._uart_file)
        self._range_writer.writerow(
            (
                "host_time_iso", "host_elapsed_s", "tag_sequence", "tag_time_ms",
                "anchor_id", "valid", "status_hex", "status_text", "age_ms",
                "raw_mm", "filtered_mm", "host_filtered_mm", "fpp_cdbm",
            )
        )
        self._stats_writer.writerow(
            (
                "host_time_iso", "host_elapsed_s", "tag_sequence", "tag_time_ms",
                "poll_count", "response_ok_count", "rx_timeout_count",
                "rx_error_count", "cycle_overrun_count", "uart_overflow_count",
                "cycle_hz", "operation_hz",
            )
        )
        self._uart_writer.writerow(
            (
                "host_time_iso", "host_elapsed_s", "byte_rate",
                "bytes_received", "valid_frames", "discarded_bytes",
                "crc_errors", "length_errors", "version_errors", "decode_errors",
            )
        )
        self._write_metadata(completed=False)
        self._thread = threading.Thread(
            target=self._run, name="uwb-session-writer", daemon=True
        )
        self._thread.start()

    def _open_text(self, name: str) -> TextIO:
        return (self.session_directory / name).open("w", encoding="utf-8", newline="")

    @property
    def active(self) -> bool:
        return self._accepting and self._thread.is_alive() and self.error is None

    def record_raw(self, data: bytes) -> None:
        self._enqueue(("raw", bytes(data)))

    def record_message(
        self,
        message: InfoMessage | RangeMessage | StatsMessage,
        host_filtered_mm: dict[int, int] | None = None,
    ) -> None:
        self._enqueue((
            "message",
            _now_iso(),
            time.monotonic() - self.started_monotonic,
            message,
            dict(host_filtered_mm or {}),
        ))

    def record_event(self, line: str) -> None:
        self._enqueue(("event", _now_iso(), line))

    def record_uart_stats(self, counters: ParserCounters, byte_rate: float) -> None:
        self._enqueue((
            "uart",
            _now_iso(),
            time.monotonic() - self.started_monotonic,
            counters.snapshot(),
            float(byte_rate),
        ))

    def _enqueue(self, item: tuple) -> None:
        if not self._accepting or self.error is not None:
            return
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.queue_drops += 1

    def stop(self) -> None:
        if not hasattr(self, "_thread"):
            return
        if self._accepting:
            self._accepting = False
            while self._thread.is_alive():
                try:
                    self._queue.put(_STOP, timeout=0.2)
                    break
                except queue.Full:
                    continue
        self._thread.join(timeout=5.0)
        if self._thread.is_alive() and self.error is None:
            self.error = "Không thể hoàn tất file log trong 5 giây"

    def _run(self) -> None:
        last_flush = time.monotonic()
        try:
            while True:
                item = self._queue.get()
                if item is _STOP:
                    break
                kind = item[0]
                if kind == "raw":
                    data = item[1]
                    self._raw_file.write(data)
                    self.raw_frames += 1
                    self.raw_bytes += len(data)
                elif kind == "event":
                    self._event_file.write(f"[{item[1]}] {item[2]}\n")
                    self.event_lines += 1
                elif kind == "uart":
                    self._write_uart_stats(item[1], item[2], item[3], item[4])
                else:
                    self._write_message(item[1], item[2], item[3], item[4])

                now = time.monotonic()
                if now - last_flush >= 1.0:
                    self._flush()
                    last_flush = now
        except (OSError, ValueError, TypeError) as exc:
            self.error = str(exc)
            self._accepting = False
        finally:
            self._flush_and_close()
            try:
                self._write_metadata(completed=self.error is None)
            except OSError as exc:
                if self.error is None:
                    self.error = str(exc)

    def _write_message(
        self,
        host_time: str,
        host_elapsed: float,
        message: InfoMessage | RangeMessage | StatsMessage,
        host_filtered_mm: dict[int, int],
    ) -> None:
        if isinstance(message, RangeMessage):
            for sample in message.samples:
                self._range_writer.writerow(
                    (
                        host_time, f"{host_elapsed:.6f}", message.sequence, message.time_ms,
                        sample.anchor_id, int(sample.valid), f"0x{sample.status:02X}",
                        "|".join(status_names(sample.status)), sample.age_ms,
                        sample.raw_mm, sample.filtered_mm,
                        host_filtered_mm.get(sample.anchor_id, ""), sample.fpp_cdbm,
                    )
                )
                self.range_samples += 1
            self.range_frames += 1
        elif isinstance(message, StatsMessage):
            self._stats_writer.writerow(
                (
                    host_time,
                    f"{host_elapsed:.6f}",
                    message.sequence,
                    message.time_ms,
                    message.poll_count,
                    message.response_ok_count,
                    message.rx_timeout_count,
                    message.rx_error_count,
                    message.cycle_overrun_count,
                    message.uart_overflow_count,
                    message.cycle_hz,
                    message.operation_hz,
                )
            )
            self.stats_frames += 1
        elif isinstance(message, InfoMessage):
            record = asdict(message)
            record["host_time_iso"] = host_time
            record["host_elapsed_s"] = round(host_elapsed, 6)
            self._info_file.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.info_frames += 1

    def _write_uart_stats(
        self,
        host_time: str,
        host_elapsed: float,
        counters: ParserCounters,
        byte_rate: float,
    ) -> None:
        self._uart_writer.writerow(
            (
                host_time,
                f"{host_elapsed:.6f}",
                f"{byte_rate:.3f}",
                counters.bytes_received,
                counters.valid_frames,
                counters.discarded_bytes,
                counters.crc_errors,
                counters.length_errors,
                counters.version_errors,
                counters.decode_errors,
            )
        )
        self.uart_snapshots += 1

    def _flush(self) -> None:
        for handle in (
            self._range_file, self._stats_file, self._uart_file, self._info_file,
            self._event_file, self._raw_file,
        ):
            handle.flush()

    def _flush_and_close(self) -> None:
        for handle in (
            self._range_file, self._stats_file, self._uart_file, self._info_file,
            self._event_file, self._raw_file,
        ):
            try:
                handle.flush()
                handle.close()
            except (OSError, ValueError):
                pass

    def _write_metadata(self, completed: bool) -> None:
        metadata = {
            "format_version": FORMAT_VERSION,
            "source": self.source,
            "baud": self.baud,
            "started_at": self.started_iso,
            "ended_at": _now_iso() if completed or self.error else None,
            "duration_s": round(time.monotonic() - self.started_monotonic, 6),
            "completed": completed,
            "error": self.error,
            "range_frames": self.range_frames,
            "range_samples": self.range_samples,
            "info_frames": self.info_frames,
            "stats_frames": self.stats_frames,
            "uart_snapshots": self.uart_snapshots,
            "raw_frames": self.raw_frames,
            "raw_bytes": self.raw_bytes,
            "event_lines": self.event_lines,
            "queue_drops": self.queue_drops,
            "host_filter": {
                "version": HOST_FILTER_VERSION,
                "model": "constant_velocity_alpha_beta",
                "position_gain": POSITION_GAIN,
                "velocity_gain": VELOCITY_GAIN,
                "velocity_damping": VELOCITY_DAMPING,
                "weak_fpp_dbm": WEAK_FPP_DBM,
                "weak_position_gain_scale": WEAK_POSITION_GAIN_SCALE,
                "max_radial_speed_mm_s": MAX_RADIAL_SPEED_MM_S,
                "gate_margin_up_mm": GATE_MARGIN_UP_MM,
                "gate_margin_down_mm": GATE_MARGIN_DOWN_MM,
                "quiet_residual_mm": QUIET_RESIDUAL_MM,
                "stale_reset_ms": STALE_RESET_MS,
                "maximum_range_mm": None,
                "snap_after_samples": None,
            },
        }
        temporary = self.session_directory / "session.json.tmp"
        temporary.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        temporary.replace(self.session_directory / "session.json")
