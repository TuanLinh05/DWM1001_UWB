"""Non-blocking session recorder for UWB telemetry."""

from __future__ import annotations

import csv
from dataclasses import asdict, fields
from datetime import datetime
import json
from pathlib import Path
import queue
import re
import threading
import time
from typing import TextIO

from anchor_timeline import SUMMARY_COLUMNS, TIMELINE_COLUMNS, AnchorTimeline
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
    INT16_MIN,
    AnchorInfoMessage,
    CmdAckMessage,
    DecodedMessage,
    DeviceInfoMessage,
    DiagAnchorMessage,
    DiagSystemMessage,
    GatewayHealthMessage,
    InfoMessage,
    ParserCounters,
    RangeMeasMessage,
    RangeMessage,
    SnifferFrameMessage,
    StatsMessage,
    status_names,
)


FORMAT_VERSION = 2
QUEUE_CAPACITY = 20_000
_STOP = object()

# Every decoded type without a hand-made layout above gets its own CSV: one
# row per frame, columns taken from the message dataclass. The file is
# created on the first frame of that type.
MESSAGE_FILES = {
    DiagAnchorMessage: "diag_anchor.csv",
    AnchorInfoMessage: "anchor_info.csv",
    DiagSystemMessage: "diag_system.csv",
    DeviceInfoMessage: "device_info.csv",
    CmdAckMessage: "cmd_ack.csv",
    GatewayHealthMessage: "gateway_health.csv",
    SnifferFrameMessage: "sniffer_frames.csv",
}
# Identifiers and bit masks read better in hex.
_HEX_FIELDS = frozenset({
    "git_hash", "build_hash", "build_config_hash", "calibration_profile_id",
    "device_id", "otp_part_id", "otp_lot_id", "otp_ldotune", "tx_power_register",
    "telemetry_features", "active_mask", "calibrated_mask", "last_config_mismatch",
    "reset_cause", "anchor_status", "boot_id", "status",
})
_TUPLE_COLUMNS = {"position_mm": ("position_x_mm", "position_y_mm", "position_z_mm")}


def _now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return cleaned.strip("._") or "UART"


def _power(cdbm: int) -> int | str:
    """Empty cell for a power the TAG did not measure (INT16_MIN)."""
    return "" if cdbm == INT16_MIN else cdbm


def _cell(name: str, value: object) -> object:
    if isinstance(value, bool):
        return int(value)
    if value is None:
        return ""
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    if isinstance(value, float):
        return f"{value:.3f}"
    if name in _HEX_FIELDS and isinstance(value, int):
        return f"0x{value:X}"
    return value


def _flatten(message: object) -> dict[str, object]:
    """One CSV row from a message dataclass; dicts and tuples become columns."""
    row: dict[str, object] = {}
    for item in fields(message):  # type: ignore[arg-type]
        value = getattr(message, item.name)
        if isinstance(value, dict):
            for key, part in value.items():
                row[f"{item.name}_{key}"] = _cell(key, part)
        elif isinstance(value, tuple):
            names = _TUPLE_COLUMNS.get(item.name)
            for index, part in enumerate(value):
                column = names[index] if names and index < len(names) else f"{item.name}_{index}"
                row[column] = _cell(item.name, part)
        else:
            row[item.name] = _cell(item.name, value)
    if isinstance(message, CmdAckMessage):
        row["result_name"] = message.result_name
    return row


def _file_name_for(message: object) -> str:
    name = MESSAGE_FILES.get(type(message))
    if name is None:
        snake = re.sub(r"(?<!^)(?=[A-Z])", "_", type(message).__name__).lower()
        name = f"{snake.removesuffix('_message')}.csv"
    return name


class SessionRecorder:
    """Write telemetry on a worker thread so serial/UI processing stays responsive."""

    def __init__(self, base_directory: Path, source: str, baud: int) -> None:
        self.source = source
        self.baud = baud
        started_wall = datetime.now().astimezone()
        self.started_iso = started_wall.isoformat(timespec="milliseconds")
        self.started_monotonic = time.monotonic()
        self.error: str | None = None
        self.queue_drops = 0
        self.range_frames = 0
        self.range_samples = 0
        self.meas_records = 0
        self.info_frames = 0
        self.stats_frames = 0
        self.uart_snapshots = 0
        self.raw_frames = 0
        self.raw_bytes = 0
        self.event_lines = 0
        # Frames written to the per-type CSVs of MESSAGE_FILES, by file name.
        self.message_counts: dict[str, int] = {}
        self.summary_written = False
        # Parser counters are cumulative per connection: sum the increments.
        self._uart_previous: ParserCounters | None = None
        self.uart_bytes = 0
        self.uart_discarded = 0
        self.uart_crc_errors = 0
        # RANGE_MEAS meas_seq per TAG boot: [first, last, received].
        self._meas_spans: dict[int, list[int]] = {}
        self._generic_files: dict[str, TextIO] = {}
        self._generic_writers: dict[str, csv.DictWriter] = {}
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
        self._meas_file = self._open_text("meas.csv")
        self._stats_file = self._open_text("stats.csv")
        self._uart_file = self._open_text("uart.csv")
        self._info_file = self._open_text("info.jsonl")
        self._event_file = self._open_text("events.log")
        self._raw_file = (self.session_directory / "raw_telemetry.bin").open("wb")
        self._range_writer = csv.writer(self._range_file)
        self._meas_writer = csv.writer(self._meas_file)
        self._stats_writer = csv.writer(self._stats_file)
        self._uart_writer = csv.writer(self._uart_file)
        self._range_writer.writerow(
            (
                "host_time_iso", "host_elapsed_s", "tag_sequence", "tag_time_ms",
                "anchor_id", "valid", "status_hex", "status_text", "age_ms",
                "raw_mm", "filtered_mm", "host_filtered_mm", "fpp_cdbm",
            )
        )
        # One row per RANGE_MEAS record (TYPE 0x10). raw_mm is before any
        # offset; corrected_mm only means something with flags bit 1 (CAL_OK)
        # and filtered_mm with bit 2 (FILTER_OK). Unknown powers are empty.
        self._meas_writer.writerow(
            (
                "host_time_iso", "host_elapsed_s", "tag_time_ms", "boot_id",
                "meas_seq", "meas_time_us", "anchor_id", "txn", "mode", "mode_text",
                "flags_hex", "status_hex", "status_text", "raw_mm", "corrected_mm",
                "filtered_mm", "fp_cdbm", "rx_cdbm", "nlos_db", "anchor_fp_cdbm",
                "anchor_rx_cdbm", "std_noise", "fp_index", "ci_ppm_x100", "slot_us",
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
        # Every second, one row per anchor A1..A8, with or without data.
        self._timeline_file = self._open_text("anchor_timeline.csv")
        self._timeline_writer = csv.DictWriter(self._timeline_file, TIMELINE_COLUMNS)
        self._timeline_writer.writeheader()
        self._timeline = AnchorTimeline(started_wall, self._timeline_writer.writerow)
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
        message: DecodedMessage,
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
            try:
                self._finish_timeline(time.monotonic() - self.started_monotonic)
            except (OSError, ValueError) as exc:
                if self.error is None:
                    self.error = str(exc)
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
        message: DecodedMessage,
        host_filtered_mm: dict[int, int],
    ) -> None:
        self._timeline.add_message(message, host_elapsed)
        if isinstance(message, RangeMeasMessage):
            nlos_db = message.nlos_indicator_db
            self._meas_writer.writerow(
                (
                    host_time, f"{host_elapsed:.6f}", message.time_ms, message.boot_id,
                    message.meas_seq, message.meas_time_us, message.anchor_id, message.txn,
                    message.mode, message.mode_name, f"0x{message.flags:04X}",
                    f"0x{message.status:02X}", "|".join(status_names(message.status)),
                    message.raw_mm, message.corrected_mm, message.filtered_mm,
                    _power(message.fp_cdbm), _power(message.rx_cdbm),
                    "" if nlos_db is None else f"{nlos_db:.2f}",
                    _power(message.anchor_fp_cdbm), _power(message.anchor_rx_cdbm),
                    message.std_noise, message.fp_index, message.ci_ppm_x100, message.slot_us,
                )
            )
            self.meas_records += 1
            span = self._meas_spans.get(message.boot_id)
            if span is None:
                self._meas_spans[message.boot_id] = [message.meas_seq, message.meas_seq, 1]
            else:
                span[0] = min(span[0], message.meas_seq)
                span[1] = max(span[1], message.meas_seq)
                span[2] += 1
        elif isinstance(message, RangeMessage):
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
        else:
            self._write_generic(host_time, host_elapsed, message)

    def _write_generic(self, host_time: str, host_elapsed: float, message: object) -> None:
        name = _file_name_for(message)
        row = {
            "host_time_iso": host_time,
            "host_elapsed_s": f"{host_elapsed:.6f}",
            **_flatten(message),
        }
        writer = self._generic_writers.get(name)
        if writer is None:
            handle = self._open_text(name)
            self._generic_files[name] = handle
            writer = csv.DictWriter(handle, list(row), extrasaction="ignore")
            writer.writeheader()
            self._generic_writers[name] = writer
        writer.writerow(row)
        self.message_counts[name] = self.message_counts.get(name, 0) + 1

    def _finish_timeline(self, elapsed_s: float) -> None:
        """Close the last timeline second and write the per-anchor summary."""
        self._timeline.finish(elapsed_s)
        with self._open_text("summary.csv") as handle:
            writer = csv.DictWriter(handle, SUMMARY_COLUMNS)
            writer.writeheader()
            writer.writerows(self._timeline.summary_rows())
        self.summary_written = True

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
        previous, self._uart_previous = self._uart_previous, counters
        # The first snapshot is the baseline; a smaller count is a new connection.
        if previous is not None:
            if counters.bytes_received < previous.bytes_received:
                previous = ParserCounters()
            self.uart_bytes += counters.bytes_received - previous.bytes_received
            self.uart_discarded += counters.discarded_bytes - previous.discarded_bytes
            self.uart_crc_errors += counters.crc_errors - previous.crc_errors
        # Arrives every second while connected: closes timeline seconds even
        # when no anchor answers.
        self._timeline.advance(host_elapsed)

    def _handles(self) -> tuple:
        return (
            self._range_file, self._meas_file, self._stats_file, self._uart_file,
            self._info_file, self._event_file, self._raw_file, self._timeline_file,
            *self._generic_files.values(),
        )

    def _flush(self) -> None:
        for handle in self._handles():
            handle.flush()

    def _flush_and_close(self) -> None:
        for handle in self._handles():
            try:
                handle.flush()
                handle.close()
            except (OSError, ValueError):
                pass

    def _link_quality(self) -> dict[str, object]:
        """How much the UART lost: parser discards and RANGE_MEAS gaps."""
        expected = sum(last - first + 1 for first, last, _ in self._meas_spans.values())
        received = sum(count for _, _, count in self._meas_spans.values())
        return {
            "uart_bytes_received": self.uart_bytes,
            "uart_discarded_bytes": self.uart_discarded,
            "uart_crc_errors": self.uart_crc_errors,
            "uart_discarded_pct": (round(100.0 * self.uart_discarded / self.uart_bytes, 2)
                                   if self.uart_bytes else None),
            "meas_seq_expected": expected,
            "meas_delivered_pct": round(100.0 * received / expected, 2) if expected else None,
        }

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
            "meas_records": self.meas_records,
            "info_frames": self.info_frames,
            "stats_frames": self.stats_frames,
            "uart_snapshots": self.uart_snapshots,
            "raw_frames": self.raw_frames,
            "raw_bytes": self.raw_bytes,
            "event_lines": self.event_lines,
            "message_counts": dict(sorted(self.message_counts.items())),
            "anchor_timeline_rows": self._timeline.rows_written,
            "summary_written": self.summary_written,
            **self._link_quality(),
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
