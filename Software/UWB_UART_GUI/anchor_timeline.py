"""Per-anchor aggregation for recorded sessions.

The recorder feeds every decoded message here from its writer thread. Two
outputs come from it:

* ``anchor_timeline.csv``: one row per anchor for every second of the
  session, including anchors that sent nothing, so a gap is a row of zeros
  instead of a missing line;
* ``summary.csv``: one row per anchor for the whole session, written when the
  recording stops.

Only running sums are kept (no per-sample lists), so memory stays constant
over long sessions. The module has no Tk or serial dependency.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timedelta
import math
from typing import Callable, Iterable

from telemetry_protocol import (
    INT16_MIN,
    AnchorInfoMessage,
    DiagAnchorMessage,
    RangeMeasMessage,
    RangeMessage,
    STATUS_RANGE_REJECT,
    STATUS_TIMEOUT,
    status_names,
)


ANCHOR_IDS = tuple(range(1, 9))
MEAS_MODE_DS = 0
MEAS_MODE_SS = 1
MEAS_MODE_SS_FALLBACK = 2
# Seconds without traffic that are still written as empty rows; a longer
# silence (e.g. the PC slept) is closed with a single jump.
MAX_EMPTY_WINDOWS = 3600

# DIAG_ANCHOR counters are cumulative since TAG boot.
DIAG_COUNTERS = (
    "success", "resp_timeouts", "poll_tx_timeouts", "rx_errors", "poll_skipped",
    "report_timeouts", "ds_fallbacks", "txn_mismatch", "probes",
)

TIMELINE_COLUMNS = (
    "host_time_iso", "window_start_s", "window_end_s", "anchor_id",
    "meas_count", "meas_hz", "ds_count", "ss_count", "ss_fallback_count",
    "reject_count", "raw_mean_mm", "raw_std_mm", "raw_min_mm", "raw_max_mm",
    "fp_dbm_mean", "rx_dbm_mean", "nlos_db_mean", "anchor_fp_dbm_mean",
    "anchor_rx_dbm_mean", "ci_ppm_mean", "slot_us_mean", "slot_us_max",
    "snap_frames", "snap_valid", "snap_timeout", "last_status_text", "last_age_ms",
    "diag_active", "diag_backed_off",
    *(f"diag_{name}" for name in DIAG_COUNTERS),
)

SUMMARY_COLUMNS = (
    "anchor_id", "duration_s", "meas_count", "meas_hz", "ds_count", "ss_count",
    "ss_fallback_count", "ds_pct", "reject_count", "first_meas_s", "last_meas_s",
    "longest_gap_s", "raw_mean_mm", "raw_std_mm", "raw_min_mm", "raw_max_mm",
    "fp_dbm_mean", "fp_dbm_min", "rx_dbm_mean", "nlos_db_mean", "nlos_db_max",
    "anchor_fp_dbm_mean", "anchor_rx_dbm_mean", "ci_ppm_mean", "ci_ppm_min",
    "ci_ppm_max", "slot_us_mean", "slot_us_max", "snap_frames", "snap_valid_pct",
    "snap_timeout_pct", "diag_frames", "diag_backed_off_last",
    *(f"{name}_total" for name in DIAG_COUNTERS),
    *(f"{name}_delta" for name in DIAG_COUNTERS),
    "slot_max_us", "resp_wait_max_us", "anchor_info_frames", "anchor_build_hash",
    "anchor_build_dirty", "anchor_config_hash", "anchor_boot_first",
    "anchor_boot_last", "anchor_tx_ant_dly", "anchor_rx_ant_dly",
    "anchor_tx_power_mode",
)


@dataclass
class RunningStats:
    """Welford mean/variance with min and max."""

    count: int = 0
    mean: float = 0.0
    m2: float = 0.0
    minimum: float = math.inf
    maximum: float = -math.inf

    def add(self, value: float) -> None:
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        self.m2 += delta * (value - self.mean)
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)

    @property
    def std(self) -> float | None:
        return math.sqrt(self.m2 / (self.count - 1)) if self.count >= 2 else None


@dataclass
class _MeasStats:
    """RANGE_MEAS and RANGE snapshot figures over one interval."""

    count: int = 0
    ds: int = 0
    ss: int = 0
    ss_fallback: int = 0
    reject: int = 0
    raw_mm: RunningStats = field(default_factory=RunningStats)
    fp_dbm: RunningStats = field(default_factory=RunningStats)
    rx_dbm: RunningStats = field(default_factory=RunningStats)
    nlos_db: RunningStats = field(default_factory=RunningStats)
    anchor_fp_dbm: RunningStats = field(default_factory=RunningStats)
    anchor_rx_dbm: RunningStats = field(default_factory=RunningStats)
    ci_ppm: RunningStats = field(default_factory=RunningStats)
    slot_us: RunningStats = field(default_factory=RunningStats)
    snap_frames: int = 0
    snap_valid: int = 0
    snap_timeout: int = 0

    def add_meas(self, message: RangeMeasMessage) -> None:
        self.count += 1
        if message.mode == MEAS_MODE_DS:
            self.ds += 1
        elif message.mode == MEAS_MODE_SS:
            self.ss += 1
        elif message.mode == MEAS_MODE_SS_FALLBACK:
            self.ss_fallback += 1
        if message.status & STATUS_RANGE_REJECT:
            self.reject += 1
        self.raw_mm.add(float(message.raw_mm))
        _add_power(self.fp_dbm, message.fp_cdbm)
        _add_power(self.rx_dbm, message.rx_cdbm)
        _add_power(self.anchor_fp_dbm, message.anchor_fp_cdbm)
        _add_power(self.anchor_rx_dbm, message.anchor_rx_cdbm)
        nlos = message.nlos_indicator_db
        if nlos is not None:
            self.nlos_db.add(nlos)
        self.ci_ppm.add(message.ci_ppm_x100 / 100.0)
        self.slot_us.add(float(message.slot_us))

    def add_snapshot(self, valid: bool, status: int) -> None:
        self.snap_frames += 1
        if valid:
            self.snap_valid += 1
        if status & STATUS_TIMEOUT:
            self.snap_timeout += 1


@dataclass
class _AnchorState:
    window: _MeasStats = field(default_factory=_MeasStats)
    total: _MeasStats = field(default_factory=_MeasStats)
    first_meas_s: float | None = None
    last_meas_s: float | None = None
    longest_gap_s: float = 0.0
    last_status: int | None = None
    last_age_ms: int | None = None
    first_diag: DiagAnchorMessage | None = None
    last_diag: DiagAnchorMessage | None = None
    diag_frames: int = 0
    first_info: AnchorInfoMessage | None = None
    last_info: AnchorInfoMessage | None = None
    info_frames: int = 0


def _add_power(stats: RunningStats, cdbm: int) -> None:
    if cdbm != INT16_MIN:
        stats.add(cdbm / 100.0)


def _fmt(value: float | None, digits: int = 1) -> str:
    if value is None or not math.isfinite(value):
        return ""
    return f"{value:.{digits}f}"


def _mean(stats: RunningStats, digits: int = 1) -> str:
    return _fmt(stats.mean if stats.count else None, digits)


def _minimum(stats: RunningStats, digits: int = 1) -> str:
    return _fmt(stats.minimum if stats.count else None, digits)


def _maximum(stats: RunningStats, digits: int = 1) -> str:
    return _fmt(stats.maximum if stats.count else None, digits)


def _pct(part: int, whole: int) -> str:
    return _fmt(100.0 * part / whole if whole else None, 1)


class AnchorTimeline:
    """Aggregate per-anchor figures into 1 s rows and a session summary.

    ``emit_row`` receives one dict per anchor and second, keyed by
    ``TIMELINE_COLUMNS``.
    """

    def __init__(
        self,
        started_wall: datetime,
        emit_row: Callable[[dict[str, object]], None],
        window_s: float = 1.0,
    ) -> None:
        self.started_wall = started_wall
        self.window_s = window_s
        self._emit_row = emit_row
        self._window_start = 0.0
        self._anchors: dict[int, _AnchorState] = {
            anchor_id: _AnchorState() for anchor_id in ANCHOR_IDS
        }
        self.rows_written = 0
        self.elapsed_s = 0.0

    def _anchor(self, anchor_id: int) -> _AnchorState:
        state = self._anchors.get(anchor_id)
        if state is None:
            state = self._anchors[anchor_id] = _AnchorState()
        return state

    # -- input -------------------------------------------------------------

    def advance(self, elapsed_s: float) -> None:
        """Close every whole window that ended before ``elapsed_s``."""
        self.elapsed_s = max(self.elapsed_s, elapsed_s)
        empty = 0
        while elapsed_s >= self._window_start + self.window_s:
            self._close_window(self._window_start + self.window_s)
            empty += 1
            if empty >= MAX_EMPTY_WINDOWS:
                skipped = math.floor((elapsed_s - self._window_start) / self.window_s)
                self._window_start += skipped * self.window_s
                break

    def add_message(self, message: object, elapsed_s: float) -> None:
        self.advance(elapsed_s)
        if isinstance(message, RangeMeasMessage):
            state = self._anchor(message.anchor_id)
            state.window.add_meas(message)
            state.total.add_meas(message)
            if state.last_meas_s is not None:
                state.longest_gap_s = max(state.longest_gap_s, elapsed_s - state.last_meas_s)
            if state.first_meas_s is None:
                state.first_meas_s = elapsed_s
            state.last_meas_s = elapsed_s
        elif isinstance(message, RangeMessage):
            for sample in message.samples:
                state = self._anchor(sample.anchor_id)
                state.window.add_snapshot(sample.valid, sample.status)
                state.total.add_snapshot(sample.valid, sample.status)
                state.last_status = sample.status
                state.last_age_ms = sample.age_ms
        elif isinstance(message, DiagAnchorMessage):
            state = self._anchor(message.anchor_id)
            if state.first_diag is None:
                state.first_diag = message
            state.last_diag = message
            state.diag_frames += 1
        elif isinstance(message, AnchorInfoMessage):
            state = self._anchor(message.anchor_id)
            if state.first_info is None:
                state.first_info = message
            state.last_info = message
            state.info_frames += 1

    def finish(self, elapsed_s: float) -> None:
        """Close the whole windows and the final partial one. A session
        shorter than the clock resolution still gets its one window."""
        self.advance(elapsed_s)
        if elapsed_s > self._window_start or self.rows_written == 0:
            self._close_window(max(elapsed_s, self._window_start))

    # -- output ------------------------------------------------------------

    def _close_window(self, window_end: float) -> None:
        duration = window_end - self._window_start
        wall = (self.started_wall + timedelta(seconds=window_end)).isoformat(
            timespec="milliseconds")
        for anchor_id in sorted(self._anchors):
            state = self._anchors[anchor_id]
            window = state.window
            diag = state.last_diag
            row: dict[str, object] = {
                "host_time_iso": wall,
                "window_start_s": f"{self._window_start:.3f}",
                "window_end_s": f"{window_end:.3f}",
                "anchor_id": anchor_id,
                "meas_count": window.count,
                "meas_hz": _fmt(window.count / duration if duration > 0 else None, 1),
                "ds_count": window.ds,
                "ss_count": window.ss,
                "ss_fallback_count": window.ss_fallback,
                "reject_count": window.reject,
                "raw_mean_mm": _mean(window.raw_mm),
                "raw_std_mm": _fmt(window.raw_mm.std),
                "raw_min_mm": _minimum(window.raw_mm, 0),
                "raw_max_mm": _maximum(window.raw_mm, 0),
                "fp_dbm_mean": _mean(window.fp_dbm, 2),
                "rx_dbm_mean": _mean(window.rx_dbm, 2),
                "nlos_db_mean": _mean(window.nlos_db, 2),
                "anchor_fp_dbm_mean": _mean(window.anchor_fp_dbm, 2),
                "anchor_rx_dbm_mean": _mean(window.anchor_rx_dbm, 2),
                "ci_ppm_mean": _mean(window.ci_ppm, 2),
                "slot_us_mean": _mean(window.slot_us, 0),
                "slot_us_max": _maximum(window.slot_us, 0),
                "snap_frames": window.snap_frames,
                "snap_valid": window.snap_valid,
                "snap_timeout": window.snap_timeout,
                "last_status_text": ("" if state.last_status is None
                                     else "|".join(status_names(state.last_status))),
                "last_age_ms": "" if state.last_age_ms is None else state.last_age_ms,
                "diag_active": "" if diag is None else int(diag.active),
                "diag_backed_off": "" if diag is None else int(diag.backed_off),
            }
            for name in DIAG_COUNTERS:
                row[f"diag_{name}"] = "" if diag is None else getattr(diag, name)
            self._emit_row(row)
            self.rows_written += 1
            state.window = _MeasStats()
        self._window_start = window_end

    def summary_rows(self) -> Iterable[dict[str, object]]:
        duration = self.elapsed_s
        for anchor_id in sorted(self._anchors):
            state = self._anchors[anchor_id]
            total = state.total
            first, last = state.first_diag, state.last_diag
            info_first, info = state.first_info, state.last_info
            row: dict[str, object] = {
                "anchor_id": anchor_id,
                "duration_s": _fmt(duration, 1),
                "meas_count": total.count,
                "meas_hz": _fmt(total.count / duration if duration > 0 else None, 2),
                "ds_count": total.ds,
                "ss_count": total.ss,
                "ss_fallback_count": total.ss_fallback,
                "ds_pct": _pct(total.ds, total.count),
                "reject_count": total.reject,
                "first_meas_s": _fmt(state.first_meas_s, 3),
                "last_meas_s": _fmt(state.last_meas_s, 3),
                "longest_gap_s": _fmt(state.longest_gap_s if total.count >= 2 else None, 3),
                "raw_mean_mm": _mean(total.raw_mm),
                "raw_std_mm": _fmt(total.raw_mm.std),
                "raw_min_mm": _minimum(total.raw_mm, 0),
                "raw_max_mm": _maximum(total.raw_mm, 0),
                "fp_dbm_mean": _mean(total.fp_dbm, 2),
                "fp_dbm_min": _minimum(total.fp_dbm, 2),
                "rx_dbm_mean": _mean(total.rx_dbm, 2),
                "nlos_db_mean": _mean(total.nlos_db, 2),
                "nlos_db_max": _maximum(total.nlos_db, 2),
                "anchor_fp_dbm_mean": _mean(total.anchor_fp_dbm, 2),
                "anchor_rx_dbm_mean": _mean(total.anchor_rx_dbm, 2),
                "ci_ppm_mean": _mean(total.ci_ppm, 2),
                "ci_ppm_min": _minimum(total.ci_ppm, 2),
                "ci_ppm_max": _maximum(total.ci_ppm, 2),
                "slot_us_mean": _mean(total.slot_us, 0),
                "slot_us_max": _maximum(total.slot_us, 0),
                "snap_frames": total.snap_frames,
                "snap_valid_pct": _pct(total.snap_valid, total.snap_frames),
                "snap_timeout_pct": _pct(total.snap_timeout, total.snap_frames),
                "diag_frames": state.diag_frames,
                "diag_backed_off_last": "" if last is None else int(last.backed_off),
                "slot_max_us": "" if last is None else last.slot_max_us,
                "resp_wait_max_us": "" if last is None else last.resp_wait_max_us,
                "anchor_info_frames": state.info_frames,
                "anchor_build_hash": "" if info is None else f"0x{info.build_hash:08X}",
                "anchor_build_dirty": "" if info is None else int(info.build_dirty),
                "anchor_config_hash": "" if info is None else f"0x{info.build_config_hash:08X}",
                "anchor_boot_first": "" if info_first is None else info_first.boot_count,
                "anchor_boot_last": "" if info is None else info.boot_count,
                "anchor_tx_ant_dly": "" if info is None else info.tx_antenna_delay,
                "anchor_rx_ant_dly": "" if info is None else info.rx_antenna_delay,
                "anchor_tx_power_mode": "" if info is None else info.tx_power_mode,
            }
            for name in DIAG_COUNTERS:
                row[f"{name}_total"] = "" if last is None else getattr(last, name)
                delta = ""
                if first is not None and last is not None and first is not last:
                    change = getattr(last, name) - getattr(first, name)
                    # Negative: the TAG rebooted in between; no valid delta.
                    delta = change if change >= 0 else ""
                row[f"{name}_delta"] = delta
            yield row
