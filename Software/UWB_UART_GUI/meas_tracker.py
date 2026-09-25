"""Per-anchor state of the RANGE_MEAS (TYPE 0x10) stream.

The TAG sends one RANGE_MEAS record per radio-successful measurement, so this
stream shows the real ranging rate, lost records and the radio diagnostics of
every single exchange (the 50 Hz snapshot only keeps the last one per anchor).
Pure logic without Tk, so the GUI view stays thin and this is unit-tested.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import statistics

from telemetry_protocol import (
    INT16_MIN,
    MEAS_FLAG_CAL_OK,
    MEAS_FLAG_FILTER_OK,
    RangeMeasMessage,
)


RATE_WINDOW_S = 2.0      # rate, noise and NLOS mean use the last 2 s
STALE_AFTER_S = 1.0      # no record for 1 s: the anchor row is stale
HISTORY_POINTS = 4000    # per anchor for the plot (> 60 s at 50 Hz)
NOISE_MIN_DIFFERENCES = 5
# DW1000 APS006 Part 3: RX level minus first-path power below ~6 dB is
# likely line of sight, above ~10 dB likely non line of sight.
NLOS_WARN_DB = 6.0
NLOS_ALERT_DB = 10.0


def power_dbm(cdbm: int) -> float | None:
    """Centi-dBm to dBm; INT16_MIN marks a power the TAG did not measure."""
    return None if cdbm == INT16_MIN else cdbm / 100.0


def difference_noise_mm(raws: list[int]) -> float | None:
    """Range noise from consecutive raw differences: std(diff) / sqrt(2).

    For white noise this equals the plain standard deviation, but a moving
    TAG adds only its (small) change of speed, not the whole trajectory, so
    the number stays meaningful while the drone moves.
    """
    differences = [after - before for before, after in zip(raws, raws[1:])]
    if len(differences) < NOISE_MIN_DIFFERENCES:
        return None
    return statistics.pstdev(differences) / math.sqrt(2.0)


def anchor_nlos_db(message: RangeMeasMessage) -> float | None:
    """RX minus FP of the FINAL at the anchor (DS-TWR REPORT), if present."""
    if INT16_MIN in (message.anchor_fp_cdbm, message.anchor_rx_cdbm):
        return None
    return (message.anchor_rx_cdbm - message.anchor_fp_cdbm) / 100.0


@dataclass(frozen=True)
class MeasPoint:
    tag_time_s: float
    raw_mm: int
    corrected_mm: int | None     # only with MEAS_FLAG_CAL_OK
    filtered_mm: int | None      # only with MEAS_FLAG_FILTER_OK
    nlos_db: float | None


@dataclass(frozen=True)
class MeasAnchorSummary:
    anchor_id: int
    latest: RangeMeasMessage
    age_s: float
    rate_hz: float | None        # None while the anchor has <0.5 s of data
    noise_mm: float | None       # see difference_noise_mm()
    nlos_mean_db: float | None
    received: int
    stale: bool


class RangeMeasTracker:
    """Rate, loss, noise and plot history of the RANGE_MEAS records."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.latest: dict[int, RangeMeasMessage] = {}
        self.received: dict[int, int] = {}
        self._last_received_s: dict[int, float] = {}
        self._first_received_s: dict[int, float] = {}
        # (host receive time, raw_mm, nlos_db) inside RATE_WINDOW_S
        self._window: dict[int, deque[tuple[float, int, float | None]]] = {}
        self.history: dict[int, deque[MeasPoint]] = {}
        self.total = 0
        self.lost = 0            # records missing from the meas_seq sequence
        self.restarts = 0        # TAG reboots seen through boot_id
        self.boot_id: int | None = None
        self._last_sequence: int | None = None

    def restart_sequence(self) -> None:
        """Forget meas_seq continuity. The TAG keeps counting while RANGE_MEAS
        is off, so re-enabling it must not be reported as lost records."""
        self._last_sequence = None

    def ingest(self, message: RangeMeasMessage, received_s: float) -> None:
        if self.boot_id is not None and message.boot_id != self.boot_id:
            # A new TAG boot restarts meas_seq and the TAG clock.
            self.restarts += 1
            self._last_sequence = None
            for history in self.history.values():
                history.clear()
        self.boot_id = message.boot_id

        # meas_seq is one counter over all anchors: a gap is a record dropped
        # by the TAG queue (DIAG meas_queue_drops) or lost on the UART.
        if self._last_sequence is not None:
            delta = (message.meas_seq - self._last_sequence) & 0xFFFFFFFF
            if 1 < delta < 0x80000000:
                self.lost += delta - 1
        self._last_sequence = message.meas_seq

        anchor_id = message.anchor_id
        self.total += 1
        self.received[anchor_id] = self.received.get(anchor_id, 0) + 1
        self.latest[anchor_id] = message
        self._last_received_s[anchor_id] = received_s
        self._first_received_s.setdefault(anchor_id, received_s)

        nlos_db = message.nlos_indicator_db
        window = self._window.setdefault(anchor_id, deque())
        window.append((received_s, message.raw_mm, nlos_db))
        self._prune(window, received_s)

        self.history.setdefault(anchor_id, deque(maxlen=HISTORY_POINTS)).append(MeasPoint(
            tag_time_s=message.meas_time_us / 1e6,
            raw_mm=message.raw_mm,
            corrected_mm=message.corrected_mm if message.flags & MEAS_FLAG_CAL_OK else None,
            filtered_mm=message.filtered_mm if message.flags & MEAS_FLAG_FILTER_OK else None,
            nlos_db=nlos_db,
        ))

    @staticmethod
    def _prune(window: deque[tuple[float, int, float | None]], now_s: float) -> None:
        while window and window[0][0] < now_s - RATE_WINDOW_S:
            window.popleft()

    def summaries(self, now_s: float) -> list[MeasAnchorSummary]:
        result = []
        for anchor_id in sorted(self.latest):
            window = self._window[anchor_id]
            self._prune(window, now_s)
            span_s = min(RATE_WINDOW_S, now_s - self._first_received_s[anchor_id])
            rate_hz = len(window) / span_s if span_s >= 0.5 else None
            nlos = [value for _, _, value in window if value is not None]
            age_s = max(0.0, now_s - self._last_received_s[anchor_id])
            result.append(MeasAnchorSummary(
                anchor_id=anchor_id,
                latest=self.latest[anchor_id],
                age_s=age_s,
                rate_hz=rate_hz,
                noise_mm=difference_noise_mm([raw for _, raw, _ in window]),
                nlos_mean_db=statistics.fmean(nlos) if nlos else None,
                received=self.received[anchor_id],
                stale=age_s > STALE_AFTER_S,
            ))
        return result

    @staticmethod
    def total_rate(summaries: list[MeasAnchorSummary]) -> float:
        return sum(item.rate_hz or 0.0 for item in summaries)

    def points(self, anchor_id: int, span_s: float) -> list[MeasPoint]:
        """The last span_s seconds of TAG time for one anchor."""
        history = self.history.get(anchor_id)
        if not history:
            return []
        start = history[-1].tag_time_s - span_s
        points = []
        for point in reversed(history):
            if point.tag_time_s < start:
                break
            points.append(point)
        points.reverse()
        return points
