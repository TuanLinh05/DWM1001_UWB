"""Radio-loss report for a recorded New_GUI session (G0.2) — SKELETON.

Plan: Plan/G0_KE_HOACH_HIEU_CHUAN_DA_DIEM_VA_DU_LIEU_CHUAN.md (section G0.2).
Contract: tests/test_g0_radio_loss_report.py.

    python tools/radio_loss_report.py logs/<session> [logs/<session> ...] [--bin-s 5] [--outage-s 0.2] [--out DIR]

tools/diag_summary.py gives whole-session totals. This tool answers WHEN and
HOW ranges go missing: per-anchor outages, anchors per cycle, time bins with
TAG temperature/voltage, lost cycle numbers (UART) and a loss-pattern verdict
(one anchor, all anchors together, or independent failures).

Inputs (schema-5 session folder; depend ONLY on these columns/fields):
  uwb_fast_log.csv   tag_time_us, boot_id, cycle_seq, period_us,
                     a<N>_mm (empty = no range), a<N>_status_code (empty = not
                     polled: TAG skipped the anchor), a<N>_flags (bit 0x10 = late)
  rx_quality.jsonl   {"message": {"t_us", "temperature_c", "vbat_v",
                     "anchors": [{"anchor_id", "flags"}]}} (flags bit 0 = RESP ok); optional
A CSV without a<N>_status_code columns (old schema): treat every anchor as polled.

Outputs: <out>/radio_loss_report.json (strict JSON: allow_nan=False) and
<out>/radio_loss_bins.csv. Default <out> is the session folder; with --out DIR
the files are named "<session>_radio_loss_report.json" / "<session>_radio_loss_bins.csv".
main() returns 0.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import sys
from typing import Sequence

import numpy as np

N_ANCHORS = 8
LATE_FLAG = 0x10
FAIL_RATE_THRESHOLD = 0.05   # an anchor "loses ranges" when it fails >= 5 % of its polled cycles


@dataclass(frozen=True)
class QualityEvent:
    tag_time_s: float
    temperature_c: float | None
    vbat_v: float | None
    resp_ok: dict[int, bool]


@dataclass
class SessionData:
    name: str                    # session folder name
    tag_time_s: np.ndarray       # (N,) TAG time of each CSV row
    boot_id: np.ndarray          # (N,) int
    cycle_seq: np.ndarray        # (N,) int
    period_us: np.ndarray        # (N,) float, nan when missing
    range_ok: np.ndarray         # (N, 8) bool, a<N>_mm non-empty
    polled: np.ndarray           # (N, 8) bool, a<N>_status_code non-empty
    late: np.ndarray             # (N, 8) bool, a<N>_flags & 0x10
    has_status_columns: bool
    quality: list[QualityEvent] = field(default_factory=list)


@dataclass(frozen=True)
class AnchorLossStats:
    anchor_id: int
    polled_pct: float                         # polled cycles / all cycles
    range_ok_pct: float                       # cycles with a range / all cycles
    range_ok_given_polled_pct: float | None   # None when never polled
    resp_ok_given_polled_pct: float | None    # from rx_quality flags bit 0; None without rx_quality
    late_pct: float | None                    # late among polled cycles
    outages: int                              # outages lasting >= outage_threshold_s
    longest_outage_s: float


@dataclass(frozen=True)
class TimeBin:
    t_start_s: float
    t_end_s: float
    cycles: int
    mean_anchors_per_cycle: float
    anchor_ok_pct: dict[int, float]
    temperature_c: float | None   # mean of rx_quality events inside the bin
    vbat_v: float | None
    mean_period_ms: float | None


@dataclass(frozen=True)
class LossPattern:
    pattern: str                      # "none" | "anchor_specific" | "common_cause" | "independent" | "mixed"
    failure_rates: dict[int, float]   # failed polled cycles / polled cycles, per anchor
    co_failure_lift: float | None
    worst_anchor_ids: tuple[int, ...]  # anchors with rate >= FAIL_RATE_THRESHOLD, worst first
    notes: tuple[str, ...]            # plain-language hints for the user


def load_session(path: Path) -> SessionData:
    """Read a session folder (or its uwb_fast_log.csv) into arrays. Close every file you open."""
    raise NotImplementedError("G0.2: load_session")


def anchors_per_cycle_histogram(data: SessionData) -> list[int]:
    """Counts of cycles with 0..8 ranges (length 9)."""
    raise NotImplementedError("G0.2: anchors_per_cycle_histogram")


def lost_cycles(data: SessionData) -> int:
    """Sum of (cycle_seq jump - 1) between consecutive rows of the same boot_id (UART/host loss)."""
    raise NotImplementedError("G0.2: lost_cycles")


def anchor_stats(data: SessionData, outage_threshold_s: float = 0.2) -> dict[int, AnchorLossStats]:
    """Per-anchor statistics for anchors 1..8.

    Outage = maximal run of rows without a range. Duration = TAG time of the
    first range after the run minus TAG time of the last range before it; a run
    at the start (end) of the session is measured from the first (to the last) row.
    Example: no range for 2.0 <= t < 3.5 s at 4 ms per cycle -> 3.500 - 1.996 = 1.504 s.
    """
    raise NotImplementedError("G0.2: anchor_stats")


def time_bins(data: SessionData, bin_s: float = 5.0) -> list[TimeBin]:
    """Fixed bins of bin_s seconds starting at the first row; skip empty bins."""
    raise NotImplementedError("G0.2: time_bins")


def classify_loss(data: SessionData) -> LossPattern:
    """Classify how ranges are lost.

    rate_a = failed polled cycles / polled cycles; bad = anchors with rate >= 0.05.
    lift = mean over anchor pairs (both rates >= 0.01) of
           P(a and b fail | both polled) / (P(a fails) * P(b fails)), None if no pair.
    pattern: "none" if no bad anchor; "anchor_specific" if 1-2 bad anchors;
             "common_cause" if >= 3 bad and lift >= 1.5;
             "independent" if >= 3 bad and lift < 1.2; else "mixed".
    """
    raise NotImplementedError("G0.2: classify_loss")


def temperature_correlation(bins: Sequence[TimeBin]) -> float | None:
    """Pearson r between bin temperature and mean anchors per cycle; None with < 3 bins or zero variance."""
    raise NotImplementedError("G0.2: temperature_correlation")


def build_report(path: Path, bin_s: float = 5.0, outage_threshold_s: float = 0.2) -> dict:
    """JSON-ready report (string keys, no nan/inf) with keys: tool, version, session,
    cycles, duration_s, mean_anchors_per_cycle, share_cycles_ge {"4","6","7","8"},
    histogram, lost_cycles, has_status_columns, has_rx_quality,
    anchors {"<aid>": AnchorLossStats fields}, bins [TimeBin fields],
    pattern {LossPattern fields}, temperature_correlation."""
    raise NotImplementedError("G0.2: build_report")


def main(argv: Sequence[str] | None = None) -> int:
    raise NotImplementedError("G0.2: main")


if __name__ == "__main__":
    sys.exit(main())
