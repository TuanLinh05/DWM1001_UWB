"""Radio-loss report for a recorded New_GUI session (G0.2).

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

from dataclasses import asdict, dataclass, field
import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Sequence

import numpy as np
from report_utils import number, strict_value

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
    path = Path(path)
    csv_path = path / "uwb_fast_log.csv" if path.is_dir() else path
    times, boots, seqs, periods, ranges, polls, lates = [], [], [], [], [], [], []
    with csv_path.open(encoding="utf-8-sig", newline="") as fh:
        reader = csv.DictReader(fh)
        columns = set(reader.fieldnames or ())
        has_status = any(f"a{a}_status_code" in columns for a in range(1, N_ANCHORS + 1))
        for row in reader:
            timestamp = number(row.get("tag_time_us"))
            if timestamp is None:
                raise ValueError(f"Invalid tag_time_us at CSV line {reader.line_num}")
            times.append(timestamp / 1e6)
            boots.append(int(number(row.get("boot_id"), 0)))
            seqs.append(int(number(row.get("cycle_seq"), 0)))
            periods.append(number(row.get("period_us"), float("nan")))
            ranges.append([number(row.get(f"a{a}_mm"), 0) > 10 for a in range(1, N_ANCHORS + 1)])
            polls.append([bool(row.get(f"a{a}_status_code", "").strip()) if has_status else True
                          for a in range(1, N_ANCHORS + 1)])
            lates.append([bool(int(number(row.get(f"a{a}_flags"), 0)) & LATE_FLAG)
                          for a in range(1, N_ANCHORS + 1)])
    data = SessionData(csv_path.parent.name, np.asarray(times), np.asarray(boots, dtype=np.int64),
                       np.asarray(seqs, dtype=np.int64), np.asarray(periods),
                       np.asarray(ranges, dtype=bool).reshape((-1, N_ANCHORS)),
                       np.asarray(polls, dtype=bool).reshape((-1, N_ANCHORS)),
                       np.asarray(lates, dtype=bool).reshape((-1, N_ANCHORS)), has_status)
    quality_path = csv_path.parent / "rx_quality.jsonl"
    if quality_path.is_file():
        with quality_path.open(encoding="utf-8-sig") as fh:
            for line_number, line in enumerate(fh, 1):
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line)["message"]
                    timestamp = number(msg.get("t_us"))
                    if timestamp is None:
                        continue
                    resp = {int(a["anchor_id"]): bool(int(a["flags"]) & 1) for a in msg.get("anchors", [])
                            if 1 <= int(a["anchor_id"]) <= N_ANCHORS}
                    data.quality.append(QualityEvent(timestamp / 1e6, number(msg.get("temperature_c")),
                                                     number(msg.get("vbat_v")), resp))
                except (ValueError, KeyError, TypeError) as exc:
                    raise ValueError(f"Invalid rx_quality.jsonl line {line_number}: {exc}") from exc
    return data


def anchors_per_cycle_histogram(data: SessionData) -> list[int]:
    """Counts of cycles with 0..8 ranges (length 9)."""
    return np.bincount(data.range_ok.sum(axis=1), minlength=N_ANCHORS + 1).tolist()


def lost_cycles(data: SessionData) -> int:
    """Sum of (cycle_seq jump - 1) between consecutive rows of the same boot_id (UART/host loss)."""
    delta = (np.diff(data.cycle_seq) + 2 ** 32) % (2 ** 32)
    same_boot = data.boot_id[1:] == data.boot_id[:-1]
    return int(np.sum(delta[same_boot & (delta > 1) & (delta < 2 ** 31)] - 1))


def anchor_stats(data: SessionData, outage_threshold_s: float = 0.2) -> dict[int, AnchorLossStats]:
    """Per-anchor statistics for anchors 1..8.

    Outage = maximal run of rows without a range. Duration = TAG time of the
    first range after the run minus TAG time of the last range before it; a run
    at the start (end) of the session is measured from the first (to the last) row.
    Example: no range for 2.0 <= t < 3.5 s at 4 ms per cycle -> 3.500 - 1.996 = 1.504 s.
    """
    if not np.isfinite(outage_threshold_s) or outage_threshold_s < 0:
        raise ValueError("outage_threshold_s must be finite and nonnegative")
    n = len(data.tag_time_s)
    boundaries = _boot_boundaries(data)
    stats = {}
    for col in range(N_ANCHORS):
        aid = col + 1
        polled = data.polled[:, col]
        n_polled = int(polled.sum())
        durations = []
        for begin, end in zip(boundaries[:-1], boundaries[1:]):
            missing = ~data.range_ok[begin:end, col]
            edges = np.diff(np.r_[False, missing, False].astype(int))
            for start, stop in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                left = begin + max(0, start - 1)
                right = begin + min(end - begin - 1, stop)
                durations.append(max(0.0, float(data.tag_time_s[right] - data.tag_time_s[left])))
        resp = [q.resp_ok[aid] for q in data.quality if aid in q.resp_ok]
        stats[aid] = AnchorLossStats(aid, 100.0 * n_polled / n if n else 0.0,
            100.0 * int(data.range_ok[:, col].sum()) / n if n else 0.0,
            100.0 * int((data.range_ok[:, col] & polled).sum()) / n_polled if n_polled else None,
            100.0 * sum(resp) / len(resp) if resp else None,
            100.0 * int((data.late[:, col] & polled).sum()) / n_polled if n_polled else None,
            sum(d >= outage_threshold_s for d in durations), max(durations, default=0.0))
    return stats


def _boot_boundaries(data):
    n = len(data.tag_time_s)
    if not n:
        return np.array([0], dtype=int)
    breaks = (np.diff(data.boot_id) != 0) | (np.diff(data.tag_time_s) < 0)
    return np.r_[0, np.flatnonzero(breaks) + 1, n]


def time_bins(data: SessionData, bin_s: float = 5.0) -> list[TimeBin]:
    """Fixed bins of bin_s seconds starting at the first row; skip empty bins."""
    if not np.isfinite(bin_s) or bin_s <= 0:
        raise ValueError("bin_s must be finite and positive")
    bins = []
    boundaries = _boot_boundaries(data)
    for begin, end in zip(boundaries[:-1], boundaries[1:]):
        origin = data.tag_time_s[begin]
        indices = np.floor((data.tag_time_s[begin:end] - origin) / bin_s + 1e-9).astype(np.int64)
        for index in np.unique(indices):
            rows = begin + np.flatnonzero(indices == index)
            start = float(origin + index * bin_s)
            stop = start + bin_s
            quality = [q for q in data.quality if start <= q.tag_time_s < stop]
            temps = [q.temperature_c for q in quality if q.temperature_c is not None]
            volts = [q.vbat_v for q in quality if q.vbat_v is not None]
            periods = data.period_us[rows]
            periods = periods[np.isfinite(periods) & (periods > 0)]
            bins.append(TimeBin(start, stop, len(rows), float(data.range_ok[rows].sum(axis=1).mean()),
                        {a: float(data.range_ok[rows, a - 1].mean() * 100) for a in range(1, N_ANCHORS + 1)},
                        float(np.mean(temps)) if temps else None, float(np.mean(volts)) if volts else None,
                        float(periods.mean() / 1000) if len(periods) else None))
    return bins


def classify_loss(data: SessionData) -> LossPattern:
    """Classify how ranges are lost.

    rate_a = failed polled cycles / polled cycles; bad = anchors with rate >= 0.05.
    lift = mean over anchor pairs (both rates >= 0.01) of
           P(a and b fail | both polled) / (P(a fails) * P(b fails)), None if no pair.
    pattern: "none" if no bad anchor; "anchor_specific" if 1-2 bad anchors;
             "common_cause" if >= 3 bad and lift >= 1.5;
             "independent" if >= 3 bad and lift < 1.2; else "mixed".
    """
    failed = data.polled & ~data.range_ok
    rates = {a: float(failed[:, a - 1].sum() / data.polled[:, a - 1].sum())
             if data.polled[:, a - 1].any() else 0.0 for a in range(1, N_ANCHORS + 1)}
    bad = tuple(sorted((a for a, rate in rates.items() if rate >= FAIL_RATE_THRESHOLD), key=lambda a: (-rates[a], a)))
    lifts = []
    for a in range(N_ANCHORS):
        for b in range(a + 1, N_ANCHORS):
            if rates[a + 1] < 0.01 or rates[b + 1] < 0.01:
                continue
            both = data.polled[:, a] & data.polled[:, b]
            if not both.any():
                continue
            fa, fb = failed[both, a], failed[both, b]
            denominator = float(fa.mean() * fb.mean())
            if denominator > 0:
                lifts.append(float((fa & fb).mean()) / denominator)
    lift = float(np.mean(lifts)) if lifts else None
    if not bad:
        pattern = "none"
        notes = ("No anchor exceeds the 5% conditional range-failure threshold.",)
    elif len(bad) <= 2:
        pattern = "anchor_specific"
        notes = ("Check power, antenna and line of sight at " + ", ".join(f"A{a}" for a in bad) + ".",)
    elif lift is not None and lift >= 1.5:
        pattern = "common_cause"
        notes = ("Anchors fail together: inspect TAG power, temperature, timing and nearby obstructions.",)
    elif lift is not None and lift < 1.2:
        pattern = "independent"
        notes = ("Failures are largely independent: inspect RF margin, distance, TX power and interference.",)
    else:
        pattern = "mixed"
        notes = ("Failures are partly correlated or joint polling is insufficient; inspect TAG and individual links.",)
    if not data.polled.any():
        notes += ("No polling observations are available; radio health cannot be established.",)
    return LossPattern(pattern, rates, lift, bad, notes)


def temperature_correlation(bins: Sequence[TimeBin]) -> float | None:
    """Pearson r between bin temperature and mean anchors per cycle; None with < 3 bins or zero variance."""
    pairs = [(b.temperature_c, b.mean_anchors_per_cycle) for b in bins if b.temperature_c is not None]
    if len(pairs) < 3:
        return None
    temp, quality = np.asarray(pairs).T
    if np.ptp(temp) < 1e-12 or np.ptp(quality) < 1e-12:
        return None
    return float(np.corrcoef(temp, quality)[0, 1])


def build_report(path: Path, bin_s: float = 5.0, outage_threshold_s: float = 0.2) -> dict:
    """JSON-ready report (string keys, no nan/inf) with keys: tool, version, session,
    cycles, duration_s, mean_anchors_per_cycle, share_cycles_ge {"4","6","7","8"},
    histogram, lost_cycles, has_status_columns, has_rx_quality,
    anchors {"<aid>": AnchorLossStats fields}, bins [TimeBin fields],
    pattern {LossPattern fields}, temperature_correlation."""
    data = load_session(path)
    bins = time_bins(data, bin_s)
    n = len(data.tag_time_s)
    counts = data.range_ok.sum(axis=1)
    boundaries = _boot_boundaries(data)
    duration = sum(float(data.tag_time_s[e - 1] - data.tag_time_s[b])
                   for b, e in zip(boundaries[:-1], boundaries[1:]))
    return strict_value({"tool": "radio_loss_report", "version": "g0-radio-loss-v1", "session": data.name,
        "cycles": n, "duration_s": duration, "mean_anchors_per_cycle": float(counts.mean()) if n else 0.0,
        "share_cycles_ge": {str(k): float(np.mean(counts >= k) * 100) if n else 0.0 for k in (4, 6, 7, 8)},
        "histogram": anchors_per_cycle_histogram(data), "lost_cycles": lost_cycles(data),
        "has_status_columns": data.has_status_columns, "has_rx_quality": bool(data.quality),
        "anchors": {a: asdict(s) for a, s in anchor_stats(data, outage_threshold_s).items()},
        "bins": [asdict(b) for b in bins], "pattern": asdict(classify_loss(data)),
        "temperature_correlation": temperature_correlation(bins)})


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", type=Path, nargs="+")
    parser.add_argument("--bin-s", type=float, default=5.0)
    parser.add_argument("--outage-s", type=float, default=0.2)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    try:
        for session in args.sessions:
            report = build_report(session, args.bin_s, args.outage_s)
            out = args.out or (session if session.is_dir() else session.parent)
            out.mkdir(parents=True, exist_ok=True)
            prefix = f"{report['session']}_" if args.out else ""
            with (out / f"{prefix}radio_loss_report.json").open("w", encoding="utf-8") as fh:
                json.dump(report, fh, indent=2, allow_nan=False)
                fh.write("\n")
            fields = ["t_start_s", "t_end_s", "cycles", "mean_anchors_per_cycle", "temperature_c", "vbat_v", "mean_period_ms"]
            fields += [f"a{a}_ok_pct" for a in range(1, N_ANCHORS + 1)]
            with (out / f"{prefix}radio_loss_bins.csv").open("w", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=fields)
                writer.writeheader()
                for b in report["bins"]:
                    row = {k: v for k, v in b.items() if k != "anchor_ok_pct"}
                    row.update({f"a{a}_ok_pct": v for a, v in b["anchor_ok_pct"].items()})
                    writer.writerow(row)
            print(f"{report['session']}: {report['cycles']} cycles, {report['duration_s']:.1f} s, "
                  f"mean {report['mean_anchors_per_cycle']:.3f}/8 anchors, UART lost {report['lost_cycles']}, "
                  f"pattern {report['pattern']['pattern']}, lift {report['pattern']['co_failure_lift']}, "
                  f"temperature r {report['temperature_correlation']}")
        return 0
    except (OSError, ValueError) as exc:
        print(f"radio_loss_report: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
