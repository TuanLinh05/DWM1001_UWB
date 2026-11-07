"""Statistics of the G0 standard static dataset (G0.3).

Plan: Plan/G0_KE_HOACH_HIEU_CHUAN_DA_DIEM_VA_DU_LIEU_CHUAN.md (section G0.3).
Contract: tests/test_g0_static_dataset_report.py.

    python tools/static_dataset_report.py logs/<session> manifest.json [--out DIR]

The user stands the TAG on surveyed points and marks each static segment in
the GUI with waypoint labels "<point>_start" and "<point>_end" (new label box
next to "Mark Point"). The manifest gives the surveyed coordinates:

    {"points": {"P1": [x, y, z], ...},
     "segments": [{"point": "P1", "start": "P1_start", "end": "P1_end"}],   (optional)
     "trim_s": 2.0}                                                          (optional)

Without "segments", every point uses "<point>_start" / "<point>_end".
trim_s seconds are dropped at both ends of each segment (walking away after
pressing the button).

Per segment and anchor the report gives: samples, success %, median raw range,
sigma, MAD-sigma, correlation time tau (for the Gauss-Markov bias model of G2),
true distance, raw bias = median - true, and the acceptance number
residual_after_cal_mm = median - host bias (a<N>_bias_mm, i.e. the calibration
active in the session) - true. Per segment it also gives the position error of
the logged fix (pos_x_m/pos_y_m/pos_z_m) against the surveyed point.

Inputs used: uwb_fast_log.csv (tag_time_us, waypoint_label, a<N>_mm,
pos_x_m, pos_y_m, pos_z_m) and session_metadata.json ("anchors": id, x, y, z,
enabled, bias_mm). Outputs: static_dataset_report.json (strict JSON) and
static_dataset_anchors.csv in --out (default: the session folder).
main() returns 0 when acceptance passes, 1 otherwise.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import numpy as np
from report_utils import number, strict_value

N_ANCHORS = 8
MAD_TO_SIGMA = 1.4826


@dataclass(frozen=True)
class Segment:
    point: str
    xyz: tuple[float, float, float]
    start_row: int       # first CSV row kept after trimming (0-based data row index)
    end_row: int         # last CSV row kept (inclusive)
    t_start_s: float
    t_end_s: float


@dataclass(frozen=True)
class AnchorStaticStats:
    anchor_id: int
    samples: int
    success_pct: float
    median_raw_mm: float | None
    sigma_mm: float | None          # population standard deviation of the raw ranges
    mad_sigma_mm: float | None      # 1.4826 * median absolute deviation
    tau_s: float | None             # autocorrelation_time of the raw ranges
    true_mm: float
    raw_bias_mm: float | None       # median_raw - true
    host_bias_mm: float             # bias active in the session (metadata)
    residual_after_cal_mm: float | None   # median_raw - host_bias - true


@dataclass(frozen=True)
class PositionStats:
    fixes: int
    mean_xyz: tuple[float, float, float] | None
    std_xyz_m: tuple[float, float, float] | None
    error_xyz_m: tuple[float, float, float] | None   # mean - surveyed
    horizontal_error_m: float | None
    error_3d_m: float | None


def load_manifest(path: Path) -> dict[str, Any]:
    """Read and validate the manifest: non-empty "points", each [x, y, z] (ValueError otherwise).
    Set "trim_s" to 2.0 when missing."""
    with Path(path).open(encoding="utf-8-sig") as fh:
        return _validate_manifest(json.load(fh))


def _validate_manifest(manifest):
    if not isinstance(manifest, Mapping) or not isinstance(manifest.get("points"), Mapping) or not manifest["points"]:
        raise ValueError("Manifest must contain non-empty points")
    out = dict(manifest)
    points = {}
    for name, xyz in manifest["points"].items():
        if not isinstance(name, str) or not name.strip() or not isinstance(xyz, (tuple, list)) or len(xyz) != 3:
            raise ValueError(f"Point {name!r} must have a name and three finite coordinates")
        values = [number(v) for v in xyz]
        if any(v is None for v in values):
            raise ValueError(f"Point {name!r} must have three finite coordinates")
        points[name] = values
    out["points"] = points
    trim = number(manifest.get("trim_s", 2.0))
    if trim is None or trim < 0:
        raise ValueError("trim_s must be finite and nonnegative")
    out["trim_s"] = trim
    if "segments" in out:
        if not isinstance(out["segments"], list) or not out["segments"]:
            raise ValueError("segments must be a non-empty list when supplied")
        for seg in out["segments"]:
            if not isinstance(seg, Mapping) or seg.get("point") not in points:
                raise ValueError("Each segment must reference a surveyed point")
            if not all(isinstance(seg.get(k), str) and seg[k].strip() for k in ("start", "end")):
                raise ValueError("Each segment must name its start and end labels")
    return out


def load_rows(session: Path) -> list[dict[str, str]]:
    """uwb_fast_log.csv rows as dicts (csv.DictReader)."""
    with (Path(session) / "uwb_fast_log.csv").open(encoding="utf-8-sig", newline="") as fh:
        return list(csv.DictReader(fh))


def anchors_from_metadata(session: Path) -> dict[int, tuple[float, float, float, float]]:
    """{anchor_id: (x, y, z, bias_mm)} for enabled anchors in session_metadata.json."""
    with (Path(session) / "session_metadata.json").open(encoding="utf-8-sig") as fh:
        metadata = json.load(fh)
    anchors = {}
    for anchor in metadata.get("anchors", []):
        if not anchor.get("enabled", True):
            continue
        aid = int(anchor.get("anchor_id", anchor.get("id", 0)))
        values = tuple(number(anchor.get(k, 0.0 if k == "bias_mm" else None)) for k in ("x", "y", "z", "bias_mm"))
        if aid < 1 or aid > N_ANCHORS or any(v is None for v in values):
            raise ValueError(f"Invalid anchor coordinates or bias in metadata: A{aid}")
        anchors[aid] = values
    if not anchors:
        raise ValueError("Metadata contains no enabled anchors")
    return anchors


def find_segments(rows: Sequence[Mapping[str, str]], manifest: Mapping[str, Any]) -> list[Segment]:
    """Locate each segment from the first row carrying its start/end waypoint label.

    Keep rows with  t(start label) + trim_s - 1e-6 <= t <= t(end label) - trim_s + 1e-6
    (t = tag_time_us / 1e6). ValueError (naming the missing label and the labels
    found) when a label is missing, when end comes before start, or when the
    trimmed segment is empty.
    """
    manifest = _validate_manifest(manifest)
    labels = {}
    times = np.array([number(row.get("tag_time_us"), math.nan) / 1e6 for row in rows])
    for index, row in enumerate(rows):
        label = row.get("waypoint_label", "").strip()
        if label:
            labels.setdefault(label, index)
    definitions = manifest.get("segments", [{"point": p, "start": p + "_start", "end": p + "_end"}
                                             for p in manifest["points"]])
    segments = []
    for spec in definitions:
        for key in ("start", "end"):
            if spec[key] not in labels:
                raise ValueError(f"Missing waypoint label {spec[key]!r}; found labels: {', '.join(labels) or '(none)'}")
        begin, end = labels[spec["start"]], labels[spec["end"]]
        if end <= begin or not np.all(np.isfinite(times[begin:end + 1])) or np.any(np.diff(times[begin:end + 1]) < 0):
            raise ValueError(f"Invalid order or TAG times for {spec['start']} / {spec['end']}; found labels: {', '.join(labels)}")
        lower = times[begin] + manifest["trim_s"] - 1e-6
        upper = times[end] - manifest["trim_s"] + 1e-6
        kept = begin + np.flatnonzero((times[begin:end + 1] >= lower) & (times[begin:end + 1] <= upper))
        if not len(kept):
            raise ValueError(f"Trimmed segment {spec['start']} / {spec['end']} is empty; found labels: {', '.join(labels)}")
        first, last = int(kept[0]), int(kept[-1])
        segments.append(Segment(spec["point"], tuple(manifest["points"][spec["point"]]), first, last,
                                float(times[first]), float(times[last])))
    return segments


def autocorrelation_time(x: Sequence[float], dt_s: float) -> float:
    """Lag where the normalized autocorrelation first drops below 1/e, times dt_s.

    Use the biased estimator (mean removed, divided by the lag-0 sum), computed
    with an FFT; interpolate linearly between the two lags around 1/e. Return
    0.0 for a constant series, math.nan for fewer than 4 finite samples and
    math.inf if the ACF stays above 1/e for N // 4 lags. For white noise the
    result is below 2 * dt_s; for first-order Gauss-Markov noise it is ~tau.
    """
    values = np.asarray(x, dtype=float)
    values = values[np.isfinite(values)]
    n = len(values)
    if n < 4 or not math.isfinite(dt_s) or dt_s <= 0:
        return math.nan
    values = values - values.mean()
    if np.max(np.abs(values)) < 1e-12:
        return 0.0
    n_fft = 1 << (2 * n - 1).bit_length()
    spectrum = np.fft.rfft(values, n=n_fft)
    acf = np.fft.irfft(spectrum * spectrum.conj(), n=n_fft)[:n // 4 + 1]
    acf /= acf[0]
    threshold = 1.0 / math.e
    crossings = np.flatnonzero(acf < threshold)
    if not len(crossings):
        return math.inf
    k = int(crossings[0])
    fraction = (acf[k - 1] - threshold) / (acf[k - 1] - acf[k])
    return float((k - 1 + fraction) * dt_s)


def segment_anchor_stats(rows: Sequence[Mapping[str, str]], seg: Segment,
                         anchors: Mapping[int, tuple[float, float, float, float]]) -> dict[int, AnchorStaticStats]:
    """Statistics of one anchor's raw ranges inside the segment.

    tau uses the gap-free sequence of successful samples with an effective
    sample time of median(row period) / success fraction.
    """
    subset = rows[seg.start_row:seg.end_row + 1]
    times = np.array([number(row.get("tag_time_us"), math.nan) / 1e6 for row in subset])
    periods = np.diff(times)
    periods = periods[np.isfinite(periods) & (periods > 0)]
    dt = float(np.median(periods)) if len(periods) else math.nan
    out = {}
    for aid, (x, y, z, metadata_bias) in anchors.items():
        values, corrected, biases = [], [], []
        for row in subset:
            raw = number(row.get(f"a{aid}_mm"))
            if raw is None or raw <= 10:
                continue
            bias = number(row.get(f"a{aid}_bias_mm"), metadata_bias)
            values.append(raw)
            biases.append(bias)
            corrected.append(raw - bias)
        n = len(values)
        success = n / len(subset) if subset else 0.0
        true = math.dist(seg.xyz, (x, y, z)) * 1000
        median = float(np.median(values)) if n else None
        sigma = float(np.std(values)) if n else None
        mad = float(MAD_TO_SIGMA * np.median(np.abs(np.asarray(values) - median))) if n else None
        tau = autocorrelation_time(values, dt / success) if n else None
        host_bias = float(np.median(biases)) if n else metadata_bias
        out[aid] = AnchorStaticStats(aid, n, success * 100, median, sigma, mad, tau, true,
                     median - true if n else None, host_bias,
                     float(np.median(corrected)) - true if n else None)
    return out


def segment_position_stats(rows: Sequence[Mapping[str, str]], seg: Segment) -> PositionStats:
    """Mean/std of the logged fix in the segment and its error against seg.xyz."""
    fixes = []
    for row in rows[seg.start_row:seg.end_row + 1]:
        point = tuple(number(row.get(f"pos_{axis}_m")) for axis in "xyz")
        if all(v is not None for v in point):
            fixes.append(point)
    if not fixes:
        return PositionStats(0, None, None, None, None, None)
    mean = np.mean(fixes, axis=0)
    error = mean - np.asarray(seg.xyz)
    return PositionStats(len(fixes), tuple(float(v) for v in mean),
                         tuple(float(v) for v in np.std(fixes, axis=0)), tuple(float(v) for v in error),
                         float(np.linalg.norm(error[:2])), float(np.linalg.norm(error)))


def acceptance(report: Mapping[str, Any], max_residual_mm: float = 50.0, max_position_error_m: float = 0.10,
               min_success_pct: float = 87.5) -> dict[str, Any]:
    """G0 exit criteria per point: every anchor |residual_after_cal_mm| <= max_residual_mm,
    position error_3d_m <= max_position_error_m, mean anchor success >= min_success_pct
    (7 of 8 anchors). Returns {"pass": bool, "points": {point: {"pass", "reasons": [..."A4: ..."]}},
    "thresholds": {...}}."""
    points = {}
    for seg in report.get("segments", []):
        reasons = []
        stats = seg.get("anchors", {})
        if not stats:
            reasons.append("No enabled anchor statistics")
        for aid, st in stats.items():
            residual = number(st.get("residual_after_cal_mm"))
            if residual is None:
                reasons.append(f"A{aid}: no usable residual")
            elif abs(residual) > max_residual_mm:
                reasons.append(f"A{aid}: residual {residual:+.1f} mm exceeds {max_residual_mm:g} mm")
        error = number(seg.get("position", {}).get("error_3d_m"))
        if error is None:
            reasons.append("No usable position fixes")
        elif error > max_position_error_m:
            reasons.append(f"Position error {error:.3f} m exceeds {max_position_error_m:g} m")
        success = float(np.mean([number(st.get("success_pct"), 0.0) for st in stats.values()])) if stats else 0.0
        if success < min_success_pct:
            reasons.append(f"Mean anchor success {success:.1f}% is below {min_success_pct:g}%")
        item = points.setdefault(seg["point"], {"pass": True, "reasons": []})
        item["pass"] = item["pass"] and not reasons
        item["reasons"].extend(reasons)
    return {"pass": bool(points) and all(p["pass"] for p in points.values()), "points": points,
            "thresholds": {"max_residual_mm": max_residual_mm, "max_position_error_m": max_position_error_m,
                           "min_success_pct": min_success_pct}}


def build_report(session: Path, manifest: Mapping[str, Any]) -> dict[str, Any]:
    """JSON-ready report: tool, version, session, segments [{point, xyz, rows, duration_s,
    anchors {"<aid>": AnchorStaticStats fields}, position {PositionStats fields}}], acceptance."""
    manifest = _validate_manifest(manifest)
    rows = load_rows(session)
    anchors = anchors_from_metadata(session)
    report = {"tool": "static_dataset_report", "version": "g0-static-dataset-v1", "session": Path(session).name,
              "segments": []}
    for seg in find_segments(rows, manifest):
        report["segments"].append({"point": seg.point, "xyz": list(seg.xyz),
             "rows": seg.end_row - seg.start_row + 1, "start_row": seg.start_row, "end_row": seg.end_row,
             "t_start_s": seg.t_start_s, "t_end_s": seg.t_end_s, "duration_s": seg.t_end_s - seg.t_start_s,
             "anchors": {str(a): asdict(s) for a, s in segment_anchor_stats(rows, seg, anchors).items()},
             "position": asdict(segment_position_stats(rows, seg))})
    report["acceptance"] = acceptance(report)
    return strict_value(report)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    try:
        report = build_report(args.session, load_manifest(args.manifest))
        out = args.out or args.session
        out.mkdir(parents=True, exist_ok=True)
        with (out / "static_dataset_report.json").open("w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2, allow_nan=False)
            fh.write("\n")
        with (out / "static_dataset_anchors.csv").open("w", newline="", encoding="utf-8") as fh:
            fields = ["point"] + list(AnchorStaticStats.__dataclass_fields__)
            writer = csv.DictWriter(fh, fieldnames=fields)
            writer.writeheader()
            for seg in report["segments"]:
                for st in seg["anchors"].values():
                    writer.writerow({"point": seg["point"], **st})
        passed = report["acceptance"]["pass"]
        print(f"{report['session']}: {'PASS' if passed else 'FAIL'} ({len(report['segments'])} static segments)")
        for point, verdict in report["acceptance"]["points"].items():
            for reason in verdict["reasons"]:
                print(f"  {point}: {reason}")
        return 0 if passed else 1
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"static_dataset_report: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
