"""Measure in-flight noise on the actual replay IMU stream; offline E2.1 only."""

import argparse
import csv
from dataclasses import asdict
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

from flight_log_io import (dependency, interval_mask, output_directory, parse_intervals,
                           read_imu, read_px4_diagnostics, read_ulog, resolve_input, write_json)
from eskf_tuning import TUNING_VERSION, decimate_stream, estimate_imu_noise, periodogram, recommend_params


def e21_output(root, requested, name):
    """E2.1 writers never create or overwrite E2.0 output directories."""
    if requested is not None:
        path = Path(requested)
        if not path.is_absolute():
            from flight_log_io import REPO_DIR
            path = REPO_DIR / path if path.parts and path.parts[0].lower() == "logs" else Path.cwd() / path
        analysis = (root / "analysis").resolve()
        path = path.resolve()
        if not path.is_relative_to(analysis) or not path.relative_to(analysis).parts \
                or not path.relative_to(analysis).parts[0].startswith("e21_"):
            raise ValueError("E2.1 outputs must be under this flight's analysis/e21_* directory")
    return output_directory(SimpleNamespace(root=root), requested, name)


def _longest(t, values, intervals):
    candidates = []
    hz = 1 / np.median(np.diff(t))
    for lo, hi in intervals:
        k = np.flatnonzero((t >= lo) & (t < hi))
        if len(k):
            candidates.extend(np.split(k, np.flatnonzero(np.diff(t[k]) > 5 / hz) + 1))
    return values[max(candidates, key=len)] if candidates else np.empty((0, 3))


def _psd(out, native, analysed, intervals, mode):
    os.environ.setdefault("MPLCONFIGDIR", str(out / ".matplotlib"))
    plt = dependency("matplotlib.pyplot", "matplotlib")
    plt.switch_backend("Agg")
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), constrained_layout=True)
    streams = ((native, "native"),) if mode == "native" else ((native, "native"), (analysed, mode))
    for stream, label in streams:
        t, acc, gyro = stream
        hz = 1 / np.median(np.diff(t))
        for row, values in enumerate((acc, gyro)):
            values = _longest(t, values, intervals)
            for col in range(3):
                frequencies, power = periodogram(values[:, col], hz)
                good = (frequencies > 0) & (power > 0)
                axes[row, col].loglog(frequencies[good], power[good], label=f"{label} ({hz:.1f} Hz)")
    for row in range(2):
        for col in range(3):
            axes[row, col].set(xlabel="Apparent frequency (Hz)", ylabel="PSD ((m/s²)²/Hz)" if row == 0 else "PSD ((rad/s)²/Hz)",
                               title=f"{'Acceleration' if row == 0 else 'Gyro'} {'xyz'[col]} (FLU)")
            axes[row, col].grid(True, alpha=.2)
            axes[row, col].legend()
    fig.suptitle("Training windows only; longest contiguous segment. Peaks may be aliased.")
    fig.savefig(out / "psd.png", dpi=140)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("flight")
    parser.add_argument("--windows", required=True, type=parse_intervals)
    parser.add_argument("--stream-hz", type=float)
    parser.add_argument("--decimate", choices=("pick", "mean"), default="pick")
    parser.add_argument("--margin", type=float, default=1.)
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    try:
        root = resolve_input(args.flight)
        paths = sorted(root.glob("*.ulg"))
        if len(paths) != 1:
            raise ValueError("Expected exactly one ULog in the flight directory")
        datasets, _ = read_ulog(paths[0])
        native = read_imu(datasets)
        analysed = decimate_stream(*native, args.stream_hz, args.decimate) if args.stream_hz is not None else native
        mode = args.decimate if args.stream_hz is not None else "native"
        noise = estimate_imu_noise(*analysed, args.windows)
        params = recommend_params(noise, margin=args.margin)
        original = estimate_imu_noise(*native, args.windows)
        warnings = list(noise.warnings)
        if args.stream_hz is not None and max(noise.acc_trend_std) > 2 * max(original.acc_trend_std):
            warnings.append("STREAM_ALIASING")
        px4 = read_px4_diagnostics(paths[0], args.windows)
        rows = []
        rf = datasets.get("distance_sensor", {})
        rf_t = np.asarray(rf.get("timestamp", [])) / 1e6
        rf_z = np.asarray(rf.get("current_distance", []))
        rf_down = np.asarray(rf.get("orientation", np.full(len(rf_t), 25))) == 25
        for start in np.arange(np.floor(native[0][0] / 10) * 10, native[0][-1], 10.):
            end = start + 10
            n = estimate_imu_noise(*analysed, [(start, end)])
            selected = interval_mask(rf_t, [(start, end)]) & rf_down & np.isfinite(rf_z) & (rf_z > 0)
            row = dict(start_s=float(start), end_s=float(end), samples=n.samples, stream_hz=n.stream_hz,
                       duration_s=n.duration_s, acc_density=n.acc_density, gyro_density=n.gyro_density,
                       rangefinder_raw_median_m=float(np.median(rf_z[selected])) if selected.any() else None,
                       warnings=",".join(n.warnings))
            for name in ("acc_std", "gyro_std", "acc_trend_std"):
                row.update({f"{name}_{axis}": float(v) for axis, v in zip("xyz", getattr(n, name))})
            rows.append(row)
        out = e21_output(root, args.out, "e21_imu_noise_" + mode)
        provenance = dict(flight=str(root), windows_s=args.windows, stream_hz=args.stream_hz, decimate=mode,
                          measured_stream_hz=noise.stream_hz, margin=args.margin, version=TUNING_VERSION)
        write_json(out / "imu_noise.json", {**noise.to_dict(), "warnings": warnings,
                   "native_noise": original.to_dict() if args.stream_hz is not None else None,
                   "ten_second_windows": rows, "px4": px4, "provenance": provenance,
                   "density_definition": "Worst-axis detrended sample std / sqrt(median stream rate)",
                   "limitations": ["Flight vibration, not a stationary pre-arm sensor noise baseline",
                                   "Spectral frequencies are apparent; decimation can alias motor vibration",
                                   "PX4 metrics are numeric evidence without additional quality thresholds"]})
        write_json(out / "recommended_params.json", {"params": asdict(params), "provenance": provenance})
        with (out / "windows.csv").open("w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        _psd(out, native, analysed, args.windows, mode)
        print(f"{mode}: {noise.stream_hz:.3f} Hz; na={params.na:.6f}; ng={params.ng:.6f}; warnings={warnings}")
        print(f"Report: {out / 'imu_noise.json'}")
        return 0
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(f"imu_noise_report: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
