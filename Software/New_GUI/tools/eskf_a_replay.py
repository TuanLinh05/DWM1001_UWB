"""Replay the onboard ESKF-A equations against logged IMU/ranges and compare TAG tracks.

python tools/eskf_a_replay.py Logs/10112026/Test_1 --lever -0.09,0,0.31
    --train 75:195 --test 198:218,235:300 --out Logs/10112026/Test_1/analysis/e20_eskf
"""

import argparse
from dataclasses import asdict
import csv
import sys
from time import perf_counter

import numpy as np

from flight_log_io import (epoch_selection, height_stats, interp_vectors, interval_mask, load_flight,
                           output_directory, parse_intervals, parse_lever, usable_epochs,
                           validate_intervals, vector_arguments, write_json)
from aided_calibration_report import fit_training
from aided_calibration import correct_ranges, solve_positions_3d
from eskf_replay import EskfParams, ImuSample, RangeSample, run_eskf_replay


def _sample_track(query, result):
    """Interpolate initialized IMU outputs; do not bridge gaps longer than 0.1 s."""
    if len(result.t) < 2:
        return np.full((len(query), 3), np.nan)
    positions = interp_vectors(query, result.t, result.tag)
    right = np.clip(np.searchsorted(result.t, query), 1, len(result.t) - 1)
    positions[result.t[right] - result.t[right - 1] > .1] = np.nan
    return positions


def _diagnostics(result, mask=None):
    selected = np.ones(len(result.update_t), dtype=bool) if mask is None else mask
    offered, accepted = int(selected.sum()), int(result.update_accepted[selected].sum())
    per_anchor = {}
    for aid in np.unique(result.update_anchor):
        rows = selected & (result.update_anchor == aid)
        nis = result.update_nis[rows]
        nis = nis[np.isfinite(nis)]
        per_anchor[str(aid)] = {"offered": int(rows.sum()), "accepted": int(result.update_accepted[rows].sum()),
                               "mean_nis": float(nis.mean()) if len(nis) else None}
    return {"offered": offered, "accepted": accepted, "acceptance_ratio": accepted / offered if offered else 0.,
            "anchors": per_anchor}


def _metrics(positions, z_ref, gps_xy):
    good = np.all(np.isfinite(positions[:, :2]), axis=1) & np.all(np.isfinite(gps_xy), axis=1)
    error = np.linalg.norm(positions[good, :2] - gps_xy[good], axis=1)
    return {"z": height_stats(positions, z_ref), "xy": {"epochs": len(error),
                "rmse_m": float(np.sqrt(np.mean(error ** 2))) if len(error) else None,
                "p95_m": float(np.percentile(error, 95)) if len(error) else None}}


def replay(data, calibrated, lever, params):
    """Use all calibrated range events within IMU coverage, including stale-RF epochs.

    RF and PX4 attitude are not ESKF update measurements. They are used only
    for the offline extra-calibration model, frame initialization and evaluation.
    """
    valid = np.isfinite(calibrated) & (calibrated > .01) & (data.t[:, None] >= data.imu_t[0]) & (data.t[:, None] <= data.imu_t[-1])
    rows, cols = np.nonzero(valid)
    anchors = [a.anchor_id for a in data.anchors]
    events = [RangeSample(float(data.t[r]), anchors[c], float(calibrated[r, c])) for r, c in zip(rows, cols)]
    imu = [ImuSample(float(t), tuple(a), tuple(g)) for t, a, g in zip(data.imu_t, data.imu_acc_flu, data.imu_gyro_flu)]
    yaw_time = data.imu_t[0] + params.init_static_s
    yaw0 = float(np.interp(yaw_time, data.t, np.unwrap(data.yaw_rig)))
    started = perf_counter()
    result = run_eskf_replay(data.anchors, imu, events, lever, yaw0, params)
    diagnostic = {**_diagnostics(result), "init_time_s": result.init_time, "reinits": result.reinits,
                  "elapsed_s": perf_counter() - started, "yaw0_rig_rad": yaw0, "input_ranges": len(events)}
    return result, diagnostic


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("flight")
    parser.add_argument("--lever", type=parse_lever, default=(-.09, 0., .31))
    parser.add_argument("--train", type=parse_intervals, required=True)
    parser.add_argument("--test", type=parse_intervals, required=True)
    parser.add_argument("--out")
    args = parser.parse_args(vector_arguments(argv))
    try:
        started = perf_counter()
        validate_intervals(args.train, args.test)
        data = load_flight(args.flight, args.train, args.lever)
        out = output_directory(data, args.out, "e20_eskf")
        fit_started = perf_counter()
        cal, settings = fit_training(data, args.train, 0)
        fit_elapsed = perf_counter() - fit_started
        params = EskfParams()
        print("Replaying default ESKF-A...", flush=True)
        raw, raw_diagnostic = replay(data, data.ranges, args.lever, params)
        print(f"Default replay done in {raw_diagnostic['elapsed_s']:.1f} s; replaying ESKF-A + aided K=0...", flush=True)
        corrected_ranges = correct_ranges(cal, data.anchors, data.ranges, data.gui_xyz[:, :2], data.yaw_rig)
        aided, aided_diagnostic = replay(data, corrected_ranges, args.lever, params)
        # Near-uniform 10 Hz scoring, using actual CSV epochs at/after each grid tick.
        grid = np.arange(np.ceil(data.t[0] * 10) / 10, np.floor(data.t[-1] * 10) / 10, .1)
        indices = np.unique(np.searchsorted(data.t, grid))
        indices = indices[indices < len(data.t)]
        t = data.t[indices]
        eligible = usable_epochs(data)[indices] & interval_mask(t, args.train + args.test)
        seed = np.column_stack((data.gui_xyz[indices, :2], data.z_ref[indices]))
        least_squares = solve_positions_3d(data.anchors, data.ranges[indices], seed)
        tracks = {"gui_filtered": data.gui_xyz[indices], "logged_raw_irls": data.irls_xyz[indices],
                  "epoch_huber": least_squares, "eskf_a_default": _sample_track(t, raw),
                  "eskf_a_aided_k0": _sample_track(t, aided)}
        segments, table = [], []
        for role, intervals in (("train", args.train), ("test", args.test)):
            for lo, hi in intervals:
                mask = (t >= lo) & (t < hi) & eligible
                entry = {"role": role, "interval_s": [lo, hi],
                         "selection": epoch_selection(data, interval_mask(data.t, [(lo, hi)])), "methods": {}}
                for name, positions in tracks.items():
                    metrics = _metrics(positions[mask], data.z_ref[indices][mask], data.gps_tag_xy[indices][mask])
                    entry["methods"][name] = metrics
                    table.append({"role": role, "start_s": lo, "end_s": hi, "method": name,
                                  "z_epochs": metrics["z"]["epochs"], "z_median_m": metrics["z"]["median_m"],
                                  "z_p95_abs_m": metrics["z"]["p95_abs_m"], "xy_epochs": metrics["xy"]["epochs"],
                                  "xy_rmse_m": metrics["xy"]["rmse_m"], "xy_p95_m": metrics["xy"]["p95_m"]})
                for name, result in (("eskf_a_default", raw), ("eskf_a_aided_k0", aided)):
                    entry["methods"][name]["range_updates"] = _diagnostics(result, (result.update_t >= lo) & (result.update_t < hi))
                segments.append(entry)
        report = {"tool": "eskf_a_replay", "version": raw.version, "training_intervals_s": args.train,
                  "test_intervals_s": args.test, "parameters": asdict(params), "parameter_selection": "Defaults fixed; no holdout tuning",
                  "lever_flu_m": args.lever, "input_audit": data.audit, "aided_calibration": cal.to_dict(),
                  "aided_settings": asdict(settings), "fit_elapsed_s": fit_elapsed, "segments": segments,
                  "replays": {"default": raw_diagnostic, "aided_k0": aided_diagnostic},
                  "scoring": "Near-uniform 10 Hz actual CSV epochs; fresh RF <=0.5 s; no interpolation across ESKF output gaps >0.1 s",
                  "limitations": ["Flat floor assumed; RF is the same source for learning and Z evaluation, not absolute truth",
                                  "GPS is rigidly aligned using training epochs; XY disagreement is not surveyed error",
                                  "All anchors in a CSV burst share its timestamp; individual slot times are not available",
                                  "Replay models ESKF equations and requested event init/rejection logic, not live ROS wall-clock freshness/timeouts",
                                  "Body initialization uses tag0 - R0*lever; the currently supplied C++ node already has this correction",
                                  "Extra calibration is fitted on training data then frozen for offline replay, including its pretraining initialization"],
                  "total_elapsed_s": perf_counter() - started}
        write_json(out / "report.json", report)
        with (out / "comparison.csv").open("w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(table[0]))
            writer.writeheader()
            writer.writerows(table)
        lines = ["| Interval | Method | Z median (m) | Z p95 abs (m) | XY RMSE (m) | XY p95 (m) |",
                 "| --- | --- | ---: | ---: | ---: | ---: |"]
        fmt = lambda value: "--" if value is None else f"{value:.4f}"
        for row in table:
            lines.append(f"| {row['role']} {row['start_s']:g}:{row['end_s']:g} | {row['method']} | "
                         + " | ".join(fmt(row[k]) for k in ("z_median_m", "z_p95_abs_m", "xy_rmse_m", "xy_p95_m")) + " |")
        with (out / "comparison.md").open("w", encoding="utf-8") as fh:
            fh.write("Z reference: the same rangefinder source used for calibration. "
                     "XY reference: GNSS rigidly aligned on training epochs. Neither is surveyed absolute truth. "
                     "Parameters remain at defaults; no holdout tuning.\n\n" + "\n".join(lines) + "\n")
        np.savez_compressed(out / "trajectories.npz", default_t=raw.t, default_tag=raw.tag, default_p=raw.p,
                            default_std=raw.p_std, aided_t=aided.t, aided_tag=aided.tag, aided_p=aided.p,
                            aided_std=aided.p_std, scoring_t=t, **tracks)
        print("\n".join(lines))
        print(f"Report: {out / 'report.json'}")
        return 0
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(f"eskf_a_replay: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
