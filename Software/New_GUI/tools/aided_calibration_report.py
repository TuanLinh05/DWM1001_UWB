"""Fit sensor-aided calibration on training epochs and evaluate separate holdouts.

python tools/aided_calibration_report.py Logs/10112026/Test_1 --train 75:195
    --test 198:218,235:300 --harmonics 0 --out Logs/10112026/Test_1/analysis/e20_aided
"""

import argparse
from dataclasses import asdict
import os
import sys
from time import perf_counter

import numpy as np

from flight_log_io import (dependency, epoch_selection, height_stats, interval_mask, load_flight,
                           output_directory, parse_intervals, parse_lever, usable_epochs,
                           validate_intervals, vector_arguments, write_json)
from aided_calibration import AidedSettings, fit_aided_calibration, holdout_report


def fit_training(data, train, harmonics=0):
    """Fit only requested training rows; stale reference rows are never offered."""
    selected = interval_mask(data.t, train) & usable_epochs(data)
    settings = AidedSettings(harmonics=harmonics)
    cal, _ = fit_aided_calibration(data.anchors, data.ranges[selected], data.z_ref[selected],
                                  data.yaw_rig[selected], data.gui_xyz[selected, :2], settings)
    if not cal.ok:
        raise ValueError("Aided calibration failed: " + "; ".join(w.message for w in cal.warnings))
    return cal, settings


def plot_height(out, data, before, after, train, tests):
    os.environ.setdefault("MPLCONFIGDIR", str(out / "matplotlib_cache"))
    matplotlib = dependency("matplotlib")
    matplotlib.use("Agg")
    plt = dependency("matplotlib.pyplot", "matplotlib")
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True, layout="constrained")
    axes[0].plot(data.t, before[:, 2], label="Free-Z range solve before", lw=.7, alpha=.8)
    axes[0].plot(data.t, after[:, 2], label="Free-Z range solve after", lw=.7, alpha=.8)
    axes[0].plot(data.t, data.z_ref, label="Rangefinder-derived TAG reference", color="black", lw=.8)
    axes[0].set_ylabel("TAG height (m)")
    axes[0].set_title("E2.0 sensor-aided calibration | same-source Z evaluation, not surveyed truth")
    axes[1].plot(data.t, before[:, 2] - data.z_ref, label="Before minus reference", lw=.7)
    axes[1].plot(data.t, after[:, 2] - data.z_ref, label="After minus reference", lw=.7)
    axes[1].axhline(0, color="black", lw=.6)
    axes[1].set_ylabel("dz (m)")
    axes[1].set_xlabel("PX4 boot time (s)")
    for ax in axes:
        for i, (lo, hi) in enumerate(train):
            ax.axvspan(lo, hi, color="green", alpha=.08, label="Training" if i == 0 else None)
        for i, (lo, hi) in enumerate(tests):
            ax.axvspan(lo, hi, color="orange", alpha=.09, label="Holdout" if i == 0 else None)
        ax.grid(alpha=.25)
        ax.legend(ncol=2, fontsize=8)
    fig.savefig(out / "dz_before_after.png", dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("flight")
    parser.add_argument("--train", type=parse_intervals, required=True)
    parser.add_argument("--test", type=parse_intervals, required=True)
    parser.add_argument("--harmonics", type=int, choices=(0, 1, 2), default=0)
    parser.add_argument("--lever", type=parse_lever, default=(-.09, 0., .31))
    parser.add_argument("--out")
    args = parser.parse_args(vector_arguments(argv))
    try:
        started = perf_counter()
        validate_intervals(args.train, args.test)
        data = load_flight(args.flight, args.train, args.lever)
        out = output_directory(data, args.out, "e20_aided")
        cal, settings = fit_training(data, args.train, args.harmonics)
        valid = usable_epochs(data) & interval_mask(data.t, args.train + args.test)
        _, solved_before, solved_after = holdout_report(data.anchors, cal, data.ranges[valid], data.z_ref[valid],
                                                       data.yaw_rig[valid], data.gui_xyz[valid, :2])
        before, after = np.full_like(data.gui_xyz, np.nan), np.full_like(data.gui_xyz, np.nan)
        before[valid], after[valid] = solved_before, solved_after
        segments = []
        for role, intervals in (("train", args.train), ("test", args.test)):
            for lo, hi in intervals:
                mask = interval_mask(data.t, [(lo, hi)])
                segments.append({"role": role, "interval_s": [lo, hi], "selection": epoch_selection(data, mask),
                                 "before": height_stats(before[mask], data.z_ref[mask]),
                                 "after": height_stats(after[mask], data.z_ref[mask])})
        run = {"training_intervals_s": args.train, "test_intervals_s": args.test, "settings": asdict(settings),
               "lever_flu_m": args.lever, "rig_yaw_offset_rad": data.audit["yaw_alignment"]["rig_yaw_offset_rad"],
               "bearing_convention": "atan2(anchor_y-tag_y,anchor_x-tag_x)-yaw_rig; yaw_rig=offset-yaw_ned"}
        write_json(out / "aided_calibration.json", {**cal.to_dict(), "run": run})
        report = {"tool": "aided_calibration_report", "run": run, "input_audit": data.audit,
                  "segments": segments, "calibration_warnings": [asdict(w) for w in cal.warnings],
                  "limitations": ["Flat floor assumed", "Training and holdout intervals are disjoint",
                                  "Height is evaluated using the same rangefinder source used for learning; not absolute surveyed accuracy",
                                  "Free-Z evaluation uses the reference only as an initial seed, never as a height constraint"],
                  "elapsed_s": perf_counter() - started}
        plot_height(out, data, before, after, args.train, args.test)
        write_json(out / "report.json", report)
        np.savez_compressed(out / "evaluation.npz", t=data.t, before=before, after=after, z_ref=data.z_ref)
        print("Extra biases (mm):", ", ".join(f"A{a}: {c.bias_m * 1000:+.2f}" for a, c in cal.corrections.items()))
        print(f"Harmonics requested/used: {cal.harmonics_requested}/{cal.harmonics_used}; yaw sectors: {cal.yaw_sectors}")
        for warning in cal.warnings:
            print(f"{warning.code}: {warning.message}")
        for segment in segments:
            print(f"{segment['role']} {segment['interval_s']}: dz median "
                  f"{segment['before']['median_m']} -> {segment['after']['median_m']} m; "
                  f"usable {segment['selection']['usable']}/{segment['selection']['requested']}")
        print(f"Report: {out / 'report.json'}")
        return 0
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(f"aided_calibration_report: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
