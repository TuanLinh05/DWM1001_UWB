"""Compare default, measured-noise and PX4-attitude-aided ESKF on a frozen holdout."""

import argparse
import csv
from dataclasses import asdict, fields, replace
import json
import math
import os
import sys
from time import perf_counter

import numpy as np

from flight_log_io import (dependency, epoch_selection, interval_mask, load_flight, parse_intervals,
                           parse_lever, read_mcap_cache, read_ulog, resolve_input, rig_attitude_rotations,
                           usable_epochs, validate_intervals, vector_arguments, write_json)
from imu_noise_report import e21_output
from eskf_a_replay import _diagnostics, _metrics, _sample_track, fit_training, replay
from aided_calibration import correct_ranges, solve_positions_3d
from eskf_replay import AttitudeSample, EskfParams, ImuSample, RangeSample, run_eskf_replay
from eskf_tuning import TUNING_VERSION, decimate_stream, innovation_autocorrelation, normalized_errors


def _unique_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def load_params(path, flight, train, stream_hz, decimate):
    """Strict recommendation schema, types, finite values and training/stream provenance."""
    def bad_constant(value):
        raise ValueError(f"Non-finite JSON constant: {value}")
    with resolve_input(path).open(encoding="utf-8-sig") as fh:
        document = json.load(fh, object_pairs_hook=_unique_keys, parse_constant=bad_constant)
    if not isinstance(document, dict) or set(document) != {"params", "provenance"}:
        raise ValueError("Parameter JSON must contain exactly params and provenance")
    values = document["params"]
    expected = {f.name for f in fields(EskfParams)}
    if not isinstance(values, dict) or set(values) != expected:
        actual = set(values) if isinstance(values, dict) else set()
        raise ValueError(f"EskfParams keys: missing={sorted(expected-actual)}, unknown={sorted(actual-expected)}")
    for name, value in values.items():
        if name == "reinit_keep_attitude":
            if type(value) is not bool:
                raise ValueError(f"{name} must be boolean")
        elif name in ("att_sig_deg", "sig_th0_deg"):
            if not isinstance(value, list) or len(value) != 3 or any(type(v) not in (int, float) for v in value) \
                    or not np.isfinite(value).all() or min(value) <= 0:
                raise ValueError(f"{name} must contain three finite positive numbers")
            values[name] = tuple(value)
        elif name in ("init_min_anchors", "reinit_after_rejects"):
            if type(value) is not int or value < (4 if name == "init_min_anchors" else 1):
                raise ValueError(f"Invalid positive integer {name}")
        else:
            zero_allowed = name in ("na", "ng", "nba", "nbg", "min_anchor_interval_s", "att_min_interval_s")
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or (not zero_allowed and value == 0):
                raise ValueError(f"Invalid finite {'nonnegative' if zero_allowed else 'positive'} {name}")
    provenance = document["provenance"]
    keys = {"flight", "windows_s", "stream_hz", "decimate", "measured_stream_hz", "margin", "version"}
    if not isinstance(provenance, dict) or set(provenance) != keys:
        raise ValueError("Missing or unknown recommendation provenance keys")
    if resolve_input(provenance["flight"]) != flight or provenance["windows_s"] != [list(i) for i in train] \
            or provenance["stream_hz"] != stream_hz or provenance["decimate"] != decimate \
            or provenance["version"] != TUNING_VERSION:
        raise ValueError("Parameter recommendation must match this flight, training windows and IMU stream")
    for name in ("measured_stream_hz", "margin"):
        value = provenance[name]
        if type(value) not in (int, float) or not math.isfinite(value) or value < (1 if name == "margin" else 1e-12):
            raise ValueError(f"Invalid recommendation {name}")
    return EskfParams(**values), provenance


def _attitude_errors(result, reference):
    relative = np.einsum("nji,njk->nik", reference, result.R)
    return (np.degrees(np.arctan2(relative[:, 1, 0], relative[:, 0, 0])),
            np.degrees(np.arccos(np.clip(relative[:, 2, 2], -1, 1))))


def _attitude_stats(yaw, tilt, selected):
    yaw, tilt = yaw[selected], tilt[selected]
    return dict(samples=len(yaw), yaw_median_deg=float(np.median(yaw)) if len(yaw) else None,
                yaw_p95_abs_deg=float(np.percentile(np.abs(yaw), 95)) if len(yaw) else None,
                tilt_p95_deg=float(np.percentile(tilt, 95)) if len(tilt) else None)


def _aided_replay(data, calibrated, lever, params, attitude):
    # Same event construction as the imported E2.0 replay(), with attitude events added.
    valid = np.isfinite(calibrated) & (calibrated > .01) & (data.t[:, None] >= data.imu_t[0]) \
        & (data.t[:, None] <= data.imu_t[-1])
    rows, cols = np.nonzero(valid)
    aids = [a.anchor_id for a in data.anchors]
    ranges = [RangeSample(float(data.t[r]), aids[c], float(calibrated[r, c])) for r, c in zip(rows, cols)]
    imu = [ImuSample(float(t), tuple(a), tuple(g)) for t, a, g in zip(data.imu_t, data.imu_acc_flu, data.imu_gyro_flu)]
    yaw0 = float(np.interp(data.imu_t[0] + params.init_static_s, data.t, np.unwrap(data.yaw_rig)))
    start = perf_counter()
    result = run_eskf_replay(data.anchors, imu, ranges, lever, yaw0, params, attitude=attitude)
    return result, {**_diagnostics(result), "elapsed_s": perf_counter()-start, "reinits": result.reinits,
                    "init_time_s": result.init_time, "yaw0_rig_rad": yaw0, "input_ranges": len(ranges)}


def acceptance_checks(report):
    """Canonical Test_1 thresholds from section 5; never used to select parameters."""
    if report["training_intervals_s"] != [(75., 195.)] or report["test_intervals_s"] != [(198., 218.), (235., 300.)]:
        return {"applicable": False, "reason": "Section 5 thresholds apply to the canonical Test_1 windows"}
    checks = {}
    def add(name, value, limit, ok):
        checks[name] = dict(value=value, requirement=limit, passed=bool(ok))
    for name in ("tuned", "tuned_att"):
        for i, segment in enumerate(report["segments"]):
            m = segment["methods"][name]
            for key, value, limit in (("xy_rmse_m", m["xy"]["rmse_m"], (.125, .185, .255)[i]),
                                      ("xy_p95_m", m["xy"]["p95_m"], (.24, .34, .34)[i]),
                                      ("z_median_abs_m", abs(m["z"]["median_m"]) if m["z"]["median_m"] is not None else None, .03),
                                      ("z_p95_m", m["z"]["p95_abs_m"], .15)):
                add(f"{name}.{i}.{key}", value, f"<= {limit}", value is not None and value <= limit)
            if name == "tuned_att":
                for key, limit in (("yaw_p95_abs_deg", 3.), ("tilt_p95_deg", 2.)):
                    value = m["attitude"][key]
                    add(f"{name}.{i}.{key}", value, f"<= {limit}", value is not None and value <= limit)
        c = report["replays"][name]["normalized_z_75_300"]
        coverage, rms = c["frac_within_2sigma"], c["rms_normalized"]
        add(name+".coverage", coverage, ">= 0.90", coverage >= .9)
        add(name+".normalized_rms", rms, "0.6 <= value <= 1.4", .6 <= rms <= 1.4)
    aided = report["replays"]["tuned_att"]
    add("tuned_att.reinits", aided["reinits"], "== 0", aided["reinits"] == 0)
    for key in ("acceptance_ratio", "attitude_acceptance_ratio"):
        add("tuned_att."+key, aided[key], ">= 0.99", aided[key] >= .99)
    default = report["replays"]["default"]
    index = 0 if report["imu_stream"]["decimate"] == "pick" else 2
    threshold = 5 if index == 0 else 1
    xy = report["segments"][index]["methods"]["default"]["xy"]["rmse_m"]
    add("default.failure_reproduced", dict(reinits=default["reinits"], xy_rmse_m=xy),
        f"reinits >= {threshold} or XY RMSE > 0.5 m", default["reinits"] >= threshold or (xy is not None and xy > .5))
    for name, diagnostic in report["replays"].items():
        add(name+".runtime_s", diagnostic["elapsed_s"], "< 60", diagnostic["elapsed_s"] < 60)
    return {"applicable": True, "all_passed": all(c["passed"] for c in checks.values()), "checks": checks}


def _plots(out, results, references, data):
    os.environ.setdefault("MPLCONFIGDIR", str(out / ".matplotlib"))
    plt = dependency("matplotlib.pyplot", "matplotlib")
    plt.switch_backend("Agg")
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True, constrained_layout=True)
    selected = interval_mask(data.t, [(75., 300.)])
    for ax, (name, res) in zip(axes, results.items()):
        ax.plot(data.t[selected][::10], data.z_ref[selected][::10], "k", lw=.8, label="RF reference (same source)")
        sigma = res.p_std[:, 2]
        # NaN separators preserve visible reinitialization gaps in the plotted estimate.
        gaps = np.r_[False, np.diff(res.t) > .1]
        z = res.tag[:, 2].copy()
        z[gaps] = np.nan
        ax.plot(res.t, z, lw=.7, label=name)
        ax.fill_between(res.t, z-2*sigma, z+2*sigma, alpha=.25, label="±2 body-position σ")
        ax.set(ylabel="TAG z (m)", title=name, xlim=(75, 300))
        ax.grid(True, alpha=.2)
        ax.legend(loc="upper left", ncol=3)
    axes[-1].set_xlabel("PX4 boot time (s)")
    fig.suptitle("Measured-noise ESKF comparison; RF is not surveyed ground truth")
    fig.savefig(out / "z_sigma.png", dpi=140)
    plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True, constrained_layout=True)
    for ax, (name, res) in zip(axes, results.items()):
        reference = references[name]
        yaw = np.unwrap(np.arctan2(res.R[:, 1, 0], res.R[:, 0, 0]))
        ref_yaw = np.unwrap(np.arctan2(reference[:, 1, 0], reference[:, 0, 0]))
        # Align equivalent 2pi branches for a readable overlay, not for scoring.
        if len(yaw):
            yaw -= round((yaw[0]-ref_yaw[0])/(2*math.pi)) * 2*math.pi
        ax.plot(res.t, np.degrees(ref_yaw), "k", lw=1., label="PX4")
        ax.plot(res.t, np.degrees(yaw), lw=.8, label=name)
        ax.set(ylabel="Yaw (deg, unwrapped)", title=name, xlim=(75, 300))
        ax.grid(True, alpha=.2)
        ax.legend()
    axes[-1].set_xlabel("PX4 boot time (s)")
    fig.savefig(out / "yaw_vs_px4.png", dpi=140)
    plt.close(fig)


def trajectory_arrays(results, references):
    """Unambiguous variant/field namespaces, including tuned vs tuned_att.

    A single underscore would collide: tuned + att_update_t and
    tuned_att + update_t both become tuned_att_update_t.
    """
    arrays = {}
    for name, res in results.items():
        for key in ("t", "p", "v", "R", "tag", "p_std", "update_t", "update_anchor", "update_accepted", "update_nis",
                    "update_innovation_m", "att_update_t", "att_accepted", "att_nis"):
            arrays[name+"__"+key] = getattr(res, key)
        arrays[name+"__px4_R"] = references[name]
    return arrays


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("flight")
    parser.add_argument("--lever", type=parse_lever, default=(-.09, 0., .31))
    parser.add_argument("--train", type=parse_intervals, required=True)
    parser.add_argument("--test", type=parse_intervals, required=True)
    parser.add_argument("--imu-stream-hz", type=float, required=True)
    parser.add_argument("--imu-decimate", choices=("pick", "mean"), required=True)
    parser.add_argument("--params-json", required=True)
    parser.add_argument("--out")
    args = parser.parse_args(vector_arguments(argv))
    try:
        start = perf_counter()
        validate_intervals(args.train, args.test)
        root = resolve_input(args.flight)
        params, provenance = load_params(args.params_json, root, args.train, args.imu_stream_hz, args.imu_decimate)
        data = load_flight(root, args.train, args.lever)
        out = e21_output(root, args.out, f"e21_eskf_{args.imu_decimate}{args.imu_stream_hz:g}")
        cal, settings = fit_training(data, args.train, 0)
        calibrated = correct_ranges(cal, data.anchors, data.ranges, data.gui_xyz[:, :2], data.yaw_rig)
        imu = decimate_stream(data.imu_t, data.imu_acc_flu, data.imu_gyro_flu, args.imu_stream_hz, args.imu_decimate)
        stream = replace(data, imu_t=imu[0], imu_acc_flu=imu[1], imu_gyro_flu=imu[2])
        bridge, _ = read_mcap_cache(root, data.session)
        datasets, _ = read_ulog(data.audit["sources"]["ulog"])
        offset = data.audit["yaw_alignment"]["rig_yaw_offset_rad"]
        att_t = np.arange(imu[0][0], imu[0][-1], .02)
        att_R = rig_attitude_rotations(att_t, bridge, datasets, offset)
        attitude = [AttitudeSample(float(t), R) for t, R in zip(att_t, att_R)]
        check_R = rig_attitude_rotations(data.t, bridge, datasets, offset)
        yaw_check = np.arctan2(check_R[:, 1, 0], check_R[:, 0, 0]) - data.yaw_rig
        yaw_check = (yaw_check + math.pi) % (2*math.pi) - math.pi
        results, diagnostics, references = {}, {}, {}
        for name, configuration in (("default", EskfParams()), ("tuned", params), ("tuned_att", params)):
            print(f"Replaying {name} ({args.imu_decimate}{args.imu_stream_hz:g})...", flush=True)
            result, diagnostic = (_aided_replay(stream, calibrated, args.lever, configuration, attitude)
                                  if name == "tuned_att" else replay(stream, calibrated, args.lever, configuration))
            results[name] = result
            references[name] = rig_attitude_rotations(result.t, bridge, datasets, offset)
            diagnostic.update(parameters=asdict(configuration), skipped_ranges=result.skipped_ranges,
                              attitude_offered=len(result.att_update_t), attitude_accepted=int(result.att_accepted.sum()),
                              attitude_acceptance_ratio=float(result.att_accepted.mean()) if len(result.att_accepted) else 0.,
                              innovation_autocorrelation=innovation_autocorrelation(result))
            selected = interval_mask(data.t, [(75., 300.)]) & usable_epochs(data)
            diagnostic["normalized_z_75_300"] = normalized_errors(result.t, result.tag[:, 2], result.p_std[:, 2],
                                                                  data.t[selected], data.z_ref[selected])
            diagnostics[name] = diagnostic
            print(f"{name}: {diagnostic['elapsed_s']:.2f} s, reinits={result.reinits}, "
                  f"range acceptance={result.acceptance_ratio():.5f}", flush=True)
        grid = np.arange(np.ceil(data.t[0]*10)/10, np.floor(data.t[-1]*10)/10, .1)
        indices = np.unique(np.searchsorted(data.t, grid))
        indices = indices[indices < len(data.t)]
        t = data.t[indices]
        eligible = usable_epochs(data)[indices]
        seed = np.c_[data.gui_xyz[indices, :2], data.z_ref[indices]]
        tracks = {"gui_filtered": data.gui_xyz[indices], "logged_raw_irls": data.irls_xyz[indices],
                  "epoch_huber": solve_positions_3d(data.anchors, data.ranges[indices], seed),
                  **{name: _sample_track(t, res) for name, res in results.items()}}
        segments, table, yaw_errors = [], [], {}
        for name, res in results.items():
            yaw_errors[name] = _attitude_errors(res, references[name])
        for role, intervals in (("train", args.train), ("test", args.test)):
            for lo, hi in intervals:
                mask = (t >= lo) & (t < hi) & eligible
                entry = dict(role=role, interval_s=[lo, hi], methods={},
                             selection=epoch_selection(data, interval_mask(data.t, [(lo, hi)])))
                for name, positions in tracks.items():
                    metric = _metrics(positions[mask], data.z_ref[indices][mask], data.gps_tag_xy[indices][mask])
                    if name in results:
                        res = results[name]
                        reference_mask = interval_mask(data.t, [(lo, hi)]) & usable_epochs(data)
                        metric["normalized_z"] = normalized_errors(res.t, res.tag[:, 2], res.p_std[:, 2],
                                                                     data.t[reference_mask], data.z_ref[reference_mask])
                        metric["attitude"] = _attitude_stats(*yaw_errors[name], interval_mask(res.t, [(lo, hi)]))
                        metric["range_updates"] = _diagnostics(res, interval_mask(res.update_t, [(lo, hi)]))
                        att_mask = interval_mask(res.att_update_t, [(lo, hi)])
                        metric["attitude_updates"] = dict(offered=int(att_mask.sum()), accepted=int(res.att_accepted[att_mask].sum()),
                                                          acceptance_ratio=float(res.att_accepted[att_mask].mean()) if att_mask.any() else 0.)
                        subset = replace(res, update_t=res.update_t[(res.update_t >= lo) & (res.update_t < hi)],
                                         update_anchor=res.update_anchor[(res.update_t >= lo) & (res.update_t < hi)],
                                         update_accepted=res.update_accepted[(res.update_t >= lo) & (res.update_t < hi)],
                                         update_nis=res.update_nis[(res.update_t >= lo) & (res.update_t < hi)],
                                         update_innovation_m=res.update_innovation_m[(res.update_t >= lo) & (res.update_t < hi)])
                        metric["innovation_autocorrelation"] = innovation_autocorrelation(subset)
                    entry["methods"][name] = metric
                    table.append(dict(role=role, start_s=lo, end_s=hi, method=name, z_epochs=metric["z"]["epochs"],
                                      z_median_m=metric["z"]["median_m"], z_p95_abs_m=metric["z"]["p95_abs_m"],
                                      xy_epochs=metric["xy"]["epochs"], xy_rmse_m=metric["xy"]["rmse_m"], xy_p95_m=metric["xy"]["p95_m"],
                                      frac_within_2sigma=metric.get("normalized_z", {}).get("frac_within_2sigma"),
                                      rms_normalized=metric.get("normalized_z", {}).get("rms_normalized"),
                                      yaw_median_deg=metric.get("attitude", {}).get("yaw_median_deg"),
                                      yaw_p95_abs_deg=metric.get("attitude", {}).get("yaw_p95_abs_deg"),
                                      tilt_p95_deg=metric.get("attitude", {}).get("tilt_p95_deg")))
                segments.append(entry)
        report = dict(tool="eskf_tuning_report", version=TUNING_VERSION, training_intervals_s=args.train, test_intervals_s=args.test,
                      imu_stream=dict(requested_hz=args.imu_stream_hz, measured_hz=float(1/np.median(np.diff(imu[0]))),
                                      samples=len(imu[0]), decimate=args.imu_decimate), parameters_provenance=provenance,
                      parameter_selection="IMU noise and aided K=0 learned on training windows only; frozen on holdout",
                      fixed_design="Per-anchor interval and attitude sigmas/gate/interval are fixed design choices, not tuned on holdout",
                      lever_flu_m=args.lever, input_audit=data.audit, aided_calibration=cal.to_dict(), aided_settings=asdict(settings),
                      attitude_source="Merged PX4 MAVLink/ULog quaternions, SLERP onto a 50 Hz grid; FLU to rig",
                      attitude_yaw_frame_max_error_deg=float(np.degrees(np.max(np.abs(yaw_check)))),
                      replays=diagnostics, segments=segments,
                      scoring="XY/Z metrics: same near-uniform 10 Hz actual CSV epochs as E2.0. Normalized Z: all usable fresh-RF epochs, gaps >0.1 s excluded",
                      limitations=["RF Z is the same source used to learn calibration, not surveyed ground truth; flat floor assumed",
                                   "XY uses GNSS rigidly aligned on training data, not surveyed ground truth",
                                   "Normalized TAG Z uses body-position std; lever-arm attitude uncertainty is omitted",
                                   "All anchors in a CSV burst share its timestamp; slot times are unavailable",
                                   "PX4 attitude aiding is correlated with PX4 IMU; nominal covariance is not an independent truth bound",
                                   "Initial static averaging is retained for E2.0 compatibility; real log starts already armed",
                                   "Replay is offline only; no onboard source, firmware, GUI configuration or original recordings changed"],
                      total_elapsed_s=perf_counter()-start)
        report["acceptance"] = acceptance_checks(report)
        write_json(out / "report.json", report)
        with (out / "comparison.csv").open("w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(table[0]))
            writer.writeheader()
            writer.writerows(table)
        lines = ["| Window (s) | Method | XY RMSE (cm) | XY p95 (cm) | Z median (cm) | Z p95 (cm) | <2σ | RMS/σ | Yaw median / p95 (deg) | Tilt p95 (deg) |",
                 "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
        fmt = lambda value, scale=1.: "--" if value is None else f"{value*scale:.3f}"
        for row in table:
            lines.append(f"| {row['start_s']:g}:{row['end_s']:g} | {row['method']} | " + " | ".join([
                *(fmt(row[k], 100) for k in ("xy_rmse_m", "xy_p95_m", "z_median_m", "z_p95_abs_m")),
                fmt(row["frac_within_2sigma"]), fmt(row["rms_normalized"]),
                fmt(row["yaw_median_deg"])+" / "+fmt(row["yaw_p95_abs_deg"]), fmt(row["tilt_p95_deg"])]) + " |")
        notes = [report["parameter_selection"], report["fixed_design"], *report["limitations"],
                 f"Yaw drift of tuned (no PX4 attitude): {[s['methods']['tuned']['attitude']['yaw_median_deg'] for s in segments]} deg median.",
                 f"Section 5 acceptance: {report['acceptance'].get('all_passed', 'not applicable')}; see report.json for every threshold."]
        (out / "comparison.md").write_text("\n\n".join(notes)+"\n\n"+"\n".join(lines)+"\n", encoding="utf-8")
        arrays = dict(scoring_t=t, z_ref=data.z_ref[indices], gps_tag_xy=data.gps_tag_xy[indices],
                      reference_t=data.t, reference_z=data.z_ref, **tracks)
        arrays.update(trajectory_arrays(results, references))
        np.savez_compressed(out / "trajectories.npz", **arrays)
        _plots(out, results, references, data)
        # Windows redirected stdout may use cp1252 even though report files are UTF-8.
        print("\n".join(lines).replace("σ", "sigma"))
        print(f"Acceptance: {report['acceptance'].get('all_passed', 'not applicable')}; report: {out / 'report.json'}")
        return 0
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        print(f"eskf_tuning_report: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
