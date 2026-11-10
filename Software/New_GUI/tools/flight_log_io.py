"""Read-only flight-log IO, sensor references and train-only frame alignment for E2.0.

Times are PX4 seconds since boot; positions are metres in rig ENU. Optional
ULog/MCAP/scipy dependencies are imported only when their readers are used.
The Test_1 clock envelope and height construction follow analyze_test1.py;
no source cache or recording is rewritten.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
import csv
import importlib
import json
import math
from pathlib import Path
import struct
import sys
from typing import Any

import numpy as np

GUI_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = GUI_DIR.parents[1]
if str(GUI_DIR) not in sys.path:
    sys.path.insert(0, str(GUI_DIR))


@dataclass(frozen=True)
class FlightAnchor:
    anchor_id: int
    x: float
    y: float
    z: float
    enabled: bool = True


@dataclass
class FlightData:
    root: Path
    session: Path
    anchors: tuple[FlightAnchor, ...]
    t: np.ndarray
    ranges: np.ndarray
    gui_xyz: np.ndarray
    irls_xyz: np.ndarray
    z_ref: np.ndarray
    rangefinder_age_s: np.ndarray
    yaw_rig: np.ndarray
    gps_tag_xy: np.ndarray
    imu_t: np.ndarray
    imu_acc_flu: np.ndarray
    imu_gyro_flu: np.ndarray
    audit: dict[str, Any]


def dependency(name, package=None):
    """Import an optional offline dependency, with an actionable error."""
    try:
        return importlib.import_module(name)
    except ImportError as exc:
        raise RuntimeError(f"Optional offline dependency '{name}' is unavailable. Install it with: "
                           f"python -m pip install {package or name.split('.')[0]}") from exc


def json_safe(value):
    """Convert report data to strict JSON primitives without nan/inf."""
    if is_dataclass(value):
        value = asdict(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [json_safe(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, Path):
        return str(value)
    return value


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(json_safe(value), fh, indent=2, allow_nan=False)
        fh.write("\n")


def resolve_input(path):
    """Accept paths relative to cwd or repository root (including Logs/...)."""
    candidate = Path(path)
    if candidate.exists():
        return candidate.resolve()
    candidate = REPO_DIR / path
    if candidate.exists():
        return candidate.resolve()
    raise FileNotFoundError(f"Flight input does not exist: {path}")


def output_directory(data, requested, default_name):
    """Keep generated files in the flight's analysis/e20_* or analysis/e21_* directories."""
    if requested is None:
        path = data.root / "analysis" / default_name
    else:
        path = Path(requested)
        if not path.is_absolute():
            path = (REPO_DIR / path) if path.parts and path.parts[0].lower() == "logs" else Path.cwd() / path
    path = path.resolve()
    analysis = (data.root / "analysis").resolve()
    if not path.is_relative_to(analysis) or not path.relative_to(analysis).parts \
            or not path.relative_to(analysis).parts[0].startswith(("e20_", "e21_")):
        raise ValueError("Offline outputs must be under this flight's analysis/e20_* or analysis/e21_* directory")
    path.mkdir(parents=True, exist_ok=True)
    return path


def parse_intervals(text):
    """Parse comma-separated, finite, nonempty half-open boot-time intervals."""
    result = []
    for item in text.split(","):
        values = item.split(":")
        if len(values) != 2:
            raise ValueError(f"Invalid time interval: {item}")
        lo, hi = map(float, values)
        if not np.isfinite([lo, hi]).all() or hi <= lo:
            raise ValueError(f"Invalid time interval: {item}")
        result.append((lo, hi))
    return result


def validate_intervals(train, tests):
    """Reject overlap so calibration or parameter selection cannot use holdout epochs."""
    for i, first in enumerate(train + tests):
        for second in (train + tests)[i + 1:]:
            if max(first[0], second[0]) < min(first[1], second[1]):
                raise ValueError("Training/test intervals must not overlap")


def interval_mask(t, intervals):
    result = np.zeros(len(t), dtype=bool)
    for lo, hi in intervals:
        result |= (t >= lo) & (t < hi)
    return result


def vector_arguments(argv):
    """Allow the documented '--lever -0.09,0,0.31' argparse spelling."""
    args = list(sys.argv[1:] if argv is None else argv)
    result = []
    i = 0
    while i < len(args):
        if args[i] == "--lever" and i + 1 < len(args) and "," in args[i + 1]:
            result.append("--lever=" + args[i + 1])
            i += 2
        else:
            result.append(args[i])
            i += 1
    return result


def parse_lever(text):
    values = tuple(map(float, text.split(",")))
    if len(values) != 3 or not np.isfinite(values).all():
        raise ValueError("Lever must contain three finite FLU coordinates in metres")
    return values


def height_stats(positions, reference):
    dz = np.asarray(positions)[:, 2] - reference
    values = dz[np.isfinite(dz)]
    return {"epochs": len(values), "median_m": float(np.median(values)) if len(values) else None,
            "p95_abs_m": float(np.percentile(np.abs(values), 95)) if len(values) else None,
            "rmse_m": float(np.sqrt(np.mean(values ** 2))) if len(values) else None}


def usable_epochs(data):
    return np.isfinite(data.z_ref) & np.isfinite(data.yaw_rig) & np.all(np.isfinite(data.gui_xyz[:, :2]), axis=1) \
        & (np.isfinite(data.ranges).sum(axis=1) >= 4)


def epoch_selection(data, mask):
    return {"requested": int(mask.sum()), "rangefinder_stale": int((mask & ~np.isfinite(data.z_ref)).sum()),
            "fewer_than_four_ranges": int((mask & (np.isfinite(data.ranges).sum(axis=1) < 4)).sum()),
            "usable": int((mask & usable_epochs(data)).sum())}


def read_csv(session):
    """Read calibrated wire ranges only when RADIO_OK, CAL_OK and VALID are all set.

    No metadata host bias is subtracted: a<N>_mm is already firmware-calibrated
    under CAL_OK. Return enabled anchors, numeric columns and the range matrix.
    """
    with (session / "session_metadata.json").open(encoding="utf-8-sig") as fh:
        metadata = json.load(fh)
    anchors = tuple(FlightAnchor(int(a["anchor_id"]), float(a["x"]), float(a["y"]), float(a["z"]))
                    for a in metadata["anchors"] if a.get("enabled", True))
    if len(anchors) < 4 or not np.isfinite([(a.x, a.y, a.z) for a in anchors]).all():
        raise ValueError("Need at least four enabled finite anchors in metadata")
    keys = ["tag_time_us", "cycle_seq", "boot_id", "host_time_s"]
    keys += [f"{prefix}_{axis}_m" for prefix in ("pos", "raw") for axis in "xyz"]
    keys += [f"a{a.anchor_id}_{suffix}" for a in anchors for suffix in ("mm", "flags")]
    columns = {key: [] for key in keys}
    with (session / "uwb_fast_log.csv").open(encoding="utf-8-sig", newline="") as fh:
        for row in csv.DictReader(fh):
            for key in keys:
                value = row.get(key, "")
                columns[key].append(float(value) if value else math.nan)
    d = {key: np.asarray(values) for key, values in columns.items()}
    if not len(d["tag_time_us"]) or not np.isfinite(d["tag_time_us"]).all() or np.any(np.diff(d["tag_time_us"]) <= 0):
        raise ValueError("CSV must contain increasing finite TAG timestamps in a single boot")
    if len(np.unique(d["boot_id"][np.isfinite(d["boot_id"])])) > 1:
        raise ValueError("Split multi-boot recordings before fitting a clock")
    ranges = []
    for anchor in anchors:
        flags = np.nan_to_num(d[f"a{anchor.anchor_id}_flags"]).astype(np.int64)
        raw = d[f"a{anchor.anchor_id}_mm"] / 1000
        ranges.append(np.where(((flags & 0x2C) == 0x2C) & np.isfinite(raw) & (raw > .01), raw, np.nan))
    return anchors, d, np.column_stack(ranges)


def read_mcap_cache(root, session):
    """Read existing bridge/receive caches, or decode MCAP in memory without writing caches."""
    cache = root / "analysis"
    bridge_path, rx_path = cache / "px4_bridge.json", cache / "uwb_receive_times.npz"
    bridge, rx = None, None
    if bridge_path.is_file():
        with bridge_path.open(encoding="utf-8") as fh:
            bridge = json.load(fh)
    if rx_path.is_file():
        with np.load(rx_path) as archive:
            rx = archive["samples"].copy()
    if bridge is None or rx is None:
        reader = dependency("mcap.reader")
        names = ("local_position_ned", "attitude", "gps_raw_int", "distance_sensor", "attitude_quaternion", "extended_sys_state", "odometry")
        topics = {f"/px4/{name}/message": name for name in names}
        decoded, receives = {name: [] for name in names}, []
        paths = sorted((session / "rosbag2").glob("*.mcap"))
        if not paths:
            raise FileNotFoundError("Missing MCAP caches and original rosbag2/*.mcap")
        for path in paths:
            with path.open("rb") as fh:
                for _, channel, message in reader.make_reader(fh).iter_messages(log_time_order=False):
                    if channel.topic not in topics and channel.topic != "/uwb/telemetry/rangeburstmessage":
                        continue
                    if message.data[:4] != b"\x00\x01\x00\x00":
                        raise ValueError("Expected little-endian CDR std_msgs/String in MCAP")
                    length = struct.unpack_from("<I", message.data, 4)[0]
                    obj = json.loads(message.data[8:8 + length - 1])
                    if channel.topic in topics:
                        obj["host_s"] = message.log_time / 1e9
                        decoded[topics[channel.topic]].append(obj)
                    else:
                        receives.append((obj["cycle_seq"], obj["t_us"] / 1e6, message.log_time / 1e9))
        bridge = decoded if bridge is None else bridge
        rx = np.asarray(receives, dtype=float) if rx is None else rx
    return bridge, rx[np.argsort(rx[:, 0], kind="stable")]


def map_tag_to_px4_clock(columns, bridge, receive_times, excluded_elapsed=((250., 290.),)):
    """Fit the low-delay 5 s receive-time envelope, then map host epoch to PX4 boot.

    The documented Test_1 queue burst at TAG elapsed 250..290 s is excluded
    from the clock envelope only. No trajectory, calibration or holdout errors
    enter this fit. PX4 boot offset is the 1st percentile of attitude source
    time plus age, exactly as in analyze_test1.py.
    """
    if len(receive_times) != len(columns["cycle_seq"]) or not np.array_equal(receive_times[:, 0], columns["cycle_seq"]):
        raise ValueError("Receive-time cache cycle sequence does not match the CSV")
    elapsed = (columns["tag_time_us"] - columns["tag_time_us"][0]) / 1e6
    bins = []
    for start in np.arange(0, elapsed[-1] + 1e-9, 5):
        if any(lo <= start < hi for lo, hi in excluded_elapsed):
            continue
        selected = (elapsed >= start) & (elapsed < start + 5)
        if selected.sum() >= 100:
            residual = receive_times[selected, 2] - receive_times[0, 2] - elapsed[selected]
            bins.append((np.median(elapsed[selected]), np.percentile(residual, 2)))
    if len(bins) < 3:
        raise ValueError("Insufficient low-delay receive samples for the quadratic clock fit")
    bins = np.asarray(bins)
    coefficients = np.polyfit(bins[:, 0], bins[:, 1], 2)
    offsets = [v["host_s"] - v["age_s"] - v["fields"]["time_boot_ms"] / 1000
               for v in bridge.get("attitude", []) if "time_boot_ms" in v.get("fields", {})]
    if not offsets:
        raise ValueError("No source-timed PX4 attitude messages for the boot clock")
    boot_offset = float(np.percentile(offsets, 1))
    boot = receive_times[0, 2] + elapsed + np.polyval(coefficients, elapsed) - boot_offset
    audit = {"envelope_quadratic_coefficients": coefficients, "px4_boot_to_host_epoch_s": boot_offset,
             "excluded_tag_elapsed_s": excluded_elapsed, "envelope_bins": len(bins),
             "envelope_residual_p95_abs_s": float(np.percentile(np.abs(bins[:, 1] - np.polyval(coefficients, bins[:, 0])), 95))}
    return boot, audit


def read_ulog(path):
    """Lazily read the sensor, attitude, land, local and GPS datasets needed offline."""
    ulog = dependency("pyulog").ULog(str(path))
    names = ("sensor_combined", "vehicle_attitude", "distance_sensor", "vehicle_land_detected", "vehicle_local_position", "vehicle_gps_position")
    return {name: ulog.get_dataset(name).data for name in names}, ulog.initial_parameters


def _unique_series(t, values):
    t, values = np.asarray(t, dtype=float), np.asarray(values, dtype=float)
    if not len(t):
        return t, values.reshape((0, values.shape[1] if values.ndim > 1 else 1))
    good = np.isfinite(t) & np.all(np.isfinite(values.reshape((len(t), -1))), axis=1)
    t, values = t[good], values[good]
    order = np.argsort(t, kind="stable")
    t, values = t[order], values[order]
    t, indices = np.unique(t, return_index=True)
    return t, values[indices]


def interp_vectors(query, times, values, extrapolate=False):
    """Column-wise linear interpolation, NaN outside source coverage by default."""
    t, v = _unique_series(times, values)
    if not len(t):
        return np.full((len(query), np.asarray(values).shape[-1]), np.nan)
    edge = {} if extrapolate else {"left": np.nan, "right": np.nan}
    return np.column_stack([np.interp(query, t, v[:, c], **edge) for c in range(v.shape[1])])


def attitude_rotations(query, bridge, datasets):
    """Merge full-session MAVLink quaternions with higher-rate ULog attitude; FRD to NED."""
    transform = dependency("scipy.spatial.transform", "scipy")
    cached = bridge.get("attitude_quaternion", [])
    times = [r["fields"]["time_boot_ms"] / 1000 for r in cached]
    quats = [[r["fields"][f"q{i}"] for i in (2, 3, 4, 1)] for r in cached]
    data = datasets["vehicle_attitude"]
    times = np.r_[times, data.get("timestamp_sample", data["timestamp"]) / 1e6]
    quats = np.vstack((np.asarray(quats).reshape((-1, 4)), np.column_stack([data[f"q[{i}]"] for i in (1, 2, 3, 0)])))
    times, quats = _unique_series(times, quats)
    nonzero = np.linalg.norm(quats, axis=1) > 1e-8
    times, quats = times[nonzero], quats[nonzero]
    if len(times) < 2:
        raise ValueError("Insufficient valid attitude quaternions")
    return transform.Slerp(times, transform.Rotation.from_quat(quats))(np.clip(query, times[0], times[-1])).as_matrix()


def rig_attitude_rotations(query, bridge, datasets, rig_yaw_offset):
    """PX4 FRD->NED attitude expressed as body FLU->rig (E2.1)."""
    angle = rig_yaw_offset - math.pi / 2
    c, s = math.cos(angle), math.sin(angle)
    Rz = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    ned_to_enu = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
    return (Rz @ ned_to_enu) @ attitude_rotations(query, bridge, datasets) @ np.diag([1., -1., -1.])


def read_px4_diagnostics(path, intervals):
    """Lazily read numeric PX4 vibration/filter/arming evidence; no heuristic thresholds.

    Clipping fields are cumulative counters: sum positive counter increments
    inside the selected windows (also report their maxima), never sum snapshots.
    An armed_time earlier than the recording start is evidence of an already
    armed recording; it is not an inferred pre-arm noise baseline.
    """
    ulog = dependency("pyulog").ULog(str(path))
    status = {}
    for dataset in ulog.data_list:
        if dataset.name != "vehicle_imu_status":
            continue
        d = dataset.data
        selected = interval_mask(d["timestamp"] / 1e6, intervals)
        metrics = {}
        for key in ("accel_vibration_metric", "gyro_vibration_metric"):
            values = np.asarray(d.get(key, []), float)
            values = values[selected] if len(values) == len(selected) else np.empty(0)
            values = values[np.isfinite(values)]
            metrics[key] = dict(median=float(np.median(values)) if len(values) else None,
                                p95=float(np.percentile(values, 95)) if len(values) else None)
        clipping, maxima = {}, {}
        for sensor in ("accel", "gyro"):
            totals, peaks = [], []
            for axis in range(3):
                values = np.asarray(d.get(f"{sensor}_clipping[{axis}]", np.zeros(len(selected))), float)
                increments = np.maximum(np.diff(values, prepend=values[0]), 0.) if len(values) else values
                totals.append(float(increments[selected].sum()))
                peaks.append(float(values[selected].max()) if selected.any() else None)
            clipping[sensor], maxima[sensor] = totals, peaks
        status[str(dataset.multi_id)] = {**metrics, "samples": int(selected.sum()),
                                        "clipping_increments_xyz": clipping, "clipping_counter_max_xyz": maxima,
                                        "total_clipping": sum(sum(v) for v in clipping.values())}
    names = ("IMU_GYRO_NF0_FRQ", "IMU_GYRO_NF0_BW", "IMU_GYRO_CUTOFF", "IMU_ACCEL_CUTOFF",
             "IMU_GYRO_DNF_EN", "IMU_INTEG_RATE", "IMU_GYRO_RATEMAX", "SDLOG_PROFILE", "SDLOG_MODE")
    arming = {"log_start_s": ulog.start_timestamp / 1e6, "armed_at_first_record": None,
              "armed_time_s": None, "log_start_after_arm_s": None}
    for dataset in ulog.data_list:
        d = dataset.data
        if dataset.name == "actuator_armed" and len(d["timestamp"]):
            arming["armed_at_first_record"] = bool(d["armed"][0])
        if dataset.name == "vehicle_status" and len(d["timestamp"]) and "armed_time" in d and d["armed_time"][0] > 0:
            arming["armed_time_s"] = float(d["armed_time"][0] / 1e6)
            arming["log_start_after_arm_s"] = arming["log_start_s"] - arming["armed_time_s"]
    return {"vehicle_imu_status": status, "parameters": {n: ulog.initial_parameters.get(n) for n in names},
            "arming": arming, "windows_s": intervals,
            "clipping_definition": "Positive cumulative counter increments in selected windows; maxima also retained"}


def height_reference(t, rotations, bridge, datasets, parameters, lever_flu, ground_tag_height_m=.51,
                     ground_window=(40., 55.), max_age_s=.5):
    """Tilt/lever-corrected TAG height, anchored to the preflight stationary window.

    Match the original full-session MAVLink distance interpolation. Freshness
    is causal: time since the latest source sample at/before each epoch, never
    time since a future sample. Stale epochs become NaN. This is an offline
    linearly interpolated same-sensor reference and assumes a flat floor.
    """
    cached = [r for r in bridge.get("distance_sensor", []) if r["fields"].get("orientation", 25) == 25]
    if cached:
        times = np.array([r["fields"]["time_boot_ms"] / 1000 for r in cached])
        distances = np.array([r["fields"]["current_distance"] / 100 for r in cached])
        source = "MCAP MAVLink distance_sensor"
    else:
        data = datasets["distance_sensor"]
        good = np.asarray(data.get("orientation", np.full(len(data["timestamp"]), 25))) == 25
        times, distances = data["timestamp"][good] / 1e6, data["current_distance"][good]
        source = "ULog distance_sensor"
    valid = np.isfinite(distances) & (distances > 0)
    times, values = _unique_series(times[valid], distances[valid, None])
    if not len(times):
        raise ValueError("No downward rangefinder samples")
    distance = np.interp(t, times, values[:, 0])
    previous = np.searchsorted(times, t, side="right") - 1
    age = np.where(previous >= 0, t - times[np.maximum(previous, 0)], np.inf)
    tag_frd = np.asarray(lever_flu) * [1, -1, -1]
    ranger_frd = np.array([parameters.get(f"EKF2_RNG_POS_{axis}", 0.0) for axis in "XYZ"])
    tag_up = -np.einsum("nij,j->ni", rotations, tag_frd)[:, 2]
    ranger_up = -np.einsum("nij,j->ni", rotations, ranger_frd)[:, 2]
    base = distance * rotations[:, 2, 2] + tag_up - ranger_up
    stationary = (t >= ground_window[0]) & (t <= ground_window[1]) & (age <= max_age_s) & np.isfinite(base)
    if not stationary.any():
        raise ValueError("No fresh rangefinder data in the stationary ground-reference window")
    zero = float(np.median(base[stationary]))
    reference = ground_tag_height_m + base - zero
    reference[(age > max_age_s) | ~np.isfinite(base)] = np.nan
    return reference, age, {"source": source, "source_samples": len(times), "max_age_s": max_age_s,
          "stale_epochs": int((age > max_age_s).sum()), "ground_window_s": ground_window,
          "ground_tag_height_m": ground_tag_height_m, "ground_zero_base_m": zero,
          "ranger_lever_frd_m": ranger_frd, "tag_lever_flu_m": lever_flu,
          "assumptions": ["Flat floor", "Ground TAG antenna height is measured correctly",
                          "Rangefinder is the same source used for learning and Z evaluation; not absolute surveyed truth",
                          "Offline linear interpolation; freshness is measured from the previous source sample"]}


def align_2d(source, target):
    """Proper rigid 2D row-vector rotation and translation, without scale/reflection."""
    source, target = np.asarray(source), np.asarray(target)
    good = np.all(np.isfinite(source), axis=1) & np.all(np.isfinite(target), axis=1)
    source, target = source[good], target[good]
    if len(source) < 3 or np.linalg.norm(source - source.mean(axis=0)) < 1e-6:
        raise ValueError("Insufficient trajectory motion for frame alignment")
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    rotation = u @ np.diag([1., np.linalg.det(u @ vt)]) @ vt
    translation = b - a @ rotation
    error = np.linalg.norm(source @ rotation + translation - target, axis=1)
    return rotation, translation, {"epochs": len(error), "rmse_m": float(np.sqrt(np.mean(error ** 2))),
                                   "p95_m": float(np.percentile(error, 95)), "rotation": rotation, "translation_m": translation}


def estimate_rig_yaw_offset(local_tag_enu, uwb_xy, selected):
    """Fit ENU-to-rig rotation on training epochs; yaw_rig = offset - yaw_ned."""
    rotation, _, audit = align_2d(local_tag_enu[selected], uwb_xy[selected])
    angle = math.atan2(rotation[0, 1], rotation[0, 0])
    offset = math.pi / 2 + angle
    audit["rig_yaw_offset_rad"] = offset
    audit["fit_scope"] = "training intervals only"
    return offset, audit


def read_imu(datasets):
    """sensor_combined accel sample timestamps and gyro, FRD->FLU exactly once.

    Accelerometer timestamp = timestamp + accelerometer_timestamp_relative (us).
    Gyro is interpolated from its own timestamp onto those accel times. Duplicate
    sample times are removed; genuine irregular dt is retained.
    """
    data = datasets["sensor_combined"]
    stamp = np.asarray(data["timestamp"], dtype=float)
    relative = np.asarray(data.get("accelerometer_timestamp_relative", np.zeros(len(stamp))), dtype=float)
    good = np.isfinite(relative) & (relative != np.iinfo(np.int32).max)
    times = (stamp[good] + relative[good]) / 1e6
    acc = np.column_stack([data[f"accelerometer_m_s2[{i}]"][good] for i in range(3)])
    times, acc = _unique_series(times, acc)
    gyro = np.column_stack([data[f"gyro_rad[{i}]"] for i in range(3)])
    gyro = interp_vectors(times, stamp / 1e6, gyro, extrapolate=True)
    flip = np.array([1., -1., -1.])
    return times, acc * flip, gyro * flip


def load_flight(path, train, lever_flu=(-.09, 0., .31), ground_window=(40., 55.), ground_tag_height_m=.51,
                max_rangefinder_age_s=.5):
    """Load one recording and construct frozen references using training-only XY alignment."""
    root = resolve_input(path)
    if (root / "uwb_fast_log.csv").is_file():
        session, root = root, root.parent
    else:
        sessions = sorted(p.parent for p in root.glob("*/uwb_fast_log.csv"))
        if len(sessions) != 1:
            raise ValueError("Expected exactly one GUI session under the flight directory")
        session = sessions[0]
    ulogs = sorted(root.glob("*.ulg"))
    if len(ulogs) != 1:
        raise ValueError("Expected exactly one ULog under the flight directory")
    anchors, columns, ranges = read_csv(session)
    bridge, rx = read_mcap_cache(root, session)
    t, clock = map_tag_to_px4_clock(columns, bridge, rx)
    datasets, parameters = read_ulog(ulogs[0])
    rotations = attitude_rotations(t, bridge, datasets)
    z_ref, age, reference = height_reference(t, rotations, bridge, datasets, parameters, lever_flu,
                                            ground_tag_height_m, ground_window, max_rangefinder_age_s)
    gui = np.column_stack([columns[f"pos_{axis}_m"] for axis in "xyz"])
    irls = np.column_stack([columns[f"raw_{axis}_m"] for axis in "xyz"])
    local = datasets["vehicle_local_position"]
    local_xy = interp_vectors(t, local.get("timestamp_sample", local["timestamp"]) / 1e6,
                               np.column_stack([local["y"], local["x"]]))
    tag_frd = np.asarray(lever_flu) * [1., -1., -1.]
    tag_ned = np.einsum("nij,j->ni", rotations, tag_frd)
    local_xy += tag_ned[:, [1, 0]]
    selected = interval_mask(t, train)
    yaw_offset, alignment = estimate_rig_yaw_offset(local_xy, gui[:, :2], selected)
    yaw_ned = np.arctan2(rotations[:, 1, 0], rotations[:, 0, 0])
    yaw_rig = (yaw_offset - yaw_ned + math.pi) % (2 * math.pi) - math.pi
    gps = datasets["vehicle_gps_position"]
    gps_t = gps["timestamp"] / 1e6
    gps_train = interval_mask(gps_t, train)
    if not gps_train.any():
        raise ValueError("No GPS samples in training intervals for horizontal reference alignment")
    lat0, lon0 = np.median(gps["latitude_deg"][gps_train]), np.median(gps["longitude_deg"][gps_train])
    latitude = math.radians(lat0)
    w = math.sqrt(1 - 6.69437999014e-3 * math.sin(latitude) ** 2)
    north_radius, east_radius = 6378137. * (1 - 6.69437999014e-3) / w ** 3, 6378137. / w
    gps_enu = np.column_stack((np.radians(gps["longitude_deg"] - lon0) * east_radius * math.cos(latitude),
                               np.radians(gps["latitude_deg"] - lat0) * north_radius))
    gps_xy = interp_vectors(t, gps_t, gps_enu)
    # Same known 18 cm fore-aft GNSS-to-TAG separation as analyze_test1.py.
    gps_xy += np.einsum("nij,j->ni", rotations, np.array([-.18, 0., 0.]))[:, [1, 0]]
    rotation, shift, gps_alignment = align_2d(gps_xy[selected], gui[selected, :2])
    gps_xy = gps_xy @ rotation + shift
    imu_t, imu_acc, imu_gyro = read_imu(datasets)
    audit = {"sources": {"csv": session / "uwb_fast_log.csv", "ulog": ulogs[0], "cache_directory": root / "analysis"},
             "clock": clock, "height_reference": reference, "yaw_alignment": alignment,
             "gps_alignment": {**gps_alignment, "fit_scope": "training intervals only", "reference": "aligned GNSS, not surveyed truth"},
             "imu": {"samples": len(imu_t), "frame_conversion": "FRD to FLU once: diag(1,-1,-1)",
                     "timestamp": "timestamp + accelerometer_timestamp_relative", "dt_p50_p95_max_s": np.percentile(np.diff(imu_t), [50, 95, 100])},
             "rows": len(t), "accepted_ranges": int(np.isfinite(ranges).sum()), "range_rule": "RADIO_OK + CAL_OK + VALID; no second bias subtraction"}
    return FlightData(root, session, anchors, t, ranges, gui, irls, z_ref, age, yaw_rig, gps_xy,
                      imu_t, imu_acc, imu_gyro, audit)
