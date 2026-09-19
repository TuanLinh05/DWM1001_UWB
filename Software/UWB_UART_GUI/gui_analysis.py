"""Numerical helpers for the native UWB diagnostic GUI.

The module intentionally depends only on the Python standard library so the
field GUI can keep running on a clean Windows machine with just PySerial.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
from typing import Iterable, Sequence


FRESH_AGE_MS = 200
MAX_ANCHORS = 8


@dataclass(frozen=True)
class AnchorPosition:
    anchor_id: int
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class TelemetryPoint:
    received_s: float
    tag_time_ms: int
    valid: bool
    status: int
    age_ms: int
    raw_mm: int
    firmware_mm: int | None
    host_mm: int | None
    fpp_dbm: float | None


@dataclass(frozen=True)
class SeriesSummary:
    count: int
    mean: float | None
    stddev: float | None
    minimum: float | None
    maximum: float | None
    p05: float | None
    median: float | None
    p95: float | None


@dataclass(frozen=True)
class AnchorMetrics:
    anchor_id: int
    total_frames: int
    observed: int
    fresh: int
    invalid: int
    stale: int
    missing: int
    availability_pct: float
    raw: SeriesSummary
    firmware: SeriesSummary
    host: SeriesSummary
    filter_delta: SeriesSummary
    fpp: SeriesSummary
    age: SeriesSummary


@dataclass(frozen=True)
class CalibrationResult:
    anchor_id: int
    reference_mm: float
    samples: int
    mean_raw_mm: float
    offset_mm: float
    stddev_mm: float
    p05_mm: float
    p95_mm: float
    drift_mm: float
    median_fpp_dbm: float | None


@dataclass(frozen=True)
class PositionSolution:
    x: float
    y: float
    z: float
    rms_m: float
    max_residual_m: float
    used_anchor_ids: tuple[int, ...]
    excluded_anchor_ids: tuple[int, ...]
    dimension: int
    geometry_condition: float


@dataclass(frozen=True)
class SignalStatistics:
    sample_count: int
    duration_s: float
    sample_rate_hz: float | None
    sample_jitter_ms: float | None
    reference_mm: float
    reference_is_mean: bool
    mean_mm: float
    median_mm: float
    stddev_mm: float
    minimum_mm: float
    maximum_mm: float
    bias_mm: float
    mae_mm: float
    rmse_mm: float
    p95_abs_error_mm: float
    p99_abs_error_mm: float
    drift_mm_per_min: float | None


@dataclass(frozen=True)
class HistogramData:
    centers: tuple[float, ...]
    density: tuple[float, ...]
    normal_density: tuple[float, ...]
    bin_width: float


@dataclass(frozen=True)
class RegularSeries:
    values: tuple[float, ...]
    sample_rate_hz: float
    source_samples: int
    gap_count: int


DEFAULT_LAYOUT_4 = (
    AnchorPosition(1, 0.0, 0.0, 0.0),
    AnchorPosition(2, 5.0, 0.0, 0.0),
    AnchorPosition(3, 2.5, 4.0, 0.0),
    AnchorPosition(4, 5.0, 4.0, 0.0),
)

DEFAULT_LAYOUT_8 = (
    AnchorPosition(1, 0.0, 0.0, 0.0),
    AnchorPosition(2, 5.0, 0.0, 0.0),
    AnchorPosition(3, 0.0, 4.0, 0.0),
    AnchorPosition(4, 5.0, 4.0, 0.0),
    AnchorPosition(5, 0.0, 0.0, 3.0),
    AnchorPosition(6, 5.0, 0.0, 3.0),
    AnchorPosition(7, 0.0, 4.0, 3.0),
    AnchorPosition(8, 5.0, 4.0, 3.0),
)


def percentile(values: Sequence[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    fraction = min(max(float(fraction), 0.0), 1.0)
    location = fraction * (len(ordered) - 1)
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    if lower == upper:
        return ordered[lower]
    weight = location - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize(values: Iterable[float]) -> SeriesSummary:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return SeriesSummary(0, None, None, None, None, None, None, None)
    return SeriesSummary(
        count=len(finite),
        mean=statistics.fmean(finite),
        stddev=statistics.pstdev(finite) if len(finite) > 1 else 0.0,
        minimum=min(finite),
        maximum=max(finite),
        p05=percentile(finite, 0.05),
        median=percentile(finite, 0.50),
        p95=percentile(finite, 0.95),
    )


def clean_time_series(
    times_s: Sequence[float], values: Sequence[float]
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Return finite, time-ordered samples with duplicate timestamps collapsed."""
    samples = sorted(
        (float(timestamp), float(value))
        for timestamp, value in zip(times_s, values)
        if math.isfinite(float(timestamp)) and math.isfinite(float(value))
    )
    clean_times: list[float] = []
    clean_values: list[float] = []
    for timestamp, value in samples:
        if clean_times and timestamp == clean_times[-1]:
            clean_values[-1] = value
        elif not clean_times or timestamp > clean_times[-1]:
            clean_times.append(timestamp)
            clean_values.append(value)
    return tuple(clean_times), tuple(clean_values)


def estimate_sample_timing(times_s: Sequence[float]) -> tuple[float | None, float | None]:
    finite = sorted(float(value) for value in times_s if math.isfinite(float(value)))
    intervals = [later - earlier for earlier, later in zip(finite, finite[1:]) if later > earlier]
    if not intervals:
        return None, None
    median_interval = percentile(intervals, 0.5)
    assert median_interval is not None
    if median_interval <= 0.0:
        return None, None
    jitter = statistics.pstdev(intervals) * 1000.0 if len(intervals) > 1 else 0.0
    return 1.0 / median_interval, jitter


def signal_statistics(
    times_s: Sequence[float],
    values_mm: Sequence[float],
    reference_mm: float | None = None,
) -> SignalStatistics | None:
    times, values = clean_time_series(times_s, values_mm)
    if not values:
        return None
    summary = summarize(values)
    assert summary.mean is not None and summary.median is not None
    assert summary.stddev is not None and summary.minimum is not None and summary.maximum is not None
    reference_is_mean = reference_mm is None
    reference = summary.mean if reference_mm is None else float(reference_mm)
    if not math.isfinite(reference):
        raise ValueError("Khoảng cách tham chiếu phải là số hữu hạn")
    errors = [value - reference for value in values]
    absolute_errors = [abs(value) for value in errors]
    rate, jitter = estimate_sample_timing(times)
    duration = times[-1] - times[0] if len(times) > 1 else 0.0
    slope_per_minute: float | None = None
    if len(times) > 1 and duration > 0.0:
        center_t = statistics.fmean(times)
        center_y = summary.mean
        denominator = sum((timestamp - center_t) ** 2 for timestamp in times)
        if denominator > 0.0:
            slope_per_minute = (
                sum((timestamp - center_t) * (value - center_y)
                    for timestamp, value in zip(times, values))
                / denominator
                * 60.0
            )
    return SignalStatistics(
        sample_count=len(values),
        duration_s=duration,
        sample_rate_hz=rate,
        sample_jitter_ms=jitter,
        reference_mm=reference,
        reference_is_mean=reference_is_mean,
        mean_mm=summary.mean,
        median_mm=summary.median,
        stddev_mm=summary.stddev,
        minimum_mm=summary.minimum,
        maximum_mm=summary.maximum,
        bias_mm=statistics.fmean(errors),
        mae_mm=statistics.fmean(absolute_errors),
        rmse_mm=math.sqrt(statistics.fmean(value * value for value in errors)),
        p95_abs_error_mm=percentile(absolute_errors, 0.95) or 0.0,
        p99_abs_error_mm=percentile(absolute_errors, 0.99) or 0.0,
        drift_mm_per_min=slope_per_minute,
    )


def histogram_density(values: Sequence[float], bins: int | None = None) -> HistogramData:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return HistogramData((), (), (), 0.0)
    minimum, maximum = min(finite), max(finite)
    if minimum == maximum:
        width = max(abs(minimum) * 0.02, 1.0)
        minimum -= width / 2.0
        maximum += width / 2.0
        bins = 1 if bins is None else max(1, int(bins))
    if bins is None:
        q25 = percentile(finite, 0.25)
        q75 = percentile(finite, 0.75)
        iqr = (q75 or 0.0) - (q25 or 0.0)
        fd_width = 2.0 * iqr / (len(finite) ** (1.0 / 3.0)) if iqr > 0.0 else 0.0
        if fd_width > 0.0:
            fd_bins = int(math.ceil((maximum - minimum) / fd_width))
        else:
            fd_bins = 1
        # Freedman-Diaconis can look extremely coarse for smooth/ramped UWB
        # captures. A display-oriented floor of 2*sqrt(N) keeps their visible
        # structure while still scaling the resolution with sample count.
        display_bins = int(math.ceil(2.0 * math.sqrt(len(finite))))
        bins = min(max(fd_bins, display_bins, 8), 80, len(finite))
    else:
        bins = min(max(int(bins), 1), 100)
    width = (maximum - minimum) / bins
    if width <= 0.0:
        width = 1.0
    counts = [0] * bins
    for value in finite:
        index = min(int((value - minimum) / width), bins - 1)
        counts[index] += 1
    centers = tuple(minimum + (index + 0.5) * width for index in range(bins))
    density = tuple(count / (len(finite) * width) for count in counts)
    mean = statistics.fmean(finite)
    sigma = statistics.pstdev(finite) if len(finite) > 1 else 0.0
    if sigma > 0.0:
        scale = 1.0 / (sigma * math.sqrt(2.0 * math.pi))
        normal = tuple(scale * math.exp(-0.5 * ((center - mean) / sigma) ** 2)
                       for center in centers)
    else:
        normal = tuple(0.0 for _ in centers)
    return HistogramData(centers, density, normal, width)


def regularize_time_series(
    times_s: Sequence[float],
    values: Sequence[float],
    *,
    maximum_gap_factor: float = 1.5,
    maximum_samples: int = 16384,
) -> RegularSeries | None:
    """Resample the longest contiguous region to a uniform grid for ADEV/PSD."""
    times, clean_values = clean_time_series(times_s, values)
    if len(times) < 4:
        return None
    intervals = [later - earlier for earlier, later in zip(times, times[1:])]
    dt = percentile(intervals, 0.5)
    if dt is None or dt <= 0.0:
        return None
    threshold = dt * max(maximum_gap_factor, 1.1)
    regions: list[tuple[int, int]] = []
    start = 0
    gap_count = 0
    for index, interval in enumerate(intervals):
        if interval > threshold:
            regions.append((start, index + 1))
            start = index + 1
            gap_count += 1
    regions.append((start, len(times)))
    region_start, region_end = max(regions, key=lambda item: (item[1] - item[0], item[1]))
    region_times = times[region_start:region_end]
    region_values = clean_values[region_start:region_end]
    if len(region_times) < 4:
        return None
    region_intervals = [
        later - earlier for earlier, later in zip(region_times, region_times[1:])
    ]
    median_absolute_jitter = percentile(
        [abs(interval - dt) for interval in region_intervals], 0.5
    ) or 0.0
    if median_absolute_jitter / dt > 0.20:
        return None
    full_grid_count = int(round((region_times[-1] - region_times[0]) / dt)) + 1
    if full_grid_count < 4:
        return None
    grid_count = min(full_grid_count, maximum_samples)
    if grid_count < full_grid_count:
        grid_start = region_times[-1] - (grid_count - 1) * dt
    else:
        grid_start = region_times[0]
    resampled: list[float] = []
    source_index = 0
    while source_index + 1 < len(region_times) and region_times[source_index + 1] < grid_start:
        source_index += 1
    for grid_index in range(grid_count):
        timestamp = grid_start + grid_index * dt
        while source_index + 1 < len(region_times) and region_times[source_index + 1] < timestamp:
            source_index += 1
        if source_index + 1 >= len(region_times):
            resampled.append(region_values[-1])
            continue
        left_t, right_t = region_times[source_index], region_times[source_index + 1]
        left_y, right_y = region_values[source_index], region_values[source_index + 1]
        fraction = 0.0 if right_t == left_t else (timestamp - left_t) / (right_t - left_t)
        fraction = min(max(fraction, 0.0), 1.0)
        resampled.append(left_y + fraction * (right_y - left_y))
    return RegularSeries(tuple(resampled), 1.0 / dt, len(region_times), gap_count)


def overlapping_allan_deviation(
    values: Sequence[float], sample_rate_hz: float, maximum_points: int = 24
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if len(finite) < 4 or not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
        return (), ()
    maximum_cluster = len(finite) // 4
    if maximum_cluster < 1:
        return (), ()
    if maximum_cluster == 1:
        clusters = [1]
    else:
        steps = max(2, maximum_points)
        clusters = sorted({
            max(1, min(maximum_cluster, int(round(10 ** (
                index * math.log10(maximum_cluster) / (steps - 1)
            )))))
            for index in range(steps)
        })
    prefix = [0.0]
    for value in finite:
        prefix.append(prefix[-1] + value)
    taus: list[float] = []
    deviations: list[float] = []
    for cluster in clusters:
        differences = []
        for start in range(0, len(finite) - 2 * cluster + 1):
            first = (prefix[start + cluster] - prefix[start]) / cluster
            second = (prefix[start + 2 * cluster] - prefix[start + cluster]) / cluster
            differences.append(second - first)
        if differences:
            deviation = math.sqrt(0.5 * statistics.fmean(value * value for value in differences))
            if deviation >= 0.0 and math.isfinite(deviation):
                taus.append(cluster / sample_rate_hz)
                deviations.append(deviation)
    return tuple(taus), tuple(deviations)


def _fft(values: Sequence[complex]) -> list[complex]:
    length = len(values)
    if length == 0 or length & (length - 1):
        raise ValueError("FFT length must be a power of two")
    output = [complex(value) for value in values]
    bit = 0
    for index in range(1, length):
        mask = length >> 1
        while bit & mask:
            bit ^= mask
            mask >>= 1
        bit ^= mask
        if index < bit:
            output[index], output[bit] = output[bit], output[index]
    block = 2
    while block <= length:
        angle = -2.0 * math.pi / block
        root = complex(math.cos(angle), math.sin(angle))
        half = block // 2
        for base in range(0, length, block):
            factor = 1.0 + 0.0j
            for offset in range(half):
                even = output[base + offset]
                odd = factor * output[base + offset + half]
                output[base + offset] = even + odd
                output[base + offset + half] = even - odd
                factor *= root
        block *= 2
    return output


def welch_psd(
    values: Sequence[float],
    sample_rate_hz: float,
    *,
    maximum_fft: int = 1024,
    maximum_segments: int = 8,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if len(finite) < 16 or not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
        return (), ()
    limit = min(len(finite), max(16, int(maximum_fft)))
    segment_length = 1 << int(math.floor(math.log2(limit)))
    while segment_length >= 32:
        candidate_step = segment_length // 2
        candidate_count = 1 + (len(finite) - segment_length) // candidate_step
        if candidate_count >= 3:
            break
        segment_length //= 2
    if segment_length < 16:
        return (), ()
    step = max(1, segment_length // 2)
    starts = list(range(0, len(finite) - segment_length + 1, step))
    if len(starts) > maximum_segments:
        if maximum_segments <= 1:
            starts = [starts[-1]]
        else:
            starts = [starts[round(index * (len(starts) - 1) / (maximum_segments - 1))]
                      for index in range(maximum_segments)]
    window = [0.5 - 0.5 * math.cos(2.0 * math.pi * index / (segment_length - 1))
              for index in range(segment_length)]
    window_power = sum(value * value for value in window)
    bins = segment_length // 2 + 1
    accumulated = [0.0] * bins
    for start in starts:
        segment = finite[start:start + segment_length]
        mean = statistics.fmean(segment)
        transformed = _fft([
            complex((value - mean) * window[index], 0.0)
            for index, value in enumerate(segment)
        ])
        for index in range(bins):
            power = abs(transformed[index]) ** 2 / (sample_rate_hz * window_power)
            if index not in (0, segment_length // 2):
                power *= 2.0
            accumulated[index] += power
    divisor = max(len(starts), 1)
    frequencies = tuple(index * sample_rate_hz / segment_length for index in range(bins))
    density = tuple(value / divisor for value in accumulated)
    return frequencies, density


def analyze_anchor(
    anchor_id: int,
    points: Sequence[TelemetryPoint],
    total_frames: int,
    fresh_age_ms: int = FRESH_AGE_MS,
) -> AnchorMetrics:
    observed = min(len(points), total_frames)
    fresh_points = [point for point in points if point.valid and point.age_ms <= fresh_age_ms]
    invalid = sum(not point.valid for point in points)
    stale = sum(point.valid and point.age_ms > fresh_age_ms for point in points)
    missing = max(total_frames - observed, 0)
    total = max(total_frames, 0)
    availability = len(fresh_points) * 100.0 / total if total else 0.0

    def usable(value: int | None) -> bool:
        return value is not None and value > 0

    raw = [point.raw_mm for point in fresh_points if usable(point.raw_mm)]
    firmware = [point.firmware_mm for point in fresh_points if usable(point.firmware_mm)]
    host = [point.host_mm for point in fresh_points if usable(point.host_mm)]
    delta = [
        abs(point.raw_mm - point.host_mm)
        for point in fresh_points
        if usable(point.raw_mm) and usable(point.host_mm)
    ]
    fpp = [point.fpp_dbm for point in fresh_points if point.fpp_dbm is not None]
    age = [point.age_ms for point in fresh_points]
    return AnchorMetrics(
        anchor_id=anchor_id,
        total_frames=total,
        observed=observed,
        fresh=len(fresh_points),
        invalid=invalid,
        stale=stale,
        missing=missing,
        availability_pct=availability,
        raw=summarize(raw),
        firmware=summarize(value for value in firmware if value is not None),
        host=summarize(value for value in host if value is not None),
        filter_delta=summarize(delta),
        fpp=summarize(value for value in fpp if value is not None),
        age=summarize(age),
    )


def calibration_result(
    anchor_id: int,
    reference_mm: float,
    points: Sequence[TelemetryPoint],
) -> CalibrationResult:
    usable = [point for point in points if point.raw_mm > 0]
    if not usable:
        raise ValueError("Không có mẫu raw hợp lệ để calibration")
    raw_values = [float(point.raw_mm) for point in usable]
    raw_summary = summarize(raw_values)
    assert raw_summary.mean is not None
    assert raw_summary.stddev is not None
    assert raw_summary.p05 is not None
    assert raw_summary.p95 is not None
    head = raw_values[: max(1, len(raw_values) // 5)]
    tail = raw_values[-max(1, len(raw_values) // 5):]
    fpp = summarize(
        point.fpp_dbm for point in usable if point.fpp_dbm is not None
    )
    return CalibrationResult(
        anchor_id=anchor_id,
        reference_mm=float(reference_mm),
        samples=len(raw_values),
        mean_raw_mm=raw_summary.mean,
        offset_mm=float(reference_mm) - raw_summary.mean,
        stddev_mm=raw_summary.stddev,
        p05_mm=raw_summary.p05,
        p95_mm=raw_summary.p95,
        drift_mm=statistics.fmean(tail) - statistics.fmean(head),
        median_fpp_dbm=fpp.median,
    )


def save_layout(path: Path, layout: Sequence[AnchorPosition]) -> None:
    payload = {
        "schema": 1,
        "coordinate_system": "ENU",
        "unit": "m",
        "anchors": [asdict(anchor) for anchor in sorted(layout, key=lambda item: item.anchor_id)],
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def load_layout(path: Path) -> tuple[AnchorPosition, ...]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    anchors = payload.get("anchors") if isinstance(payload, dict) else payload
    if not isinstance(anchors, list):
        raise ValueError("File layout không có danh sách anchors")
    result: list[AnchorPosition] = []
    seen: set[int] = set()
    for item in anchors:
        if not isinstance(item, dict):
            continue
        anchor_id = int(item.get("anchor_id", item.get("id", 0)))
        if not 1 <= anchor_id <= MAX_ANCHORS or anchor_id in seen:
            continue
        values = (float(item.get("x", 0)), float(item.get("y", 0)), float(item.get("z", 0)))
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"Tọa độ A{anchor_id} không hữu hạn")
        result.append(AnchorPosition(anchor_id, *values))
        seen.add(anchor_id)
    if len(result) < 3:
        raise ValueError("Layout cần ít nhất 3 anchor")
    return tuple(sorted(result, key=lambda item: item.anchor_id))


def _solve_linear(matrix: list[list[float]], vector: list[float]) -> list[float] | None:
    size = len(vector)
    augmented = [row[:] + [vector[index]] for index, row in enumerate(matrix)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-10:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return [augmented[index][-1] for index in range(size)]


def _matrix_condition_diagonal(matrix: list[list[float]]) -> float:
    diagonal = [abs(matrix[index][index]) for index in range(len(matrix))]
    smallest = min(diagonal, default=0.0)
    return max(diagonal, default=0.0) / smallest if smallest > 1e-10 else math.inf


def _fit_position(
    layout: dict[int, AnchorPosition],
    ranges_m: dict[int, float],
    used: Sequence[int],
    dimension: int,
    fixed_z: float,
) -> tuple[list[float], dict[int, float], float] | None:
    estimate = [
        statistics.fmean(getattr(layout[anchor_id], axis) for anchor_id in used)
        for axis in ("x", "y", "z")[:dimension]
    ]
    normal: list[list[float]] = []
    for _ in range(20):
        residuals: dict[int, float] = {}
        jacobian: dict[int, list[float]] = {}
        for anchor_id in used:
            anchor = layout[anchor_id]
            z = estimate[2] if dimension == 3 else fixed_z
            dx, dy, dz = estimate[0] - anchor.x, estimate[1] - anchor.y, z - anchor.z
            distance = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
            residuals[anchor_id] = distance - ranges_m[anchor_id]
            jacobian[anchor_id] = [dx / distance, dy / distance] + ([dz / distance] if dimension == 3 else [])
        absolute = sorted(abs(value) for value in residuals.values())
        robust_scale = max(percentile(absolute, 0.50) or 0.0, 0.04)
        huber_limit = max(2.5 * robust_scale, 0.12)
        normal = [[0.0] * dimension for _ in range(dimension)]
        rhs = [0.0] * dimension
        for anchor_id in used:
            residual = residuals[anchor_id]
            row = jacobian[anchor_id]
            weight = min(1.0, huber_limit / max(abs(residual), 1e-9))
            for i in range(dimension):
                rhs[i] -= weight * row[i] * residual
                for j in range(dimension):
                    normal[i][j] += weight * row[i] * row[j]
        step = _solve_linear(normal, rhs)
        if step is None:
            return None
        estimate = [value + delta for value, delta in zip(estimate, step)]
        if math.sqrt(sum(delta * delta for delta in step)) < 1e-5:
            break

    residuals = {}
    for anchor_id in used:
        anchor = layout[anchor_id]
        z = estimate[2] if dimension == 3 else fixed_z
        distance = math.dist((estimate[0], estimate[1], z), (anchor.x, anchor.y, anchor.z))
        residuals[anchor_id] = distance - ranges_m[anchor_id]
    return estimate, residuals, _matrix_condition_diagonal(normal)


def solve_position(
    layout_values: Sequence[AnchorPosition],
    ranges_mm: dict[int, int | float],
    dimension: int = 2,
    fixed_z: float = 0.0,
) -> PositionSolution | None:
    if dimension not in (2, 3):
        raise ValueError("dimension phải là 2 hoặc 3")
    layout = {anchor.anchor_id: anchor for anchor in layout_values}
    ranges_m = {
        anchor_id: float(distance) / 1000.0
        for anchor_id, distance in ranges_mm.items()
        if anchor_id in layout and float(distance) > 0 and math.isfinite(float(distance))
    }
    minimum = dimension + 1
    if len(ranges_m) < minimum:
        return None
    used = sorted(ranges_m)
    excluded: list[int] = []
    fit = _fit_position(layout, ranges_m, used, dimension, fixed_z)
    if fit is None:
        return None

    estimate, residuals, condition = fit
    if len(used) > minimum:
        worst = max(residuals, key=lambda anchor_id: abs(residuals[anchor_id]))
        remaining_abs = [abs(value) for anchor_id, value in residuals.items() if anchor_id != worst]
        threshold = max(0.35, 4.0 * (percentile(remaining_abs, 0.50) or 0.0))
        if abs(residuals[worst]) > threshold:
            candidate = [anchor_id for anchor_id in used if anchor_id != worst]
            refit = _fit_position(layout, ranges_m, candidate, dimension, fixed_z)
            if refit is not None:
                estimate, residuals, condition = refit
                excluded.append(worst)
                used = candidate

    residual_values = list(residuals.values())
    rms = math.sqrt(statistics.fmean(value * value for value in residual_values))
    z = estimate[2] if dimension == 3 else fixed_z
    return PositionSolution(
        x=estimate[0],
        y=estimate[1],
        z=z,
        rms_m=rms,
        max_residual_m=max(abs(value) for value in residual_values),
        used_anchor_ids=tuple(used),
        excluded_anchor_ids=tuple(excluded),
        dimension=dimension,
        geometry_condition=condition,
    )
