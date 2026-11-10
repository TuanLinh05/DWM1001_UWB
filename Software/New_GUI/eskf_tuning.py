"""E2.1 — in-flight IMU noise, ESKF-A tuning and consistency diagnostics (offline).

Plan: Plan/E2_1_YEU_CAU_TINH_CHINH_ESKF_THEO_RUNG_KHI_BAY.md (sections 3-4).
Contract: tests/test_e21_imu_noise.py, tests/test_e21_eskf_tuning.py. Do not loosen the tests.

Why: on Test_1 the onboard noise model (BMI088 datasheet, na = 1.7e-3 m/s^2/sqrtHz) is
~500x below the real in-flight vibration (per-sample accel std 2-7 m/s^2, gyro 8-13 deg/s),
so ESKF-A becomes overconfident, gates out good ranges and diverges. Q must come from the
IMU stream the filter actually consumes (rate and decimation included).

Conventions (keep them, the tests rely on them):
  * IMU arrays: t (N,) seconds, acc (N, 3) specific force body FLU m/s^2, gyro (N, 3) rad/s FLU.
  * "per-sample std" = std of the signal minus its centered moving mean over detrend_s seconds,
    computed per contiguous segment (a gap > 5 median dt or an interval edge splits segments,
    nothing is bridged); multiply by sqrt(n/(n-1)) for the n-sample window.
  * density = worst-axis per-sample std / sqrt(stream_hz), stream_hz = 1 / median(dt).
    That is the white-noise density the ESKF's Q = n^2 dt needs for that stream.
Pure numpy (no scipy needed). No file IO here; tools/imu_noise_report.py does the IO.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math
from typing import Any, Sequence

import numpy as np  # noqa: F401

from eskf_replay import EskfParams

TUNING_VERSION = "eskf-tuning-v1"
VIBRATION_ACC_WARN = 1.0                    # m/s^2 per-sample std, worst axis -> warning VIBRATION_HIGH
VIBRATION_GYRO_WARN = math.radians(3.0)     # rad/s per-sample std, worst axis -> warning VIBRATION_HIGH
MIN_SAMPLES = 500                           # fewer residual samples -> warning TOO_FEW_SAMPLES


@dataclass(frozen=True)
class ImuNoise:
    stream_hz: float                        # 1 / median(dt) of the analysed stream
    samples: int                            # residual samples used (after detrend edges)
    duration_s: float                       # sum of analysed segment durations
    acc_std: tuple[float, float, float]     # per-sample std, body FLU (m/s^2)
    gyro_std: tuple[float, float, float]    # per-sample std, body FLU (rad/s)
    acc_density: float                      # max(acc_std) / sqrt(stream_hz)   (m/s^2/sqrtHz)
    gyro_density: float                     # max(gyro_std) / sqrt(stream_hz)  (rad/s/sqrtHz)
    acc_trend_std: tuple[float, float, float]   # std of the removed moving mean (motion + low-frequency alias)
    acc_peaks_hz: tuple[float, ...]         # strongest spectral peaks >= 2 Hz of the worst accel axis (apparent)
    gyro_peaks_hz: tuple[float, ...]        # same for the worst gyro axis
    warnings: tuple[str, ...]               # TOO_FEW_SAMPLES, IRREGULAR_RATE, VIBRATION_HIGH
    version: str = TUNING_VERSION

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def decimate_stream(t, acc, gyro, stream_hz: float, mode: str = "pick"):
    """Emulate a lower-rate IMU stream (e.g. MAVLink HIGHRES_IMU at 50 Hz).

    Bins: floor((t - t[0]) * stream_hz). mode "pick": the first sample of every non-empty bin,
    returned unchanged (what a rate-limited MAVLink stream does). mode "mean": the bin average of
    t, acc and gyro (what an averaging bridge would publish). Returns (t, acc, gyro) arrays.
    ValueError for stream_hz <= 0 / non-finite or an unknown mode. Input order does not matter.
    """
    if not math.isfinite(stream_hz) or stream_hz <= 0 or mode not in ("pick", "mean"):
        raise ValueError("Expected a finite positive stream rate and mode pick or mean")
    t, acc, gyro = _stream(t, acc, gyro)
    if not len(t):
        return t, acc, gyro
    bins = np.floor((t - t[0]) * stream_hz).astype(np.int64)
    starts = np.r_[0, np.flatnonzero(np.diff(bins)) + 1]
    if mode == "pick":
        return t[starts], acc[starts], gyro[starts]
    counts = np.diff(np.r_[starts, len(t)])
    return (np.add.reduceat(t, starts) / counts,
            np.add.reduceat(acc, starts, axis=0) / counts[:, None],
            np.add.reduceat(gyro, starts, axis=0) / counts[:, None])


def _stream(t, acc, gyro):
    """Finite, deterministically ordered samples, with duplicate timestamps removed."""
    t, acc, gyro = np.asarray(t, float), np.asarray(acc, float), np.asarray(gyro, float)
    if t.ndim != 1 or acc.shape != (len(t), 3) or gyro.shape != acc.shape:
        raise ValueError("Expected IMU arrays with shapes (N,), (N,3), (N,3)")
    good = np.isfinite(t) & np.isfinite(acc).all(axis=1) & np.isfinite(gyro).all(axis=1)
    t, acc, gyro = t[good], acc[good], gyro[good]
    # Payload tie breaks make duplicate handling independent of input order.
    order = np.lexsort(tuple(np.c_[acc, gyro][:, k] for k in range(5, -1, -1)) + (t,))
    t, acc, gyro = t[order], acc[order], gyro[order]
    unique = np.r_[True, np.diff(t) > 0] if len(t) else np.zeros(0, bool)
    return t[unique], acc[unique], gyro[unique]


def periodogram(values, hz):
    """Hann, half-overlapping averaged PSD; no scipy dependency."""
    values = np.asarray(values, float)
    n = min(256, len(values))
    if n < 4:
        return np.empty(0), np.empty(0)
    window = np.hanning(n)
    blocks = np.array([values[i:i+n] for i in range(0, len(values)-n+1, max(1, n//2))])
    blocks -= blocks.mean(axis=1, keepdims=True)
    psd = np.mean(np.abs(np.fft.rfft(blocks * window, axis=1)) ** 2, axis=0) / (hz * (window @ window))
    psd[1:-1 if n % 2 == 0 else None] *= 2
    return np.fft.rfftfreq(n, 1 / hz), psd


def _peaks(values, hz, count):
    frequencies, psd = periodogram(values, hz)
    candidates = np.flatnonzero((frequencies >= 2) & (psd > 0))
    chosen = []
    for k in candidates[np.argsort(psd[candidates])[::-1]]:
        if all(abs(k - other) >= 3 for other in chosen):
            chosen.append(int(k))
            if len(chosen) >= count:
                break
    return tuple(float(frequencies[k]) for k in chosen) if count > 0 else ()


def estimate_imu_noise(t, acc, gyro, intervals: Sequence[tuple[float, float]] | None = None,
                       detrend_s: float = 0.2, n_peaks: int = 3) -> ImuNoise:
    """Per-sample IMU noise of a flight stream (see module docstring for the definition).

    - drop rows with non-finite t/acc/gyro; sort by t;
    - keep samples inside the half-open intervals [lo, hi) (all samples if None/empty);
    - segments: split at interval edges and at gaps > 5 * median dt; skip segments shorter
      than the detrend window + 2 samples; window n = odd number of samples ~ detrend_s * stream_hz (>= 3);
    - acc_std/gyro_std from the concatenated residuals; acc_trend_std from the removed moving means;
    - peaks: averaged periodogram (e.g. Hann, 256-sample segments) of the worst axis of the longest
      segment, ignore < 2 Hz, peaks at least 3 frequency bins apart;
    - nothing usable -> NaN stds/densities and warnings ("TOO_FEW_SAMPLES",); no exception.
    """
    if not math.isfinite(detrend_s) or detrend_s <= 0 or n_peaks < 0:
        raise ValueError("Expected a positive detrend window and nonnegative peak count")
    t, acc, gyro = _stream(t, acc, gyro)
    intervals = sorted(intervals or [(float('-inf'), float('inf'))])
    if any(hi <= lo for lo, hi in intervals) or any(b[0] < a[1] for a, b in zip(intervals, intervals[1:])):
        raise ValueError("Noise intervals must be nonempty and disjoint")
    dt = np.diff(t)
    hz = float(1 / np.median(dt)) if len(dt) else float('nan')
    window = max(3, int(round(detrend_s * hz)) | 1) if math.isfinite(hz) else 3
    residuals, trends, longest = [], [], None
    duration = 0.
    analysed_dt = []
    for lo, hi in intervals:
        selected = np.flatnonzero((t >= lo) & (t < hi))
        if not len(selected):
            continue
        cuts = np.flatnonzero(np.diff(t[selected]) > 5 / hz) + 1
        for indices in np.split(selected, cuts):
            if len(indices) < window + 2:
                continue
            values = np.c_[acc[indices], gyro[indices]]
            # Valid convolution discards both edges instead of padding across a gap.
            cumulative = np.vstack((np.zeros((1, 6)), np.cumsum(values, axis=0)))
            trend = (cumulative[window:] - cumulative[:-window]) / window
            half = window // 2
            residuals.append((values[half:-half] - trend) * math.sqrt(window / (window - 1)))
            trends.append(trend[:, :3])
            duration += float(t[indices[-1]] - t[indices[0]])
            analysed_dt.extend(np.diff(t[indices]))
            if longest is None or len(values) > len(longest):
                longest = values
    if not residuals:
        nan3 = (float('nan'),) * 3
        return ImuNoise(hz, 0, 0., nan3, nan3, float('nan'), float('nan'), nan3, (), (), ("TOO_FEW_SAMPLES",))
    residuals, trends = np.vstack(residuals), np.vstack(trends)
    std = residuals.std(axis=0)
    warnings = []
    if len(residuals) < MIN_SAMPLES:
        warnings.append("TOO_FEW_SAMPLES")
    if np.mean(np.abs(np.asarray(analysed_dt) * hz - 1) > .1) > .05:
        warnings.append("IRREGULAR_RATE")
    if max(std[:3]) > VIBRATION_ACC_WARN or max(std[3:]) > VIBRATION_GYRO_WARN:
        warnings.append("VIBRATION_HIGH")
    return ImuNoise(hz, len(residuals), duration, tuple(std[:3]), tuple(std[3:]),
                    float(max(std[:3]) / math.sqrt(hz)), float(max(std[3:]) / math.sqrt(hz)),
                    tuple(trends.std(axis=0)), _peaks(longest[:, np.argmax(std[:3])], hz, n_peaks),
                    _peaks(longest[:, 3 + np.argmax(std[3:])], hz, n_peaks), tuple(warnings))


def recommend_params(noise: ImuNoise, base: EskfParams = EskfParams(), margin: float = 1.0,
                     min_anchor_interval_s: float = 0.02, reinit_keep_attitude: bool = True) -> EskfParams:
    """EskfParams for this stream: na = max(base.na, margin * acc_density), ng = max(base.ng,
    margin * gyro_density), plus min_anchor_interval_s and reinit_keep_attitude; every other field
    copied from base. ValueError if margin < 1 or the densities are not finite.
    """
    if not math.isfinite(margin) or margin < 1 or not np.isfinite([noise.acc_density, noise.gyro_density]).all() \
            or min(noise.acc_density, noise.gyro_density) < 0:
        raise ValueError("Expected finite nonnegative densities and margin >= 1")
    if not math.isfinite(min_anchor_interval_s) or min_anchor_interval_s < 0:
        raise ValueError("Expected a finite nonnegative anchor interval")
    return replace(base, na=max(base.na, margin * noise.acc_density), ng=max(base.ng, margin * noise.gyro_density),
                   min_anchor_interval_s=min_anchor_interval_s, reinit_keep_attitude=reinit_keep_attitude)


def normalized_errors(est_t, est, est_std, ref_t, ref, max_gap_s: float = 0.1) -> dict[str, float]:
    """Consistency of a 1-D estimate (e.g. TAG z) against a reference.

    Interpolate est and est_std at ref_t; drop reference times outside the estimate, inside an
    estimate gap > max_gap_s, or with non-finite values / std <= 0. Returns
    {"n", "frac_within_1sigma", "frac_within_2sigma", "rms_normalized", "rms_error", "median_sigma"}
    (|e|/sigma < 1, < 2; sqrt(mean((e/sigma)^2)); NaN fractions and n = 0 when nothing is usable).
    A consistent filter gives ~0.68 / ~0.95 / ~1.0.
    """
    t, est, std, rt, ref = map(lambda x: np.asarray(x, float), (est_t, est, est_std, ref_t, ref))
    empty = dict(n=0, frac_within_1sigma=float('nan'), frac_within_2sigma=float('nan'),
                 rms_normalized=float('nan'), rms_error=float('nan'), median_sigma=float('nan'))
    if not math.isfinite(max_gap_s) or max_gap_s <= 0:
        raise ValueError("Expected a positive finite interpolation gap")
    if t.ndim != 1 or est.shape != t.shape or std.shape != t.shape or rt.ndim != 1 or ref.shape != rt.shape:
        raise ValueError("Expected matching one-dimensional estimate/reference arrays")
    finite_t = np.isfinite(t)
    order = np.argsort(t[finite_t], kind='stable')
    t, est, std = t[finite_t][order], est[finite_t][order], std[finite_t][order]
    unique = np.r_[True, np.diff(t) > 0] if len(t) else np.zeros(0, bool)
    t, est, std = t[unique], est[unique], std[unique]
    if len(t) < 2:
        return empty
    values, sigma = np.interp(rt, t, est), np.interp(rt, t, std)
    right = np.clip(np.searchsorted(t, rt), 1, len(t)-1)
    exact = (rt == t[right]) | (rt == t[right-1])
    good = np.isfinite(rt) & np.isfinite(ref) & np.isfinite(values) & np.isfinite(sigma) & (sigma > 0) \
        & (rt >= t[0]) & (rt <= t[-1]) & ((t[right] - t[right-1] <= max_gap_s) | exact)
    if not good.any():
        return empty
    error, sigma = values[good] - ref[good], sigma[good]
    normalized = error / sigma
    return dict(n=int(good.sum()), frac_within_1sigma=float(np.mean(np.abs(normalized) < 1)),
                frac_within_2sigma=float(np.mean(np.abs(normalized) < 2)),
                rms_normalized=float(np.sqrt(np.mean(normalized ** 2))),
                rms_error=float(np.sqrt(np.mean(error ** 2))), median_sigma=float(np.median(sigma)))


def innovation_autocorrelation(result, lags_s=(0.1, 0.5, 1.0), grid_dt: float = 0.01,
                               accepted_only: bool = True) -> dict[int, dict[str, Any]]:
    """Per-anchor normalized autocorrelation of the range innovations of a ReplayResult.

    For each anchor: (accepted) finite innovations, linearly resampled on a grid_dt grid, mean
    removed, autocorr(lag) = sum(y[:-L] y[L:]) / sum(y y), L = round(lag / grid_dt).
    Returns {anchor_id: {"samples", "rate_hz", "std_m", "lags_s", "autocorr"}}; anchors whose span
    is shorter than the longest lag are omitted. Correlated (non-white) range errors show values
    well above 0 at 0.5-1 s (Test_1: 0.5-0.9 at 1 s).
    """
    if not math.isfinite(grid_dt) or grid_dt <= 0 or not len(lags_s) \
            or not np.isfinite(lags_s).all() or min(lags_s) < 0:
        raise ValueError("Expected a positive grid interval and finite nonnegative lags")
    output = {}
    for aid in np.unique(result.update_anchor):
        mask = (result.update_anchor == aid) & np.isfinite(result.update_t) & np.isfinite(result.update_innovation_m)
        if accepted_only:
            mask &= result.update_accepted
        t, innovation = result.update_t[mask], result.update_innovation_m[mask]
        order = np.argsort(t, kind='stable')
        t, innovation = t[order], innovation[order]
        if len(t) < 2 or t[-1] - t[0] <= max(lags_s):
            continue
        grid = np.arange(t[0], t[-1], grid_dt)
        y = np.interp(grid, t, innovation)
        y -= y.mean()
        denominator = float(y @ y)
        ac = []
        for lag in lags_s:
            k = int(round(lag / grid_dt))
            ac.append(float((y[:-k] @ y[k:] if k else denominator) / denominator)
                      if denominator > 0 else float('nan'))
        output[int(aid)] = dict(samples=len(t), rate_hz=float((len(t)-1)/(t[-1]-t[0])),
                                std_m=float(innovation.std()), lags_s=tuple(lags_s), autocorr=ac)
    return output
