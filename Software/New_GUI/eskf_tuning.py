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

from dataclasses import asdict, dataclass
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
    raise NotImplementedError("E2.1: implement decimate_stream")


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
    raise NotImplementedError("E2.1: implement estimate_imu_noise")


def recommend_params(noise: ImuNoise, base: EskfParams = EskfParams(), margin: float = 1.0,
                     min_anchor_interval_s: float = 0.02, reinit_keep_attitude: bool = True) -> EskfParams:
    """EskfParams for this stream: na = max(base.na, margin * acc_density), ng = max(base.ng,
    margin * gyro_density), plus min_anchor_interval_s and reinit_keep_attitude; every other field
    copied from base. ValueError if margin < 1 or the densities are not finite.
    """
    raise NotImplementedError("E2.1: implement recommend_params")


def normalized_errors(est_t, est, est_std, ref_t, ref, max_gap_s: float = 0.1) -> dict[str, float]:
    """Consistency of a 1-D estimate (e.g. TAG z) against a reference.

    Interpolate est and est_std at ref_t; drop reference times outside the estimate, inside an
    estimate gap > max_gap_s, or with non-finite values / std <= 0. Returns
    {"n", "frac_within_1sigma", "frac_within_2sigma", "rms_normalized", "rms_error", "median_sigma"}
    (|e|/sigma < 1, < 2; sqrt(mean((e/sigma)^2)); NaN fractions and n = 0 when nothing is usable).
    A consistent filter gives ~0.68 / ~0.95 / ~1.0.
    """
    raise NotImplementedError("E2.1: implement normalized_errors")


def innovation_autocorrelation(result, lags_s=(0.1, 0.5, 1.0), grid_dt: float = 0.01,
                               accepted_only: bool = True) -> dict[int, dict[str, Any]]:
    """Per-anchor normalized autocorrelation of the range innovations of a ReplayResult.

    For each anchor: (accepted) finite innovations, linearly resampled on a grid_dt grid, mean
    removed, autocorr(lag) = sum(y[:-L] y[L:]) / sum(y y), L = round(lag / grid_dt).
    Returns {anchor_id: {"samples", "rate_hz", "std_m", "lags_s", "autocorr"}}; anchors whose span
    is shorter than the longest lag are omitted. Correlated (non-white) range errors show values
    well above 0 at 0.5-1 s (Test_1: 0.5-0.9 at 1 s).
    """
    raise NotImplementedError("E2.1: implement innovation_autocorrelation")
