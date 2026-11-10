"""High-speed UWB state engine, 2D/3D robust position solver, DOP calculator, and calibration engine.

Designed for DWM1001 Tag telemetry (Burst DS-TWR 0x18 at 50-300 Hz, RANGE_MEAS 0x10,
RANGE snapshot 0x01, RX_QUALITY 0x19, INFO 0x00, DEVICE_INFO 0x16, DIAG_SYSTEM 0x15).
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field, replace
import json
import math
from pathlib import Path
import re
import statistics
import struct
import sys
import time
from typing import Callable, Iterable, Mapping, Sequence

import numpy as np

# Reuse the verified protocol parser & host CV alpha-beta filter without modifying old GUI
_OLD_GUI_DIR = Path(__file__).resolve().parent.parent / "UWB_UART_GUI"
if str(_OLD_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(_OLD_GUI_DIR))

import telemetry_protocol as tp  # noqa: E402
from host_range_filter import HostRangeFilter  # noqa: E402
from robust_range_ekf import RobustRangeEkf, RangeObservation  # noqa: E402
from radio_quality import RadioQualityModel, QualityResult, QualityReplay, nlos_variance_scale  # noqa: E402
from diag_rates import DiagTracker  # noqa: E402
from multi_spot_calibration import (JointCalibrationResult, JointCalibrationSettings,
                                    SOLVER_VERSION, observations_from_captures,
                                    solve_joint_calibration)  # noqa: E402


MAX_ANCHORS = 8
FRESH_AGE_S = 2.0
STATIONARY_HOLD_AGE_S = 9.0
RING_HISTORY_LEN = 9000   # Holds >= 30s of high-rate history per anchor even at 300 Hz
TRAIL_MAX_LEN = 9000      # Holds >= 30s of solved XYZ history for the 30s Rolling Chart
MAP_TRAIL_LEN = 120000    # Full-loop spatial path capacity on the 2D/3D canvas (hours of walking)
NLOS_WARN_DB = 6.0
NLOS_ALERT_DB = 10.0
# Spot-calibration sanity checks (engineering thresholds, not probabilities).
SPOT_OFFSET_WARN_M = 0.25        # ranges imply the TAG is this far from the entered spot
SPOT_RESIDUAL_WARN_MM = 150.0    # anchors disagree even after a shift + common offset
CAL_BIAS_WARN_MM = 300.0         # per-anchor DS-TWR bias is normally well inside this
CALIBRATION_METHODS = ("joint", "median")
JOINT_WARNING_CODES = ("SPOT_OFFSET", "RESIDUAL_OUTLIER", "RESIDUAL_RMS", "BIAS_LARGE", "NOT_IDENTIFIABLE", "MANY_SPOT_OFFSETS")
SPOT_BIAS_SPREAD_M = 0.05        # assumed per-anchor bias spread for the offset uncertainty


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float


@dataclass
class AnchorConfig:
    anchor_id: int
    x: float
    y: float
    z: float = 0.0
    enabled: bool = True
    bias_mm: float = 0.0       # meas_mm = true_mm + bias_mm  (corrected = raw - bias)
    sigma_mm: float = 30.0     # white noise std estimate (mm)
    fw_calibrated: bool = False
    fw_bias_mm: float = 0.0


@dataclass(frozen=True)
class DopMetrics:
    gdop: float
    pdop: float
    hdop: float
    vdop: float
    condition_number: float
    coplanar_warning: bool


@dataclass(frozen=True)
class PositionResult:
    x: float
    y: float
    z: float
    rms_m: float
    max_residual_m: float
    residuals_m: dict[int, float]
    used_anchor_ids: tuple[int, ...]
    excluded_anchor_ids: tuple[int, ...]
    dimension: int
    dop: DopMetrics
    tag_time_s: float
    raw_x: float = 0.0
    raw_y: float = 0.0
    raw_z: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    speed_mps: float = 0.0
    step_jump_mm: float = 0.0
    motion_state: str = "STATIONARY"      # "MOVING" | "STATIONARY"
    stability_state: str = "STABLE"       # "STABLE" | "WARN" | "UNSTABLE"
    instability_reason: str = ""
    filter_mode: str = "legacy"
    position_std_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    prediction_age_s: float = 0.0


@dataclass
class AnchorLiveState:
    anchor_id: int
    last_update_s: float = -1.0
    tag_time_s: float = 0.0
    mode_name: str = "—"
    status: int = 0
    valid: bool = False
    fw_calibrated: bool = False
    radio_ok: bool = False
    late: bool = False
    raw_mm: int = 0
    corrected_mm: float = 0.0
    fw_filtered_mm: int | None = None
    host_filtered_mm: float | None = None
    fp_dbm: float | None = None
    rx_dbm: float | None = None
    nlos_db: float | None = None
    ci_ppm: float | None = None
    rate_hz: float = 0.0
    noise_sigma_mm: float | None = None
    residual_mm: float | None = None
    rx_count: int = 0
    quality: QualityResult = field(default_factory=QualityResult)

    def is_fresh(self, now_s: float) -> bool:
        return self.last_update_s >= 0.0 and (now_s - self.last_update_s) <= FRESH_AGE_S


@dataclass(frozen=True)
class CalibrationAnchorResult:
    anchor_id: int
    samples: int
    true_distance_mm: float
    mean_raw_mm: float
    median_raw_mm: float
    bias_mm: float          # mean_raw_mm - true_distance_mm  (firmware SET_DS_CAL sign)
    offset_mm: float        # true_distance_mm - mean_raw_mm  (GUI legacy sign)
    sigma_mm: float
    p05_mm: float
    p95_mm: float
    drift_mm: float
    mean_nlos_db: float | None
    mean_fp_dbm: float | None
    quality: str            # "GOOD" | "WARN" | "LOW_SAMPLES"


@dataclass(frozen=True)
class SpotConsistencyCheck:
    """Where the raw spot ranges place the TAG when one common range offset is allowed.

    A wrongly entered spot shows up as a position shift; per-anchor antenna-delay
    differences remain as residuals. This checks the entered coordinates; it does
    not replace a measured reference position.
    """
    implied_xyz: tuple[float, float, float] | None
    offset_m: float | None
    offset_std_m: float | None
    common_bias_mm: float | None
    residual_rms_mm: float | None
    anchors_used: int
    warnings: tuple[str, ...] = ()


@dataclass(frozen=True)
class CalibrationSpotCapture:
    spot_index: int
    spot_xyz: tuple[float, float, float] | None
    single_anchor_id: int | None
    reference_distance_m: float | None
    timestamp_s: float
    anchor_results: dict[int, CalibrationAnchorResult]
    consistency: SpotConsistencyCheck | None = None


@dataclass(frozen=True)
class EkfGuardSettings:
    """Re-anchor the EKF when a consistent, independent IRLS fix disagrees for long.

    The gate and link-health logic can keep the EKF on a wrong solution: anchors
    that disagree with the state are rejected, so they cannot correct it. The
    guard acts only when IRLS has redundant anchors and a small residual.
    """
    enabled: bool = True
    max_irls_rms_m: float = 0.06
    max_irls_pdop: float = 4.0
    min_offset_m: float = 0.15
    sigma_multiplier: float = 3.0
    min_rms_ratio: float = 2.0         # ranges must fit IRLS clearly better than the EKF
    min_rms_gain_m: float = 0.03
    range_age_margin_s: float = 0.16   # IRLS may use ranges this old; allow speed * age
    persist_s: float = 0.5
    max_evidence_gap_s: float = 0.12


@dataclass(frozen=True)
class AccuracyReport:
    samples: int
    dimension: int
    reference_xyz: tuple[float, float, float]
    mean_xyz: tuple[float, float, float]
    bias_xyz_m: tuple[float, float, float]
    std_xyz_m: tuple[float, float, float]
    rmse_2d_m: float
    rmse_3d_m: float
    cep50_m: float
    cep95_m: float
    sep50_m: float
    sep95_m: float
    mean_pdop: float


DEFAULT_PRESET_ROOM_66_36 = (
    AnchorConfig(1, 0.00, 0.00, 0.48, True),
    AnchorConfig(2, 6.60, 0.00, 0.48, True),
    AnchorConfig(3, 6.60, 3.60, 0.48, True),
    AnchorConfig(4, 0.00, 3.60, 0.48, True),
    AnchorConfig(5, 0.00, 0.00, 1.58, True),
    AnchorConfig(6, 6.60, 0.00, 1.58, True),
    AnchorConfig(7, 6.60, 3.60, 1.58, True),
    AnchorConfig(8, 0.00, 3.60, 1.58, True),
)

DEFAULT_PRESET_BOX8 = (
    AnchorConfig(1, 0.0, 0.0, 0.0, True),
    AnchorConfig(2, 5.0, 0.0, 0.0, True),
    AnchorConfig(3, 0.0, 4.0, 0.0, True),
    AnchorConfig(4, 5.0, 4.0, 0.0, True),
    AnchorConfig(5, 0.0, 0.0, 3.0, True),
    AnchorConfig(6, 5.0, 0.0, 3.0, True),
    AnchorConfig(7, 0.0, 4.0, 3.0, True),
    AnchorConfig(8, 5.0, 4.0, 3.0, True),
)

DEFAULT_PRESET_RIG40 = (
    AnchorConfig(1, 0.00, 0.00, 0.48, True, bias_mm=0.0),
    AnchorConfig(2, 0.40, 0.40, 0.48, True, bias_mm=-17.0),
    AnchorConfig(3, 0.40, 0.00, 0.48, True, bias_mm=-178.0),
    AnchorConfig(4, 0.00, 0.40, 0.48, True, bias_mm=146.0),
    AnchorConfig(5, 0.00, 0.00, 1.58, True, bias_mm=52.0),
    AnchorConfig(6, 0.40, 0.40, 1.58, True, bias_mm=-33.0),
    AnchorConfig(7, 0.40, 0.00, 1.58, True, bias_mm=63.0),
    AnchorConfig(8, 0.00, 0.40, 1.58, True, bias_mm=-59.0),
)

DEFAULT_PRESET_FLAT4 = (
    AnchorConfig(1, 0.0, 0.0, 0.0, True),
    AnchorConfig(2, 5.0, 0.0, 0.0, True),
    AnchorConfig(3, 0.0, 4.0, 0.0, True),
    AnchorConfig(4, 5.0, 4.0, 0.0, True),
    AnchorConfig(5, 0.0, 0.0, 3.0, False),
    AnchorConfig(6, 5.0, 0.0, 3.0, False),
    AnchorConfig(7, 0.0, 4.0, 3.0, False),
    AnchorConfig(8, 5.0, 4.0, 3.0, False),
)


def _percentile(values: Sequence[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(v) for v in values)
    pos = min(max(q, 0.0), 1.0) * (len(ordered) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    w = pos - lo
    return ordered[lo] * (1.0 - w) + ordered[hi] * w


def _invert_small_matrix(mat: list[list[float]]) -> list[list[float]] | None:
    """Gauss-Jordan matrix inverse for 2x2 or 3x3 normal matrices."""
    n = len(mat)
    aug = [
        [float(mat[r][c]) for c in range(n)] + [1.0 if r == c else 0.0 for c in range(n)]
        for r in range(n)
    ]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-10:
            return None
        aug[col], aug[pivot] = aug[pivot], aug[col]
        div = aug[col][col]
        aug[col] = [v / div for v in aug[col]]
        for r in range(n):
            if r == col:
                continue
            factor = aug[r][col]
            aug[r] = [aug[r][c] - factor * aug[col][c] for c in range(2 * n)]
    return [row[n:] for row in aug]


def compute_dop(
    anchors: Sequence[AnchorConfig],
    position_xyz: tuple[float, float, float],
    dimension: int = 3,
) -> DopMetrics:
    enabled = [a for a in anchors if a.enabled]
    min_needed = dimension + 1
    if len(enabled) < min_needed:
        return DopMetrics(99.9, 99.9, 99.9, 99.9, math.inf, True)

    zs = [a.z for a in enabled]
    z_span = max(zs) - min(zs) if zs else 0.0
    coplanar = dimension == 3 and z_span < 0.08

    px, py, pz = position_xyz
    dim = 3 if dimension == 3 else 2
    ht_h = [[0.0] * dim for _ in range(dim)]
    for a in enabled:
        dx = px - a.x
        dy = py - a.y
        dz = pz - a.z
        dist = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-4)
        row = [dx / dist, dy / dist] + ([dz / dist] if dim == 3 else [])
        for i in range(dim):
            for j in range(dim):
                ht_h[i][j] += row[i] * row[j]

    inv = _invert_small_matrix(ht_h)
    if inv is None:
        return DopMetrics(99.9, 99.9, 99.9, 99.9, math.inf, True)

    var_x = max(inv[0][0], 0.0)
    var_y = max(inv[1][1], 0.0)
    var_z = max(inv[2][2], 0.0) if dim == 3 else 0.0
    hdop = math.sqrt(var_x + var_y)
    vdop = math.sqrt(var_z) if dim == 3 else 0.0
    pdop = math.sqrt(var_x + var_y + var_z)
    gdop = pdop
    diag = [abs(ht_h[i][i]) for i in range(dim)]
    cond = max(diag) / min(diag) if min(diag) > 1e-9 else math.inf
    return DopMetrics(
        gdop=min(gdop, 99.9),
        pdop=min(pdop, 99.9),
        hdop=min(hdop, 99.9),
        vdop=min(vdop, 99.9),
        condition_number=cond,
        coplanar_warning=coplanar or vdop > 15.0,
    )


def _solve_linear(matrix: list[list[float]], rhs: list[float]) -> list[float] | None:
    inv = _invert_small_matrix(matrix)
    if inv is None:
        return None
    n = len(rhs)
    return [sum(inv[i][j] * rhs[j] for j in range(n)) for i in range(n)]


def _fit_irls_single(
    anchor_map: dict[int, AnchorConfig],
    ranges_m: dict[int, float],
    weights_prior: dict[int, float],
    used_ids: Sequence[int],
    dimension: int,
    fixed_z: float,
    est_init: list[float],
) -> tuple[list[float], dict[int, float]] | None:
    est = list(est_init)
    for _ in range(25):
        residuals: dict[int, float] = {}
        jac: dict[int, list[float]] = {}
        for aid in used_ids:
            a = anchor_map[aid]
            z = est[2] if dimension == 3 else fixed_z
            dx, dy, dz = est[0] - a.x, est[1] - a.y, z - a.z
            dist = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-6)
            residuals[aid] = dist - ranges_m[aid]
            jac[aid] = [dx / dist, dy / dist] + ([dz / dist] if dimension == 3 else [])

        abs_res = sorted(abs(v) for v in residuals.values())
        scale = max(_percentile(abs_res, 0.50), 0.020)
        huber = max(1.8 * scale, 0.045)

        normal = [[0.0] * dimension for _ in range(dimension)]
        rhs = [0.0] * dimension
        for aid in used_ids:
            r = residuals[aid]
            row = jac[aid]
            w_huber = 1.0 if abs(r) <= huber else (huber / abs(r)) ** 2
            w = w_huber * weights_prior.get(aid, 1.0)
            for i in range(dimension):
                rhs[i] -= w * row[i] * r
                for j in range(dimension):
                    normal[i][j] += w * row[i] * row[j]

        step = _solve_linear(normal, rhs)
        if step is None:
            return None
        est = [v + dv for v, dv in zip(est, step)]
        if math.sqrt(sum(dv * dv for dv in step)) < 1e-5:
            break

    final_residuals: dict[int, float] = {}
    for aid in used_ids:
        a = anchor_map[aid]
        z = est[2] if dimension == 3 else fixed_z
        dist = math.dist((est[0], est[1], z), (a.x, a.y, a.z))
        final_residuals[aid] = dist - ranges_m[aid]
    return est, final_residuals


def _fit_irls(
    anchor_map: dict[int, AnchorConfig],
    ranges_m: dict[int, float],
    weights_prior: dict[int, float],
    used_ids: Sequence[int],
    dimension: int,
    fixed_z: float,
    initial_xyz: tuple[float, float, float] | None = None,
) -> tuple[list[float], dict[int, float]] | None:
    mx = statistics.fmean(anchor_map[aid].x for aid in used_ids)
    my = statistics.fmean(anchor_map[aid].y for aid in used_ids)
    if dimension != 3:
        seed = [initial_xyz[0], initial_xyz[1]] if initial_xyz is not None else [mx, my]
        return _fit_irls_single(anchor_map, ranges_m, weights_prior, used_ids, dimension, fixed_z, seed)

    zs = [anchor_map[aid].z for aid in used_ids]
    z_min, z_max = min(zs), max(zs)
    z_span = z_max - z_min
    if initial_xyz is not None and (z_span <= 0.05 or initial_xyz[2] <= z_max):
        res = _fit_irls_single(
            anchor_map,
            ranges_m,
            weights_prior,
            used_ids,
            dimension,
            fixed_z,
            [initial_xyz[0], initial_xyz[1], initial_xyz[2]],
        )
        if res is not None and (z_span <= 0.05 or res[0][2] <= z_max + 0.05):
            return res

    z_mid = 0.5 * (z_min + z_max)
    if z_span > 0.05:
        z_seeds = [z_min, z_mid - 0.20 * z_span, z_mid + 0.20 * z_span]
    else:
        z_seeds = [z_mid]

    best: tuple[list[float], dict[int, float]] | None = None
    best_cost = math.inf
    for z0 in z_seeds:
        res = _fit_irls_single(anchor_map, ranges_m, weights_prior, used_ids, dimension, fixed_z, [mx, my, z0])
        if res is not None:
            est, residuals = res
            cost = sum(weights_prior.get(aid, 1.0) * (r * r) for aid, r in residuals.items())
            if z_span > 0.05 and est[2] > z_max + 0.10:
                cost *= 1.35
            if cost < best_cost:
                best_cost = cost
                best = res
    return best


def solve_position_robust(
    anchors: Sequence[AnchorConfig],
    ranges_mm: Mapping[int, float],
    dimension: int = 3,
    fixed_z: float = 0.0,
    nlos_db_map: Mapping[int, float | None] | None = None,
    initial_xyz: tuple[float, float, float] | None = None,
    tag_time_s: float = 0.0,
) -> PositionResult | None:
    if dimension not in (2, 3):
        raise ValueError("dimension must be 2 or 3")
    anchor_map = {a.anchor_id: a for a in anchors if a.enabled}
    ranges_m = {
        aid: float(r_mm) / 1000.0
        for aid, r_mm in ranges_mm.items()
        if aid in anchor_map and math.isfinite(float(r_mm)) and float(r_mm) > 10.0
    }
    min_needed = dimension + 1
    if len(ranges_m) < min_needed:
        return None

    weights_prior: dict[int, float] = {}
    for aid in ranges_m:
        sigma_m = max(anchor_map[aid].sigma_mm / 1000.0, 0.015)
        w = (0.03 / sigma_m) ** 2
        if nlos_db_map:
            nlos = nlos_db_map.get(aid)
            if nlos is not None and math.isfinite(nlos) and nlos > 7.5:
                w /= 1.0 + 0.25 * min(nlos - 7.5, 10.0) ** 2
        weights_prior[aid] = min(max(w, 0.05), 4.0)

    used = sorted(ranges_m)
    excluded: list[int] = []
    fit = _fit_irls(anchor_map, ranges_m, weights_prior, used, dimension, fixed_z, initial_xyz)
    if fit is None:
        return None
    est, residuals = fit

    # Single-step largest outlier leave-one-out check when redundant anchors exist
    while len(used) > min_needed:
        worst_id = max(used, key=lambda aid: abs(residuals[aid]))
        others = [abs(residuals[aid]) for aid in used if aid != worst_id]
        median_other = _percentile(others, 0.50)
        threshold = max(0.28, 3.8 * median_other)
        if abs(residuals[worst_id]) <= threshold:
            break
        candidate = [aid for aid in used if aid != worst_id]
        refit = _fit_irls(anchor_map, ranges_m, weights_prior, candidate, dimension, fixed_z, initial_xyz)
        if refit is None:
            break
        est, residuals = refit
        excluded.append(worst_id)
        used = candidate

    z_val = est[2] if dimension == 3 else fixed_z
    pos_xyz = (est[0], est[1], z_val)
    res_vals = list(residuals.values())
    rms_m = math.sqrt(statistics.fmean(r * r for r in res_vals)) if res_vals else 0.0
    max_res_m = max((abs(r) for r in res_vals), default=0.0)
    used_configs = [anchor_map[aid] for aid in used]
    dop = compute_dop(used_configs, pos_xyz, dimension=dimension)

    return PositionResult(
        x=pos_xyz[0],
        y=pos_xyz[1],
        z=pos_xyz[2],
        rms_m=rms_m,
        max_residual_m=max_res_m,
        residuals_m=residuals,
        used_anchor_ids=tuple(used),
        excluded_anchor_ids=tuple(excluded),
        dimension=dimension,
        dop=dop,
        tag_time_s=tag_time_s,
        raw_x=pos_xyz[0],
        raw_y=pos_xyz[1],
        raw_z=pos_xyz[2],
    )


def check_spot_consistency(
    anchors: Sequence[AnchorConfig],
    median_raw_mm: Mapping[int, float],
    spot_xyz: tuple[float, float, float],
    dimension: int = 3,
) -> SpotConsistencyCheck:
    """Fit the TAG position plus one common range offset to raw spot ranges.

    A spot entered at the wrong place produces biases that follow the anchor
    geometry (positive on one wall, negative on the opposite wall). Allowing a
    position shift explains that pattern; a common offset absorbs the shared
    antenna delay. Huber weights limit the effect of one NLOS anchor.
    """
    amap = {a.anchor_id: a for a in anchors if a.enabled}
    ids = sorted(
        aid for aid, v in median_raw_mm.items()
        if aid in amap and math.isfinite(float(v)) and float(v) > 10.0
    )
    warnings: list[str] = []
    biases = {
        aid: float(median_raw_mm[aid]) - math.dist(spot_xyz, (amap[aid].x, amap[aid].y, amap[aid].z)) * 1000.0
        for aid in ids
    }
    big = [f"A{aid} {b:+.0f}" for aid, b in biases.items() if abs(b) > CAL_BIAS_WARN_MM]
    if big:
        warnings.append(
            f"Bias beyond ±{CAL_BIAS_WARN_MM:.0f} mm ({', '.join(big)} mm): usually a wrong spot "
            "or anchor coordinate, not antenna delay."
        )
    n_pos = 3 if dimension == 3 else 2
    if len(ids) < n_pos + 2:
        # Position and common offset are not over-determined; only the bias check applies.
        return SpotConsistencyCheck(None, None, None, None, None, len(ids), tuple(warnings))

    pos_a = np.array([[amap[a].x, amap[a].y, amap[a].z] for a in ids], dtype=float)
    rng = np.array([float(median_raw_mm[a]) for a in ids], dtype=float) / 1000.0
    p = np.array(spot_xyz, dtype=float)
    c = float(np.median(rng - np.linalg.norm(p - pos_a, axis=1)))
    huber_m = 0.15
    normal = None
    for _ in range(50):
        delta = p - pos_a
        dist = np.maximum(np.linalg.norm(delta, axis=1), 1e-6)
        res = rng - dist - c
        weights = np.where(np.abs(res) <= huber_m, 1.0, huber_m / np.maximum(np.abs(res), 1e-9))
        jac = np.column_stack([delta[:, :n_pos] / dist[:, None], np.ones(len(ids))])
        jw = jac * weights[:, None]
        normal = jac.T @ jw
        try:
            step = np.linalg.solve(normal, jw.T @ res)
        except np.linalg.LinAlgError:
            normal = None
            break
        p[:n_pos] += step[:n_pos]
        c += float(step[-1])
        if not np.all(np.isfinite(p)) or float(np.linalg.norm(step)) < 1e-7:
            break
    if normal is None or not np.all(np.isfinite(p)) or not math.isfinite(c):
        return SpotConsistencyCheck(None, None, None, None, None, len(ids), tuple(warnings))

    res = rng - np.linalg.norm(p - pos_a, axis=1) - c
    rms_m = float(np.sqrt(np.mean(res**2)))
    try:
        cov = np.linalg.inv(normal) * max(rms_m, SPOT_BIAS_SPREAD_M) ** 2
        offset_std = float(math.sqrt(max(float(np.linalg.eigvalsh(cov[:n_pos, :n_pos]).max()), 0.0)))
    except np.linalg.LinAlgError:
        offset_std = math.inf
    implied = (float(p[0]), float(p[1]), float(p[2]))
    offset = math.dist(implied, spot_xyz)
    if offset > max(SPOT_OFFSET_WARN_M, 3.0 * offset_std):
        warnings.append(
            f"Ranges place the TAG near ({implied[0]:.2f}, {implied[1]:.2f}, {implied[2]:.2f}) m, "
            f"{offset:.2f} m from the entered spot ({spot_xyz[0]:.2f}, {spot_xyz[1]:.2f}, {spot_xyz[2]:.2f}). "
            "Check the spot coordinates and anchor layout before applying."
        )
    if rms_m * 1000.0 > SPOT_RESIDUAL_WARN_MM:
        warnings.append(
            f"Anchors disagree by {rms_m * 1000.0:.0f} mm RMS even after a position shift: "
            "NLOS at this spot or a wrong anchor coordinate."
        )
    return SpotConsistencyCheck(
        implied_xyz=implied,
        offset_m=offset,
        offset_std_m=offset_std,
        common_bias_mm=c * 1000.0,
        residual_rms_mm=rms_m * 1000.0,
        anchors_used=len(ids),
        warnings=tuple(warnings),
    )


def encode_set_ds_cal_command(
    sequence: int, anchor_id: int, bias_mm: float, calibrated: bool = True
) -> bytes:
    """Encode CMD_SET_DS_CAL (0x06): u16 anchor_id, i32 bias_um, u8 cal.

    Matches Firmware/common/services/uwb_cmd.c: corrected = measured - bias.
    """
    bias_um = int(round(float(bias_mm) * 1000.0))
    payload = struct.pack("<HiB", int(anchor_id), bias_um, 1 if calibrated else 0)
    return tp.encode_command(sequence & 0xFFFFFFFF, tp.CMD_SET_DS_CAL, payload)


def load_anchors_rig_txt(path: Path) -> list[AnchorConfig]:
    """Load anchors_rig40.txt format (anN_p, x, y, z and anN_bias / anN_sigma)."""
    pos_map: dict[int, tuple[float, float, float]] = {}
    bias_m_map: dict[int, float] = {}
    sigma_m_map: dict[int, float] = {}

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2:
            continue
        key = parts[0]
        m_pos = re.match(r"^an(\d+)_p$", key)
        if m_pos and len(parts) >= 4:
            aid = int(m_pos.group(1))
            pos_map[aid] = (float(parts[1]), float(parts[2]), float(parts[3]))
            continue
        m_bias = re.match(r"^an(\d+)_bias$", key)
        if m_bias:
            aid = int(m_bias.group(1))
            bias_m_map[aid] = float(parts[1])
            continue
        m_sigma = re.match(r"^an(\d+)_sigma$", key)
        if m_sigma:
            aid = int(m_sigma.group(1))
            sigma_m_map[aid] = float(parts[1])

    if len(pos_map) < 3:
        raise ValueError(f"File {path.name} does not contain at least 3 'anN_p' entries")

    configs: list[AnchorConfig] = []
    for aid in range(1, MAX_ANCHORS + 1):
        if aid in pos_map:
            x, y, z = pos_map[aid]
            bias_mm = bias_m_map.get(aid, 0.0) * 1000.0
            sigma_mm = sigma_m_map.get(aid, 0.030) * 1000.0
            configs.append(
                AnchorConfig(
                    anchor_id=aid,
                    x=x,
                    y=y,
                    z=z,
                    enabled=True,
                    bias_mm=bias_mm,
                    sigma_mm=sigma_mm,
                )
            )
        else:
            configs.append(AnchorConfig(aid, 0.0, 0.0, 0.0, enabled=False))
    return configs


def save_anchors_rig_txt(path: Path, anchors: Sequence[AnchorConfig]) -> None:
    """Save anchor positions, biases (m), and sigmas (m) in anchors_rig40.txt format."""
    lines = [
        "# UWB Anchor Layout & Calibration file (compatible with uwb_calib.py / uwb_pos.py)",
        "# Format: anN_p, x_m, y_m, z_m  |  anN_bias, bias_m (meas = true + bias)",
    ]
    ordered = sorted(anchors, key=lambda a: a.anchor_id)
    for a in ordered:
        if a.enabled:
            lines.append(f"an{a.anchor_id}_p, {a.x:.3f}, {a.y:.3f}, {a.z:.3f}")
    lines.append("# Per-anchor range bias (m) and white noise sigma (m)")
    for a in ordered:
        if a.enabled:
            lines.append(f"an{a.anchor_id}_bias, {a.bias_mm / 1000.0:.4f}")
            lines.append(f"an{a.anchor_id}_sigma, {a.sigma_mm / 1000.0:.4f}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def load_anchor_layout_json(path: Path) -> list[AnchorConfig]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    items = payload.get("anchors", payload) if isinstance(payload, dict) else payload
    by_id: dict[int, AnchorConfig] = {}
    for item in items:
        aid = int(item.get("anchor_id", item.get("id", 0)))
        if 1 <= aid <= MAX_ANCHORS:
            by_id[aid] = AnchorConfig(
                anchor_id=aid,
                x=float(item.get("x", 0.0)),
                y=float(item.get("y", 0.0)),
                z=float(item.get("z", 0.0)),
                enabled=bool(item.get("enabled", True)),
                bias_mm=float(item.get("bias_mm", 0.0)),
                sigma_mm=float(item.get("sigma_mm", 30.0)),
            )
    return [
        by_id.get(aid, AnchorConfig(aid, 0.0, 0.0, 0.0, enabled=False))
        for aid in range(1, MAX_ANCHORS + 1)
    ]


def save_anchor_layout_json(path: Path, anchors: Sequence[AnchorConfig]) -> None:
    payload = {
        "schema": 2,
        "coordinate_system": "ENU",
        "unit": "m",
        "anchors": [
            {
                "anchor_id": a.anchor_id,
                "x": round(a.x, 4),
                "y": round(a.y, 4),
                "z": round(a.z, 4),
                "enabled": a.enabled,
                "bias_mm": round(a.bias_mm, 2),
                "sigma_mm": round(a.sigma_mm, 2),
            }
            for a in sorted(anchors, key=lambda x: x.anchor_id)
            if a.enabled
        ],
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


class UwbStateEngine:
    """High-rate state engine for live telemetry, 2D/3D solver, and fast calibration."""

    def __init__(self, initial_anchors: Sequence[AnchorConfig] = DEFAULT_PRESET_BOX8) -> None:
        self.anchors: dict[int, AnchorConfig] = {
            a.anchor_id: AnchorConfig(**asdict(a)) for a in initial_anchors
        }
        for aid in range(1, MAX_ANCHORS + 1):
            self.anchors.setdefault(aid, AnchorConfig(aid, 0.0, 0.0, 0.0, enabled=False))

        # Solver configuration
        self.solver_dimension: int = 3
        self.fixed_z_m: float = 0.0
        self.allow_uncalibrated_positioning: bool = True
        self.use_host_filter_for_pos: bool = True
        self.use_nlos_downweight: bool = True
        self.use_signal_quality: bool = True
        self.position_filter_mode: str = "robust_ekf"
        self.ekf_guard = EkfGuardSettings()
        # A capture ends when every enabled anchor reaches the target or after
        # this many seconds; slow anchors are then marked LOW_SAMPLES.
        self.cal_timeout_s: float = 30.0

        self.reset_live_state()
        self.spot_captures: list[CalibrationSpotCapture] = []
        self.calibration_method: str = "joint"
        self.joint_settings = JointCalibrationSettings()
        self.last_applied_calibration: dict | None = None
        self._joint_cache_key = None
        self._joint_cache_captures: tuple[CalibrationSpotCapture, ...] = ()
        self._joint_cache_result: JointCalibrationResult | None = None

    def reset_live_state(self) -> None:
        self.live: dict[int, AnchorLiveState] = {
            aid: AnchorLiveState(aid) for aid in range(1, MAX_ANCHORS + 1)
        }
        self.host_filters: dict[int, HostRangeFilter] = {}
        self.range_ekf = RobustRangeEkf()
        self.radio_quality = RadioQualityModel()
        self._ekf_consumed: dict[int, int] = {}
        self._ekf_config = (self.solver_dimension, self.fixed_z_m)
        # Divergence guard: independent IRLS seed and evidence timer.
        self.ekf_guard_resets: int = 0
        self._irls_xyz: tuple[float, float, float] | None = None
        self._div_start_s: float | None = None
        self._div_last_s: float = -math.inf
        self._last_burst_received_s = -math.inf
        self._last_burst_tag_s = -math.inf
        # Ring buffers per anchor: (tag_time_s, raw_mm, corrected_mm, host_mm, nlos_db)
        self.anchor_history: dict[int, deque[tuple[float, float, float, float, float | None]]] = {
            aid: deque(maxlen=RING_HISTORY_LEN) for aid in range(1, MAX_ANCHORS + 1)
        }
        # Device-time rate window and robust MAD noise from 48 range samples.
        self._rate_window: dict[int, deque[tuple[float, int]]] = {
            aid: deque() for aid in range(1, MAX_ANCHORS + 1)
        }
        self._noise_window: dict[int, deque[int]] = {
            aid: deque(maxlen=48) for aid in range(1, MAX_ANCHORS + 1)
        }
        self._recent_corr_mm: dict[int, deque[float]] = {
            aid: deque(maxlen=9) for aid in range(1, MAX_ANCHORS + 1)
        }
        self._motion_win: dict[int, deque[tuple[float, float]]] = {
            aid: deque(maxlen=512) for aid in range(1, MAX_ANCHORS + 1)
        }
        self._anchor_moving_until: dict[int, float] = {}
        self._smooth_range_mm: dict[int, float] = {}
        self._lock_range_mm: dict[int, float] = {}
        self._last_range_t: dict[int, float] = {}
        self._first_rx_s: float | None = None
        self._smooth_xyz: tuple[float, float, float] | None = None
        self._prev_smooth_xyz: tuple[float, float, float] | None = None
        self._prev_raw_xyz: tuple[float, float, float] | None = None
        self._vel_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._best_anchor_count: int = 0
        self._last_solve_s: float | None = None
        self._global_moving_until_s: float = -1.0
        self.is_tag_moving: bool = False
        self._cycle_times: deque[tuple[float, int]] = deque(maxlen=250)

        self.latest_solution: PositionResult | None = None
        self.position_trail: deque[tuple[float, float, float, float]] = deque(maxlen=TRAIL_MAX_LEN)
        # Spatial/time-decimated path for full-loop 2D/3D rendering: (x, y, z, tag_time_s, stability_state, rms_mm)
        self.map_path: deque[tuple[float, float, float, float, str, float]] = deque(maxlen=MAP_TRAIL_LEN)
        # User-marked waypoints: (wp_index, label, x, y, z, tag_time_s)
        self.waypoints: list[tuple[int, str, float, float, float, float]] = []
        self.pending_waypoint_label: str = ""

        self.boot_id: int | None = None
        self.last_cycle_seq: int | None = None
        self.lost_cycles: int = 0
        self.total_bursts: int = 0
        self.cycle_rate_hz: float = 0.0
        self.period_us: int = 0
        self.ranging_mode_name: str = "—"
        self.calibrated_mask: int = 0
        self.active_anchor_mask: int = 0xFF
        self.uart_baud: int | None = None
        self.telemetry_features: int | None = None
        self.temperature_c: float | None = None
        self.vbat_v: float | None = None
        self.device_info: tp.DeviceInfoMessage | None = None
        self.diag_system: tp.DiagSystemMessage | None = None
        self.diag_anchor: dict[int, tp.DiagAnchorMessage] = {}
        self.diag_tracker = DiagTracker()
        self.rx_quality: tp.RxQualityMessage | None = None
        self.burst_settings: tp.BurstSettings | None = None
        self.info_flags: int = tp.INFO_FLAG_FPP_CORRECTED

        # Calibration state
        self.calibration_active: bool = False
        self._cal_mode: str = "spot"  # "spot" (multi-anchor 2D/3D) or "single" (1D)
        self._cal_spot_xyz: tuple[float, float, float] | None = None
        self._cal_single_aid: int | None = None
        self._cal_single_ref_mm: float = 1000.0
        self.cal_target_samples: int = 200
        self.cal_collected: int = 0
        self._cal_samples: dict[int, list[tuple[float, int, float | None, float | None]]] = {}
        self._cal_started_s: float | None = None
        self._cal_last_s: float = 0.0
        self.cal_elapsed_s: float = 0.0
        self.cal_progress: float = 0.0
        self.cal_anchor_counts: dict[int, int] = {}
        self.cal_last_message: str = ""

        # Accuracy evaluation state
        self.accuracy_eval_active: bool = False
        self.accuracy_ref_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.accuracy_target_samples: int = 100
        self._accuracy_fixes: list[PositionResult] = []
        self.latest_accuracy_report: AccuracyReport | None = None

    def set_anchors(self, configs: Sequence[AnchorConfig]) -> None:
        for a in configs:
            if 1 <= a.anchor_id <= MAX_ANCHORS:
                self.anchors[a.anchor_id] = AnchorConfig(**asdict(a))
        self.host_filters.clear()
        for q in self._recent_corr_mm.values():
            q.clear()
        for q in self._motion_win.values():
            q.clear()
        self._anchor_moving_until.clear()
        self._smooth_range_mm.clear()
        self._lock_range_mm.clear()
        self._last_range_t.clear()
        self._smooth_xyz = None
        self._prev_smooth_xyz = None
        self._prev_raw_xyz = None
        self._vel_xyz = (0.0, 0.0, 0.0)
        self._best_anchor_count = 0
        self._last_solve_s = None
        self._global_moving_until_s = -1.0
        self.is_tag_moving = False
        self._first_rx_s = None
        self._reset_ekf()
        self.radio_quality.reset()
        for win in self._noise_window.values():
            win.clear()
        for state in self.live.values():
            state.noise_sigma_mm = None
            state.quality = QualityResult()

    def _reset_ekf(self) -> None:
        self.range_ekf.reset()
        self._ekf_consumed.clear()
        self._ekf_config = (self.solver_dimension, self.fixed_z_m)
        self._irls_xyz = None
        self._div_start_s = None
        self._div_last_s = -math.inf
        self.latest_solution = None
        self._prev_smooth_xyz = None
        self._prev_raw_xyz = None

    def set_position_filter_mode(self, mode: str) -> None:
        if mode not in ("legacy", "robust_ekf"):
            raise ValueError("Unknown position filter mode")
        if mode != self.position_filter_mode:
            self.position_filter_mode = mode
            self.set_anchors(list(self.anchors.values()))

    def mark_waypoint(self, label: str = "") -> tuple[int, str, float, float, float, float] | None:
        """Record a user waypoint marker at the current solved 3D position."""
        sol = self.latest_solution
        if sol is None:
            return None
        idx = len(self.waypoints) + 1
        clean_lbl = label.strip() if label.strip() else f"W{idx} ({sol.x:.2f},{sol.y:.2f})"
        wp = (idx, clean_lbl, sol.x, sol.y, sol.z, sol.tag_time_s)
        self.waypoints.append(wp)
        self.pending_waypoint_label = clean_lbl
        return wp

    def clear_trail_and_waypoints(self) -> None:
        self.position_trail.clear()
        self.map_path.clear()
        self.waypoints.clear()
        self.pending_waypoint_label = ""

    def current_dop(self) -> DopMetrics:
        pos = (
            (self.latest_solution.x, self.latest_solution.y, self.latest_solution.z)
            if self.latest_solution is not None
            else (0.0, 0.0, self.fixed_z_m)
        )
        return compute_dop(list(self.anchors.values()), pos, dimension=self.solver_dimension)

    # ------------------------------------------------------------------
    # Calibration & Accuracy Controls
    # ------------------------------------------------------------------
    def start_spot_calibration(
        self, spot_xyz: tuple[float, float, float], target_samples: int = 200
    ) -> None:
        """Calibrate all enabled anchors simultaneously from a known 2D/3D Tag position."""
        self._cal_mode = "spot"
        self._cal_spot_xyz = (float(spot_xyz[0]), float(spot_xyz[1]), float(spot_xyz[2]))
        self._cal_single_aid = None
        self.cal_target_samples = max(10, int(target_samples))
        self.cal_collected = 0
        self._cal_samples = {aid: [] for aid in range(1, MAX_ANCHORS + 1)}
        self._reset_calibration_progress()
        self.calibration_active = True

    def start_single_anchor_calibration(
        self, anchor_id: int, reference_m: float, target_samples: int = 200
    ) -> None:
        """Calibrate a single anchor at a known tape-measured distance (1D)."""
        self._cal_mode = "single"
        self._cal_spot_xyz = None
        self._cal_single_aid = int(anchor_id)
        self._cal_single_ref_mm = float(reference_m) * 1000.0
        self.cal_target_samples = max(10, int(target_samples))
        self.cal_collected = 0
        self._cal_samples = {int(anchor_id): []}
        self._reset_calibration_progress()
        self.calibration_active = True

    def _reset_calibration_progress(self) -> None:
        self._cal_started_s = None
        self._cal_last_s = 0.0
        self.cal_elapsed_s = 0.0
        self.cal_progress = 0.0
        self.cal_anchor_counts = {}
        self.cal_last_message = ""

    def cancel_calibration(self) -> None:
        self.calibration_active = False
        self.cal_collected = 0
        self._cal_samples.clear()
        self._reset_calibration_progress()

    def calibration_min_samples(self) -> int:
        """Fewest samples for a usable bias; fewer are marked LOW_SAMPLES."""
        return min(self.cal_target_samples, max(20, self.cal_target_samples // 4))

    def finish_calibration_now(self) -> CalibrationSpotCapture | None:
        """Stop a running capture early and use the samples collected so far."""
        return self.finish_calibration(self._cal_last_s)

    def finish_calibration(self, now_s: float = 0.0) -> CalibrationSpotCapture | None:
        if not self.calibration_active:
            return None
        self.calibration_active = False
        results: dict[int, CalibrationAnchorResult] = {}
        min_samples = self.calibration_min_samples()
        slow_notes: list[str] = []

        for aid, samples in self._cal_samples.items():
            if self._cal_mode == "spot" and not self.anchors[aid].enabled:
                continue
            if len(samples) < min_samples:
                slow_notes.append(f"A{aid} {len(samples)}")
            if len(samples) < 5:
                continue
            cfg = self.anchors[aid]
            if self._cal_mode == "spot" and self._cal_spot_xyz is not None:
                true_mm = math.dist(self._cal_spot_xyz, (cfg.x, cfg.y, cfg.z)) * 1000.0
            else:
                true_mm = self._cal_single_ref_mm

            raws = [float(s[1]) for s in samples]
            mean_raw = statistics.fmean(raws)
            med_raw = _percentile(raws, 0.50)
            sigma = statistics.pstdev(raws) if len(raws) > 1 else 0.0
            p05 = _percentile(raws, 0.05)
            p95 = _percentile(raws, 0.95)
            k = max(1, len(raws) // 5)
            drift = statistics.fmean(raws[-k:]) - statistics.fmean(raws[:k])
            nlos_vals = [s[2] for s in samples if s[2] is not None]
            fp_vals = [s[3] for s in samples if s[3] is not None]
            bias_mm = med_raw - true_mm
            offset_mm = true_mm - med_raw
            if len(samples) < min_samples:
                quality = "LOW_SAMPLES"   # reported, but not used for biases
            elif sigma <= 45.0 and abs(drift) <= 35.0:
                quality = "GOOD"
            else:
                quality = "WARN"
            results[aid] = CalibrationAnchorResult(
                anchor_id=aid,
                samples=len(samples),
                true_distance_mm=true_mm,
                mean_raw_mm=mean_raw,
                median_raw_mm=med_raw,
                bias_mm=bias_mm,
                offset_mm=offset_mm,
                sigma_mm=sigma,
                p05_mm=p05,
                p95_mm=p95,
                drift_mm=drift,
                mean_nlos_db=statistics.fmean(nlos_vals) if nlos_vals else None,
                mean_fp_dbm=statistics.fmean(fp_vals) if fp_vals else None,
                quality=quality,
            )

        slow_msg = (
            f"Too few samples after {self.cal_elapsed_s:.0f} s (need {min_samples}): "
            f"{', '.join(slow_notes)}. These anchors are not calibrated; check their power, "
            "line of sight and radio link, or disable them."
            if slow_notes else ""
        )
        if not any(r.quality != "LOW_SAMPLES" for r in results.values()):
            self.cal_last_message = "Capture produced no usable anchor. " + slow_msg
            if not results:
                return None

        usable = {aid: r for aid, r in results.items() if r.quality != "LOW_SAMPLES"}
        consistency = None
        if self._cal_mode == "spot" and self._cal_spot_xyz is not None:
            consistency = check_spot_consistency(
                list(self.anchors.values()),
                {aid: r.median_raw_mm for aid, r in (usable or results).items()},
                self._cal_spot_xyz,
                self.solver_dimension,
            )
        elif self._cal_single_aid is not None and self._cal_single_aid in results:
            b = results[self._cal_single_aid].bias_mm
            if abs(b) > CAL_BIAS_WARN_MM:
                consistency = SpotConsistencyCheck(
                    None, None, None, None, None, 1,
                    (f"A{self._cal_single_aid} bias {b:+.0f} mm is beyond ±{CAL_BIAS_WARN_MM:.0f} mm: "
                     "check the tape distance.",),
                )

        if slow_msg:
            consistency = (
                replace(consistency, warnings=consistency.warnings + (slow_msg,))
                if consistency is not None
                else SpotConsistencyCheck(None, None, None, None, None, len(usable), (slow_msg,))
            )

        capture = CalibrationSpotCapture(
            spot_index=max((c.spot_index for c in self.spot_captures), default=0) + 1,
            spot_xyz=self._cal_spot_xyz,
            single_anchor_id=self._cal_single_aid,
            reference_distance_m=(self._cal_single_ref_mm / 1000.0) if self._cal_single_aid else None,
            timestamp_s=now_s,
            anchor_results=results,
            consistency=consistency,
        )
        self.spot_captures.append(capture)
        return capture

    def set_calibration_method(self, method: str) -> None:
        if method not in CALIBRATION_METHODS:
            raise ValueError(f"Unknown calibration method: {method}")
        self.calibration_method = method

    def joint_calibration(self) -> JointCalibrationResult | None:
        """Solve once per capture identity, anchor geometry, dimension and settings."""
        if not self.spot_captures:
            self._joint_cache_key = None
            self._joint_cache_captures = ()
            self._joint_cache_result = None
            return None
        key = (tuple(id(c) for c in self.spot_captures),
               tuple((a.anchor_id, a.x, a.y, a.z, a.enabled) for a in sorted(self.anchors.values(), key=lambda a: a.anchor_id)),
               self.solver_dimension, self.joint_settings,
               tuple(type(value) for value in vars(self.joint_settings).values()))
        if key != self._joint_cache_key:
            anchors = [a for a in self.anchors.values() if a.enabled]
            spots, tapes = observations_from_captures(self.spot_captures, (a.anchor_id for a in anchors))
            self._joint_cache_result = solve_joint_calibration(anchors, spots, tapes, self.solver_dimension, self.joint_settings)
            self._joint_cache_captures = tuple(self.spot_captures)
            self._joint_cache_key = key
        return self._joint_cache_result

    def active_calibration_method(self) -> str:
        result = self.joint_calibration() if self.calibration_method == "joint" else None
        return "joint" if result is not None and result.ok and result.method == "joint" and result.identifiable else "median"

    def remove_capture(self, spot_index: int) -> bool:
        for i, capture in enumerate(self.spot_captures):
            if capture.spot_index == spot_index:
                del self.spot_captures[i]
                return True
        return False

    def aggregated_calibration(self) -> dict[int, tuple[float, float, int]]:
        """Return active biases, sample noise and number of usable captures."""
        legacy = self._median_aggregated_calibration()
        if self.active_calibration_method() != "joint":
            return legacy
        result = self.joint_calibration()
        return {a: (b.bias_mm, legacy.get(a, (0.0, 30.0, 0))[1], b.spots_used + b.tape_used)
                for a, b in result.biases.items()}

    def calibration_summary(self) -> dict:
        result = self.joint_calibration()
        aggregated = self.aggregated_calibration()
        return {"method_selected": self.calibration_method, "method_applied": self.active_calibration_method(),
                "aggregated": {str(a): {"bias_mm": float(b) if math.isfinite(b) else None,
                                       "sigma_mm": float(s) if math.isfinite(s) else None, "captures": n}
                               for a, (b, s, n) in aggregated.items()},
                "joint": result.to_dict() if result is not None else None}

    def _median_aggregated_calibration(self) -> dict[int, tuple[float, float, int]]:
        """Return {anchor_id: (median_bias_mm, mean_sigma_mm, spot_count)} across all captures."""
        out: dict[int, tuple[float, float, int]] = {}
        for aid in range(1, MAX_ANCHORS + 1):
            items = [
                c.anchor_results[aid]
                for c in self.spot_captures
                if aid in c.anchor_results and c.anchor_results[aid].quality != "LOW_SAMPLES"
            ]
            if not items:
                continue
            med_bias = _percentile([r.bias_mm for r in items], 0.50)
            mean_sigma = max(statistics.fmean(r.sigma_mm for r in items), 10.0)
            out[aid] = (med_bias, mean_sigma, len(items))
        return out

    def calibration_warnings(self) -> list[str]:
        """Sanity warnings of all current captures, newest last, prefixed by capture."""
        out: list[str] = []
        for cap in self.spot_captures:
            if cap.consistency is None:
                continue
            for msg in cap.consistency.warnings:
                out.append(f"Capture #{cap.spot_index}: {msg}")
        result = self.joint_calibration() if self.calibration_method == "joint" else None
        if result is not None and result.method == "joint":
            out.extend(f"Joint solve: {w.message}" for w in result.warnings if w.code in JOINT_WARNING_CODES)
        return out

    def apply_calibration_to_host(self) -> dict[int, float]:
        """Copy aggregated calibration biases & sigmas into active host AnchorConfigs."""
        agg = self.aggregated_calibration()
        applied: dict[int, float] = {}
        for aid, (bias_mm, sigma_mm, _count) in agg.items():
            self.anchors[aid].bias_mm = bias_mm
            self.anchors[aid].sigma_mm = sigma_mm
            applied[aid] = bias_mm
        method = self.active_calibration_method()
        result = self.joint_calibration()
        self.last_applied_calibration = {
            "applied_unix_s": time.time(), "method": method,
            "version": SOLVER_VERSION if method == "joint" else "per-spot-median-v1",
            "biases_mm": {str(a): float(b) if math.isfinite(b) else None for a, b in applied.items()},
            "spots": sum(c.spot_xyz is not None and c.single_anchor_id is None for c in self.spot_captures),
            "joint": result.to_dict() if result is not None else None,
        }
        return applied

    def start_accuracy_eval(
        self, reference_xyz: tuple[float, float, float], target_samples: int = 100
    ) -> None:
        self.accuracy_ref_xyz = (
            float(reference_xyz[0]),
            float(reference_xyz[1]),
            float(reference_xyz[2]),
        )
        self.accuracy_target_samples = max(10, int(target_samples))
        self._accuracy_fixes.clear()
        self.latest_accuracy_report = None
        self.accuracy_eval_active = True

    def stop_accuracy_eval(self) -> AccuracyReport | None:
        self.accuracy_eval_active = False
        if not self._accuracy_fixes:
            return None
        xs = [p.x for p in self._accuracy_fixes]
        ys = [p.y for p in self._accuracy_fixes]
        zs = [p.z for p in self._accuracy_fixes]
        rx, ry, rz = self.accuracy_ref_xyz

        err_2d = [math.hypot(x - rx, y - ry) for x, y in zip(xs, ys)]
        err_3d = [math.dist((x, y, z), (rx, ry, rz)) for x, y, z in zip(xs, ys, zs)]
        mx, my, mz = statistics.fmean(xs), statistics.fmean(ys), statistics.fmean(zs)
        sx = statistics.pstdev(xs) if len(xs) > 1 else 0.0
        sy = statistics.pstdev(ys) if len(ys) > 1 else 0.0
        sz = statistics.pstdev(zs) if len(zs) > 1 else 0.0

        self.latest_accuracy_report = AccuracyReport(
            samples=len(self._accuracy_fixes),
            dimension=self._accuracy_fixes[-1].dimension,
            reference_xyz=self.accuracy_ref_xyz,
            mean_xyz=(mx, my, mz),
            bias_xyz_m=(mx - rx, my - ry, mz - rz),
            std_xyz_m=(sx, sy, sz),
            rmse_2d_m=math.sqrt(statistics.fmean(e * e for e in err_2d)),
            rmse_3d_m=math.sqrt(statistics.fmean(e * e for e in err_3d)),
            cep50_m=_percentile(err_2d, 0.50),
            cep95_m=_percentile(err_2d, 0.95),
            sep50_m=_percentile(err_3d, 0.50),
            sep95_m=_percentile(err_3d, 0.95),
            mean_pdop=statistics.fmean(p.dop.pdop for p in self._accuracy_fixes),
        )
        return self.latest_accuracy_report

    # ------------------------------------------------------------------
    # Telemetry Stream Ingestion
    # ------------------------------------------------------------------
    def ingest_message(self, message: object, received_s: float) -> None:
        if isinstance(message, tp.RangeBurstMessage):
            self._ingest_burst(message, received_s)
        elif isinstance(message, tp.RangeMeasMessage):
            self._ingest_range_meas(message, received_s)
        elif isinstance(message, tp.RangeMessage):
            self._ingest_range_snapshot(message, received_s)
        elif isinstance(message, tp.InfoMessage):
            self._ingest_info(message)
        elif isinstance(message, tp.DeviceInfoMessage):
            self._ingest_device_info(message)
        elif isinstance(message, tp.DiagSystemMessage):
            self._ingest_diag_system(message)
        elif isinstance(message, tp.DiagAnchorMessage):
            self.diag_anchor[message.anchor_id] = message
            self.diag_tracker.ingest(message)
        elif isinstance(message, tp.RxQualityMessage):
            if self.radio_quality.ingest(message, self.boot_id):
                self.rx_quality = message
                if message.temperature_c is not None:
                    self.temperature_c = message.temperature_c
                if message.vbat_v is not None:
                    self.vbat_v = message.vbat_v
        elif isinstance(message, tp.CmdAckMessage):
            self._ingest_cmd_ack(message)

    def _update_anchor_rate_and_noise(self, aid: int, received_s: float, raw_mm: int) -> None:
        win = self._rate_window[aid]
        win.append((received_s, raw_mm))
        cutoff = received_s - 2.0
        while win and win[0][0] < cutoff:
            win.popleft()
        state = self.live[aid]
        if len(win) >= 2:
            span = max(received_s - win[0][0], 0.1)
            state.rate_hz = (len(win) - 1) / span if span >= 0.15 else float(len(win))

        # Always advance the window: rejecting relative to its last retained
        # sample can lock the estimator forever after a >500 mm range change.
        nwin = self._noise_window[aid]
        nwin.append(raw_mm)
        if len(nwin) >= 6 and (state.rx_count % 8 == 0 or state.noise_sigma_mm is None):
            diffs = [b - a for a, b in zip(nwin, list(nwin)[1:])]
            center = statistics.median(diffs)
            mad = statistics.median(abs(v - center) for v in diffs)
            inst_sig = max(5.0, min(500.0, 1.4826 * mad / math.sqrt(2.0)))
            old = state.noise_sigma_mm
            state.noise_sigma_mm = inst_sig if old is None else 0.8 * old + 0.2 * inst_sig

    def _update_filtered_range(
        self,
        aid: int,
        corrected_mm: float,
        fpp_cdbm: int,
        tag_time_ms: int,
        received_s: float,
    ) -> float:
        if self._first_rx_s is None:
            self._first_rx_s = received_s
        dt_a = min(max(received_s - self._last_range_t.get(aid, received_s - 0.02), 0.002), 1.5)
        self._last_range_t[aid] = received_s

        # Time-aware spike gate so slow anchors (e.g. A2 at 1 Hz) are never blocked during walking
        win = self._recent_corr_mm[aid]
        if dt_a > 0.35:
            win.clear()
        if len(win) >= 5:
            med_prev = statistics.median(win)
            win.append(corrected_mm)
            spike_limit = 160.0 + 1800.0 * dt_a
            if abs(corrected_mm - med_prev) > spike_limit:
                gated_mm = statistics.median(win)
            else:
                gated_mm = corrected_mm
        else:
            win.append(corrected_mm)
            gated_mm = corrected_mm

        cv_mm = float(
            self.host_filters.setdefault(aid, HostRangeFilter()).update(
                round(gated_mm),
                tp.fpp_to_legacy_scale_cdbm(fpp_cdbm, self.info_flags),
                tag_time_ms,
            )
        )

        sig = max(self.live[aid].noise_sigma_mm or self.anchors[aid].sigma_mm, 18.0)

        mwin = self._motion_win[aid]
        mwin.append((received_s, gated_mm))
        while len(mwin) > 2 and (received_s - mwin[0][0]) > 0.45:
            mwin.popleft()
        drift_045_mm = (
            abs(gated_mm - mwin[0][1])
            if len(mwin) >= 3 and (received_s - mwin[0][0]) >= 0.12
            else 0.0
        )

        if aid not in self._lock_range_mm:
            self._lock_range_mm[aid] = gated_mm
        else:
            dev = abs(gated_mm - self._lock_range_mm[aid])
            if dev >= max(2.8 * sig, 55.0) or drift_045_mm >= max(2.5 * sig, 50.0):
                self._anchor_moving_until[aid] = received_s + 0.55
                self._lock_range_mm[aid] = gated_mm
        # The checkbox now really selects the CV output. EKF uses corrected
        # observations directly and does not consume this correlated estimate.
        return cv_mm

    def _reset_device_epoch(self) -> None:
        self.radio_quality.reset()
        self.rx_quality = None
        self.host_filters.clear()
        for q in self._recent_corr_mm.values():
            q.clear()
        for q in self._motion_win.values():
            q.clear()
        self._anchor_moving_until.clear()
        self._smooth_range_mm.clear()
        self._lock_range_mm.clear()
        self._last_range_t.clear()
        self._smooth_xyz = None
        self._prev_smooth_xyz = None
        self._prev_raw_xyz = None
        self._vel_xyz = (0.0, 0.0, 0.0)
        self._best_anchor_count = 0
        self._last_solve_s = None
        self._global_moving_until_s = -1.0
        self.is_tag_moving = False
        self.last_cycle_seq = None
        self._reset_ekf()
        self._first_rx_s = None
        self._last_burst_tag_s = -math.inf
        self._cycle_times.clear()
        for q in self._noise_window.values():
            q.clear()
        for q in self._rate_window.values():
            q.clear()
        for state in self.live.values():
            state.last_update_s = -1.0
            state.noise_sigma_mm = None
            state.quality = QualityResult()

    def _assess_radio_quality(self, state: AnchorLiveState) -> None:
        state.quality = self.radio_quality.assess(
            state.anchor_id, self.boot_id, state.tag_time_s,
            fp_dbm=state.fp_dbm, nlos_db=state.nlos_db, ci_ppm=state.ci_ppm,
            corrected_power=bool(self.info_flags & tp.INFO_FLAG_FPP_CORRECTED),
            nlos_enabled=self.use_nlos_downweight, late=state.late,
        )

    def applied_radio_variance_scale(self, aid: int) -> float | None:
        if self.position_filter_mode != "robust_ekf":
            return None
        state = self.live[aid]
        base = nlos_variance_scale(state.nlos_db) if self.use_nlos_downweight else 1.0
        return base * (state.quality.extra_variance_scale if self.use_signal_quality else 1.0)

    def _ingest_burst(self, msg: tp.RangeBurstMessage, received_s: float) -> None:
        if self.boot_id is not None and msg.boot_id != self.boot_id:
            self._reset_device_epoch()
        if msg.t_us / 1e6 <= self._last_burst_tag_s:
            return  # Duplicate or out-of-order Burst must not feed any filter.
        self._last_burst_tag_s = msg.t_us / 1e6
        self._last_burst_received_s = received_s
        self.boot_id = msg.boot_id
        if self.last_cycle_seq is not None:
            delta = (msg.cycle_seq - self.last_cycle_seq) & 0xFFFFFFFF
            if 1 < delta < 100000:
                self.lost_cycles += delta - 1
        self.last_cycle_seq = msg.cycle_seq
        self.total_bursts += 1
        self.period_us = msg.period_us
        self.ranging_mode_name = "Burst DS-TWR"

        self._cycle_times.append((msg.t_us / 1e6, msg.cycle_seq))
        while len(self._cycle_times) > 2 and (msg.t_us / 1e6 - self._cycle_times[0][0]) > 2.0:
            self._cycle_times.popleft()
        if len(self._cycle_times) >= 2:
            dt = self._cycle_times[-1][0] - self._cycle_times[0][0]
            dc = (self._cycle_times[-1][1] - self._cycle_times[0][1]) & 0xFFFFFFFF
            if dt > 0.02 and 0 < dc < 100000:
                self.cycle_rate_hz = dc / dt

        tag_time_s = msg.t_us / 1e6
        tag_time_ms = int(msg.t_us // 1000) & 0xFFFFFFFF
        updated_any = False

        for entry in msg.anchors:
            aid = entry.anchor_id
            if not (1 <= aid <= MAX_ANCHORS):
                continue
            state = self.live[aid]
            if not entry.radio_ok or entry.range_mm is None:
                if not state.is_fresh(received_s):
                    state.radio_ok = False
                    state.status = entry.status
                continue

            updated_any = True
            state.mode_name = entry.mode_name
            state.status = entry.status
            state.radio_ok = True
            state.late = bool(entry.flags & tp.BURST_FLAG_ANCHOR_LATE)

            cfg = self.anchors[aid]
            cal_ok = bool(entry.flags & tp.BURST_FLAG_CAL_OK)
            valid_flag = bool(entry.flags & tp.BURST_FLAG_VALID)
            state.fw_calibrated = cal_ok
            state.valid = valid_flag
            state.last_update_s = received_s
            state.tag_time_s = tag_time_s
            state.rx_count += 1

            # Reconstruct raw_mm before firmware offset if TAG applied its own calibration
            if cal_ok and cfg.fw_bias_mm != 0.0:
                raw_mm = int(round(entry.range_mm + cfg.fw_bias_mm))
            else:
                raw_mm = int(entry.range_mm)
            state.raw_mm = raw_mm

            # Corrected range: use firmware value directly if firmware calibrated and host has no distinct override
            if cal_ok and (cfg.bias_mm == 0.0 or abs(cfg.bias_mm - cfg.fw_bias_mm) < 1.0):
                corrected_mm = float(entry.range_mm)
            else:
                corrected_mm = float(raw_mm) - cfg.bias_mm
            state.corrected_mm = corrected_mm
            state.fw_filtered_mm = entry.range_mm if valid_flag else None

            self._update_anchor_rate_and_noise(aid, tag_time_s, raw_mm)
            fpp_cdbm = entry.fp_cdbm if entry.fp_cdbm != tp.INT16_MIN else -8200
            host_mm = self._update_filtered_range(aid, corrected_mm, fpp_cdbm, tag_time_ms, tag_time_s)
            state.host_filtered_mm = host_mm

            fp_dbm = None if entry.fp_cdbm == tp.INT16_MIN else entry.fp_cdbm / 100.0
            rx_dbm = None if entry.rx_cdbm == tp.INT16_MIN else entry.rx_cdbm / 100.0
            nlos_db = (rx_dbm - fp_dbm) if (fp_dbm is not None and rx_dbm is not None) else None
            ci_ppm = None if entry.ci_ppm_x100 == tp.INT16_MIN else entry.ci_ppm_x100 / 100.0
            state.fp_dbm = fp_dbm
            state.rx_dbm = rx_dbm
            state.nlos_db = nlos_db
            state.ci_ppm = ci_ppm
            self._assess_radio_quality(state)

            self.anchor_history[aid].append((tag_time_s, float(raw_mm), corrected_mm, host_mm, nlos_db))

            if self.calibration_active and aid in self._cal_samples:
                self._cal_samples[aid].append((tag_time_s, raw_mm, nlos_db, fp_dbm))

        self._check_calibration_progress(received_s)
        if updated_any or self.position_filter_mode == "robust_ekf":
            self._solve_current_position(received_s, tag_time_s)
        elif self.latest_solution is not None:
            self.position_trail.append(
                (self.latest_solution.x, self.latest_solution.y, self.latest_solution.z, tag_time_s)
            )

    def _ingest_range_meas(self, msg: tp.RangeMeasMessage, received_s: float) -> None:
        if self.boot_id is not None and msg.boot_id != self.boot_id:
            self._reset_device_epoch()
            self._last_burst_received_s = -math.inf
        self.boot_id = msg.boot_id
        if received_s - self._last_burst_received_s < 0.2:
            return  # Burst is authoritative; RANGE_MEAS can duplicate its radio samples.
        if not (msg.flags & tp.MEAS_FLAG_RADIO_OK) or msg.raw_mm <= 0:
            return
        aid = msg.anchor_id
        if not (1 <= aid <= MAX_ANCHORS):
            return
        cfg = self.anchors[aid]
        state = self.live[aid]
        tag_time_s = msg.meas_time_us / 1e6
        if state.last_update_s >= 0.0 and tag_time_s <= state.tag_time_s:
            return
        state.last_update_s = received_s
        state.tag_time_s = tag_time_s
        state.mode_name = msg.mode_name
        state.status = msg.status
        state.radio_ok = bool(msg.flags & tp.MEAS_FLAG_RADIO_OK)
        state.fw_calibrated = msg.calibrated
        state.valid = bool(msg.flags & tp.MEAS_FLAG_FILTER_OK)
        state.raw_mm = msg.raw_mm
        state.rx_count += 1

        corrected_mm = (
            float(msg.corrected_mm)
            if (msg.calibrated and cfg.bias_mm == 0.0)
            else float(msg.raw_mm) - cfg.bias_mm
        )
        state.corrected_mm = corrected_mm
        state.fw_filtered_mm = msg.filtered_mm if state.valid else None

        fpp_cdbm = msg.fp_cdbm if msg.fp_cdbm != tp.INT16_MIN else -8200
        host_mm = self._update_filtered_range(
            aid, corrected_mm, fpp_cdbm, int(tag_time_s * 1000) & 0xFFFFFFFF, tag_time_s
        )
        state.host_filtered_mm = host_mm
        state.fp_dbm = None if msg.fp_cdbm == tp.INT16_MIN else msg.fp_cdbm / 100.0
        state.rx_dbm = None if msg.rx_cdbm == tp.INT16_MIN else msg.rx_cdbm / 100.0
        state.nlos_db = msg.nlos_indicator_db
        state.ci_ppm = None if msg.ci_ppm_x100 == tp.INT16_MIN else msg.ci_ppm_x100 / 100.0
        self._assess_radio_quality(state)

        self._update_anchor_rate_and_noise(aid, tag_time_s, msg.raw_mm)
        self.anchor_history[aid].append(
            (tag_time_s, float(msg.raw_mm), corrected_mm, host_mm, state.nlos_db)
        )
        if self.calibration_active and aid in self._cal_samples and msg.raw_mm > 0:
            self._cal_samples[aid].append((tag_time_s, msg.raw_mm, state.nlos_db, state.fp_dbm))
            self._check_calibration_progress(received_s)
        self._solve_current_position(received_s, tag_time_s)

    def _ingest_range_snapshot(self, msg: tp.RangeMessage, received_s: float) -> None:
        # If Burst mode is actively streaming (<0.2s ago), do not let the 5 Hz snapshot
        # overwrite fresher burst measurements or duplicate history points.
        if self.total_bursts > 0 and any(
            (received_s - s.last_update_s) < 0.15 for s in self.live.values() if s.last_update_s > 0
        ):
            return

        tag_time_s = msg.time_ms / 1000.0
        for sample in msg.samples:
            aid = sample.anchor_id
            if not (1 <= aid <= MAX_ANCHORS):
                continue
            diagnostic = bool(
                sample.status & (tp.STATUS_CALIBRATION_MISSING | tp.STATUS_RANGE_REJECT)
            )
            if not (sample.valid or diagnostic) or sample.raw_mm <= 0:
                continue
            cfg = self.anchors[aid]
            state = self.live[aid]
            state.last_update_s = received_s
            state.tag_time_s = tag_time_s
            state.status = sample.status
            state.valid = sample.valid
            state.radio_ok = True
            state.raw_mm = sample.raw_mm
            state.rx_count += 1
            corrected_mm = float(sample.raw_mm) - cfg.bias_mm
            state.corrected_mm = corrected_mm
            state.fw_filtered_mm = sample.filtered_mm if sample.valid and sample.filtered_mm > 0 else None
            host_mm = self._update_filtered_range(
                aid, corrected_mm, sample.fpp_cdbm, msg.time_ms, tag_time_s
            )
            state.host_filtered_mm = host_mm
            state.fp_dbm = sample.fpp_cdbm / 100.0
            state.nlos_db = state.ci_ppm = None
            self._assess_radio_quality(state)
            self._update_anchor_rate_and_noise(aid, tag_time_s, sample.raw_mm)
            self.anchor_history[aid].append(
                (tag_time_s, float(sample.raw_mm), corrected_mm, host_mm, state.nlos_db)
            )
            if self.calibration_active and aid in self._cal_samples:
                self._cal_samples[aid].append((tag_time_s, sample.raw_mm, state.nlos_db, state.fp_dbm))

        self._check_calibration_progress(received_s)
        self._solve_current_position(received_s, tag_time_s)

    def _check_calibration_progress(self, now_s: float) -> None:
        if not self.calibration_active:
            return
        if self._cal_started_s is None or now_s < self._cal_started_s:
            self._cal_started_s = now_s
        self._cal_last_s = now_s
        self.cal_elapsed_s = now_s - self._cal_started_s
        target = self.cal_target_samples
        if self._cal_mode == "single" and self._cal_single_aid is not None:
            ids = [self._cal_single_aid]
        else:
            ids = [aid for aid in self._cal_samples if self.anchors[aid].enabled]
        counts = {aid: len(self._cal_samples.get(aid, ())) for aid in ids}
        self.cal_anchor_counts = counts
        # Mean progress: one slow or dead anchor must not freeze the bar (the
        # old minimum-count rule stalled near 15% when one link barely worked).
        self.cal_progress = (
            statistics.fmean(min(c, target) / target for c in counts.values()) if counts else 0.0
        )
        self.cal_collected = int(round(self.cal_progress * target))
        all_done = bool(counts) and all(c >= target for c in counts.values())
        if all_done or self.cal_elapsed_s >= self.cal_timeout_s:
            self.finish_calibration(now_s)

    def _solve_ekf_position(self, tag_time_s: float) -> None:
        if self._ekf_config != (self.solver_dimension, self.fixed_z_m):
            self._reset_ekf()
        ekf = self.range_ekf
        tracking = ekf.predict(tag_time_s)
        fresh = {
            aid: st for aid, st in self.live.items()
            if self.anchors[aid].enabled and st.last_update_s >= 0.0
            and 0.0 <= tag_time_s - st.tag_time_s <= 0.16
            and st.corrected_mm > 10.0 and math.isfinite(st.corrected_mm)
            and (self.allow_uncalibrated_positioning or st.valid or st.fw_calibrated)
        }
        # IRLS remains an independent diagnostic and bootstrap, never a second
        # EKF measurement derived from the same radio observations.
        configs = []
        for a in self.anchors.values():
            ac = AnchorConfig(**asdict(a))
            ac.sigma_mm = max(20.0, self.live[a.anchor_id].noise_sigma_mm or a.sigma_mm)
            if self.use_signal_quality and not tracking:
                ac.sigma_mm *= math.sqrt(self.live[a.anchor_id].quality.extra_variance_scale)
            configs.append(ac)
        # Seed IRLS from its own previous fix, not from the EKF state, so the
        # diagnostic cannot be pulled into the same wrong solution as the EKF.
        diagnostic = solve_position_robust(
            configs, {aid: st.corrected_mm for aid, st in fresh.items()},
            dimension=self.solver_dimension, fixed_z=self.fixed_z_m,
            nlos_db_map={aid: st.nlos_db for aid, st in fresh.items()} if self.use_nlos_downweight else None,
            initial_xyz=self._irls_xyz if tracking else None, tag_time_s=tag_time_s,
        )
        if diagnostic is not None:
            self._irls_xyz = (diagnostic.x, diagnostic.y, diagnostic.z)
        guard_reason = ""
        if not tracking:
            if diagnostic is None or diagnostic.dop.pdop > 10.0 or diagnostic.rms_m > 0.25:
                self._publish_solution(None, tag_time_s)
                return
            ekf.initialize((diagnostic.x, diagnostic.y, diagnostic.z), tag_time_s,
                           self.solver_dimension, self.fixed_z_m)
            self._prev_smooth_xyz = None
        elif diagnostic is not None and self._ekf_diverged(
            diagnostic, {aid: st.corrected_mm for aid, st in fresh.items()}, tag_time_s
        ):
            offset_mm = math.dist(tuple(float(v) for v in ekf.x[:3]),
                                  (diagnostic.x, diagnostic.y, diagnostic.z)) * 1000.0
            # Restart from the consistent fix; this cycle's ranges then update it normally.
            ekf.initialize((diagnostic.x, diagnostic.y, diagnostic.z), tag_time_s,
                           self.solver_dimension, self.fixed_z_m)
            self.ekf_guard_resets += 1
            guard_reason = f"EKF re-anchored to IRLS ({offset_mm:.0f}mm)"
        observations = []
        for aid, st in fresh.items():
            if self._ekf_consumed.get(aid) == st.rx_count:
                continue
            self._ekf_consumed[aid] = st.rx_count
            a = self.anchors[aid]
            # Do not fuse delayed observations at the current epoch. Missing
            # links are handled by prediction, not repeated stale ranges.
            if tag_time_s - st.tag_time_s > 0.000001:
                continue
            sigma = max(a.sigma_mm, st.noise_sigma_mm or 0.0, 20.0) / 1000.0
            observations.append(RangeObservation(
                aid, (a.x, a.y, a.z), st.corrected_mm / 1000.0, sigma,
                st.nlos_db if self.use_nlos_downweight else None,
                st.quality.extra_variance_scale if self.use_signal_quality else 1.0,
            ))
        ekf.update(observations, tag_time_s)
        if ekf.prediction_age_s > ekf.settings.max_prediction_s:
            self._publish_solution(None, tag_time_s)
            return
        assert ekf.x is not None
        xyz = tuple(float(v) for v in ekf.x[:3])
        velocity = tuple(float(v) for v in ekf.x[3:])
        speed = math.sqrt(sum(v * v for v in velocity))
        self.is_tag_moving = speed > 0.08
        jump_mm = math.dist(xyz, self._prev_smooth_xyz) * 1000 if self._prev_smooth_xyz else 0.0
        self._prev_smooth_xyz = self._smooth_xyz = xyz
        raw = ((diagnostic.x, diagnostic.y, diagnostic.z) if diagnostic else (self._prev_raw_xyz or xyz))
        self._prev_raw_xyz = raw
        by_id = {o.anchor_id: o for o in observations}
        residuals = {aid: math.dist(xyz, by_id[aid].xyz) - by_id[aid].range_m for aid in ekf.accepted_ids}
        rms = math.sqrt(statistics.fmean(r * r for r in residuals.values())) if residuals else 0.0
        # Geometry and fix completeness use the anchors accepted within the
        # full-fix window: a lossy Burst may carry only one range per cycle.
        recent = ekf.recent_ids
        dop = compute_dop([self.anchors[aid] for aid in recent], xyz, self.solver_dimension)
        reasons = [guard_reason] if guard_reason else []
        if ekf.rejected:
            reasons.append("Gate " + ",".join(f"A{aid}" for aid in sorted(ekf.rejected)))
        if len(recent) < self.solver_dimension + 1:
            reasons.append(f"Partial/predicted fix ({ekf.prediction_age_s:.3f}s)")
        if rms > 0.110:
            reasons.append(f"High RMS ({rms * 1000:.0f}mm)")
        if dop.pdop > 5.0:
            reasons.append(f"Geometry (PDOP {dop.pdop:.1f})")
        unstable = rms > 0.110 or dop.pdop > 5.0 or len(recent) < self.solver_dimension + 1
        stability = "UNSTABLE" if unstable else (
            "WARN" if rms > 0.065 or ekf.rejected or guard_reason else "STABLE"
        )
        sol = PositionResult(
            *xyz, rms, max((abs(r) for r in residuals.values()), default=0.0), residuals,
            recent, tuple(sorted(ekf.rejected)), self.solver_dimension, dop, tag_time_s,
            raw_x=raw[0], raw_y=raw[1], raw_z=raw[2],
            vx_mps=velocity[0], vy_mps=velocity[1], vz_mps=velocity[2], speed_mps=speed,
            step_jump_mm=jump_mm, motion_state="MOVING" if self.is_tag_moving else "STATIONARY",
            stability_state=stability, instability_reason=" | ".join(reasons),
            filter_mode="robust_ekf", position_std_m=ekf.position_std_m,
            prediction_age_s=ekf.prediction_age_s,
        )
        self._publish_solution(sol, tag_time_s)

    def _ekf_diverged(
        self, diagnostic: PositionResult, ranges_mm: Mapping[int, float], tag_time_s: float
    ) -> bool:
        """True once a consistent IRLS fix has disagreed with the EKF for persist_s.

        Evidence also requires the same ranges to fit the IRLS position clearly
        better than the EKF position, so an IRLS local minimum that the EKF
        explains equally well never moves the EKF.
        """
        g = self.ekf_guard
        ekf = self.range_ekf
        if not g.enabled or ekf.x is None or ekf.p is None:
            return False
        if (
            len(diagnostic.used_anchor_ids) < self.solver_dimension + 2
            or diagnostic.rms_m > g.max_irls_rms_m
            or diagnostic.dop.pdop > g.max_irls_pdop
        ):
            # No trustworthy independent evidence this cycle. The timer survives
            # only short gaps (max_evidence_gap_s) between pieces of evidence.
            return False
        n = 3 if self.solver_dimension == 3 else 2
        irls = np.array((diagnostic.x, diagnostic.y, diagnostic.z)[:n])
        offset = float(np.linalg.norm(ekf.x[:n] - irls))
        std = math.sqrt(max(float(np.trace(ekf.p[:n, :n])), 0.0))
        speed = float(np.linalg.norm(ekf.x[3:3 + n]))
        tolerance = max(g.min_offset_m, g.sigma_multiplier * std) + speed * g.range_age_margin_s
        ekf_xyz = (float(ekf.x[0]), float(ekf.x[1]), float(ekf.x[2]) if n == 3 else self.fixed_z_m)
        ekf_res = [
            ranges_mm[aid] / 1000.0 - math.dist(ekf_xyz, (self.anchors[aid].x, self.anchors[aid].y, self.anchors[aid].z))
            for aid in diagnostic.used_anchor_ids if aid in ranges_mm
        ]
        ekf_rms = math.sqrt(statistics.fmean(r * r for r in ekf_res)) if ekf_res else 0.0
        ranges_prefer_irls = (
            ekf_rms >= g.min_rms_ratio * diagnostic.rms_m
            and ekf_rms - diagnostic.rms_m >= g.min_rms_gain_m
        )
        if offset <= tolerance or not ranges_prefer_irls:
            self._div_start_s = None
            return False
        if self._div_start_s is None or tag_time_s - self._div_last_s > g.max_evidence_gap_s:
            self._div_start_s = tag_time_s
        self._div_last_s = tag_time_s
        if tag_time_s - self._div_start_s < g.persist_s:
            return False
        self._div_start_s = None
        return True

    def _solve_current_position(self, now_s: float, tag_time_s: float) -> None:
        if self.position_filter_mode == "robust_ekf":
            self._solve_ekf_position(tag_time_s)
            return
        now_s = tag_time_s
        moving_anchors = 0
        if self.use_host_filter_for_pos:
            for aid, state in self.live.items():
                if not self.anchors[aid].enabled or state.last_update_s < 0.0:
                    continue
                if (now_s - state.tag_time_s) <= 0.65 and now_s < self._anchor_moving_until.get(aid, -1.0):
                    moving_anchors += 1
            if moving_anchors >= 2:
                self._global_moving_until_s = max(self._global_moving_until_s, now_s + 0.65)
            self.is_tag_moving = (now_s < self._global_moving_until_s) or (moving_anchors >= 2)
        else:
            self.is_tag_moving = True

        max_age = (
            (1.2 if self.is_tag_moving else STATIONARY_HOLD_AGE_S)
            if self.use_host_filter_for_pos
            else FRESH_AGE_S
        )

        ranges_mm: dict[int, float] = {}
        nlos_map: dict[int, float | None] = {}
        fresh_ranges_mm: dict[int, float] = {}
        fresh_nlos_map: dict[int, float | None] = {}
        for aid, state in self.live.items():
            cfg = self.anchors[aid]
            age = now_s - state.tag_time_s
            if not cfg.enabled or state.last_update_s < 0.0 or age > max_age:
                continue
            if not self.allow_uncalibrated_positioning and not (state.valid or state.fw_calibrated):
                continue
            val = state.host_filtered_mm if self.use_host_filter_for_pos else state.corrected_mm
            if val is not None and val > 10.0:
                ranges_mm[aid] = val
                nlos_map[aid] = state.nlos_db
                if age <= 0.16:
                    fresh_ranges_mm[aid] = val
                    fresh_nlos_map[aid] = state.nlos_db

        min_needed = 4 if self.solver_dimension == 3 else 3
        if self.is_tag_moving and len(fresh_ranges_mm) >= min_needed:
            ranges_mm = fresh_ranges_mm
            nlos_map = fresh_nlos_map

        # When >=6 anchors are active, exclude any anchor with severe live white noise (>45 mm)
        # provided the remaining anchors still span 3D height cleanly.
        if self.use_host_filter_for_pos and len(ranges_mm) >= 6:
            noisy_candidates = sorted(
                (
                    aid
                    for aid in ranges_mm
                    if (self.live[aid].noise_sigma_mm or 0.0) > 45.0
                ),
                key=lambda a: self.live[a].noise_sigma_mm or 0.0,
                reverse=True,
            )
            for noisy_aid in noisy_candidates:
                if len(ranges_mm) < 6:
                    break
                rem_zs = [self.anchors[a].z for a in ranges_mm if a != noisy_aid]
                if max(rem_zs) - min(rem_zs) > 0.08:
                    ranges_mm.pop(noisy_aid, None)

        solver_anchors: list[AnchorConfig] = []
        for a in self.anchors.values():
            ac = AnchorConfig(**asdict(a))
            if self.use_host_filter_for_pos and a.anchor_id in ranges_mm:
                live_sig = self.live[a.anchor_id].noise_sigma_mm
                base_sig = live_sig if live_sig is not None else a.sigma_mm
                ac.sigma_mm = max(20.0, min(45.0, base_sig))
            solver_anchors.append(ac)

        prev_xyz = self._smooth_xyz or (
            (self.latest_solution.x, self.latest_solution.y, self.latest_solution.z)
            if self.latest_solution is not None
            else None
        )
        sol = solve_position_robust(
            anchors=solver_anchors,
            ranges_mm=ranges_mm,
            dimension=self.solver_dimension,
            fixed_z=self.fixed_z_m,
            nlos_db_map=nlos_map if self.use_nlos_downweight else None,
            initial_xyz=prev_xyz,
            tag_time_s=tag_time_s,
        )

        if sol is not None:
            raw_xyz = (sol.x, sol.y, sol.z)
            dt_s = min(
                max(now_s - (self._last_solve_s if self._last_solve_s is not None else now_s - 0.005), 0.001),
                0.25,
            )
            self._last_solve_s = now_s
            uptime = now_s - (self._first_rx_s if self._first_rx_s is not None else now_s)
            n_used = len(sol.used_anchor_ids)

            if self.use_host_filter_for_pos:
                if self._smooth_xyz is None or (n_used > self._best_anchor_count and uptime < 1.0):
                    self._smooth_xyz = raw_xyz
                    self._best_anchor_count = max(self._best_anchor_count, n_used)
                else:
                    if self.is_tag_moving:
                        self._best_anchor_count = n_used
                        ax = 1.0
                        az = 1.0
                    elif uptime < 2.0:
                        self._best_anchor_count = max(self._best_anchor_count, n_used)
                        ax = dt_s / (0.25 + dt_s)
                        az = dt_s / (0.35 + dt_s)
                    elif moving_anchors == 0:
                        self._best_anchor_count = max(self._best_anchor_count, n_used)
                        if n_used < self._best_anchor_count:
                            ax = 0.0
                            az = 0.0
                        else:
                            ax = dt_s / (4.0 + dt_s)
                            az = dt_s / (6.0 + dt_s)
                    else:
                        self._best_anchor_count = max(self._best_anchor_count, n_used)
                        ax = dt_s / (1.2 + dt_s)
                        az = dt_s / (2.0 + dt_s)

                    sx = self._smooth_xyz[0] + ax * (raw_xyz[0] - self._smooth_xyz[0])
                    sy = self._smooth_xyz[1] + ax * (raw_xyz[1] - self._smooth_xyz[1])
                    sz = (
                        (self._smooth_xyz[2] + az * (raw_xyz[2] - self._smooth_xyz[2]))
                        if self.solver_dimension == 3
                        else self.fixed_z_m
                    )
                    self._smooth_xyz = (sx, sy, sz)
            else:
                self._smooth_xyz = raw_xyz

            final_xyz = self._smooth_xyz if self._smooth_xyz is not None else raw_xyz
            if self._prev_smooth_xyz is not None:
                inst_vx = (final_xyz[0] - self._prev_smooth_xyz[0]) / dt_s
                inst_vy = (final_xyz[1] - self._prev_smooth_xyz[1]) / dt_s
                inst_vz = (final_xyz[2] - self._prev_smooth_xyz[2]) / dt_s
                av = dt_s / (0.12 + dt_s)
                self._vel_xyz = (
                    self._vel_xyz[0] + av * (inst_vx - self._vel_xyz[0]),
                    self._vel_xyz[1] + av * (inst_vy - self._vel_xyz[1]),
                    self._vel_xyz[2] + av * (inst_vz - self._vel_xyz[2]),
                )
                smooth_jump_mm = math.dist(final_xyz, self._prev_smooth_xyz) * 1000.0
            else:
                smooth_jump_mm = 0.0

            raw_jump_mm = (
                math.dist(raw_xyz, self._prev_raw_xyz) * 1000.0
                if self._prev_raw_xyz is not None
                else 0.0
            )
            self._prev_smooth_xyz = final_xyz
            self._prev_raw_xyz = raw_xyz

            vx, vy, vz = self._vel_xyz
            speed_mps = math.sqrt(vx * vx + vy * vy + vz * vz)
            step_jump_mm = smooth_jump_mm
            rms_mm = sol.rms_m * 1000.0
            max_res_mm = sol.max_residual_m * 1000.0
            motion_str = "MOVING" if self.is_tag_moving else "STATIONARY"

            reasons: list[str] = []
            if rms_mm > 110.0:
                reasons.append(f"High RMS ({rms_mm:.0f}mm)")
            elif rms_mm > 65.0:
                reasons.append(f"RMS {rms_mm:.0f}mm")
            if raw_jump_mm > 180.0 and not self.is_tag_moving:
                reasons.append(f"Raw spike ({raw_jump_mm:.0f}mm)")
            if step_jump_mm > 90.0:
                reasons.append(f"Jump {step_jump_mm:.0f}mm")
            if sol.excluded_anchor_ids:
                reasons.append("Outlier " + ",".join(f"A{a}" for a in sol.excluded_anchor_ids))
            if sol.dop.pdop > 4.5 or (sol.dimension == 3 and n_used < 4):
                reasons.append(f"Geometry (PDOP {sol.dop.pdop:.1f}, {n_used}A)")
            if sol.residuals_m:
                worst_aid = max(sol.residuals_m, key=lambda k: abs(sol.residuals_m[k]))
                worst_mm = sol.residuals_m[worst_aid] * 1000.0
                if abs(worst_mm) > 95.0:
                    reasons.append(f"A{worst_aid} res {worst_mm:+.0f}mm")

            if rms_mm > 110.0 or step_jump_mm > 90.0 or sol.dop.pdop > 5.0 or (sol.dimension == 3 and n_used < 4):
                stab_str = "UNSTABLE"
            elif rms_mm > 65.0 or step_jump_mm > 45.0 or bool(sol.excluded_anchor_ids) or raw_jump_mm > 160.0:
                stab_str = "WARN"
            else:
                stab_str = "STABLE"

            sol = PositionResult(
                x=final_xyz[0],
                y=final_xyz[1],
                z=final_xyz[2],
                rms_m=sol.rms_m,
                max_residual_m=sol.max_residual_m,
                residuals_m=sol.residuals_m,
                used_anchor_ids=sol.used_anchor_ids,
                excluded_anchor_ids=sol.excluded_anchor_ids,
                dimension=sol.dimension,
                dop=sol.dop,
                tag_time_s=sol.tag_time_s,
                raw_x=raw_xyz[0],
                raw_y=raw_xyz[1],
                raw_z=raw_xyz[2],
                vx_mps=vx,
                vy_mps=vy,
                vz_mps=vz,
                speed_mps=speed_mps,
                step_jump_mm=step_jump_mm,
                motion_state=motion_str,
                stability_state=stab_str,
                instability_reason=" | ".join(reasons),
            )

        self._publish_solution(sol, tag_time_s)

    def _publish_solution(self, sol: PositionResult | None, tag_time_s: float) -> None:
        self.latest_solution = sol
        for aid, state in self.live.items():
            if sol is not None and aid in sol.residuals_m:
                state.residual_mm = sol.residuals_m[aid] * 1000.0
            else:
                state.residual_mm = None

        if sol is not None:
            self.position_trail.append((sol.x, sol.y, sol.z, tag_time_s))
            if not self.map_path:
                self.map_path.append((sol.x, sol.y, sol.z, tag_time_s, sol.stability_state, sol.rms_m * 1000.0))
            else:
                lx, ly, lz, lt, lstab, _lrms = self.map_path[-1]
                if (
                    math.dist((sol.x, sol.y, sol.z), (lx, ly, lz)) >= 0.010
                    or abs(tag_time_s - lt) >= 0.50
                    or (sol.stability_state != lstab and abs(tag_time_s - lt) >= 0.15)
                ):
                    self.map_path.append((sol.x, sol.y, sol.z, tag_time_s, sol.stability_state, sol.rms_m * 1000.0))
            if self.accuracy_eval_active:
                self._accuracy_fixes.append(sol)
                if len(self._accuracy_fixes) >= self.accuracy_target_samples:
                    self.stop_accuracy_eval()

    def _ingest_info(self, msg: tp.InfoMessage) -> None:
        self.info_flags = msg.flags
        self.calibrated_mask = msg.calibrated_mask
        if msg.flags & tp.INFO_FLAG_BURST:
            self.ranging_mode_name = "Burst DS-TWR"
        else:
            self.ranging_mode_name = "DS-TWR" if msg.ranging_mode else "SS-TWR"
        for aid, offset_um in msg.active_offsets_um:
            if 1 <= aid <= MAX_ANCHORS:
                fw_b = offset_um / 1000.0
                cal_ok = bool(msg.calibrated_mask & (1 << (aid - 1)))
                self.anchors[aid].fw_bias_mm = fw_b
                self.anchors[aid].fw_calibrated = cal_ok
                if cal_ok and self.anchors[aid].bias_mm == 0.0 and fw_b != 0.0:
                    self.anchors[aid].bias_mm = fw_b

    def _ingest_device_info(self, msg: tp.DeviceInfoMessage) -> None:
        self.device_info = msg
        self.uart_baud = msg.uart_baud
        self.telemetry_features = msg.telemetry_features
        self.calibrated_mask = msg.calibrated_mask
        self.active_anchor_mask = msg.active_mask

    def _ingest_diag_system(self, msg: tp.DiagSystemMessage) -> None:
        self.diag_system = msg
        self.diag_tracker.ingest(msg)
        self.active_anchor_mask = msg.active_mask
        if msg.temperature_c is not None:
            self.temperature_c = msg.temperature_c
        if msg.vbat_v is not None:
            self.vbat_v = msg.vbat_v

    def _ingest_cmd_ack(self, ack: tp.CmdAckMessage) -> None:
        if not ack.ok:
            return
        if ack.command_id == tp.CMD_SET_TELEMETRY and ack.data:
            self.telemetry_features = ack.data[0]
        elif ack.command_id == tp.CMD_SET_BURST and len(ack.data) == 11:
            try:
                self.burst_settings = tp.BurstSettings.decode(ack.data)
            except tp.ProtocolError:
                pass
        elif ack.command_id == tp.CMD_GET_DS_CAL and len(ack.data) >= 7:
            for offset in range(0, len(ack.data) - 6, 7):
                aid, bias_um, cal_flag = struct.unpack_from("<HiB", ack.data, offset)
                if 1 <= aid <= MAX_ANCHORS:
                    self.anchors[aid].fw_bias_mm = bias_um / 1000.0
                    self.anchors[aid].fw_calibrated = bool(cal_flag)
                    self.anchors[aid].bias_mm = bias_um / 1000.0


# ----------------------------------------------------------------------
# Log Session Replay & Instability Analysis Engine
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class ReplayAnchorSnap:
    anchor_id: int
    raw_mm: float | None = None
    corr_mm: float | None = None
    filt_mm: float | None = None
    res_mm: float | None = None
    rate_hz: float = 0.0
    sigma_mm: float | None = None
    fp_dbm: float | None = None
    rx_dbm: float | None = None
    nlos_db: float | None = None
    status_str: str = "OFF"


@dataclass(frozen=True)
class ReplayFrame:
    frame_index: int
    host_time_s: float
    rel_time_s: float
    cycle_seq: int
    x: float
    y: float
    z: float
    raw_x: float
    raw_y: float
    raw_z: float
    speed_mps: float
    rms_mm: float
    max_res_mm: float
    step_jump_mm: float
    pdop: float
    hdop: float
    vdop: float
    used_anchor_ids: tuple[int, ...]
    excluded_anchor_ids: tuple[int, ...]
    motion_state: str
    stability_state: str
    instability_reason: str
    waypoint_label: str
    anchors: dict[int, ReplayAnchorSnap]


@dataclass(frozen=True)
class InstabilitySegment:
    segment_id: int
    severity: str               # "UNSTABLE" | "WARN"
    start_frame_idx: int
    peak_frame_idx: int
    end_frame_idx: int
    start_rel_s: float
    end_rel_s: float
    duration_s: float
    xyz_m: tuple[float, float, float]
    max_rms_mm: float
    max_jump_mm: float
    min_anchors: int
    used_anchors_str: str
    cause_summary: str


@dataclass
class LoadedLogSession:
    csv_path: Path
    anchors: list[AnchorConfig]
    frames: list[ReplayFrame]
    map_path: list[tuple[float, float, float, float, str, float]]
    waypoints: list[tuple[int, str, float, float, float, float]]
    unstable_segments: list[InstabilitySegment]
    duration_s: float
    path_length_2d_m: float
    path_length_3d_m: float
    stable_pct: float
    warn_pct: float
    unstable_pct: float
    total_csv_rows: int


def _safe_float(val: str | None, default: float | None = None) -> float | None:
    if val is None:
        return default
    s = val.strip()
    if not s:
        return default
    try:
        v = float(s)
        return v if math.isfinite(v) else default
    except ValueError:
        return default


def load_log_session(
    csv_path: Path,
    fallback_anchors: Sequence[AnchorConfig] = DEFAULT_PRESET_ROOM_66_36,
    re_solve: bool = False,
    max_frames: int = 12000,
    filter_mode: str | None = None,
    signal_quality: bool | None = None,
    progress: Callable[[int, int], None] | None = None,
    cancel_requested: Callable[[], bool] | None = None,
) -> LoadedLogSession:
    """Load a UWB CSV session log (enriched or legacy), reconstruct 3D loop trajectory, and detect unstable segments."""
    import csv as _csv

    anchors_list: list[AnchorConfig] = [AnchorConfig(**asdict(a)) for a in fallback_anchors]
    by_id: dict[int, AnchorConfig] = {a.anchor_id: a for a in anchors_list}
    for aid in range(1, MAX_ANCHORS + 1):
        by_id.setdefault(aid, AnchorConfig(aid, 0.0, 0.0, 0.0, enabled=False))

    waypoints: list[tuple[int, str, float, float, float, float]] = []
    meta: dict = {}
    meta_path = csv_path.parent / "session_metadata.json"
    if meta_path.is_file():
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            if isinstance(meta.get("anchors"), list):
                for item in meta["anchors"]:
                    aid = int(item.get("anchor_id", 0))
                    if 1 <= aid <= MAX_ANCHORS:
                        by_id[aid] = AnchorConfig(
                            anchor_id=aid,
                            x=float(item.get("x", 0.0)),
                            y=float(item.get("y", 0.0)),
                            z=float(item.get("z", 0.0)),
                            enabled=bool(item.get("enabled", True)),
                            bias_mm=float(item.get("bias_mm", 0.0)),
                            sigma_mm=float(item.get("sigma_mm", 30.0)),
                            fw_calibrated=bool(item.get("fw_calibrated", False)),
                            fw_bias_mm=float(item.get("fw_bias_mm", 0.0)),
                        )
            if isinstance(meta.get("waypoints"), list):
                for idx, wp in enumerate(meta["waypoints"], start=1):
                    waypoints.append(
                        (
                            int(wp.get("index", idx)),
                            str(wp.get("label", f"W{idx}")),
                            float(wp.get("x", 0.0)),
                            float(wp.get("y", 0.0)),
                            float(wp.get("z", 0.0)),
                            float(wp.get("time_s", 0.0)),
                        )
                    )
        except Exception:
            pass

    anchors_sorted = [by_id[aid] for aid in range(1, MAX_ANCHORS + 1)]

    # Count rows quickly to choose a clean stride if the log has hundreds of thousands of rows
    total_rows = 0
    with csv_path.open("r", encoding="utf-8", errors="replace") as f:
        for _ in f:
            total_rows += 1
    total_rows = max(0, total_rows - 1)
    stride = max(1, math.ceil(total_rows / max(1, max_frames)))

    sim_engine = UwbStateEngine(initial_anchors=anchors_sorted) if re_solve else None
    if sim_engine is not None:
        solver_config = meta.get("solver", {})
        sim_engine.solver_dimension = int(solver_config.get("dimension", 3))
        sim_engine.fixed_z_m = float(solver_config.get("fixed_z_m", 0.0))
        sim_engine.allow_uncalibrated_positioning = bool(solver_config.get("allow_uncalibrated_positioning", True))
        sim_engine.use_host_filter_for_pos = bool(solver_config.get("use_host_filter_for_pos", True))
        sim_engine.use_nlos_downweight = bool(solver_config.get("use_nlos_downweight", True))
        sim_engine.use_signal_quality = (bool(solver_config.get("use_signal_quality", True))
                                        if signal_quality is None else signal_quality)
        sim_engine.info_flags = int(meta.get("power_info_flags", sim_engine.info_flags))
        sim_engine.set_position_filter_mode(filter_mode or solver_config.get("position_filter_mode", "robust_ekf"))

    frames: list[ReplayFrame] = []
    map_path: list[tuple[float, float, float, float, str, float]] = []
    t0_host: float | None = None
    prev_xyz: tuple[float, float, float] | None = None
    last_row_xyz: tuple[float, float, float] | None = None
    prev_rel_s: float = 0.0
    path_2d = 0.0
    path_3d = 0.0
    n_stable = 0
    n_warn = 0
    n_unstable = 0
    last_seen_rel_s: dict[int, float] = {}
    last_raw_by_aid: dict[int, float] = {}
    last_fp_by_aid: dict[int, float | None] = {}
    last_nlos_by_aid: dict[int, float | None] = {}
    recent_rx_times: dict[int, deque[float]] = {aid: deque(maxlen=40) for aid in range(1, MAX_ANCHORS + 1)}

    with csv_path.open("r", encoding="utf-8", errors="replace", newline="") as f, QualityReplay(csv_path, re_solve) as quality_replay:
        header_line = f.readline()
        if not header_line:
            return LoadedLogSession(
                csv_path=csv_path,
                anchors=anchors_sorted,
                frames=[],
                map_path=[],
                waypoints=waypoints,
                unstable_segments=[],
                duration_s=0.0,
                path_length_2d_m=0.0,
                path_length_3d_m=0.0,
                stable_pct=100.0,
                warn_pct=0.0,
                unstable_pct=0.0,
                total_csv_rows=0,
            )
        headers = next(_csv.reader([header_line]))
        for row_idx, raw_line in enumerate(f):
            if cancel_requested is not None and cancel_requested():
                raise InterruptedError("Replay cancelled")
            if progress is not None and row_idx % 1000 == 0:
                progress(row_idx, total_rows)
            # Fast skip for massive logs when stride > 24: only parse the 20 rows leading up to each sampled frame
            if stride > 24 and not re_solve:
                rem = row_idx % stride
                if rem != 0 and (stride - rem) > 20 and ("W" not in raw_line):
                    continue
            vals = next(_csv.reader([raw_line]))
            row = dict(zip(headers, vals))

            host_t = _safe_float(row.get("host_time_s"), 0.0) or 0.0
            if t0_host is None:
                t0_host = host_t
            rel_s = max(0.0, host_t - t0_host)

            # Track per-anchor rolling state across all rows so TDMA burst logs have complete 8-anchor state
            for aid in range(1, MAX_ANCHORS + 1):
                rmm_row = _safe_float(row.get(f"a{aid}_mm"))
                if rmm_row is not None and rmm_row > 0:
                    last_seen_rel_s[aid] = rel_s
                    last_raw_by_aid[aid] = rmm_row
                    last_fp_by_aid[aid] = _safe_float(row.get(f"a{aid}_fp_dbm"))
                    last_nlos_by_aid[aid] = _safe_float(row.get(f"a{aid}_nlos_db"))
                    recent_rx_times[aid].append(rel_s)

            row_px = _safe_float(row.get("pos_x_m"))
            row_py = _safe_float(row.get("pos_y_m"))
            row_pz = _safe_float(row.get("pos_z_m"))
            row_step_jump_mm = 0.0
            if row_px is not None and row_py is not None and row_pz is not None:
                if last_row_xyz is not None:
                    row_step_jump_mm = math.dist((row_px, row_py, row_pz), last_row_xyz) * 1000.0
                last_row_xyz = (row_px, row_py, row_pz)

            # Restore anchor coords from CSV row 0 if metadata JSON was not present
            if row_idx == 0 and not meta_path.is_file():
                updated_cfg = False
                for aid in range(1, MAX_ANCHORS + 1):
                    cx = _safe_float(row.get(f"a{aid}_cfg_x"))
                    cy = _safe_float(row.get(f"a{aid}_cfg_y"))
                    cz = _safe_float(row.get(f"a{aid}_cfg_z"))
                    cb = _safe_float(row.get(f"a{aid}_bias_mm"))
                    if cx is not None and cy is not None and cz is not None:
                        by_id[aid].x = cx
                        by_id[aid].y = cy
                        by_id[aid].z = cz
                        if cb is not None:
                            by_id[aid].bias_mm = cb
                        updated_cfg = True
                if updated_cfg and sim_engine is not None:
                    sim_engine.set_anchors([by_id[a] for a in range(1, MAX_ANCHORS + 1)])

            wp_lbl = (row.get("waypoint_label") or "").strip()
            stab_col = (row.get("stability_state") or "").strip()
            is_important = bool(wp_lbl)
            display_row = row_idx % stride == 0 or is_important or row_idx == total_rows - 1
            if not display_row and sim_engine is None:
                continue

            seq = int(_safe_float(row.get("cycle_seq"), float(row_idx)) or row_idx)
            tag_us = int(_safe_float(row.get("tag_time_us"), rel_s * 1e6) or int(rel_s * 1e6))

            px = _safe_float(row.get("pos_x_m"))
            py = _safe_float(row.get("pos_y_m"))
            pz = _safe_float(row.get("pos_z_m"))

            if sim_engine is not None or px is None or py is None or pz is None:
                if sim_engine is None:
                    sim_engine = UwbStateEngine(initial_anchors=anchors_sorted)
                entries: list[tp.BurstAnchor] = []
                for aid in range(1, MAX_ANCHORS + 1):
                    rmm = _safe_float(row.get(f"a{aid}_mm"))
                    fp = _safe_float(row.get(f"a{aid}_fp_dbm"))
                    nlos = _safe_float(row.get(f"a{aid}_nlos_db"))
                    ci = _safe_float(row.get(f"a{aid}_ci_ppm"))
                    if rmm is not None and rmm > 0:
                        fp_c = int(round(fp * 100.0)) if fp is not None else -8200
                        rx_c = int(round((fp + nlos) * 100.0)) if (fp is not None and nlos is not None) else fp_c
                        ci_c = int(round(ci * 100.0)) if ci is not None else tp.INT16_MIN
                        entries.append(
                            tp.BurstAnchor(
                                anchor_id=aid,
                                status=int(_safe_float(row.get(f"a{aid}_status_code"), 0) or 0),
                                flags=int(_safe_float(row.get(f"a{aid}_flags"),
                                    tp.BURST_FLAG_RADIO_OK | (tp.BURST_FLAG_VALID | tp.BURST_FLAG_CAL_OK
                                    if by_id[aid].fw_calibrated else 0)) or 0),
                                range_mm=int(round(rmm)),
                                fp_cdbm=fp_c,
                                rx_cdbm=rx_c,
                                ci_ppm_x100=ci_c,
                            )
                        )
                burst = tp.RangeBurstMessage(
                    sequence=seq,
                    time_ms=tag_us // 1000,
                    boot_id=int(_safe_float(row.get("boot_id"), 1) or 1),
                    cycle_seq=seq,
                    meas_seq_first=seq * 8,
                    t_us=tag_us,
                    period_us=int(_safe_float(row.get("period_us"), 5000) or 5000),
                    anchors=tuple(entries),
                )
                quality_replay.feed_before_row(sim_engine, row_idx, host_t)
                sim_engine.info_flags = int(_safe_float(row.get("power_info_flags"), sim_engine.info_flags))
                if signal_quality is None and row.get("signal_quality_enabled"):
                    sim_engine.use_signal_quality = bool(int(row["signal_quality_enabled"]))
                sim_engine.ingest_message(burst, host_t if host_t > 0 else rel_s + 1.0)
                # Decimate only AFTER ingest. All radio samples feed the filter.
                if not display_row:
                    continue
                sol = sim_engine.latest_solution
                if sol is None:
                    continue
                px, py, pz = sol.x, sol.y, sol.z
                rx, ry, rz = sol.raw_x, sol.raw_y, sol.raw_z
                rms_mm = sol.rms_m * 1000.0
                max_res_mm = sol.max_residual_m * 1000.0
                jump_mm = sol.step_jump_mm
                speed = sol.speed_mps
                pdop, hdop, vdop = sol.dop.pdop, sol.dop.hdop, sol.dop.vdop
                used_ids = sol.used_anchor_ids
                excl_ids = sol.excluded_anchor_ids
                motion_str = sol.motion_state
                stab_str = sol.stability_state
                reason_str = sol.instability_reason
                snap_map: dict[int, ReplayAnchorSnap] = {}
                for aid in range(1, MAX_ANCHORS + 1):
                    st = sim_engine.live[aid]
                    snap_map[aid] = ReplayAnchorSnap(
                        anchor_id=aid,
                        raw_mm=float(st.raw_mm) if st.raw_mm > 0 else None,
                        corr_mm=st.corrected_mm if st.raw_mm > 0 else None,
                        filt_mm=st.host_filtered_mm,
                        res_mm=st.residual_mm,
                        rate_hz=st.rate_hz,
                        sigma_mm=st.noise_sigma_mm,
                        fp_dbm=st.fp_dbm,
                        rx_dbm=st.rx_dbm,
                        nlos_db=st.nlos_db,
                        status_str="OK" if st.radio_ok and st.raw_mm > 0 else "OFF",
                    )
            else:
                rx = _safe_float(row.get("raw_x_m"), px) or px
                ry = _safe_float(row.get("raw_y_m"), py) or py
                rz = _safe_float(row.get("raw_z_m"), pz) or pz
                rms_m = _safe_float(row.get("pos_rms_m"), 0.0) or 0.0
                rms_mm = rms_m * 1000.0
                max_res_mm = _safe_float(row.get("max_res_mm"), rms_mm) or rms_mm
                dt = max(rel_s - prev_rel_s, 0.005)
                strided_jump_mm = math.dist((px, py, pz), prev_xyz) * 1000.0 if prev_xyz is not None else 0.0
                calc_jump_mm = row_step_jump_mm if stride > 1 else strided_jump_mm
                jump_mm = _safe_float(row.get("step_jump_mm"), calc_jump_mm) or calc_jump_mm
                speed = _safe_float(row.get("speed_mps"), (strided_jump_mm / 1000.0) / dt) or 0.0
                pdop = _safe_float(row.get("pdop"), 1.8) or 1.8
                hdop = _safe_float(row.get("hdop"), 1.1) or 1.1
                vdop = _safe_float(row.get("vdop"), 1.4) or 1.4

                used_raw = (row.get("used_anchors") or "").strip()
                if used_raw:
                    used_ids = tuple(
                        int(tok[1:]) for tok in used_raw.replace(",", "|").split("|") if tok.startswith("A") and tok[1:].isdigit()
                    )
                else:
                    used_ids = tuple(
                        aid for aid in range(1, MAX_ANCHORS + 1)
                        if aid in last_seen_rel_s and (rel_s - last_seen_rel_s[aid]) <= 2.5
                    )
                excl_raw = (row.get("excluded_anchors") or "").strip()
                excl_ids = tuple(
                    int(tok[1:]) for tok in excl_raw.replace(",", "|").split("|") if tok.startswith("A") and tok[1:].isdigit()
                )
                motion_str = (row.get("motion_state") or ("MOVING" if speed > 0.08 else "STATIONARY")).strip()

                snap_map = {}
                worst_aid = 0
                worst_res = 0.0
                for aid in range(1, MAX_ANCHORS + 1):
                    active_recently = aid in last_seen_rel_s and (rel_s - last_seen_rel_s[aid]) <= 2.5
                    rmm = _safe_float(row.get(f"a{aid}_mm"))
                    if rmm is None and active_recently:
                        rmm = last_raw_by_aid.get(aid)
                    ac = by_id.get(aid)
                    default_corr = (rmm - ac.bias_mm) if (rmm is not None and ac is not None) else rmm
                    cmm = _safe_float(row.get(f"a{aid}_corr_mm"), default_corr)
                    fmm = _safe_float(row.get(f"a{aid}_filt_mm"), cmm)
                    res_a = _safe_float(row.get(f"a{aid}_res_mm"))
                    if res_a is None and fmm is not None and ac is not None and ac.enabled:
                        res_a = math.dist((px, py, pz), (ac.x, ac.y, ac.z)) * 1000.0 - fmm
                    if res_a is not None and abs(res_a) > abs(worst_res):
                        worst_res = res_a
                        worst_aid = aid
                    rx_win = recent_rx_times[aid]
                    while rx_win and (rel_s - rx_win[0]) > 2.5:
                        rx_win.popleft()
                    est_hz = (len(rx_win) - 1) / max(rel_s - rx_win[0], 0.2) if len(rx_win) >= 2 else 0.0
                    snap_map[aid] = ReplayAnchorSnap(
                        anchor_id=aid,
                        raw_mm=rmm,
                        corr_mm=cmm,
                        filt_mm=fmm,
                        res_mm=res_a,
                        rate_hz=_safe_float(row.get(f"a{aid}_hz"), est_hz) or est_hz,
                        sigma_mm=_safe_float(row.get(f"a{aid}_sigma_mm"), ac.sigma_mm if ac else None),
                        fp_dbm=_safe_float(row.get(f"a{aid}_fp_dbm"), last_fp_by_aid.get(aid)),
                        rx_dbm=_safe_float(row.get(f"a{aid}_rx_dbm")),
                        nlos_db=_safe_float(row.get(f"a{aid}_nlos_db"), last_nlos_by_aid.get(aid)),
                        status_str=(row.get(f"a{aid}_status") or ("OK" if active_recently else "OFF")).strip(),
                    )

                if stab_col in ("STABLE", "WARN", "UNSTABLE"):
                    stab_str = stab_col
                elif rms_mm > 110.0 or jump_mm > 90.0 or pdop > 5.0 or (rel_s > 0.6 and len(used_ids) < 4):
                    stab_str = "UNSTABLE"
                elif rms_mm > 65.0 or jump_mm > 45.0 or bool(excl_ids):
                    stab_str = "WARN"
                else:
                    stab_str = "STABLE"

                reason_str = (row.get("instability_reason") or "").strip()
                if not reason_str and stab_str != "STABLE":
                    r_parts: list[str] = []
                    if rms_mm > 65.0:
                        r_parts.append(f"RMS {rms_mm:.0f}mm")
                    if jump_mm > 45.0:
                        r_parts.append(f"Jump {jump_mm:.0f}mm")
                    if worst_aid and abs(worst_res) > 85.0:
                        r_parts.append(f"A{worst_aid} res {worst_res:+.0f}mm")
                    if len(used_ids) < 5:
                        r_parts.append(f"Only {len(used_ids)}A")
                    reason_str = " | ".join(r_parts) if r_parts else "Elevated variance"

            if prev_xyz is not None:
                d2 = math.hypot(px - prev_xyz[0], py - prev_xyz[1])
                d3 = math.dist((px, py, pz), prev_xyz)
                if d3 >= 0.008:
                    path_2d += d2
                    path_3d += d3
            prev_xyz = (px, py, pz)
            prev_rel_s = rel_s

            if stab_str == "UNSTABLE":
                n_unstable += 1
            elif stab_str == "WARN":
                n_warn += 1
            else:
                n_stable += 1

            if wp_lbl and not any(w[1] == wp_lbl for w in waypoints):
                waypoints.append((len(waypoints) + 1, wp_lbl, px, py, pz, rel_s))

            frame = ReplayFrame(
                frame_index=len(frames),
                host_time_s=host_t,
                rel_time_s=rel_s,
                cycle_seq=seq,
                x=px,
                y=py,
                z=pz,
                raw_x=rx,
                raw_y=ry,
                raw_z=rz,
                speed_mps=speed,
                rms_mm=rms_mm,
                max_res_mm=max_res_mm,
                step_jump_mm=jump_mm,
                pdop=pdop,
                hdop=hdop,
                vdop=vdop,
                used_anchor_ids=used_ids,
                excluded_anchor_ids=excl_ids,
                motion_state=motion_str,
                stability_state=stab_str,
                instability_reason=reason_str,
                waypoint_label=wp_lbl,
                anchors=snap_map,
            )
            frames.append(frame)

            if not map_path:
                map_path.append((px, py, pz, rel_s, stab_str, rms_mm))
            else:
                lx, ly, lz, lt, lstab, _ = map_path[-1]
                if (
                    math.dist((px, py, pz), (lx, ly, lz)) >= 0.010
                    or (rel_s - lt) >= 0.50
                    or (stab_str != lstab and (rel_s - lt) >= 0.15)
                ):
                    map_path.append((px, py, pz, rel_s, stab_str, rms_mm))

    # Build contiguous InstabilitySegments from frames where stability_state != "STABLE"
    total_f = max(1, len(frames))
    duration_s = frames[-1].rel_time_s if frames else 0.0
    avg_dt = duration_s / total_f if total_f > 1 else 0.02
    merge_gap_s = max(1.2, 4.0 * avg_dt)

    segments: list[InstabilitySegment] = []
    cur_frames: list[ReplayFrame] = []
    for fr in frames:
        if fr.stability_state in ("WARN", "UNSTABLE"):
            if cur_frames and (fr.rel_time_s - cur_frames[-1].rel_time_s) > merge_gap_s:
                segments.append(_summarize_instability_segment(len(segments) + 1, cur_frames))
                cur_frames = [fr]
            else:
                cur_frames.append(fr)
        else:
            if cur_frames and (fr.rel_time_s - cur_frames[-1].rel_time_s) > merge_gap_s:
                segments.append(_summarize_instability_segment(len(segments) + 1, cur_frames))
                cur_frames = []
    if cur_frames:
        segments.append(_summarize_instability_segment(len(segments) + 1, cur_frames))
    return LoadedLogSession(
        csv_path=csv_path,
        anchors=anchors_sorted,
        frames=frames,
        map_path=map_path,
        waypoints=waypoints,
        unstable_segments=segments,
        duration_s=duration_s,
        path_length_2d_m=path_2d,
        path_length_3d_m=path_3d,
        stable_pct=100.0 * n_stable / total_f,
        warn_pct=100.0 * n_warn / total_f,
        unstable_pct=100.0 * n_unstable / total_f,
        total_csv_rows=total_rows,
    )


def _summarize_instability_segment(seg_id: int, seg_frames: Sequence[ReplayFrame]) -> InstabilitySegment:
    peak = max(seg_frames, key=lambda f: f.rms_mm + f.step_jump_mm * 1.2)
    sev = "UNSTABLE" if any(f.stability_state == "UNSTABLE" for f in seg_frames) else "WARN"
    t0 = seg_frames[0].rel_time_s
    t1 = seg_frames[-1].rel_time_s
    max_rms = max(f.rms_mm for f in seg_frames)
    max_jmp = max(f.step_jump_mm for f in seg_frames)
    min_anc = min((len(f.used_anchor_ids) for f in seg_frames), default=0)
    used_str = ",".join(f"A{a}" for a in peak.used_anchor_ids) if peak.used_anchor_ids else "none"
    cause = peak.instability_reason or f"RMS {max_rms:.0f}mm / Jump {max_jmp:.0f}mm"
    return InstabilitySegment(
        segment_id=seg_id,
        severity=sev,
        start_frame_idx=seg_frames[0].frame_index,
        peak_frame_idx=peak.frame_index,
        end_frame_idx=seg_frames[-1].frame_index,
        start_rel_s=t0,
        end_rel_s=t1,
        duration_s=max(0.005, t1 - t0),
        xyz_m=(peak.x, peak.y, peak.z),
        max_rms_mm=max_rms,
        max_jump_mm=max_jmp,
        min_anchors=min_anc,
        used_anchors_str=used_str,
        cause_summary=cause,
    )

