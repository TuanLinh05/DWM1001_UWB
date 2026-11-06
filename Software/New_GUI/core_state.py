"""High-speed UWB state engine, 2D/3D robust position solver, DOP calculator, and calibration engine.

Designed for DWM1001 Tag telemetry (Burst DS-TWR 0x18 at 50-300 Hz, RANGE_MEAS 0x10,
RANGE snapshot 0x01, RX_QUALITY 0x19, INFO 0x00, DEVICE_INFO 0x16, DIAG_SYSTEM 0x15).
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
import re
import statistics
import struct
import sys
from typing import Iterable, Mapping, Sequence

# Reuse the verified protocol parser & host CV alpha-beta filter without modifying old GUI
_OLD_GUI_DIR = Path(__file__).resolve().parent.parent / "UWB_UART_GUI"
if str(_OLD_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(_OLD_GUI_DIR))

import telemetry_protocol as tp  # noqa: E402
from host_range_filter import HostRangeFilter  # noqa: E402


MAX_ANCHORS = 8
FRESH_AGE_S = 2.0
STATIONARY_HOLD_AGE_S = 9.0
RING_HISTORY_LEN = 9000  # Holds >= 30s of high-rate history per anchor even at 300 Hz
TRAIL_MAX_LEN = 9000     # Holds >= 30s of solved XYZ history for the 30s Rolling Chart
MAP_TRAIL_LEN = 220      # Recent comet-tail length drawn on the 2D/3D spatial canvas
NLOS_WARN_DB = 6.0
NLOS_ALERT_DB = 10.0


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
class CalibrationSpotCapture:
    spot_index: int
    spot_xyz: tuple[float, float, float] | None
    single_anchor_id: int | None
    reference_distance_m: float | None
    timestamp_s: float
    anchor_results: dict[int, CalibrationAnchorResult]


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

        self.reset_live_state()
        self.spot_captures: list[CalibrationSpotCapture] = []

    def reset_live_state(self) -> None:
        self.live: dict[int, AnchorLiveState] = {
            aid: AnchorLiveState(aid) for aid in range(1, MAX_ANCHORS + 1)
        }
        self.host_filters: dict[int, HostRangeFilter] = {}
        # Ring buffers per anchor: (tag_time_s, raw_mm, corrected_mm, host_mm, nlos_db)
        self.anchor_history: dict[int, deque[tuple[float, float, float, float, float | None]]] = {
            aid: deque(maxlen=RING_HISTORY_LEN) for aid in range(1, MAX_ANCHORS + 1)
        }
        # 2-second sliding window per anchor for rate_hz and 48-sample window for stable difference noise
        self._rate_window: dict[int, deque[tuple[float, int]]] = {
            aid: deque() for aid in range(1, MAX_ANCHORS + 1)
        }
        self._noise_window: dict[int, deque[int]] = {
            aid: deque(maxlen=48) for aid in range(1, MAX_ANCHORS + 1)
        }
        self._recent_corr_mm: dict[int, deque[float]] = {
            aid: deque(maxlen=9) for aid in range(1, MAX_ANCHORS + 1)
        }
        self._smooth_range_mm: dict[int, float] = {}
        self._lock_range_mm: dict[int, float] = {}
        self._last_range_t: dict[int, float] = {}
        self._first_rx_s: float | None = None
        self._smooth_xyz: tuple[float, float, float] | None = None
        self._best_anchor_count: int = 0
        self._last_solve_s: float | None = None
        self._cycle_times: deque[tuple[float, int]] = deque(maxlen=250)

        self.latest_solution: PositionResult | None = None
        self.position_trail: deque[tuple[float, float, float, float]] = deque(maxlen=TRAIL_MAX_LEN)

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
        self._smooth_range_mm.clear()
        self._lock_range_mm.clear()
        self._last_range_t.clear()
        self._smooth_xyz = None
        self._best_anchor_count = 0
        self._last_solve_s = None

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
        self.calibration_active = True

    def cancel_calibration(self) -> None:
        self.calibration_active = False
        self.cal_collected = 0
        self._cal_samples.clear()

    def finish_calibration(self, now_s: float = 0.0) -> CalibrationSpotCapture | None:
        if not self.calibration_active:
            return None
        self.calibration_active = False
        results: dict[int, CalibrationAnchorResult] = {}

        for aid, samples in self._cal_samples.items():
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
            quality = (
                "GOOD"
                if sigma <= 45.0 and abs(drift) <= 35.0 and len(samples) >= min(20, self.cal_target_samples)
                else "WARN"
            )
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

        if not results:
            return None

        capture = CalibrationSpotCapture(
            spot_index=len(self.spot_captures) + 1,
            spot_xyz=self._cal_spot_xyz,
            single_anchor_id=self._cal_single_aid,
            reference_distance_m=(self._cal_single_ref_mm / 1000.0) if self._cal_single_aid else None,
            timestamp_s=now_s,
            anchor_results=results,
        )
        self.spot_captures.append(capture)
        return capture

    def aggregated_calibration(self) -> dict[int, tuple[float, float, int]]:
        """Return {anchor_id: (median_bias_mm, mean_sigma_mm, spot_count)} across all captures."""
        out: dict[int, tuple[float, float, int]] = {}
        for aid in range(1, MAX_ANCHORS + 1):
            items = [
                c.anchor_results[aid]
                for c in self.spot_captures
                if aid in c.anchor_results
            ]
            if not items:
                continue
            med_bias = _percentile([r.bias_mm for r in items], 0.50)
            mean_sigma = max(statistics.fmean(r.sigma_mm for r in items), 10.0)
            out[aid] = (med_bias, mean_sigma, len(items))
        return out

    def apply_calibration_to_host(self) -> dict[int, float]:
        """Copy aggregated calibration biases & sigmas into active host AnchorConfigs."""
        agg = self.aggregated_calibration()
        applied: dict[int, float] = {}
        for aid, (bias_mm, sigma_mm, _count) in agg.items():
            self.anchors[aid].bias_mm = bias_mm
            self.anchors[aid].sigma_mm = sigma_mm
            applied[aid] = bias_mm
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
        elif isinstance(message, tp.RxQualityMessage):
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

        nwin = self._noise_window[aid]
        if not nwin or abs(raw_mm - nwin[-1]) < 500:
            nwin.append(raw_mm)
        if len(nwin) >= 6 and (state.rx_count % 4 == 0 or state.noise_sigma_mm is None):
            diffs = [b - a for a, b in zip(nwin, list(nwin)[1:])]
            if len(diffs) >= 5:
                inst_sig = statistics.pstdev(diffs) / math.sqrt(2.0)
                if state.noise_sigma_mm is None:
                    state.noise_sigma_mm = inst_sig
                else:
                    state.noise_sigma_mm = 0.85 * state.noise_sigma_mm + 0.15 * inst_sig

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
        win = self._recent_corr_mm[aid]
        if len(win) >= 5:
            med_prev = statistics.median(win)
            win.append(corrected_mm)
            if abs(corrected_mm - med_prev) > 160.0:
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

        dt_a = min(max(received_s - self._last_range_t.get(aid, received_s - 0.02), 0.002), 0.5)
        self._last_range_t[aid] = received_s
        uptime = received_s - self._first_rx_s
        sig = max(self.live[aid].noise_sigma_mm or self.anchors[aid].sigma_mm, 18.0)

        if aid not in self._smooth_range_mm:
            self._smooth_range_mm[aid] = gated_mm
            self._lock_range_mm[aid] = gated_mm
        else:
            dev = abs(gated_mm - self._lock_range_mm[aid])
            thresh = max(2.8 * sig, 55.0)
            if uptime < 1.8:
                tau_r = 0.22
                self._lock_range_mm[aid] += (dt_a / (0.30 + dt_a)) * (gated_mm - self._lock_range_mm[aid])
            elif dev < thresh:
                tau_r = 1.45
                self._lock_range_mm[aid] += (dt_a / (10.0 + dt_a)) * (gated_mm - self._lock_range_mm[aid])
            else:
                tau_r = 0.02
                self._lock_range_mm[aid] = gated_mm
            alpha_r = dt_a / (tau_r + dt_a)
            target_mm = gated_mm if dev < thresh else cv_mm
            self._smooth_range_mm[aid] += alpha_r * (target_mm - self._smooth_range_mm[aid])

        return self._smooth_range_mm[aid]

    def _ingest_burst(self, msg: tp.RangeBurstMessage, received_s: float) -> None:
        if self.boot_id is not None and msg.boot_id != self.boot_id:
            self.host_filters.clear()
            for q in self._recent_corr_mm.values():
                q.clear()
            self._smooth_range_mm.clear()
            self._lock_range_mm.clear()
            self._last_range_t.clear()
            self._smooth_xyz = None
            self._best_anchor_count = 0
            self._last_solve_s = None
            self.last_cycle_seq = None
        self.boot_id = msg.boot_id
        if self.last_cycle_seq is not None:
            delta = (msg.cycle_seq - self.last_cycle_seq) & 0xFFFFFFFF
            if 1 < delta < 100000:
                self.lost_cycles += delta - 1
        self.last_cycle_seq = msg.cycle_seq
        self.total_bursts += 1
        self.period_us = msg.period_us
        self.ranging_mode_name = "Burst DS-TWR"

        self._cycle_times.append((received_s, msg.cycle_seq))
        while len(self._cycle_times) > 2 and (received_s - self._cycle_times[0][0]) > 2.0:
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

            self._update_anchor_rate_and_noise(aid, received_s, raw_mm)
            fpp_cdbm = entry.fp_cdbm if entry.fp_cdbm != tp.INT16_MIN else -8200
            host_mm = self._update_filtered_range(aid, corrected_mm, fpp_cdbm, tag_time_ms, received_s)
            state.host_filtered_mm = host_mm

            fp_dbm = None if entry.fp_cdbm == tp.INT16_MIN else entry.fp_cdbm / 100.0
            rx_dbm = None if entry.rx_cdbm == tp.INT16_MIN else entry.rx_cdbm / 100.0
            nlos_db = (rx_dbm - fp_dbm) if (fp_dbm is not None and rx_dbm is not None) else None
            ci_ppm = None if entry.ci_ppm_x100 == tp.INT16_MIN else entry.ci_ppm_x100 / 100.0
            state.fp_dbm = fp_dbm
            state.rx_dbm = rx_dbm
            state.nlos_db = nlos_db
            state.ci_ppm = ci_ppm

            self.anchor_history[aid].append((tag_time_s, float(raw_mm), corrected_mm, host_mm, nlos_db))

            if self.calibration_active and aid in self._cal_samples:
                self._cal_samples[aid].append((tag_time_s, raw_mm, nlos_db, fp_dbm))

        self._check_calibration_progress(received_s)
        if updated_any:
            self._solve_current_position(received_s, tag_time_s)
        elif self.latest_solution is not None:
            self.position_trail.append(
                (self.latest_solution.x, self.latest_solution.y, self.latest_solution.z, tag_time_s)
            )

    def _ingest_range_meas(self, msg: tp.RangeMeasMessage, received_s: float) -> None:
        aid = msg.anchor_id
        if not (1 <= aid <= MAX_ANCHORS):
            return
        cfg = self.anchors[aid]
        state = self.live[aid]
        tag_time_s = msg.meas_time_us / 1e6
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
        host_mm = float(
            self.host_filters.setdefault(aid, HostRangeFilter()).update(
                round(corrected_mm),
                tp.fpp_to_legacy_scale_cdbm(fpp_cdbm, self.info_flags),
                msg.time_ms,
            )
        )
        state.host_filtered_mm = host_mm
        state.fp_dbm = None if msg.fp_cdbm == tp.INT16_MIN else msg.fp_cdbm / 100.0
        state.rx_dbm = None if msg.rx_cdbm == tp.INT16_MIN else msg.rx_cdbm / 100.0
        state.nlos_db = msg.nlos_indicator_db
        state.ci_ppm = None if msg.ci_ppm_x100 == tp.INT16_MIN else msg.ci_ppm_x100 / 100.0

        self._update_anchor_rate_and_noise(aid, received_s, msg.raw_mm)
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
            host_mm = float(
                self.host_filters.setdefault(aid, HostRangeFilter()).update(
                    round(corrected_mm),
                    tp.fpp_to_legacy_scale_cdbm(sample.fpp_cdbm, self.info_flags),
                    msg.time_ms,
                )
            )
            state.host_filtered_mm = host_mm
            state.fp_dbm = sample.fpp_cdbm / 100.0
            self._update_anchor_rate_and_noise(aid, received_s, sample.raw_mm)
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
        if self._cal_mode == "single" and self._cal_single_aid is not None:
            self.cal_collected = len(self._cal_samples.get(self._cal_single_aid, ()))
        else:
            counts = [
                len(v)
                for aid, v in self._cal_samples.items()
                if self.anchors[aid].enabled and len(v) > 0
            ]
            self.cal_collected = min(counts) if counts else 0
        if self.cal_collected >= self.cal_target_samples:
            self.finish_calibration(now_s)

    def _solve_current_position(self, now_s: float, tag_time_s: float) -> None:
        fast_moving = 0
        if self.use_host_filter_for_pos:
            for aid, state in self.live.items():
                if not self.anchors[aid].enabled or state.last_update_s < 0.0:
                    continue
                if (now_s - state.last_update_s) <= 0.6 and aid in self._smooth_range_mm and aid in self._lock_range_mm:
                    sig = max(state.noise_sigma_mm or self.anchors[aid].sigma_mm, 18.0)
                    if abs(self._smooth_range_mm[aid] - self._lock_range_mm[aid]) > max(2.5 * sig, 50.0):
                        fast_moving += 1

        max_age = (
            (1.2 if fast_moving >= 2 else STATIONARY_HOLD_AGE_S)
            if self.use_host_filter_for_pos
            else FRESH_AGE_S
        )

        ranges_mm: dict[int, float] = {}
        nlos_map: dict[int, float | None] = {}
        for aid, state in self.live.items():
            cfg = self.anchors[aid]
            age = now_s - state.last_update_s
            if not cfg.enabled or state.last_update_s < 0.0 or age > max_age:
                continue
            if not self.allow_uncalibrated_positioning and not (state.valid or state.fw_calibrated):
                continue
            val = state.host_filtered_mm if self.use_host_filter_for_pos else state.corrected_mm
            if val is not None and val > 10.0:
                ranges_mm[aid] = val
                nlos_map[aid] = state.nlos_db

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

        if sol is not None and self.use_host_filter_for_pos:
            raw_xyz = (sol.x, sol.y, sol.z)
            dt_s = min(
                max(now_s - (self._last_solve_s if self._last_solve_s is not None else now_s - 0.005), 0.001),
                0.25,
            )
            self._last_solve_s = now_s
            uptime = now_s - (self._first_rx_s if self._first_rx_s is not None else now_s)
            n_used = len(sol.used_anchor_ids)

            if self._smooth_xyz is None or (n_used > self._best_anchor_count and uptime < 1.0):
                self._smooth_xyz = raw_xyz
                self._best_anchor_count = max(self._best_anchor_count, n_used)
            else:
                self._best_anchor_count = max(self._best_anchor_count, n_used)
                if uptime < 2.0:
                    ax = dt_s / (0.25 + dt_s)
                    az = dt_s / (0.35 + dt_s)
                elif fast_moving == 0:
                    if n_used < self._best_anchor_count:
                        ax = 0.0
                        az = 0.0
                    else:
                        ax = dt_s / (4.0 + dt_s)
                        az = dt_s / (6.0 + dt_s)
                elif fast_moving == 1:
                    ax = dt_s / (1.2 + dt_s)
                    az = dt_s / (2.0 + dt_s)
                else:
                    ax = dt_s / (0.025 + dt_s)
                    az = dt_s / (0.04 + dt_s)
                sx = self._smooth_xyz[0] + ax * (raw_xyz[0] - self._smooth_xyz[0])
                sy = self._smooth_xyz[1] + ax * (raw_xyz[1] - self._smooth_xyz[1])
                sz = (
                    (self._smooth_xyz[2] + az * (raw_xyz[2] - self._smooth_xyz[2]))
                    if self.solver_dimension == 3
                    else self.fixed_z_m
                )
                self._smooth_xyz = (sx, sy, sz)

            if self._smooth_xyz != raw_xyz:
                sol = PositionResult(
                    x=self._smooth_xyz[0],
                    y=self._smooth_xyz[1],
                    z=self._smooth_xyz[2],
                    rms_m=sol.rms_m,
                    max_residual_m=sol.max_residual_m,
                    residuals_m=sol.residuals_m,
                    used_anchor_ids=sol.used_anchor_ids,
                    excluded_anchor_ids=sol.excluded_anchor_ids,
                    dimension=sol.dimension,
                    dop=sol.dop,
                    tag_time_s=sol.tag_time_s,
                )

        self.latest_solution = sol
        for aid, state in self.live.items():
            if sol is not None and aid in sol.residuals_m:
                state.residual_mm = sol.residuals_m[aid] * 1000.0
            else:
                state.residual_mm = None

        if sol is not None:
            self.position_trail.append((sol.x, sol.y, sol.z, tag_time_s))
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
