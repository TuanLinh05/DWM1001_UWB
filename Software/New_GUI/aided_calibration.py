"""Sensor-aided UWB range calibration (E2.0-A).

Plan: Plan/E2_0_YEU_CAU_HIEU_CHUAN_CO_CAM_BIEN_VA_REPLAY_ESKF_A.md (section 3).
Contract: tests/test_e20_aided_calibration.py. Do not loosen the tests.

Why
---
Test_1 (10/11/2026) showed a stable +12…+18 cm UWB height bias (calibration
captured at one height only) and range changes of ±15 cm per anchor after a
~98° yaw. IMU fusion smooths but cannot remove such systematic errors; another
absolute sensor can. Here the height of the TAG above the rig floor comes from
the downward rangefinder (tilt- and lever-compensated, z_ref) and the heading
from PX4 attitude. With z fixed by z_ref, the per-anchor range error becomes
observable:

    r_ij = || (x_j, y_j, z_ref_j) - a_i || + e_i(theta_ij) + noise
    e_i(theta) = b_i + sum_{k=1..K} ( c_ik cos(k theta) + s_ik sin(k theta) )

theta_ij is the bearing of anchor i seen from the TAG, in the BODY frame:
    theta = atan2(a_y - y, a_x - x) - yaw_rig        wrapped to [-pi, pi)
yaw_rig = heading of body +X in the rig ENU frame, counter-clockwise.
From PX4 (NED, clockwise): yaw_rig = rig_yaw_offset - yaw_ned. A constant
rig_yaw_offset only rotates the phase of the heading terms, but the SAME
convention must be used when fitting and when correcting.

K = 0 is a constant extra bias per anchor (on top of the firmware/host bias
already applied to the ranges). K >= 1 models antenna pattern / airframe
shadowing, and is only allowed when the data contain enough distinct headings.

Units: metres and radians. Ranges columns follow the order of `anchors`.
Pure numpy; no Qt; no import of core_state.

Weight convention: the supplied numerical reference uses the Huber factor
min(1, huber_m / abs(residual)) as a row multiplier, on BOTH the Jacobian and
residual (and on design/target in regression). Thus its normal-equation weight
is the square of that factor, stronger than standard square-root-weight Huber
M-estimation. This convention is kept explicit for reproducibility.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Sequence

import numpy as np

AIDED_VERSION = "aided-uwb-cal-v1"
N_SECTORS = 8                 # 45-degree sectors

WARNING_CODES = (
    "FEW_EPOCHS",             # fewer than settings.min_epochs usable epochs: ok=False, no corrections
    "HEADING_COVERAGE",       # harmonics requested but yaw covers < min_yaw_sectors sectors: harmonics_used=0
    "BIAS_LARGE",             # |extra bias| > settings.bias_warn_m for an anchor (anchor_id set)
)


@dataclass(frozen=True)
class AidedSettings:
    harmonics: int = 0              # 0, 1 or 2 (else ValueError)
    huber_m: float = 0.10           # Huber threshold on range residuals (m)
    iterations: int = 15            # outer iterations of the alternating solve
    min_epochs: int = 50
    min_yaw_sectors: int = 4        # distinct 45-degree yaw sectors needed for harmonics > 0
    bias_warn_m: float = 0.30


@dataclass(frozen=True)
class AidedWarning:
    code: str
    message: str
    anchor_id: int | None = None


@dataclass(frozen=True)
class AnchorCorrection:
    anchor_id: int
    bias_m: float
    cos_m: tuple[float, ...]        # c_i1..c_iK (empty when K = 0)
    sin_m: tuple[float, ...]        # s_i1..s_iK
    body_sectors_covered: int       # distinct 45-degree sectors of this anchor's body bearing in the fit data
    epochs_used: int                # epochs with a finite range for this anchor
    residual_rms_m: float | None    # RMS of final residuals with |r| <= 3 * huber_m; None if none


@dataclass(frozen=True)
class AidedCalibration:
    ok: bool
    harmonics_requested: int
    harmonics_used: int
    yaw_sectors: int
    corrections: dict[int, AnchorCorrection]
    residual_rms_m: float | None
    epochs: int                     # usable epochs
    warnings: tuple[AidedWarning, ...]
    version: str = AIDED_VERSION

    def warning_codes(self) -> tuple[str, ...]:
        return tuple(w.code for w in self.warnings)

    def correction_m(self, anchor_id: int, theta_body_rad):
        """e_i(theta) for one anchor; array-friendly; 0 for an anchor without a correction."""
        theta = np.asarray(theta_body_rad, dtype=float)
        correction = self.corrections.get(anchor_id)
        result = np.zeros_like(theta)
        if correction is not None:
            result += correction.bias_m
            for k, (c, s) in enumerate(zip(correction.cos_m, correction.sin_m), 1):
                result += c * np.cos(k * theta) + s * np.sin(k * theta)
        return float(result) if result.ndim == 0 else result

    def to_dict(self) -> dict[str, Any]:
        """Strict JSON (json.dumps(..., allow_nan=False) works): string keys, nan/inf -> None."""
        return _strict_json(asdict(self))


def _strict_json(value):
    if isinstance(value, dict):
        return {str(k): _strict_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_strict_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, np.integer):
        return int(value)
    return value


def _anchor_xyz(anchors):
    xyz = np.asarray([(a.x, a.y, a.z) for a in anchors], dtype=float).reshape((-1, 3))
    if not np.all(np.isfinite(xyz)):
        raise ValueError("Anchor coordinates must be finite")
    return xyz


def _sectors(angles):
    return len(np.unique(np.rint((np.asarray(angles) % (2 * math.pi)) / (math.pi / 4)).astype(int) % N_SECTORS))


def body_bearing(anchors: Sequence[Any], tag_xy, yaw_rig_rad) -> np.ndarray:
    """(N, n_anchors) body-frame bearing of each anchor seen from the TAG, wrapped to [-pi, pi)."""
    xyz = _anchor_xyz(anchors)
    xy = np.asarray(tag_xy, dtype=float).reshape((-1, 2))
    yaw = np.broadcast_to(np.asarray(yaw_rig_rad, dtype=float), (len(xy),))
    delta = xyz[None, :, :2] - xy[:, None, :]
    theta = np.arctan2(delta[:, :, 1], delta[:, :, 0]) - yaw[:, None]
    return (theta + math.pi) % (2 * math.pi) - math.pi


def correct_ranges(cal: AidedCalibration, anchors: Sequence[Any], ranges_m, tag_xy, yaw_rig_rad) -> np.ndarray:
    """ranges - e_i(theta) for every anchor/epoch; NaN stays NaN; input not modified."""
    result = np.array(ranges_m, dtype=float, copy=True)
    theta = body_bearing(anchors, tag_xy, yaw_rig_rad)
    if result.shape != theta.shape:
        raise ValueError("Ranges must have one row per epoch and one column per anchor")
    for col, anchor in enumerate(anchors):
        result[:, col] -= cal.correction_m(anchor.anchor_id, theta[:, col])
    return result


def _solve_positions(xyz, ranges, seeds, huber_m, dimension=3, fixed_z=None):
    """Batched per-epoch Huber Gauss-Newton; no temporal coupling or position prior."""
    if not np.isfinite(huber_m) or huber_m <= 0:
        raise ValueError("huber_m must be finite and positive")
    ranges = np.asarray(ranges, dtype=float)
    p = np.array(seeds, dtype=float, copy=True)
    valid = np.isfinite(ranges)
    usable = (valid.sum(axis=1) >= dimension + 1) & np.all(np.isfinite(p), axis=1)
    if fixed_z is not None:
        usable &= np.isfinite(fixed_z)
        p[:, 2] = fixed_z
    p[~usable] = np.nan
    indices = np.flatnonzero(usable)
    if not len(indices):
        return p
    positions = p[indices].copy()
    observations = ranges[indices]
    mask = valid[indices]
    for _ in range(12):
        delta = positions[:, None, :] - xyz[None, :, :]
        distances = np.linalg.norm(delta, axis=2)
        residual = np.where(mask, observations - distances, 0.0)
        jac = delta[:, :, :dimension] / np.maximum(distances[:, :, None], 1e-12)
        row_factor = mask * np.minimum(1.0, huber_m / np.maximum(np.abs(residual), 1e-15))
        weights = row_factor ** 2
        normal = np.einsum("nai,na,naj->nij", jac, weights, jac)
        rhs = np.einsum("nai,na,na->ni", jac, weights, residual)
        full_rank = np.linalg.eigvalsh(normal)[:, 0] > 1e-12
        step = np.zeros((len(indices), dimension))
        if full_rank.any():
            step[full_rank] = np.linalg.solve(normal[full_rank], rhs[full_rank, :, None])[:, :, 0]
        positions[~full_rank] = np.nan
        positions[:, :dimension] += step
        if np.max(np.abs(step), initial=0.0) < 1e-7:
            break
    p[indices] = positions
    return p


def solve_positions_3d(anchors: Sequence[Any], ranges_m, seed_xyz, huber_m: float = 0.10) -> np.ndarray:
    """Per-epoch robust 3D Gauss-Newton (Z FREE), Huber weights, <= 12 iterations from seed_xyz.
    Epochs with fewer than 4 finite ranges -> NaN row. Returns (N, 3)."""
    xyz = _anchor_xyz(anchors)
    ranges = np.asarray(ranges_m, dtype=float)
    seed = np.asarray(seed_xyz, dtype=float)
    if ranges.ndim != 2 or ranges.shape[1] != len(anchors) or seed.shape != (len(ranges), 3):
        raise ValueError("Expected ranges (N, anchors) and seed_xyz (N, 3)")
    return _solve_positions(xyz, ranges, seed, huber_m)


def fit_aided_calibration(anchors: Sequence[Any], ranges_m, z_ref_m, yaw_rig_rad, xy_seed_m,
                          settings: AidedSettings = AidedSettings()) -> tuple[AidedCalibration, np.ndarray]:
    """Fit per-anchor corrections with the TAG height fixed to z_ref. Returns (calibration, fitted_xy (N, 2)).

    1. settings.harmonics not in {0, 1, 2} -> ValueError.
    2. Usable epoch: finite z_ref, yaw and xy_seed, and >= 4 finite ranges.
       Fewer than min_epochs -> ok=False, FEW_EPOCHS, corrections={}.
    3. yaw_sectors = number of distinct round(wrap_0_2pi(yaw) / 45deg) mod 8 over usable epochs.
       If harmonics > 0 and yaw_sectors < min_yaw_sectors -> harmonics_used = 0 for every anchor
       and one HEADING_COVERAGE warning. (With one or two headings, heading terms would only
       absorb position-dependent errors such as wrong anchor coordinates.)
    4. Alternate for `iterations` rounds, starting from xy_seed and zero corrections:
       a) per epoch, solve (x, y) with z = z_ref by Huber-weighted Gauss-Newton on the corrected ranges;
       b) per anchor, regress the residual r - ||p - a|| on [1, cos(k theta), sin(k theta)]_{k<=K}
          with Huber IRLS (threshold huber_m), theta from the current (x, y) and yaw.
    5. Fill AnchorCorrection for every anchor in `anchors`; BIAS_LARGE when |b_i| > bias_warn_m.
    """
    if settings.harmonics not in (0, 1, 2):
        raise ValueError("harmonics must be 0, 1 or 2")
    if settings.iterations < 1 or settings.min_epochs < 1 or not np.isfinite(settings.huber_m) or settings.huber_m <= 0:
        raise ValueError("Invalid aided calibration settings")
    xyz = _anchor_xyz(anchors)
    ranges = np.asarray(ranges_m, dtype=float)
    z_ref = np.asarray(z_ref_m, dtype=float)
    yaw = np.asarray(yaw_rig_rad, dtype=float)
    xy = np.array(xy_seed_m, dtype=float, copy=True)
    if ranges.ndim != 2 or ranges.shape[1] != len(anchors) or xy.shape != (len(ranges), 2) \
            or z_ref.shape != (len(ranges),) or yaw.shape != (len(ranges),):
        raise ValueError("Inconsistent aided calibration input shapes")
    usable = np.isfinite(z_ref) & np.isfinite(yaw) & np.all(np.isfinite(xy), axis=1) & (np.isfinite(ranges).sum(axis=1) >= 4)
    n = int(usable.sum())
    sectors = _sectors(yaw[usable]) if n else 0
    xy[~usable] = np.nan
    warnings = []
    if n < settings.min_epochs:
        warning = AidedWarning("FEW_EPOCHS", f"Only {n} usable epochs; need at least {settings.min_epochs}.")
        return AidedCalibration(False, settings.harmonics, 0, sectors, {}, None, n, (warning,)), xy
    harmonics = settings.harmonics
    if harmonics and sectors < settings.min_yaw_sectors:
        harmonics = 0
        warnings.append(AidedWarning("HEADING_COVERAGE", f"Only {sectors} yaw sectors; using constant biases instead of heading terms."))
    r, z, heading = ranges[usable], z_ref[usable], yaw[usable]
    fit_xy = xy[usable].copy()
    coefficients = np.zeros((len(anchors), 1 + 2 * harmonics))
    for _ in range(settings.iterations):
        theta = body_bearing(anchors, fit_xy, heading)
        design = _design(theta, harmonics)
        error = np.einsum("nak,ak->na", design, coefficients)
        positions = _solve_positions(xyz, r - error, np.column_stack((fit_xy, z)), settings.huber_m, dimension=2, fixed_z=z)
        fit_xy = positions[:, :2]
        theta = body_bearing(anchors, fit_xy, heading)
        design = _design(theta, harmonics)
        residual = r - np.linalg.norm(positions[:, None, :] - xyz[None, :, :], axis=2)
        for i in range(len(anchors)):
            finite = np.isfinite(residual[:, i]) & np.all(np.isfinite(design[:, i]), axis=1)
            if finite.sum() < coefficients.shape[1]:
                continue
            X, y = design[finite, i], residual[finite, i]
            beta = coefficients[i].copy()
            for _ in range(12):
                weights = np.minimum(1.0, settings.huber_m / np.maximum(np.abs(y - X @ beta), 1e-15))
                candidate = np.linalg.lstsq(X * weights[:, None], y * weights, rcond=None)[0]
                change = np.max(np.abs(candidate - beta))
                beta = candidate
                if change < 1e-8:
                    break
            coefficients[i] = beta
    xy[usable] = fit_xy
    theta = body_bearing(anchors, fit_xy, heading)
    residual = r - np.linalg.norm(np.column_stack((fit_xy, z))[:, None, :] - xyz[None, :, :], axis=2) \
        - np.einsum("nak,ak->na", _design(theta, harmonics), coefficients)
    corrections = {}
    for i, anchor in enumerate(anchors):
        finite = np.isfinite(residual[:, i])
        values = residual[finite, i]
        good = values[np.abs(values) <= 3 * settings.huber_m]
        beta = coefficients[i]
        corrections[anchor.anchor_id] = AnchorCorrection(anchor.anchor_id, float(beta[0]),
            tuple(float(v) for v in beta[1::2]), tuple(float(v) for v in beta[2::2]),
            _sectors(theta[finite, i]) if finite.any() else 0, int(finite.sum()), _rms(good))
        if abs(beta[0]) > settings.bias_warn_m:
            warnings.append(AidedWarning("BIAS_LARGE", f"A{anchor.anchor_id}: extra bias {beta[0]:+.3f} m exceeds {settings.bias_warn_m:.3f} m.", anchor.anchor_id))
    good = residual[np.isfinite(residual) & (np.abs(residual) <= 3 * settings.huber_m)]
    return AidedCalibration(True, settings.harmonics, harmonics, sectors, corrections, _rms(good), n, tuple(warnings)), xy


def _design(theta, harmonics):
    columns = [np.ones_like(theta)]
    for k in range(1, harmonics + 1):
        columns.extend((np.cos(k * theta), np.sin(k * theta)))
    return np.stack(columns, axis=-1)


def _rms(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None


def holdout_report(anchors: Sequence[Any], cal: AidedCalibration, ranges_m, z_ref_m, yaw_rig_rad, xy_seed_m,
                   huber_m: float = 0.10) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    """Evaluate on epochs NOT used for fitting: solve 3D with Z free, without and with correction.

    Seeds: (xy_seed, z_ref). Bearings for the correction use xy_seed and yaw.
    Returns (report, positions_before (N, 3), positions_after (N, 3)) where report has
    epochs, dz_median_before_m, dz_p95_abs_before_m, solved_before,
    dz_median_after_m, dz_p95_abs_after_m, solved_after (dz = solved z - z_ref).
    """
    ranges = np.asarray(ranges_m, dtype=float)
    z = np.asarray(z_ref_m, dtype=float)
    seed = np.column_stack((xy_seed_m, z))
    before = solve_positions_3d(anchors, ranges, seed, huber_m)
    corrected = correct_ranges(cal, anchors, ranges, xy_seed_m, yaw_rig_rad)
    after = solve_positions_3d(anchors, corrected, seed, huber_m)
    report = {"epochs": len(ranges)}
    for name, positions in (("before", before), ("after", after)):
        dz = positions[:, 2] - z
        values = dz[np.isfinite(dz)]
        report[f"dz_median_{name}_m"] = float(np.median(values)) if len(values) else None
        report[f"dz_p95_abs_{name}_m"] = float(np.percentile(np.abs(values), 95)) if len(values) else None
        report[f"solved_{name}"] = int(np.all(np.isfinite(positions), axis=1).sum())
    return report, before, after
