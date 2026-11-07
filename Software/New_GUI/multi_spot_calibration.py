"""Joint multi-spot UWB range-bias calibration (G0.1).

Plan: Plan/G0_KE_HOACH_HIEU_CHUAN_DA_DIEM_VA_DU_LIEU_CHUAN.md (section G0.1).
Contract: tests/test_g0_multi_spot_solver.py. Do not loosen the tests.

Problem
-------
The TAG antenna stands at calibration spot j (true position p_j, typed into
the GUI as e_j). For every enabled anchor i at known position a_i the capture
gives the median raw range m_ij (mm). Firmware sign convention, the same as
CMD_SET_DS_CAL and AnchorConfig.bias_mm:

    m_ij = ||p_j - a_i|| + b_i + e_ij          (measured = true + bias)

The legacy method computes b_i = m_ij - ||e_j - a_i|| per spot and trusts e_j
completely. With one spot there are 8 equations and 8 + 3 unknowns, so a
wrongly typed spot is absorbed into the biases (07/11 log: a spot 0.65 m off
produced +/-0.5 m "biases"). With two or more spots the data-only Jacobian is
full rank, so this module estimates the biases b_i and the spot positions p_j
together; the typed coordinates e_j are only a (robust) prior.

Tape (1D) captures add direct rows m_k = d_k + b_i + e_k (d_k = tape distance).

Pure numpy, no Qt, no import of core_state (core_state imports this module).
Anchors are duck-typed: any object with anchor_id, x, y, z and optionally
enabled (AnchorConfig works). Disabled anchors are ignored.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import math
from typing import Any, Iterable, Mapping, Protocol, Sequence

import numpy as np

SOLVER_VERSION = "multi-spot-joint-v1"
# Standard error of a median of N Gaussian samples = 1.2533 * sigma / sqrt(N).
MEDIAN_EFFICIENCY = 1.2533

# Stable warning codes. Tests and the GUI rely on these exact strings.
WARNING_CODES = (
    "NO_DATA",            # no usable spot or tape capture
    "SINGLE_SPOT",        # fewer than settings.min_spots spot captures: legacy per-spot result returned
    "SPOT_FEW_ANCHORS",   # a spot has fewer than (position dims + 1) usable anchors: spot excluded
    "NOT_IDENTIFIABLE",   # data-only Jacobian rank < number of unknowns, or the solver failed
    "SPOT_OFFSET",        # estimated spot is far from the typed coordinates
    "RESIDUAL_OUTLIER",   # one anchor at one spot disagrees (NLOS at that spot?)
    "RESIDUAL_RMS",       # overall residual too large (wrong anchor coordinate? NLOS everywhere?)
    "BIAS_LARGE",         # |bias| above settings.bias_warn_mm
    "FLAT_HEIGHTS",       # 3D: all spots at about one height (common bias vs. height weakly separated)
    "ANCHOR_ONE_SPOT",    # an anchor was seen at only one spot and has no tape capture
)


class AnchorLike(Protocol):
    anchor_id: int
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class SpotObservation:
    """One spot capture reduced to per-anchor statistics (ranges in mm, raw = before host bias)."""
    spot_index: int
    entered_xyz: tuple[float, float, float]
    median_raw_mm: Mapping[int, float]
    sigma_mm: Mapping[int, float] = field(default_factory=dict)   # sample standard deviation
    samples: Mapping[int, int] = field(default_factory=dict)


@dataclass(frozen=True)
class TapeObservation:
    """One 1D tape capture: the TAG at a measured distance from a single anchor."""
    capture_index: int
    anchor_id: int
    reference_mm: float
    median_raw_mm: float
    sigma_mm: float = 0.0
    samples: int = 0


@dataclass(frozen=True)
class JointCalibrationSettings:
    spot_prior_sigma_m: float = 0.05        # trust in the typed spot coordinates (per axis)
    systematic_sigma_mm: float = 20.0       # spot-dependent multipath error that averaging does not remove
    tape_sigma_mm: float = 10.0             # tape / laser distance error
    bias_prior_sigma_mm: float = 2000.0     # very weak prior; keeps the normal matrix invertible
    huber_k: float = 2.0                    # robust threshold on normalized residuals
    max_iterations: int = 50
    step_tolerance_m: float = 1e-7          # stop when max |step| (m) is below this
    min_spots: int = 2                      # fewer spot captures -> "single_spot_fallback"
    spot_offset_warn_m: float = 0.10
    residual_warn_sigma: float = 3.0        # |normalized residual| above this -> outlier
    residual_rms_warn_mm: float = 50.0
    bias_warn_mm: float = 300.0
    min_height_span_m: float = 0.30


@dataclass(frozen=True)
class CalibrationWarning:
    code: str                    # one of WARNING_CODES
    message: str                 # English, shown in the GUI; mention "Spot #<index>" / "A<id>" when relevant
    spot_index: int | None = None
    anchor_id: int | None = None


@dataclass(frozen=True)
class JointAnchorBias:
    anchor_id: int
    bias_mm: float
    std_mm: float                          # 1-sigma, from the scaled covariance (see solve_joint_calibration)
    spots_used: int                        # spot captures that contain this anchor
    tape_used: int                         # tape captures of this anchor
    per_spot_median_bias_mm: float | None  # legacy value, median over spots/tapes, for comparison
    residual_rms_mm: float | None          # RMS of this anchor's non-outlier residuals


@dataclass(frozen=True)
class JointSpotResult:
    spot_index: int
    entered_xyz: tuple[float, float, float]
    estimated_xyz: tuple[float, float, float]   # 2D mode: z is exactly entered_xyz[2]
    offset_m: float                             # |estimated - entered|
    offset_std_m: float                         # sqrt(largest eigenvalue of the spot position covariance)
    anchors_used: tuple[int, ...]
    residuals_mm: dict[int, float]              # measured - model, per anchor
    residual_rms_mm: float                      # over the non-outlier anchors of this spot
    outlier_anchor_ids: tuple[int, ...]


@dataclass(frozen=True)
class JointCalibrationResult:
    ok: bool                  # biases usable (False only for "failed")
    method: str               # "joint" | "single_spot_fallback" | "failed"
    dimension: int            # 2 or 3
    biases: dict[int, JointAnchorBias]
    spots: tuple[JointSpotResult, ...]   # empty unless method == "joint"
    residual_rms_mm: float | None
    iterations: int
    identifiable: bool        # data-only Jacobian has full column rank
    rank: int
    unknowns: int
    condition_number: float   # math.inf when rank-deficient
    warnings: tuple[CalibrationWarning, ...]
    settings: JointCalibrationSettings
    version: str = SOLVER_VERSION

    def warning_codes(self) -> tuple[str, ...]:
        return tuple(w.code for w in self.warnings)

    def to_dict(self) -> dict[str, Any]:
        """JSON-ready dict; json.dumps(result.to_dict(), allow_nan=False) must work.

        Keys: version, ok, method, dimension, identifiable, rank, unknowns,
        condition_number, iterations, residual_rms_mm,
        biases {"<aid>": {bias_mm, std_mm, spots_used, tape_used,
                          per_spot_median_bias_mm, residual_rms_mm}},
        spots [{spot_index, entered_xyz, estimated_xyz, offset_m, offset_std_m,
                anchors_used, residuals_mm {"<aid>": mm}, residual_rms_mm, outlier_anchor_ids}],
        warnings [{code, message, spot_index, anchor_id}],
        settings {field: value}.
        Dict keys are strings; non-finite floats (nan, inf) become None.
        """
        return _json_value(asdict(self))


def _json_value(value: Any) -> Any:
    """Convert nested dataclasses' values to strict JSON primitives."""
    if isinstance(value, Mapping):
        return {str(k): _json_value(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def observations_from_captures(
    captures: Iterable[Any], enabled_anchor_ids: Iterable[int] | None = None
) -> tuple[list[SpotObservation], list[TapeObservation]]:
    """Convert core_state.CalibrationSpotCapture objects (duck-typed) to observations.

    - A capture with single_anchor_id set and reference_distance_m set is a
      tape capture -> one TapeObservation (reference_mm = reference_distance_m * 1000).
    - A capture with spot_xyz set (and no single_anchor_id) is a spot capture
      -> one SpotObservation with spot_index = capture.spot_index.
    - Per anchor use CalibrationAnchorResult.median_raw_mm / sigma_mm / samples.
    - Skip results with quality == "LOW_SAMPLES" and anchors not in
      enabled_anchor_ids (when given). Skip a spot capture with no usable anchor.
    - Keep the capture order.
    """
    enabled = None if enabled_anchor_ids is None else set(enabled_anchor_ids)
    spots, tapes = [], []
    for cap in captures:
        usable = {aid: r for aid, r in cap.anchor_results.items()
                  if r.quality != "LOW_SAMPLES" and (enabled is None or aid in enabled)}
        if cap.single_anchor_id is not None and cap.reference_distance_m is not None:
            aid = cap.single_anchor_id
            if aid in usable:
                r = usable[aid]
                tapes.append(TapeObservation(cap.spot_index, aid, cap.reference_distance_m * 1000.0,
                                             r.median_raw_mm, r.sigma_mm, r.samples))
        elif cap.single_anchor_id is None and cap.spot_xyz is not None and usable:
            spots.append(SpotObservation(cap.spot_index, tuple(cap.spot_xyz),
                                         {a: r.median_raw_mm for a, r in usable.items()},
                                         {a: r.sigma_mm for a, r in usable.items()},
                                         {a: r.samples for a, r in usable.items()}))
    return spots, tapes


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (ValueError, TypeError, OverflowError):
        return False


def _clean_inputs(anchors, spots, tapes):
    coords = {a.anchor_id: np.array((a.x, a.y, a.z), dtype=float) for a in anchors
              if getattr(a, "enabled", True) and all(_finite(v) for v in (a.x, a.y, a.z))}
    clean_spots = []
    for spot in spots:
        if len(spot.entered_xyz) != 3 or not all(_finite(v) for v in spot.entered_xyz):
            continue
        med = {a: float(v) for a, v in spot.median_raw_mm.items()
               if a in coords and _finite(v) and float(v) > 10.0}
        if med:
            clean_spots.append(replace(spot, entered_xyz=tuple(float(v) for v in spot.entered_xyz),
                                       median_raw_mm=med))
    clean_tapes = [t for t in tapes if t.anchor_id in coords and _finite(t.median_raw_mm)
                   and float(t.median_raw_mm) > 10.0 and _finite(t.reference_mm) and t.reference_mm > 0]
    return coords, clean_spots, clean_tapes


def _data_matrix(coords, spots, tapes, dimension):
    ids = sorted({a for s in spots for a in s.median_raw_mm} | {t.anchor_id for t in tapes})
    columns = {a: i for i, a in enumerate(ids)}
    unknowns = len(ids) + dimension * len(spots)
    rows = []
    for j, s in enumerate(spots):
        for a in s.median_raw_mm:
            row = np.zeros(unknowns)
            row[columns[a]] = 1.0
            delta = np.asarray(s.entered_xyz) - coords[a]
            row[len(ids) + j * dimension:len(ids) + (j + 1) * dimension] = (
                delta[:dimension] / max(float(np.linalg.norm(delta)), 1e-12))
            rows.append(row)
    for t in tapes:
        row = np.zeros(unknowns)
        row[columns[t.anchor_id]] = 1.0
        rows.append(row)
    return np.asarray(rows).reshape((-1, unknowns)) if unknowns else np.empty((0, 0))


def _matrix_rank(matrix):
    unknowns = matrix.shape[1]
    if not unknowns or not matrix.shape[0]:
        return 0, unknowns, math.inf
    norm = np.linalg.norm(matrix, axis=0)
    scaled = matrix / np.where(norm > 0, norm, 1.0)
    s = np.linalg.svd(scaled, compute_uv=False)
    rank = int(np.count_nonzero(s > 1e-9 * s[0])) if s[0] > 0 else 0
    return rank, unknowns, float(s[0] / s[-1]) if rank == unknowns else math.inf


def data_identifiability(
    anchors: Sequence[AnchorLike],
    spots: Sequence[SpotObservation],
    tapes: Sequence[TapeObservation] = (),
    dimension: int = 3,
) -> tuple[int, int, float]:
    """Return (rank, unknowns, condition_number) of the data-only Jacobian.

    Unknowns: one bias per anchor that appears in any observation, plus the
    estimated position components of every spot (x, y, z in 3D; x, y in 2D).
    Rows: one per (spot, anchor) range and one per tape capture; no prior
    rows, no weights. Evaluate at the entered spot coordinates. Scale every
    column to unit norm, take singular values s; rank = count(s > 1e-9 * s[0]);
    condition = s[0] / s[-1] when rank == unknowns, else math.inf.

    Expected for the 07/11 rig (8 anchors): 1 spot -> (8, 11, inf);
    2 spots -> (14, 14, finite); 4 spots -> (20, 20, ~10). In 2D: (8, 10), (12, 12).
    """
    if dimension not in (2, 3):
        raise ValueError("dimension must be 2 or 3")
    coords, spots, tapes = _clean_inputs(anchors, spots, tapes)
    return _matrix_rank(_data_matrix(coords, spots, tapes, dimension))


def suggest_calibration_spots(
    anchors: Sequence[AnchorLike],
    count: int = 4,
    xy_fractions: tuple[float, float] = (0.25, 0.75),
    z_fractions: tuple[float, float] = (0.3, 0.7),
) -> list[tuple[float, float, float]]:
    """Propose `count` (2..6, else ValueError) spots inside the enabled anchors' bounding box.

    With lo/hi the bounding box, fx(f) = lo.x + f*(hi.x - lo.x) (same for y),
    a, b = xy_fractions, zl/zh = lo.z + z_fractions * (hi.z - lo.z) when the
    anchors span >= 0.3 m in z, else zl = lo.z + 0.8, zh = lo.z + 1.4.
    Order (take the first `count`):
        (fx(a), fy(a), zl), (fx(b), fy(b), zh), (fx(b), fy(a), zl),
        (fx(a), fy(b), zh), (fx(.5), fy(.5), zl), (fx(.5), fy(.5), zh)
    The first two already have different heights and make the problem
    identifiable. Round to 3 decimals. ValueError when no anchor is enabled.
    """
    if count not in range(2, 7):
        raise ValueError("count must be between 2 and 6")
    xyz = [(a.x, a.y, a.z) for a in anchors if getattr(a, "enabled", True)
           and all(_finite(v) for v in (a.x, a.y, a.z))]
    if not xyz:
        raise ValueError("No enabled anchor with finite coordinates")
    lo, hi = np.min(xyz, axis=0), np.max(xyz, axis=0)
    a, b = xy_fractions
    xl, yl = lo[:2] + a * (hi[:2] - lo[:2])
    xh, yh = lo[:2] + b * (hi[:2] - lo[:2])
    zl, zh = (lo[2] + np.asarray(z_fractions) * (hi[2] - lo[2])
              if hi[2] - lo[2] >= 0.3 else (lo[2] + 0.8, lo[2] + 1.4))
    xc, yc = (lo[:2] + hi[:2]) / 2.0
    points = [(xl, yl, zl), (xh, yh, zh), (xh, yl, zl), (xl, yh, zh), (xc, yc, zl), (xc, yc, zh)]
    return [tuple(round(float(v), 3) for v in p) for p in points[:count]]


def solve_joint_calibration(
    anchors: Sequence[AnchorLike],
    spots: Sequence[SpotObservation],
    tapes: Sequence[TapeObservation] = (),
    dimension: int = 3,
    settings: JointCalibrationSettings = JointCalibrationSettings(),
) -> JointCalibrationResult:
    """Estimate per-anchor biases and spot positions together. Never raises for bad data.

    ValueError only when dimension is not 2 or 3.

    1. Clean the input. Ignore anchors that are disabled/unknown and ranges
       that are non-finite or <= 10 mm. A spot with fewer than (n_pos + 1)
       usable anchors (n_pos = 3 in 3D, 2 in 2D) is excluded with a
       SPOT_FEW_ANCHORS warning (spot_index set).
    2. No spot and no tape left -> ok=False, method="failed", NO_DATA.
       Fewer than settings.min_spots spots -> method="single_spot_fallback",
       ok=True, identifiable=False, SINGLE_SPOT warning (only when >= 1 spot):
       bias = median over the anchor's spots of (m_ij - ||e_j - a_i||) and
       over its tapes of (m_k - d_k), i.e. exactly the legacy numbers;
       rank/unknowns/condition from data_identifiability; spots=().
    3. Otherwise solve, internally in metres:
       unknowns  theta = [b_i for every anchor seen] + [p_j position components]
                 (2D: z of every spot stays at the typed value and is not a parameter)
       sigma_ij  = sqrt((1.2533 * s_ij / sqrt(N_ij))**2 + systematic_sigma**2)
                   (only the systematic term when s or N is missing/zero);
                   tape rows use tape_sigma_mm instead of systematic_sigma_mm
       data rows r = (m - model) / sigma with model = ||p_j - a_i|| + b_i (spot)
                 or d_k + b_i (tape)
       prior rows (e_j,c - p_j,c) / spot_prior_sigma_m for each spot component,
                 and (0 - b_i) / bias_prior_sigma for each bias
       weights   Huber w = 1 if |r| <= k else k / |r| (k = huber_k), recomputed
                 every iteration, applied to data rows AND spot-prior rows
                 (a grossly wrong typed spot must not pull the solution),
                 not to the bias prior rows
       start     p_j = e_j; b_i = legacy median bias
       iterate   Gauss-Newton on the weighted normal equations until
                 max |step| < step_tolerance_m or max_iterations
    4. Outputs at the solution:
       covariance C = inv(J^T W J + priors) * max(1, sum(w r^2 over data rows) / max(1, n_data - unknowns))
       bias std_mm = sqrt(C_ii) * 1000; spot offset_std_m = sqrt(max eig of its block)
       outliers: data rows with |r| > residual_warn_sigma (RESIDUAL_OUTLIER per row,
       spot_index and anchor_id set); residual RMS values exclude outliers
       identifiable / rank / condition: data-only Jacobian at the solution
       (same recipe as data_identifiability)
    5. Warnings (one per item, with spot_index/anchor_id where relevant):
       SPOT_OFFSET when offset_m > max(spot_offset_warn_m, 3 * offset_std_m);
       RESIDUAL_RMS when the overall RMS > residual_rms_warn_mm;
       BIAS_LARGE when |bias| > bias_warn_mm; ANCHOR_ONE_SPOT;
       NOT_IDENTIFIABLE when rank < unknowns;
       FLAT_HEIGHTS (3D) when the estimated spot heights span < min_height_span_m.
    6. If the normal matrix is singular or the iteration produces non-finite
       numbers, return the legacy numbers with method="failed", ok=True when any
       bias exists, plus NOT_IDENTIFIABLE.
    """
    if dimension not in (2, 3):
        raise ValueError("dimension must be 2 or 3")
    coords, spots, tapes = _clean_inputs(anchors, spots, tapes)
    warnings = []
    kept = []
    for s in spots:
        if len(s.median_raw_mm) < dimension + 1:
            warnings.append(CalibrationWarning("SPOT_FEW_ANCHORS",
                            f"Spot #{s.spot_index} has fewer than {dimension + 1} usable anchors; excluded.",
                            spot_index=s.spot_index))
        else:
            kept.append(s)
    spots = kept
    ids = sorted({a for s in spots for a in s.median_raw_mm} | {t.anchor_id for t in tapes})
    legacy = {a: [] for a in ids}
    for s in spots:
        for a, measured in s.median_raw_mm.items():
            legacy[a].append(measured - float(np.linalg.norm(np.asarray(s.entered_xyz) - coords[a])) * 1000)
    for t in tapes:
        legacy[t.anchor_id].append(t.median_raw_mm - t.reference_mm)
    rank, unknowns, condition = _matrix_rank(_data_matrix(coords, spots, tapes, dimension))

    def fallback(method):
        biases = {a: JointAnchorBias(a, float(np.median(legacy[a])), 30.0,
                  sum(a in s.median_raw_mm for s in spots), sum(t.anchor_id == a for t in tapes),
                  float(np.median(legacy[a])), None) for a in ids}
        return JointCalibrationResult(bool(biases), method, dimension, biases, (), None, 0,
                                      False, rank, unknowns, condition, tuple(warnings), settings)

    if not ids:
        warnings.append(CalibrationWarning("NO_DATA", "No usable spot or tape observations."))
        return fallback("failed")
    if len(spots) < settings.min_spots:
        if spots:
            warnings.append(CalibrationWarning("SINGLE_SPOT", "Joint solve needs at least 2 spot captures; using per-spot median."))
        return fallback("single_spot_fallback")

    bias_columns = {a: i for i, a in enumerate(ids)}
    n_bias = len(ids)
    entered = np.asarray([s.entered_xyz for s in spots], dtype=float)
    theta = np.concatenate(([np.median(legacy[a]) / 1000 for a in ids], entered[:, :dimension].ravel()))
    records = [(j, a, m / 1000, _row_sigma(s.sigma_mm.get(a), s.samples.get(a), settings.systematic_sigma_mm))
               for j, s in enumerate(spots) for a, m in s.median_raw_mm.items()]
    n_spot_rows = len(records)
    n_data = n_spot_rows + len(tapes)

    def evaluate(values):
        positions = entered.copy()
        positions[:, :dimension] = values[n_bias:].reshape((-1, dimension))
        residual = []
        jac = []
        for j, a, measured, sigma in records:
            delta = positions[j] - coords[a]
            distance = float(np.linalg.norm(delta))
            row = np.zeros(unknowns)
            row[bias_columns[a]] = 1.0 / sigma
            start = n_bias + j * dimension
            row[start:start + dimension] = delta[:dimension] / max(distance, 1e-12) / sigma
            residual.append((measured - distance - values[bias_columns[a]]) / sigma)
            jac.append(row)
        for t in tapes:
            sigma = _row_sigma(t.sigma_mm, t.samples, settings.tape_sigma_mm)
            row = np.zeros(unknowns)
            row[bias_columns[t.anchor_id]] = 1.0 / sigma
            residual.append((t.median_raw_mm / 1000 - t.reference_mm / 1000 - values[bias_columns[t.anchor_id]]) / sigma)
            jac.append(row)
        for j in range(len(spots)):
            for c in range(dimension):
                col = n_bias + j * dimension + c
                row = np.zeros(unknowns)
                row[col] = 1.0 / settings.spot_prior_sigma_m
                residual.append((entered[j, c] - values[col]) / settings.spot_prior_sigma_m)
                jac.append(row)
        for col in range(n_bias):
            row = np.zeros(unknowns)
            row[col] = 1000.0 / settings.bias_prior_sigma_mm
            residual.append(-values[col] * row[col])
            jac.append(row)
        r, J = np.asarray(residual), np.asarray(jac)
        w = np.ones(len(r))
        robust = np.abs(r[:-n_bias])
        w[:-n_bias] = np.minimum(1.0, settings.huber_k / np.maximum(robust, 1e-15))
        return positions, r, J, w

    try:
        positive = (settings.spot_prior_sigma_m, settings.systematic_sigma_mm, settings.tape_sigma_mm,
                    settings.bias_prior_sigma_mm, settings.huber_k, settings.step_tolerance_m)
        if not all(_finite(v) and v > 0 for v in positive) or settings.max_iterations < 1:
            raise ValueError("Invalid joint calibration settings")
        iterations = 0
        for iterations in range(1, settings.max_iterations + 1):
            _, r, J, w = evaluate(theta)
            normal = J.T @ (w[:, None] * J)
            step = np.linalg.solve(normal, J.T @ (w * r))
            if not np.all(np.isfinite(step)):
                raise ValueError("Non-finite joint calibration step")
            theta += step
            if float(np.max(np.abs(step))) < settings.step_tolerance_m:
                break
        positions, r, J, w = evaluate(theta)
        covariance = np.linalg.inv(J.T @ (w[:, None] * J))
        covariance *= max(1.0, float(np.sum(w[:n_data] * r[:n_data] ** 2)) / max(1, n_data - unknowns))
        if not np.all(np.isfinite(theta)) or not np.all(np.isfinite(covariance)):
            raise ValueError("Non-finite joint calibration solution")
        solved_spots = [replace(s, entered_xyz=tuple(p)) for s, p in zip(spots, positions)]
        rank, unknowns, condition = _matrix_rank(_data_matrix(coords, solved_spots, tapes, dimension))
    except (np.linalg.LinAlgError, ValueError, FloatingPointError, OverflowError):
        warnings.append(CalibrationWarning("NOT_IDENTIFIABLE", "Joint solve failed; review geometry and observations."))
        return fallback("failed")

    residuals = {(j, a): float(r[k] * sigma * 1000) for k, (j, a, _, sigma) in enumerate(records)}
    outlier = np.abs(r[:n_data]) > settings.residual_warn_sigma
    all_good = []
    anchor_good = {a: [] for a in ids}
    spot_results = []
    row_index = 0
    for j, (s, position) in enumerate(zip(spots, positions)):
        good, bad = [], []
        for a in s.median_raw_mm:
            value = residuals[j, a]
            if outlier[row_index]:
                bad.append(a)
                warnings.append(CalibrationWarning("RESIDUAL_OUTLIER", f"Spot #{s.spot_index}, A{a}: residual {value:+.0f} mm.", s.spot_index, a))
            else:
                good.append(value)
                anchor_good[a].append(value)
                all_good.append(value)
            row_index += 1
        start = n_bias + j * dimension
        block = covariance[start:start + dimension, start:start + dimension]
        std = math.sqrt(max(0.0, float(np.linalg.eigvalsh(block)[-1])))
        offset = float(np.linalg.norm(position - entered[j]))
        if offset > max(settings.spot_offset_warn_m, 3 * std):
            warnings.append(CalibrationWarning("SPOT_OFFSET", f"Spot #{s.spot_index} is {offset:.3f} m from the entered coordinates; check its surveyed position.", s.spot_index))
        spot_results.append(JointSpotResult(s.spot_index, s.entered_xyz, tuple(float(v) for v in position),
                            offset, std, tuple(sorted(s.median_raw_mm)),
                            {a: residuals[j, a] for a in s.median_raw_mm}, _rms(good) or 0.0, tuple(sorted(bad))))
    for k, t in enumerate(tapes, n_spot_rows):
        value = float(r[k] * _row_sigma(t.sigma_mm, t.samples, settings.tape_sigma_mm) * 1000)
        if outlier[k]:
            warnings.append(CalibrationWarning("RESIDUAL_OUTLIER", f"Tape #{t.capture_index}, A{t.anchor_id}: residual {value:+.0f} mm.", anchor_id=t.anchor_id))
        else:
            anchor_good[t.anchor_id].append(value)
            all_good.append(value)
    biases = {}
    for a, col in bias_columns.items():
        bias = float(theta[col] * 1000)
        ns, nt = sum(a in s.median_raw_mm for s in spots), sum(t.anchor_id == a for t in tapes)
        biases[a] = JointAnchorBias(a, bias, math.sqrt(max(0.0, float(covariance[col, col]))) * 1000,
                                    ns, nt, float(np.median(legacy[a])), _rms(anchor_good[a]))
        if abs(bias) > settings.bias_warn_mm:
            warnings.append(CalibrationWarning("BIAS_LARGE", f"A{a}: bias {bias:+.0f} mm exceeds {settings.bias_warn_mm:.0f} mm.", anchor_id=a))
        if ns == 1 and not nt:
            warnings.append(CalibrationWarning("ANCHOR_ONE_SPOT", f"A{a} appears at only one spot; collect another spot or tape capture.", anchor_id=a))
    rms = _rms(all_good)
    if rms is not None and rms > settings.residual_rms_warn_mm:
        warnings.append(CalibrationWarning("RESIDUAL_RMS", f"Joint residual RMS {rms:.0f} mm exceeds {settings.residual_rms_warn_mm:.0f} mm."))
    if rank < unknowns:
        warnings.append(CalibrationWarning("NOT_IDENTIFIABLE", f"Data-only Jacobian rank {rank}/{unknowns}; collect more diverse spots."))
    if dimension == 3 and np.ptp(positions[:, 2]) < settings.min_height_span_m:
        warnings.append(CalibrationWarning("FLAT_HEIGHTS", "Spot heights are too similar; collect spots at different heights."))
    return JointCalibrationResult(True, "joint", dimension, biases, tuple(spot_results), rms, iterations,
                                  rank == unknowns, rank, unknowns, condition, tuple(warnings), settings)


def _row_sigma(sigma, samples, floor_mm):
    median_error = MEDIAN_EFFICIENCY * float(sigma) / math.sqrt(float(samples)) if (
        _finite(sigma) and _finite(samples) and float(sigma) > 0 and float(samples) > 0) else 0.0
    return math.hypot(median_error, floor_mm) / 1000.0


def _rms(values):
    return float(np.sqrt(np.mean(np.square(values)))) if values else None
