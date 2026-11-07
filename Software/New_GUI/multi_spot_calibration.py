"""Joint multi-spot UWB range-bias calibration (G0.1) — SKELETON.

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

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Protocol, Sequence

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
        raise NotImplementedError("G0.1: JointCalibrationResult.to_dict")


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
    raise NotImplementedError("G0.1: observations_from_captures")


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
    raise NotImplementedError("G0.1: data_identifiability")


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
    raise NotImplementedError("G0.1: suggest_calibration_spots")


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
    raise NotImplementedError("G0.1: solve_joint_calibration")
