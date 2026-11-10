"""Sensor-aided UWB range calibration (E2.0-A) — SKELETON.

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
"""

from __future__ import annotations

from dataclasses import dataclass
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
        raise NotImplementedError("E2.0-A: AidedCalibration.correction_m")

    def to_dict(self) -> dict[str, Any]:
        """Strict JSON (json.dumps(..., allow_nan=False) works): string keys, nan/inf -> None."""
        raise NotImplementedError("E2.0-A: AidedCalibration.to_dict")


def body_bearing(anchors: Sequence[Any], tag_xy, yaw_rig_rad) -> np.ndarray:
    """(N, n_anchors) body-frame bearing of each anchor seen from the TAG, wrapped to [-pi, pi)."""
    raise NotImplementedError("E2.0-A: body_bearing")


def correct_ranges(cal: AidedCalibration, anchors: Sequence[Any], ranges_m, tag_xy, yaw_rig_rad) -> np.ndarray:
    """ranges - e_i(theta) for every anchor/epoch; NaN stays NaN; input not modified."""
    raise NotImplementedError("E2.0-A: correct_ranges")


def solve_positions_3d(anchors: Sequence[Any], ranges_m, seed_xyz, huber_m: float = 0.10) -> np.ndarray:
    """Per-epoch robust 3D Gauss-Newton (Z FREE), Huber weights, <= 12 iterations from seed_xyz.
    Epochs with fewer than 4 finite ranges -> NaN row. Returns (N, 3)."""
    raise NotImplementedError("E2.0-A: solve_positions_3d")


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
    raise NotImplementedError("E2.0-A: fit_aided_calibration")


def holdout_report(anchors: Sequence[Any], cal: AidedCalibration, ranges_m, z_ref_m, yaw_rig_rad, xy_seed_m,
                   huber_m: float = 0.10) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    """Evaluate on epochs NOT used for fitting: solve 3D with Z free, without and with correction.

    Seeds: (xy_seed, z_ref). Bearings for the correction use xy_seed and yaw.
    Returns (report, positions_before (N, 3), positions_after (N, 3)) where report has
    epochs, dz_median_before_m, dz_p95_abs_before_m, solved_before,
    dz_median_after_m, dz_p95_abs_after_m, solved_after (dz = solved z - z_ref).
    """
    raise NotImplementedError("E2.0-A: holdout_report")
