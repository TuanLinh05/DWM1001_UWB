"""Synthetic data for the E2.0 contract tests (aided UWB calibration, ESKF-A replay).

Deterministic generators only; no project imports besides numpy. Frames:
rig = ENU of the anchor box (z up); body = FLU; yaw_rig = heading of body +X,
counter-clockwise in the rig frame. TAG lever is given in body FLU.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys

import numpy as np

NEW_GUI_DIR = Path(__file__).resolve().parents[1]
if str(NEW_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(NEW_GUI_DIR))


@dataclass(frozen=True)
class Anchor:
    anchor_id: int
    x: float
    y: float
    z: float
    enabled: bool = True


# Rig of the 10/11/2026 flight: 4.8 x 3.6 m, anchors at 0.48 m and 2.208 m.
ROOM_48 = (
    Anchor(1, 0.0, 0.0, 0.48), Anchor(2, 4.8, 0.0, 0.48), Anchor(3, 4.8, 3.6, 0.48), Anchor(4, 0.0, 3.6, 0.48),
    Anchor(5, 0.0, 0.0, 2.208), Anchor(6, 4.8, 0.0, 2.208), Anchor(7, 4.8, 3.6, 2.208), Anchor(8, 0.0, 3.6, 2.208),
)
ANCHOR_XYZ = np.array([[a.x, a.y, a.z] for a in ROOM_48])
LEVER_FLU = np.array([-0.09, 0.0, 0.31])          # TAG 9 cm behind and 31 cm above the IMU
# Vertical-geometry pattern seen in Test_1: lower anchors read long, upper anchors short (m).
TRUE_BIAS_M = np.array([0.018, 0.036, 0.043, 0.083, -0.021, -0.036, -0.080, -0.024])
# Heading-dependent part per anchor: c*cos(theta) + s*sin(theta), theta = anchor bearing in body frame (m).
TRUE_COS_M = np.array([0.040, -0.030, 0.050, 0.020, -0.025, 0.035, -0.040, 0.030])
TRUE_SIN_M = np.array([-0.020, 0.045, 0.010, -0.050, 0.030, -0.015, 0.025, 0.040])
HEADINGS_8 = tuple(math.radians(d) for d in range(0, 360, 45))


def body_bearing(xy: np.ndarray, yaw_rig: np.ndarray) -> np.ndarray:
    """(N, 8) bearing of every anchor seen from the TAG, in the body frame, wrapped to [-pi, pi)."""
    dx = ANCHOR_XYZ[None, :, 0] - xy[:, None, 0]
    dy = ANCHOR_XYZ[None, :, 1] - xy[:, None, 1]
    th = np.arctan2(dy, dx) - yaw_rig[:, None]
    return (th + math.pi) % (2 * math.pi) - math.pi


def true_error_m(xy: np.ndarray, yaw_rig: np.ndarray, harmonics: bool) -> np.ndarray:
    err = np.tile(TRUE_BIAS_M, (len(xy), 1))
    if harmonics:
        th = body_bearing(xy, yaw_rig)
        err = err + TRUE_COS_M * np.cos(th) + TRUE_SIN_M * np.sin(th)
    return err


def survey(points, yaws, *, per=30, noise_m=0.02, z_noise_m=0.01, harmonics=True, outlier_frac=0.0,
           outlier_m=0.5, seed=0) -> dict:
    """Static TAG captures at (x, y, z) points and body headings (radians).

    Returns ranges (N, 8) in m, z_ref (N) = TAG height from an independent sensor,
    yaw (N), xy_true (N, 2), xyz_true (N, 3), xy_seed (N, 2) = truth + 5 cm error.
    """
    rng = np.random.default_rng(seed)
    xyz = np.array([p for p in points for _ in yaws for _ in range(per)], dtype=float)
    yaw = np.array([y for _ in points for y in yaws for _ in range(per)], dtype=float)
    dist = np.linalg.norm(xyz[:, None, :] - ANCHOR_XYZ[None], axis=2)
    ranges = dist + true_error_m(xyz[:, :2], yaw, harmonics) + rng.normal(0.0, noise_m, dist.shape)
    if outlier_frac:
        mask = rng.random(dist.shape) < outlier_frac
        ranges[mask] += outlier_m
    return {
        "ranges": ranges, "z_ref": xyz[:, 2] + rng.normal(0.0, z_noise_m, len(xyz)), "yaw": yaw,
        "xy_true": xyz[:, :2].copy(), "xyz_true": xyz, "xy_seed": xyz[:, :2] + rng.normal(0.0, 0.05, (len(xyz), 2)),
    }


GRID_POINTS = tuple((x, y, z) for x in (1.2, 2.4, 3.6) for y in (0.9, 1.8, 2.7) for z in (1.0, 1.5))
TEST_POINTS = ((1.8, 1.35, 1.25), (3.0, 2.25, 1.25), (2.4, 1.8, 1.6), (1.5, 2.4, 1.1))


# --------------------------------------------------------------------------------------------
# Flight simulation for the ESKF-A replay
# --------------------------------------------------------------------------------------------
def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _log_so3(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2
    return th / (2 * math.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def _attitude(acc_world, yaw):
    """Quadrotor attitude: body z along thrust (acc + g), body x towards yaw."""
    zb = acc_world + np.array([0.0, 0.0, 9.81])
    zb = zb / np.linalg.norm(zb)
    xc = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    yb = np.cross(zb, xc); yb /= np.linalg.norm(yb)
    xb = np.cross(yb, zb)
    return np.column_stack([xb, yb, zb])


def simulate_flight(*, duration_s=30.0, hover_s=2.0, imu_hz=200.0, uwb_hz=100.0, radius_m=1.0, period_s=6.0,
                    center=(2.4, 1.8, 1.3), z_amp_m=0.15, yaw0=math.radians(30.0), yaw_rate=0.0,
                    lever=LEVER_FLU, range_noise_m=0.03, acc_noise=0.03, gyro_noise=0.002,
                    acc_bias=(0.05, -0.03, 0.02), gyro_bias=(0.002, -0.001, 0.0015),
                    spikes_frac=0.0, spike_m=1.0, anchor_offsets=None, dropouts=(), seed=0) -> dict:
    """Hover for hover_s seconds, then blend (C2-continuous) into a circle with a vertical wobble.

    Returns imu (t, acc FLU specific force, gyro), range events (t, anchor_id, range_m),
    and the truth sampled at the IMU times (body/IMU position p, TAG position tag, R).
    anchor_offsets: {anchor_id: (t0, t1, offset_m)}; dropouts: [(t0, t1)] with no ranges at all.
    """
    rng = np.random.default_rng(seed)
    fine_dt = 1e-3
    t = np.arange(0.0, duration_s + 2 * fine_dt, fine_dt)
    s = np.clip((t - hover_s) / 2.0, 0.0, 1.0)
    blend = s * s * s * (10 - 15 * s + 6 * s * s)         # smootherstep: continuous acceleration
    w = 2 * math.pi / period_s
    phase = w * np.maximum(t - hover_s, 0.0)
    c = np.array(center)
    pos = np.column_stack([
        c[0] + blend * radius_m * (np.cos(phase) - 1.0),
        c[1] + blend * radius_m * np.sin(phase),
        c[2] + blend * z_amp_m * np.sin(2 * phase),
    ])
    vel = np.gradient(pos, fine_dt, axis=0)
    acc = np.gradient(vel, fine_dt, axis=0)
    yaw = yaw0 + yaw_rate * np.maximum(t - hover_s, 0.0)
    Rs = np.array([_attitude(acc[k], yaw[k]) for k in range(len(t))])
    step = int(round(1.0 / (imu_hz * fine_dt)))
    idx = np.arange(1, len(t) - 1, step)
    imu_t = t[idx]
    omega = np.array([_log_so3(Rs[k - 1].T @ Rs[k + 1]) / (2 * fine_dt) for k in idx])
    f_body = np.array([Rs[k].T @ (acc[k] + np.array([0.0, 0.0, 9.81])) for k in idx])
    imu_acc = f_body + np.array(acc_bias) + rng.normal(0.0, acc_noise, f_body.shape)
    imu_gyro = omega + np.array(gyro_bias) + rng.normal(0.0, gyro_noise, omega.shape)
    tag = np.array([pos[k] + Rs[k] @ np.asarray(lever) for k in range(len(t))])
    events = []
    for tu in np.arange(0.0, duration_s, 1.0 / uwb_hz):
        if any(a <= tu < b for a, b in dropouts):
            continue
        k = int(round(tu / fine_dt))
        for i, a in enumerate(ROOM_48):
            r = float(np.linalg.norm(tag[k] - ANCHOR_XYZ[i])) + rng.normal(0.0, range_noise_m)
            if anchor_offsets and a.anchor_id in anchor_offsets:
                t0, t1, off = anchor_offsets[a.anchor_id]
                if t0 <= tu < t1:
                    r += off
            if spikes_frac and rng.random() < spikes_frac:
                r += spike_m
            events.append((tu + i * 2.7e-4, a.anchor_id, r))      # slots ~0.27 ms apart
    return {
        "imu_t": imu_t, "imu_acc": imu_acc, "imu_gyro": imu_gyro, "ranges": events,
        "truth_t": imu_t, "truth_p": pos[idx], "truth_tag": tag[idx], "truth_R": Rs[idx], "lever": np.asarray(lever),
        "yaw0": float(yaw0),
    }


def epoch_least_squares(events, t_grid, window_s=0.012) -> np.ndarray:
    """Per-epoch unweighted 3D least squares of the TAG (comparison baseline, no filtering)."""
    ev = np.array(events)
    out = []
    p = np.array([2.4, 1.8, 1.3])
    for tg in t_grid:
        m = (ev[:, 0] >= tg - window_s) & (ev[:, 0] < tg)
        if m.sum() < 4:
            out.append([np.nan] * 3)
            continue
        ids = ev[m, 1].astype(int) - 1
        r = ev[m, 2]
        for _ in range(8):
            d = p - ANCHOR_XYZ[ids]
            dist = np.linalg.norm(d, axis=1)
            p = p + np.linalg.lstsq(d / dist[:, None], r - dist, rcond=None)[0]
        out.append(p.copy())
    return np.array(out)
