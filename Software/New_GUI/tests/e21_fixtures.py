"""Synthetic data for the E2.1 contract tests (in-flight IMU noise, ESKF-A tuning).

Builds on tests/e20_fixtures.py without changing it. Deterministic, numpy only.
Numbers come from Logs/10112026/Test_1 (hover 127-147 s, PX4 sensor_combined ~221 Hz):
per-sample accel std about (1.2, 4.3, 1.8) m/s^2 and gyro std about (9, 6.6, 5.5) deg/s in
body FLU, most of it above 25 Hz; UWB range innovations stay correlated for more than 1 s.
"""

from __future__ import annotations

import copy
import math

import numpy as np

from e20_fixtures import (ANCHOR_XYZ, LEVER_FLU, ROOM_48, epoch_least_squares,  # noqa: F401  (re-export)
                          simulate_flight)

VIB_ACC_STD = (1.2, 4.3, 1.8)            # m/s^2, body FLU, per sample
VIB_GYRO_STD_DPS = (9.0, 6.6, 5.5)       # deg/s, body FLU, per sample
VIB_TONES_HZ = (137.0, 274.0)            # motor / blade-pass like tones (true frequency; aliased by the stream)


def add_vibration(sim: dict, acc_std=VIB_ACC_STD, gyro_std_dps=VIB_GYRO_STD_DPS, tone_frac=0.5,
                  tones_hz=VIB_TONES_HZ, seed=0) -> dict:
    """Copy of sim with additive vibration on the IMU samples.

    Per axis the added signal has the requested std: tone_frac of its variance in sinusoids at
    tones_hz (random phase, equal share) evaluated at the IMU sample times, the rest white.
    Truth (positions, attitude) is unchanged: vibration amplitude in metres/radians is negligible.
    """
    rng = np.random.default_rng(seed)
    out = copy.copy(sim)
    t = np.asarray(sim["imu_t"], dtype=float)

    def vibration(std):
        cols = []
        for s in std:
            var_tone = tone_frac * s * s / len(tones_hz)
            x = sum(math.sqrt(2 * var_tone) * np.sin(2 * math.pi * f * t + rng.uniform(0, 2 * math.pi))
                    for f in tones_hz)
            cols.append(x + rng.normal(0.0, s * math.sqrt(1 - tone_frac), len(t)))
        return np.column_stack(cols)

    out["imu_acc"] = np.asarray(sim["imu_acc"]) + vibration(acc_std)
    out["imu_gyro"] = np.asarray(sim["imu_gyro"]) + vibration(np.radians(gyro_std_dps))
    return out


def add_correlated_range_errors(sim: dict, sigma_m=0.06, tau_s=3.0, seed=0) -> dict:
    """Copy of sim with a first-order Gauss-Markov error per anchor added to every range event."""
    rng = np.random.default_rng(seed)
    out = copy.copy(sim)
    events = sorted(sim["ranges"], key=lambda e: e[0])
    state, last, new = {}, {}, []
    for t, aid, r in events:
        if aid not in state:
            state[aid] = rng.normal(0.0, sigma_m)
        else:
            phi = math.exp(-(t - last[aid]) / tau_s)
            state[aid] = phi * state[aid] + rng.normal(0.0, sigma_m * math.sqrt(1 - phi * phi))
        last[aid] = t
        new.append((t, aid, r + state[aid]))
    out["ranges"] = new
    return out


def vibrating_flight(*, duration_s=30.0, imu_hz=50.0, uwb_hz=100.0, yaw_rate=0.0, anchor_offsets=None,
                     dropouts=(), correlated=False, seed=11) -> dict:
    """Circle flight (e20 simulate_flight) with Test_1-like vibration on a 50 Hz IMU stream."""
    sim = simulate_flight(duration_s=duration_s, imu_hz=imu_hz, uwb_hz=uwb_hz, yaw_rate=yaw_rate,
                          anchor_offsets=anchor_offsets, dropouts=dropouts, seed=seed)
    sim = add_vibration(sim, seed=seed + 100)
    if correlated:
        sim = add_correlated_range_errors(sim, seed=seed + 200)
    return sim
