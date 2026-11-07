"""Shared synthetic data for the G0 tests (calibration, radio loss, static dataset).

Everything here is deterministic. It builds the same kinds of objects the GUI
records: calibration captures (through the real UwbStateEngine) and schema-5
session folders (uwb_fast_log.csv + session_metadata.json + rx_quality.jsonl).
Only the CSV columns that the G0 tools are allowed to depend on are written.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys
from typing import Callable, Iterable, Mapping, Sequence

import numpy as np

NEW_GUI_DIR = Path(__file__).resolve().parents[1]
OLD_GUI_DIR = NEW_GUI_DIR.parent / "UWB_UART_GUI"
for _p in (NEW_GUI_DIR, OLD_GUI_DIR, NEW_GUI_DIR / "tools"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import telemetry_protocol as tp  # noqa: E402
from core_state import AnchorConfig, UwbStateEngine  # noqa: E402

# 07/11 rig: 5.4 x 3.6 m, anchors at 0.48 m and 2.208 m (last_rig_config.json).
ROOM_54 = (
    AnchorConfig(1, 0.0, 0.0, 0.48), AnchorConfig(2, 5.4, 0.0, 0.48),
    AnchorConfig(3, 5.4, 3.6, 0.48), AnchorConfig(4, 0.0, 3.6, 0.48),
    AnchorConfig(5, 0.0, 0.0, 2.208), AnchorConfig(6, 5.4, 0.0, 2.208),
    AnchorConfig(7, 5.4, 3.6, 2.208), AnchorConfig(8, 0.0, 3.6, 2.208),
)
# Realistic per-anchor DS-TWR biases (mm), firmware sign: measured = true + bias.
TRUE_BIAS_MM = {1: 35.0, 2: 117.0, 3: 99.0, 4: 243.0, 5: -22.0, 6: 75.0, 7: 89.0, 8: 91.0}
# Four spots, two heights, inside the rig (what suggest_calibration_spots should propose).
SPOTS_4 = ((1.35, 0.90, 1.00), (4.05, 2.70, 1.70), (4.05, 0.90, 1.00), (1.35, 2.70, 1.70))
# Small deterministic "noise" on the medians (mm), different per spot and anchor.
_NOISE_MM = (3.0, -2.0, 1.5, -1.0, 2.5, -3.0, 0.5, -1.5)


def true_range_mm(xyz: Sequence[float], anchor: AnchorConfig) -> float:
    return math.dist(tuple(xyz), (anchor.x, anchor.y, anchor.z)) * 1000.0


def spot_observation(spot_index: int, true_xyz, entered_xyz, *, extra_mm: Mapping[int, float] | None = None,
                     anchors: Sequence[AnchorConfig] = ROOM_54, bias_mm: Mapping[int, float] = TRUE_BIAS_MM,
                     noise: bool = True, sigma_mm: float = 25.0, samples: int = 200):
    """SpotObservation as the joint solver receives it (median raw range per anchor)."""
    from multi_spot_calibration import SpotObservation

    med = {}
    for k, a in enumerate(anchors):
        v = true_range_mm(true_xyz, a) + bias_mm[a.anchor_id]
        if noise:
            v += _NOISE_MM[(k + 3 * spot_index) % len(_NOISE_MM)]
        if extra_mm and a.anchor_id in extra_mm:
            v += extra_mm[a.anchor_id]
        med[a.anchor_id] = v
    return SpotObservation(
        spot_index=spot_index,
        entered_xyz=tuple(float(v) for v in entered_xyz),
        median_raw_mm=med,
        sigma_mm={aid: sigma_mm for aid in med},
        samples={aid: samples for aid in med},
    )


def feed_spot_capture(engine: UwbStateEngine, true_xyz, entered_xyz, *, cycles: int = 30,
                      bias_mm: Mapping[int, float] = TRUE_BIAS_MM, first_cycle: int = 1) -> int:
    """Run one real spot capture through UwbStateEngine. Returns the next cycle number."""
    engine.start_spot_calibration(spot_xyz=tuple(entered_xyz), target_samples=20)
    cycle = first_cycle
    while engine.calibration_active and cycle < first_cycle + cycles:
        entries = tuple(
            tp.BurstAnchor(a.anchor_id, tp.STATUS_CALIBRATION_MISSING, tp.BURST_FLAG_RADIO_OK,
                           round(true_range_mm(true_xyz, a) + bias_mm[a.anchor_id]), -8000, -7600, 0)
            for a in engine.anchors.values() if a.enabled
        )
        engine.ingest_message(
            tp.RangeBurstMessage(cycle, cycle * 4, 1, cycle, cycle * 8, cycle * 4000, 4000, entries),
            cycle * 0.004,
        )
        cycle += 1
    if engine.calibration_active:
        engine.finish_calibration_now()
    return cycle


def feed_tape_capture(engine: UwbStateEngine, anchor_id: int, reference_m: float, *,
                      bias_mm: Mapping[int, float] = TRUE_BIAS_MM, first_cycle: int = 1) -> int:
    engine.start_single_anchor_calibration(anchor_id=anchor_id, reference_m=reference_m, target_samples=20)
    cycle = first_cycle
    while engine.calibration_active and cycle < first_cycle + 40:
        entry = tp.BurstAnchor(anchor_id, tp.STATUS_CALIBRATION_MISSING, tp.BURST_FLAG_RADIO_OK,
                               round(reference_m * 1000.0 + bias_mm[anchor_id]), -8000, -7600, 0)
        engine.ingest_message(
            tp.RangeBurstMessage(cycle, cycle * 4, 1, cycle, cycle * 8, cycle * 4000, 4000, (entry,)),
            cycle * 0.004,
        )
        cycle += 1
    return cycle


# ----------------------------------------------------------------------------
# Synthetic schema-5 session folders for the tools
# ----------------------------------------------------------------------------
BASE_COLUMNS = ["host_time_s", "boot_id", "cycle_seq", "tag_time_us", "period_us",
                "pos_x_m", "pos_y_m", "pos_z_m", "pos_rms_m", "waypoint_label"]


def _anchor_columns(aid: int) -> list[str]:
    return [f"a{aid}_mm", f"a{aid}_flags", f"a{aid}_status_code", f"a{aid}_bias_mm"]


def write_session(folder: Path, n_cycles: int, *, dt_s: float = 0.004, boot_id: int = 7,
                  range_mm: Callable[[int, int], float | None],
                  polled: Callable[[int, int], bool] | None = None,
                  late: Callable[[int, int], bool] | None = None,
                  cycle_seq: Callable[[int], int] | None = None,
                  position: Callable[[int], tuple[float, float, float] | None] | None = None,
                  labels: Mapping[int, str] | None = None,
                  anchors: Sequence[AnchorConfig] = ROOM_54,
                  host_bias_mm: Mapping[int, float] | None = None,
                  quality_every: int = 25,
                  temperature_c: Callable[[float], float] | None = None,
                  resp_ok: Callable[[int, int], bool] | None = None) -> Path:
    """Write uwb_fast_log.csv, session_metadata.json and rx_quality.jsonl.

    range_mm(row, aid) -> raw range in mm, or None when the anchor gave no range.
    polled(row, aid)   -> False writes an empty status code (TAG skipped the anchor).
    """
    folder.mkdir(parents=True, exist_ok=True)
    host_bias_mm = host_bias_mm or {a.anchor_id: 0.0 for a in anchors}
    cols = list(BASE_COLUMNS)
    for a in anchors:
        cols += _anchor_columns(a.anchor_id)
    t0_us = 1_000_000
    with (folder / "uwb_fast_log.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for row in range(n_cycles):
            t_us = t0_us + int(round(row * dt_s * 1e6))
            pos = position(row) if position else None
            out = [f"{1_700_000_000 + row * dt_s:.6f}", boot_id,
                   cycle_seq(row) if cycle_seq else 1000 + row, t_us, int(dt_s * 1e6),
                   "" if pos is None else f"{pos[0]:.4f}", "" if pos is None else f"{pos[1]:.4f}",
                   "" if pos is None else f"{pos[2]:.4f}", "" if pos is None else "0.0100",
                   (labels or {}).get(row, "")]
            for a in anchors:
                aid = a.anchor_id
                is_polled = polled(row, aid) if polled else True
                r = range_mm(row, aid) if is_polled else None
                flags = 0x20 | (0x10 if (late and late(row, aid)) else 0)
                out += ["" if r is None else int(round(r)), flags if is_polled else "",
                        (0 if r is not None else 3) if is_polled else "", f"{host_bias_mm[aid]:.1f}"]
            w.writerow(out)
    meta = {
        "schema": 5,
        "anchors": [
            {"anchor_id": a.anchor_id, "x": a.x, "y": a.y, "z": a.z, "enabled": True,
             "bias_mm": host_bias_mm[a.anchor_id], "sigma_mm": 30.0, "fw_calibrated": False, "fw_bias_mm": 0.0}
            for a in anchors
        ],
        "waypoints": [],
    }
    (folder / "session_metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    with (folder / "rx_quality.jsonl").open("w", encoding="utf-8") as fh:
        for row in range(0, n_cycles, quality_every):
            t_s = row * dt_s
            msg = {
                "sequence": 1000 + row, "time_ms": int(t_s * 1000), "boot_id": boot_id,
                "cycle_seq": 1000 + row, "t_us": t0_us + int(round(t_s * 1e6)),
                "temperature_c": temperature_c(t_s) if temperature_c else 40.0, "vbat_v": 3.7,
                "anchors": [{"anchor_id": a.anchor_id,
                             "flags": int(resp_ok(row, a.anchor_id)) if resp_ok else 1}
                            for a in anchors],
            }
            fh.write(json.dumps({"after_burst_rows": row, "message": msg}) + "\n")
    return folder


def ar1_noise(n: int, tau_s: float, dt_s: float, sigma: float, seed: int) -> np.ndarray:
    """Stationary first-order Gauss-Markov noise with correlation time tau_s."""
    rng = np.random.default_rng(seed)
    phi = math.exp(-dt_s / tau_s)
    out = np.empty(n)
    out[0] = rng.normal(0.0, sigma)
    drive = sigma * math.sqrt(1.0 - phi * phi)
    for k in range(1, n):
        out[k] = phi * out[k - 1] + rng.normal(0.0, drive)
    return out
