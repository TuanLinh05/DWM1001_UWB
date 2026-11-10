# E2.0 — sensor-aided UWB calibration and offline ESKF-A replay

Implemented 10 November 2026 on `feature/g0-calibration-dataset`.

## Behavior and scope

`aided_calibration.py` learns extra per-anchor range bias from recorded,
already-calibrated UWB ranges, using fixed rangefinder-derived TAG Z and PX4
heading. It alternates free XY solves and robust constant/harmonic regression.
Body bearing is `atan2(anchor_y-tag_y, anchor_x-tag_x) - yaw_rig`, with
`yaw_rig = rig_yaw_offset - yaw_ned`. Requested K=1/2 falls back to K=0 unless
the training data cover at least four 45-degree yaw sectors. Holdout solves
leave Z free; RF is only their initial height seed, not a solve constraint.

`eskf_replay.py` mirrors the 15-state `filter.hpp` predict, scalar asymmetric
Huber/gating update, Joseph covariance, right-multiplicative attitude injection,
Rodrigues exponential, static attitude initialization and GN position
initialization. It merges chronological IMU/range events deterministically and
supports position reinitialization after more than 200 consecutive refusals.
The filter has no anchor-bias states and does not remove persistent range bias.

Shared `tools/flight_log_io.py` reads CSV, existing MCAP caches (or decodes MCAP
in memory), ULog, the TAG/PX4 clock model, attitude and height reference. It
converts IMU FRD to FLU exactly once and uses actual accel sample timestamps
and irregular dt. It neither rewrites caches nor changes input recordings.

Only offline E2.0 modules/tools, their unchanged supplied contracts, an extra
offline IO regression test file and E2.0 documentation hunks are committed.
GUI/onboard/MCAP changes already present in the working tree, PX4 configuration,
firmware, `requirements.txt` and `last_rig_config.json` are untouched. There is
no connection to the physical drone or UP7000.

## Numerical conventions and current C++ source

**Aided IRLS convention requiring explicit review:** Huber factor
`min(1, huber_m/abs(residual))` multiplies both each Jacobian/design row and its
residual/target, giving squared factors in normal equations. This is stronger
than standard square-root-weight Huber M-estimation. The supplied fixture
reference values are reproduced by this row convention: gross-outlier maximum
bias error is 7.449 mm (the request quotes 7.4 mm). A standard square-root-weight
implementation gave 12.540 mm on that fixture, beyond its fixed 12 mm test
threshold. No test, threshold, default Huber scale or holdout parameter was
changed. This ambiguity is documented rather than silently calling the two
weighting conventions equivalent. The reviewer's private implementation was
not available, so no claim is made about its source code.

The core ESKF scalar update independently follows **exactly** the explicit
`filter.hpp` Huber variance-inflation equations, not the aided calibration's
row convention.

The request describes the old C++ initialization as `p=tag0`, but the actual
current `uwb_eskf_node.cpp::try_init` already uses `p0-R0_*lever_`. Python also
uses **body = tag0 - R0 @ lever**. Thus the historical initialization discrepancy
is fixed in the source supplied for this task; no C++ change was made. An older
deployed binary may still differ and was not inspected or updated.

Replay matches the requested offline event driver. It does not reproduce live
ROS arrival-age, confirmed-configuration, publication-time range timeout or
IMU timeout guards. The synthetic 1 s outage test demonstrates filter
prediction, not a guarantee that the live node would publish through that gap.
CSV ranges within one burst share its timestamp; unavailable per-anchor slot
timestamps are not invented.

## Tests and direct C++ parity

- Initial supplied E2.0 contract: 14 errors from skeleton functions.
- Supplied E2.0 contract after implementation: **14/14 pass unchanged**.
- Full requested command: **179/179 pass**, including the pre-existing 165 tests.
- Final source with eight new offline IO/edge tests: **187/187 pass**.
- No supplied `tests/test_e20_*.py` or `tests/e20_fixtures.py` edits. Baseline
  commit `d635d94` records those files and the two skeletons unchanged.

```powershell
cd "Software/New_GUI"
$env:QT_QPA_PLATFORM = "offscreen"
python -m unittest discover -s tests -p "test_*.py"
```

Tests ran with a writable TEMP/TMP directory and existing UDP loopback access.
Final output: `logs/e20_validation/full_suite_final.txt` (187 tests, 62.623 s).
The new tests check required range flags/no double bias, disjoint windows,
causal RF freshness, actual accel timestamps/one frame flip, train-only yaw
alignment, cache/CSV sequence identity, missing dependency messages, negative
lever CLI syntax and isolation of a degenerate position epoch.

Additional integration checks decoded the original MCAP in memory while hiding
only the cache-existence checks: all bridge messages and receive-time samples
matched the existing caches exactly, without rewriting them. Both CLI
entrypoints were also run with a simulated missing `pyulog` import and returned
exit 2 with an actionable dependency message. Requested real-data acceptance
was checked independently after report generation; machine results are in
`analysis/e20_aided/acceptance_checks.json`.

An independent local WSL/Eigen harness compiled the **unchanged production
`filter.hpp`** and compared it against Python on 360 predict/update events:

| Check | Result |
| --- | ---: |
| Acceptance flags | Identical |
| Maximum state/covariance/NIS difference | 1.4211e-13 |
| GN initialization maximum coordinate difference | 4.4409e-16 m |
| Small/finite-angle SO(3) maximum difference | 1.1102e-16 |

This covers covariance entries, IMU-bias injection, positive/negative Huber,
gated ranges and attitude integration. The harness only reads the C++ source;
build products and traces are in `logs/e20_validation/`, not the onboard tree.
Reproduction files are `filter_parity.cpp`, `check_cpp_parity.py` and
`cpp_parity_report.json` in that directory.

## Test_1 aided-calibration acceptance

Executed the exact requested commands from `Software/New_GUI`:

```powershell
python tools/aided_calibration_report.py Logs/10112026/Test_1 --train 75:195 --test 198:218,235:300 --harmonics 0 --out Logs/10112026/Test_1/analysis/e20_aided
python tools/aided_calibration_report.py Logs/10112026/Test_1 --train 75:195 --test 198:218,235:300 --harmonics 1 --out Logs/10112026/Test_1/analysis/e20_aided_h1
python tools/eskf_a_replay.py Logs/10112026/Test_1 --lever -0.09,0,0.31 --train 75:195 --test 198:218,235:300 --out Logs/10112026/Test_1/analysis/e20_eskf
```

Extra bias applies **after** the existing firmware calibration:

| Anchor | Request reference (mm) | Learned (mm) | Difference (mm) |
| --- | ---: | ---: | ---: |
| A1 | +18 | +13.66 | -4.34 |
| A2 | +36 | +28.48 | -7.52 |
| A3 | +43 | +49.27 | +6.27 |
| A4 | +83 | +91.40 | +8.40 |
| A5 | -21 | -25.16 | -4.16 |
| A6 | -36 | -42.72 | -6.72 |
| A7 | -80 | -77.87 | +2.13 |
| A8 | -24 | -20.08 | +3.92 |

Every anchor is within the requested **±15 mm** tolerance.

| Interval (PX4 boot s) | Usable epochs / requested | Median dz before | Median dz after | Requested bound |
| --- | ---: | ---: | ---: | ---: |
| Training 75–195 | 30,594 / 30,595 | +14.38 cm | +0.13 cm | In-sample only |
| Holdout 198–218 | 5,102 / 5,102 | +15.95 cm | **+3.17 cm** | absolute <=6 cm: pass |
| Holdout 235–300 | 15,758 / 16,598 | +9.72 cm | **+1.46 cm** | absolute <=5 cm: pass |

Per-burst dispersion remains: holdout p95 absolute dz after calibration is
22.97 cm and 22.12 cm respectively. The smaller median bias should not be
interpreted as centimetre-level accuracy for every instantaneous solution.

K=1 requested on the same training interval reports **HEADING_COVERAGE** and
`harmonics_used=0`: the training interval covers one yaw sector. The later yaw
change belongs to holdout, so it cannot increase training coverage. K=0 and
this fallback return identical constant corrections.

Selection/reference differences are explicit: all three firmware flags are
required, RF freshness is measured from the most recent past sample with a
0.5 s limit, and the ground anchor uses fresh samples in 40–55 s. Clock fitting
uses the original 5 s low-delay envelope and its TAG-elapsed 250–290 s queue
exclusion. Height retains the original offline linear RF interpolation with
tilt and rangefinder/TAG lever correction, and ground TAG height 0.51 m.
Across the whole record 11,595/76,126 epochs have stale RF; 840 such epochs are
excluded from the late holdout, and one training epoch has fewer than four
valid ranges. These choices are not tuned against holdout errors.

## Test_1 ESKF comparison

Defaults remain fixed (`sig_r=0.15 m`); no parameter search on holdout. Full-rate
IMU/range replay covers the ULog span; the table scores near-uniform 10 Hz actual
CSV epochs with fresh RF. TAG positions, rather than body origins, are compared
for all methods. XY is disagreement against **training-aligned GNSS**, not
surveyed absolute positioning error.

| Interval | Method | Z median (cm) | Z p95 abs (cm) | XY RMSE (cm) | XY p95 (cm) |
| --- | --- | ---: | ---: | ---: | ---: |
| Train 75–195 | GUI filtered | +14.32 | 24.61 | 10.67 | 19.61 |
| Train 75–195 | Logged raw IRLS | +15.68 | 31.34 | 11.11 | 20.37 |
| Train 75–195 | Epoch robust LS | +14.43 | 25.24 | 10.67 | 19.41 |
| Train 75–195 | ESKF-A default | +15.40 | 36.39 | 87.84 | 262.27 |
| Train 75–195 | ESKF-A + aided K=0 | -0.36 | 33.20 | 50.20 | 154.93 |
| Test 198–218 | GUI filtered | +16.50 | 44.35 | 17.91 | 33.81 |
| Test 198–218 | Logged raw IRLS | +17.13 | 30.77 | 16.11 | 24.57 |
| Test 198–218 | Epoch robust LS | +15.22 | 24.94 | 16.32 | 24.59 |
| Test 198–218 | ESKF-A default | +17.87 | 25.67 | 21.56 | 34.71 |
| Test 198–218 | ESKF-A + aided K=0 | +2.96 | 13.35 | 25.87 | 40.21 |
| Test 235–300 | GUI filtered | +32.32 | 74.04 | 25.80 | 32.97 |
| Test 235–300 | Logged raw IRLS | +29.56 | 64.91 | 23.67 | 31.19 |
| Test 235–300 | Epoch robust LS | +9.59 | 31.85 | 26.54 | 33.69 |
| Test 235–300 | ESKF-A default | +13.19 | 30.44 | 28.01 | 47.16 |
| Test 235–300 | ESKF-A + aided K=0 | -2.41 | 20.28 | 36.93 | 66.99 |

| Replay diagnostic | Default ESKF | ESKF + aided K=0 |
| --- | ---: | ---: |
| Input valid ranges within IMU coverage | 507,563 | 507,563 |
| Ranges offered to initialized filter | 505,953 | 505,959 |
| Accepted ranges | 494,095 | 496,798 |
| Acceptance | 97.656% | 98.189% |
| First initialization (boot s) | 58.5323 | 58.5323 |
| Drops after rejection streaks | 1 | 0 |
| Replay time | 28.56 s | 28.28 s |

Per-anchor mean NIS, including refused updates:

| Anchor | Default | Aided K=0 |
| --- | ---: | ---: |
| A1 | 4.626 | 5.157 |
| A2 | 10.388 | 1.020 |
| A3 | 10.910 | 4.232 |
| A4 | 3.245 | 1.577 |
| A5 | 4.034 | 4.641 |
| A6 | 10.616 | 1.074 |
| A7 | 11.473 | 4.751 |
| A8 | 3.271 | 0.989 |

The added calibration improves Z and reduces rejected ranges/reinitialization.
It does **not** establish uniform ESKF superiority on this recording: XY is
worse than the epoch/logged estimators in several windows, and the pre-yaw
training trajectory has large ESKF tails. There is no attempt to tune those
away using holdout. Synthetic clean-IMU performance does not transfer directly
to recorded IMU, model error, timing uncertainty or unmodelled range effects.

## Artifacts, limits and preservation

`Logs/10112026/Test_1/analysis/e20_aided/` contains strict calibration/report JSON,
evaluation arrays and `dz_before_after.png`; `e20_aided_h1/` demonstrates the
coverage fallback. `e20_eskf/` contains report JSON, comparison CSV/Markdown and
trajectories. Existing source caches and earlier analysis are unchanged.

The RF reference assumes a flat floor and measured mounting/ground geometry.
Z is evaluated against **the same sensor source used to learn the correction**,
not independent surveyed truth. GNSS XY alignment is estimated on training
data and cannot establish absolute positioning accuracy. Heading harmonics
still need the owner's multi-heading recording; no hardware logger/PX4 setup
was changed. A 65-file SHA-256 audit covers all protected tests, original flight
files/caches/analysis, C++ sources, rig and existing user modifications. The only
allowed changes among the original snapshots are the two E2 cores and the E2
README addition; the audit passed. Details are in
`logs/e20_validation/baseline_hashes.json` and `preservation_report.json`.
