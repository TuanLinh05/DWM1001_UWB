# UWB Fast Studio (`New_GUI`) — High-Speed 2D/3D Positioning & Calibration GUI

A streamlined, high-FPS **PyQt6** desktop application built for real-time **1D/2D/3D UWB Calibration** and **2D/3D Positioning & Accuracy Evaluation** with the DWM3000 / nRF52840 firmware (`Burst DS-TWR` 50–300 Hz).

## Why `New_GUI` Was Built

1. **Full Compatibility with Burst DS-TWR (`0x18`) Firmware**:
   - Current TAG firmware defaults to `UWB_TAG_BURST_DEFAULT=1` (`TYPE_RANGE_BURST = 0x18` at 50–300 Hz) while throttling legacy `TYPE_RANGE` (`0x01`) snapshots to 5 Hz.
   - `New_GUI` ingests `RANGE_BURST` (`0x18`), `RANGE_MEAS` (`0x10`), and `RANGE` (`0x01`) directly into the host constant-velocity (CV) filter, 2D/3D solver, and calibration engine at full rate.
2. **Uncalibrated (`CAL_MISSING = 0x20`) + Host/TAG Bias Support**:
   - Works out-of-the-box even when firmware ships with `UWB_DS_CALIBRATED_MASK = 0`, and writes calibrated biases directly to the TAG via `CMD_SET_DS_CAL (0x06)` + `CMD_SAVE_SETTINGS (0x0A)`.
3. **Zero UI Lag at 200+ Hz**:
   - Serial decoding runs in a dedicated `QThread` (`SerialTelemetryWorker`) that batches frames every 25 ms, while `QPainter` canvases (`PositionMapCanvas`, `FastRollingChart`) repaint smoothly at a fixed **30 FPS** on pre-allocated ring buffers.

---

## Quick Start

Install dependencies with the same Python used to launch the GUI:
```powershell
python -m pip install -r requirements.txt
```

### 1. Run with Real TAG Hardware (UART)
```powershell
.\run_gui.ps1 -Port COM6 -Baud 460800
```
Or double-click `CHAY_GUI_MOI.cmd`.

### 2. Run with Built-in 120 Hz 3D Burst Simulator
```powershell
.\run_demo.ps1
```

### 3. Run Automated Unit & Integration Tests
```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

## E2.3 onboard shadow sessions

The UP7000 ESKF panel can import **aided_calibration.json** (E2.0 K=0, same
rig) and display/clear its extra bias per anchor. This is opt-in; the node
subtracts `bias_m` once. Enter the measured TAG minus IMU offset in body FLU
and initial heading, then confirm the physical configuration. Starting a log
with zero offset or no confirmation shows a nonblocking warning and saves it
in the recording manifests. An unconfirmed node records inputs but waits at
`WAIT_CONFIGURATION` without publishing positions.

Read-only analysis from this directory:

```powershell
python tools/onboard_session_report.py Logs/10112026/Test_1/onboard/eskf/20261110_101854_5500 --flight Logs/10112026/Test_1 --tlog Logs/10112026/Test_1/onboard/20261110_100743/px4.tlog --train 75:195 --test 198:218,235:300 --what-if-confirm --lever -0.09,0,0.31 --yaw0-from-flight --aided-bias-json Logs/10112026/Test_1/analysis/e20_aided/aided_calibration.json --out Logs/10112026/Test_1/analysis/e23_onboard_session
```

Without `--flight`, a native synthetic `smoke_test_eskf.py --record-session`
session can be analyzed directly. Outputs stay in `analysis/e23_*`. Offline
replay excludes live freshness/timeouts and publication timing. Flow is only
analyzed for feasibility; no PX4 vision input or flow fusion is enabled.

Use `bash UP7000/scripts/deploy_onboard_eskf.sh` for a read-only deployment
plan, `--build-only` for a separate validated overlay, and explicit `--apply`
for backed-up installation. `--restart` is separate. The source archive uses
`git -c core.autocrlf=false archive` and rejects CR, including CSV fixtures.
Build and test must use the same `--base-paths`; logger tests need
`PYTHONPATH=$HOME/drone_tools/python` and `UP7000/scripts/reset_tag_usb.py`.
See [E23_CHANGE_DESCRIPTION.md](E23_CHANGE_DESCRIPTION.md) for results and
deployment/rollback instructions.

## Position filters

The sidebar defaults to **Robust Adaptive EKF**. It estimates XYZ and velocity
directly from bias-corrected ranges, using the TAG clock. Per-anchor robust noise
estimates, NLOS weighting, normalized innovation gating and gradual link-health
recovery reduce the effect of radio spikes and anchor changes. Missing ranges
are not reused as independent measurements. A full fix needs dimension+1
distinct anchors accepted within the last 0.2 s (`full_fix_window_s`); they do
not have to arrive in the same Burst, so a lossy link carrying one or two ranges
per cycle still keeps a fix (13:16 log: 75.6% -> 99.9% of rows with a position).
Fewer recent anchors mark the fix UNSTABLE; prediction expires after 0.35 s
without a full fix. Used anchors and PDOP refer to that window. A gap above 0.8 s
requires IRLS initialization again. A 2D mode fixes both Z and vertical velocity.
The covariance describes the configured model; it is not a ground-truth error bound.

**Divergence guard.** The IRLS diagnostic is seeded from its own previous fix,
not from the EKF, so it stays independent. If IRLS uses at least dimension+2
anchors with RMS <= 60 mm and PDOP <= 4, sits more than max(0.15 m, 3 sigma)
+ speed x 0.16 s from the EKF, and the same ranges fit IRLS at least twice as
well (and 30 mm better) than the EKF position for 0.5 s, the EKF restarts at
the IRLS fix. The log reason reads `EKF re-anchored to IRLS`; metadata stores
`ekf_guard` settings and `ekf_guard_resets`. When the ranges disagree with
each other (wrong bias, several NLOS anchors) the guard does nothing, because
neither answer can be trusted.

Replay with the guard (EKF, quality on): 08:29:55 — 0 resets in 99,775 rows;
08:17:49 — 4 resets in 129,590 rows, all near the far wall, EKF 0.2-0.45 m from
a consistent IRLS fix and staying at the new fix afterwards; 12:31:10 — 1 reset
(start of log, EKF 437 mm -> 78 mm from IRLS over 3.5-7 s). Each reset is one
visible step (the 4 steps > 90 mm in 08:17:49).

**Legacy CV + IRLS** retains the earlier solver route, with the CV output now
actually used, device-time filtering, and a noise window that recovers after large
range changes. The Host CV checkbox applies only to Legacy. EKF does not stack
the CV/XYZ smoothers or fuse its IRLS diagnostic as another measurement.

Schema-5 logs keep the previous columns, including filter mode, position
standard deviations, prediction age, original Burst flags/status, innovations,
NIS, health and rejection reasons. Metadata records the EKF version/settings.
`raw_x_m/raw_y_m/raw_z_m` remain the independent IRLS diagnostic and can differ
from EKF output. Calibration continues to use the original pre-offset ranges.

On the replay page, **Re-solve** uses the estimator selected in the sidebar.
Every CSV sample is processed before display decimation. Large replays run in
a background worker, with progress, and closing the GUI cancels the worker.

For a full-rate continuity benchmark against the output recorded in an old log:
```powershell
python tools/replay_filter_benchmark.py logs/uwb_fast_20261107_081749_COM14 --output filter_validation
```
This writes JSON statistics and NPZ trajectories. It reports missing fixes and
track extent as well as jumps; it cannot measure absolute position accuracy
without ground truth. Unit tests cover known trajectories, noise, NLOS, spikes,
dropouts, slow motion, reboot, duplicate telemetry and host batch timing.

## Calibration capture progress

Progress is the mean over enabled anchors of min(samples / target, 1); one
slow anchor no longer freezes the bar. A capture ends when every enabled
anchor reaches the target, after `cal_timeout_s` (30 s), or on **Finish now**.
Anchors below min(target, max(20, target/4)) samples are kept in the table as
LOW_SAMPLES, excluded from the applied/written biases and listed in a warning.
While capturing, the status line shows elapsed time and the slowest anchors.

## Spot calibration sanity check

After each spot capture the raw medians are fitted with a free TAG position
plus one common range offset (Huber-weighted). A spot entered at the wrong
coordinates shows up as a position shift; antenna-delay differences remain as
residuals. The calibration page shows a red warning, and **Apply to Host** /
**Write to TAG** ask for confirmation, when:

- the ranges place the TAG more than max(0.25 m, 3 sigma) from the entered spot;
- any anchor bias exceeds ±300 mm (also checked for 1D tape captures);
- anchors still disagree by more than 150 mm RMS (NLOS at the spot or a wrong
  anchor coordinate).

Example: the 07/11 12:31 rig had biases +576/-537/-387/+786/+532/-346/-325/+580
mm. With the default spot (2.5, 2.0, 1.2) the check places the TAG at
(3.14, 1.90, 1.18), 0.64 m away. A spot check needs at least dimension+2
anchors; with fewer, only the bias magnitude is checked.

## UP4000 Onboard tab

Page **5. UP4000 Onboard** listens on UDP 15557 for the `up4000_status` node of the drone computer
(`UP4000/`): host health (CPU, temperature, memory, disk, WiFi), logger counters, session files and
the latest PX4/EKF values (mode, arming, battery, ODOMETRY, attitude, ESTIMATOR_STATUS ratios and
flags, vibration, STATUSTEXT). The "All values received" tree shows every field. Read only; the
node broadcasts once per second until the WiFi relay knows this PC, then sends ~4 times per second.

Status datagrams are capped at 1,200 bytes including the JSON envelope. Large dictionaries are
merged by path; an oversized atomic value (e.g. a long STATUSTEXT list) is reassembled only after
all its fragments arrive. Each PX4 message tracks its own receive time and the source's age.
After 2 seconds without fresh data, its summary values are hidden and marked **STALE**, even
when other messages keep arriving. The all-values tree retains the last received values with a
STALE marker. Host/session/logger sections also show independent stale warnings.

## Radio loss diagnostics

New logs also write `diag.jsonl`: every STATS, DIAG_SYSTEM and DIAG_ANCHOR
frame (cumulative TAG counters, ~1 Hz) with the host time and the number of
Burst rows before it. The Diagnostics page shows the per-second rates for the
last second and hints computed over the session. To explain missing ranges:

```powershell
python tools/diag_summary.py logs/<session>
```

How the TAG firmware (`Firmware/common/ranging/tag_ranging.c`) reports a
missing range — the CSV status is TIMEOUT in all three cases:

- **No RESP**: the anchor was polled and nothing was decoded in its slot
  (`resp_timeouts`). Corrupted frames also raise rx_errors phy_header/fcs/
  sync_loss/lde; a RESP never detected raises only `resp_timeouts`.
- **No Rb**: RESP arrived but the next cycle brought no Rb for it (FINAL late,
  FINAL not received by the anchor, or the next RESP lost; `report_timeouts`).
  A DS range needs RESP in two consecutive cycles, so range success is about
  RESP success squared.
- **Offline**: after 8 missed RESPs in a row (~26 ms at 300 Hz) the anchor is
  not polled and is probed only every 25 cycles (`poll_skipped`). The CSV then
  has no status for that anchor in that cycle.

## Signal quality weighting

**Signal Quality Weight** is enabled by default for Robust Adaptive EKF. The
live table shows a quality score (0-100, not a LOS probability) and **R x**, the
applied radio variance multiplier. Hover a score for reasons, available features
and raw-diagnostic age. Larger R means the EKF trusts that range less. The
checkbox also selects quality on/off for re-solving a recorded session.

The model combines corrected-scale first-path power, RX-FP gap, first-path
amplitude/noise proxy, anchor-side FINAL reception, sudden clock changes and
late replies. Correlated channel indicators use their maximum penalty; the old
NLOS weight is not multiplied twice. Quality only inflates update variance and
does not loosen the innovation gate. It does not shift the measured range.
Weights recover with a 0.25 s time constant. Raw diagnostics expire after 0.30 s and are never
borrowed from the future or another boot. Missing/invalid diagnostics add no
penalty. Unknown indicators are not advertised as proof of good LOS.

Thresholds in `radio_quality.py` are configurable engineering heuristics, not
device-calibrated probabilities. Absolute power thresholds require the firmware
FPP_CORRECTED flag. Stable crystal offset is normal; only sudden changes are
penalized. `RXPACC` and validity flags guard diagnostic use. `FP_INDEX`, raw
carrier integrator, temperature, voltage and CIR power are preserved for later
calibration, without inventing bias corrections from those values. Full CIR
waveforms are not part of this telemetry frame.

New logs append per-range quality, reasons, applied R, feature coverage and
diagnostic age. `rx_quality.jsonl` stores the complete RX_QUALITY messages plus
the number of accepted Burst CSV rows preceding each message. Replay restores
the received order, including diagnostics that arrived after their own Burst.
Metadata records radio profile fields, model settings and dropped-message counts.
Keep the JSONL sidecar beside its CSV. Older logs still work with their available
power/ppm data; they cannot reconstruct missing raw diagnostics.

```powershell
python tools/replay_filter_benchmark.py logs/uwb_fast_20261107_081749_COM14 --output radio_validation --signal-quality on
python tools/replay_filter_benchmark.py logs/uwb_fast_20261107_081749_COM14 --output radio_validation --signal-quality off
```

See `RADIO_QUALITY_VALIDATION.md` for validation and limitations. Signal-weighting
improvements should be measured against independently known positions; smoother
output or smaller solver residual alone does not establish position accuracy.

## E2.1 offline IMU noise and ESKF tuning

E2.1 studies flight vibration on the IMU stream actually consumed by replay.
Run these commands from `Software/New_GUI`; `Logs/...` resolves from the repository
root. Optional readers/plots use the same `pyulog`, `mcap`, `scipy` and `matplotlib`
environment as E2.0. No GUI or onboard configuration is applied by these tools.

```powershell
python tools/imu_noise_report.py Logs/10112026/Test_1 --windows 75:195 --out Logs/10112026/Test_1/analysis/e21_imu_noise_native
python tools/imu_noise_report.py Logs/10112026/Test_1 --windows 75:195 --stream-hz 50 --decimate pick --out Logs/10112026/Test_1/analysis/e21_imu_noise_pick50
python tools/imu_noise_report.py Logs/10112026/Test_1 --windows 75:195 --stream-hz 50 --decimate mean --out Logs/10112026/Test_1/analysis/e21_imu_noise_mean50
python tools/eskf_tuning_report.py Logs/10112026/Test_1 --lever -0.09,0,0.31 --train 75:195 --test 198:218,235:300 --imu-stream-hz 50 --imu-decimate mean --params-json Logs/10112026/Test_1/analysis/e21_imu_noise_mean50/recommended_params.json --out Logs/10112026/Test_1/analysis/e21_eskf_mean50
python tools/eskf_tuning_report.py Logs/10112026/Test_1 --lever -0.09,0,0.31 --train 75:195 --test 198:218,235:300 --imu-stream-hz 50 --imu-decimate pick --params-json Logs/10112026/Test_1/analysis/e21_imu_noise_pick50/recommended_params.json --out Logs/10112026/Test_1/analysis/e21_eskf_pick50
```

Noise outputs: `imu_noise.json`, `recommended_params.json`, `windows.csv`, `psd.png`.
The density is the worst-axis detrended sample standard deviation divided by
the square root of the measured stream rate. Detrending never crosses a data
gap or a selected interval edge. `pick` retains the first sample in each 20 ms
bin; `mean` averages the bin, including its timestamp. Native sample timing
remains irregular. Test_1 produces approximately:

| Stream | na (m/s²/sqrtHz) | ng (rad/s/sqrtHz) | Warnings |
| --- | ---: | ---: | --- |
| Native, median 221.63 Hz | 0.4619 | 0.01506 | VIBRATION_HIGH, IRREGULAR_RATE |
| Pick50, median 51.16 Hz | 0.8966 | 0.02790 | VIBRATION_HIGH, IRREGULAR_RATE, STREAM_ALIASING |
| Mean50, median 49.88 Hz | 0.2932 | 0.01074 | VIBRATION_HIGH, IRREGULAR_RATE |

The PX4 section reports each IMU instance's measured vibration statistics,
clipping counter increments, recorded filter parameters and arming evidence.
No additional threshold is applied to those PX4 metrics. The recording starts
already armed, so it cannot provide a stationary pre-arm noise baseline.

`recommended_params.json` has exactly `params` (every `EskfParams` field) and
`provenance` (flight, windows, requested/measured stream rate, decimation, margin,
version). The replay tool rejects unknown/missing fields, invalid types, nonfinite
values, or provenance that differs from its training windows and IMU stream.
Recommendations preserve the other defaults, set `min_anchor_interval_s=0.02`
and `reinit_keep_attitude=True`, and never reduce noise below the supplied base.

Replay outputs: `report.json`, `comparison.md/.csv`, `trajectories.npz`,
`z_sigma.png`, `yaw_vs_px4.png`.
NPZ state/event keys use `variant__field`, for example `tuned_att__update_t`;
this keeps range and attitude event times in distinct namespaces.
The variants share the same decimated IMU and training-only aided K=0 range correction:
`default` uses `EskfParams()`,
`tuned` uses the recommendation, and `tuned_att` also uses PX4 attitude at 50 Hz.
The per-anchor interval and attitude noise/gate/interval are fixed design choices;
there is no holdout parameter search. Inspect `report.json` → `acceptance` for
every unchanged section 5 threshold, including any failed sanity check.

On Test_1, both tuned variants pass the requested position/consistency bounds.
Pick50 passes all acceptance checks. Mean50 retains one failed baseline sanity
check: its mandatory `EskfParams()` baseline uses every range and does not diverge
as much as the reference expected. A separate diagnostic with the reference's
fixed 0.02 s interval reproduces one reinit and 1.58 m final-window XY RMSE.
The required default and all acceptance thresholds remain as specified.

`normalized_z` interpolates the estimate and body-position sigma at usable RF
epochs, excludes RF age >0.5 s and estimate gaps >0.1 s, and reports coverage and
RMS error/sigma. It omits lever-arm attitude covariance. Innovation correlation
is reported per anchor at 0.1/0.5/1 s. PX4 attitude and IMU are correlated inputs.
RF is the same source used to learn calibration, and GNSS is aligned on training
data: neither reference is surveyed ground truth. Pick50 without attitude aiding
still exhibits substantial yaw drift even when its position metrics pass.

New tools only write below this flight's `analysis/e21_*`; they reject E2.0 and
original-data destinations. `eskf_replay` is now `eskf-a-replay-v2`; its default
trajectories retain E2.0 numerical behavior, with version/new-field metadata added.
See [E21_CHANGE_DESCRIPTION.md](E21_CHANGE_DESCRIPTION.md) for validation and limitations.

## E2.0 offline tools

These tools read a recorded flight and generate files under its
`analysis/e20_*` directories. They do not connect to the drone or apply changes
to the GUI, firmware, PX4 or onboard estimator. The two core modules use only
numpy and the standard library. Optional readers/plots lazily load `pyulog`,
`mcap`, `scipy` and `matplotlib`; if needed, install these separately:

```powershell
python -m pip install pyulog mcap scipy matplotlib
```

These research dependencies are intentionally absent from GUI requirements.
Run from `Software/New_GUI`; paths beginning with `Logs/` resolve against the
repository root:

```powershell
python tools/aided_calibration_report.py Logs/10112026/Test_1 --train 75:195 --test 198:218,235:300 --harmonics 0 --out Logs/10112026/Test_1/analysis/e20_aided
python tools/eskf_a_replay.py Logs/10112026/Test_1 --lever -0.09,0,0.31 --train 75:195 --test 198:218,235:300 --out Logs/10112026/Test_1/analysis/e20_eskf
```

Training/test intervals are half-open PX4 boot seconds and must be disjoint.
Defaults are fixed; no parameter selection uses holdout data. Both tools require
calibrated valid radio ranges (`RADIO_OK + CAL_OK + VALID`) and never subtract
the firmware bias a second time. Clock alignment uses receive-time caches and
the original Test_1 low-delay envelope, including its documented queue-burst
exclusion, rather than fitting trajectories in time.

The aided tool fixes TAG height to a tilt/lever-corrected downward range
reference, anchored to TAG height 0.51 m during the stationary 40–55 s interval.
Scoring/learning excludes epochs more than 0.5 s after the previous rangefinder
sample. It estimates additional per-anchor range errors; K=1/2 may also model
body-bearing harmonics when training yaw covers at least four 45-degree sectors.
Insufficient yaw emits `HEADING_COVERAGE` and uses K=0. Heading is
`yaw_rig = rig_yaw_offset - yaw_ned`; local-NED/GNSS-to-rig alignment is fitted
on training epochs only. The IRLS convention applies Huber factors to both
design and residual rows (squared normal weights), reproducing the provided
numerical examples; it differs from standard square-root-weight Huber
M-estimation. See the change description for this explicit review point.

Outputs are `aided_calibration.json`, `report.json`, `evaluation.npz` and
`dz_before_after.png`. The JSON includes settings, frame conventions, sample
exclusions and per-holdout results. Try `--harmonics 1` with a different
`analysis/e20_*` output directory to inspect the coverage warning on Test_1.

The ESKF replay converts ULog `sensor_combined` FRD to FLU once and uses
`timestamp + accelerometer_timestamp_relative` with actual irregular dt.
It mirrors the 15-state C++ F/Q, asymmetric Huber gate, Joseph covariance and
error-state injection. Its inertial state is the IMU origin; the TAG position
is `p + R @ lever`. Initialization therefore uses `p = tag0 - R0 @ lever`.
The current workspace's C++ node already includes this correction, although
the request described an older node. Live ROS freshness/configuration and
wall-clock timeout guards are outside this offline event driver.

Replay outputs `report.json`, `comparison.csv`, `comparison.md` and
`trajectories.npz`. The five methods are logged GUI filtering, logged raw IRLS,
free-Z robust epoch solves, default ESKF-A and ESKF-A with train-only K=0 aided
correction. Scores use near-uniform 10 Hz epochs, with reference-age exclusions;
raw range/IMU replay itself uses full-rate inputs within IMU coverage. Reports
include range acceptance, per-anchor NIS, resets, timing and input assumptions.

Exit 0 means report generation completed; exit 2 reports input/dependency errors.
All JSON is strict. The RF reference assumes a flat floor and is the same source
used for learning and Z evaluation, so these scores are not independent absolute
accuracy. XY uses training-aligned GNSS and is also not surveyed truth. Real
Test_1 Z improves after calibration; its ESKF XY errors are not uniformly better.
Read `E20_CHANGE_DESCRIPTION.md` before drawing hardware conclusions.

## Joint multi-spot calibration

The Calibration page defaults to **Joint multi-spot (recommended)**. It jointly
estimates an independent bias for each enabled anchor and the true TAG position
at each captured spot. Entered coordinates are robust priors (50 mm per axis),
so one badly entered point does not have to become a false anchor bias. The
sign convention is **measured = true + bias**; the host subtracts the bias.
The solver uses numpy only, Huber weights on ranges and position priors, and a
20 mm systematic range uncertainty in addition to the median sampling error.
Bias uncertainties are scaled covariance estimates under this model.

Version `multi-spot-joint-v2` can run additional solve passes when a small
number of spots have significant `SPOT_OFFSET` warnings. It removes those
spots' position priors entirely and starts the next pass from the preceding
solution. By default at most `floor(max_released_fraction * usable_spots)`
priors are released (`max_released_fraction=0.5`), with at most three passes in
total (`max_passes=3`). Setting `release_flagged_spot_priors=False` keeps the
original single-pass numerical behavior. In 2D a released spot still keeps its
entered Z. The status line lists released spot IDs; their offset cells show
`(released)` and explain that position was estimated from ranges only. The
offset warning remains available for survey review. A spot with no non-outlier
residuals shows `--` for RMS and exports JSON `null`.

If releasing all flagged spots would exceed the configured fraction, the solver
keeps the current pass and reports `MANY_SPOT_OFFSETS`; check anchor coordinates
and the rig origin/axes. Applied metadata and calibration JSON record the
solver version, total passes, released spot IDs and each spot's release state.
Prior release is a safety net: the correct response to a red spot remains to
remeasure its position, or remove it and capture it again, then verify accuracy
at independent surveyed points.

1. Check anchor coordinates, enabled anchors and the sidebar's 2D/3D setting.
   In 2D each spot keeps exactly its entered Z; verify that height carefully.
2. Press **Suggest spots** for four locations at 25%/75% of the anchor box with
   two heights. It loads the first location not already captured. Measure each
   TAG antenna centre with a tape/laser, keep its flight orientation and enter
   the measured coordinates. Suggestions are planning aids, not surveyed truth.
3. Capture P1–P4 with **Samples >= 1000**. A joint solution needs at least two
   usable spots and full data-only Jacobian rank. Each spot needs at least four
   anchors in 3D or three in 2D. Tape captures can constrain individual biases.
4. Inspect **rank**, condition number, residual RMS and the spot table's entered
   versus estimated XYZ, offset, residual and outlier anchor IDs. Red rows have
   a significant `SPOT_OFFSET`. Check the physical location, select the row and
   **Remove selected spot**, then capture again if necessary.
5. Resolve warnings and **Apply to Host Solver**. Only the existing explicit
   Apply action saves the rig configuration. Metadata records the applied
   method, solver version, biases and solution. Export Calibration JSON also
   includes the selected/active method, aggregate and joint diagnostics.

With fewer than two usable spots, a failed solution or deficient rank, the
active biases fall back to the original per-spot median. **Median per spot
(legacy)** selects those original numbers explicitly. The aggregate bias column
shows `J` with the estimated bias standard deviation for joint, or `M` with the
capture count for median; the noise column still describes sample noise.

| Warning code | Meaning / action |
| --- | --- |
| `NO_DATA` | No usable spot/tape result; collect data and check enabled anchors. |
| `SINGLE_SPOT` | Fewer than two spots; the legacy median is used. |
| `SPOT_FEW_ANCHORS` | A spot has insufficient usable anchors and is excluded. |
| `NOT_IDENTIFIABLE` | Data rank is insufficient or solving failed; collect more spatially diverse spots. |
| `SPOT_OFFSET` | Estimated position differs by more than max(10 cm, three position standard deviations); check the survey. |
| `MANY_SPOT_OFFSETS` | Releasing flagged priors would exceed the allowed fraction; check anchor coordinates and rig origin/axes. |
| `RESIDUAL_OUTLIER` | An anchor/spot or tape row exceeds three normalized standard deviations; inspect NLOS and the survey. |
| `RESIDUAL_RMS` | Non-outlier residual RMS exceeds 50 mm; check layout and multipath. |
| `BIAS_LARGE` | A bias exceeds ±300 mm; check distances and geometry. |
| `FLAT_HEIGHTS` | 3D spot heights span less than 30 cm; add another height. |
| `ANCHOR_ONE_SPOT` | An anchor is seen at one spot with no tape constraint; capture it elsewhere. |

Existing per-capture sanity warnings remain available alongside joint warnings.
Verify the calibration at independent surveyed **V1/V2**, excluded from the
calibration captures, before treating its bias accuracy as established.

## G0 tools

Run these commands from `Software/New_GUI`. Both tools use only numpy and the
standard library, leave input logs intact, close their files and write strict
JSON (unavailable/non-finite statistics are `null`). Keep the CSV, metadata,
`rx_quality.jsonl` and `diag.jsonl` together.

### Radio loss

```powershell
python tools/radio_loss_report.py logs/uwb_fast_20261107_123110_COM14 logs/uwb_fast_20261107_131601_COM14 --bin-s 5 --out logs/g0_validation
```

Without `--out`, each session gets `radio_loss_report.json` and
`radio_loss_bins.csv`; with `--out`, filenames are prefixed by the session name.
The report contains polling/range/RESP/late percentages, outage runs >= 0.2 s,
the longest outage, the 0–8 anchors-per-cycle histogram, cycle-sequence gaps,
time bins with TAG temperature/voltage and Pearson temperature correlation.
`share_cycles_ge` values are percentages. RESP statistics use the sampled
RX_QUALITY anchor entries; they need not equal full-rate range percentages.
Missing RX_QUALITY leaves RESP/environment statistics unavailable. Older CSVs
without status-code columns assume all eight anchors were polled.

Classification uses failure rates among polled cycles: 1–2 anchors losing >=5%
are `anchor_specific`; with >=3 bad anchors, co-failure lift >=1.5 means
`common_cause`, <1.2 means `independent`, otherwise `mixed`. A lift near 1 means
little extra simultaneous failure beyond the individual link failure rates.
These are diagnostic categories, not proof of a hardware root cause; check
per-anchor results too. `none` means no measured conditional failure rate
crosses the threshold, and does not establish health for unpolled anchors.
Temperature correlation does not establish causation, especially in short logs.

### Static dataset and acceptance

Keep the TAG on a tripod in its flight orientation; survey the antenna centre
from two walls and the floor. After calibration, record at least 60 s at each
P1–P4 and independent V1/V2. Enter `P1_start` beside **Mark Point**, then press it
at the beginning; the field advances to `P1_end` for the final marker. Repeat
for each point. Empty labels keep the usual W1/W2 labels. People should stand
away from the radio paths. Record an additional 10 minutes stationary LOS for
radio acceptance (mean >=7/8 anchors per cycle).

Copy `tools/dataset_manifest_example.json`, replace every coordinate with the
survey and run:

```powershell
python tools/static_dataset_report.py logs/<session> surveyed_manifest.json --out logs/static_validation
```

The optional `segments` list supplies explicit label pairs; otherwise every
point uses `<point>_start` / `<point>_end`. Default `trim_s=2` discards walking
time at both ends. Missing labels, reversed times and empty trimmed segments
are reported as errors. Outputs are `static_dataset_report.json` and
`static_dataset_anchors.csv`: sample/success counts, raw median, population
sigma, MAD sigma, FFT 1/e autocorrelation time, surveyed distance, raw bias,
active host bias and residual after correction. Per-row bias columns take
precedence over metadata when available; this also handles bias changes while
logging. If the TAG already applied a firmware offset (per-row CAL_OK flag, or
metadata for older logs), the tool first restores the raw range with metadata's
`fw_bias_mm`. Unknown firmware offsets exclude affected samples and appear in
`input_warnings`; keep firmware offsets constant throughout the recording so
the snapshot describes them. Position statistics compare the logged fix with the surveyed point.
Correlation time uses successful samples and a sampling period adjusted for
missing ranges; long structured outages limit that approximation.

Exit **0** means every requested segment passes: each enabled anchor's absolute
corrected residual <=50 mm, mean position error <=0.10 m, mean anchor success
>=87.5%. Missing ranges/fixes fail acceptance. Exit **1** means acceptance
failed; exit **2** means an input/output error. V1/V2 are mandatory in the final
hardware acceptance manifest even though the tool also supports smaller
manifests during diagnosis. Old logs without the surveyed labels cannot certify
this acceptance. Raw recordings remain the reference dataset for later filters.

### G0 automated validation

```powershell
$env:QT_QPA_PLATFORM = "offscreen"
python -m unittest discover -s tests -p "test_*.py"
```

On a restricted runner, point `TEMP` and `TMP` at a writable directory before
running. Existing UDP loopback tests also require local socket access. No COM
connection or Apply action is needed. See `G0_CHANGE_DESCRIPTION.md` for the
implementation validation and the two recorded-session radio summaries.
