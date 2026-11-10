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
