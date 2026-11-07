# G0: joint multi-spot calibration and dataset diagnostics

## Change description / PR body

Single-spot calibration treated entered TAG coordinates as exact, so a misplaced
point could become a false per-anchor bias. The default joint solver now fits
enabled-anchor biases and spot positions together with robust range and position
priors. The original median method remains selectable and is the automatic
fallback for insufficient or unidentifiable data.

- Pure numpy Gauss–Newton/Huber solver, data-only rank/condition diagnostics,
  covariance scaling, 2D fixed entered heights, tape constraints, stable warning
  codes, strict JSON and suggested capture locations.
- Cached engine integration, capture removal with unique remaining IDs, applied
  method/version/bias metadata, and JSON calibration summaries.
- Calibration method selector, entered/estimated spot table, significant-offset
  highlighting, suggestions, capture removal and `J`/`M` aggregate markers.
- Radio-loss CLI: per-anchor polling/range/RESP/late statistics, outages,
  anchors-per-cycle histogram, UART cycle gaps, environmental bins, temperature
  correlation and specified co-failure classification.
- Labelled waypoints, automatic `_start` to `_end` label progression, surveyed
  static-segment statistics, FFT correlation time and explicit acceptance exits.
  Per-row host bias takes precedence over metadata. Optional firmware offsets
  are restored before host correction, using CAL_OK and metadata `fw_bias_mm`;
  unknown firmware offsets invalidate affected samples and are reported.
- README instructions and independent regression tests. No dependencies added.

Branch: `feature/g0-calibration-dataset`. Baseline: `6f71f6e` (40 existing
New_GUI files, including the original contracts). The existing local
`Software/New_GUI/.gitignore` already excludes `logs/` and was included in that
baseline. Implementation commits follow G0.1/G0.2/G0.3, with focused follow-up
fixes and separate validation/documentation commits.

## Validation

Windows / Python 3.12.10, PyQt6 offscreen:

```powershell
cd "Software/New_GUI"
$env:QT_QPA_PLATFORM = "offscreen"
python -m unittest discover -s tests -p "test_*.py"
```

**128/128 pass: 78 existing tests + 39 unchanged G0 contract tests + 11 added
regression tests.** The original 117-test suite also passed before the extra
regression tests were added. The final run is stored locally at
`logs/g0_validation/unittest_128.txt`. On the restricted runner, TEMP/TMP were
set to `logs/g0_test_tmp`; UDP tests ran with loopback access. No COM port was
opened and no GUI Apply action was invoked.

The extra tests cover deficient-rank median fallback and strict JSON; cache
invalidation by settings, mutable anchor geometry/enabled state and capture
identity; reset persistence; unique capture IDs after removal and numbering
after clearing; tape-only/non-finite inputs; invalid solver settings; old CSVs
and late flags; sequence wrap and boot-separated outages; missing range/fix
acceptance failures; per-sample host bias; firmware-offset restoration,
metadata fallback and unknown-offset rejection; and empty radio recordings.

Original `tests/test_g0_*.py` and `tests/g0_fixtures.py` have no diff from the
baseline. `last_rig_config.json` is unchanged; SHA-256:
`EAB2461C2C178DEC869CED6280472DE64A327A5DF66DF339163AE286E8F4EDD9`.
Unrelated pre-existing UART/UP4000 work was left in place and excluded from
these commits. No firmware changes (G0.4) were made.

The Calibration page was rendered offscreen at 1700×1100 and the default
1540×920 window size using four captures including a 0.65 m entered-coordinate
error. Its estimated offset is 0.633 m, rank 20/20, condition 10.2 and residual
RMS 2.4 mm. The flagged row and active joint biases are visible in light and
dark themes; the anchor table scrolls at smaller heights. PNGs are in
`logs/g0_validation/`. An additional GUI smoke check verified strict JSON export,
selected-row removal, method switching and suggestions with `_persist_rig_config`
patched to reject any implicit save; all passed. The five G0 GUI tests passed
again after the theme refinement.

An additional deterministic Monte Carlo check used seed `20261108`, the
fixture's eight-anchor rig and four surveyed points, known biases, and fresh
independent 2.5 mm Gaussian median errors. Each scenario ran 200 trials:

| Scenario | Maximum bias error | 95th percentile of maximum bias error | Offset detection |
| --- | ---: | ---: | --- |
| Clean entered points | 5.88 mm | 4.94 mm | 0/200 false offset warnings |
| P2 entered 0.65 m low in X | 17.30 mm | 14.80 mm | 200/200 detected |
| P3/A6 has +400 mm NLOS error | 26.43 mm | 24.32 mm | A6 flagged 200/200; 0 false spot offsets |

All trials returned identifiable joint solutions. Maximum position errors were
10.8, 24.1 and 91.7 mm respectively. Details:
`logs/g0_validation/monte_carlo.json`. This is a synthetic check against known
truth, not a comparison with the reviewer's private reference implementation or
a measurement of physical accuracy.

## Recorded-session radio results

```powershell
python tools/radio_loss_report.py logs/uwb_fast_20261107_123110_COM14 logs/uwb_fast_20261107_131601_COM14 --bin-s 5 --out logs/g0_validation
```

Both original recordings have status columns and RX_QUALITY data. The command
completed successfully and wrote strict JSON plus time-bin CSV for each session.

| Metric | `uwb_fast_20261107_123110_COM14` | `uwb_fast_20261107_131601_COM14` |
| --- | ---: | ---: |
| Recorded cycles | 24,908 | 8,992 |
| TAG-time duration | 89.525 s | 28.276 s |
| Mean anchors per cycle | **3.662/8** | **1.073/8** |
| Cycles with at least 7 ranges | 2.987% | 0% |
| Missing cycle sequence numbers (UART/host) | 15 | 33 |
| Co-failure lift | 1.0184 | 1.0059 |
| Specified classification | **independent** | **independent** |
| Number of 5 s bins | 18 | 6 |
| Bin mean TAG temperature span | 44.427–45.800 °C | 49.220–49.453 °C |
| Temperature vs. mean range-count Pearson r | -0.7840 | -0.3265 |

Per-anchor percentages below condition range/RESP on polling. RESP is sampled
from RX_QUALITY entries, whereas ranges use every recorded cycle:

| Anchor | 12:31 polled | 12:31 range / polled | 12:31 RESP | 13:16 polled | 13:16 range / polled | 13:16 RESP |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A1 | 99.42% | 51.47% | 75.11% | 94.91% | 22.53% | 53.99% |
| A2 | 99.23% | 38.21% | 69.60% | 99.73% | 33.35% | 60.51% |
| A3 | 100.00% | 53.65% | 78.07% | 4.00% | 0.00% | 0.00% |
| A4 | 99.90% | 50.39% | 76.91% | 82.63% | 11.83% | 41.99% |
| A5 | 98.27% | 36.71% | 67.21% | 58.15% | 4.82% | 36.36% |
| A6 | 100.00% | 51.65% | 77.50% | 98.67% | 21.35% | 52.38% |
| A7 | 100.00% | 44.25% | 72.27% | 81.15% | 10.52% | 39.91% |
| A8 | 99.61% | 41.35% | 72.21% | 89.02% | 11.77% | 54.07% |

The hypothesis that many anchors fail *together in the same cycles* is not
supported by the plan's lift criterion: neither session reaches 1.5; both are
close to 1. This favors inspecting RF margin, antenna/orientation, distance, TX
power and interference. It does not rule out a TAG/environment issue that
reduces all links' margins without synchronizing each failure. Very high
marginal loss rates also limit what this classifier can distinguish.

In the 13:16 recording, **A3 produces no range or sampled RESP for the entire
28.276 s** and is polled in only 4% of cycles; inspect A3 power/antenna/link in
addition to the overall RF problem. A5's longest outage is 1.204 s; A7's is
0.983 s. The negative temperature correlations, especially the six-bin short
recording, do not establish a thermal cause. Neither recording meets the
mean >=7/8 radio target, and neither is a 10-minute acceptance recording.

Local artifacts (excluded from Git with the original logs):

- `logs/g0_validation/uwb_fast_20261107_123110_COM14_radio_loss_report.json`
- `logs/g0_validation/uwb_fast_20261107_123110_COM14_radio_loss_bins.csv`
- `logs/g0_validation/uwb_fast_20261107_131601_COM14_radio_loss_report.json`
- `logs/g0_validation/uwb_fast_20261107_131601_COM14_radio_loss_bins.csv`

## Remaining hardware acceptance

G0.1–G0.3 code and automated validation are complete. Physical acceptance still
requires the owner's surveyed P1–P4 plus independent V1/V2 recordings, using
the README workflow and a real-coordinate manifest. Confirm full-rank joint
calibration, resolve spot-offset warnings, then check all V1/V2 residuals
<=50 mm, mean position error <=10 cm and success >=87.5%. Record the additional
10-minute stationary LOS radio session. The existing recordings diagnose radio
loss; they do not establish independent surveyed-position accuracy.
