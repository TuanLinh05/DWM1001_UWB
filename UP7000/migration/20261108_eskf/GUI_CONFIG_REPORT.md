# GUI → UP7000 ESKF configuration — 2026-11-08

Implemented and deployed configuration synchronization on each GUI UDP Log
start. The user configures anchor XYZ/enabled rows, TAG-minus-IMU body-FLU
offset, initial rig yaw and the exposed ESKF noise/initialization/timeout
parameters on **3. Rig Layout & Diag**. A measured-setup checkbox controls
whether ESKF may initialize. The laptop position solver remains independent.

## Applied behavior

- GUI validates and freezes the session snapshot. Inputs are locked while
  recording; editing geometry/offset/yaw invalidates the previous confirmation.
- The control request includes normalized JSON and canonical SHA-256. Payloads
  larger than 1,200 bytes use bounded, checksummed fragments and retransmission.
- The root-owned helper validates before changing services or active config.
  It renders a new immutable HDD version and atomically selects it through
  `~/drone_tools/eskf/active_config.json`. The shared schema is installed
  root-owned; privileged code does not import a writable HDD module.
- Supervisor snapshots `gui_config.json`, rig, ROS parameters and physical
  setup into each HDD session. GUI accepts success only after the actual C++
  node publishes the same `config_sha256` and the native recorder subscribes.
- Local `onboard_recording.json` retains the sent config and hash. HDD session
  metadata and `/uwb_eskf/status` retain the same hash. Hash mismatch is PARTIAL.
- Stop/save closes the GUI-owned native MCAP session, then starts a new
  autonomous HDD session using the last applied configuration. That version
  also persists across service/machine restarts.
- Unconfirmed configurations can record raw inputs even with no enabled
  anchors. Confirmed 3D configurations require enough enabled, distinct,
  non-coplanar anchors; C++ also checks the currently received anchor subset.
- Per-anchor GUI host bias/sigma remain metadata. ESKF uses TAG-calibrated
  valid ranges and global `sig_r`, avoiding a second host bias subtraction.

## Validation

1. **44 tests passed** across configuration, fragmented UDP control, GUI
   lifecycle, MCAP, worker/UI, onboard panel and relay integration tests.
   Windows sandbox blocks socket traffic; the successful run used the normal
   network environment. Output: `gui_config_pytest.txt`.
2. Actual offscreen GUI → Tailscale UDP → privileged helper → native ESKF → HDD
   recorder round trip passed. Sent and actual node hash:
   `979d3e77d33e4bf25e52960f3672cea433c8372ed1e0f1e73abd700eb648cb6b`.
   Repeated start did not create a second session; repeated finish returned the
   same autonomous session. Local MCAP closed with 89,327 messages and zero
   dropped events; CSV contained 2,901 rows.
3. Native ROS 2 deserialization of the closed HDD bag passed: 14 PX4 message
   types, IMU, UWB and ESKF status. Snapshot JSON, parameters, session and actual
   node hashes all matched. Deliberately confirmed coplanar config was rejected
   without changing the active config or recording session.
4. Two distinct confirmed configurations were applied to the built C++ node in
   isolated DDS domain 187 using ideal synthetic IMU/ranges. Both reached
   RUNNING, returned the expected hash, and reproduced the supplied yaw and
   TAG–IMU-corrected position. Each produced 244 odometry messages. This checks
   parameter use and frame/lever handling, not physical positioning accuracy.
5. Visible Windows GUI inspection verified that the anchor table, all twelve
   ESKF inputs and the unchecked measured-setup checkbox fit the Rig Layout
   page without clipping. The GUI was left open on that page without logging.

Evidence: `gui_control_results.json`, `gui_hdd_validation.json`,
`gui_config_native_results.json`, `gui_config_live_status.json` and
`gui_config_pytest.txt` in this report directory.

## Current physical setup

Actual anchor geometry and TAG-to-IMU offset/yaw are still unconfirmed. The
active hardware configuration stays `confirmed: false` / WAIT_CONFIGURATION.
The provided TAG distance behind the drone centre (76.3 mm) and ground height
(50.5 cm) are not interpreted as TAG-to-IMU offset. Current raw recording and
all runtime/config/session files remain on HDD. No flight-control output was
added. The native node source was rebuilt successfully on UP7000; prior
sources/helper versions remain backed up on HDD.

User workflow: see `Software/New_GUI/ROS2_BAG_HUONG_DAN.md`, section
**Gửi cấu hình GUI xuống ESKF khi bấm Log**.
