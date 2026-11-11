# Unified GUI / UP7000 recording — 2026-11-08

Previously the GUI log button only controlled laptop CSV/MCAP; the onboard
recorder ran independently. Live WiFi UDP logging now starts/ensures the onboard
UWB/PX4 logger and opens a correlated ESKF/native-MCAP session on HDD. The GUI
auto-connects UDP input if needed and waits for acknowledgement before reporting
both sides confirmed. A bounded control channel is separate from USB readers.

The user selected: stop/save closes the GUI-owned HDD session, then continues
autonomous background recording. Window close follows the same workflow.
Record IDs and paths are persisted on both sides. Retry IDs are idempotent;
status is polled while recording. Failures show Log: PARTIAL and preserve local
logs. Demo/local UART do not control onboard hardware. Laptop CSV-only selection
still starts the full HDD MCAP for live UDP. Anchor/filter configuration gates
are unchanged; ESKF currently waits for verified physical configuration.

## Deployed components

- Laptop: onboard_recording.py and GUI lifecycle/status integration.
- HDD: recording_control_server.py, updated run_onboard.py (session context,
  PX4 snapshot topic), application/control logs and action-result cache.
- Small system files: uwb-recording-control.service, the root-owned fixed-action
  helper, and a helper-specific sudoers entry (validated with visudo).
- UDP 15558 only permits source 100.77.150.59 and 127.0.0.1. All control replies
  are limited to 1,200 bytes. No SSH password is stored in the GUI.
- Existing source/runtime backups remain under drone_tools/eskf/backups/,
  including gui_control_20261108_053233 and gui_control_20261108_053556.

## Verification

- 38 tests passed across onboard control, MCAP, workers/GUI, onboard panel and
  UDP relay suites. They cover packet loss/retry, duplicate request rejection,
  source allowlist, malformed input, two-sided start/save, visible failure,
  closing during start, Demo isolation, MCAP serialization and existing flows.
- Real GUI workflow over Tailscale auto-connected, opened an HDD session with
  the same recording ID, recorded actual TAG/PX4, then closed both local MCAP
  and native HDD MCAP. A repeated start kept the same session; a repeated finish
  kept the same background session.
- That laptop test wrote 3,058 CSV rows and 94,216 MCAP messages, with zero local
  queue drops. Native ROS 2 on UP7000 decoded the matching HDD bag, including
  363 IMU, 363 PX4 reference odometry, 327 UWB cycle messages, valid ranges from
  A5–A8, 29 snapshots covering 14 PX4 types, and 7 ESKF status messages.
- The native session matched the GUI recording ID and laptop session name;
  every file was verified on the HDD filesystem; closed=true, error empty.
- A native visible GUI was then opened and its log button clicked directly,
  starting from DISCONNECTED. It auto-connected and displayed Log: ON plus
  'Recording: laptop + UP7000 HDD · ESKF WAIT_CONFIGURATION'. Stop displayed
  SAVING then OFF and 'Saved laptop + UP7000 HDD session · UP7000 continues
  recording in background'. The GUI was left open and connected after saving.

See gui_control_results.json, gui_hdd_validation.json and gui_visible_ui_result.json.
No valid physical ESKF position was claimed: configuration remains unconfirmed.
