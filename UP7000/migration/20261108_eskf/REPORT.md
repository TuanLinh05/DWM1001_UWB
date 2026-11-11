# UP7000 onboard ESKF + HDD MCAP — 2026-11-08

Deployed and enabled `uwb-eskf-recorder.service`. It supervises the existing
C++ uwb_eskf node and native rosbag2 MCAP recording, independently of the GUI.
Runtime, source/build products, configuration snapshots, ROS/Python caches and
session logs reside on the HDD. The small systemd unit resides in /etc.
The existing onboard logger remains the sole USB serial reader.

## Implementation

- Added full-stream HIGHRES_IMU -> sensor_msgs/Imu at /px4/imu_frd and device
  clock at /px4/imu_device_time_us. Maintained FRD on the input topic; new
  `imu:=mavlink` mode converts once to FLU in the estimator.
- Source IMU intervals are anchored to UP7000 receive time; not hardware sync.
- Added configuration confirmation gate, input validity/order checks, static
  initialization checks, IMU/range timeouts, local estimator status and recovery.
- Corrected nonzero-lever initialization (GN solves TAG position; inertial
  state represents body/IMU) and linear velocity covariance frame conversion.
- Configured calibrated valid UWB inputs with ignore_rig_bias=true to avoid
  subtracting legacy rig bias again.
- Added native MCAP recorder, configuration snapshots/hashes, process/status
  monitoring, sufficient-space/UUID guard, graceful finalization and autostart.
- Expanded Fast DDS maxInitialPeersRange from default 4 to 32. Initial live
  verification found that the recorder did not discover the C++ ESKF publisher
  in the larger process graph. After the profile change, it discovered both
  status and odometry publishers. Preflight/monitoring now checks this path.

## User-provided physical facts and active configuration

The user could not confirm anchor layout. TAG is behind the drone center by
76.3 mm, at 50.5 cm above ground; Pixhawk arrow points forward. These facts
are preserved in physical_setup.json. They do not establish the TAG-IMU lever
or the initial yaw relative to the anchor rig. The GUI anchor layout is only
a provisional reference. configuration_confirmed remains false; actual pose
output is intentionally waiting for the missing measurements.

## Validation

- Both packages built successfully on UP7000, entirely on HDD.
- Logger Python tests: 28 passed, 1 skipped (existing optional dependency test).
- Isolated DDS domain 187: stationary ESKF with nonzero lever initialized at
  the expected body position; stopped output after missing IMU/ranges; recovered
  after inputs returned; rejected stale/NaN input; two anchors did not initialize;
  unconfirmed configuration did not produce a pose.
- The synthetic test also recorded/decoded native MCAP: 463 odometry messages,
  1,184 IMU messages and the range/status inputs. This is an ideal synthetic
  correctness check, not an accuracy measurement of the physical system.
- A complete live session was finalized and every message deserialized using
  ROS 2 on UP7000. The latest validated session lasted about 111 seconds:
  5,535 IMU, 5,535 PX4 reference odometry, 4,931 A5 valid ranges, 4,936 A8 valid
  ranges, 111 ESKF status messages, plus raw clock/diagnostic/session topics.
- IMU sample interval median: 19.55 ms. Hardware estimator status was
  WAIT_CONFIGURATION with zero pose output and zero invalid IMU/range samples
  in that session. There were only two active anchor topics during this test.
- Graceful restart returned estimator=0, recorder=0; session.closed=true.
- All recording files were verified on the HDD filesystem device. Wrong UUID,
  missing UUID and low-space guard cases were checked using mocks; HDD was not
  physically unplugged. No reboot test was performed.

See onboard_results.json and synthetic_results.json beside this report. Raw
native bags, recorder/estimator output and original backups remain on HDD.

## Active paths

- Service: uwb-eskf-recorder (enabled); existing uwb-px4-logger active.
- Configuration: /home/thoa/drone_tools/eskf/config/
- Latest session link: /home/thoa/drone_tools/eskf/latest_session
- Session root: /mnt/droneup/onboard_20261107/thoa/flight_logs/eskf/
- Before-change source backup: /home/thoa/drone_tools/eskf/backups/20261108_050629/
- Original DDS profile: /home/thoa/drone_tools/eskf/backups/fastdds_20261108_051438.before

The runtime remains waiting for verified anchor coordinates, TAG-to-IMU offset
and initial yaw. The implementation does not send estimator output to PX4 flight
control; local odometry and its input/status streams are recorded for evaluation.
