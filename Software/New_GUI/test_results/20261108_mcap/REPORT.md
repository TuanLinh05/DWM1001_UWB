# New_GUI ROS 2 MCAP recording — 2026-11-08

Implemented CSV + MCAP recording using a bounded background writer, ROS 2 CDR
serialization and standard std_msgs/geometry_msgs schemas embedded in MCAP.
Each session contains a rosbag2 directory with data_0.mcap, metadata.yaml,
topic counts, recording status, and a playback guide. CSV-only remains available.
Demo sessions are marked simulated and exclude real onboard status/PX4 input.

The existing PlotJuggler launcher is available through **Plot MCAP...** on the
Replay page; **Log Folder** opens the recording directory. The GUI was opened,
connected to UP7000, and its real log button was used to start/stop a recording.
The MCAP file picker was opened. Subsequent launcher logs confirmed PlotJuggler
started and loaded its ROS 2 and MCAP data-loader plugins. Selecting/loading
series in PlotJuggler itself was not visually verified. The GUI and PlotJuggler
were no longer running at the final process check.

## Automated checks

30 tests passed across test_rosbag_logger.py, test_worker_and_gui.py,
test_up4000_panel.py and test_udp_relay_input.py. Coverage includes real MCAP
and CDR round trips, split PX4 snapshots, stale/missing age fields, queue
overflow, write failure reporting, rapid session creation, GUI start/stop,
CSV-only selection, Demo isolation and finalization on GUI close.

## Actual ROS 2 Humble verification in WSL Ubuntu-22.04

| Recording | Messages decoded by rosbag2_py | Topics | Native playback check |
| --- | ---: | ---: | --- |
| UP7000 live, 20 seconds | 24,872 | 174 | 80 Float64 samples received on /px4/attitude/roll |
| Demo, 3 seconds | 13,578 | 51 | 277 PointStamped samples received on /uwb/position |

Every message was deserialized using the installed ROS 2 message classes.
Both bags passed `ros2 bag info`. `ros2 bag play` was tested with actual rclpy
subscribers, using ROS_DOMAIN_ID=187 and localhost-only DDS for this test.
Both recording queues reported zero drops and successful finalization.
This is not a measurement of WiFi/Tailscale packet loss.

Live source was 100.123.216.14 with 14 PX4 message types. The hardware run had
no UWB position fix; no synthetic XYZ topic was emitted for it. XYZ/header
timestamps and native playback were checked separately with Demo data.

The separate recording made by clicking the visible GUI's log button wrote
35,841 ROS 2 messages from 4,641 accepted input events, zero queue drops,
an empty queue on stop, and `closed: true`. It is located at:

`Software/New_GUI/logs/uwb_fast_20261108_114827_udp_15555/rosbag2`

Detailed validation inputs and native read/play results are in
`.tmp_up7000_tailscale/mcap_validation/`, including results.json,
ros2_native_results.json and the ros2 bag info/play outputs.

## Limits

- PX4 data is the received approximately 4 Hz status snapshot feed, with
  unchanged payloads deduplicated. It does not replace full-rate onboard logs.
- Bag timestamps represent laptop receive time; device clocks remain separate.
- Missing packets or missing anchor ranges are not reconstructed.
- MCAP/CSV files are recorded on the laptop. No onboard service or eMMC storage
  configuration was changed for this feature.
