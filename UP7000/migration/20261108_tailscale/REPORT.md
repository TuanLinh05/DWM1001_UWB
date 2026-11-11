# UP7000 to New_GUI over Tailscale — 2026-11-08

Persistent configuration was applied to UP7000 (`thoa`, `100.123.216.14`):

```sh
# /etc/default/uwb-px4-logger
LAUNCH_ARGS='log_root:=/mnt/droneup/onboard_20261107/thoa/flight_logs ground_host:=100.77.150.59'
```

The logger service was restarted and is enabled/active. The HDD log root and storage guard remain configured. The original defaults file was backed up on the HDD at:

`/mnt/droneup/onboard_20261107/thoa/up4000_migration_20261107/network_backups/logger-defaults-20261108_031116.before`

## Verification

The laptop received actual data from `100.123.216.14` over Tailscale using New_GUI's status listener and telemetry parser. In a 30-second receive test:

- 870 status datagrams, maximum 1,104 bytes, zero malformed status packets.
- 14 PX4 message types, PX4 connected.
- 3,091 UWB RangeBurst messages (103.03/s), plus diagnostic and command acknowledgement messages.
- One telemetry CRC error during this interval; upstream UART parser reported five cumulative CRC errors. This test does not establish a network packet-loss percentage.
- Active session: `/mnt/droneup/onboard_20261107/thoa/flight_logs/20261108_031647`.

See `receive_test.json` and `last_state.json` for captured results. The first receive attempt was blocked by the command sandbox; the same test passed with network access allowed. No firewall changes were needed.

New_GUI was then opened normally with `--port udp:15555`, and its **Connect UART** button was clicked. The GUI showed **TAG Online · WiFi 100.123.216.14**. The **5. UP4000 Onboard** tab showed **ONLINE**, hostname `thoa`, and live PX4 attitude, position and estimator data. The GUI was left open and connected. No flight-control buttons or configuration commands were used.

The positioning page showed zero reachable anchors during this UI check. Receiving TAG telemetry alone does not establish that usable anchor ranging or a 3D position fix is available. The onboard counters also showed existing UART CRC errors and four ULog lost chunks; these were not resolved as part of establishing the GUI connection.

## Daily connection

1. Attach the HDD before powering UP7000, and attach TAG/Pixhawk as needed.
2. Connect UP7000 and the laptop to Internet access. They may use different networks.
3. Keep both devices online in the same Tailscale network. This configuration targets laptop Tailscale IP `100.77.150.59` even when using the same LAN.
4. Open New_GUI, choose **WiFi UDP :15555 (UP4000 relay)**, then click **Connect UART**. Despite its label, this button connects via UDP when that port is selected.
5. Select **5. UP4000 Onboard** to view UP7000 system and PX4 data; its label is shared with the UP4000 implementation.

No routine SSH command or service restart is required. Run only one GUI/receiver on these UDP ports at a time. UP4000 was offline and was not changed; cross-network operation on UP4000 would require its own target configuration when it becomes reachable.
