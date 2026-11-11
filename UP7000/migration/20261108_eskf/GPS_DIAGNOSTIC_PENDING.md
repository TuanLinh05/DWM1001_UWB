# GPS loss when connecting UP7000 — investigation pending live access

2026-11-08, Asia/Bangkok. User reports zero GPS satellites with UP7000
connected and recovery when unplugging. User confirmed direct USB UP7000 →
USB Pixhawk, not a USB-UART cable into GPS1/TELEM.

UP7000 100.123.216.14 is offline on Tailscale (last seen 07:00 UTC) and SSH
times out. Laptop Tailscale is running, 100.77.150.59. User was asked to bring
UP7000 back online. No runtime service or PX4 parameter was changed.

Read-only QGroundControl cache decoding using Qt QDataStream succeeded for
1,109 entries, with valid stream status and exact EOF. Cache timestamp is
14:10:14 local; relevant values are retained in
`gps_diagnostic_cached_parameters.json`. This is not a live controller read;
cache vehicle/system identity must be confirmed before drawing conclusions.

- GPS_1_CONFIG=201 (GPS1), GPS_2_CONFIG=0.
- MAV_0_CONFIG=101 (TELEM1); MAV_1_CONFIG=0; MAV_2_CONFIG=0.
- SYS_USB_AUTO=2, USB_MAV_MODE=2; UXRCE_DDS_CFG=0.
- No MAVLink/GPS1 collision is present in this cached mapping.

Current logger source opens the PX4 USB alias and requests MAVLink streams,
ULog and TIMESYNC. It does not itself write GPS/MAV serial-port parameters;
it does pass authorized ground-station MAVLink traffic through.

Next: read actual parameters/firmware and GPS status through the existing
logger forwarding path (avoid a second serial reader). Confirm disarmed bench
state before any service interruption or parameter changes. Determine whether
GPS driver is absent/stale or continues reporting fresh zero-satellite samples.
If necessary compare USB-connected logger-running/logger-stopped conditions,
keeping GPS placement and vehicle power unchanged. Inspect uORB GPS/driver
and MAVLink status, CPU/load and GNSS jamming metrics where available.
USB/USB3 interference and power/ground changes remain hypotheses; do not
assign the cause or change port mappings without measurements.

PX4 references:
- https://docs.px4.io/main/en/peripherals/serial_configuration
- https://docs.px4.io/main/en/advanced_config/parameter_reference#MAV_0_CONFIG
- https://docs.px4.io/main/en/log/flight_review#gps-noise-jamming

Cache format reference:
https://github.com/mavlink/qgroundcontrol/blob/master/src/FactSystem/ParameterManager.cc
