# HDD USB2 change — UP7000 — 2026-11-08

Changed the existing HDD connection from USB3 SuperSpeed **5000 Mb/s** at
`2-1` to USB2 High Speed **480 Mb/s** at `1-1`, without changing cable,
formatting, filesystem UUID or log locations. HDD bridge: `152d:0578`, serial
`DD564198838A2`; filesystem UUID
`896b17fa-8fe0-4111-853b-57b93ec87c60`.

## Applied configuration

The tested USB controller is Intel `8086:54ed`, PCI `0000:00:14.0`. Only its
physical connector port 1 is restricted: the USB3 logical port is disabled
and its USB2 peer remains enabled. WiFi on USB2 port 3 remains connected.
The configuration applies to the **current HDD socket**. Moving HDD to
another socket requires updating/verifying the selected port pair.

Installed small OS startup files:

- `/usr/local/libexec/up7000-hdd-usb2`
- `/etc/systemd/system/up7000-hdd-usb2.service` (enabled)
- `/etc/systemd/system/mnt-droneup.mount.d/usb2.conf`

The startup service runs before both the HDD filesystem check and HDD mount.
It verifies controller/device identity and refuses to disconnect a mounted
filesystem or a disk undergoing fsck. If a connected USB3 disk does not return
at USB2 speed, the helper restores its original port setting and fails.
An empty HDD connector is restricted before later attachment. The initial
experiment also used an independent 45-second rollback watcher in RAM.

Source copies are in `UP4000/scripts/up7000-hdd-usb2.py`,
`up7000-hdd-usb2.service`, and `mnt-droneup-usb2.conf`. Runtime data, source
copies, logs and validation files stay on HDD. No PX4 parameter was changed.

## Verification

- Initial transition used the exact expected HDD while it was unmounted and
  all three onboard services were stopped. UUID and serial remained correct.
- USB topology confirmed `480M`; SuperSpeed port disable readback is `1` and
  its USB2 peer remains enabled (`0`). Driver remains UAS, which does not imply
  SuperSpeed; negotiated speed is confirmed separately.
- systemd unit validation passed using the generated mount unit path. The
  dependency graph explicitly places USB2 policy before the correct fsck
  instance as well as mount. A pre-existing snapd `RestartMode` warning was
  unrelated to these units.
- HDD remounted at `/mnt/droneup`, UUID checked before writing.
- 32 MiB write/fsync/read test passed SHA-256 comparison: **32.93 MiB/s write,
  39.11 MiB/s read**. File cache was advised to discard the test data before
  reading. The temporary binary was removed; result JSON remains on HDD.
- `uwb-px4-logger`, `uwb-eskf-recorder`, `uwb-recording-control`, mount and USB2
  policy all active. Native MCAP/ESKF subscriptions and heartbeat verified;
  autonomous session remains on HDD.
- Directly running the installed helper again on the mounted USB2 HDD
  reported unchanged and did not disconnect it. No full reboot was performed.

Evidence: `usb2_io_validation.json`, `usb2_final_status.json`.

## GPS verification still required

Pixhawk and TAG were not connected at final verification. GPS improvement
therefore has **not** been measured after this change. Compare UP on/off at a
fixed outdoor GPS position, including a real recording session, using
satellite count, C/N0, fix and estimated accuracy. USB2 reduces one possible
noise source; it is not a guarantee that all HDD/bridge/power noise is removed.

## Reverting

Do not enable SuperSpeed on a mounted/in-use HDD. To return to USB3, first
close logs, stop the three onboard services, unmount HDD, remove the mount
drop-in and disable the USB2 policy, then reboot or re-enumerate the connector
with both logical ports enabled. Keep the source/UUID data unchanged.

Kernel references:

- https://docs.kernel.org/6.8/driver-api/usb/power-management.html
- https://github.com/torvalds/linux/blob/v6.8/Documentation/ABI/testing/sysfs-bus-usb
