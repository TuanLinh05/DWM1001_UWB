# UART quality / calibration tester package

[UWB_tester_quality_20261025.zip](UWB_tester_quality_20261025.zip) contains
the verified TAG image, Windows GUI, patched ImGui calibration-console source,
UART/ROS adapter, protocol documentation and synthetic validation fixtures.
Extract the archive and follow its README. The Windows GUI is in `windows/`;
the Ubuntu GUI source is in `IMGUI_UWB_CALIB/`.

- TAG configuration: `0xb5f8839d`; existing 300-profile/PHY64 anchors remain compatible.
- ZIP SHA256: `f93c3f1572579f5e5a5067f877c79bac6d60581db553e8e711a5159f98508726`.
- TAG SHA256: `bd1b1571d9474f1b2e1835dde0ccf6d1b4405c8ceaf17dbe00ba5e7ab8f337bf`.
- Windows EXE SHA256: `7c8605df2d37476862e71059a327ecaa0e158ea9c922b9235af978076cc4f96a`.

All nine current HEX images and their manifest are in
[`Firmware/release/highrate_300`](../../../Firmware/release/highrate_300).
The portable source integration guide is
[`TESTER_QUALITY_20261025.md`](../ros2/TESTER_QUALITY_20261025.md).

The prebuilt files were generated and checked before the source commit.
DEVICE_INFO therefore retains the earlier base revision plus its dirty flag;
the config ID, per-file checksums and validation receipts identify these images.
Hardware throughput after adding quality telemetry still needs a board log.
