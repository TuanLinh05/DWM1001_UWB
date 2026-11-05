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
py -3.12 -m unittest discover -s tests -p "test_*.py" -v
```
