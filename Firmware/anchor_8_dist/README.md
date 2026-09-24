# Bộ HEX cho tám anchor vật lý

Chọn file đúng nhãn A1–A8 theo `manifest.csv` (gồm địa chỉ UWB, MCU,
công cụ nạp và SHA-256). A1–A4 là DWM1001C/nRF52832, nạp bằng
J-Link/OpenOCD. A5–A8 là STM32F103C8T6 + DW1000, nạp bằng ST-LINK
Utility với Program & Verify, BOOT0 = 0.

Nguồn A5–A8: `../STM32_Anchor_5` … `../STM32_Anchor_8`.
Build lại: `../scripts/build_stm32_anchors.ps1`.
Gom lại bộ tám file: `../scripts/package_8_anchors.ps1`.
Quy trình kiểm phần cứng và calibration: `../ANCHOR_DEPLOYMENT.md`.

Ảnh STM32 đã build và kiểm Intel HEX nhưng chưa được nạp/đo trên board thật.
