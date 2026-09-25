# Firmware DWM1001C Ranging System

Hệ định vị UWB cho drone: một TAG DWM1001C đo DS-TWR lần lượt với tám anchor,
đẩy telemetry nhị phân qua UART → gateway ESP32-C3 → PC/UP 7000. Firmware chỉ
**đo**; bộ ước lượng vị trí chạy trên host (ADR-001 trong `Plan/`).

> **Phần cứng hiện có (xác nhận 2026-09-23):** A1–A4 là DWM1001C; A5–A8 là
> STM32F103 + DW1000. A5–A8 nay có project Zephyr `STM32_Anchor_5..8` dùng
> giao thức v2 và HEX cho ST-LINK Utility. Các project `Anchor_5..Anchor_8`
> cũ vẫn là target nRF52832. Bản STM32 đã build và kiểm file; còn cần thử
> trên board thật. Xem `stm32_anchor/README.md`.

## 1. Bố cục

```
Firmware/
├── common/                  ← toàn bộ mã dùng chung (nguồn duy nhất)
│   ├── include/             ← header chung
│   ├── src/                 ← drivers, ranging, app, telemetry, filters, platform
│   └── cmake/uwb_node.cmake ← uwb_node_setup(TAG|ANCHOR|SNIFFER) + build id
├── Tag/                     ← DWM1001C trên PCB RangingSystemClassic
├── Tag_DevKit/              ← cùng vai trò TAG, chạy trên DWM1001-DEV (J-Link)
├── Anchor_1 … Anchor_8/     ← tám responder, short address 0x0001..0x0008
├── STM32_Anchor_5 … 8/      ← bốn responder cho board STM32F103 vật lý
├── stm32_anchor/dist/       ← HEX STM32 A5–A8 đã kiểm tra
├── anchor_8_dist/           ← bộ tám HEX theo đúng board vật lý + manifest
├── Sniffer_DevKit/          ← chỉ nghe, đẩy mọi frame kèm timestamp DW1000
├── ESP32C3_Gateway/         ← ESP-IDF gateway UART ↔ USB
├── tests/                   ← test host (gcc + mô phỏng thanh ghi DW1000)
├── tools/uwb_sniffer.py     ← dựng timeline khe thời gian từ sniffer
└── scripts/                 ← build_all, build_stm32_anchors, package_8_anchors
```

Mỗi project nRF node chỉ còn `CMakeLists.txt`, `prj.conf`, `app.overlay`,
`include/uwb_app_config.h` và `scripts/`. Bốn project STM32 giữ bốn file
cấu hình tương ứng và cũng dùng mã ở `common/`. Các project nRF không giữ
bản sao mã nguồn chung — `tests/test_node_projects.py` sẽ báo lỗi nếu có.

## 2. Build

Yêu cầu nRF Connect SDK v3.4.0, board Zephyr `decawave_dwm1001_dev/nrf52832`.

```powershell
Set-Location .\Firmware
.\scripts\build_all.ps1                       # cả 11 project
.\scripts\build_all.ps1 -Projects Tag,Anchor_1  # chỉ một phần
```

File để flash: `<Project>/build/zephyr/zephyr.hex`.
Mỗi project cũng có `scripts/build.ps1` và `scripts/flash.ps1` riêng.

Với bốn board STM32 vật lý, chạy `scripts/build_stm32_anchors.ps1`, sau đó
`python tools/verify_stm32_images.py`. Chạy `scripts/package_8_anchors.ps1`
để chép tám ảnh tương ứng board vật lý vào `anchor_8_dist/` cùng SHA-256.
A1–A4 nạp qua J-Link/OpenOCD; A5–A8 nạp qua ST-LINK Utility.

Firmware mang theo git hash, cờ dirty (kể cả file untracked) và hash cấu hình
riêng của node trong `DEVICE_INFO`, nên có thể đối chiếu source lẫn cấu hình
đã tạo ra binary.

> **Thứ tự flash bắt buộc: tám anchor trước, TAG sau.** Khung tin trên không
> trung là v2; anchor v2 hiểu cả v1 lẫn v2, nhưng anchor v1 không hiểu POLL v2.
> Chi tiết ở `CHANGELOG.md` §1.

## 3. Kiểm thử trên PC (bắt buộc trước khi flash)

```powershell
Set-Location .\Firmware
.\tests\run_host_tests.ps1
```

Cần `gcc` (MinGW-w64) và Python 3. 13 nhóm test, không cần phần cứng:

| Test | Nội dung |
|---|---|
| `test_driver` | SPI một giao dịch, phân loại RX, OTP, TX power, chẩn đoán |
| `test_uwb_frame` | mã hoá/giải mã khung v1 & v2, kiểm tra biên TLV |
| `test_tag_state` | máy trạng thái TAG thật chạy trên mô phỏng DW1000: DS-TWR, txn, F1, fallback, mask, pause |
| `test_anchor_state` | responder thật: v1/v2, WAIT4RESP, lỗi RX, timeout |
| `test_cmd_parser` | CRC, tái đồng bộ, gói phân mảnh, giới hạn độ dài |
| `test_cmd_executor` | ACK, quy tắc LOCK/PAUSE, tác động lên radio |
| `test_settings` | snapshot NVS nguyên khối, CRC, PARTID/profile binding, lỗi factory reset |
| `test_telemetry_golden` (C + Python) | bộ mã hoá C đối chiếu chéo bộ giải mã Python |
| `test_node_projects.py` | cấu trúc 11 project, địa chỉ anchor, mask fail-closed |
| `test_sniffer_tool.py` | công cụ sniffer giải mã đúng timeline |
| `gui_python_tests` | 48 test của GUI/giao thức, gồm heartbeat host→TAG |
| `gateway_parser` | bộ phân tích khung của gateway ESP32-C3 |

Test chạy **chính mã sản phẩm** trên `tests/dw1000_sim.c` (mô phỏng mức thanh
ghi), nên máy trạng thái và công thức DS-TWR được kiểm chứng trước khi nạp board.

## 4. Cấu hình 8 Anchor

Bảng dưới là cấu hình của tám **project nRF/Zephyr** cũ. Trên phần cứng hiện
tại chỉ A1–A4 dùng các project này; A5–A8 dùng `STM32_Anchor_5..8`.

| Project | Short address | Nhịp cập nhật |
|---|---:|---:|
| Anchor_1 … Anchor_8 | `0x0001` … `0x0008` | 20 ms / 50 Hz (chu kỳ đủ 8 anchor) |

TAG probe xoay vòng các anchor mất kết nối (F1), nên một module hỏng không chiếm
hết thời gian chờ của chu kỳ. Ngân sách thời gian thực tế ở 50 Hz **phải đo bằng
sniffer** trước khi chốt (xem `HARDWARE_AB_CHECKLIST.md` bước 2).

## 5. Telemetry và lệnh

Khung: `AA 55 | VER=1 | TYPE | LEN | SEQ | TIME | payload | CRC16-CCITT`.

| Type | Gói | Nhịp |
|---|---|---|
| 0x00 / 0x01 / 0x02 | INFO / RANGE / STATS | 1 Hz / mỗi chu kỳ / 1 Hz |
| 0x10 | RANGE_MEAS (từng phép đo) | cần UART ≥ 460800 baud; Tag_DevKit (1 Mbaud) bật mặc định, Tag (115200) tắt |
| 0x11 / 0x12 | DIAG_ANCHOR / ANCHOR_INFO | 2 anchor/s / khi nhận TLV |
| 0x13 | CMD_ACK | trả lời lệnh |
| 0x14 | SNIFFER_FRAME | chỉ vai trò sniffer |
| 0x15 / 0x16 | DIAG_SYSTEM / DEVICE_INFO | 1 Hz / boot + 10 s |
| 0x17 | GATEWAY_HEALTH | ESP32-C3 phát, 1 Hz |
| 0x20 | CMD (host → TAG) | khi gửi |

Kênh lệnh có 15 lệnh (`uwb_cmd.h`). Lệnh đổi RF chỉ chạy khi TAG đã `PAUSE` và
chưa `LOCK`; đổi sang giá trị RF mới tự xoá cờ calibration. Cấu hình được lưu
trong một snapshot NVS schema v2 có CRC + generation; calibration chỉ được nạp
khi PARTID và RF/PHY profile ID khớp.

```powershell
Set-Location ..\Software\UWB_UART_GUI
py -3.12 uwb_command.py --port COM7 info
py -3.12 uwb_command.py --port COM7 set-cal 1 12.5 --save
```

## 6. Cờ RF — mặc định giữ nguyên hành vi cũ

| Cờ | Mặc định | Ý nghĩa khi bật |
|---|---|---|
| `UWB_DW_REFERENCE_TUNING` | `0` | LDE NTM = 13, LDOTUNE + crystal trim từ OTP |
| `UWB_TX_POWER_MODE` | `LEGACY` | REFERENCE / SMART / CUSTOM theo DW1000 UM |
| `UWB_USE_CLOCK_CORRECTION` | `0` | bù trôi clock cho SS-TWR (hệ số đã sửa dấu) |
| `UWB_FILTER_FPP_COMPAT_DB` | `-12.04f` | giữ thang FPP cũ cho bộ lọc sau khi sửa lỗi RXPACC |
| `UWB_DS_CALIBRATED_MASK` | `0` | chỉ mở sau khi hiệu chuẩn từng anchor |

Đổi bất kỳ cờ nào ở ba dòng đầu đều làm lệch bias ⇒ phải hiệu chuẩn lại.
Quy trình A/B và tiêu chí nghiệm thu: `HARDWARE_AB_CHECKLIST.md`.

## 7. Tương thích phần cứng

- DW1000 dùng SPI2 và chân cố định của module DWM1001C; LED trạng thái P0.12
  (active-high) tắt khi không có peer, sáng trong 1 s sau exchange thành công.
- TAG DevKit dùng D9 xanh cho liên kết UWB, D8 đỏ cho fault và D11 xanh dương
  cho heartbeat/lệnh GUI.
- TAG (PCB) dùng UART0 115200 baud sang ESP32-C3; anchor tắt UART0.
- Tag_DevKit dùng UARTE 1 Mbaud qua J-Link VCOM để mang RANGE_MEAS; GUI và
  `uwb_command.py` mặc định 1000000.
- SPI1 ngoài tắt ở mọi vai trò để không chạm các đường R9–R12 sang ESP32-C3.
- ESP GPIO10 là đầu vào `ESP_IRQ/RDY`, không dùng làm LED.
- Sniffer_DevKit dùng UARTE 1 Mbaud (DWM1001-DEV, J-Link VCOM).

Chi tiết chân và quyết định thiết kế: `HARDWARE_COMPATIBILITY.md`.

## 8. Calibration — vẫn fail-closed

Antenna delay/offset của hệ STM32 + DW1000 rời **không** dùng lại được cho
DWM1001C. Firmware dùng hardware antenna delay, tắt legacy offset, và
`UWB_DS_CALIBRATED_MASK = 0`: TAG vẫn phát đủ chẩn đoán nhưng đánh dấu range là
chưa hợp lệ. Bảng calibration nay sửa được lúc chạy bằng lệnh `SET_DS_CAL` và
lưu vào NVS, không cần build lại. Ba key schema cũ (`uwb/radio`, `uwb/cal`,
`uwb/tag`) bị bỏ qua fail-closed; `FACTORY_RESET` sẽ dọn cả schema cũ và mới.

Không bật mask và không đưa range vào bộ điều khiển bay trước khi hoàn tất
`HARDWARE_AB_CHECKLIST.md`.

## 9. Tài liệu liên quan

| File | Nội dung |
|---|---|
| `CHANGELOG.md` | nội dung bản nâng cấp v2, thứ tự flash, tương thích |
| `HARDWARE_AB_CHECKLIST.md` | quy trình A/B trên board thật và tiêu chí nghiệm thu |
| `ANCHOR_DEPLOYMENT.md` | gán module, flash, kiểm tra địa chỉ, calibration |
| `DEPLOYMENT_MANIFEST.md` | bảng truy vết module ↔ vai trò ↔ firmware đang nạp |
| `HARDWARE_COMPATIBILITY.md` | sơ đồ chân, quyết định thiết kế |
| `FIRMWARE_REVIEW_2026-09-15.md` | rà soát trước nâng cấp (F1–F10) |
| `../Plan/KE_HOACH_NANG_CAP_TOAN_DIEN_FIRMWARE_DWM1001.md` | kế hoạch GĐ0–GĐ6 |
