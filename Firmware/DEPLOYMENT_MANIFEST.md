# Bảng truy vết module ↔ vai trò ↔ firmware

Điền bảng này **mỗi lần flash**. Mục đích: khi một anchor đo sai, biết ngay
module nào, đang chạy commit nào, hiệu chuẩn theo profile RF nào.

**Phần cứng thực tế:** A1–A4 là DWM1001C/nRF52832; A5–A8 là STM32F103C8T6
+ DW1000. Firmware Zephyr STM32 v2 đã build ở `STM32_Anchor_5..8`, với HEX
trong `stm32_anchor/dist/`; chưa xác nhận hoạt động trên board thật. Các
project `Anchor_5..Anchor_8` cũ đều là target nRF. SHA-256 của tám ảnh nạp
nằm trong `anchor_8_dist/manifest.csv`.

Cách lấy số liệu:

- **Serial / PARTID / LOTID**: đọc từ gói `DEVICE_INFO` (TYPE 0x16), hoặc
  `py -3.12 uwb_command.py --port COM7 info` với TAG.
- **Git hash**: `git rev-parse --short HEAD` lúc build — firmware phát lại đúng
  giá trị này trong `DEVICE_INFO`, dùng để đối chiếu.
- **Config hash**: lấy từ `DEVICE_INFO` của TAG hoặc `ANCHOR_INFO` của từng
  Anchor; giá trị phải khớp dòng `config 0x...` của lần build project đó.
- **SHA-256 file nạp**: `Get-FileHash <Project>\build\zephyr\zephyr.hex`.

## Lần flash hiện tại

| Nhãn dán | Project | ANCHOR_ADDR | PARTID | LOTID | Git hash | Config hash | Dirty | SHA-256 (8 ký tự đầu) | Ngày flash | Người flash |
|---|---|---:|---|---|---|---|---|---|---|---|
| TAG |  `Tag` | `0x0000` |  |  |  |  |  |  |  |  |
| A1 | `Anchor_1` | `0x0001` |  |  |  |  |  |  |  |  |
| A2 | `Anchor_2` | `0x0002` |  |  |  |  |  |  |  |  |
| A3 | `Anchor_3` | `0x0003` |  |  |  |  |  |  |  |  |
| A4 | `Anchor_4` | `0x0004` |  |  |  |  |  |  |  |  |
| A5 | `STM32_Anchor_5` | `0x0005` |  |  |  |  |  |  |  |  |
| A6 | `STM32_Anchor_6` | `0x0006` |  |  |  |  |  |  |  |  |
| A7 | `STM32_Anchor_7` | `0x0007` |  |  |  |  |  |  |  |  |
| A8 | `STM32_Anchor_8` | `0x0008` |  |  |  |  |  |  |  |  |
| SNIFFER | `Sniffer_DevKit` | — |  |  |  |  |  |  |  |  |
| GATEWAY | `ESP32C3_Gateway` | — |  |  |  |  |  |  |  |  |

## Cấu hình RF đang nạp

Ghi lại vì mọi giá trị dưới đây đều làm dịch bias khoảng cách:

Bảng mặc định dưới đây dùng driver DW1000 chung cho cả nRF và STM32. Antenna
delay là giá trị danh định; cần hiệu chuẩn lại cho từng board.

| Cờ | Giá trị lần flash này |
|---|---|
| `UWB_TX_POWER_MODE` | `LEGACY` |
| `UWB_DW_REFERENCE_TUNING` | `0` |
| `UWB_USE_CLOCK_CORRECTION` | `0` |
| `UWB_USE_DS_TWR` | `1` |
| `UWB_DS_CALIBRATED_MASK` | `0x00` |
| `UWB_TX_ANT_DLY` / `UWB_RX_ANT_DLY` | `16436` / `16436` |

Nếu đổi bất kỳ dòng nào: ghi lại giá trị mới, ngày đổi, và **hiệu chuẩn lại**
theo `HARDWARE_AB_CHECKLIST.md` bước 7.

## Lịch sử

| Ngày | Thay đổi | Lý do |
|---|---|---|
|  |  |  |
