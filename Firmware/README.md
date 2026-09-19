# Firmware DWM1001C Ranging System

Các project trong thư mục này đã được tách theo đúng vai trò phần cứng của PCB
`RangingSystemClassic`:

- `Tag/`: DWM1001C chủ động DS-TWR tuần tự với A1..A8 và phát telemetry qua UART.
- `Tag_DevKit/`: bản clone TAG dành cho DWM1001-DEV, dùng LED D9, J-Link USB
  Virtual COM và J-Link tích hợp để nạp/debug; không cần ESP32-C3 khi test.
- `Anchor_1/` ... `Anchor_8/`: tám project DWM1001C responder độc lập, có
  short address duy nhất từ `0x0001` đến `0x0008`.
- `ESP32C3_Gateway/`: ESP-IDF gateway nhận, kiểm tra CRC và giải mã telemetry
  binary từ Tag.

`Firmware Code Base` vẫn là bản tham chiếu ban đầu. Các project trong `Firmware`
là nơi phát triển theo từng vai trò và theo PCB thực tế.

## Build DWM1001C

Yêu cầu nRF Connect SDK v3.4.0 và board Zephyr
`decawave_dwm1001_dev/nrf52832`.

Mở terminal đã kích hoạt nRF Connect SDK v3.4.0, sau đó từ thư mục gốc
repository:

```powershell
Set-Location .\Firmware
.\scripts\build_all.ps1
```

Script trên build `Tag` và toàn bộ tám Anchor. File để flash:

- `Tag/build/zephyr/zephyr.hex`
- `Anchor_N/build/zephyr/zephyr.hex`, với `N` từ 1 đến 8

Chạy kiểm thử host và kiểm tra tính nhất quán địa chỉ trước khi flash:

```powershell
Set-Location .\Firmware
.\tests\run_host_tests.ps1
```

Mỗi project Anchor có `scripts/build.ps1` và `scripts/flash.ps1` riêng. Không
flash hàng loạt khi chưa dán nhãn vật lý cho từng module.

## Cấu hình 8 Anchor

| Project | Short address | Chu kỳ TAG |
|---|---:|---:|
| Anchor_1 | `0x0001` | 20 ms / 50 Hz |
| Anchor_2 | `0x0002` | 20 ms / 50 Hz |
| Anchor_3 | `0x0003` | 20 ms / 50 Hz |
| Anchor_4 | `0x0004` | 20 ms / 50 Hz |
| Anchor_5 | `0x0005` | 20 ms / 50 Hz |
| Anchor_6 | `0x0006` | 20 ms / 50 Hz |
| Anchor_7 | `0x0007` | 20 ms / 50 Hz |
| Anchor_8 | `0x0008` | 20 ms / 50 Hz |

Chu kỳ đầy đủ A1..A8 là 20 ms: mỗi Anchor được cập nhật ở 50 Hz, tương đương tối
đa 400 phép đo thành công mỗi giây. TAG có backoff riêng cho Anchor mất kết nối
để một module lỗi không liên tục chiếm toàn bộ response timeout. Khi test đủ tám
module phải giám sát cycle overrun và thời gian slot thực tế.

## Trạng thái tương thích phần cứng

- DW1000 nội bộ dùng SPI2 và các chân cố định của module DWM1001C.
- LED trạng thái của PCB dùng nRF P0.12, active-high.
- Tag dùng UART0 115200 baud để truyền sang ESP32-C3.
- Anchor không dùng UART nên UART0 được tắt.
- SPI1 ngoài được tắt ở cả hai vai trò để không điều khiển các đường SPI nối qua
  R9-R12 sang ESP32-C3.
- ESP GPIO10 là đầu vào `ESP_IRQ/RDY`, không còn dùng làm LED.

Chi tiết chân, các quyết định thiết kế và trình tự kiểm thử nằm trong
`HARDWARE_COMPATIBILITY.md`.

## Lưu ý calibration bắt buộc

Các antenna delay và offset cũ của hệ STM32/DW1000 rời không thể tái sử dụng an
toàn cho DWM1001C/PCB mới. Firmware hiện dùng hardware antenna delay, tắt legacy
offset và đặt calibration mask bằng 0. Cả tám offset A1..A8 khởi tạo bằng 0;
TAG vẫn xuất raw diagnostic nhưng đánh dấu range là chưa hợp lệ cho đến khi từng
Anchor được hiệu chuẩn.

Không nên bật `UWB_DS_CALIBRATED_MASK` hoặc đưa range vào bộ điều khiển bay trước
khi hoàn tất đo calibration trên phần cứng thật.

Quy trình gán module, flash, kiểm tra địa chỉ và calibration nằm trong
`ANCHOR_DEPLOYMENT.md`.
