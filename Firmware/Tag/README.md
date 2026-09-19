# DWM1001 TAG

Project Zephyr độc lập dành riêng cho TAG trên DWM1001/nRF52832. TAG thăm dò
tuần tự tám Anchor `0x0001..0x0008` bằng chu kỳ 20 ms (50 Hz) và phát một record
telemetry cho mỗi Anchor.

## Module

```text
src/
├── main.c                         # Khởi tạo và vòng lặp TAG
├── drivers/dw1000.c              # Truy cập register DW1000
├── platform/uwb_platform_zephyr.c# SPI/GPIO/IRQ/timer Zephyr
├── ranging/tag_ranging.c         # State machine DS-TWR/SS fallback
├── filters/range_filter.c        # Lọc và ổn định khoảng cách
└── telemetry/                    # Đóng gói và truyền UART
```

Địa chỉ TAG và chế độ telemetry nằm trong `include/uwb_app_config.h`. Đổi
`TELEM_ASCII` thành `1` khi cần CSV để bring-up; mặc định dùng packet binary
tương thích `ESP32C3_Gateway`.

`app.overlay` ánh xạ LED trạng thái sang P0.12 active-high, giữ UART0 cho đường
DWM1001C -> ESP32-C3 và tắt SPI1 ngoài. DW1000 nội bộ vẫn dùng SPI2 của module.

Firmware cố ý chưa đánh dấu bất kỳ Anchor nào đã calibration. Raw range vẫn được
gửi để đo, nhưng trường `valid` chỉ được bật sau khi cấu hình offset/antenna delay
và bit A1..A8 tương ứng trong `UWB_DS_CALIBRATED_MASK` cho đúng bộ phần cứng.

Chu kỳ đầy đủ A1..A8 chạy ở 50 Hz theo yêu cầu của hệ thống. Đây là 400 phép đo
Anchor/giây khi cả tám link phản hồi. Phải theo dõi `tag_cycle_duration_max_us`,
`cycle_overrun_count` và timeout trên đủ tám module thật; nếu có overrun thì
tối ưu radio slot/PHY thay vì âm thầm giảm tần số xuất dữ liệu.

Một frame RANGE tám Anchor dài 145 byte. Ở 50 Hz, luồng chính dùng khoảng
72,5 kbit/s trên UART 115200 8N1; compile-time assertion giữ mức này dưới 80%
băng thông đường truyền để còn chỗ cho INFO/STATS và jitter của ring buffer.

## Build và flash

Mở terminal đã kích hoạt nRF Connect SDK v3.4.0, sau đó từ thư mục gốc
repository:

```powershell
Set-Location .\Firmware\Tag
.\scripts\build.ps1
.\scripts\flash.ps1
```

Script dùng ổ `U:` tạm thời để tránh lỗi Zephyr/Kconfig khi đường dẫn project có
khoảng trắng và tự gỡ ánh xạ sau khi chạy.

Xem thêm `../HARDWARE_COMPATIBILITY.md` và `../ANCHOR_DEPLOYMENT.md` trước khi
flash lên PCB.
