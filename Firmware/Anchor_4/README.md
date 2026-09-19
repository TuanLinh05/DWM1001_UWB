# DWM1001 Anchor 4

Project Zephyr độc lập dành riêng cho Anchor địa chỉ `0x0004` trên
DWM1001/nRF52832. Project này không bật UART telemetry, nhờ đó giảm phần driver
và RAM không cần thiết ở Anchor.

`app.overlay` ánh xạ LED trạng thái sang P0.12 active-high, đồng thời tắt UART0
và SPI1 ngoài. DW1000 nội bộ vẫn dùng SPI2 cố định của module DWM1001C.

## Module

```text
src/
├── main.c                          # Khởi tạo và vòng lặp Anchor
├── drivers/dw1000.c               # Truy cập register DW1000
├── platform/uwb_platform_zephyr.c # SPI/GPIO/IRQ/timer Zephyr
└── ranging/anchor_ranging.c        # State machine responder DS-TWR
```

Địa chỉ node được cố định rõ trong `include/uwb_app_config.h`:

```text
#define ANCHOR_ADDR ((uint16_t)4U)
```

## Build và flash

Mở terminal đã kích hoạt nRF Connect SDK v3.4.0, sau đó từ thư mục gốc
repository:

```powershell
Set-Location .\Firmware\Anchor_4
.\scripts\build.ps1
.\scripts\flash.ps1
```

Script dùng ổ `U:` tạm thời để tránh lỗi Zephyr/Kconfig khi đường dẫn project có
khoảng trắng và tự gỡ ánh xạ sau khi chạy.

Xem thêm `../HARDWARE_COMPATIBILITY.md` và hoàn tất calibration trước khi đánh
dấu range hợp lệ ở Tag.
