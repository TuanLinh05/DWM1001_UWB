# DWM1001-DEV TAG

Đây là bản clone độc lập của `Firmware/Tag`, chuyển phần cứng ngoại vi từ PCB
`RangingSystemClassic` sang board phát triển DWM1001-DEV. Logic DS-TWR, bộ lọc,
định dạng telemetry và danh sách tám Anchor được giữ nguyên để có thể A/B test
hai loại phần cứng.

## Ánh xạ phần cứng devkit

| Chức năng | Kết nối DWM1001-DEV | Ghi chú |
|---|---|---|
| DW1000 SCK/MOSI/MISO/CS | SPI2: P0.16/P0.20/P0.18/P0.17 | Bus nội bộ module, lấy từ board devicetree |
| DW1000 IRQ | P0.19 | Bus nội bộ module |
| DW1000 RESET | P0.24, active-low | Bus nội bộ module |
| LED trạng thái | D9 xanh, P0.30, active-low | Alias `status-led` trỏ tới `led1_green` của board |
| Telemetry UART TX/RX | P0.05/P0.11 | Đi qua J-Link Virtual COM khi J14/J15 ở trạng thái mặc định (closed) |
| Nạp/debug | J-Link tích hợp trên devkit | Dùng cùng cáp USB cấp nguồn |

SPI1 ngoài trên header Raspberry Pi được tắt vì firmware TAG không sử dụng.
UART0 chạy 115200 baud, 8N1. Không cần ESP32-C3 để test board devkit.

## Build và flash

Yêu cầu nRF Connect SDK v3.4.0 và SEGGER J-Link. Script tự nạp môi trường SDK,
toolchain và J-Link đã cài trên máy, nên có thể chạy từ PowerShell hoặc task
VS Code thông thường. Kết nối cổng USB của DWM1001-DEV, sau đó chạy:

```powershell
Set-Location .\Firmware\Tag_DevKit
.\scripts\build.ps1
.\scripts\flash.ps1
```

Script build dùng board Zephyr `decawave_dwm1001_dev/nrf52832`. Script flash ép
runner `jlink` để sử dụng debugger tích hợp của devkit. File sinh ra để nạp là:

```text
Firmware/Tag_DevKit/build/zephyr/zephyr.hex
```

Có thể mở workspace `DWM1001-DEV-Tag.code-workspace` và dùng task
`DWM1001-DEV: Build Tag` hoặc `DWM1001-DEV: Flash Tag via on-board J-Link`.

`scripts/ncs_env.ps1` tự phát hiện SDK/toolchain/J-Link. Nếu chuyển bộ cài sang
ổ khác, đặt các biến `NCS_SDK_ROOT`, `NCS_TOOLCHAIN_ROOT` và `JLINK_ROOT` trước
khi chạy script.

## Kiểm tra nhanh

1. Chỉ cắm một DWM1001-DEV cần nạp và chạy build/flash.
2. Sau reset, LED xanh D9 phải chớp khi TAG chạy ranging. Nếu khởi tạo lỗi, LED
   sẽ chớp nhanh theo vòng `fatal_blink()`.
3. Mở cổng COM do J-Link tạo ra ở 115200 8N1.
4. Mặc định `TELEM_ASCII=0`, dùng GUI/parser hiện tại để đọc packet binary.
5. Để xem trực tiếp bằng terminal, đặt `TELEM_ASCII=1` trong
   `include/uwb_app_config.h`, build và flash lại; khi test xong đổi về `0` để
   tương thích gateway/GUI binary.
6. Bật lần lượt A1, A1+A2 rồi đủ tám Anchor; kiểm tra timeout, lỗi SPI, tần số
   chu kỳ và UART overflow trước khi chạy lâu.

`UWB_DS_CALIBRATED_MASK` cố ý bằng `0`. Range thô vẫn được gửi để bring-up,
nhưng trạng thái hợp lệ sẽ báo `CAL_MISSING` cho đến khi devkit được hiệu chuẩn.
Không sao chép offset/antenna delay đã đo trên PCB tự phát triển sang board này.

## Các file chỉ thay đổi cho devkit

- `app.overlay`: dùng D9/P0.30 và UART qua J-Link VCOM; tắt SPI1 ngoài.
- `scripts/build.ps1`: build project `Tag_DevKit`.
- `scripts/flash.ps1`: flash qua runner J-Link.
- `scripts/ncs_env.ps1`: tự nạp môi trường NCS/toolchain/J-Link cho task VS Code.
- `.vscode/`: task/debug dùng J-Link, không còn cấu hình ST-LINK/OpenOCD.
- `include/uwb_app_config.h`: ghi rõ profile hiệu chuẩn riêng cho devkit.

Các file driver, ranging, filter và telemetry còn lại là bản clone của TAG tại
thời điểm chuyển đổi và không được thay đổi về thuật toán.
