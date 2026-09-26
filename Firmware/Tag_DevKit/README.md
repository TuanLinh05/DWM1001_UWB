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
| LED liên kết UWB | D9 xanh, P0.30, active-low | Sáng khi vừa có phép đo UWB thành công; tắt sau 1 s không có phản hồi |
| LED lỗi | D8 đỏ, P0.14, active-low | Sáng khi radio đang ở fault/recovery |
| LED host | D11 xanh dương, P0.31, active-low | Sáng khi TAG đang nhận heartbeat/lệnh từ GUI |
| Telemetry UART TX/RX | P0.05/P0.11 | Đi qua J-Link Virtual COM khi J14/J15 ở trạng thái mặc định (closed) |
| Nạp/debug | J-Link tích hợp trên devkit | Dùng cùng cáp USB cấp nguồn |

SPI1 ngoài trên header Raspberry Pi được tắt vì firmware TAG không sử dụng.
UART0 chạy UARTE (EasyDMA) 460800 baud, 8N1, đủ cho `RANGE_MEAS` từng phép đo
(tám anchor khoảng 20 kB/s, 44 % dung lượng). Không dùng 1 Mbaud: J-Link VCOM
của devkit làm sai CRC khoảng 70 % byte ở tốc độ đó, kể cả khi đã tắt MSD.
Không cần ESP32-C3 để test board devkit.

Hệ UWB không có bước pair thủ công: TAG thăm dò các địa chỉ Anchor cố định
trong cùng PAN và Anchor chỉ trả lời POLL hợp lệ dành cho nó. Kết nối GUI là
đường UART riêng, không điều khiển việc TAG/Anchor bắt đầu ranging. LED xanh
biểu diễn link UWB; LED xanh dương biểu diễn heartbeat GUI nên hai trạng thái
này không còn bị nhầm với nhau.

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
2. Sau reset, khi chưa bật Anchor cả ba LED phải tắt. Khi có trao đổi UWB thành
   công, D9 xanh sáng liên tục và tắt khoảng 1 s sau khi mất toàn bộ Anchor.
   D8 đỏ báo lỗi radio; D11 xanh dương chỉ sáng khi GUI đang kết nối hai chiều.
3. Mở cổng COM do J-Link tạo ra ở 460800 8N1 (baud mặc định của GUI và
   `uwb_command.py`).
4. Mặc định `TELEM_ASCII=0`, dùng GUI/parser hiện tại để đọc packet binary. GUI
   gửi `PING` mỗi giây và tự bật `RANGE_SNAPSHOT` cho phiên hiện tại nếu cấu
   hình lưu trước đó đã tắt nó. Tab **RANGE_MEAS** hiển thị từng phép đo: tần số
   đo thật của mỗi anchor, bản ghi mất, nhiễu, FP/RX, chỉ báo NLOS, clock offset
   và thời gian slot.
5. Firmware bật sẵn `RANGE_MEAS`. Nếu TAG từng `SAVE_SETTINGS` với firmware cũ
   (115200), feature đã lưu không có RANGE_MEAS: GUI tự bật cho phiên; muốn lưu
   hẳn thì chạy
   `py -3.12 uwb_command.py --port COMx set-telemetry --snapshot --meas --diag --save`.
6. Để xem trực tiếp bằng terminal, đặt `TELEM_ASCII=1` trong
   `include/uwb_app_config.h`, build và flash lại, mở terminal ở 460800 baud;
   khi test xong đổi về `0` để tương thích gateway/GUI binary.
7. Bật lần lượt A1, A1+A2 rồi đủ tám Anchor; kiểm tra timeout, lỗi SPI, tần số
   chu kỳ và UART overflow trước khi chạy lâu.

`UWB_DS_CALIBRATED_MASK` cố ý bằng `0`. Range thô vẫn được gửi để bring-up,
nhưng trạng thái hợp lệ sẽ báo `CAL_MISSING` cho đến khi devkit được hiệu chuẩn.
Không sao chép offset/antenna delay đã đo trên PCB tự phát triển sang board này.

## Các file chỉ thay đổi cho devkit

- `app.overlay`: dùng D9/P0.30 và UARTE 460800 qua J-Link VCOM; tắt SPI1 ngoài.
- `scripts/build.ps1`: build project `Tag_DevKit`.
- `scripts/flash.ps1`: flash qua runner J-Link.
- `scripts/ncs_env.ps1`: tự nạp môi trường NCS/toolchain/J-Link cho task VS Code.
- `.vscode/`: task/debug dùng J-Link, không còn cấu hình ST-LINK/OpenOCD.
- `include/uwb_app_config.h`: ghi rõ profile hiệu chuẩn riêng cho devkit, bộ lọc
  `MEDIAN_GATE` và telemetry mặc định có `RANGE_MEAS`.

Các file driver, ranging, filter và telemetry còn lại là bản clone của TAG tại
thời điểm chuyển đổi và không được thay đổi về thuật toán.
