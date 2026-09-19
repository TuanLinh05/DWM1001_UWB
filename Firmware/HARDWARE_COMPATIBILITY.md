# Đối chiếu firmware với RangingSystemClassic

Tài liệu này đối chiếu firmware với thiết kế phần cứng
`RangingSystemClassic_2026-08-19`.

## DWM1001C và DW1000 nội bộ

Firmware Zephyr dùng board `decawave_dwm1001_dev/nrf52832`. Mapping cố định của
DW1000 bên trong module:

| Chức năng | nRF52832 | Zephyr |
|---|---:|---|
| DW1000 SCK | P0.16 | SPI2 SCK |
| DW1000 MOSI | P0.20 | SPI2 MOSI |
| DW1000 MISO | P0.18 | SPI2 MISO |
| DW1000 CS | P0.17 | SPI2 CS0, active-low |
| DW1000 IRQ | P0.19 | GPIO input |
| DW1000 RESET | P0.24 | GPIO, active-low |

Driver ứng dụng lấy node `ieee802154` của board để dùng chính xác bus này; không
dùng các chân SPI ngoài của DWM1001C.

## Các kết nối trên PCB

| Tín hiệu PCB | DWM1001C / nRF | ESP32-C3 | Cách dùng hiện tại |
|---|---|---:|---|
| LED3 | IO12 / P0.12 | - | LED trạng thái active-high |
| UART_TX | pin 20 / P0.05 | GPIO20 RX | Telemetry Tag, 115200 8N1 |
| UART_RX | pin 18 / P0.11 | GPIO21 TX | Dành cho lệnh điều khiển sau này |
| RDY | pin 19 / P0.26 | GPIO10 | Dự phòng handshake/IRQ; ESP đặt input |
| SPI MISO | pin 26 / P0.07 | GPIO4 | Chưa sử dụng |
| SPI MOSI | pin 27 / P0.06 | GPIO5 | Chưa sử dụng |
| SPI CLK | pin 28 / P0.04 | GPIO6 | Chưa sử dụng |
| SPI CS | pin 29 / P0.03 | GPIO7 | Chưa sử dụng |

Zephyr board mặc định bật SPI1 trên P0.03/P0.04/P0.06/P0.07. Overlay của cả Tag
và Anchor tắt peripheral này để tránh drive các đường đang nối sang ESP32-C3.

Anchor không cần gửi telemetry nên overlay tắt UART0. Tag giữ UART0 để gửi packet
binary. ESP gateway có parser streaming, không giả định mỗi lần đọc UART chứa
đúng một packet.

## Các sửa lỗi DW1000 quan trọng

- Ghi/xóa đủ 5 byte của `SYS_STATUS`.
- Sửa `TXPUTE` thành bit 34 thay vì bit 26.
- Bổ sung `LDEERR`, `RXOVRR`, `RXSFDTO` và `AFFREJ` vào mask lỗi RX.
- Bật interrupt cho các lỗi RX có thể phục hồi để state machine không phải chờ
  hết software timeout.
- Dùng đơn vị thời gian DW1000 chính xác và tốc độ truyền trong không khí nhất
  quán cho phép tính DS-TWR.

## Trình tự bring-up đề nghị

1. Dán nhãn module A1..A8 trước khi flash đúng project `Anchor_N` tương ứng;
   kiểm tra log khởi động và LED P0.12 từng module.
2. Flash Tag với `TELEM_ASCII=1`, đo từng Anchor ở vài khoảng cách cố định và kiểm
   tra `raw`, status, timeout, RX error và FPP.
3. Hiệu chuẩn TX/RX antenna delay theo từng module hoặc theo từng lô module.
4. Ghi offset DS-TWR đo được vào `uwb_app_config.h`, sau đó mới bật bit tương ứng
   trong `UWB_DS_CALIBRATED_MASK`.
5. Chuyển `TELEM_ASCII=0`, flash `ESP32C3_Gateway` và xác nhận INFO/RANGE/STATS
   cùng CRC trên UART.
6. Bring-up theo thứ tự A1, A1+A2, bốn Anchor 2D, rồi đủ tám Anchor 3D; ở mỗi
   bước phải xác nhận không trùng short address và không có cycle overrun.
7. Chỉ sau khi range ổn định mới đưa localization và dữ liệu vào flight
   controller.

Build phần mềm không thay thế phép thử RF. Những mục chưa được xác nhận trên mạch
thật gồm antenna delay/offset riêng của A1..A8, tải DS-TWR tám node trong môi
trường thực, độ lệch khoảng cách, chất lượng nguồn/antenna, RDY handshake và
thông lượng UART trong điều kiện bay.
