# ESP32-C3 UWB telemetry gateway

Firmware ESP-IDF dành cho ESP32-C3 Zero trên PCB `RangingSystemClassic`.

- UART1 RX GPIO20 nhận UART_TX từ DWM1001C.
- UART1 TX GPIO21 nối UART_RX của DWM1001C.
- GPIO10 là `ESP_IRQ`, nối với DWM1001C pin 19 (`nRF P0.26/RDY`), vì vậy không
  được dùng GPIO10 làm LED output như project `ESP_Test` trong repository mẫu.
- Parser hỗ trợ stream phân mảnh/gộp packet, giới hạn length, version và
  CRC16-CCITT của telemetry binary từ TAG.
- Mỗi packet hợp lệ được đóng lại nguyên frame và chuyển sang cổng USB
  Serial/JTAG để GUI Windows đọc trực tiếp. USB là luồng **binary-only**;
  `ESP_LOGx`, `printf` và console đã tắt để không chen byte chữ vào telemetry.
- SPI ngoài qua GPIO4/5/6/7 chưa sử dụng. Phía nRF đã tắt `spi1` để tránh tranh
  chấp bus qua R9-R12.

## Build

```powershell
idf.py set-target esp32c3
idf.py build
idf.py flash
```

Sau khi flash, đóng mọi cửa sổ monitor đang giữ cổng COM. Mở
`Software\UWB_UART_GUI\run_gui.ps1` và chọn COM của ESP32-C3. USB Serial/JTAG
bỏ qua baud phía PC, nên giữ baud mặc định của GUI (460800) hay chọn 115200
đều được. Không dùng `idf.py monitor` đồng thời với GUI.

ESP-IDF chưa được cài trong môi trường hiện tại nên project này cần được build
xác nhận sau khi cài toolchain. Không cần ESP32-C3 để Anchor thực hiện ranging;
ESP gateway chủ yếu nhận telemetry từ TAG.

## Test parser không cần ESP-IDF

Nếu `gcc` có trong `PATH`:

```powershell
.\tests\run_host_test.ps1
```

Test bao phủ encode/decode, dữ liệu nhiễu trước SOF, packet bị phân mảnh, CRC
sai, payload quá lớn và packet RANGE đủ tám Anchor (129-byte payload,
145-byte frame). Binary test sinh ra trong thư mục tạm của Windows.

## Lưu ý băng thông và debug

UART DWM1001C vẫn chạy 115200 8N1 trên GPIO20/21. Gateway dành riêng USB TX cho
telemetry và dùng buffer 4096 byte. Nếu PC không đọc kịp và buffer USB đầy,
gateway bỏ cả frame thay vì chặn vòng nhận UART; GUI phát hiện khoảng trống qua
sequence counter. Các biến `s_usb_forwarded_frames` và `s_usb_dropped_frames`
có thể xem bằng debugger ESP32 nếu cần chẩn đoán gateway.
