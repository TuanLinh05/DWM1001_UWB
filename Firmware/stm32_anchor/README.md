# Zephyr cho anchor STM32F103 A5–A8

Bốn project `STM32_Anchor_5` … `STM32_Anchor_8` dành cho board
**STM32F103C8T6 + DW1000** trong `D:\Drone Project\UWB Drone\STM32_UWB`.
Chúng dùng chung driver DW1000 và responder DS-TWR v2 ở `Firmware/common`
với TAG DevKit và A1–A4. ID ngắn lần lượt là `0x0005` … `0x0008`.

## File để nạp

| Board vật lý | File Intel HEX | Công cụ |
|---|---|---|
| A5 | `dist/anchor_5_stm32f103.hex` | ST-LINK Utility |
| A6 | `dist/anchor_6_stm32f103.hex` | ST-LINK Utility |
| A7 | `dist/anchor_7_stm32f103.hex` | ST-LINK Utility |
| A8 | `dist/anchor_8_stm32f103.hex` | ST-LINK Utility |

Các HEX bắt đầu ở `0x08000000`, không có bootloader offset. Mỗi ảnh có
26.096 byte dữ liệu flash; linker dùng 26.100/65.536 byte flash và
7.500/20.480 byte RAM. Chạy `python Firmware/tools/verify_stm32_images.py`
để kiểm checksum Intel HEX, vector khởi động, giới hạn flash, ID nguồn và
việc bốn ảnh khác nhau. **Build và kiểm file không thay thế thử nghiệm trên
board thật.**

## Chân và radio

| Tín hiệu | STM32 |
|---|---|
| DW1000 CS, SCK, MISO, MOSI | PA4, PA5, PA6, PA7 (SPI1, mode 0) |
| DW1000 RSTn, IRQ | PA2, PA3 (IRQ rising/high) |
| DW1000 EXTON, WAKE | PA1, PB0 (MCU input pull-down, như mã STM32 gốc) |
| LED trạng thái | PC13, active low |
| CPU | HSI/2 × 16 = 64 MHz; APB1 = 32 MHz |

Pinout này được đối chiếu với `STM32_UWB/Anchor/Core/Inc/dw1000_hw.h`,
`Core/Src/dw1000_hw.c` và cấu hình SPI1 trong `Core/Src/stm32f1xx_hal_msp.c`
của project gốc; bốn project Anchor gốc dùng cùng định nghĩa chân. Repo
`UWB Drone` hiện không có file sơ đồ mạch/PCB để xác nhận đường đồng trên
board vật lý. Vì vậy cần kiểm tra DW1000 Device ID và IRQ trên một board
sau khi nạp trước khi triển khai cả bốn board.

Overlay tắt USART2 mặc định của board Zephyr vì nó dùng PA2/PA3, và bỏ
chức năng SPI NSS trên PA4 để GPIO điều khiển CS. SPI bắt đầu ở 2 MHz,
sau khi nhận diện DW1000 chuyển 8 MHz. Watchdog IWDG đặt 1 giây.
EXTON PA1 và WAKE PB0 được cấu hình input pull-down giống mã CubeIDE cũ;
firmware không dùng chúng để quyết định trạng thái ranging.

## Build lại

Yêu cầu NCS v3.4.0 tại `E:\software\nordic\ncs\v3.4.0` hoặc đặt
`NCS_SDK_ROOT`/`NCS_TOOLCHAIN_ROOT`. Bản NCS của máy hiện tại thiếu
`hal_stm32`; đã cài bản Zephyr ở commit
`39130f29ae37c1db34095478ca02b6419b70dcdc` vào
`<NCS>\modules\hal\stm32`. Nếu build trên máy khác, cài đúng commit đó
vào đường dẫn này. Script thêm module bằng `ZEPHYR_EXTRA_MODULES` vì
manifest NCS không liệt kê STM32. Tránh đường dẫn module chứa khoảng trắng.

```powershell
& 'D:\Drone Project\UWB DW1001\Firmware\scripts\build_stm32_anchors.ps1'
python 'D:\Drone Project\UWB DW1001\Firmware\tools\verify_stm32_images.py'
```

Script build dùng ổ `U:` tạm thời vì đường dẫn project chứa khoảng trắng,
sau đó gỡ ổ này. Muốn build riêng: `-Anchors 5` hoặc `-Anchors 6,7,8`.
Ảnh mới được chép từ `STM32_Anchor_N/build/zephyr/zephyr.hex` vào `dist`.

## Nạp và kiểm trên board

1. Dán nhãn A5–A8 lên bốn board. Chỉ nối một board với ST-LINK trong mỗi lần
   nạp. Nối SWDIO, SWCLK, GND và 3,3 V đúng sơ đồ board; đặt BOOT0 = 0.
2. Trong ST-LINK Utility, kết nối target, mở đúng `anchor_N_stm32f103.hex`,
   chọn **Program & Verify**, rồi reset board. Intel HEX mang địa chỉ
   `0x08000000`; không nhập offset khác.
3. Bật TAG và chỉ một anchor vừa nạp. Kiểm TAG nhận đúng ID A5/A6/A7/A8,
   có RESPONSE/REPORT và raw range thay đổi khi di chuyển. Sau đó bật dần
   đến đủ tám anchor, kiểm timeout và chu kỳ bằng telemetry/sniffer.

LED PC13 (active-low) cho biết anchor đi được tới bước nào, không cần UART:

| LED | Nghĩa |
|---|---|
| 3 chớp nhanh ngay khi cấp nguồn/reset | Firmware đã chạy, LED nối đúng. Lặp lại liên tục = board tự reset (nguồn, watchdog) |
| Sáng liên tục | Có ít nhất một exchange DS-TWR hoàn chỉnh (REPORT đã phát) trong 1 s gần nhất |
| Nháy 1 Hz (500 ms sáng/500 ms tắt) | Nhận được POLL gửi cho anchor này nhưng exchange không hoàn tất: RESP trễ (HPDWARN), mất FINAL hoặc REPORT |
| Tắt, chớp ngắn mỗi 2 s | Radio chạy và đang nghe, nhưng chưa nhận POLL nào cho anchor này |
| Nháy nhanh 150 ms | Radio lỗi (không đọc được DW1000, PLL không khóa…), đang thử khởi tạo lại |
| Tắt hẳn, không có 3 chớp lúc cấp nguồn | Firmware không chạy: kiểm nạp (Program & Verify), BOOT0 = 0, nguồn 3,3 V |
4. A1–A4 là DWM1001C/nRF52832: dùng ảnh `Firmware/Anchor_1` …
   `Firmware/Anchor_4` và J-Link/OpenOCD, **không dùng ST-LINK Utility**.

TAG DevKit hiện giữ `UWB_DS_CALIBRATED_MASK=0U`: phép đo chưa hiệu chuẩn
được giữ làm chẩn đoán, còn cờ `valid` cho định vị là 0. Sau khi đo khoảng
cách chuẩn và lưu bias của từng board bằng `SET_DS_CAL`/`SAVE_SETTINGS`, mới
dùng range đó cho định vị. Xem `Firmware/ANCHOR_DEPLOYMENT.md`.
