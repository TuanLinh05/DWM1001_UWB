# Firmware UWB cho DWM1001 / nRF52832 / Zephyr

Project này chuyển phần firmware ứng dụng từ STM32F103 + DW1000 sang MCU
nRF52832 nằm sẵn trong DWM1001. Thuật toán DS-TWR/SS fallback, lọc range,
calibration và định dạng telemetry được giữ tương thích với firmware cũ;
HAL STM32 được thay bằng API Zephyr và devicetree.

## Cấu trúc

```text
Firmware/
├── config/                 # Vai trò TAG, Anchor 1..4, hardware test
├── include/                # API và cấu hình dùng chung
├── scripts/build_all.ps1   # Build toàn bộ biến thể
└── src/
    ├── diagnostics/        # Kiểm tra SPI, device ID, register, nhiệt độ
    ├── drivers/            # Driver register-level DW1000
    ├── filters/            # Range conditioner/Kalman
    ├── platform/           # SPI/GPIO/IRQ/timer Zephyr cho DWM1001
    ├── ranging/            # State machine TAG và Anchor
    └── telemetry/          # Packet + UART TX không blocking
```

Các bản Anchor không còn có source riêng bị lặp. `CONFIG_UWB_ANCHOR_ADDRESS`
chọn địa chỉ 1..4 trong từng file `config/anchor_N.conf`.

## Phần cứng lấy từ devicetree của board

| Tín hiệu DW1000 | nRF52832 |
|---|---:|
| SPI SCK | P0.16 |
| SPI MOSI | P0.20 |
| SPI MISO | P0.18 |
| SPI CS | P0.17 |
| IRQ | P0.19 |
| RESETn | P0.24 |
| UART TX telemetry | P0.05 |
| UART RX | P0.11 |

SPI khởi tạo ở 2 MHz rồi chuyển sang 8 MHz sau khi nạp LDE. Không dùng 16 MHz
như bản STM32 vì SPI của nRF52832 và devicetree DWM1001 giới hạn ở 8 MHz.

## Chuẩn bị môi trường

Mở terminal đã kích hoạt nRF Connect SDK v3.4.0, sau đó từ thư mục gốc
repository:

```powershell
Set-Location '.\Firmware Code Base'
```

Project đã được kiểm tra với nRF Connect SDK v3.4.0 và board
`decawave_dwm1001_dev/nrf52832`.

## Build

TAG binary telemetry:

```powershell
west build --no-sysbuild --pristine=always `
  -b decawave_dwm1001_dev/nrf52832 `
  -d build/tag --extra-conf config/tag.conf .
```

TAG CSV để bring-up bằng serial monitor:

```powershell
west build --no-sysbuild --pristine=always `
  -b decawave_dwm1001_dev/nrf52832 `
  -d build/tag-ascii --extra-conf config/tag_ascii.conf .
```

Anchor 1 (đổi file cấu hình thành `anchor_2.conf`, `anchor_3.conf` hoặc
`anchor_4.conf` cho các node còn lại):

```powershell
west build --no-sysbuild --pristine=always `
  -b decawave_dwm1001_dev/nrf52832 `
  -d build/anchor-1 --extra-conf config/anchor_1.conf .
```

Build tất cả biến thể:

```powershell
.\scripts\build_all.ps1
```

`--no-sysbuild` là chủ ý: nó tránh lỗi Kconfig của SDK trên Windows khi project
nằm trong đường dẫn có khoảng trắng.

File để flash nằm tại `build/<variant>/zephyr/zephyr.hex`.

## Flash

Kết nối DWM1001-DEV bằng J-Link rồi dùng đúng thư mục build:

```powershell
west flash -d build/tag
# hoặc
west flash -d build/anchor-1
```

Không flash nhầm firmware TAG vào Anchor. Firmware Anchor tự lấy short address
từ file cấu hình đã dùng lúc build.

## Hardware test trước khi chạy ranging

```powershell
west build --no-sysbuild --pristine=always `
  -b decawave_dwm1001_dev/nrf52832 `
  -d build/hardware-test --extra-conf config/hardware_test.conf .
west flash -d build/hardware-test
```

UART 115200 8N1 trả một dòng:

```text
DWM1001_TEST,init=0,devid=0xdeca0130,spi_mhz=8,config=0x00000000,temp_raw=...
```

LED xanh nháy 200 ms khi đạt; nháy 1000 ms khi lỗi.

## Lưu ý calibration

Mặc định vẫn dùng các software offset cũ và DS-TWR đang bật để không tự ý đổi
profile đo đã được hiệu chỉnh. Khi chuyển sang antenna delay phần cứng, chọn
`UWB_COMPENSATION_ANTENNA_DELAY` bằng Kconfig và phải calibration lại toàn bộ
TAG/Anchor theo đúng PHY Ch5, PRF16, preamble 256, 6.8 Mbps. Không dùng đồng
thời software offset và antenna delay.

Zephyr IEEE 802.15.4 DW1000 driver bị tắt có chủ ý. Driver generic đó không
expose các timestamp 40-bit, delayed TX và chẩn đoán cần cho state machine TWR
hiện tại; firmware này truy cập register DW1000 trực tiếp nhưng toàn bộ I/O vật
lý vẫn đi qua module platform Zephyr.
