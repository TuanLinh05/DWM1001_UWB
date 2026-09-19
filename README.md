# DWM1001 UWB Ranging System

Hệ thống đo khoảng cách UWB dùng module DWM1001C (DW1000 + nRF52832), gồm
firmware Tag/Anchor, gateway ESP32-C3 và GUI Windows để giám sát, ghi log, phân
tích và hiệu chuẩn.

> [!WARNING]
> Firmware hiện đặt `UWB_DS_CALIBRATED_MASK = 0`. Dữ liệu `raw` có thể dùng để
> hiệu chuẩn, nhưng chưa được coi là khoảng cách hợp lệ cho điều khiển drone.
> Không đưa dữ liệu vào flight controller trước khi hoàn tất hiệu chuẩn và kiểm
> thử trên phần cứng thật.

## Kiến trúc hệ thống

```text
┌──────────────────┐       DS-TWR/UWB       ┌────────────────────┐
│ DWM1001C Tag     │ <────────────────────> │ DWM1001C Anchor(s) │
│ DW1000+nRF52832  │                        │ address 0x0001..04 │
└────────┬─────────┘                        └────────────────────┘
         │ UART 115200, binary + CRC16
         ▼
┌──────────────────┐       USB Serial       ┌────────────────────┐
│ ESP32-C3 Gateway │ ─────────────────────> │ Python GUI / PC    │
│ stream bridge    │                        │ monitor & calibrate│
└──────────────────┘                        └────────────────────┘
```

- Tag lần lượt thực hiện `POLL -> RESP -> FINAL -> REPORT` với tối đa 4 Anchor.
- PHY hiện dùng channel 5, PRF16, preamble 256, PAC16 và 6.8 Mbps.
- Tag đóng gói INFO/RANGE/STATS theo protocol nhị phân có CRC16-CCITT.
- ESP32-C3 kiểm tra frame rồi chuyển tiếp nguyên gói qua USB Serial/JTAG.
- GUI giải mã telemetry, hiển thị range/quality/counters, ghi phiên đo, hỗ trợ
  replay, phân tích 2D/3D và quy trình calibration.

## Cấu trúc repository

| Đường dẫn | Vai trò |
|---|---|
| `Firmware/Tag/` | Firmware Zephyr đang phát triển cho Tag DWM1001C |
| `Firmware/Anchor_1/` | Firmware Zephyr cho Anchor địa chỉ `0x0001` |
| `Firmware/ESP32C3_Gateway/` | Gateway ESP-IDF nhận UART từ Tag và phát USB binary |
| `Firmware/tests/` | Host test cho driver và state machine Tag |
| `Software/UWB_UART_GUI/` | GUI Python, decoder, recorder, filter và unit test |
| `Firmware Code Base/` | Bản tham chiếu dùng chung/Kconfig; không phải source active |
| `Plan/` | Kế hoạch mở rộng hệ thống định vị và tích hợp drone |

Build output, log đo, bytecode Python, toolchain tải về, secret và cấu hình cục
bộ được loại khỏi Git bởi `.gitignore`.

## Phân tích nhanh

### Điểm mạnh

- Tách rõ platform, driver DW1000, state machine ranging, filter và telemetry.
- I/O radio chạy qua Zephyr nhưng vẫn truy cập register DW1000 để hỗ trợ
  timestamp 40-bit và delayed TX cần cho TWR.
- Protocol có length/version/CRC và parser streaming chịu được packet phân mảnh.
- Có watchdog, counter chẩn đoán, trạng thái stale và guard ngăn dùng range chưa
  calibration.
- Có host test C cho firmware/gateway và unit test Python cho GUI.

### Giới hạn hiện tại

- Chỉ có project `Anchor_1` sẵn sàng build; Tag vẫn khai báo 4 Anchor. Muốn chạy
  đủ A1-A4 phải tạo thêm ba project với short address riêng.
- Chưa Anchor nào được đánh dấu đã calibration; giá trị `valid` sẽ vẫn false.
- Gateway ESP32-C3 cần được build/flash và xác nhận lại bằng ESP-IDF trên phần
  cứng thật.
- `Firmware Code Base/` và `Firmware/` có mã trùng nhau. Khi phát triển mới,
  dùng `Firmware/` làm nguồn chính để tránh sửa nhầm bản tham chiếu.
- Solver 3D cần ít nhất 4 range hợp lệ và hình học Anchor không đồng phẳng.
- Build thành công không thay thế kiểm thử RF, nguồn, antenna, NLOS và rung động
  trong điều kiện bay.

## Yêu cầu

### Phần cứng tối thiểu

- 2 module/board DWM1001C: 1 Tag và ít nhất 1 Anchor.
- Probe SWD tương thích J-Link, OpenOCD hoặc pyOCD để flash nRF52832.
- ESP32-C3 cho gateway, hoặc USB-UART mức 3.3 V để nối trực tiếp với Tag.
- Nguồn và GND chung; không đưa TTL 5 V vào chân DWM1001C.

### Phần mềm

- Windows PowerShell.
- [nRF Connect SDK](https://docs.nordicsemi.com/r/bundle/nrf-connect-vscode/page/get_started/quick_setup.html/installing-sdk-and-toolchain-for-the-first-time)
  v3.4.0, với `west` trong `PATH`.
- Board Zephyr
  [`decawave_dwm1001_dev/nrf52832`](https://docs.zephyrproject.org/latest/boards/qorvo/decawave_dwm1001_dev/doc/index.html).
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
  để build gateway.
- Python 3.12 và PySerial 3.5.x để chạy GUI.
- `gcc` trong `PATH` nếu muốn chạy host test C.

Tài liệu phần cứng chính thức:
[DWM1001 datasheet](https://store.qorvo.com/datasheets/qorvo/dwm1001datasheet.pdf).

## Bắt đầu nhanh

### 1. Clone và mở project

```powershell
git clone <repository-url>
Set-Location .\<repository-folder>
```

Các lệnh dưới đây giả định terminal đã được kích hoạt bằng nRF Connect SDK
v3.4.0. Kiểm tra bằng:

```powershell
west --version
```

### 2. Build firmware Tag và Anchor 1

```powershell
Set-Location .\Firmware
.\scripts\build_all.ps1
```

Hoặc build riêng từng node:

```powershell
Set-Location .\Tag
.\scripts\build.ps1

Set-Location ..\Anchor_1
.\scripts\build.ps1
```

Các script tạm ánh xạ thư mục `Firmware` sang ổ `U:` để tránh lỗi Kconfig với
đường dẫn có khoảng trắng, sau đó tự gỡ mapping. Không chạy script nếu ổ `U:`
đang được dùng cho mục đích khác.

File firmware sau khi build:

- `Firmware/Tag/build/zephyr/zephyr.hex`
- `Firmware/Anchor_1/build/zephyr/zephyr.hex`

### 3. Flash DWM1001C

Nối đúng probe SWD và flash từng board từ terminal nRF Connect SDK:

```powershell
Set-Location .\Firmware\Anchor_1
.\scripts\flash.ps1

Set-Location ..\Tag
.\scripts\flash.ps1
```

Flash Anchor trước, sau đó flash Tag. Hướng dẫn runner/probe được Zephyr liệt kê
trong tài liệu board `decawave_dwm1001_dev` ở phần Yêu cầu.

### 4. Tạo Anchor 2-4

Sao chép toàn bộ `Firmware/Anchor_1` thành `Anchor_2`, `Anchor_3`, `Anchor_4`, rồi
đổi `ANCHOR_ADDR` trong `include/uwb_app_config.h` tương ứng thành `2U`, `3U`,
`4U`. Mỗi Anchor phải có short address duy nhất và cùng PHY profile với Tag.

### 5. Build gateway ESP32-C3

Mở ESP-IDF PowerShell/Command Prompt:

```powershell
Set-Location .\Firmware\ESP32C3_Gateway
idf.py set-target esp32c3
idf.py build
idf.py flash
```

Kết nối PCB hiện tại:

| DWM1001C / nRF52832 | ESP32-C3 | Chức năng |
|---|---:|---|
| UART TX P0.05 | GPIO20 RX | Telemetry binary từ Tag |
| UART RX P0.11 | GPIO21 TX | Kênh lệnh dự phòng |
| RDY P0.26 | GPIO10 | Handshake/IRQ dự phòng |
| GND | GND | Mass chung |

Không mở `idf.py monitor` đồng thời với GUI vì cả hai sẽ tranh cổng COM. USB của
gateway là luồng binary-only; không chèn log text vào stream này.

### 6. Chạy GUI

```powershell
Set-Location .\Software\UWB_UART_GUI
py -3.12 -m pip install -r requirements.txt
.\run_gui.ps1
```

Trong GUI, chọn COM của ESP32-C3/USB-UART, baud `115200`, rồi nhấn **Kết nối**.
Để thử không cần phần cứng:

```powershell
.\run_demo.ps1
```

## Hiệu chuẩn trước khi sử dụng range

1. Giữ `TELEM_ASCII = 0` để GUI đọc protocol binary.
2. Đặt Tag và từng Anchor tại nhiều khoảng cách chuẩn, đo giữa hai tâm antenna.
3. Trong tab **Calibration**, capture đủ mẫu ở từng khoảng cách.
4. Kiểm tra noise, drift, FPP, P05-P95 và tách riêng DS-TWR/SS-TWR.
5. Ghi offset đã xác nhận vào `Firmware/Tag/include/uwb_app_config.h`.
6. Chỉ bật bit tương ứng trong `UWB_DS_CALIBRATED_MASK` sau khi vượt qua một
   khoảng cách validation không dùng khi fit.
7. Build/flash lại Tag và xác nhận GUI hiển thị `valid = true` ổn định.

Định nghĩa offset của GUI:

```text
offset_mm = reference_mm - mean_raw_mm
```

## Chạy test

Từ thư mục gốc repository:

```powershell
# Driver và state machine Tag
.\Firmware\tests\run_host_tests.ps1

# Parser gateway ESP32-C3
.\Firmware\ESP32C3_Gateway\tests\run_host_test.ps1

# GUI, decoder, recorder, filter và analysis
Set-Location .\Software\UWB_UART_GUI
py -3.12 -m unittest discover -s tests -v
```

Build Zephyr đầy đủ:

```powershell
Set-Location .\Firmware
.\scripts\build_all.ps1
```

## Tài liệu chi tiết

- [`Firmware/HARDWARE_COMPATIBILITY.md`](Firmware/HARDWARE_COMPATIBILITY.md):
  pin mapping, quyết định phần cứng và trình tự bring-up.
- [`Firmware/SOURCE_REVIEW_2026-09-12.md`](Firmware/SOURCE_REVIEW_2026-09-12.md):
  lỗi đã sửa, phạm vi test và giới hạn đã biết.
- [`Software/UWB_UART_GUI/README.md`](Software/UWB_UART_GUI/README.md): chức năng
  GUI, format log và calibration.
- [`Plan/KE_HOACH_TRIEN_KHAI_UWB_DRONE_UP7000_PIXHAWK6C.md`](Plan/KE_HOACH_TRIEN_KHAI_UWB_DRONE_UP7000_PIXHAWK6C.md):
  kế hoạch tích hợp định vị vào hệ thống drone.

## Trước khi public repository

Thư mục local chưa có Git metadata. Sau khi tạo một repository rỗng trên GitHub,
chạy từ thư mục gốc project:

```powershell
git init
git add .
git status --short
git diff --cached --stat
git commit -m "Initial public release"
git branch -M main
git remote add origin <repository-url>
git push -u origin main
```

Đọc lại danh sách staged trước khi commit; `.gitignore` chỉ bảo vệ các file khớp
quy tắc và không thay thế bước review thủ công.

- Chạy `git status --short` và kiểm tra `git diff --cached` trước mỗi lần push.
- Không dùng `git add -f` cho log, build output, toolchain, datasheet hoặc secret.
- Nếu một secret từng được commit, xóa file ở working tree là chưa đủ: phải thu
  hồi secret và làm sạch toàn bộ Git history trước khi push.
- Repository hiện chưa khai báo giấy phép. Hãy thêm `LICENSE` phù hợp trước khi
  cho phép người khác sử dụng hoặc phân phối lại mã nguồn.
