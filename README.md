# DWM1001 UWB Ranging System

<a id="english"></a>**🇬🇧 English** · [🇻🇳 Tiếng Việt](#tieng-viet)

A UWB ranging system based on the **DWM1001C** module (DW1000 + nRF52832). It includes Tag/Anchor firmware, an **ESP32-C3 gateway** and a **Windows GUI** for monitoring, logging, analysis and calibration.

> [!WARNING]
> The firmware currently sets `UWB_DS_CALIBRATED_MASK = 0`. `raw` data can be used for
> calibration, but it is not yet treated as a valid distance for drone control.
> Do not feed this data to a flight controller until calibration and testing on
> real hardware are complete.

## System architecture

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

- The Tag runs `POLL -> RESP -> FINAL -> REPORT` in turn with up to 4 Anchors.
- PHY: channel 5, PRF16, preamble 256, PAC16, 6.8 Mbps.
- The Tag packs INFO/RANGE/STATS frames into a binary protocol with CRC16-CCITT.
- The ESP32-C3 validates each frame and forwards it unchanged over USB Serial/JTAG.
- The GUI decodes telemetry, shows range/quality/counters, records sessions, and
  supports replay, 2D/3D analysis and a calibration workflow.

## Repository structure

| Path | Role |
|---|---|
| `Firmware/Tag/` | Zephyr firmware (in development) for the DWM1001C Tag |
| `Firmware/Anchor_1/` | Zephyr firmware for the Anchor at address `0x0001` |
| `Firmware/ESP32C3_Gateway/` | ESP-IDF gateway: receives UART from the Tag and streams binary over USB |
| `Firmware/tests/` | Host tests for the driver and the Tag state machine |
| `Software/UWB_UART_GUI/` | Python GUI, decoder, recorder, filters and unit tests |
| `Firmware Code Base/` | Shared reference copy / Kconfig; not the active source |
| `Plan/` | Plan for extending the positioning system and integrating it into a drone |

Build output, measurement logs, Python bytecode, downloaded toolchains, secrets and local
configuration are excluded from Git by `.gitignore`.

## Quick analysis

### Strengths

- Clear separation between the platform, DW1000 driver, ranging state machine, filters and telemetry.
- Radio I/O goes through Zephyr, but DW1000 registers are still accessed directly to support
  40-bit timestamps and delayed TX, which TWR needs.
- The protocol has length/version/CRC fields, and the streaming parser handles fragmented packets.
- Watchdog, diagnostic counters, stale state, and a guard that blocks uncalibrated ranges.
- C host tests for the firmware/gateway and Python unit tests for the GUI.

### Current limitations

- Only the `Anchor_1` project is ready to build, while the Tag still declares 4 Anchors. To run
  A1–A4, create three more projects with their own short addresses.
- No Anchor is marked as calibrated yet, so `valid` stays false.
- The ESP32-C3 gateway still needs to be built, flashed and verified with ESP-IDF on real hardware.
- `Firmware Code Base/` and `Firmware/` contain duplicated code. Use `Firmware/` as the main
  source for new work so the reference copy is not edited by mistake.
- The 3D solver needs at least 4 valid ranges and a non-coplanar Anchor layout.
- A successful build does not replace testing RF, power, antennas, NLOS and vibration
  under flight conditions.

## Requirements

### Minimum hardware

- 2 DWM1001C modules/boards: 1 Tag and at least 1 Anchor.
- An SWD probe compatible with J-Link, OpenOCD or pyOCD to flash the nRF52832.
- An ESP32-C3 for the gateway, or a 3.3 V USB-UART adapter connected directly to the Tag.
- Shared power and GND; never apply 5 V TTL to the DWM1001C pins.

### Software

- Windows PowerShell.
- [nRF Connect SDK](https://docs.nordicsemi.com/r/bundle/nrf-connect-vscode/page/get_started/quick_setup.html/installing-sdk-and-toolchain-for-the-first-time)
  v3.4.0, with `west` in `PATH`.
- Zephyr board
  [`decawave_dwm1001_dev/nrf52832`](https://docs.zephyrproject.org/latest/boards/qorvo/decawave_dwm1001_dev/doc/index.html).
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
  to build the gateway.
- Python 3.12 and PySerial 3.5.x to run the GUI.
- `gcc` in `PATH` to run the C host tests.

Official hardware documentation:
[DWM1001 datasheet](https://store.qorvo.com/datasheets/qorvo/dwm1001datasheet.pdf).

## Quick start

### 1. Clone and open the project

```powershell
git clone https://github.com/TuanLinh05/DWM1001_UWB.git
Set-Location .\DWM1001_UWB
```

The commands below assume a terminal set up with nRF Connect SDK v3.4.0. Check with:

```powershell
west --version
```

### 2. Build the Tag and Anchor 1 firmware

```powershell
Set-Location .\Firmware
.\scripts\build_all.ps1
```

Or build each node separately:

```powershell
Set-Location .\Tag
.\scripts\build.ps1

Set-Location ..\Anchor_1
.\scripts\build.ps1
```

The scripts temporarily map the `Firmware` folder to drive `U:` to avoid Kconfig errors with
paths that contain spaces, then remove the mapping. Do not run them if drive `U:` is already in use.

Firmware output:

- `Firmware/Tag/build/zephyr/zephyr.hex`
- `Firmware/Anchor_1/build/zephyr/zephyr.hex`

### 3. Flash the DWM1001C

Connect the SWD probe and flash each board from the nRF Connect SDK terminal:

```powershell
Set-Location .\Firmware\Anchor_1
.\scripts\flash.ps1

Set-Location ..\Tag
.\scripts\flash.ps1
```

Flash the Anchor first, then the Tag. Runner/probe instructions are listed in the Zephyr
`decawave_dwm1001_dev` board documentation (see Requirements).

### 4. Create Anchors 2–4

Copy the whole `Firmware/Anchor_1` folder to `Anchor_2`, `Anchor_3` and `Anchor_4`, then set
`ANCHOR_ADDR` in `include/uwb_app_config.h` to `2U`, `3U` and `4U`. Each Anchor must have a
unique short address and the same PHY profile as the Tag.

### 5. Build the ESP32-C3 gateway

Open an ESP-IDF PowerShell/Command Prompt:

```powershell
Set-Location .\Firmware\ESP32C3_Gateway
idf.py set-target esp32c3
idf.py build
idf.py flash
```

Current PCB connections:

| DWM1001C / nRF52832 | ESP32-C3 | Function |
|---|---:|---|
| UART TX P0.05 | GPIO20 RX | Binary telemetry from the Tag |
| UART RX P0.11 | GPIO21 TX | Spare command channel |
| RDY P0.26 | GPIO10 | Spare handshake/IRQ |
| GND | GND | Common ground |

Do not open `idf.py monitor` together with the GUI, because both will compete for the COM port.
The gateway's USB stream is binary-only; do not insert text logs into it.

### 6. Run the GUI

```powershell
Set-Location .\Software\UWB_UART_GUI
py -3.12 -m pip install -r requirements.txt
.\run_gui.ps1
```

In the GUI, choose the ESP32-C3/USB-UART COM port, baud `115200`, and click **Kết nối** (Connect).
To try it without hardware:

```powershell
.\run_demo.ps1
```

## Calibrate before using ranges

1. Keep `TELEM_ASCII = 0` so the GUI reads the binary protocol.
2. Place the Tag and each Anchor at several reference distances, measured between antenna centres.
3. In the **Calibration** tab, capture enough samples at each distance.
4. Check noise, drift, FPP and P05–P95, and keep DS-TWR and SS-TWR separate.
5. Write the confirmed offsets to `Firmware/Tag/include/uwb_app_config.h`.
6. Only set the matching bit in `UWB_DS_CALIBRATED_MASK` after passing a validation
   distance that was not used for fitting.
7. Rebuild and reflash the Tag, then confirm the GUI shows `valid = true` consistently.

GUI offset definition:

```text
offset_mm = reference_mm - mean_raw_mm
```

## Running tests

From the repository root:

```powershell
# Driver and Tag state machine
.\Firmware\tests\run_host_tests.ps1

# ESP32-C3 gateway parser
.\Firmware\ESP32C3_Gateway\tests\run_host_test.ps1

# GUI, decoder, recorder, filters and analysis
Set-Location .\Software\UWB_UART_GUI
py -3.12 -m unittest discover -s tests -v
```

Full Zephyr build:

```powershell
Set-Location .\Firmware
.\scripts\build_all.ps1
```

## Detailed documentation

- [`Firmware/HARDWARE_COMPATIBILITY.md`](Firmware/HARDWARE_COMPATIBILITY.md):
  pin mapping, hardware decisions and bring-up sequence.
- [`Firmware/SOURCE_REVIEW_2026-09-12.md`](Firmware/SOURCE_REVIEW_2026-09-12.md):
  fixed bugs, test coverage and known limitations.
- [`Software/UWB_UART_GUI/README.md`](Software/UWB_UART_GUI/README.md): GUI features,
  log format and calibration.
- [`Plan/KE_HOACH_TRIEN_KHAI_UWB_DRONE_UP7000_PIXHAWK6C.md`](Plan/KE_HOACH_TRIEN_KHAI_UWB_DRONE_UP7000_PIXHAWK6C.md):
  plan for integrating positioning into the drone system.

## Contributing safely

- Run `git status --short` and review `git diff --cached` before every push.
- Do not use `git add -f` for logs, build output, toolchains, datasheets or secrets.
- If a secret was ever committed, deleting the file from the working tree is not enough: revoke
  the secret and clean the whole Git history before pushing.
- The repository does not declare a license yet. Add a suitable `LICENSE` before allowing
  others to use or redistribute the code.

---

<a id="tieng-viet"></a>

## 🇻🇳 Tiếng Việt

[🇬🇧 English](#english) · **🇻🇳 Tiếng Việt**

Hệ thống đo khoảng cách UWB dùng module DWM1001C (DW1000 + nRF52832), gồm
firmware Tag/Anchor, gateway ESP32-C3 và GUI Windows để giám sát, ghi log, phân
tích và hiệu chuẩn.

> [!WARNING]
> Firmware hiện đặt `UWB_DS_CALIBRATED_MASK = 0`. Dữ liệu `raw` có thể dùng để
> hiệu chuẩn, nhưng chưa được coi là khoảng cách hợp lệ cho điều khiển drone.
> Không đưa dữ liệu vào flight controller trước khi hoàn tất hiệu chuẩn và kiểm
> thử trên phần cứng thật.

### Kiến trúc hệ thống

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

### Cấu trúc repository

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

### Phân tích nhanh

#### Điểm mạnh

- Tách rõ platform, driver DW1000, state machine ranging, filter và telemetry.
- I/O radio chạy qua Zephyr nhưng vẫn truy cập register DW1000 để hỗ trợ
  timestamp 40-bit và delayed TX cần cho TWR.
- Protocol có length/version/CRC và parser streaming chịu được packet phân mảnh.
- Có watchdog, counter chẩn đoán, trạng thái stale và guard ngăn dùng range chưa
  calibration.
- Có host test C cho firmware/gateway và unit test Python cho GUI.

#### Giới hạn hiện tại

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

### Yêu cầu

#### Phần cứng tối thiểu

- 2 module/board DWM1001C: 1 Tag và ít nhất 1 Anchor.
- Probe SWD tương thích J-Link, OpenOCD hoặc pyOCD để flash nRF52832.
- ESP32-C3 cho gateway, hoặc USB-UART mức 3.3 V để nối trực tiếp với Tag.
- Nguồn và GND chung; không đưa TTL 5 V vào chân DWM1001C.

#### Phần mềm

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

### Bắt đầu nhanh

#### 1. Clone và mở project

```powershell
git clone https://github.com/TuanLinh05/DWM1001_UWB.git
Set-Location .\DWM1001_UWB
```

Các lệnh dưới đây giả định terminal đã được kích hoạt bằng nRF Connect SDK
v3.4.0. Kiểm tra bằng:

```powershell
west --version
```

#### 2. Build firmware Tag và Anchor 1

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

#### 3. Flash DWM1001C

Nối đúng probe SWD và flash từng board từ terminal nRF Connect SDK:

```powershell
Set-Location .\Firmware\Anchor_1
.\scripts\flash.ps1

Set-Location ..\Tag
.\scripts\flash.ps1
```

Flash Anchor trước, sau đó flash Tag. Hướng dẫn runner/probe được Zephyr liệt kê
trong tài liệu board `decawave_dwm1001_dev` ở phần Yêu cầu.

#### 4. Tạo Anchor 2-4

Sao chép toàn bộ `Firmware/Anchor_1` thành `Anchor_2`, `Anchor_3`, `Anchor_4`, rồi
đổi `ANCHOR_ADDR` trong `include/uwb_app_config.h` tương ứng thành `2U`, `3U`,
`4U`. Mỗi Anchor phải có short address duy nhất và cùng PHY profile với Tag.

#### 5. Build gateway ESP32-C3

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

#### 6. Chạy GUI

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

### Hiệu chuẩn trước khi sử dụng range

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

### Chạy test

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

### Tài liệu chi tiết

- [`Firmware/HARDWARE_COMPATIBILITY.md`](Firmware/HARDWARE_COMPATIBILITY.md):
  pin mapping, quyết định phần cứng và trình tự bring-up.
- [`Firmware/SOURCE_REVIEW_2026-09-12.md`](Firmware/SOURCE_REVIEW_2026-09-12.md):
  lỗi đã sửa, phạm vi test và giới hạn đã biết.
- [`Software/UWB_UART_GUI/README.md`](Software/UWB_UART_GUI/README.md): chức năng
  GUI, format log và calibration.
- [`Plan/KE_HOACH_TRIEN_KHAI_UWB_DRONE_UP7000_PIXHAWK6C.md`](Plan/KE_HOACH_TRIEN_KHAI_UWB_DRONE_UP7000_PIXHAWK6C.md):
  kế hoạch tích hợp định vị vào hệ thống drone.

### Đóng góp an toàn

- Chạy `git status --short` và kiểm tra `git diff --cached` trước mỗi lần push.
- Không dùng `git add -f` cho log, build output, toolchain, datasheet hoặc secret.
- Nếu một secret từng được commit, xóa file ở working tree là chưa đủ: phải thu
  hồi secret và làm sạch toàn bộ Git history trước khi push.
- Repository hiện chưa khai báo giấy phép. Hãy thêm `LICENSE` phù hợp trước khi
  cho phép người khác sử dụng hoặc phân phối lại mã nguồn.
