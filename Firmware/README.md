# Firmware DWM1001C Ranging System

Hệ định vị UWB cho drone: một TAG DWM1001C đo DS-TWR với tám anchor, đẩy
telemetry nhị phân qua UART → gateway ESP32-C3 → PC/UP 7000. Firmware chỉ
**đo**; bộ ước lượng vị trí chạy trên host (ADR-001 trong `Plan/`).

Từ 0.5 (2026-09-26) Tag_DevKit mặc định chạy **burst DS-TWR một-nhiều**: một
POLL broadcast, tám RESP theo slot, một FINAL broadcast. Profile ban đầu
~4,5 ms/chu kỳ (~220 Hz cho mỗi anchor); Tag_DevKit hiện dùng timing 20 ms
và mask A1–A8 sau khi thử trên phần cứng thật. PHY chuyển sang preamble 128 /
PAC 8. Chi tiết:
`CHANGELOG.md` §0.5.

> **Phần cứng hiện có (xác nhận 2026-09-23):** A1–A4 là DWM1001C; A5–A8 là
> STM32F103 + DW1000. A5–A8 dùng project Zephyr `nodes/anchors/stm32/STM32_Anchor_5..8`
> (giao thức v2, HEX cho ST-LINK Utility). Các project `nodes/anchors/nrf/Anchor_5..8`
> cũ vẫn là target nRF52832. Bản STM32 đã build và kiểm file; còn cần thử
> trên board thật. Xem `nodes/anchors/stm32/README.md`.

## 1. Bố cục

```
Firmware/
├── common/                      ← toàn bộ mã dùng chung (nguồn duy nhất), chia theo module;
│   │                              mỗi module chứa .c và .h của nó ngay cạnh nhau
│   ├── app/                     ← điểm vào: main_tag.c, main_anchor.c, main_sniffer.c
│   ├── drivers/                 ← driver thanh ghi DW1000 (dw1000.c, dw1000_hw.h)
│   ├── platform/                ← lớp Zephyr: SPI, GPIO, IRQ, thời gian
│   ├── ranging/                 ← máy trạng thái TAG/anchor, khung UWB (uwb_frame)
│   ├── filters/                 ← bộ lọc range (range_filter)
│   ├── telemetry/               ← khung nhị phân UART + uart_tx
│   ├── services/                ← kênh lệnh (uwb_cmd), NVS (uwb_settings), health
│   ├── config/                  ← bảng calibration dùng chung (uwb_calibration.h)
│   └── cmake/uwb_node.cmake     ← uwb_node_setup(TAG|ANCHOR|SNIFFER) + build id
├── nodes/                       ← mỗi node chỉ giữ cấu hình, không giữ mã nguồn
│   ├── tag/Tag/                 ← TAG trên PCB RangingSystemClassic
│   ├── tag/Tag_DevKit/          ← cùng vai trò TAG, chạy trên DWM1001-DEV (J-Link)
│   ├── anchors/nrf/Anchor_1…8/  ← responder DWM1001C, short address 0x0001..0x0008
│   ├── anchors/stm32/STM32_Anchor_5…8/ ← responder cho board STM32F103 vật lý
│   └── sniffer/Sniffer_DevKit/  ← chỉ nghe, đẩy mọi frame kèm timestamp DW1000
├── gateway/ESP32C3_Gateway/     ← ESP-IDF gateway UART ↔ USB
├── profiles/                    ← overlay/conf bổ sung cho bản high-rate
├── release/                     ← HEX đã đóng gói để nạp
│   ├── anchor_8/                ← bộ tám HEX theo đúng board vật lý + manifest
│   ├── stm32_anchor/            ← HEX STM32 A5–A8 đã kiểm tra
│   └── highrate_200/            ← bộ thử nghiệm ≥200 Hz (TAG + 8 anchor)
├── docs/                        ← checklist, triển khai, tương thích phần cứng
├── scripts/                     ← build/flash (xem bảng dưới)
├── tests/                       ← test host (gcc + mô phỏng thanh ghi DW1000)
├── tools/                       ← uwb_sniffer.py, verify_*.py, uwb_layout.py
└── CHANGELOG.md
```

Mỗi project node chỉ còn `CMakeLists.txt`, `prj.conf`, `app.overlay`,
`include/uwb_app_config.h` (node nRF có thêm `scripts/` và `.vscode/`). Không
node nào giữ bản sao mã nguồn chung — `tests/test_node_projects.py` sẽ báo lỗi
nếu có, hoặc nếu một file `.c/.h` trong `common/` nằm ngoài thư mục module.

### Debug nhanh: lỗi ở đâu thì mở thư mục nào

| Triệu chứng | Xem trước |
|---|---|
| Không đọc được Device ID, SPI/IRQ lỗi | `common/drivers/`, `common/platform/`, `app.overlay` của node |
| Sai khoảng cách, timeout, slot burst | `common/ranging/` (+ `tests/test_tag_*.c`, `test_anchor_*.c`) |
| Range nhảy/nhiễu | `common/filters/`, `common/config/uwb_calibration.h` |
| GUI không giải mã được / CRC lỗi | `common/telemetry/` |
| Lệnh từ host không có tác dụng, NVS | `common/services/` |
| Hành vi khác nhau giữa các node | `nodes/<...>/include/uwb_app_config.h`, `prj.conf` |

### Script

| Script | Việc |
|---|---|
| `scripts/uwb_nodes.ps1` | **registry duy nhất** tên node → đường dẫn/board/cách flash, + helper map ổ `U:` |
| `scripts/node.ps1 -Project <tên> -Action Build\|Flash` | build/flash một node (script `scripts/*.ps1` trong từng node gọi vào đây) |
| `scripts/build_all.ps1` | build nhiều node |
| `scripts/build_stm32_anchors.ps1` | build A5–A8 STM32 → `release/stm32_anchor/` |
| `scripts/package_8_anchors.ps1` | gom 8 HEX anchor → `release/anchor_8/` |
| `scripts/build_highrate.ps1`, `flash_highrate.ps1` | bản high-rate → `release/highrate_200/` |
| `scripts/ncs_env.ps1` | tìm NCS SDK/toolchain, J-Link, OpenOCD (dùng chung) |

Thêm/di chuyển node: sửa `scripts/uwb_nodes.ps1` **và** `tools/uwb_layout.py`
(test sẽ báo nếu hai file lệch nhau).

## 2. Build

Bản tối ưu ngày **2026-10-02**, mục tiêu ≥200 kết quả DS/s trên mỗi anchor,
được build riêng bằng `scripts/build_highrate.ps1` cho TAG + tám anchor đúng
board vật lý. Period 4500 µs (~222 Hz danh định), anchor auto RX,
STM32 SPI trực tiếp 16 MHz, nRF FPU. Cần nạp lại đủ chín image và đo log để
xác nhận tần số thực tế. Xem [hướng dẫn highrate](release/highrate_200/README.md).

Yêu cầu nRF Connect SDK v3.4.0, board Zephyr `decawave_dwm1001_dev/nrf52832`.

```powershell
Set-Location .\Firmware
.\scripts\build_all.ps1                         # 11 project nRF
.\scripts\build_all.ps1 -Projects Tag,Anchor_1  # chỉ một phần
.\scripts\node.ps1 -Project Tag_DevKit -Action Build
```

File để flash: `nodes/<...>/<Project>/build/zephyr/zephyr.hex`.
Mỗi project nRF cũng có `scripts/build.ps1` và `scripts/flash.ps1` riêng (dùng
bởi task VS Code khi mở thư mục node).

Với bốn board STM32 vật lý, chạy `scripts/build_stm32_anchors.ps1`, sau đó
`python tools/verify_stm32_images.py`. Chạy `scripts/package_8_anchors.ps1`
để chép tám ảnh tương ứng board vật lý vào `release/anchor_8/` cùng SHA-256.
A1–A4 nạp qua ST-LINK/OpenOCD theo cấu hình VS Code của từng project;
A5–A8 nạp qua ST-LINK Utility.

Firmware mang theo git hash, cờ dirty (kể cả file untracked) và hash cấu hình
riêng của node trong `DEVICE_INFO`, nên có thể đối chiếu source lẫn cấu hình
đã tạo ra binary.

> **Thứ tự flash: các anchor đang dùng trước, TAG sau.** Anchor 0.5 trả lời POLL
> v1, v2 và v3 (burst). Firmware trước 0.5 dùng preamble 256 nên **không nghe
> được** node 0.5 (preamble 128) và ngược lại; mọi node tham gia cần dùng cùng PHY.
> Chi tiết ở `CHANGELOG.md` §0.5 và §1.

## 3. Kiểm thử trên PC (bắt buộc trước khi flash)

```powershell
Set-Location .\Firmware
.\tests\run_host_tests.ps1
```

Cần `gcc` (MinGW-w64) và Python 3. 13 nhóm test, không cần phần cứng:

| Test | Nội dung |
|---|---|
| `test_driver` | SPI một giao dịch, phân loại RX, OTP, TX power, chẩn đoán |
| `test_uwb_frame` | mã hoá/giải mã khung v1 & v2, kiểm tra biên TLV |
| `test_tag_state` | máy trạng thái TAG thật chạy trên mô phỏng DW1000: DS-TWR, txn, F1, fallback, mask, pause |
| `test_anchor_state` | responder thật: v1/v2/v3, WAIT4RESP, lỗi RX, timeout, slot burst, RX FINAL hẹn giờ |
| `test_tag_burst` | burst v3 với 8 anchor mô phỏng (crystal offset riêng): Rb trễ một chu kỳ, fallback SS, offline/probe, SET_BURST |
| `test_cmd_parser` | CRC, tái đồng bộ, gói phân mảnh, giới hạn độ dài |
| `test_cmd_executor` | ACK, quy tắc LOCK/PAUSE, tác động lên radio |
| `test_settings` | snapshot NVS nguyên khối, CRC, PARTID/profile binding, lỗi factory reset |
| `test_telemetry_golden` (C + Python) | bộ mã hoá C đối chiếu chéo bộ giải mã Python |
| `test_node_projects.py` | cấu trúc 15 project + module `common/`, registry node, địa chỉ anchor, mask fail-closed |
| `test_sniffer_tool.py` | công cụ sniffer giải mã đúng timeline |
| `gui_python_tests` | 48 test của GUI/giao thức, gồm heartbeat host→TAG |
| `gateway_parser` | bộ phân tích khung của gateway ESP32-C3 |

Test chạy **chính mã sản phẩm** trên `tests/dw1000_sim.c` (mô phỏng mức thanh
ghi), nên máy trạng thái và công thức DS-TWR được kiểm chứng trước khi nạp board.

## 4. Cấu hình 8 Anchor

Bảng dưới là cấu hình của tám **project nRF/Zephyr** (`nodes/anchors/nrf/`). Trên phần cứng hiện
tại chỉ A1–A4 dùng các project này; A5–A8 dùng `nodes/anchors/stm32/STM32_Anchor_5..8`.

| Project | Short address | Nhịp cập nhật |
|---|---:|---:|
| Anchor_1 … Anchor_8 | `0x0001` … `0x0008` | phụ thuộc timing TAG; Tag_DevKit hiện đặt tối thiểu 20 ms / ~50 Hz cho A1–A8 |

TAG probe xoay vòng các anchor mất kết nối (F1), nên một module hỏng không chiếm
hết thời gian chờ của chu kỳ. Ở burst, anchor im lặng 8 chu kỳ bị bỏ khỏi mask
và được thử lại mỗi 25 chu kỳ. `period_us` của RANGE_BURST là chu kỳ đo thật.

### Chỉnh timing burst (`SET_BURST`, chỉ lưu RAM)

Bảng là mặc định mới của **Tag_DevKit** cho tám anchor vật lý. Project `Tag`
cho PCB riêng vẫn giữ profile cũ; thay đổi qua `SET_BURST` sẽ mất khi TAG reset.

| Thông số | Mặc định | Ý nghĩa |
|---|---:|---|
| base | 2500 UUS | POLL → RESP đầu tiên: thời gian anchor chậm nhất đọc POLL và hẹn RESP |
| slot | 1000 UUS | khoảng cách hai RESP: air time (~205 µs) + TAG đọc RESP |
| margin | 1200 UUS | RESP cuối → FINAL |
| gap | 1500 µs | FINAL → POLL kế tiếp (anchor mở lại receiver, TAG gửi UART) |
| period | 20000 µs | chu kỳ POLL tối thiểu |

```powershell
py -3.12 uwb_command.py --port COM14 set-burst                   # xem
py -3.12 uwb_command.py --port COM14 set-mask 0xFF           # bật cả A1–A8
py -3.12 uwb_command.py --port COM14 set-burst --base 2500 --slot 1000 --margin 1200 --gap 1500 --period 20000
py -3.12 uwb_command.py --port COM14 set-burst --mode sequential   # A/B với sơ đồ cũ
```

Giảm dần từng thông số và theo dõi: `burst_late` của từng anchor (DIAG_ANCHOR,
anchor lỡ slot), `burst_final_late` (DIAG_SYSTEM), tỷ lệ `SS_FALLBACK` và
`period_us`. Tăng lại khi các bộ đếm này bắt đầu tăng.

## 5. Telemetry và lệnh

Khung: `AA 55 | VER=1 | TYPE | LEN | SEQ | TIME | payload | CRC16-CCITT`.

| Type | Gói | Nhịp |
|---|---|---|
| 0x00 / 0x01 / 0x02 | INFO / RANGE / STATS | 1 Hz / mỗi chu kỳ / 1 Hz |
| 0x10 | RANGE_MEAS (từng phép đo) | cần UART ≥ 460800 baud; Tag_DevKit (460800) bật mặc định, Tag (115200) tắt; chỉ ở chế độ tuần tự |
| 0x18 | RANGE_BURST (cả chu kỳ burst, 22 + 8·n byte) | mỗi chu kỳ (Tag_DevKit: khoảng 50 Hz), cùng cờ feature với RANGE_MEAS; snapshot RANGE giảm còn 5 Hz |
| 0x11 / 0x12 | DIAG_ANCHOR / ANCHOR_INFO | 2 anchor/s / khi nhận TLV |
| 0x13 | CMD_ACK | trả lời lệnh |
| 0x14 | SNIFFER_FRAME | chỉ vai trò sniffer |
| 0x15 / 0x16 | DIAG_SYSTEM / DEVICE_INFO | 1 Hz / boot + 10 s |
| 0x17 | GATEWAY_HEALTH | ESP32-C3 phát, 1 Hz |
| 0x20 | CMD (host → TAG) | khi gửi |

Kênh lệnh có 16 lệnh (`uwb_cmd.h`; 0x10 `SET_BURST`). Lệnh đổi RF chỉ chạy khi TAG đã `PAUSE` và
chưa `LOCK`; đổi sang giá trị RF mới tự xoá cờ calibration. Cấu hình được lưu
trong một snapshot NVS schema v2 có CRC + generation; calibration chỉ được nạp
khi PARTID và RF/PHY profile ID khớp.

```powershell
Set-Location ..\Software\UWB_UART_GUI
py -3.12 uwb_command.py --port COM7 info
py -3.12 uwb_command.py --port COM7 set-cal 1 12.5 --save
```

## 6. Cờ RF — mặc định giữ nguyên hành vi cũ

| Cờ | Mặc định | Ý nghĩa khi bật |
|---|---|---|
| `UWB_DW_REFERENCE_TUNING` | `0` | LDE NTM = 13, LDOTUNE + crystal trim từ OTP |
| `UWB_TX_POWER_MODE` | `LEGACY` | REFERENCE / SMART / CUSTOM theo DW1000 UM |
| `UWB_USE_CLOCK_CORRECTION` | `0` | bù trôi clock cho SS-TWR (hệ số đã sửa dấu) |
| `UWB_FILTER_FPP_COMPAT_DB` | `-12.04f` | giữ thang FPP cũ cho bộ lọc sau khi sửa lỗi RXPACC |
| `UWB_DS_CALIBRATED_MASK` | `0` | chỉ mở sau khi hiệu chuẩn từng anchor |

Đổi bất kỳ cờ nào ở ba dòng đầu đều làm lệch bias ⇒ phải hiệu chuẩn lại.
Quy trình A/B và tiêu chí nghiệm thu: `docs/HARDWARE_AB_CHECKLIST.md`.

## 7. Tương thích phần cứng

- DW1000 dùng SPI2 và chân cố định của module DWM1001C; LED trạng thái P0.12
  (active-high) tắt khi không có peer, sáng trong 1 s sau exchange thành công.
- TAG DevKit dùng D9 xanh cho liên kết UWB, D8 đỏ cho fault và D11 xanh dương
  cho heartbeat/lệnh GUI.
- TAG (PCB) dùng UART0 115200 baud sang ESP32-C3; anchor tắt UART0.
- Tag_DevKit dùng UARTE 460800 qua J-Link VCOM để mang RANGE_MEAS; GUI và
  `uwb_command.py` mặc định 460800. Ở 1 Mbaud J-Link VCOM làm sai CRC ~70 % byte.
- SPI1 ngoài tắt ở mọi vai trò để không chạm các đường R9–R12 sang ESP32-C3.
- ESP GPIO10 là đầu vào `ESP_IRQ/RDY`, không dùng làm LED.
- Sniffer_DevKit dùng UARTE 460800 (DWM1001-DEV, J-Link VCOM; từ 0.5, trước đó
  1 Mbaud làm hỏng dữ liệu). Mang được ~750 frame/s, trong khi burst phát
  ~2200 frame/s: khi sniff, giảm nhịp TAG bằng `set-burst --period 20000`.
  `tools/uwb_sniffer.py` giải mã khung v3 và báo offset RESP từng anchor.
  Chưa kiểm trên phần cứng.

Chi tiết chân và quyết định thiết kế: `docs/HARDWARE_COMPATIBILITY.md`.

## 8. Calibration — vẫn fail-closed

Antenna delay/offset của hệ STM32 + DW1000 rời **không** dùng lại được cho
DWM1001C. Firmware dùng hardware antenna delay, tắt legacy offset, và
`UWB_DS_CALIBRATED_MASK = 0`: TAG vẫn phát đủ chẩn đoán nhưng đánh dấu range là
chưa hợp lệ. Bảng calibration nay sửa được lúc chạy bằng lệnh `SET_DS_CAL` và
lưu vào NVS, không cần build lại. Ba key schema cũ (`uwb/radio`, `uwb/cal`,
`uwb/tag`) bị bỏ qua fail-closed; `FACTORY_RESET` sẽ dọn cả schema cũ và mới.

Không bật mask và không đưa range vào bộ điều khiển bay trước khi hoàn tất
`docs/HARDWARE_AB_CHECKLIST.md`.

## 9. Tài liệu liên quan

| File | Nội dung |
|---|---|
| `release/highrate_200/README.md` | bộ thử nghiệm mục tiêu ≥ 200 Hz/tám anchor, SPI trực tiếp, DS-only và hướng dẫn bench |
| `CHANGELOG.md` | nội dung bản nâng cấp v2, thứ tự flash, tương thích |
| `docs/HARDWARE_AB_CHECKLIST.md` | quy trình A/B trên board thật và tiêu chí nghiệm thu |
| `docs/ANCHOR_DEPLOYMENT.md` | gán module, flash, kiểm tra địa chỉ, calibration |
| `docs/DEPLOYMENT_MANIFEST.md` | bảng truy vết module ↔ vai trò ↔ firmware đang nạp |
| `docs/HARDWARE_COMPATIBILITY.md` | sơ đồ chân, quyết định thiết kế |
| `docs/FIRMWARE_TEST_CHECKLIST.md` | checklist kiểm thử firmware trên bench |
| `../Plan/KE_HOACH_NANG_CAP_TOAN_DIEN_FIRMWARE_DWM1001.md` | kế hoạch GĐ0–GĐ6 |
