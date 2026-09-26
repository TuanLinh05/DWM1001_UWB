# Nhật ký nâng cấp firmware — v2 (2026-09-20)

Bản nâng cấp này bám theo `Plan/KE_HOACH_NANG_CAP_TOAN_DIEN_FIRMWARE_DWM1001.md`
(GĐ0–GĐ4). Nguyên tắc xuyên suốt: **mọi thay đổi ảnh hưởng RF đều nằm sau cờ
biên dịch và mặc định giữ nguyên hành vi cũ**, calibration vẫn fail-closed
(`UWB_DS_CALIBRATED_MASK = 0`), nên firmware mới không tự ý đổi bias khoảng cách.

Chưa có thay đổi nào được nạp lên phần cứng: toàn bộ kiểm chứng dưới đây chạy
trên PC (host test + build Zephyr). Xem `HARDWARE_AB_CHECKLIST.md` cho quy trình
nghiệm thu trên board thật.

---

## 0. Cập nhật sau bản v2

### 0.4 — 2026-09-26: Tag_DevKit UART 460800 thay cho 1 Mbaud

Chỉ đổi `Tag_DevKit/app.overlay` (`current-speed = <460800>`), chú thích trong
`uwb_app_config.h`, baud mặc định của GUI 1.4.2 và `uwb_command.py`. Không đổi
RF hay giao thức; mục 0.2 bị thay thế ở phần chọn baud.

**Lý do (hai log phần cứng 2026-09-26, TAG DevKit + 8 anchor):**

- PC nhận khoảng 18,9 kB/s, gần bằng lượng TAG gửi (khoảng 20 kB/s), nhưng
  72 % byte bị parser loại vì sai CRC. Kết quả giống hệt sau khi đã tắt MSD của
  J-Link OB-STM32F072 (`MSDDisable`).
- Frame dài hoặc gửi liền nhau không bao giờ tới: 0 snapshot RANGE, 0 INFO,
  0 DIAG trong 128 s; RANGE_MEAS lọt khoảng 33 %. Bản ghi của A8 luôn đi ngay
  sau snapshot nên mất 100 %, khiến A8 trông như không có tín hiệu.
- Phía UWB bình thường: 234 exchange OK/s = 8 anchor × 29,3 Hz, 26 timeout
  từ lúc TAG boot; A5–A7 (STM32) hoàn tất DS-TWR 99,7–99,9 %.

Tám anchor ở chu kỳ 29 Hz tạo khoảng 20 kB/s, bằng 44 % dung lượng của 460800.
nRF52 chạy mức này ở 457143 baud thực, J-Link khoảng 461538: lệch khoảng 1 %.

Firmware Tag_DevKit cũ vẫn phát 1 Mbaud: phải chọn 1000000 trong GUI tới khi
nạp bản mới. `Sniffer_DevKit` vẫn 1 Mbaud và chưa kiểm trên phần cứng.

### 0.3 — 2026-09-25: LED trạng thái anchor chẩn đoán được

Chỉ đổi `common/src/app/main_anchor.c`; không đổi RF, giao thức hay timing
ranging. Lần nạp thử đầu tiên, bốn anchor STM32 A5–A8 đều không nháy LED. Với
quy ước cũ, điều đó không phân biệt được "firmware không chạy", "không nghe
được POLL" và "có POLL nhưng DS-TWR không hoàn tất": LED chỉ sáng khi có
REPORT, còn lại tắt, và không có tín hiệu khởi động như firmware CubeIDE cũ.

| LED | Nghĩa |
|---|---|
| 3 chớp nhanh khi khởi động | Firmware chạy; lặp lại = reboot loop |
| Sáng liên tục | Exchange DS-TWR hoàn chỉnh trong 1 s gần nhất (như cũ) |
| Nháy 1 Hz | Có POLL cho anchor này (`anchor_stats.polls` tăng) nhưng không hoàn tất |
| Chớp ngắn 50 ms mỗi 2 s | Radio đang nghe, chưa có POLL cho anchor này |
| Nháy nhanh 150 ms | Radio fault (như cũ) |

Ghi LED chỉ khi mức thay đổi, nên vòng lặp anchor không thêm truy cập GPIO mỗi
lượt. Chớp khởi động (0,72 s) chạy trước khi watchdog bật. Áp cho cả A1–A4
nếu build lại, nhưng không bắt buộc nạp lại A1–A4.

### 0.2 — 2026-09-25: UART 1 Mbaud và RANGE_MEAS trên TAG DevKit

Giống mục 0.1, đây là thay đổi hành vi có chủ đích và **chỉ áp cho TAG DevKit**.

**Lý do.** `RANGE_MEAS` (0x10) gửi một frame 66 byte cho mỗi phép đo: khoảng
300–400 frame/s với tám anchor, cộng snapshot 50 Hz, tổng cỡ 30 kB/s. UART 115200
chỉ mang được 11,5 kB/s, nên firmware từ chối RANGE_MEAS dưới
`TELEM_RANGE_MEAS_MIN_BAUD` (460800). Đây là mục §8.10 (V10) trong
`Plan/BAO_CAO_KIEM_TRA_FIRMWARE_V2_2026-09-24.md`.

**Vì sao chọn 1 000 000 baud, không phải 921600 như §8.10 viết:**

- nRF52 không tạo được đúng 921600: giá trị `Baud921600` của thanh ghi BAUDRATE
  thực chạy 941176 baud (+2,1 %), còn 1 000 000 là chính xác;
- `Sniffer_DevKit` đã chạy UARTE 1 Mbaud qua đúng J-Link VCOM này của DWM1001-DEV.

**Firmware (chỉ `Tag_DevKit`):**

- `app.overlay`: UART0 dùng `nordic,nrf-uarte` (EasyDMA), `current-speed = <1000000>`,
  giống hệt khối UART của `Sniffer_DevKit`. Driver UARTE nạp tới 32 byte cho mỗi
  ngắt TX, thay cho 1 byte mỗi ngắt của UART cũ (ở 30 kB/s: khoảng 1000 thay vì
  30 000 ngắt/s). `uart_tx_zephyr.c` đã hỗ trợ sẵn cả hai driver.
- `include/uwb_app_config.h`: `UWB_TELEM_DEFAULT_FEATURES` =
  SNAPSHOT | RANGE_MEAS | DIAG.
- Settings đã `SAVE_SETTINGS` vẫn thắng giá trị mặc định (thiết kế sẵn có của
  `uwb_settings.c`). TAG từng lưu settings với firmware cũ sẽ khởi động **không**
  có RANGE_MEAS. GUI tự bật lại cho phiên; lưu hẳn bằng
  `uwb_command.py --port COMx set-telemetry --snapshot --meas --diag --save`
  (hoặc `factory-reset`, nhưng lệnh này xoá cả calibration).

**PC (`Software/UWB_UART_GUI`, bản 1.3.0):**

- Tab mới **RANGE_MEAS**, logic nằm trong `meas_tracker.py` (không phụ thuộc Tk).
  - Mỗi anchor một dòng: tần số đo thật, mode, status, raw/corrected/FW filter,
    nhiễu, FP/RX, NLOS Δ (RX − FP), Anchor Δ, clock offset, `std_noise`, slot, tuổi.
  - Màu dòng theo NLOS/CAL_MISSING/stale.
  - Dòng tổng hợp: meas/s, số bản ghi mất (khe hở `meas_seq`), queue drop,
    UART TX overflow và đỉnh bộ đệm TX của TAG.
  - Biểu đồ từng phép đo theo đồng hồ TAG (khoảng cách + NLOS Δ).
  - Nút bật/tắt RANGE_MEAS, luôn giữ snapshot cho tab Live.
- Baud mặc định của GUI và `uwb_command.py` là 1000000. Gateway ESP32-C3 (USB
  Serial/JTAG) bỏ qua baud; Tag PCB nối USB-UART trực tiếp phải chọn 115200.
- `SerialWorker`:
  - khi `DEVICE_INFO` báo UART ≥ 460800 mà RANGE_MEAS đang tắt, tự bật cho phiên
    (ô "Tự bật khi kết nối");
  - lệnh từ GUI đi qua hàng đợi, luồng đọc vẫn là nơi duy nhất ghi cổng serial;
  - sau 3 s không có frame hợp lệ thì ghi gợi ý kiểm tra baud.
- Recorder: thêm `meas.csv` (một dòng mỗi bản ghi; công suất không đo được để
  trống) và `meas_records` trong `session.json`.
- `uwb_command.py set-telemetry` có thêm `--save`.
- Demo phát `DEVICE_INFO`, RANGE_MEAS (A3 bị NLOS 6 s trong mỗi 20 s) và trả lời
  `SET_TELEMETRY`.

**Không đổi:**

- `Tag` (PCB) và gateway ESP32-C3 vẫn 115200, UART legacy, mặc định không
  RANGE_MEAS. Build lại image `Tag` cho features khởi động 0x05.
- Giao thức, khung frame, anchor, PHY, calibration.

**Test:**

- `tests/test_node_projects.py`: TAG bật sẵn RANGE_MEAS thì UART phải ≥ 460800,
  và mọi UART trên 115200 phải là UARTE. Hai mutation tương ứng đều bị bắt.
- GUI:
  - `tests/test_meas_tracker.py` (mới);
  - thêm test cho SerialWorker, tab RANGE_MEAS, demo, recorder, encoder RANGE_MEAS
    và đối chiếu hằng số với header firmware;
  - tổng 66 test Python, 12 mutation đều bị bắt;
  - `test_frozen_application_directory_is_executable_parent` chỉ đúng trên
    Windows và vẫn lỗi trên Linux, giống hệt ở `06f72de`.
- Runner: 16 bước C/Python của firmware đạt, gateway parser đạt.
- Tải GUI ở 408 meas/s + 50 snapshot/s, tab RANGE_MEAS đang hiển thị và đang ghi:
  - luồng GUI bận 9,5 %;
  - lượt poll dài nhất 10,5 ms (nhịp 40 ms);
  - recorder ghi 3200/3200 bản ghi, 0 drop.
- Build Zephyr Tag_DevKit, 0 cảnh báo:
  - devicetree `nordic,nrf-uarte` @ 1000000, `CONFIG_UART_NRFX_UARTE=y`;
  - `Telem_UartBaud()` = 1000000;
  - `s_features` khởi động = 0x07; `factory-reset` cũng về 0x07.

**Nạp:** chỉ cần nạp lại TAG DevKit, chưa thử trên phần cứng. Nghiệm thu theo §8.10
của báo cáo:

- chạy 10 phút;
- ở "Hiện chi tiết", CRC lỗi và byte bỏ qua không tăng;
- tab RANGE_MEAS báo mất 0 và đỉnh TX < 50 %.

### 0.1 — 2026-09-24: bộ lọc range của TAG DevKit

Mục này là ngoại lệ có chủ đích với nguyên tắc "mặc định giữ nguyên hành vi cũ":
**hành vi của TAG DevKit thay đổi**. Lý do và số liệu nằm trong
`Plan/BAO_CAO_KIEM_TRA_FIRMWARE_V2_2026-09-24.md`.

**Tóm tắt lý do.** Test chuyển động cho thấy bộ lọc Legacy (median-3 + Kalman
tĩnh) khi TAG di chuyển 0,5–2 m/s:

- trễ 0,75–2,9 s;
- chỉ 13–19 % range được đánh dấu hợp lệ.

**Thay đổi:**

- `Tag_DevKit/include/uwb_app_config.h`: bộ lọc chuyển sang **MEDIAN_GATE** (`UWB_RANGE_FILTER_MODE = 1U`). Có `#ifndef` để build A/B vẫn chọn lại được Legacy (`-DUWB_RANGE_FILTER_MODE=0U`) hoặc CV (`2U`).
- `common/src/filters/range_filter.c`: tái bắt khoá có nhận biết NLOS, dùng cho MEDIAN_GATE và CV_KALMAN_V2. Cụ thể:
  - track bắt đầu lại từ nhóm mẫu **ngắn nhất** gần đây, không từ mẫu đầu tiên sau khi mất tín hiệu;
  - nhóm dài hơn track (hình dạng của NLOS) phải có đủ 5 mẫu khớp nhau;
  - mẫu LOS bị loại được giữ lại qua các mẫu NLOS được chấp nhận;
  - cổng FPP chỉ áp khi đang bám, nên liên kết yếu vẫn khởi động được.
- Tác dụng phụ đã biết: range hợp lệ đầu tiên sau khi bật máy hoặc sau khi mất tín hiệu đến chậm hơn khoảng 60–80 ms (4 mẫu thay vì 1).

**Không đổi:**

- anchor A1–A8: chỉ khác git hash nhúng trong image;
- project `Tag` (PCB riêng): vẫn Legacy;
- PHY, TX power, giao thức, calibration (vẫn fail-closed).

**Test:**

- `tests/test_range_filter.c` (mới): 7 quy tắc tái bắt khoá, chạy ở mode 1 và 2.
- `tests/test_tag_motion.c`: 14 kịch bản chuyển động, có `--seeds N`.
- `run_host_tests.ps1`:
  - 16 bước test C/Python của firmware đạt, gateway parser cũng đạt;
  - bước mặc định (MEDIAN_GATE) đạt 14/14 kịch bản, kể cả khi chạy 20 seed;
  - Legacy chỉ còn chạy để khảo sát (`--report`).
- Build Zephyr: 10 image, 0 cảnh báo.

**Nạp:** chỉ cần nạp lại TAG DevKit. Chưa thử trên phần cứng. Trước khi bay,
làm hai bài T9 và T10 trong báo cáo: di chuyển có ground truth, và che chắn
NLOS.

---

## 1. Thứ tự flash bắt buộc

Khung tin trên không trung lên **v2** (có version + transaction id). Anchor v2
hiểu cả POLL v1 lẫn v2, nhưng anchor v1 **không** hiểu POLL v2.

```
1. Flash 8 Anchor trước  (Anchor_1 .. Anchor_8)
2. Flash TAG sau
3. Flash Sniffer_DevKit (tuỳ chọn, chỉ để quan sát)
4. Flash ESP32-C3 gateway (nếu dùng RANGE_MEAS / lệnh từ host)
```

Nếu flash TAG trước, các anchor chưa nâng cấp sẽ im lặng cho tới khi được flash.
Rollback đi theo chiều ngược lại: TAG về v1 trước, rồi tới anchor.

## 2. Cấu trúc mã nguồn

| Trước | Sau |
|---|---|
| Mỗi project có bản sao riêng của `dw1000.c`, `anchor_ranging.c`, header… | Nguồn dùng chung ở `Firmware/common/{include,src}` |
| 11 bản sao dễ lệch nhau | Mỗi project chỉ còn `CMakeLists.txt`, `prj.conf`, `app.overlay`, `include/uwb_app_config.h`, `scripts/` |
| Vai trò xác định bằng file khác nhau | `uwb_node_setup(TAG / ANCHOR / SNIFFER)` trong `common/cmake/uwb_node.cmake` |
| Không biết firmware đang chạy từ commit nào | Git hash + cờ dirty biên dịch vào firmware, phát trong `DEVICE_INFO` và TLV của anchor |

Project mới: `Sniffer_DevKit/` — DWM1001-DEV chỉ nghe, đẩy mọi frame kèm
timestamp DW1000 lên UART 1 Mbaud cho `tools/uwb_sniffer.py`.

`test_node_projects.py` khoá cấu trúc này: không project nào được tái lập
`src/` riêng hay bản sao header dùng chung.

## 3. Driver DW1000

- **Một truy cập = một giao dịch SPI.** Bản cũ tách header và dữ liệu thành hai
  giao dịch; với DW1000 đó là hai lần chọn chip, dễ đọc rác khi bus bận.
- **Sửa lỗi đọc RXPACC**: bản cũ lấy bit [27:18] của `RX_FINFO` thay vì
  [31:20], tức RXPACC nhỏ đi 4 lần → First-Path Power báo thấp hơn thực tế
  ≈ 20·log10(4) = **12,04 dB**. Telemetry nay phát FPP/RX power đúng và bật cờ
  `INFO.flags bit 0x20` để host biết thang đo đã đổi.
- `DW1000_ClassifyRx()` phân loại RX (OK / CRC / RXPHE / RXSFDTO / RXRFTO /
  PREJ…) thay vì chỉ "không nhận được".
- Đọc OTP: PARTID, LOTID, LDOTUNE, VBAT, VTEMP, XTRIM — luôn đọc và báo cáo.
- `UWB_DW_REFERENCE_TUNING` (mặc định **0**): khi bật mới áp dụng LDE NTM = 13
  (`LDE_CFG1 = 0x6D`), LDOTUNE từ OTP và crystal trim từ OTP.
- `UWB_TX_POWER_MODE` (mặc định **LEGACY** = `0x1E1E1E1E`, smart TX off) —
  thêm REFERENCE / SMART / CUSTOM. Đổi profile làm lệch bias ⇒ phải calibration lại.
- `DW1000_VerifyConfig()` kiểm tra thêm nhiều thanh ghi (bitmask `DW_VERIFY_*`),
  chạy định kỳ trong cửa sổ an toàn; HPDWARN được kiểm tra sau mỗi delayed TX.

## 4. Giao thức trên không (frame v2)

- POLL / RESP / FINAL / REPORT mang thêm `version` + `txn` (transaction id).
  Gói trễ của chu kỳ trước bị loại thay vì tạo khoảng cách sai.
- RESP/REPORT có TLV: vị trí anchor, build id, FP/RX power đo tại anchor.
- Anchor trả lời đúng phiên bản của POLL nhận được (v1 hoặc v2).
- Anchor dùng WAIT4RESP + delayed TX, có cửa sổ im lặng khi cần.

## 5. TAG

- **F1** — chống đói anchor offline: probe xoay vòng, một anchor hỏng không còn
  chiếm toàn bộ thời gian chờ của chu kỳ.
- Timeout REPORT → tự rơi về SS-TWR cho chu kỳ đó; lỗi RX → soft reset RX.
- Bảng calibration **runtime** (`Tag_SetDsCalibration`) thay cho hằng số biên
  dịch; mask vẫn mặc định 0 (range đánh dấu không hợp lệ cho tới khi đo).
- Active anchor mask, pause/resume, `Tag_RecoverRadio`.
- Hàng đợi phép đo → telemetry `RANGE_MEAS` (từng phép đo, tuỳ chọn).
- Bộ lọc host/firmware giữ nguyên hành vi đã hiệu chỉnh: FPP đưa vào bộ lọc
  được bù `UWB_FILTER_FPP_COMPAT_DB = -12.04 dB`, còn telemetry phát giá trị đúng.

## 6. Sức khoẻ hệ thống, cấu hình, lệnh

- `uwb_health`: watchdog **theo tiến độ** (không chỉ "còn sống"), recovery theo
  bậc 20/100/500 ms, tối đa 3 lần reboot do lỗi (đếm trong RAM `__noinit`), sau
  đó giữ trạng thái fault và thử lại mỗi 10 s thay vì reboot vô hạn.
- `uwb_settings`: lưu radio + calibration + TAG config trong **một snapshot
  schema v2** tại `uwb/config`, có CRC32, generation, PARTID và RF/PHY profile
  ID. Calibration không khớp bị từ chối fail-closed; factory reset tổng hợp lỗi
  xóa thay vì ACK thành công khi còn dữ liệu cũ.
- `uwb_cmd`: 15 lệnh (`PING`, `GET_DEVICE_INFO`, `PAUSE`, `RESUME`,
  `SET_ANCHOR_MASK`, `SET_DS_CAL`, `GET_DS_CAL`, `SET_ANT_DELAY`,
  `SET_TX_POWER`, `SAVE_SETTINGS`, `FACTORY_RESET`, `REBOOT`, `TIME_SYNC`,
  `SET_LOCK`, `SET_TELEMETRY`) với 9 mã kết quả. Lệnh đổi RF chỉ chạy khi TAG
  đã `PAUSE` và chưa `LOCK`; thay đổi thành công tự vô hiệu hóa calibration cũ.
- Bật detector `EC_CTRL_PLLLCK` như driver tham khảo Decawave. `CPLOCK` là cờ
  sự kiện W1C: firmware không còn xóa một lock hợp lệ rồi bắt phần cứng phát
  lại cạnh khi ghi cùng PLL config. Nếu chưa có lock thì vẫn chờ tối đa 100 ms
  và init/recovery không tiếp tục khi PLL thực sự chưa khóa.
- LED không còn toggle trực tiếp trong state machine ranging. LED trạng thái
  chỉ sáng trong 1 s sau một exchange thành công; khi không có peer thì tắt.
  Board TAG DevKit dùng thêm D8 đỏ cho fault và D11 xanh dương cho host/GUI.

## 7. Telemetry

Khung giữ nguyên `AA 55 | VER | TYPE | LEN | SEQ | TIME | payload | CRC16`, nên
gateway và GUI cũ vẫn đọc được các gói cũ. Loại mới:

| Type | Tên | Nhịp |
|---|---|---|
| 0x10 | RANGE_MEAS | từng phép đo (cần UART ≥ 460800 baud, mặc định tắt; Tag_DevKit bật từ mục 0.2) |
| 0x11 | DIAG_ANCHOR | xoay vòng 2 anchor/s |
| 0x12 | ANCHOR_INFO | khi nhận được TLV từ anchor |
| 0x13 | CMD_ACK | trả lời lệnh host |
| 0x14 | SNIFFER_FRAME | chỉ vai trò sniffer |
| 0x15 | DIAG_SYSTEM | 1 Hz |
| 0x16 | DEVICE_INFO | lúc boot và mỗi 10 s; thêm config hash + calibration profile ID |
| 0x17 | GATEWAY_HEALTH | ESP32-C3 phát, 1 Hz |
| 0x20 | CMD | host → TAG |

## 8. Gateway ESP32-C3

UART chuyển sang hướng sự kiện: FIFO overflow, buffer full, lỗi khung/parity
được **đếm** thay vì mất gói im lặng; thêm luồng USB → UART cho lệnh và gói
`GATEWAY_HEALTH` 1 Hz.

> ⚠️ Mã gateway **chưa được biên dịch** trong lần nâng cấp này vì máy phát triển
> không cài ESP-IDF. Chỉ bộ phân tích khung (`telemetry_protocol.c`) được kiểm
> thử trên host. Cần `idf.py build` trước khi nạp.

## 9. Công cụ trên PC

- `Software/UWB_UART_GUI/telemetry_protocol.py`: giải mã đủ 9 loại gói mới,
  `encode_command()`, `fpp_to_legacy_scale_cdbm()` để so sánh log cũ/mới.
- GUI phân biệt "mở được COM" với "TAG online", gửi `PING` 1 Hz để xác nhận
  đường RX/TX và tự bật lại `RANGE_SNAPSHOT` (không ghi NVS) nếu phiên trước đã
  tắt loại telemetry dùng cho đồ thị.
- `Software/UWB_UART_GUI/uwb_command.py`: CLI gửi lệnh xuống TAG.
- `Firmware/tools/uwb_sniffer.py`: dựng timeline khe thời gian, thống kê reply
  delay và thời lượng trao đổi từng anchor (dùng để chứng minh ngân sách 50 Hz).

## 10. Kiểm chứng đã chạy

```
Firmware\tests\run_host_tests.ps1      → 12/12 nhóm test đạt
Firmware\scripts\build_all.ps1         → 11/11 project build, 0 cảnh báo
```

Kích thước flash của vòng build sạch 2026-09-20 sau sửa link/LED: TAG 57 560 B,
Tag_DevKit 57 672 B, Sniffer_DevKit 33 676 B, Anchor_1 33 512 B và
Anchor_2–Anchor_8 33 512 B mỗi bản.

Các test host chạy **chính mã sản phẩm** trên một bộ mô phỏng thanh ghi DW1000
(`tests/dw1000_sim.c`): máy trạng thái TAG/anchor, công thức DS-TWR (khớp lý
thuyết trong ±1 mm, kể cả khi lệch clock +20 ppm), bộ thực thi lệnh và bộ mã hoá
telemetry (đối chiếu chéo với bộ giải mã Python bằng file golden).

## 11. Việc còn lại trước khi bay

Xem `HARDWARE_AB_CHECKLIST.md`. Tóm tắt: đo calibration từng anchor, A/B từng cờ
RF, xác nhận ngân sách thời gian 8 anchor ở 50 Hz bằng sniffer, rồi mới bật
`UWB_DS_CALIBRATED_MASK`.
