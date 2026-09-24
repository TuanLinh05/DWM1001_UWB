# Nhật ký nâng cấp firmware — v2 (2026-09-20)

Bản nâng cấp này bám theo `Plan/KE_HOACH_NANG_CAP_TOAN_DIEN_FIRMWARE_DWM1001.md`
(GĐ0–GĐ4). Nguyên tắc xuyên suốt: **mọi thay đổi ảnh hưởng RF đều nằm sau cờ
biên dịch và mặc định giữ nguyên hành vi cũ**, calibration vẫn fail-closed
(`UWB_DS_CALIBRATED_MASK = 0`), nên firmware mới không tự ý đổi bias khoảng cách.

Chưa có thay đổi nào được nạp lên phần cứng: toàn bộ kiểm chứng dưới đây chạy
trên PC (host test + build Zephyr). Xem `HARDWARE_AB_CHECKLIST.md` cho quy trình
nghiệm thu trên board thật.

---

## 0. Cập nhật 2026-09-24: bộ lọc range của TAG DevKit

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
| 0x10 | RANGE_MEAS | từng phép đo (cần UART ≥ 460800 baud, mặc định tắt) |
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
