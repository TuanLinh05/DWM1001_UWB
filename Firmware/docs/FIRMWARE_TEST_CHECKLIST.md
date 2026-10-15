# Checklist kiểm thử firmware UWB — burst v3, 1 TAG, 8 Anchor

Tài liệu này dùng cho repository `Github Clone UWB Test`, nhánh firmware hiện
tại có giao thức DS-TWR tuần tự v2 và burst DS-TWR một-nhiều v3.

Phạm vi:

- TAG DWM1001C hoặc DWM1001-DEV;
- A1–A4: DWM1001C/nRF52832;
- A5–A8: STM32F103C8T6 + DW1000;
- GUI/logger Python và gateway ESP32-C3 nếu có;
- nghiệm thu firmware trên PC, bench HIL và độ chính xác tĩnh.

Tài liệu này bổ sung cho `HARDWARE_AB_CHECKLIST.md`. Phần timing 50 Hz trong
checklist cũ mô tả đường tuần tự; khi chạy burst v3 phải dùng quy trình H-07,
H-08 và H-12 dưới đây.

> **Cảnh báo:** build/test thành công không có nghĩa hệ thống đã sẵn sàng bay.
> Không đưa range vào bộ điều khiển khi calibration còn thiếu, telemetry mất
> gói, chưa thử đủ tám Anchor hoặc chưa hoàn tất các cổng nghiệm thu cuối.

---

## 1. Chọn profile cần nghiệm thu

Không trộn log của hai profile trong cùng một kết luận.

| Profile | TAG | Giao thức mặc định | UART | Mục đích |
|---|---|---|---:|---|
| `DEVKIT_BURST` | `Tag_DevKit` | burst v3 | 460800 | Bring-up và tune burst |
| `DRONE_SEQUENTIAL` | `Tag` PCB | tuần tự v2 | 115200 | Cấu hình production hiện tại |
| `DRONE_BURST` | `Tag` PCB | burst v3 | phải xác nhận lại | Chỉ dùng sau khi link TAG→gateway mang đủ telemetry |

Trước khi test, ghi rõ:

| Trường | Giá trị |
|---|---|
| Profile | |
| Git branch / commit | |
| Git dirty | `0` / `1` |
| TAG project | |
| Cổng UART / baud | |
| Gateway | không dùng / ESP32-C3 |
| RF profile | Ch5, PRF16, preamble 128 hoặc giá trị khác |
| Burst base / slot / margin / gap / period | |
| Active mask | |
| Calibration profile ID | |
| Người chạy / ngày giờ | |

### Quy tắc đọc kết quả

- Trước calibration, `valid=0` hoặc `CAL_MISSING` là hành vi fail-closed đúng.
  Ở L1 phải chấm theo `RADIO_OK`, mode, timeout và diagnostic; không dùng
  `valid` làm tiêu chí link.
- Kiểm tra độ chính xác bằng `raw_mm` hoặc `corrected_mm`; không dùng output đã
  lọc làm bằng chứng cho độ chính xác RF.
- Không xóa outlier trước khi chấm. Báo riêng median, mean, sigma, P95/P99,
  tỷ lệ outlier và chuỗi mất dữ liệu dài nhất.
- Mỗi lần chỉ đổi một tham số. Sau khi đổi RF, TX power, antenna delay,
  preamble hoặc module vật lý phải calibration lại.
- Cấp nguồn tối thiểu 2 phút trước khi ghi log đo chính xác.
- Luôn giữ nguyên log gốc, `raw_telemetry.bin`, `session.json` và thông tin
  build/config hash.

### Trạng thái test

- `PASS`: đạt đủ mọi điều kiện bắt buộc.
- `FAIL`: vi phạm ít nhất một điều kiện.
- `BLOCKED`: firmware/tool hiện tại chưa có instrumentation cần thiết.
- `INFO`: bài đặc tính, chưa có ngưỡng nghiệm thu được phê duyệt.

---

## 2. Thiết bị và bố trí

### Thiết bị tối thiểu

- [ ] TAG và đủ A1–A8 đúng phần cứng.
- [ ] J-Link cho DWM1001C/DWM1001-DEV.
- [ ] ST-Link cho A5–A8 STM32F103.
- [ ] USB-UART/gateway đúng loại sẽ dùng thật.
- [ ] Máy đo laser sai số không quá 2 mm.
- [ ] Giá đỡ gỗ/nhựa cao 1,2–1,5 m.
- [ ] Logic analyzer tối thiểu 8 MHz.
- [ ] Nguồn ổn định; nếu test drone, dùng cả nguồn/battery production.
- [ ] Nhãn vật lý A1–A8 và bảng `DEPLOYMENT_MANIFEST.md`.

### Setup chuẩn

- `BENCH-1`: một TAG và một Anchor, LOS, cùng độ cao, cách 1/3/5/10/15 m tùy
  test; trong bán kính 1 m không có người hoặc kim loại.
- `BENCH-8`: TAG ở giữa, đủ tám Anchor xung quanh; ghi tọa độ và chiều cao tâm
  anten. Không thay đổi vị trí trong một nhóm so sánh.
- Anten phải giữ đúng hướng dự kiến khi lắp thật. Với bài so sánh A/B, đánh dấu
  vị trí chân giá và góc anten.

### Cấu trúc log đề xuất

Sau khi GUI dừng và lưu, đặt tên hoặc ghi chú session theo mẫu:

```text
YYYYMMDD_HHMM_<TEST-ID>_<PROFILE>_<LABEL>
```

Ví dụ:

```text
20260930_0930_H12_DEVKIT_BURST_bench8_default
20260930_1400_A02_DEVKIT_BURST_5m
```

Mỗi session phải còn tối thiểu:

- `session.json`, `device_info.csv`, `anchor_info.csv`;
- `diag_system.csv`, `diag_anchor.csv`, `uart.csv`;
- `burst.csv` hoặc `meas.csv`;
- `summary.csv`, `events.log`, `raw_telemetry.bin`;
- `anchor_layout.json` nếu test position.

---

## 3. L0 — Kiểm thử trên PC và build

### H-00 · Host tests

```powershell
Set-Location 'D:\Drone Project\Github Clone UWB Test\Firmware'
git status --short --branch
git rev-parse --short HEAD
.\tests\run_host_tests.ps1
```

Nếu chưa cần parser gateway:

```powershell
.\tests\run_host_tests.ps1 -SkipGateway
```

**PASS khi:**

- [ ] Tất cả nhóm test được chạy đều PASS.
- [ ] `test_tag_burst` PASS cho tám Anchor và clock offset mô phỏng.
- [ ] `test_telemetry_golden` C↔Python PASS.
- [ ] GUI tests và project-layout tests PASS.
- [ ] Không bỏ qua lỗi compiler/warning; script dùng `-Werror`.

### H-00B · Kiểm tra cấu hình và build image

```powershell
Set-Location 'D:\Drone Project\Github Clone UWB Test\Firmware'
py -3.12 .\tools\verify_stm32_images.py --source-only
.\scripts\build_all.ps1
.\scripts\build_stm32_anchors.ps1
py -3.12 .\tools\verify_stm32_images.py
.\scripts\package_8_anchors.ps1
```

**PASS khi:**

- [ ] Các project nRF build sạch.
- [ ] A5–A8 build và HEX qua checksum/vector/bounds/duplicate check.
- [ ] `release/anchor_8/manifest.csv` có đủ tám image và SHA-256.
- [ ] Build dùng cho nghiệm thu có `dirty=0`.
- [ ] `UWB_DS_CALIBRATED_MASK=0` nếu chưa provision calibration thật.

---

## 4. Chuẩn bị và flash phần cứng

### Thứ tự bắt buộc

1. [ ] Dán nhãn vật lý và ghi PARTID/LOTID nếu đọc được.
2. [ ] Flash A1–A4 bằng đúng project `Anchor_1..Anchor_4`.
3. [ ] Flash A5–A8 bằng đúng image `STM32_Anchor_5..8`.
4. [ ] Program & Verify từng image; BOOT0 STM32 = 0; reset sau flash.
5. [ ] Flash TAG sau khi tám Anchor đã tương thích cùng PHY/protocol.
6. [ ] Flash gateway nếu dùng.
7. [ ] Điền `DEPLOYMENT_MANIFEST.md` và SHA-256 image.

Không bật hai module có cùng `ANCHOR_ADDR` trong cùng PAN.

### Kết nối GUI

```powershell
Set-Location 'D:\Drone Project\Github Clone UWB Test\Software\UWB_UART_GUI'
.\run_gui.ps1
```

- `Tag_DevKit`: chọn 460800 baud.
- `Tag` PCB/gateway: chọn đúng cấu hình production.
- Dòng “đã mở COM” chưa đủ; phải thấy “TAG online” sau khi nhận frame hợp lệ.
- Bắt đầu recorder trước khi thay đổi setup nếu muốn ghi cả quá trình khởi động.

CLI dùng trong các test:

```powershell
$port = 'COM14'
py -3.12 .\uwb_command.py --port $port ping
py -3.12 .\uwb_command.py --port $port info
py -3.12 .\uwb_command.py --port $port get-cal
py -3.12 .\uwb_command.py --port $port set-burst
```

---

## 5. L1 — Bench HIL chạy được với firmware hiện tại

### H-01 · SPI, DEVID, identity và config readback

**Mục tiêu:** từng node khởi động được DW1000 ở tốc độ init và run, đúng image,
đúng địa chỉ, không tự recovery.

**Thực hiện:** cấp nguồn từng node riêng trong 30 giây, sau đó bring-up cùng TAG.

- [ ] LED khởi động đúng mô tả; không có pattern fault 150 ms.
- [ ] TAG báo đúng git hash, config hash, dirty, PARTID/LOTID, UART baud.
- [ ] `ANCHOR_INFO` báo đúng A1…A8, build/config hash và antenna delay.
- [ ] `recovery_count=0`, `fault_reboot_count=0`.
- [ ] Không có `CFG_DRIFT`, SPI error hoặc register verify mismatch.
- [ ] Bật một Anchor thì chỉ đúng ID đó xuất hiện.

**PASS:** toàn bộ node qua các mục trên. Một board sai ID, sai PHY hoặc cần
recovery là FAIL, không tiếp tục H-12.

### H-02 · Link một Anchor

**Mục tiêu:** xác nhận đầy đủ POLL→RESP→FINAL và pipeline Rb một chu kỳ.

Chọn A1 trước, sau đó lặp lại với ít nhất A5 để bao phủ hai MCU:

```powershell
py -3.12 .\uwb_command.py --port $port set-mask 0x01
```

Mask từng Anchor:

| Anchor | Mask |
|---|---:|
| A1 | `0x01` |
| A2 | `0x02` |
| A3 | `0x04` |
| A4 | `0x08` |
| A5 | `0x10` |
| A6 | `0x20` |
| A7 | `0x40` |
| A8 | `0x80` |

Ghi 300 giây tại 1 m.

**PASS mỗi Anchor:**

- [ ] Có ít nhất 1000 record `RADIO_OK`.
- [ ] Radio success ≥ 99%.
- [ ] DS ≥ 99%; SS fallback ≤ 1%.
- [ ] Timeout/RX error ngoài giai đoạn bật nguồn ≤ 1%.
- [ ] `burst_late=0`, `final_late=0`, queue drop=0, CRC=0.
- [ ] Raw range đổi đúng chiều khi di chuyển TAG gần/xa.

`CAL_MISSING` được phép ở H-02 nếu chưa calibration.

### H-04 · Range không phụ thuộc slot

**Mục tiêu:** cùng một A8 ở rank 0 và rank 7 phải cho cùng raw range.

1. Đặt A8 cố định ở 3,000 m; chỉ bật A8 hoặc dùng mask `0x80`.
2. Chờ 2 phút, ghi 60 giây với label `a8_rank0`.
3. Bật đủ A1–A8, đặt mask `0xFF`; không di chuyển A8 hoặc TAG.
4. Chờ 2 phút, ghi 60 giây với label `a8_rank7`.
5. So sánh `raw_mm`, không dùng filtered output.

**PASS:**

- [ ] Mỗi log có ≥1000 mẫu radio tốt.
- [ ] |median(rank7) − median(rank0)| ≤ 10 mm.
- [ ] Không tăng timeout/fallback riêng tại rank 7.
- [ ] Diagnostic xác nhận A8 thực sự ở slot cuối khi mask `0xFF`.

### H-07 · MCU chậm nhất ở rank 0

**Mục tiêu:** DWM1001C và STM32 đều kịp xử lý POLL khi không có thời gian chờ
từ các slot trước.

Với mỗi Anchor, đặt mask chỉ chứa Anchor đó để rank luôn bằng 0. Ưu tiên chạy
A1 và A5 trước; sau đó chạy đủ A1–A8, 300 giây/node.

**PASS mỗi node:**

- [ ] Radio success ≥ 99%.
- [ ] RESP timeout ≤ 0,5%.
- [ ] `burst_late` không tăng.
- [ ] `delayed_tx_late` không tăng.
- [ ] Không có radio recovery.

Nếu STM32 fail còn DWM1001C pass, tăng `base_uus`; không che lỗi bằng cách chỉ
đẩy STM32 sang slot cao.

### H-08 · Quét biên timing burst

**Mục tiêu:** tìm biên thực của base, slot, final margin và gap, sau đó chọn
giá trị có margin cho production.

Khôi phục baseline trước khi bắt đầu:

```powershell
py -3.12 .\uwb_command.py --port $port set-mask 0xFF
py -3.12 .\uwb_command.py --port $port set-burst --mode burst `
  --base 800 --slot 350 --margin 450 --gap 500 --period 0
```

Mỗi bước ghi ít nhất 60 giây. Không dùng một log cho nhiều cấu hình.

#### H-08A · Base

1. Đặt MCU chậm nhất ở rank 0.
2. Giảm `base`: 800 → 750 → 700… mỗi bước 50 UUS.
3. Dừng tại bước đầu tiên có `burst_late`, delayed-TX late hoặc success <98%.
4. Chọn base cuối = biên lỗi + ít nhất 50–100 UUS.

#### H-08B · Slot

1. Bật đủ A1–A8.
2. Giữ base an toàn.
3. Giảm `slot`: 350 → 330 → 310… mỗi bước 20 UUS.
4. Theo dõi timeout cụm, processing max và lỗi các Anchor sau.
5. Chọn slot cuối = biên lỗi + ít nhất 30–50 UUS.

Lưu ý: chu kỳ có `ANCHOR_INFO` sử dụng slot dài hơn. Cấu hình cuối phải qua cả
chu kỳ thường và chu kỳ info.

#### H-08C · Final margin

Giảm margin từng 20–25 UUS tới khi `tag_burst_final_late_count`, SS fallback
hoặc Anchor-missed-FINAL bắt đầu tăng. Chọn biên lỗi + ít nhất 50 UUS.

#### H-08D · Gap

Giảm gap từng 50 UUS. Theo dõi Anchor bỏ lỡ POLL kế tiếp, UART queue, record
drop và recovery. Chọn biên lỗi + ít nhất 100 UUS.

**PASS cấu hình cuối:**

- [ ] Mười phút BENCH-8 không có `burst_late` hoặc `final_late`.
- [ ] Radio success từng Anchor ≥98%.
- [ ] SS fallback ≤1%.
- [ ] Record/UART/recorder drop = 0.
- [ ] P99 `period_us` nằm trong ngân sách đã chọn.
- [ ] Ghi config cuối vào biên bản và chạy lại H-04, H-07, H-12.

Sau test, trả về config đã được chọn; reboot để xóa cấu hình runtime nếu cần.

### H-11 · UART, gateway và recorder

#### H-11A · Chạy ổn định

Ghi BENCH-8 liên tục tối thiểu 10 phút bằng đúng cáp, gateway, baud và máy tính
sẽ dùng thật.

**PASS:**

- [ ] Parser CRC error = 0.
- [ ] UART RX/TX overflow = 0.
- [ ] TAG `burst_record_drops=0`, telemetry queue drop=0.
- [ ] Recorder `queue_drops=0`, `error=null` trong `session.json`.
- [ ] `burst_delivered_pct ≥ 99,5%` ở profile burst.
- [ ] UART high-water <80% ring buffer.
- [ ] Không có boot ID mới hoặc recovery ngoài dự kiến.

#### H-11B · Mất/kết nối lại

Thực hiện ba lần. Nếu phần cứng cho phép, ngắt riêng đường UART TX khoảng 2
giây mà không reset TAG. Nếu USB/VCOM buộc reset board, ghi rõ đây là bài reboot,
không gọi là hotplug UART.

**PASS:** host tự đồng bộ lại frame, nhận `DEVICE_INFO`, không cần khởi động lại
GUI, và có dữ liệu hợp lệ trở lại trong tối đa 2 giây.

#### H-11C · Production 115200

Với `Tag` PCB, xác nhận firmware từ chối per-measurement telemetry vượt ngân
sách. Không dùng kết quả Tag_DevKit 460800 để nghiệm thu link production.

### H-12 · Đủ tám Anchor

**Setup:** timing đã chốt ở H-08, mask `0xFF`, BENCH-8, ghi 10 phút.

**PASS profile `DEVKIT_BURST`:**

- [ ] Đủ A1–A8, đúng build/config hash và không trùng ID.
- [ ] Radio success từng Anchor ≥98%.
- [ ] DS từng Anchor ≥99%; fallback ≤1%.
- [ ] `burst_late=0`, `final_late=0`.
- [ ] `burst_record_drops=0`, CRC/overflow/recorder drop=0.
- [ ] Với timing mặc định, median period mục tiêu ≤5500 µs; P99 ≤6500 µs.
- [ ] Không có recovery hoặc fault reboot.
- [ ] Tắt một Anchor không làm giảm nhịp các Anchor còn lại.
- [ ] Cắm lại Anchor được nhận trong ≤2 giây.

**Profile `DRONE_SEQUENTIAL`:** ghi riêng chu kỳ tám Anchor và tỷ lệ thành công.
Không tuyên bố đạt 220 Hz. Chu kỳ khoảng 29 Hz hiện chỉ là dữ liệu đặc tính;
việc đủ hay không đủ cho điều khiển phải do yêu cầu hệ thống quyết định.

---

## 6. L1 mở rộng — cần firmware HIL instrumentation

Các bài dưới đây là `BLOCKED` nếu build chưa có `UWB_HIL_RAW_TRACE` hoặc
`UWB_HIL_TEST`. Không thay thế chúng bằng filtered range rồi kết luận PASS.

### H-03 · Host tính lại Alt-DS từ timestamp thô

**Cần log:** txn, T1/POLL TX, T4/RESP RX, T5/FINAL TX, Da, Rb và CI.

Chạy với A1 DWM1001C và A5 STM32, mỗi log ≥1000 mẫu.

Host tính:

```text
Ra = T4 - T1
Db = T5 - T4
ToF = (Ra·Rb - Da·Db) / (Ra + Rb + Da + Db)
```

Mọi phép trừ phải mask 40-bit. Oracle phải dùng cùng quy tắc chuyển mm của
firmware, gồm truncate/round đúng như code.

**PASS:** 100% mẫu DS `RADIO_OK` lệch host↔firmware không quá 1 mm.

### H-05 · Wrap timestamp DW1000 40-bit

Ghi RAW tối thiểu 600 giây, đủ ≥20 lần wrap. Có thể giảm nhịp RAW xuống 20–50
Hz để giữ UART an toàn, nhưng không được dừng ranging burst.

**PASS:** trong ±1 chu kỳ quanh mỗi wrap:

- [ ] Không có range spike >5 robust sigma.
- [ ] Không tăng compute error, timeout hoặc txn mismatch.
- [ ] Host oracle H-03 vẫn khớp.

### H-06 · Miễn nhiễm clock drift

Dùng cùng một Anchor ở cùng vị trí, hai test build/override XTAL trim tạo chênh
clock ít nhất 20 ppm. Ghi 60 giây mỗi cấu hình sau khi ổn nhiệt.

**PASS:**

- [ ] |Δmedian DS| ≤10 mm.
- [ ] Carrier-integrator báo đúng chiều và độ lớn clock shift.
- [ ] SS control dịch theo dự đoán; nếu clock shift thực <10 ppm thì kết quả
  đối chứng là INCONCLUSIVE, không phải PASS.

### H-09 · Fault injection pipeline

Test profile đề xuất:

```text
skip RESP A3 khi txn % 7 == 0
skip FINAL khi txn % 11 == 0
A5 ignore FINAL khi txn % 13 == 0
```

Ghi 300 giây. Oracle của firmware hiện tại:

- skip RESP ở vòng `k`: record vòng `k−1` của A3 thành `SS_FALLBACK`; record
  vòng `k` của A3 thành `TIMEOUT`;
- skip FINAL vòng `k`: các Anchor có RESP ở vòng `k` thành `SS_FALLBACK`;
- A5 ignore FINAL vòng `k`: chỉ A5 fallback, các Anchor khác giữ DS;
- lỗi không được lan sang Anchor/vòng ngoài pattern.

**PASS:** ≥99% ô khớp oracle, lỗi ngoài pattern ≤1%, mỗi loại sự kiện ≥50.

### H-10 · Wrap txn 8-bit

Ghi đủ ≥100 lần actual txn 255→0. Ở 50 Hz cần khoảng 9 phút; ở 220 Hz thời
gian ngắn hơn nhưng vẫn nên chạy chung log 10 phút H-05/H-12.

**PASS:** không tăng txn mismatch, timeout, fallback hoặc range spike tập trung
quanh wrap.

---

## 7. L2 — Calibration và độ chính xác tĩnh

### A-01 · Calibration từng đường TAG–Anchor

Calibration phải thực hiện với đúng TAG, Anchor, antenna/enclosure, PHY và TX
power sẽ dùng thật. Không chuyển calibration từ Tag_DevKit sang Tag PCB.

Với mỗi Anchor A1…A8:

1. Chỉ bật/mask Anchor cần calibration.
2. Đặt tại 5,000 m LOS, đo tâm anten tới tâm anten.
3. Thu ít nhất 2000 raw sample.
4. Tính `bias = median(raw_mm) − truth_mm`.
5. Nhập bias và lưu.

Ví dụ raw median = 5012,5 mm ở khoảng cách thật 5000 mm:

```powershell
py -3.12 .\uwb_command.py --port $port set-cal 1 12.5 --save
```

Nếu dùng giá trị “GUI offset = truth − raw”, phải dùng tùy chọn `--gui-offset`
để tránh ngược dấu.

6. Reboot TAG, chạy `get-cal`, xác nhận calibration còn đúng.
7. Đo lại tại 1 m và 10 m.

**PASS mỗi Anchor:**

- [ ] Sau calibration, |median error| ≤10 mm tại 5 m.
- [ ] Sai số còn lại ≤±30 mm tại 1 m và 10 m.
- [ ] Sigma tĩnh ≤30 mm.
- [ ] Calibration còn đúng sau reboot.
- [ ] Đúng calibration profile ID và đúng module vật lý trong manifest.

Giữ compile-time mask fail-closed đối với firmware generic. Nếu thay module,
antenna, enclosure hoặc RF profile, xóa trạng thái calibrated của đường đó.

### A-02 · Tuyến tính theo khoảng cách

Đo đủ tám Anchor tại 1/3/5/10/15 m hoặc tới khoảng cách vận hành tối đa. Mỗi
điểm tối thiểu 60 giây và 1000 radio-good sample.

**Báo cáo:** bias, RMSE, sigma, P95/P99 absolute error, min/max, success,
fallback, FPP/RX và NLOS delta.

**PASS ban đầu:**

- [ ] 1–10 m: |bias| ≤30 mm, sigma ≤30 mm.
- [ ] 15 m/tầm xa: |bias| ≤50 mm, sigma ≤50 mm, nếu yêu cầu dự án không chặt hơn.
- [ ] Radio success ≥98%.
- [ ] Không có outlier >0,5 m trong LOS.

### A-03 · Burst so với tuần tự

Chọn A8 tại 5 m, giữ nguyên vị trí:

```powershell
py -3.12 .\uwb_command.py --port $port set-mask 0x80
py -3.12 .\uwb_command.py --port $port set-burst --mode burst
# ghi 60 giây
py -3.12 .\uwb_command.py --port $port set-burst --mode sequential
# ghi 60 giây
py -3.12 .\uwb_command.py --port $port set-burst --mode burst
```

**PASS:** |Δmedian raw/corrected| <50 mm; mục tiêu tốt là ≤10 mm. Mỗi mode
radio success ≥98%, không so sánh filtered output.

### A-04 · Ảnh hưởng hướng anten và thân drone

Đo 0/90/180/270° yaw; thêm pitch và roll ±30/60° nếu drone có thể đạt các góc
đó. Lặp lại khi TAG đã gắn trong enclosure/thân drone thật.

Ghi biên độ range, P95 error, success, NLOS delta và góc xấu nhất.

Test là `INFO` cho đến khi dự án đặt ngưỡng. Trước khi bay phải chọn một cách
lắp có P95 error và dropout nằm trong ngân sách điều khiển.

### A-05 · LOS/NLOS và detector

Tại 5 m, mỗi điều kiện ghi tối thiểu 60 giây:

- LOS chuẩn;
- người đứng chắn;
- vách gỗ/thạch cao;
- tấm kim loại chắn một phần;
- thân/carbon/battery drone che anten.

**Báo cáo:** bias, P95/P99, success, fallback, tỷ lệ NLOS ở ngưỡng 6 dB và 10
dB, false-positive LOS và detection rate từng vật cản.

**Ngưỡng khởi đầu:** false-positive LOS ≤5%; body/metal detection ≥80%. Nếu
không đạt, không dùng cờ NLOS như một classifier production cho tới khi retune.

---

## 8. Cổng hệ thống trước khi bay

Các bài này không thay thế L1/L2; chỉ chạy sau khi H-12 và A-01/A-02 PASS.

### S-01 · Static position grid

- [ ] Survey tọa độ Anchor và lưu `anchor_layout.json`.
- [ ] Đo nhiều điểm ở tâm, mép và ngoài convex hull.
- [ ] Đo nhiều độ cao; không chỉ mặt phẳng 2D.
- [ ] Báo horizontal/vertical RMSE, P95/P99, GDOP và availability.
- [ ] Mỗi epoch ghi số Anchor radio-good/valid được solver sử dụng.

### S-02 · Chuyển động và latency

- [ ] Chạy tuyến thẳng, dừng/đi, quay và thay đổi độ cao.
- [ ] Đo end-to-end latency và jitter TAG→gateway→host.
- [ ] Kiểm tra timestamp từng slot; không mặc định tám range hoàn toàn đồng thời.
- [ ] Ghi maximum position outage và thời gian reacquire.

### S-03 · Nguồn, motor và EMI

- [ ] Props tháo: motor idle, 25/50/75/100% throttle.
- [ ] Lặp lại với CPU/gateway/telemetry tải cao.
- [ ] Kiểm tra nguồn pin thấp/cao trong dải vận hành.
- [ ] So sánh success, FPP, range P95, recovery và UART lỗi.

### S-04 · Fault và fail-safe

- [ ] Tắt lần lượt 1, 2, 3 và 4 Anchor.
- [ ] Che NLOS theo burst dài 0,1/0,5/2 giây.
- [ ] Reset một Anchor và reset gateway.
- [ ] Bộ điều khiển không dùng range stale/uncalibrated.
- [ ] Khi định vị không đủ tin cậy, hệ thống chuyển trạng thái fail-safe đã định.

---

## 9. Khôi phục cấu hình sau test

Trước khi kết thúc một ngày test:

- [ ] Tắt mọi HIL RAW/fault/XTAL override build.
- [ ] Trả active mask đúng cấu hình production.
- [ ] Trả burst timing về cấu hình đã chốt.
- [ ] Trả mode đúng profile production.
- [ ] Kiểm tra `get-cal` và calibration profile ID.
- [ ] Reboot toàn hệ thống và chạy nhanh H-01/H-02.
- [ ] Không dùng artifact có `dirty=1` hoặc `UWB_HIL_TEST=1` để bay.

---

## 10. Bảng tổng hợp

| ID | Profile | Build/git/config | Ngày | Log/session | Kết quả | Ghi chú |
|---|---|---|---|---|---|---|
| H-00 | | | | | | |
| H-01 | | | | | | |
| H-02 | | | | | | |
| H-03 | | | | | | RAW required |
| H-04 | | | | | | Δslot = |
| H-05 | | | | | | wrap count = |
| H-06 | | | | | | Δppm = |
| H-07 | | | | | | |
| H-08 | | | | | | base/slot/margin/gap = |
| H-09 | | | | | | FI profile = |
| H-10 | | | | | | txn wrap = |
| H-11 | | | | | | CRC/drop/high-water = |
| H-12 | | | | | | period P50/P99 = |
| A-01 | | | | | | calibration table |
| A-02 | | | | | | |
| A-03 | | | | | | Δburst-seq = |
| A-04 | | | | | | worst angle = |
| A-05 | | | | | | NLOS FP/TP = |
| S-01 | | | | | | position P95/P99 = |
| S-02 | | | | | | latency/outage = |
| S-03 | | | | | | |
| S-04 | | | | | | |

## 11. Điều kiện kết luận

Chỉ được kết luận **firmware ranging đạt bench** khi:

- H-00, H-01, H-02, H-04, H-07, H-08, H-11 và H-12 PASS;
- H-03/H-05/H-06/H-09/H-10 PASS sau khi HIL instrumentation được bổ sung,
  hoặc được ghi rõ là BLOCKED và chưa dùng cho flight approval;
- toàn bộ build/config/hardware identity có thể truy vết.

Chỉ được kết luận **range đã hiệu chuẩn** khi A-01 và A-02 PASS trên đúng TAG,
Anchor, RF profile và cơ khí production.

Chỉ được kết luận **sẵn sàng thử bay có kiểm soát** khi S-01…S-04 có ngưỡng
được dự án phê duyệt và PASS. Không dùng kết quả bench range thay cho nghiệm thu
position, latency hoặc fail-safe.
