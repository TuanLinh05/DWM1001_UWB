# FIRMWARE — tag UWB ALT DS-TWR (STM32F103 + DW1000): frame gửi và cách tính các trường

> **Đối chiếu source ngày25/10/2026:** ghi chú tester bên dưới được giữ để
> truy vết, nhưng có các suy luận cần sửa. Header burst offset11 là u64
> thời gian bắt đầu POLL; record byte6 là RX−FP/0,1dB, byte7 là i8 CI/0,25ppm.
> `0x20` trong flags là RADIO_OK; CAL_MISSING là thiếu residual calibration,
> trong khi hardware antenna delay đã được cấu hình. Firmware hiện tại dùng
> TAG nRF52832/DWM1001-DEV; metadata DEVICE_INFO xác nhận phần cứng.
> Đặc tả đã đối chiếu và bổ sung raw quality/temperature/GUI nằm ở
> [TELEMETRY_QUALITY_20261025.md](../Firmware/docs/TELEMETRY_QUALITY_20261025.md).

Tag gửi range thô tới 8 anchor qua USB VCOM. Host giải mã bằng `~/UWB_DATA_ULTIL/real_data/vcom_node.py`
(publish `/uwb/burst_metadata`). File này mô tả **giao thức nhìn từ phía host**.

Mức độ tin cậy của từng mục:

- **[đã kiểm]** — giải mã khớp dữ liệu thật: 12.783 cycle của `20260930_194034_COM14/burst.csv` (GUI hãng
  giải mã) khớp 100% về `range_mm`, `status`, `mode`, `fp_dbm`; selftest trong `vcom_node.py --selftest`.
- **[suy luận]** — lấy từ DW1000 User Manual / cách làm chuẩn của DS-TWR, **chưa đối chiếu với mã nguồn
  firmware** (mã nguồn không có trong repo này). Cần sửa lại khi có `telemetry.c`.

---

## 1. Đường truyền

| Mục | Giá trị |
|---|---|
| Cổng | USB CDC (J-Link VCOM), thường `/dev/ttyACM0` |
| Baud | 460800, 8N1 → tối đa ≈ 46 kB/s |
| Hướng | Tag tự stream, không cần lệnh khởi động |
| Bẫy | VCOM của J-Link nhớ line coding cũ: mở cổng phải **đổi baud một nhịp** (115200 → 460800), nếu không UART chạy sai tốc độ, mọi frame trượt CRC mà không báo lỗi. `VcomNode.open_port()` đã làm việc này |

## 2. Khung chung [đã kiểm]

Mọi trường little-endian.

```
offset  0   1   2    3     4..5   6..9   10..13   14 .. 14+len-1   14+len .. +1
       AA  55  ver  type  len    seq    t_ms     payload[len]     crc16
       └sync┘  └──────────── vùng tính CRC ─────────────────┘
```

| Trường | Kiểu | Cách tính / ý nghĩa |
|---|---|---|
| `sync` | `AA 55` | Hằng số. Có thể xuất hiện giữa payload → host luôn xác nhận bằng CRC |
| `ver` | u8 | Phiên bản khung |
| `type` | u8 | Loại payload, mục 3–5 |
| `len` | u16 | Số byte payload. Host coi `len > 512` là header rác (mất byte) |
| `seq` | u32 | Đếm khung tăng dần [suy luận: một bộ đếm chung cho mọi type] |
| `t_ms` | u32 | Thời gian tag, ms [suy luận: `HAL_GetTick()`] |
| `crc16` | u16 | CRC-16/CCITT-FALSE trên `ver .. payload` (không gồm `AA 55`) |

CRC-16/CCITT-FALSE: đa thức `0x1021`, giá trị đầu `0xFFFF`, không đảo bit, không XOR đầu ra.

```python
def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c
```

Tổng độ dài khung = `16 + len` byte (14 header + 2 CRC).

## 3. Type `0x18` — BURST DS-TWR (nguồn dữ liệu chính)

Một khung = một **cycle**: tag đo xong với cả 8 anchor. Nhịp = `period_us` (≈ 4.5 ms ở profile 200 Hz,
10 ms ở profile 100 Hz). Payload = header 22 B + `n` record × 8 B (n = 8 → 86 B, khung 102 B,
220 Hz × 102 B ≈ 22 kB/s ≈ 49% băng thông UART).

### 3.1 Header 22 B [đã kiểm]

| Off | Trường | Kiểu | Cách tính / ý nghĩa |
|---|---|---|---|
| 0 | `ver` | u8 | Phiên bản layout burst |
| 1 | `boot_id` | u16 | Đổi mỗi lần tag khởi động lại → host biết `cycle_seq` bị reset |
| 3 | `cycle_seq` | u32 | +1 mỗi cycle. **Lỗ hổng `cycle_seq` = cycle bị mất** (UART hoặc tag bỏ). Trùng giá trị = khung lặp, host bỏ |
| 7 | `meas_seq` | u32 | Đếm phép đo [suy luận: tăng theo số anchor đã đo, không chỉ theo cycle] |
| 11 | `tag_time_us` | u32 | Đồng hồ tag, µs, lúc cycle kết thúc [suy luận về thời điểm]. Quay vòng 2³² µs = **71.6 phút** → host phải unwrap (`uwb_calib.py`, `uwb_pos_node` đã làm). Là mốc thời gian dùng cho EKF và calib, không dùng giờ máy |
| 15 | `rsv` | u32 | Dự phòng, bỏ qua |
| 19 | `period_us` | u16 | Chu kỳ cycle đã cấu hình, µs |
| 21 | `n` | u8 | Số record theo sau |

### 3.2 Record 8 B mỗi anchor

| Off | Trường | Kiểu | Cách tính / ý nghĩa |
|---|---|---|---|
| 0 | `anchor_id` | u8 | 1..8 [đã kiểm] |
| 1 | `status` | u8 | `0x20` = CAL_MISSING: DS-TWR hợp lệ nhưng **chưa trừ antenna delay** (xem 3.3). `0x01` = anchor không trả lời [đã kiểm] |
| 2 | `mode` | u8 | `0x20` = DS-TWR, `0x03` = NONE (timeout) [đã kiểm] |
| 3 | `range_mm` | u16 | Khoảng cách, mm. `0xFFFF` = timeout → host đổi thành `None`, **không** được hiểu là 65.535 m [đã kiểm]. Cách tính ở 3.3 |
| 5 | `fp` | u8 | Công suất first path, `fp_dbm = −fp / 2` (bước 0.5 dB). `0xFF` = không có [đã kiểm giải mã]. **Đo thật luôn ≈ −84 dBm, dao động 0.1–0.2 dB** → không dùng được làm cờ NLOS (mục 6) |
| 6 | `ci` | i16 | Clock offset thô giữa tag và anchor, ≈ ppm × 1000 [suy luận: từ carrier integrator DW1000, xem 3.4]. Số thô jitter hơn số GUI hãng hiển thị (GUI hãng lọc) |

### 3.3 `range_mm` — DS-TWR bất đối xứng [suy luận]

Mỗi cặp tag–anchor trao đổi 3 gói, mỗi bên tự đo khoảng thời gian bằng đồng hồ của mình (đơn vị DW1000
≈ 15.65 ps):

```
tag:    Poll ──────────────(Ra)────────────── Resp nhận ──(Db)── Final
anchor:      Poll nhận ──(Da)── Resp ────────────(Rb)────────────── Final nhận
```

- `Ra` = round tag, `Da` = reply anchor, `Rb` = round anchor, `Db` = reply tag.
- Công thức bất đối xứng (Decawave APS013):

```
ToF = (Ra·Rb − Da·Db) / (Ra + Rb + Da + Db)
range = c · ToF − (antenna delay)      ← firmware hiện chưa trừ, nên status = CAL_MISSING
```

"ALT" / broadcast: tag gửi một Poll chung cho cả 8 anchor, anchor trả lời theo slot riêng
[suy luận từ tên chế độ — chi tiết slot cần mã nguồn].

Hệ quả đã kiểm trên dữ liệu (`CALIB_UWB.md`):

- Công thức trên **triệt tiêu clock offset bậc 1**: `ci` trôi 0.26 ppm mà range đổi 0.0 cm. Nếu là SS-TWR,
  1 ppm × T_reply 0.5 ms ≈ 7.5 cm.
- Phần chưa trừ antenna delay hiện ra thành **bias hằng −13 … +14 cm mỗi anchor** → calib bằng
  `uwb_calib.py spots` (hoặc `uwb_gui`), ghi vào `anN_bias` của file anchors, trừ ở host.

### 3.4 `ci` — clock offset [suy luận]

DW1000 có thanh ghi carrier integrator (`DRX_CAR_INT`); decadriver đổi ra ppm bằng
`dwt_readcarrierintegrator() × hệ số theo data rate` (User Manual mục 7.2.40.11). Firmware có vẻ gửi
`ppm × 1000` dạng i16 (phạm vi ±32.767 ppm). Dùng: theo dõi nhiệt / trôi thạch anh (`uwb_calib.py warmup`),
không cần để tính range vì DS-TWR đã triệt tiêu.

## 4. Type `0x01` — RANGE snapshot (~5 Hz) [đã kiểm giải mã]

Ảnh chụp **đã lọc bởi tag** của burst, không phải phép đo độc lập → `vcom_node` publish riêng ra
`/uwb/snapshot/...`, không trộn với 0x18.

```
num_rec u8, rồi num_rec × 16 B:
  anchor_id u16 | valid u8 | status u8 | age_ms u16 | raw_mm u32 | filtered_mm u32 | fpp_cdbm i16
```

| Trường | Ý nghĩa |
|---|---|
| `valid` | Firmware hiện luôn gửi 0 (chưa bật calibrated mask) |
| `age_ms` | Tuổi mẫu; hiện luôn `0xFFFF` |
| `raw_mm` | Range thô. `0` = anchor không trả lời → host bỏ |
| `filtered_mm` | Range sau bộ lọc trong tag [cách lọc chưa rõ] |
| `fpp_cdbm` | First path power, 0.01 dBm |

## 5. Các type khác (host bỏ qua, chưa giải mã)

`0x00` info, `0x02` stats, `0x11 0x12 0x13 0x15 0x16` diag / ack lệnh. Cần mã nguồn để mô tả.

## 6. JSON trên `/uwb/burst_metadata` (host thêm vào)

`vcom_node` chuyển mỗi burst 0x18 thành một `std_msgs/String` JSON:

```json
{"boot_id": 40338, "cycle_seq": 254794, "meas_seq": 1706338, "tag_time_us": 1626982590, "period_us": 10016,
 "anchors": [{"anchor_id": 1, "status": 32, "mode": 32, "range_mm": 244, "fp_dbm": -84.0, "ci": -4040, "valid": false}, ...]}
```

`range_mm`/`fp_dbm` = `null` khi timeout. `valid` luôn `false` (chờ firmware bật calib).

## 7. Vector kiểm tra [đã kiểm]

Payload 0x18 thật, cycle 254794 (từ `vcom_node.py` selftest):

```
01929d4ae3030062091a00bec8f96000000000202708
012020f400a838f0 0220201801a628fe 0320209901a737f1 0420209a02aa3901
052020ac04a834c4 0620200c04a939ae 0720200504a82ab0 082020ea03a93aaa
```

Giải mã đúng: `boot_id 0x9D92`, `cycle_seq 254794`, `meas_seq 1706338`, `tag_time_us 1626982590`,
`period_us 10016`, `range_mm = 244 280 409 666 1196 1036 1029 1002`, anchor 1 `fp_dbm −84.0`, `mode 0x20`.

## 8. Còn thiếu cho lớp lọc L1 (cờ NLOS RX − FP)

Kế hoạch P0.1 cần công suất thật. Firmware cần đọc thêm từ DW1000 sau mỗi lần nhận Resp:

| Trường | Thanh ghi DW1000 | Kiểu |
|---|---|---|
| `fp_ampl1` | `RX_TIME` (0x15), byte 7–8 | u16 |
| `fp_ampl2` | `RX_FQUAL` (0x12), byte 2–3 | u16 |
| `fp_ampl3` | `RX_FQUAL` (0x12), byte 4–5 | u16 |
| `cir_pwr` | `RX_FQUAL` (0x12), byte 6–7 | u16 |
| `rxpacc` | `RX_FINFO` (0x10), bit 20–31 | u12 |

Host tính theo User Manual mục 4.7 (N = `rxpacc`, xem 4.7.1 về hiệu chỉnh N; A = 113.77 ở PRF 16 MHz,
121.74 ở PRF 64 MHz):

```
FP = 10·log10((F1² + F2² + F3²) / N²) − A        [dBm]
RX = 10·log10(C · 2¹⁷ / N²) − A                   [dBm]
RX − FP < ~6 dB → LOS;  > ~10 dB → nghi NLOS       (Decawave APS006; ngưỡng thật đo ở P1.6)
```

Băng thông: thêm 10 B/record → record 18 B, khung 182 B × 220 Hz ≈ 40 kB/s ≈ **87% của 460800 baud** —
quá sát. Hai lựa chọn:

- Tính trên STM32 và gửi `rx_dbm`, `fp_dbm` mỗi cái u8 bước 0.5 dB (+1 B/record, như `fp` hiện có).
  F103 không có FPU nhưng 1760 lần `log10`/s vẫn nhẹ. Mất khả năng tính lại trên host.
- Gửi thô nhưng giảm nhịp (profile 100 Hz) hoặc nâng baud (921600).
