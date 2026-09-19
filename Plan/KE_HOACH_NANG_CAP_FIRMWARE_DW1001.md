# Kế hoạch Nâng cấp Firmware DW1001
**Ngày lập**: 2026-09-19  
**Cơ sở**: Phân tích so sánh firmware DW1001 hiện tại với Bitcraze LPS Node Firmware  
**Phạm vi**: 5 hạng mục nâng cấp (P0–P4), ưu tiên theo giá trị cho hệ thống drone  

---

## Mục lục

- [Bối cảnh & Động lực](#bối-cảnh--động-lực)
- [P0 — Anchor Position Broadcast](#p0--anchor-position-broadcast)
- [P1 — Dynamic Anchor Discovery](#p1--dynamic-anchor-discovery)
- [P2 — OTA Configuration Commands](#p2--ota-configuration-commands)
- [P3 — Multi-PHY Profile](#p3--multi-phy-profile)
- [P4 — TDOA Mode](#p4--tdoa-mode)
- [Tổng kết rủi ro & Phụ thuộc](#tổng-kết-rủi-ro--phụ-thuộc)

---

## Bối cảnh & Động lực

Firmware DW1001 hiện tại đã mạnh ở tầng **ranging core**: delayed TX, per-anchor
calibration, DS-TWR với SS fallback, multi-layer filtering (Median → Kalman →
Adaptive), comprehensive error recovery. Những tính năng này vượt trội so với
Bitcraze LPS.

Tuy nhiên, hệ thống Bitcraze có các **tính năng ở tầng ứng dụng** mà firmware
DW1001 còn thiếu — và đây chính là những yếu tố cần thiết để chuyển từ "hệ
thống đo khoảng cách" thành "hệ thống định vị hoàn chỉnh cho drone":

| Thiếu gì | Hệ quả |
|----------|--------|
| Anchor không tự gửi tọa độ | Drone cần máy tính ngoài để biết anchor ở đâu |
| Số anchor hardcode = 4 | Không thể thêm/bớt anchor mà không flash lại firmware |
| Không có OTA command | Mỗi lần đổi config (vị trí, power) phải flash lại |
| Chỉ 1 PHY profile | Không tối ưu được cho indoor/outdoor khác nhau |
| Chỉ có TWR | Giới hạn số drone đồng thời (~5–8 chiếc) |

---

## P0 — Anchor Position Broadcast

### 1. Mục tiêu

Anchor tự gửi tọa độ XYZ (float × 3 = 12 bytes) kèm theo RESP frame. Tag nhận
RESP → vừa có khoảng cách, vừa biết anchor ở đâu → tự trilateration onboard.

**Đây là bước bắt buộc** nếu muốn drone chạy autonomous mà không cần ground
station. Pixhawk cần nhận vị trí từ UWB tag, và tag chỉ có thể tính vị trí nếu
biết tọa độ các anchor.

### 2. Thiết kế chi tiết

#### 2.1 Frame format mới cho RESP

**RESP hiện tại** (14 bytes data + 2 FCS = 16 bytes on air):

```
Byte  Nội dung            Nguồn
───── ─────────────────── ──────────────────────────
 [0]  0x41                FCF low (Data frame)
 [1]  0x88                FCF high (16-bit addr, Intra-PAN)
 [2]  s_tx_seq++          Sequence number
 [3]  DW_PAN_ID & 0xFF    PAN ID low  (0xCA)
 [4]  DW_PAN_ID >> 8      PAN ID high (0xDE)
 [5]  dst_addr low        Tag address low (từ POLL source)
 [6]  dst_addr high       Tag address high
 [7]  ANCHOR_ADDR low     Anchor address low
 [8]  ANCHOR_ADDR high    Anchor address high
 [9]  FRAME_RESP_FUNC     0x10
[10]  t_reply byte 0      ┐
[11]  t_reply byte 1      │ Anchor reply delay
[12]  t_reply byte 2      │ (uint32_t, little-endian)
[13]  t_reply byte 3      ┘
```

**RESP mới** (27 bytes data + 2 FCS = 29 bytes on air):

```
Byte  Nội dung            Thay đổi
───── ─────────────────── ──────────────────────────
 [0]  0x41                (giữ nguyên)
 [1]  0x88                (giữ nguyên)
 [2]  s_tx_seq++          (giữ nguyên)
 [3]  DW_PAN_ID & 0xFF    (giữ nguyên)
 [4]  DW_PAN_ID >> 8      (giữ nguyên)
 [5]  dst_addr low        (giữ nguyên)
 [6]  dst_addr high       (giữ nguyên)
 [7]  ANCHOR_ADDR low     (giữ nguyên)
 [8]  ANCHOR_ADDR high    (giữ nguyên)
 [9]  FRAME_RESP_FUNC     0x10 (giữ nguyên)
[10]  t_reply byte 0      (giữ nguyên)
[11]  t_reply byte 1      (giữ nguyên)
[12]  t_reply byte 2      (giữ nguyên)
[13]  t_reply byte 3      (giữ nguyên)
────────────── PHẦN MỚI ────────────────────────────
[14]  pos_flags            ← MỚI: bit0=position_valid
[15]  pos_x byte 0        ┐
[16]  pos_x byte 1        │ X coordinate
[17]  pos_x byte 2        │ (float, little-endian, mét)
[18]  pos_x byte 3        ┘
[19]  pos_y byte 0        ┐
[20]  pos_y byte 1        │ Y coordinate
[21]  pos_y byte 2        │ (float, little-endian, mét)
[22]  pos_y byte 3        ┘
[23]  pos_z byte 0        ┐
[24]  pos_z byte 1        │ Z coordinate
[25]  pos_z byte 2        │ (float, little-endian, mét)
[26]  pos_z byte 3        ┘
```

#### 2.2 Vấn đề cần xử lý

**Vấn đề 1: Airtime tăng → ảnh hưởng timing budget**

RESP frame dài thêm 13 bytes. Ở 6.8 Mbps, mỗi byte tốn ~1.18 µs → thêm
~15 µs. **Không ảnh hưởng đáng kể** so với preamble 256 symbols (~295 µs) và
reply delay 1200 UUS (~1.23 ms).

Tuy nhiên phải cập nhật `RESP_FRAME_LEN` và `MAX_RX_FRAME_LEN`:

```c
// anchor_ranging.h — hiện tại
#define RESP_FRAME_LEN  14
// → phải đổi thành
#define RESP_FRAME_LEN  27

// anchor_ranging.c & tag_ranging.c — hiện tại
#define MAX_RX_FRAME_LEN  20
// → phải đổi thành
#define MAX_RX_FRAME_LEN  30
```

**Vấn đề 2: Tag phải parse position mà không làm hỏng SS-TWR timing**

Tag đọc RESP payload để lấy `t_reply` (bytes 10-13). Phần position (bytes 14-26)
phải được parse **sau** khi tính ToF xong, không chen vào giữa critical timing
path. Nếu `pos_flags == 0` (anchor chưa cấu hình vị trí), Tag bỏ qua.

**Vấn đề 3: Backward compatibility**

Nếu Tag mới gặp Anchor cũ (RESP chỉ 14 bytes), hoặc Anchor mới gặp Tag cũ:

- **Tag mới ← Anchor cũ**: `rx_len < 27` → không parse position, chỉ dùng
  `t_reply`. Ranging vẫn hoạt động bình thường.
- **Anchor mới ← Tag cũ**: POLL frame không đổi (10 bytes) → hoàn toàn tương
  thích. Anchor gửi RESP dài hơn, Tag cũ chỉ đọc đến byte 13, bỏ qua phần
  thừa.

**Kết luận**: Backward-compatible tự nhiên, không cần version negotiation.

**Vấn đề 4: Anchor cần lưu trữ tọa độ**

Hai phương án:

| Phương án | Mô tả | Ưu | Nhược |
|-----------|-------|-----|-------|
| A: Kconfig compile-time | Thêm `CONFIG_UWB_ANCHOR_POS_X/Y/Z` vào Kconfig, ghi vào `anchor_N.conf` | Đơn giản, an toàn | Phải flash lại khi di chuyển anchor |
| B: NVS runtime | Lưu vào Zephyr NVS (flash), cho phép OTA update | Linh hoạt | Phức tạp hơn, cần P2 OTA |

**Đề xuất**: Bắt đầu phương án A (compile-time), sau đó P2 sẽ thêm NVS.

#### 2.3 Danh sách file cần sửa

| File | Thay đổi |
|------|----------|
| `Kconfig` | Thêm `UWB_ANCHOR_POS_X`, `UWB_ANCHOR_POS_Y`, `UWB_ANCHOR_POS_Z` (int, milimét, depends on `UWB_ROLE_ANCHOR`) |
| `include/uwb_app_config.h` | Map Kconfig → `ANCHOR_POS_X_MM` / `ANCHOR_POS_Y_MM` / `ANCHOR_POS_Z_MM` macro |
| `include/anchor_ranging.h` | Tăng `RESP_FRAME_LEN` 14→27, thêm `RESP_POS_FLAGS_OFFSET 14`, `RESP_POS_PAYLOAD_OFFSET 15` |
| `src/ranging/anchor_ranging.c` | Trong `handle_poll()`: thêm position bytes vào RESP payload (bytes 14-26). Cập nhật `DW1000_SetTxFrameCtrl(RESP_FRAME_LEN + 2)` |
| `include/tag_ranging.h` | Thêm `TagAnchorPosition_t { float x, y, z; uint8_t valid; }`, thêm vào `TagCycleSnapshot_t` hoặc riêng. Tăng `MAX_RX_FRAME_LEN` 20→30 |
| `src/ranging/tag_ranging.c` | Trong WAIT_RESP handler: sau khi tính distance xong, nếu `rx_len >= 27 && rx_buf[14] & 0x01` → parse position. Lưu vào `s_anchor_position[anchor_index]` |
| `include/telemetry.h` | Thêm `TELEM_TYPE_ANCHOR_MAP 0x03` |
| `src/telemetry/telemetry.c` | Thêm `Telem_SendAnchorMap()` — gửi position N anchors đã biết. Gọi 1 lần/giây cùng Stats |
| `config/anchor_1.conf` | Thêm `CONFIG_UWB_ANCHOR_POS_X=0`, `CONFIG_UWB_ANCHOR_POS_Y=0`, `CONFIG_UWB_ANCHOR_POS_Z=1500` (ví dụ: 0, 0, 1.5m) |
| `config/anchor_2.conf` | Tương tự, với tọa độ thực tế |
| `config/anchor_3.conf` | Tương tự |
| `config/anchor_4.conf` | Tương tự |

#### 2.4 Rủi ro

| Rủi ro | Mức | Giảm thiểu |
|--------|-----|------------|
| RESP dài hơn → delayed TX trễ (HPDWARN) | Thấp | Thêm 13 bytes chỉ tốn ~15 µs; `ANCHOR_REPLY_DELAY_UUS=1200` dư margin lớn |
| DS-TWR path bị ảnh hưởng | Trung bình | RESP format thay đổi ảnh hưởng cả DS path → phải test cả SS lẫn DS |
| Tag parsing sai offset | Trung bình | Unit test: ghi frame mẫu → verify parse position đúng |
| Anchor chưa cấu hình tọa độ | Thấp | `pos_flags = 0` → Tag bỏ qua. Mặc định Kconfig = 0,0,0 |

#### 2.5 Kế hoạch test

1. **Unit test**: Ghi RESP frame 27 bytes → verify parse `t_reply` và position
2. **Single anchor**: Flash 1 anchor với position, verify Tag nhận đúng
3. **Mixed firmware**: Tag mới + 1 anchor cũ (14-byte RESP) + 1 anchor mới →
   verify ranging vẫn hoạt động, position chỉ có ở anchor mới
4. **DS-TWR path**: Verify FINAL/REPORT flow không bị ảnh hưởng bởi RESP dài hơn
5. **Timing**: Đo `anchor_slot_duration_max_us` trước/sau → confirm không quá
   budget 20ms

#### 2.6 Effort ước tính

- Sửa code: **1–2 ngày**
- Test & debug: **1 ngày**
- Tổng: **~3 ngày**

---

## P1 — Dynamic Anchor Discovery

### 1. Mục tiêu

Tag tự động phát hiện anchor khi khởi động, không cần hardcode
`TAG_NUM_ANCHORS = 4` và danh sách `{0x0001, 0x0002, 0x0003, 0x0004}`.

### 2. Thiết kế chi tiết

#### 2.1 Discovery protocol

Thêm 1 frame type mới:

```c
#define FRAME_DISCOVERY_FUNC  0x24  // Discovery request (Tag → broadcast)
#define FRAME_ANNOUNCE_FUNC   0x25  // Anchor announcement (Anchor → Tag)
```

**Flow**:

```
Tag boot → gửi DISCOVERY broadcast (dst=0xFFFF) mỗi 100ms, tối đa 10 lần
    ↓
Mỗi Anchor nhận DISCOVERY → chờ random delay (0–5ms, theo ANCHOR_ADDR)
    ↓
Anchor gửi ANNOUNCE: [header 10B][func 1B][anchor_addr 2B][pos_x 4B][pos_y 4B][pos_z 4B]
    ↓
Tag nhận ANNOUNCE → thêm anchor vào bảng nếu chưa có
    ↓
Sau 10 lần DISCOVERY (hoặc đủ anchor) → chuyển sang ranging bình thường
```

#### 2.2 Vấn đề cần xử lý

**Vấn đề 1: `TAG_NUM_ANCHORS` là compile-time constant**

Hiện tại `TAG_NUM_ANCHORS = 4U` được dùng ở khắp nơi: kích thước mảng
`s_kf[]`, `s_mf[]`, `s_track[]`, `calibration_offset_m[]`, và tất cả diagnostic
counters. Thay đổi thành runtime sẽ phải:

- Đổi tất cả `[TAG_NUM_ANCHORS]` thành `[TAG_MAX_ANCHORS]` (giới hạn tĩnh, ví
  dụ 8)
- Thêm `uint8_t active_anchor_count` (runtime)
- Sửa tất cả vòng lặp `for (i = 0; i < TAG_NUM_ANCHORS; i++)` thành dùng
  `active_anchor_count`

**Đây là thay đổi lớn**, ảnh hưởng đến tag_ranging.c (2295 dòng), telemetry.c,
range_filter.c, và tất cả header files.

**Vấn đề 2: Calibration per-anchor**

Discovery tự động đồng nghĩa anchor mới (chưa calibrate) có thể xuất hiện
runtime. Cần policy: ranging vẫn hoạt động nhưng `status = TAG_ST_CALIBRATION_MISSING`
cho đến khi có offset.

**Vấn đề 3: Anchor collision khi reply**

Nếu 4 anchors cùng nhận DISCOVERY và cùng reply → collision. Giải pháp:
mỗi anchor chờ `ANCHOR_ADDR × 2 ms` trước khi reply.

**Vấn đề 4: Telemetry record size thay đổi**

`TELEM_RANGE_RECORD_LEN = 16` bytes × N anchors. Nếu N thay đổi runtime →
phải đảm bảo không vượt `UART_TX_BUF_SIZE = 1024`.
Với `MAX_ANCHORS = 8`: header 14 + 1 + 8 × 16 = 143 bytes + CRC 2 = 145 bytes.
Vẫn an toàn.

**Vấn đề 5: 50Hz cycle budget**

Thêm anchor → thêm slot → cycle time tăng. Mỗi anchor slot tốn ~3–5 ms
(POLL TX + reply delay + RESP RX + processing). Với 8 anchors:

| Anchors | Estimated cycle | 50Hz (20ms) khả thi? |
|---------|-----------------|----------------------|
| 4 | ~14–16 ms | ✅ Dư |
| 6 | ~21–24 ms | ⚠️ Sát giới hạn |
| 8 | ~28–32 ms | ❌ Phải giảm xuống 30Hz |

**Đề xuất**: Giữ `TAG_MAX_ANCHORS = 6`, tự động giảm cycle rate nếu > 4 anchors.

#### 2.3 Danh sách file cần sửa

| File | Thay đổi |
|------|----------|
| `include/tag_ranging.h` | `TAG_NUM_ANCHORS` → `TAG_MAX_ANCHORS = 6`, thêm `extern uint8_t active_anchor_count` |
| `include/dw1000_hw.h` | Thêm `FRAME_DISCOVERY_FUNC 0x24`, `FRAME_ANNOUNCE_FUNC 0x25` |
| `src/ranging/tag_ranging.c` | Thêm `TAG_STATE_DISCOVERY`, sửa tất cả vòng lặp, thêm `s_anchor_table[]` runtime |
| `src/ranging/anchor_ranging.c` | Xử lý DISCOVERY frame trong `ANCHOR_STATE_RX_WAIT`, gửi ANNOUNCE |
| `include/anchor_ranging.h` | Thêm prototype cho discovery handler |
| `src/telemetry/telemetry.c` | Record count dùng `active_anchor_count` thay vì `TAG_NUM_ANCHORS` |
| `include/telemetry.h` | Cập nhật `TELEM_RANGE_PAYLOAD_MAX` |
| `Kconfig` | Thêm `UWB_MAX_ANCHORS` (default 6) |

#### 2.4 Rủi ro

| Rủi ro | Mức | Giảm thiểu |
|--------|-----|------------|
| Refactor lớn ảnh hưởng 2300+ dòng tag_ranging.c | **Cao** | Làm từng bước: đầu tiên chỉ rename constant, test đúng, rồi mới thêm discovery |
| Adaptive filter state mismatch khi anchor xuất hiện/biến mất | Trung bình | Reset filter state cho anchor mới; giữ state cũ cho anchor biến mất (timeout tự xử lý) |
| Cycle overrun khi > 4 anchors | Trung bình | Auto-detect: nếu cycle > 20ms → tự giảm cycle rate |

#### 2.5 Effort ước tính

- Refactor `TAG_NUM_ANCHORS` → `TAG_MAX_ANCHORS`: **2–3 ngày** (nhiều file, cần cẩn thận)
- Discovery protocol: **2 ngày**
- Test: **2 ngày**
- Tổng: **~1–1.5 tuần**

---

## P2 — OTA Configuration Commands

### 1. Mục tiêu

Tag gửi command tới Anchor qua UWB để cấu hình runtime (vị trí, TX power,
reboot, đổi mode), không cần flash lại firmware.

### 2. Thiết kế chi tiết

#### 2.1 Command frame format

Tái sử dụng frame structure hiện tại, thêm function code mới:

```c
#define FRAME_CMD_FUNC   0x30  // Command frame (Tag → Anchor)
#define FRAME_CMD_ACK    0x31  // Command acknowledgment (Anchor → Tag)
```

**Command payload** (sau header 10 bytes):

```
Byte  Nội dung
───── ─────────────────────────────
[10]  CMD_ID       (uint8_t — loại command)
[11]  CMD_LEN      (uint8_t — payload length)
[12+] CMD_PAYLOAD  (variable)
```

#### 2.2 Bảng command

| CMD_ID | Tên | Payload | Mô tả |
|--------|-----|---------|-------|
| `0x01` | `CMD_SET_POSITION` | `float x, y, z` (12 bytes) | Đặt tọa độ anchor |
| `0x02` | `CMD_SET_TX_POWER` | `uint32_t tx_power` (4 bytes) | Đổi công suất TX |
| `0x03` | `CMD_REBOOT` | `uint8_t mode` (1 byte) | 0=firmware, 1=bootloader |
| `0x04` | `CMD_SET_PHY` | `uint8_t profile_id` (1 byte) | Đổi PHY profile (P3) |
| `0x05` | `CMD_SAVE_CONFIG` | (none) | Ghi config hiện tại vào NVS |
| `0x06` | `CMD_FACTORY_RESET` | (none) | Xóa NVS, khôi phục Kconfig mặc định |

#### 2.3 Vấn đề cần xử lý

**Vấn đề 1: Khi nào Tag gửi command?**

Command KHÔNG được gửi trong ranging cycle (sẽ phá vỡ timing). Phương án:

- **Sau cycle xong**: Nếu cycle kết thúc trước 20ms deadline, dùng thời gian rỗi
  để gửi command. Cần state `TAG_STATE_CMD_TX` mới.
- **Dedicated command cycle**: Mỗi N cycle (ví dụ mỗi 1 giây), skip 1 anchor
  slot để gửi command.

**Đề xuất**: Phương án 1 — gửi command trong idle time cuối cycle.

**Vấn đề 2: NVS (Non-Volatile Storage)**

Zephyr NVS API yêu cầu flash partition. DWM1001 dùng nRF52832 (512 KB flash).
Cần thêm NVS partition vào devicetree overlay:

```dts
&flash0 {
    partitions {
        nvs_partition: partition@7e000 {
            label = "nvs_storage";
            reg = <0x7e000 0x2000>;   /* 8 KB */
        };
    };
};
```

**Vấn đề 3: Bảo mật**

Hiện tại không có authentication. Bất kỳ device nào gửi FRAME_CMD_FUNC đều
được Anchor xử lý. Cho prototype đủ dùng, nhưng production cần thêm:

- Simple shared key (4 bytes) trong command payload
- Hoặc chỉ chấp nhận command từ TAG_ADDR đã biết

**Vấn đề 4: Acknowledgment và retry**

Anchor phải ACK command (FRAME_CMD_ACK) để Tag biết thành công. Nếu không nhận
ACK trong 10ms → Tag retry tối đa 3 lần.

#### 2.4 Danh sách file cần sửa

| File | Thay đổi |
|------|----------|
| `include/dw1000_hw.h` | Thêm `FRAME_CMD_FUNC 0x30`, `FRAME_CMD_ACK 0x31` |
| `src/ranging/anchor_ranging.c` | Xử lý `FRAME_CMD_FUNC` trong RX_WAIT, thêm command handler, NVS read/write |
| `include/anchor_ranging.h` | Thêm command ID defines, NVS key defines |
| `src/ranging/tag_ranging.c` | Thêm `TAG_STATE_CMD_TX`, command queue, ACK handler |
| `include/tag_ranging.h` | Thêm `Tag_SendCommand()` API |
| `prj.conf` | Thêm `CONFIG_NVS=y`, `CONFIG_FLASH=y`, `CONFIG_FLASH_MAP=y` |
| DTS overlay | Thêm NVS partition |

#### 2.5 Rủi ro

| Rủi ro | Mức | Giảm thiểu |
|--------|-----|------------|
| NVS write blocking (flash erase ~20ms) | **Cao** | Chỉ write khi CMD_SAVE_CONFIG, không trong ranging cycle |
| Command xung đột với ranging timing | Trung bình | Chỉ gửi command trong idle time cuối cycle |
| Bảo mật yếu | Thấp (prototype) | Chấp nhận cho development; thêm key check cho production |

#### 2.6 Effort ước tính

- Command protocol: **2 ngày**
- NVS integration: **1–2 ngày**
- Tag command queue: **1 ngày**
- Test: **1–2 ngày**
- Tổng: **~5–7 ngày**

---

## P3 — Multi-PHY Profile

### 1. Mục tiêu

Hỗ trợ 2–3 PHY profile để tối ưu cho các điều kiện môi trường khác nhau.

### 2. Thiết kế chi tiết

#### 2.1 Bảng profile đề xuất

| ID | Tên | Channel | PRF | Preamble | Data Rate | PAC | Use case |
|----|-----|---------|-----|----------|-----------|-----|----------|
| 1 | `FAST_128` | 5 | 16 MHz | 128 | 6.8 Mbps | 8 | Indoor gần (<15m), tốc độ cao |
| 2 | `FAST_256` (**hiện tại**) | 5 | 16 MHz | 256 | 6.8 Mbps | 16 | General purpose (default) |
| 3 | `LONG_1024` | 5 | 64 MHz | 1024 | 6.8 Mbps | 32 | Outdoor xa (>30m) |

#### 2.2 Vấn đề cần xử lý

**Vấn đề 1 (CRITICAL): Mỗi profile yêu cầu calibration riêng**

Đây là vấn đề nghiêm trọng nhất. Antenna delay và calibration offset thay đổi
theo PRF và preamble length. Offset đã calibrate cho FAST_256 (~154–157m) sẽ
**SAI HOÀN TOÀN** cho LONG_1024.

Cần calibrate lại cho mỗi profile × mỗi anchor = 3 profiles × 4 anchors = 12
lần calibration. Nếu dùng DS-TWR: 12 DS + 12 SS = 24 lần.

**Vấn đề 2: DW1000 tuning registers thay đổi theo profile**

Mỗi profile cần set khác nhau cho:
- `AGC_TUNE1`: khác nhau cho PRF 16 vs 64
- `DRX_TUNE1a`: khác nhau cho PRF 16 vs 64
- `DRX_TUNE2`: khác nhau cho PRF × PAC
- `DRX_SFDTOC`: khác nhau cho preamble length
- `DRX_TUNE4H`: khác nhau cho preamble length
- `LDE_CFG2`: khác nhau cho PRF 16 vs 64
- `CHAN_CTRL`: mã preamble khác nhau
- `TX_FCTRL`: preamble length encoding khác nhau
- `LDE_REPC`: khác nhau cho mã preamble

Phải tạo lookup table cho tất cả register values theo profile.

**Vấn đề 3: Anchor và Tag phải dùng cùng profile**

Nếu Tag chuyển sang `LONG_1024` mà Anchor vẫn ở `FAST_256` → không thể
giao tiếp. Phương án:

- **Compile-time**: Chọn profile qua Kconfig, giống hiện tại. An toàn nhưng phải
  flash lại.
- **Runtime switch** (cần P2 OTA): Tag gửi `CMD_SET_PHY` → Anchor đổi profile →
  cả hai restart. Phức tạp, rủi ro.

**Đề xuất**: Bắt đầu compile-time. Runtime switch chỉ khi P2 đã ổn định.

**Vấn đề 4: FPP constant A khác nhau theo PRF**

Công thức First-Path Power dùng hằng số A:
- PRF 16 MHz: A = 113.77
- PRF 64 MHz: A = 121.74

Hiện tại `UWB_FPP_A_CONST = 113.77f` hardcode cho PRF16. Phải thay bằng
profile-dependent.

#### 2.3 Danh sách file cần sửa

| File | Thay đổi |
|------|----------|
| `include/dw1000_hw.h` | Thêm `UwbPhyProfile_t` struct, lookup table cho register values |
| `include/uwb_calibration.h` | Thêm calibration offsets per-profile, FPP A constant per-profile |
| `src/drivers/dw1000.c` | `DW1000_Configure()` nhận `UwbPhyProfile_t*` parameter thay vì hardcode |
| `Kconfig` | Thêm `choice UWB_PHY_PROFILE` |
| `include/uwb_app_config.h` | Map Kconfig → profile selection |

#### 2.4 Rủi ro

| Rủi ro | Mức | Giảm thiểu |
|--------|-----|------------|
| **Calibration effort cực lớn** | **Cao** | Bắt đầu chỉ với 2 profiles; calibrate từng cái |
| Register value sai → DW1000 không hoạt động | Cao | Verify bằng `DW1000_VerifyConfig()` sau mỗi lần đổi profile |
| FPP/Kalman tuning khác nhau theo profile | Trung bình | FPP A constant per-profile; Kalman parameters giữ nguyên (đủ robust) |

#### 2.5 Effort ước tính

- Code: **3–5 ngày**
- Calibration: **2–5 ngày per-profile** (cần đo thực tế nhiều cự ly)
- Test: **2 ngày per-profile**
- Tổng: **~2–3 tuần** (bao gồm calibration)

---

## P4 — TDOA Mode

### 1. Mục tiêu

Cho phép Tag chỉ nghe (passive RX), anchors tự broadcast và đồng bộ clock.
Không giới hạn số Tag/drone.

### 2. Thiết kế chi tiết

#### 2.1 Kiến trúc TDoA2 (đơn giản nhất)

```
Anchor 0 (Master):  TX slot 0 ──→ TX slot 4 ──→ TX slot 8 ──→ ...
Anchor 1 (Slave):   ... TX slot 1 ──→ TX slot 5 ──→ ...
Anchor 2 (Slave):   ... ... TX slot 2 ──→ TX slot 6 ──→ ...
Anchor 3 (Slave):   ... ... ... TX slot 3 ──→ TX slot 7 ──→ ...

Tag (passive):      RX all ──→ compute TDoA ──→ position
```

Mỗi anchor broadcast gồm:
- Anchor ID
- Anchor position (x, y, z)
- Sequence number
- Embedded timestamp (TX time của lần broadcast trước, nếu là slave)

#### 2.2 Vấn đề cần xử lý

**Vấn đề 1 (CRITICAL): Clock synchronization giữa anchors**

DW1000 counter chạy ở 499.2 MHz × 128 = ~63.9 GHz. Mỗi anchor có crystal
riêng, drift khoảng ±2 ppm. Ở 10m distance → 2 ppm drift gây lỗi ~3.3 cm sau
1 giây.

Giải pháp: Master broadcast định kỳ, Slave đo TDoA giữa Master TX timestamp
(embedded) và local RX timestamp → tính clock offset → bù.

**Vấn đề 2: TDMA scheduling**

Anchors phải TX theo lịch (slot-based) để tránh collision. Cần:
- Slot duration: ~2 ms (preamble 256 + data + guard)
- 4 anchors × 2 ms = 8 ms per round
- Repeat rate: ~100–125 Hz

**Vấn đề 3: Thuật toán định vị khác hoàn toàn**

TWR cho ra **khoảng cách** (circle) → trilateration.
TDoA cho ra **hiệu khoảng cách** (hyperbola) → hyperbolic positioning.

Cần thay đổi solver phía Tag (hoặc ground station):
- Input: N-1 TDoA measurements (với N anchors)
- Output: position (x, y, z)
- Thuật toán: Chan's algorithm hoặc Taylor series iterative

**Vấn đề 4: Coexistence TWR + TDoA**

Lý tưởng: hệ thống hỗ trợ cả TWR (cho 1-2 drone) và TDoA (cho fleet).
Cần protocol để Anchor biết khi nào chạy mode nào. Phức tạp.

#### 2.3 Rủi ro

| Rủi ro | Mức | Giảm thiểu |
|--------|-----|------------|
| **Clock sync accuracy** | **Rất cao** | Cần crystal tốt (±0.5 ppm), hoặc sync thường xuyên (>50Hz) |
| TDMA collision | Cao | Phải implement slot-based scheduling chính xác |
| Hyperbolic solver phức tạp | Cao | Có thể dùng library có sẵn; hoặc chạy solver trên ground station |
| Firmware gần như phải viết lại phía Anchor | **Rất cao** | Anchor hiện tại là reactive (chờ POLL); TDoA yêu cầu proactive (tự broadcast) |

#### 2.4 Effort ước tính

- Anchor TDoA firmware: **1–2 tuần**
- Tag passive RX: **3–5 ngày**
- Clock sync protocol: **1 tuần**
- Hyperbolic solver: **3–5 ngày**
- Integration test: **1 tuần**
- Tổng: **~4–6 tuần**

#### 2.5 Đề xuất

TDOA là nâng cấp **lớn nhất** và **rủi ro cao nhất**. Chỉ nên bắt đầu khi:

1. P0–P2 đã hoàn thành và ổn định
2. Có nhu cầu thực tế chạy > 3 drone đồng thời
3. Đã test TWR đầy đủ với Pixhawk và tích hợp hoàn chỉnh

---

## Tổng kết rủi ro & Phụ thuộc

### Biểu đồ phụ thuộc

```
P0 (Position Broadcast)     P3 (Multi-PHY)
   │                            │
   ├──→ P1 (Discovery)         (độc lập)
   │       │
   │       ├──→ P4 (TDOA)
   │       │
   └──→ P2 (OTA Config)
            │
            └──→ P4 (TDOA, runtime mode switch)
```

### Ma trận rủi ro tổng hợp

| Hạng mục | Effort | Rủi ro kỹ thuật | Rủi ro regression | Giá trị |
|----------|--------|-----------------|-------------------|---------|
| **P0** | 3 ngày | 🟢 Thấp | 🟡 Trung bình (RESP format đổi → test DS-TWR) | 🔴 Rất cao |
| **P1** | 1–1.5 tuần | 🟠 Cao (refactor lớn) | 🔴 Cao (tag_ranging.c gần như toàn bộ) | 🟡 Trung bình |
| **P2** | 5–7 ngày | 🟡 Trung bình | 🟡 Trung bình (NVS + command parsing) | 🟡 Trung bình |
| **P3** | 2–3 tuần | 🟠 Cao (recalibration) | 🟡 Trung bình | 🟢 Thấp–TB |
| **P4** | 4–6 tuần | 🔴 Rất cao | 🔴 Cao (anchor firmware gần như viết lại) | 🔴 Cao (nếu multi-drone) |

### Thứ tự triển khai khuyến nghị

```
Tuần 1       : P0 — Anchor Position Broadcast
Tuần 2–3     : P2 — OTA Config Commands
Tuần 4–5     : P1 — Dynamic Anchor Discovery
Tuần 6–8     : P3 — Multi-PHY Profile (+ calibration)
Tuần 9–14    : P4 — TDOA Mode (chỉ khi cần multi-drone)
```

### Nguyên tắc an toàn xuyên suốt

1. **Feature flag cho mọi thay đổi**: Dùng `#if` / Kconfig để toggle. Nếu hỏng
   → tắt flag → rollback ngay mà không mất code cũ.
2. **Test regression trước khi merge**: Mỗi P phải verify SS-TWR, DS-TWR,
   DS fallback, timing budget, telemetry format đều còn đúng.
3. **Không sửa calibration hiện tại**: Offset đã validate (A1-A3 SS, A1-A4 DS)
   phải giữ nguyên. Thêm mới thì dùng key mới.
4. **Binary telemetry backward-compatible**: Thêm TYPE mới (0x03, 0x04...) thay
   vì sửa TYPE cũ (0x00, 0x01, 0x02). GUI cũ bỏ qua TYPE lạ.
