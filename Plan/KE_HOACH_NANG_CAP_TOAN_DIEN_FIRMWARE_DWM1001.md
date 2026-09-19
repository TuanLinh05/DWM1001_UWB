# Kế hoạch nâng cấp toàn diện firmware DWM1001 — đối chiếu Bitcraze LPS

**Phiên bản:** 1.0
**Ngày lập:** 2026-09-19
**Phạm vi:** nguồn đang phát triển trong `Firmware/` (Tag, Tag_DevKit, Anchor_1…Anchor_8, ESP32C3_Gateway). `Firmware Code Base/` chỉ dùng làm tham chiếu.
**Tham chiếu ngoài:** Bitcraze `lps-node-firmware`, nhánh master, commit `6a85c68` (2025-01-02), kèm `vendor/libdw1000`.
**Đọc cùng:**
- `Firmware/FIRMWARE_REVIEW_2026-09-15.md` — các lỗi F1–F10.
- `Plan/KE_HOACH_TRIEN_KHAI_UWB_DRONE_UP7000_PIXHAWK6C.md` — kiến trúc hệ drone (gọi tắt: *kế hoạch drone*).
- `Plan/KE_HOACH_NANG_CAP_FIRMWARE_DW1001.md` — bản kế hoạch lập trước, được đánh giá ở mục 4.

> **Quy ước**
> - **[Đã kiểm]**: đã đối chiếu trực tiếp trên mã nguồn trong lần rà soát này.
> - **[Ước lượng]**: tính từ mã + datasheet; phải đo trên board trước khi dùng để quyết định.
> - Đường dẫn tính từ gốc `D:\Drone Project\UWB DW1001`. Dòng mã tính theo `Firmware/Tag` và `Firmware/Anchor_1` (các bản copy khác giống hệt).

---

## Mục lục

1. [Tóm tắt điều hành](#1-tóm-tắt-điều-hành)
2. [Hiện trạng firmware](#2-hiện-trạng-firmware)
3. [Đối chiếu với Bitcraze LPS](#3-đối-chiếu-với-bitcraze-lps)
4. [Đánh giá bản kế hoạch trước](#4-đánh-giá-bản-kế-hoạch-trước)
5. [Phát hiện kỹ thuật trong mã nguồn](#5-phát-hiện-kỹ-thuật-trong-mã-nguồn)
6. [Nguyên tắc nâng cấp](#6-nguyên-tắc-nâng-cấp)
7. [Lộ trình chi tiết GĐ0–GĐ6](#7-lộ-trình-chi-tiết)
8. [Đặc tả giao thức đề xuất](#8-đặc-tả-giao-thức-đề-xuất)
9. [Chiến lược kiểm thử](#9-chiến-lược-kiểm-thử)
10. [Rủi ro tổng hợp](#10-rủi-ro-tổng-hợp)
11. [Các quyết định cần chốt](#11-các-quyết-định-cần-chốt)
- [Phụ lục A — Truy vết phát hiện → công việc](#phụ-lục-a--truy-vết-phát-hiện--công-việc)
- [Phụ lục B — Vị trí mã tham khảo trong Bitcraze](#phụ-lục-b--vị-trí-mã-tham-khảo-trong-bitcraze)

---

## 1. Tóm tắt điều hành

### 1.1 Kết luận chính

1. **Lõi đo hiện tại đã tốt hơn Bitcraze ở nhiều điểm**: DS-TWR bất đối xứng có delayed TX, dự đoán RMARKER có antenna delay, fail-closed khi chưa calibration, counter chẩn đoán chi tiết, host test. Không cần thay lõi đo bằng mã Bitcraze.
2. **Nên học Bitcraze ở tầng hệ thống**: một firmware nhiều chế độ, cấu hình lưu bền và đổi được từ xa, anchor tự khai báo thông tin, chế độ sniffer để gỡ lỗi, bù bias theo công suất thu. TDoA3 để dành cho giai đoạn nhiều drone.
3. **Ba việc phải làm trước khi hiệu chuẩn chính thức**:
   - Driver DW1000 lệch cấu hình tham chiếu. TX_POWER đang là `0x1E1E1E1E` khi Smart TX power tắt, cao hơn giá trị tham chiếu khoảng 16 dB. Driver cũng không nạp XTAL trim/LDOTUNE từ OTP, không chỉnh NTM và không RX soft-reset sau lỗi. Sửa các điểm này sẽ làm dịch bias khoảng cách, nên **calibration nào làm trước đó cũng phải làm lại**.
   - Lỗi F1–F4 trong review ngày 2026-09-15 **chưa được sửa** trong mã hiện tại.
   - Mã nguồn đang bị nhân bản 10 lần, và thư mục dự án chưa có Git.
4. **8 anchor chạy DS-TWR tuần tự ước tính mất 25–33 ms mỗi chu kỳ**, vượt mức 20 ms. Với 4 anchor (2D) thì vừa. Muốn 8 anchor chạy 50 Hz phải đổi lịch radio sang **DS-TWR một-nhiều**: POLL và FINAL phát broadcast, các anchor trả lời theo slot. Ước tính cách này mất 9–15 ms cho 8 anchor.
5. **Bản `KE_HOACH_NANG_CAP_FIRMWARE_DW1001.md` được lập trên `Firmware Code Base/`** (4 anchor, Kconfig), không phải nguồn active. Một số giả định trong đó sai: coi offset là đã validate, để TAG tự trilateration, và đặt phân vùng NVS 0x7E000 trùng `storage_partition` của board. Các ý tốt được giữ lại và điều chỉnh (mục 4).

### 1.2 Lộ trình tóm tắt

| GĐ | Nội dung | Công [Ước lượng] | Cổng nghiệm thu chính | Gắn với kế hoạch drone |
|---|---|---|---|---|
| 0 | Nền móng: Git, gộp mã nguồn chung, build ID, test | 1–1,5 tuần | Build mới cho kết quả trùng bản cũ (A/B) | Phase 0 |
| 1 | Độ tin cậy: F1–F4, recovery, phân loại lỗi RX, WAIT4RESP | 1,5–2 tuần | Soak 2 h + fault injection, không treo | Phase 0–2 |
| 2 | Driver RF, hiệu năng SPI, chẩn đoán tín hiệu, sniffer | 2 tuần + thời gian đo | Bảng A/B RF; slot time giảm | Trước Phase 3 |
| 3 | Kiến trúc runtime, cấu hình bền, kênh lệnh, cấu hình anchor từ xa | 2–3 tuần | Đổi cấu hình từ GUI, giữ sau mất điện | Phase 2–3 |
| 4 | Telemetry v2, đồng bộ thời gian, gateway | 2 tuần | Soak 24 h, 0 lỗi CRC trên bench | Phase 2 |
| 5 | Lịch radio 8 anchor (DS-TWR một-nhiều) | 2–3 tuần | 8 anchor, p99 chu kỳ ≤ 15 ms | Trước Phase 10 |
| 6 | Mở rộng: đo anchor–anchor, DFU, BLE, gia tốc kế, TDoA3… | Tuỳ chọn | Theo từng hạng mục | Phase 9–11 |

Giả định công: một người làm firmware, cộng thời gian user build/nạp/đo trên phần cứng. GĐ0–GĐ5 tổng khoảng 11–15 tuần.

```text
GĐ0 → GĐ1 → GĐ2 ─┬──────────────────→ Chiến dịch hiệu chuẩn chính thức (drone plan Phase 3)
                 └→ GĐ3 → GĐ4 → GĐ5 → GĐ6
```

Hiệu chuẩn chính thức làm sau GĐ2. Nên làm sau cả GĐ3.2 để lưu giá trị vào settings mà không phải build lại.

### 1.3 Mười việc nên làm ngay, theo thứ tự

1. `git init`, commit baseline, gắn tag; ghi lại hash firmware đang nạp trên từng board.
2. Tách mã chung Tag/Anchor vào `Firmware/common/`, giữ 10 project làm vỏ build (0.2).
3. Sửa F1: probe anchor offline theo vòng tròn (1.1).
4. Sửa F3 và bổ sung RX soft-reset (1.2).
5. Thêm transaction ID (F4, 1.3).
6. Gộp header và data SPI vào **một** lần `spi_transceive`, rồi đo lại `anchor_slot_duration_max_us` (2.1).
7. Đọc OTP (XTAL trim, LDOTUNE…) và A/B TX power **trước** khi hiệu chuẩn (2.2–2.4).
8. Thay `fatal_blink` bằng cơ chế recovery; watchdog chỉ được feed khi state machine còn tiến triển (1.5).
9. Làm kênh lệnh UART và lưu cấu hình vào `storage_partition` để hiệu chuẩn không cần build lại (3.2, 3.4).
10. Dựng chế độ sniffer trên một board DWM1001-DEV (2.7).

---

## 2. Hiện trạng firmware

### 2.1 Số liệu đã kiểm

| Hạng mục | Giá trị hiện tại | Nguồn |
|---|---|---|
| Nền tảng | nRF52832 + DW1000 (DWM1001C), Zephyr/NCS v3.4.0, board `decawave_dwm1001_dev/nrf52832` | `Firmware/README.md`, `Firmware/Anchor_1/build/CMakeCache.txt` |
| Project | Tag, Tag_DevKit, Anchor_1…8 = 10 project; `dw1000.c` giống nhau, chỉ khác khoảng trắng | `diff -rq` |
| Số anchor | `TAG_NUM_ANCHORS = 8` | `Firmware/Tag/include/tag_ranging.h:29` |
| Chu kỳ | 20 ms; guard giữa anchor 150 µs; nghỉ sau chu kỳ 500 µs | `tag_ranging.h:36, 52, 61` |
| Giao thức | DS-TWR 4 bản tin POLL → RESP → FINAL → REPORT, SS fallback khi mất REPORT | `tag_ranging.c:2101–2353`, `anchor_ranging.c:377–477` |
| Timing | Reply delay 1200 UUS (≈ 1,23 ms); RESP timeout 2600 µs; REPORT timeout 3000 µs | `anchor_ranging.h:29`, `tag_ranging.h:39, 78` |
| PHY | Ch5, PRF16, preamble 256, PAC16, 6,8 Mbps, PCODE 4 | `Firmware/Tag/src/drivers/dw1000.c:332–405` |
| TX power | `0x1E1E1E1E`, Smart TX power tắt (DIS_STXP) | `dw1000.c:337–339, 428–430` |
| Antenna delay | TX/RX = 16436 (mặc định), chưa hiệu chuẩn | `uwb_calibration.h`, `dw1000.c:407–416` |
| Calibration | `UWB_DS_CALIBRATED_MASK = 0`, offset DS = 0 → mọi range có `valid = 0` | `Firmware/Tag/include/uwb_app_config.h:17–26` |
| Bộ lọc trên TAG | Legacy Kalman (`double`) + Adaptive ở chế độ SHADOW chạy song song | `uwb_app_config.h:36` |
| Telemetry | UART0 115200; khung `AA55/VER=1/TYPE/LEN/SEQ/TIME + CRC16`; gói RANGE 8 anchor = 145 B | `Firmware/Tag/src/telemetry/telemetry.c` |
| Kênh lệnh | Không có. UART chỉ TX; chân TX ESP→nRF và RDY đã đi dây nhưng không dùng | `uart_tx_zephyr.c`, `ESP32C3_Gateway/main/main.c` |
| Lưu cấu hình | Không có; mọi thông số là macro build-time | — |
| Flash | DTS board đã có layout MCUboot: boot 48 KB, slot0/slot1 mỗi slot 200 KB, scratch 40 KB, `storage_partition` 0x7A000 (24 KB). Ảnh hiện tại: Anchor ≈ 26 KB, Tag_DevKit ≈ 41 KB | `Firmware/Anchor_1/build/zephyr/zephyr.dts:506–540`, `zephyr.bin` |
| Phần cứng module | DTS có `lis2dh12` (gia tốc kế) trên I2C0 | `zephyr.dts:199` |
| Git | Thư mục chưa phải Git repository | `git status` |
| Test | Host test C (driver, state TAG), compile check, parser gateway, parity anchor | `Firmware/tests/run_host_tests.ps1` |

### 2.2 Điểm mạnh cần giữ

- Delayed TX cho RESP, dự đoán RMARKER có cộng antenna delay (FIX-01) và tự đo sai số dự đoán trên anchor.
- Fail-closed: chưa calibration thì `valid = 0`, nhưng vẫn xuất raw để hiệu chuẩn.
- Bắt HPDWARN/TXPUTE ngay khi delayed TX bị trễ.
- Timeout µs an toàn khi counter wrap; xử lý IRQ trước khi xét timeout (FIX-02/03).
- Counter theo từng anchor, đo thời gian slot/chu kỳ, `DW1000_VerifyConfig()`.
- Protocol có CRC; parser streaming ở gateway và GUI.
- Host test chạy được mà không cần NCS.

---

## 3. Đối chiếu với Bitcraze LPS

### 3.1 Khác biệt về bài toán

| | Bitcraze LPS | Hệ DWM1001 này |
|---|---|---|
| Đối tượng | Nhiều Crazyflie nhỏ; anchor LPS node (STM32F072 + DWM1000) | 1 drone, TAG DWM1001C, 8 anchor DWM1001C |
| Ai tính vị trí | Crazyflie tự tính (EKF onboard) | UP 7000 (ADR-001); TAG chỉ đo |
| Chế độ chính | TDoA3 (tag chỉ nghe); TWR khi ít anchor | DS-TWR |
| Cấu hình | EEPROM + lệnh LPP qua sóng + menu USB | Macro build-time |
| Radio | Ch2, PRF64, preamble 128 (mặc định), Smart TX power | Ch5, PRF16, preamble 256, TX power đặt tay |

Mã Bitcraze là tham chiếu về **kiến trúc và tính năng hệ thống**, không phải để chép nguyên lõi đo.

### 3.2 Bảng tính năng

| # | Tính năng Bitcraze | Bitcraze làm thế nào | DWM1001 hiện tại | Quyết định | GĐ |
|---|---|---|---|---|---|
| B1 | Một firmware, nhiều chế độ | `uwbAlgorithm_t {init, onEvent}`; mode đọc từ cấu hình lúc boot (`src/uwb.c`) | 10 project copy | **Áp dụng**: mã chung, role qua Kconfig; sau đó role/ID qua settings | 0, 3 |
| B2 | Vòng radio hướng sự kiện | Task FreeRTOS chờ semaphore từ IRQ; `onEvent()` trả về timeout kế tiếp (`uwbTask`) | `while(1)` + `k_busy_wait(10)` | **Áp dụng**: thread Zephyr + `k_sem`; state machine trả về deadline | 3 |
| B3 | Cấu hình lưu bền | `cfg.c` ghi EEPROM: address, mode, anchor list, vị trí, TX power, PHY | Không có | **Áp dụng**: Zephyr settings trên `storage_partition` | 3 |
| B4 | Đổi cấu hình qua sóng (LPP short) | Gói `SHORT_LPP` (0xF0) đi qua tag tới anchor: vị trí, reboot, mode, TX power, PHY (`src/lpp.c`) | Không có | **Điều chỉnh**: thêm ACK/NACK, nonce, version; chỉ nhận khi ranging tạm dừng | 3 |
| B5 | Anchor phát vị trí | Kèm LPP position trong ANSWER (TWR) và trong mọi gói TDoA | Không có | **Điều chỉnh**: TLV thông tin anchor gửi thưa (theo yêu cầu), không nằm trong mọi RESP | 3 |
| B6 | Ràng buộc giao dịch | `payload[SEQ]` được echo trong ANSWER/FINAL/REPORT; tag so với `curr_seq` (`uwb_twr_tag.c`) | Không có (F4) | **Áp dụng** | 1 |
| B7 | Tự bật RX sau TX | `dwWaitForResponse(dev, true)` (WAIT4RESP) | MCU tự `StartRx` sau IRQ TXFRS | **Áp dụng** | 1 |
| B8 | REPORT chứa timestamp thô | Anchor gửi `pollRx, answerTx, finalRx` 40-bit, tag tự tính | REPORT chỉ chứa Rb 32-bit | **Cân nhắc** cho frame một-nhiều (giúp replay/kiểm chứng ở backend) | 5 |
| B9 | Bù bias theo công suất thu | `dwCorrectTimestamp()` nội suy bảng theo RX power (`libdw1000.c:702–750`) | Không có | **Điều chỉnh**: bảng riêng đo trên DWM1001C, đặt sau feature flag | 2 |
| B10 | Nạp XTAL trim từ OTP, chỉnh NTM | `libdw1000.c:1066` (LDE_CFG1), `:1237–1244` (OTP 0x1E → FS_XTALT) | Không có | **Áp dụng** | 2 |
| B11 | Smart TX power / TX power cấu hình được | Bảng TX power theo kênh/PRF; đổi qua LPP | Hardcode `0x1E1E1E1E` | **Áp dụng** sau khi đo | 2 |
| B12 | Sniffer | Nghe thụ động, xuất frame + timestamp nhị phân qua USB; script `tools/sniffer/*.py` | Không có | **Áp dụng**, chạy trên DWM1001-DEV | 2 |
| B13 | TDoA2 | TDMA 8 slot × ~2 ms, anchor 0 làm master | Không có | **Bỏ qua**: master là điểm chết đơn | — |
| B14 | TDoA3 | Anchor phát ngẫu nhiên, tự đo khoảng cách anchor–anchor, bù clock kiểu "bucket" | Không có | **Để dành** cho nhiều drone / tag thụ động | 6 |
| B15 | Khoảng cách anchor–anchor | Có sẵn trong TDoA2/TDoA3 | Không có | **Điều chỉnh**: chế độ đo anchor–anchor để tự hiệu chuẩn bản đồ | 6 |
| B16 | Reboot về bootloader qua sóng | `LPP_SHORT_REBOOT` + DFU qua USB | Không có; nạp bằng SWD | **Điều chỉnh**: MCUboot + mcumgr (UART cho Tag, BLE cho anchor) | 6 |
| B17 | Menu cấu hình qua serial | `main.c`: đổi ID, mode, reset cấu hình | Không có | **Áp dụng**: Zephyr shell qua RTT/UART khi commissioning | 3 |
| B18 | Unit test + CI | Unity + GitHub Actions | Host test PowerShell | **Mở rộng**: vector test DS/timestamp, fuzz parser, CI khi có Git remote | 0 |

### 3.3 Những điểm không nên sao chép từ Bitcraze

- **Offset antenna hằng số 154,6 m** (`ANTENNA_OFFSET` trong `uwb_twr_anchor.c`, `uwb_tdoa_anchor3.c`) cộng/trừ thẳng vào timestamp. Đây đúng là kiểu "legacy offset" mà dự án đã bỏ để dùng antenna delay phần cứng.
- **Frame 802.15.4 địa chỉ 64-bit** (header 21 byte) với ID 8-bit: tăng airtime, trong khi địa chỉ ngắn 16-bit hiện có là đủ.
- **ANSWER gửi ngay, không dùng delayed TX**: chỉ hợp lệ khi REPORT mang timestamp thật, và mất khả năng SS fallback.
- **Lệnh LPP không có ACK và reboot ngay**: không chấp nhận được với drone; cần ACK/NACK và khoá khi đang vận hành.
- **Chọn kênh/PRF theo Bitcraze** (Ch2/PRF64): không đổi PHY chỉ vì Bitcraze dùng; mỗi lần đổi PHY phải hiệu chuẩn lại.
- **Giấy phép**: `lps-node-firmware` theo LGPL-3.0, `libdw1000` theo Apache-2.0. Nên viết lại theo khái niệm; nếu chép bảng/mã (ví dụ bảng bias) thì giữ ghi công và điều khoản giấy phép tương ứng.

---

## 4. Đánh giá bản kế hoạch trước

Bản được đánh giá: `Plan/KE_HOACH_NANG_CAP_FIRMWARE_DW1001.md` (2026-09-19, P0–P4).

| Mục trong bản trước | Nhận định | Xử lý trong kế hoạch này |
|---|---|---|
| Baseline | Dẫn `Kconfig`, `config/anchor_N.conf`, `TAG_NUM_ANCHORS = 4`, tức là `Firmware Code Base/`. Nguồn active `Firmware/` đã có 8 anchor và không dùng Kconfig. | Dùng `Firmware/` làm baseline (mục 2). |
| "Offset đã validate (A1–A3 SS, A1–A4 DS)", "offset ~154–157 m" | Không đúng với DWM1001: DS mask = 0, offset DS = 0. Giá trị 156,x m là offset SS của hệ STM32 cũ. Mảng này vẫn còn ở `tag_ranging.c:50–59` nhưng không active (`UWB_USE_LEGACY_OFFSET = 0`). | Hiện chưa có calibration nào cần giữ nguyên. Hiệu chuẩn sau GĐ2. |
| P0 — anchor gửi vị trí trong mọi RESP để TAG tự trilateration | Việc TAG tự định vị đi ngược ADR-001 (UP 7000 mới định vị). Gửi vị trí trong mọi RESP tốn airtime ×8 anchor ×50 Hz mà không đem lại gì thêm. Phần lập luận tương thích ngược thì hợp lý. | Giữ ý tưởng, đổi thành TLV thông tin anchor gửi thưa để backend đối chiếu bản đồ (3.6). |
| P1 — Discovery, refactor `TAG_NUM_ANCHORS → MAX = 6` | Bảng anchor đã là 8. Discovery ngẫu nhiên ít giá trị khi anchor cố định và đã được khảo sát vị trí. Giới hạn 6 anchor mâu thuẫn mục tiêu 8 anchor/3D. | Thay bằng danh sách anchor cấu hình được qua lệnh, cùng scheduler theo sức khoẻ anchor (3.4, GĐ5). |
| P2 — OTA command, NVS | Đúng hướng. Nhưng phân vùng đề xuất `0x7E000 / 8 KB` **trùng** `storage_partition` 0x7A000–0x80000 đã có trong DTS board. Gửi command ở khoảng rảnh cuối chu kỳ là hợp lý. | Dùng `storage_partition` sẵn có + Zephyr settings; thêm ACK/nonce/khoá (3.2–3.5). |
| P3 — Multi-PHY | Hợp lý, nhưng giá trị thấp so với chi phí hiệu chuẩn lại. | Giữ ở GĐ6, dạng compile-time. |
| P4 — TDoA | Hợp lý, rủi ro cao. | GĐ6, chỉ khi cần nhiều drone; chọn kiểu TDoA3. |
| Phần còn thiếu | Lỗi F1–F10, driver RF (N1–N3), hiệu năng SPI, telemetry v2/đồng bộ thời gian, DFU, gộp mã nguồn, chiến lược test. | Bổ sung trong kế hoạch này. |

---

## 5. Phát hiện kỹ thuật trong mã nguồn

### 5.1 Trạng thái F1–F10 (review 2026-09-15)

Đã kiểm lại trên mã hiện tại: **chưa mục nào được sửa**.

- **F1**: `select_poll_anchor()` vẫn quét từ `first_idx` tăng dần (`tag_ranging.c:1791–1809`), còn `anchor_should_poll()` chỉ cho 1 probe mỗi chu kỳ (`:1771–1789`), nên anchor cuối danh sách có thể không bao giờ được probe.
- **F2**: `fatal_blink()` vẫn feed watchdog vô hạn (`Firmware/Tag/src/main.c:59–66`, `Firmware/Anchor_1/src/main.c:57–64`).
- **F3**: điều kiện nhận frame chỉ kiểm `DW_ALL_RX_GOOD` (`tag_ranging.c:2039, 2266`; `anchor_ranging.c:289, 384`).
- **F4**: không có transaction ID; `s_seq_num` (TAG) và `s_tx_seq` (anchor) tăng độc lập với nhau.
- **F5–F10**: vẫn như mô tả trong review.

### 5.2 Phát hiện bổ sung

| ID | Mức | Phát hiện | Bằng chứng | Trạng thái |
|---|---|---|---|---|
| N1 | Cao | TX_POWER `0x1E1E1E1E` khi Smart TX power tắt tương đương gain ≈ 30 dB (coarse 15 dB + fine 15 dB). Mức này cao hơn ≈ 16 dB so với giá trị tham chiếu `0x48484848` (≈ 14 dB) cho Ch5/PRF16 trong DW1000 User Manual và `libdw1000`. Rủi ro: vượt mặt nạ phổ −41,3 dBm/MHz; bão hoà máy thu ở cự ly gần, gây bias khoảng cách và sai lệch FPP/RX power. | `dw1000.c:337–339, 428–430`; `libdw1000.c`, bảng Ch5: `0x0E082848` (smart) / `0x48484848` (đặt tay) | Mã: [Đã kiểm]. Ảnh hưởng RF: cần đo |
| N2 | Cao | Thiếu các bước cấu hình khuyến nghị: không ghi LDE_CFG1 (NTM = 13), không nạp LDOTUNE, không nạp XTAL trim từ OTP vào FS_XTALT. Không trim XTAL thì lệch clock giữa các node lớn hơn cần thiết, làm SS fallback sai nhiều (mục 5.3). | `DW1000_Configure()` `dw1000.c:332–458` không có các thanh ghi này; `libdw1000.c:1066, 1237–1244` có | [Đã kiểm] |
| N3 | TB | Sau lỗi RX chỉ làm TRXOFF + RXENAB, không RX soft-reset (Decawave khuyến nghị reset RX để khởi tạo lại LDE sau sự kiện lỗi). | `anchor_ranging.c:117–124`; `tag_ranging.c:2039–2048` | [Đã kiểm] |
| N4 | Cao (hiệu năng) | Mỗi lần đọc/ghi thanh ghi dùng 2–4 lệnh `spi_transceive()` riêng (từng byte header, rồi data), CS điều khiển bằng GPIO. Một trao đổi DS có hàng chục lần truy cập thanh ghi, nên overhead SPI có thể chiếm 0,5–1,5 ms mỗi anchor [Ước lượng]. | `dw1000.c:131–157, 175–199`; `uwb_platform_spi_transfer()` | Mã: [Đã kiểm]. Thời gian: cần đo |
| N5 | Thấp | `DW1000_ForceRxOff()` luôn busy-wait 100 µs. Hàm này được gọi ở mọi lần chuyển slot và mọi đường lỗi; anchor gọi cả khi nhận frame lạ. | `dw1000.c:773–781`; `tag_ranging.c:1842`; `anchor_ranging.c:119` | [Đã kiểm] |
| N6 | TB | Không dùng WAIT4RESP, nên bước FINAL → REPORT là một cuộc đua: anchor gửi REPORT ngay lập tức, còn TAG phải nhận IRQ TXFRS và đọc timestamp rồi mới bật RX. Nếu TAG trễ thì mất đầu preamble của REPORT: timeout tăng hoặc timestamp kém chất lượng. | `tag_ranging.c:2240–2247`; `anchor_ranging.c:438–442` | Logic: [Đã kiểm]. Tần suất: cần đo |
| N7 | Cao | 8 anchor DS tuần tự không vừa ngân sách 20 ms (mục 5.3). | Tính toán | [Ước lượng] |
| N8 | TB | Bộ lọc trên TAG dùng `double`, trong khi Cortex-M4F chỉ có FPU đơn nên `double` chạy bằng soft-float. Chế độ SHADOW còn chạy song song bộ lọc thứ hai cho mỗi mẫu. Việc này tốn CPU trong slot, trong khi kiến trúc mới đặt estimator ở UP 7000. | `tag_ranging.c:482–489, 515–520`; `uwb_app_config.h:36` | [Đã kiểm] |
| N9 | TB | Watchdog được feed vô điều kiện mỗi vòng lặp: chỉ bắt được treo vòng lặp, không bắt được trường hợp state machine vẫn chạy nhưng không còn đo. | `Firmware/Tag/src/main.c:118`; `Firmware/Anchor_1/src/main.c:85` | [Đã kiểm] |
| N10 | TB | Không có UART RX / kênh lệnh; chân ESP TX → nRF RX và chân RDY đã đi dây nhưng firmware không dùng. | `uart_tx_zephyr.c`; `Firmware/HARDWARE_COMPATIBILITY.md` | [Đã kiểm] |
| N11 | TB | Anchor chỉ nhận POLL/FINAL từ `TAG_ADDR = 0`: cứng một tag, không có đường cho chế độ đo anchor–anchor. | `anchor_ranging.c:302–306, 394–398` | [Đã kiểm] |
| N12 | TB | Mã nhân bản: 10 bản `dw1000.c`, 2 bản `tag_ranging.c`. Parity test chỉ phát hiện lệch, không ngăn được việc sửa sót. `Firmware Code Base/` có sẵn khung Kconfig một ứng dụng nhưng mã đã cũ (4 anchor). | `diff -rq`; `Firmware Code Base/Kconfig` | [Đã kiểm] |
| N13 | TB | Chưa có Git: không có baseline để A/B, không truy được binary nào đang nạp trên board nào. | `git status` | [Đã kiểm] |
| N14 | Thấp | Nhiều chú thích lỗi thời: bit-bang ~33 kHz, EXTI3/PA3 của STM32, header TAG ghi "SS-TWR" và reply delay "2500 UUS". | `dw1000.c:88–91`; `anchor_ranging.c:27–31`; `anchor_ranging.h:24–27`; `tag_ranging.h:4` | [Đã kiểm] |
| N15 | Thông tin | Gateway và GUI Python từ chối frame có `VER != 1`; gateway giới hạn payload 256 B. Telemetry v2 phải tương thích với ràng buộc này. | `ESP32C3_Gateway/main/telemetry_protocol.h:13–17`; `Software/UWB_UART_GUI/telemetry_protocol.py:196–198` | [Đã kiểm] |

### 5.3 Ngân sách thời gian và sai số SS [Ước lượng]

**Airtime một frame ngắn** (12–16 B, preamble 256, PRF16, 6,8 Mbps): SHR ≈ 262 µs + PHR ≈ 25 µs + data ≈ 20 µs, **tổng ≈ 0,31 ms**.

| Bước trong một slot DS tuần tự | Thời gian |
|---|---|
| POLL: từ lúc bắt đầu TX tới RMARKER | ≈ 0,26 ms |
| Reply delay anchor (1200 UUS) | 1,23 ms |
| Phần cuối RESP + IRQ | ≈ 0,05 ms |
| TAG xử lý RESP, nạp FINAL (≈ 9 lần truy cập thanh ghi) | 0,3–0,7 ms |
| FINAL airtime | 0,31 ms |
| Anchor xử lý FINAL, nạp REPORT | 0,2–0,45 ms |
| REPORT airtime | 0,31 ms |
| TAG xử lý REPORT + tính DS + lọc (`double`, SHADOW) | 0,15–0,5 ms |
| Chuyển slot: TRXOFF 100 µs + guard 150 µs + SPI | ≈ 0,28 ms |
| **Tổng mỗi anchor** | **≈ 3,1–4,1 ms** |

Kết quả: 4 anchor ≈ 12–16 ms (vừa 20 ms); **8 anchor ≈ 25–33 ms (không vừa)**. Số thật lấy từ `anchor_slot_duration_max_us`, `anchor_processing_max_us` và `tag_cycle_duration_max_us`. Các counter này đã có, hiện đọc qua debugger; GĐ4 sẽ đưa chúng lên telemetry.

**Sai số SS fallback do lệch clock** ≈ ½ × T_reply × Δf. Với T_reply = 1,23 ms và Δf = 10 ppm: ≈ 6 ns, tức **≈ 1,8 m**. Hiện `UWB_SS_RESIDUAL_CALIBRATED_MASK = 0` nên SS fallback luôn bị gắn `CAL_MISSING`. Cách này an toàn, nhưng có nghĩa mỗi lần mất REPORT là mất một mẫu. Muốn SS fallback dùng được thì cần trim XTAL (N2) và bật hiệu chỉnh carrier integrator (`UWB_USE_CLOCK_CORRECTION`).

---

## 6. Nguyên tắc nâng cấp

1. **TAG đo, UP 7000 định vị** (ADR-001). Firmware ưu tiên phép đo thô đúng, timestamp đúng, chỉ số chất lượng tín hiệu và trạng thái lỗi trung thực. Không đưa trilateration/EKF lên TAG, trừ một chế độ debug tuỳ chọn.
2. **Giữ fail-closed**: chưa calibration hoặc có cờ lỗi RX thì `valid = 0`.
3. **Mọi thay đổi hành vi đều có cờ** (Kconfig/macro), mặc định giữ hành vi cũ cho tới khi A/B trên board đạt. Sau đó mới đổi mặc định và xoá nhánh cũ.
4. **Wire format có version**: frame UWB mang byte phiên bản. Telemetry thêm TYPE mới thay vì sửa TYPE cũ khi GUI/gateway cũ còn dùng.
5. **Không ghi flash khi đang ranging**: xoá/ghi trang flash trên nRF52 làm CPU dừng (thời gian theo datasheet nRF52832), nên chỉ ghi settings khi ranging đã tạm dừng.
6. **Không hiệu chuẩn chính thức trước khi xong GĐ2**: TX power, XTAL trim, NTM và bù bias đều làm dịch bias.
7. **Một nguồn mã duy nhất** cho mọi node; khác biệt giữa các node chỉ nằm ở cấu hình.
8. **Mỗi GĐ kết thúc bằng số đo** (log A/B, counter, soak). "Build OK" không phải tiêu chí nghiệm thu.
9. **Quy trình làm việc**: mỗi GĐ có spec → code → host test → user build/nạp/đo → chốt. Nếu nhờ công cụ AI khác viết code, spec phải đủ chi tiết để không cần đọc lại toàn bộ source.

---

## 7. Lộ trình chi tiết

### GĐ0 — Nền móng (1–1,5 tuần)

**Mục tiêu:** có baseline truy vết được và một nguồn mã chung, không đổi hành vi.

| ID | Công việc | Chi tiết | File |
|---|---|---|---|
| 0.1 | Git baseline | `git init`, commit toàn bộ trạng thái hiện tại, tag `baseline-2026-09-19`. Lập `Firmware/DEPLOYMENT_MANIFEST.md` ghi serial module ↔ vai trò ↔ hash binary đang nạp. | gốc repo |
| 0.2 | Tách mã chung, bước 1 | Tạo `Firmware/common/{include,src}` chứa `dw1000.c/.h`, `uwb_platform_zephyr.c`, `uwb_calibration.h`, `anchor_ranging.c/.h`, `tag_ranging.c/.h`, filters, telemetry. Giữ nguyên 10 thư mục project; `CMakeLists.txt` của chúng trỏ tới `../common`. Mỗi project chỉ còn `prj.conf`, `app.overlay`, `uwb_app_config.h` và scripts, để VSCode task và script build/flash cũ vẫn chạy. | `Firmware/*/CMakeLists.txt` |
| 0.3 | Tách mã chung, bước 2 (bắt buộc trước GĐ3) | Một ứng dụng `Firmware/uwb_node/`: Kconfig chọn vai trò (kế thừa khung `Firmware Code Base/Kconfig`: `UWB_ROLE_TAG/ANCHOR/SNIFFER`), overlay theo board (`boards/custom_carrier.overlay`, `boards/dwm1001_dev.overlay`), file cấu hình `conf/anchor_N.conf`. `build_all.ps1` build 1 tag + 8 anchor + devkit từ một nguồn. | mới |
| 0.4 | Build ID | Nhúng git hash, cờ dirty, thời gian build và CRC32 của cấu hình build vào gói INFO. | telemetry, CMake |
| 0.5 | Mở rộng test | Vector test DS-TWR (giá trị chuẩn tính tay); wrap timestamp 40-bit; wrap CYCCNT; bộ "golden frame" dùng chung cho parser C và Python; chạy host test với `-fsanitize=address,undefined`. | `Firmware/tests/` |
| 0.6 | Dọn chú thích lỗi thời (N14) | Không đổi logic. | nhiều file |

**Nghiệm thu:**
- Build từ mã chung, nạp lên board, so với baseline: `DW1000_VerifyConfig() = 0`; INFO giống nhau (trừ build ID); RANGE/STATS giống về tần số và phân bố trạng thái trong 10 phút với 1–2 anchor.
- Toàn bộ host test pass. `test_anchor_projects.py` được thay bằng kiểm tra cấu hình (địa chỉ anchor duy nhất).

**Rủi ro:** đổi đường dẫn include làm lệch macro giữa các vai trò. Giảm rủi ro bằng cách so sánh `.map` và dump macro trước/sau.

### GĐ1 — Độ tin cậy và sửa lỗi (1,5–2 tuần)

| ID | Công việc | Chi tiết | Nghiệm thu |
|---|---|---|---|
| 1.1 | F1: probe công bằng; F8: tách sức khoẻ | Con trỏ round-robin riêng cho anchor offline, chọn anchor quá hạn lâu nhất; mỗi anchor có thời gian chờ probe tối đa xác định. Tách "sức khoẻ RESP" khỏi "sức khoẻ hoàn tất DS": anchor trả RESP nhưng liên tục mất REPORT cũng phải được backoff. | Host test: 8 anchor offline trong 500 chu kỳ → số lần probe mỗi anchor chênh nhau ≤ 1; anchor ID lớn hồi phục ≤ 1 s sau khi bật lại. |
| 1.2 | F3 + N3: phân loại lỗi RX | Chỉ nhận frame khi có đủ `RXDFR`, `RXFCG`, `LDEDONE` và **không** có `RXPHE/RXFCE/RXRFSL/LDEERR/RXOVRR/RXSFDTO/RXPTO`. Mọi lỗi → RX soft-reset (PMSC) rồi bật lại RX; đếm riêng từng loại lỗi. | Host test đủ tổ hợp cờ; trên board có counter lỗi theo loại. |
| 1.3 | F4: transaction ID | POLL mang `txn` (u8, tăng mỗi slot); RESP/FINAL/REPORT echo `txn`. Bên nhận coi frame lệch `txn` là frame lạ và giữ nguyên deadline. Đổi wire format thì phải cập nhật đồng bộ Tag + 8 anchor và thêm byte version (mục 8.1). | Probe RESP sai `txn` bị loại; không phát sinh range sai. |
| 1.4 | N6: WAIT4RESP | Bật WAIT4RESP khi TX POLL/FINAL (TAG) và RESP (anchor), đặt `W4R_TIM` phù hợp. Bật RX frame-wait timeout phần cứng (RXWTOE + RX_FWTO) làm lớp timeout đầu tiên; timeout phần mềm giữ làm backstop. | `ds_report_timeout_count` giảm về ≈ 0; slot ngắn hơn. |
| 1.5 | F2 + N9: recovery | Trạng thái `FAULT → RECOVERING`: đánh dấu mọi range invalid, ghi nguyên nhân, reset + khởi tạo lại DW1000 tối đa 3 lần có backoff, sau đó reset MCU. Lưu `reset_cause` (API hwinfo) và `boot_count` (RAM noinit; lưu bền ở GĐ3). Watchdog chỉ được feed khi có tiến triển: TAG khi `tag_cycle_count` tăng trong 200 ms; anchor khi radio còn quay vòng RX bình thường. Tách `Tag_Init()` thành cold-init và warm-reset cho state machine/filter. | Fault injection (làm hỏng SPI, làm DW1000 treo) → node tự hồi phục; không rơi vào vòng reboot. |
| 1.6 | N5 | Thay delay cố định 100 µs bằng chờ trạng thái radio về IDLE, có trần 100 µs. | Slot time giảm, lỗi không tăng. |
| 1.7 | F5: quy ước dấu calibration | Đổi tên rõ nghĩa `bias_to_subtract_mm` ở firmware; exporter GUI đổi dấu và có test. | Test chuyển đổi hai chiều. |
| 1.8 | Chế độ 4 anchor | Cấu hình tập anchor hoạt động (A1–A4) để phục vụ Phase 2D mà không phải chờ GĐ5. | 4 anchor, mỗi anchor ≥ 45 Hz khi khoẻ. |

**Nghiệm thu GĐ1:** soak 2 h với 4 anchor, ngắt/cấp nguồn anchor ngẫu nhiên mỗi 30–120 s: không treo; mọi anchor hồi phục ≤ 1 s; không có range `valid` nào đến từ frame lỗi. Các test F1/F3/F4 trở thành regression test.

### GĐ2 — Driver RF, hiệu năng SPI, chẩn đoán (2 tuần + thời gian đo)

| ID | Công việc | Chi tiết |
|---|---|---|
| 2.1 | N4: SPI | Mỗi lần truy cập thanh ghi dùng **một** `spi_transceive()` với `spi_buf_set` [header, data]. Để driver SPI tự điều khiển CS (`spi_cs_control` lấy từ devicetree) thay vì GPIO thủ công. Gộp các lần đọc liền kề khi địa chỉ cho phép. Đo `anchor_processing_max_us` trước và sau. |
| 2.2 | Công cụ đọc OTP | Chế độ chẩn đoán (role SNIFFER/DIAG) in ra: part ID, lot ID, LDOTUNE, XTAL trim (OTP 0x1E), và antenna delay/TX power nếu nhà sản xuất có ghi. Chạy trên cả 9 module và lưu thành bảng. |
| 2.3 | N2: cấu hình khuyến nghị | LDE_CFG1 NTM = 13; nạp LDOTUNE từ OTP nếu khác 0; FS_XTALT = trim từ OTP, nếu không có thì dùng giữa dải (0x10). Đặt sau cờ `UWB_DW_REF_TUNING`. Mở rộng `DW1000_VerifyConfig()` cho các thanh ghi mới. |
| 2.4 | N1: TX power | A/B ba cấu hình: hiện tại `0x1E1E1E1E`; tham chiếu đặt tay `0x48484848`; Smart TX power `0x0E082848`. Nếu OTP module có giá trị TX power hiệu chuẩn từ nhà máy (theo 2.2) thì ưu tiên giá trị đó. Mỗi cấu hình đo ở 0,5 / 1 / 2 / 5 / 10 m LOS: mean/std range, tỉ lệ thành công, FPP, RX power. Kiểm mặt nạ phổ nếu có thiết bị. Chọn một cấu hình; từ GĐ3 cho phép chỉnh qua settings. |
| 2.5 | Chẩn đoán tín hiệu đầy đủ | Đọc thêm CIR_PWR, STD_NOISE, FP_INDEX (RXPACC đã có). Tính RX level và chênh lệch `RX − FP`, chỉ báo NLOS theo APS006 Part 3. Đổi carrier integrator ra ppm. Nhiệt độ/điện áp SAR (blocking ≈ 1 ms) chỉ đọc giữa các chu kỳ, tối đa 1 Hz. Dữ liệu đưa lên host qua telemetry v2 (GĐ4). |
| 2.6 | B9: bù bias theo công suất thu | Cờ `UWB_RANGE_BIAS_CORRECTION`. Khởi điểm dùng bảng của `libdw1000` (500 MHz, PRF16), sau đó thay bằng bảng đo trên DWM1001C (kế hoạch drone §10.2). Ưu tiên làm ở host trước, vì host đã có raw + RX power; chỉ đưa vào firmware nếu cần. INFO phải ghi rõ bù bias đang bật hay tắt. |
| 2.7 | B12: sniffer | Role SNIFFER: RX liên tục; mỗi frame xuất `{rx_ts 40-bit, len, bytes, fp_power, rx_power}` qua UART, dùng framing telemetry với TYPE mới. Script Python giải mã POLL/RESP/FINAL/REPORT, vẽ timeline slot, phát hiện va chạm. Chạy trên DWM1001-DEV (có J-Link VCOM). |
| 2.8 | Antenna delay theo APS014 | Cho phép đặt TX/RX antenna delay lúc chạy qua lệnh (GĐ3) mà không build lại; script host thực hiện phương pháp 3 thiết bị. |

**Nghiệm thu GĐ2:**
- Có bảng A/B RF (TX power × khoảng cách) và đã chọn cấu hình chính thức.
- Slot time trung bình giảm ≥ 20% so với sau GĐ1 [mục tiêu, xác nhận bằng đo].
- Sniffer ghi được đủ chuỗi 4 bản tin của 4 anchor.

**Sau GĐ2 mới bắt đầu chiến dịch hiệu chuẩn** (antenna delay + bias) trên phần cứng.

### GĐ3 — Kiến trúc runtime, cấu hình bền, kênh lệnh (2–3 tuần)

| ID | Công việc | Chi tiết |
|---|---|---|
| 3.1 | B2: mô hình thread | Thread `uwb` ưu tiên cao (cooperative) chờ `k_sem` từ ISR GPIO hoặc tới deadline; state machine trả về deadline kế tiếp, giống `onEvent()` của Bitcraze. Thread `telemetry` (ưu tiên thấp hơn) lấy record từ `k_msgq`; thread `cmd` parse UART RX. Bỏ vòng lặp `k_busy_wait(10)`. |
| 3.2 | B3: settings | Zephyr settings (backend NVS hoặc ZMS) đặt trên `storage_partition` (0x7A000, 24 KB); không tự định nghĩa phân vùng mới. Khoá: `node/role`, `node/id`, `radio/tx_power`, `radio/ant_dly_tx`, `radio/ant_dly_rx`, `radio/xtal_trim_override`, `anchor/pos_mm`, `tag/anchor_list`, `cal/profile`. `cal/profile` gồm offset/bias theo anchor + mode + PHY, kèm version và CRC. Giá trị mặc định lấy từ Kconfig; có lệnh `factory_reset`. |
| 3.3 | Định danh node | ID anchor lấy từ settings. Dự phòng: bảng `FICR DEVICEID → ID` nhúng trong build, cho phép **một binary dùng chung cho cả 8 anchor**. |
| 3.4 | N10: kênh lệnh UART | Host (UP 7000, hoặc GUI qua gateway) → TAG. Dùng cùng framing telemetry với TYPE lệnh, `cmd_id`, ACK/NACK + mã lỗi; host lo timeout/retry; lệnh idempotent. Lệnh ban đầu: `GET_INFO`, `GET_DIAG`, `SET_ANCHOR_LIST`, `SET_CAL`, `SET_RADIO`, `PAUSE_RANGING` / `RESUME`, `SAVE`, `REBOOT`, `TIME_SYNC` (GĐ4), `ENTER_CAL_MODE`. Lệnh ghi bị từ chối khi chưa `PAUSE_RANGING`, và khi host đã đặt cờ `LOCK` (UP 7000 đặt cờ này khi PX4 armed). |
| 3.5 | B4: cấu hình anchor qua UWB | TAG chuyển lệnh tới anchor trong khoảng rảnh cuối chu kỳ, hoặc khi ranging tạm dừng. Frame `CFG` (func mới) gồm `txn`, nonce, payload TLV; anchor trả `CFG_ACK` kèm mã kết quả; retry ≤ 3 lần. Nhóm lệnh: đặt vị trí, antenna delay, TX power, đổi ID, lưu, reboot. |
| 3.6 | B5: thông tin anchor | TLV `ANCHOR_INFO` gồm ID, vị trí (mm, int32 × 3), antenna delay, build ID, CRC cấu hình và cờ "bị dịch chuyển" (từ 6.4). Chỉ gửi khi POLL đặt cờ yêu cầu; TAG yêu cầu xoay vòng khoảng 1 anchor mỗi giây. TAG xuất ra `TELEM_TYPE_ANCHOR_INFO`. Backend so với bản đồ khảo sát; không dùng làm ground truth. |
| 3.7 | B17: shell | Zephyr shell qua RTT (anchor, vì UART đã tắt) hoặc UART (DWM1001-DEV) cho commissioning tại bàn: xem/đổi settings, đọc OTP, test radio. Tắt trong build dùng để bay. |

**Nghiệm thu GĐ3:**
- Từ GUI đặt antenna delay và vị trí cho một anchor thông qua TAG → nhận ACK → ngắt điện → giá trị vẫn còn.
- Settings hỏng CRC → tự về mặc định và báo lỗi.
- Khi không có lệnh, ranging không bị ảnh hưởng (so counter với sau GĐ2).

### GĐ4 — Telemetry v2, đồng bộ thời gian, gateway (2 tuần)

| ID | Công việc | Chi tiết |
|---|---|---|
| 4.1 | Record theo từng phép đo | TYPE `0x10 RANGE_MEAS`: gửi một record ngay khi mỗi phép đo xong, không chờ đủ chu kỳ (8.3). Giữ TYPE `0x01` (snapshot) sau cờ trong giai đoạn chuyển tiếp để GUI cũ vẫn chạy. |
| 4.2 | TYPE `0x11 DIAG` 1 Hz | Counter theo anchor (timeout RESP/REPORT, lỗi RX theo loại, skip/backoff, cal missing), max thời gian slot, max chu kỳ, lỗi SPI, reset cause, boot count, uptime, build ID, calibration ID, nhiệt độ/điện áp, high-water/drop của hàng đợi UART. Chia thành nhiều frame nếu vượt 256 B (giới hạn gateway). |
| 4.3 | Thời gian | `boot_id` ngẫu nhiên mỗi lần boot; `tag_time_us` 64-bit; `meas_time_us` = trung điểm từ POLL TX tới REPORT RX. Quy đổi thời gian DW1000 sang thời gian MCU bằng cặp mẫu SYS_TIME / cycle counter lấy mỗi chu kỳ. |
| 4.4 | Đồng bộ với host | Lệnh `TIME_SYNC` theo t1…t4 (kế hoạch drone §5.3); TAG trả t2/t3 theo `tag_time_us`. |
| 4.5 | UART | Chuyển sang UART async/DMA API của Zephyr. Thử 921600 (CP2102N) sau khi 115200 đã ổn định; baud là một settings. Ngân sách: record ≈ 64 B (kể cả header) × 8 anchor × 50 Hz ≈ 25,6 kB/s, **vượt trần 115200** (≈ 11,5 kB/s), nên với 8 anchor cần tối thiểu 460800, khuyến nghị 921600. |
| 4.6 | Gateway | Chuyển tiếp lệnh USB → UART; phát health frame (forwarded/drop/CRC/UART overflow); dùng event queue của UART; xử lý USB short-write. |
| 4.7 | N8: bộ lọc trên TAG | Đưa bộ lọc TAG về vai trò chẩn đoán: bỏ SHADOW khỏi build dùng để bay; chuyển Kalman sang `float`; chỉ giữ `double` cho phép toán timestamp. Estimator chính chạy ở UP 7000. |
| 4.8 | Framing | Giữ `SOF + LEN + CRC16` và `VER = 1` ở lớp khung để gateway/GUI hiện tại không bị vỡ; thêm byte schema trong payload của TYPE mới. Chỉ chuyển sang COBS + CRC32C nếu benchmark ở 921600 cho thấy cần (kế hoạch drone §5.2). |

**Nghiệm thu GĐ4:**
- Soak 24 h qua cả hai đường (ESP32-C3 gateway và CP2102N → PC): 0 lỗi CRC trên bench; mọi gap sequence đều được đếm.
- GUI cũ vẫn chạy với TYPE cũ; parser Python đọc đúng TYPE mới theo golden vector.

### GĐ5 — Lịch radio cho 8 anchor (2–3 tuần)

**Điều kiện bắt đầu:** số đo thật sau GĐ2 cho thấy 8 anchor tuần tự không giữ được margin ≥ 25% trong 20 ms (dự kiến đúng như vậy). GĐ5 chỉ cần trước Phase 10 (8 anchor/3D) của kế hoạch drone.

| Phương án | Mô tả | Frame/chu kỳ (8 anchor) | Chu kỳ [Ước lượng] | Quyết định |
|---|---|---|---|---|
| A | 2 nhóm × 4 anchor xen kẽ (kế hoạch drone §8.2), DS tuần tự | 16 | 12–16 ms; mỗi anchor 25 Hz | Dự phòng nhanh |
| B | DS-TWR một-nhiều: POLL broadcast → RESP theo slot → FINAL broadcast → REPORT theo slot | 18 | ≈ 9–15 ms; mỗi anchor 50 Hz | **Mục tiêu** |
| C | Biến thể pipeline: bỏ REPORT; Rb của lần trước đi kèm RESP lần sau; FINAL có thể gộp với POLL chu kỳ sau | 9–10 | ≈ 5–8 ms | R&D sau khi B ổn định |
| D | TDoA3 kiểu Bitcraze | Anchor tự phát | Tag thụ động | GĐ6, khi cần nhiều drone |

**Thiết kế B** (đặc tả ở 8.2):
- POLL_B chứa danh sách anchor theo thứ tự slot của chu kỳ đó. Nhờ vậy TAG tự chọn tập anchor: phương án A trở thành một trường hợp riêng của B, anchor offline bị bỏ mà không tốn slot, và probe công bằng được chèn vào.
- Anchor ở slot k phát tại `T_pollrx + D0 + k × S` (delayed TX, căn 512 tick). Mọi anchor tham chiếu cùng một POLL, nên **không cần đồng bộ clock giữa các anchor**: lệch clock trong ~5 ms × 20 ppm ≈ 0,1 µs, rất nhỏ so với guard.
- `S` = airtime RESP (≈ 0,31 ms) + thời gian TAG đọc frame và bật lại RX + guard. Bước 1: single-buffer, S ≈ 0,6–0,75 ms. Bước 2 (tuỳ chọn): RX double-buffer, S ≈ 0,4 ms.
- FINAL_B phát broadcast sau slot cuối, mang bitmap các anchor mà TAG đã nhận được RESP. Anchor k gửi REPORT tại `T_finalrx + D1 + rank_k × S`; chỉ anchor có trong bitmap mới gửi, nên không phí slot cho anchor bị mất.
- Rủi ro chính: va chạm slot nếu tính sai thời điểm, và delayed TX bị trễ nếu D0 quá nhỏ. Hai công cụ bắt buộc là sniffer (2.7) và counter HPDWARN.

**Nghiệm thu GĐ5:**
- 8 anchor khoẻ: p99 chu kỳ ≤ 15 ms (margin ≥ 25%); tỉ lệ thành công ≥ 98% mỗi anchor khi LOS ≤ 10 m; ≥ 45 Hz mỗi anchor.
- Tắt một anchor → các anchor khác không giảm tần số; bật lại → hồi phục ≤ 200 ms.
- So sánh B với tuần tự tại cùng vị trí: chênh lệch trung bình ≤ 1 cm [mục tiêu].

### GĐ6 — Mở rộng theo nhu cầu

| ID | Hạng mục | Khi nào làm | Ghi chú |
|---|---|---|---|
| 6.1 | Chế độ đo anchor–anchor (B15) | Trước Phase 9 (auto-calibration) của kế hoạch drone | Lệnh cho anchor i làm initiator tới anchor j, dùng lại DS-TWR. Kết quả gửi về qua TAG hoặc sniffer, tạo ma trận khoảng cách cho MDS. Cần sửa N11 (anchor chấp nhận initiator khác TAG khi ở chế độ calibration). |
| 6.2 | DFU | Khi việc nạp lại anchor trên trần trở thành gánh nặng | DTS đã có layout MCUboot; slot 200 KB dư nhiều so với ảnh 26–41 KB. Tag: mcumgr SMP qua UART (gateway hoặc CP2102N). Anchor: SMP qua BLE, hoặc relay qua UWB (chậm). Ảnh có chữ ký, cơ chế test/confirm và rollback. |
| 6.3 | BLE commissioning | Khi cần đổi cấu hình anchor mà không đi qua TAG | Chỉ bật ở chế độ bảo trì. BLE controller có ngắt ưu tiên cao, có thể làm trễ state machine UWB, nên **không bật BLE trên TAG khi bay**. Đo kích thước ảnh BLE + mcumgr so với slot 200 KB. |
| 6.4 | Gia tốc kế LIS2DH12 | Sau GĐ3 | DTS đã có node `lis2dh12` (I2C0). Anchor: phát hiện va chạm/dịch chuyển → bật cờ trong `ANCHOR_INFO`; backend đánh dấu vị trí anchor đó là nghi ngờ. TAG: cờ đứng yên/chuyển động làm gợi ý cho estimator. |
| 6.5 | Multi-PHY | Chỉ khi môi trường cần tầm xa | Profile compile-time; mỗi profile một bộ calibration riêng; hằng số A của FPP theo PRF (113,77 cho PRF16; 121,74 cho PRF64). |
| 6.6 | TDoA3 | Khi cần nhiều hơn 1 drone, hoặc cần tag thụ động | Lấy khái niệm từ `uwb_tdoa_anchor3.c`: phát ngẫu nhiên, bù clock kiểu bucket, dữ liệu anchor lân cận. TAG chỉ nghe và gửi timestamp thô cho UP; solver TDoA chạy ở UP. |
| 6.7 | Nhiều TAG ở chế độ TWR | Khi có 2 drone mà chưa làm TDoA | Địa chỉ TAG cấu hình được; chia slot TDMA theo TAG; anchor chấp nhận một danh sách TAG. |

---

## 8. Đặc tả giao thức đề xuất

### 8.1 Frame UWB v2 cho TWR tuần tự (GĐ1)

Header 802.15.4 giữ nguyên: 9 byte + 1 byte func.

```text
[0-1] FC 0x41 0x88 | [2] MAC seq | [3-4] PAN 0xDECA | [5-6] dst | [7-8] src | [9] func
```

| Frame | func | Payload | Độ dài (không kể FCS) |
|---|---|---|---|
| POLL | 0x21 | [10] ver = 2, [11] txn, [12] flags (bit0 = yêu cầu ANCHOR_INFO) | 13 |
| RESP | 0x10 | [10] ver, [11] txn, [12–15] Da u32, [16] anchor_status, [17] tlv_len, [18…] TLV tuỳ chọn | 18 + TLV |
| FINAL | 0x23 | [10] ver, [11] txn | 12 |
| REPORT | 0x22 | [10] ver, [11] txn, [12–15] Rb u32, [16–17] fp_power của FINAL (cdBm, i16), [18–19] rx_power của FINAL (cdBm, i16) | 20 |
| CFG | 0x30 | [10] ver, [11] txn, [12–13] nonce, [14] tlv_len, TLV | thay đổi |
| CFG_ACK | 0x31 | [10] ver, [11] txn, [12–13] nonce, [14] result | 15 |

- REPORT mang chất lượng tín hiệu đo ở phía anchor, cho backend chỉ báo LOS/NLOS theo cả hai chiều.
- Bên nhận kiểm `ver`. Trong giai đoạn chuyển tiếp, anchor chấp nhận cả v1 (độ dài 12) và v2; TAG luôn gửi v2. Vì vậy **nạp anchor trước, TAG sau**.
- Nâng `MAX_RX_FRAME_LEN` lên ≥ 64 để chứa TLV.
- TLV có dạng `type(1) len(1) value`: `0x01 ANCHOR_POS` (int32 mm × 3), `0x02 ANT_DLY` (u16 × 2), `0x03 BUILD_ID` (u32), `0x04 CFG_CRC` (u32), `0x05 FLAGS`.

### 8.2 DS-TWR một-nhiều (GĐ5)

```text
POLL_B   (dst 0xFFFF): ver, txn, n, id[0..n-1] (u16), D0_uus, S_uus
RESP_k   (dst TAG):    ver, txn, k, Da_k (u32), anchor_status
FINAL_B  (dst 0xFFFF): ver, txn, rx_bitmap, D1_uus
REPORT_k (dst TAG):    ver, txn, Rb_k (u32), fp/rx power của FINAL
```

- TAG tính `Ra_k = T_resp_rx_k − T_poll_tx` và `Db_k = T_final_tx − T_resp_rx_k`. Anchor cung cấp `Da_k` và `Rb_k = T_final_rx_k − T_resp_tx_k`. Công thức bất đối xứng `(Ra·Rb − Da·Db)/(Ra + Rb + Da + Db)` hiện có giữ nguyên.
- Các giá trị 32-bit chứa được khoảng thời gian tới ~67 ms, dư cho chu kỳ ≤ 20 ms.
- Vì `D0`, `S`, `D1` nằm trong frame, có thể tinh chỉnh chúng mà không phải nạp lại anchor.
- Anchor chỉ trả REPORT khi bit của mình trong `rx_bitmap` bằng 1, và anchor không bao giờ nhận FINAL cho `txn` mà nó chưa trả RESP.

### 8.3 Telemetry v2 qua UART (GĐ4)

Khung giữ nguyên: `AA 55 | VER = 1 | TYPE | LEN | SEQ | TIME_ms | payload | CRC16`.

**TYPE 0x10 `RANGE_MEAS`** — một record cho mỗi phép đo:

| Trường | Kiểu | Ghi chú |
|---|---|---|
| schema | u8 | = 1 |
| boot_id | u16 | |
| meas_seq | u32 | tăng mỗi phép đo |
| meas_time_us | u64 | trung điểm của trao đổi, theo clock TAG |
| anchor_id | u16 | |
| txn | u8 | |
| mode | u8 | DS / SS_FALLBACK / DS_1TOM |
| flags | u16 | radio_ok, cal_ok, bias_corr, nlos_hint, loại lỗi RX… |
| raw_mm | i32 | **luôn** là giá trị trước offset (sửa F7) |
| corrected_mm | i32 | sau offset/bias; 0 nếu chưa calibration |
| tag_filtered_mm | i32 | chỉ để chẩn đoán |
| fp_power_cdbm, rx_power_cdbm | i16 × 2 | đo ở TAG (RESP) |
| anchor_fp_cdbm, anchor_rx_cdbm | i16 × 2 | đo ở anchor (FINAL) |
| std_noise, fp_index | u16 × 2 | |
| ci_ppm_x100 | i16 | lệch clock ước lượng |
| slot_us | u16 | |

Payload ≈ 50 B; tính cả header và CRC ≈ 64 B. 8 anchor × 50 Hz ≈ 25,6 kB/s, nên cần UART ≥ 460800 (khuyến nghị 921600).

**Các TYPE khác:** `0x11 DIAG`, `0x12 ANCHOR_INFO`, `0x13 CMD_ACK`, `0x14 SNIFFER_FRAME`, `0x20 CMD` (host → TAG).

---

## 9. Chiến lược kiểm thử

| Lớp | Nội dung | Khi nào |
|---|---|---|
| Host (không cần board) | Unit test state machine TAG/anchor với SPI/timer giả (đã có khung); vector DS/timestamp; fuzz parser (C + Python); test tương thích frame v1/v2 | Mọi GĐ, chạy trước khi nạp |
| Bench HIL tối thiểu | 1 TAG + 2 anchor + 1 DWM1001-DEV làm sniffer + PC. Kịch bản: khoảng cách cố định, bật/tắt anchor, bơm frame nhiễu, rút UART, reset DW1000 | GĐ1 trở đi |
| A/B | Mỗi thay đổi RF/timing chạy cùng vị trí, cùng thời lượng; so counter và phân bố range | GĐ1, GĐ2, GĐ5 |
| Soak | 2 h (GĐ1); 24 h (GĐ4); có motor/Wi-Fi gần (EMI) trước khi bay | Theo cổng |
| Hồi quy | Mỗi lỗi đã sửa (F*, N*) có ít nhất một test tái hiện | Liên tục |

---

## 10. Rủi ro tổng hợp

| Rủi ro | Mức | Giảm thiểu |
|---|---|---|
| Gộp mã nguồn làm lệch hành vi giữa các vai trò | TB | A/B trên board, so `.map` và dump macro; giữ project cũ tới khi đạt parity |
| Thay đổi RF (TX power, XTAL, NTM) làm mất calibration | Cao (chắc chắn xảy ra) | Làm GĐ2 trước mọi calibration chính thức; lưu cấu hình RF trong calibration profile |
| Đổi wire format làm lẫn firmware giữa các node | TB | Byte version, anchor chấp nhận v1/v2 trong giai đoạn chuyển tiếp, manifest nạp |
| Ghi flash làm lỡ deadline radio | Cao nếu ghi khi đang ranging | Chỉ ghi khi `PAUSE_RANGING` |
| Va chạm slot ở DS-TWR một-nhiều | TB | Sniffer, guard rộng ban đầu, D0/S cấu hình được qua frame |
| BLE làm trễ state machine UWB | TB | Không bật BLE trên TAG khi bay; anchor chỉ bật ở chế độ bảo trì |
| Lệnh từ xa đổi cấu hình khi drone đang bay | Cao | Cờ `LOCK` + ranging phải tạm dừng mới được ghi; ACK/NACK; log mọi lệnh |
| Công ước lượng thấp hơn thực tế | TB | Chốt theo cổng nghiệm thu, không theo lịch |

---

## 11. Các quyết định cần chốt

1. **Yêu cầu tần số cuối cùng**: 8 anchor × 50 Hz mỗi anchor, hay chấp nhận 25 Hz mỗi anchor (phương án A)? Câu trả lời quyết định có cần GĐ5-B hay không.
2. **Đường dữ liệu chính trên drone**: CP2102N nối thẳng UP 7000, hay đi qua ESP32-C3? Ảnh hưởng tới kênh lệnh và baud.
3. **Có cần nhiều drone trong 12 tháng tới không?** Ảnh hưởng thứ tự làm TDoA3.
4. **Có dùng BLE** cho commissioning/DFU anchor không?
5. **Có thiết bị đo phổ** và khung đo khoảng cách chuẩn cho GĐ2 không?
6. **Có cho phép chuyển ngay sang một ứng dụng duy nhất** (0.3) không, hay giữ 10 thư mục project thêm một thời gian (0.2)?

---

## Phụ lục A — Truy vết phát hiện → công việc

| Phát hiện | Công việc |
|---|---|
| F1 | 1.1 |
| F2 | 1.5 |
| F3 | 1.2 |
| F4 | 1.3, 8.1 |
| F5 | 1.7 |
| F6 | 3.2, 3.4 |
| F7 | 4.1, 8.3 |
| F8 | 1.1, GĐ5 |
| F9 | 4.2 |
| F10 | 4.6 |
| N1 | 2.4 |
| N2 | 2.2, 2.3 |
| N3 | 1.2 |
| N4 | 2.1 |
| N5 | 1.6 |
| N6 | 1.4 |
| N7 | GĐ5 |
| N8 | 4.7 |
| N9 | 1.5 |
| N10 | 3.4 |
| N11 | 6.1, 6.7 |
| N12 | 0.2, 0.3 |
| N13 | 0.1 |
| N14 | 0.6 |
| N15 | 4.8 |

## Phụ lục B — Vị trí mã tham khảo trong Bitcraze

Repository `bitcraze/lps-node-firmware`, commit `6a85c68`.

| Chủ đề | File |
|---|---|
| Khung chế độ, vòng sự kiện radio | `src/uwb.c`, `inc/uwb.h` |
| TWR anchor / tag | `src/uwb_twr_anchor.c`, `src/uwb_twr_tag.c` |
| TDoA2 | `src/uwb_tdoa_anchor2.c`, `docs/protocols/tdoa2_protocol.md` |
| TDoA3 | `src/uwb_tdoa_anchor3.c`, `docs/functional-areas/tdoa3_implementation.md`, `docs/functional-areas/tdoa2-vs-tdoa3.md` |
| LPP (cấu hình qua sóng) | `src/lpp.c`, `inc/lpp.h`, `docs/protocols/lpp-short-packets-protocol.md` |
| Cấu hình lưu bền | `src/cfg.c`, `inc/cfg.h` |
| Sniffer | `src/uwb_sniffer.c`, `tools/sniffer/*.py` |
| Bù bias, OTP, tuning | `vendor/libdw1000/src/libdw1000.c` (`dwCorrectTimestamp`, `dwTune`) |

**Tài liệu Qorvo nên đọc kèm:**
- DW1000 User Manual: mục cấu hình mặc định cần sửa, bảng TX_POWER, WAIT4RESP / RX timeout, RX soft reset.
- APS011: nguồn sai số trong TWR.
- APS014: hiệu chuẩn antenna delay.
- APS006 Part 3: chỉ số NLOS.
- Datasheet DWM1001C.
