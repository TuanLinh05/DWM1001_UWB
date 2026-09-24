# Báo cáo kiểm tra firmware v2: TAG DevKit + 8 anchor hỗn hợp

- **Ngày:** 2026-09-24
- **Người viết:** Claude (Claude Code), theo yêu cầu của chủ dự án
- **Mã được kiểm:** nhánh `feature/firmware-upgrade-v2`, commit `06f72de`
  ("Nâng cấp firmware v2: mã dùng chung, giao thức v2, anchor STM32 và kế hoạch").
- **Nhánh làm việc:** `claude/friendly-mendel-0ay4oi`. Nhánh này chỉ thêm vào
  `06f72de` hai loại file:
  - test: `Firmware/tests/test_tag_motion.c`, `Firmware/tests/motion_cfg/`, và 3 mục mới trong `run_host_tests.ps1`;
  - tài liệu: báo cáo này và `Plan/patches/0001-range-filter-nlos-aware-reacquire.patch`.

  **Không sửa một dòng firmware nào chạy trên board.** Mọi đề xuất sửa firmware
  đều nằm trong báo cáo (dạng sườn code) hoặc trong bản vá (áp khi đã duyệt).
- **Trạng thái:** mới kiểm trên PC, gồm:
  - đọc mã;
  - host test, sanitizer, phân tích tĩnh;
  - mô phỏng kênh vô tuyến chạy qua máy trạng thái TAG thật;
  - build Zephyr thật cho 9 image.

  Chưa có log phần cứng. Mọi con số từ mô phỏng đều gắn nhãn `[MÔ HÌNH]` và
  cần xác nhận bằng các thí nghiệm ở mục 10.
- **Đọc cùng:**
  - `Plan/RA_SOAT_TAG_DEVKIT_8_ANCHOR_HON_HOP_2026-09-23.md`: bản rà soát 23/9. Mục 4 của báo cáo này đối chiếu từng kết luận của bản đó.
  - `Plan/DANH_GIA_SO_SANH_BITCRAZE_VA_HUONG_GIAM_SAI_SO_2026-09-24.md`: hướng giảm sai số. Báo cáo này dùng lại các mã P1–P11, H1–H7, S1–S4, T1–T8 của tài liệu đó.
  - `Firmware/HARDWARE_AB_CHECKLIST.md`.

> **Cập nhật cùng ngày, sau khi viết báo cáo.** Theo đồng ý của chủ dự án, hai
> việc sau đã được làm trên nhánh `claude/friendly-mendel-0ay4oi`:
>
> - áp bản vá §8.1 (tái bắt khoá có nhận biết NLOS);
> - chuyển **TAG DevKit** sang `MEDIAN_GATE` (§8.2).
>
> Mọi chỗ trong báo cáo ghi "chưa áp", "mặc định LEGACY" hay "known issue" là mô
> tả trạng thái của `06f72de`, lúc viết báo cáo. Các mục §8.3–§8.14 **chưa làm**.
> Project `Tag` (PCB riêng) vẫn dùng LEGACY. Chi tiết thay đổi nằm ở mục 0 của
> `Firmware/CHANGELOG.md`.

---

## Mục lục

0. [Cách đọc báo cáo](#0-cách-đọc-báo-cáo)
1. [Tóm tắt](#1-tóm-tắt)
2. [Phạm vi, phương pháp, giới hạn](#2-phạm-vi-phương-pháp-giới-hạn)
3. [Kiến trúc firmware v2 (để đối chiếu)](#3-kiến-trúc-firmware-v2-để-đối-chiếu)
4. [Đối chiếu bản rà soát 23/9](#4-đối-chiếu-bản-rà-soát-239)
5. [Kiểm chứng trên PC: build, host test, sanitizer, phân tích tĩnh](#5-kiểm-chứng-trên-pc-build-host-test-sanitizer-phân-tích-tĩnh)
6. [Test chuyển động](#6-test-chuyển-động)
7. [Thời gian slot, ngân sách anchor, SPI](#7-thời-gian-slot-ngân-sách-anchor-spi)
8. [Những điểm cần sửa: sườn code và hướng làm](#8-những-điểm-cần-sửa-sườn-code-và-hướng-làm)
9. [Lộ trình và tiêu chí nghiệm thu](#9-lộ-trình-và-tiêu-chí-nghiệm-thu)
10. [Thí nghiệm phần cứng cần làm](#10-thí-nghiệm-phần-cứng-cần-làm)
11. [Rủi ro còn lại và câu hỏi mở](#11-rủi-ro-còn-lại-và-câu-hỏi-mở)
- [Phụ lục A. Lệnh tái lập](#phụ-lục-a-lệnh-tái-lập)
- [Phụ lục B. Bảng kết quả đầy đủ](#phụ-lục-b-bảng-kết-quả-đầy-đủ)
- [Phụ lục C. File trong nhánh này](#phụ-lục-c-file-trong-nhánh-này)

---

## 0. Cách đọc báo cáo

| Nhãn | Ý nghĩa | Cách kiểm lại |
|---|---|---|
| `[CODE]` | Đọc trực tiếp trong mã. Có `file:dòng`, số dòng theo commit `06f72de` | Mở đúng file:dòng |
| `[TEST]` | Kết quả chạy test trong phiên này | Lệnh ở Phụ lục A |
| `[MÔ HÌNH]` | Kết quả của `test_tag_motion.c`. Chỉ đúng trong các giả định ở mục 6.2 | Chạy lại test; xác nhận bằng log phần cứng |
| `[BUILD]` | Build Zephyr thật: NCS v3.4.0, Zephyr SDK 1.0.1 (GCC 14.3.0) | Lệnh ở Phụ lục A |
| `[NGOÀI]` | Kiến thức từ tài liệu Decawave/Qorvo. Trong phiên này không mở tài liệu gốc | Đối chiếu DW1000 User Manual, APS006/011/013/014 |
| `[GIẢ THUYẾT]` | Nhận định kỹ thuật cần thí nghiệm để xác nhận | Mục 10 |

Mức độ nghiêm trọng:

- **Cao:** ảnh hưởng trực tiếp tới dữ liệu đưa vào điều khiển bay.
- **Trung bình:** ảnh hưởng sai số, khả năng chẩn đoán hoặc truy vết.
- **Thấp:** tiện dụng, test, dọn mã.

Đường dẫn trong báo cáo tính từ gốc repo.

---

## 1. Tóm tắt

### 1.1 Trả lời ba câu hỏi

**1. Đã đọc branch v2 chưa?**

Rồi. Đã đọc toàn bộ `feature/firmware-upgrade-v2` @ `06f72de`, gồm:

- `Firmware/common`: driver DW1000, ranging TAG/anchor, frame v2, bộ lọc, telemetry, lệnh, settings, platform Zephyr;
- 4 project Zephyr mới cho STM32 (`STM32_Anchor_5..8`);
- cấu hình của TAG DevKit và các anchor;
- gateway ESP32-C3, GUI và host filter;
- công cụ `verify_stm32_images.py`, `CHANGELOG`, `DEPLOYMENT_MANIFEST`;
- các file kế hoạch trong `Plan/`.

**2. Đã test chuyển động chưa?**

Rồi. Test nằm ở `Firmware/tests/test_tag_motion.c`. Test chạy **đúng mã TAG
đang nạp trên board** (`tag_ranging.c` gồm lịch anchor, toán DS-TWR, calibration,
bộ lọc và hàng đợi đo) trên bộ mô phỏng DW1000 mức thanh ghi. Một mô hình
kênh vô tuyến đóng vai 8 anchor. Mô hình gồm:

- quỹ đạo thật;
- hai đồng hồ 40-bit trôi ±20 ppm, có vượt qua điểm wrap;
- airtime đúng PHY đang dùng;
- đúng luật reply của anchor, kể cả delayed TX bị trễ;
- nhiễu timestamp;
- bias riêng từng cặp TAG-anchor;
- NLOS và anchor mất tín hiệu.

Có 14 kịch bản, 3 bộ lọc, quét 20 seed ngẫu nhiên. Test đã được đưa vào
`run_host_tests.ps1`.

**3. Kết luận chính**

- **Toán DS-TWR, calibration và wrap 40-bit: đúng** `[TEST][MÔ HÌNH]`.
  - Bias của range thô không quá 5,4 mm trên 20 seed × 14 kịch bản.
  - σ ≈ 30 mm, bằng đúng nhiễu đưa vào mô hình.
  - Qua wrap 40-bit và lệch ±20 ppm vẫn không sai.
- **Bộ lọc mặc định LEGACY không dùng được khi bay** `[MÔ HÌNH]`.
  - Ở 0,5–2 m/s chỉ 13–19 % phép đo được đánh dấu hợp lệ.
  - Trễ 0,75–2,9 s.
  - Trong bài bay 8 anchor có 1 351 range sai hơn 30 cm vẫn mang cờ hợp lệ.
- **Hai bộ lọc C9 theo được chuyển động nhưng có thể khoá vào mức NLOS khi tái bắt khoá.**
  - `MEDIAN_GATE` và `CV_KALMAN_V2` hợp lệ 100 %, trễ khoảng 20–30 ms.
  - Lỗi khoá NLOS: 5/20 seed ở kịch bản `reacq_nlos`, 4/20 seed ở `spikes_*`.
  - Với bản vá đi kèm, `MEDIAN_GATE` đạt **14/14 kịch bản trên cả 20 seed, không còn FA nào**. `CV_KALMAN_V2` cũng hết FA, và đạt 14/14 khi bỏ độ bù FPP (V05). Bản vá có unit test riêng và build Zephyr đạt 0 cảnh báo.
- **8 anchor DS-TWR không vừa chu kỳ 20 ms** `[MÔ HÌNH]`: 26 ms mỗi chu kỳ (39 Hz), gần như mọi chu kỳ đều overrun.
- **Các việc còn lại:**
  - calibration chưa gắn với danh tính anchor;
  - không quan sát được sức khỏe anchor STM32;
  - TX power LEGACY cao hơn mức tham chiếu 16 dB;
  - truy vết bản phát hành: ảnh dist được build từ cây mã chưa commit, SHA-256 trong manifest tính trên file CRLF;
  - UART 115200 chặn `RANGE_MEAS`.

### 1.2 Bảng phát hiện

| ID | Mức | Phát hiện | Bằng chứng | Sửa ở |
|---|---|---|---|---|
| **V01** | **Cao** | Bộ lọc mặc định `LEGACY` (median-3 + Kalman tĩnh) không bám được chuyển động. Ở 0,5 / 1 / 2 m/s chỉ 18,8 / 15,4 / 13,5 % mẫu hợp lệ, trễ 2 146 / 1 283 / 754 ms. Bài bay 8 anchor: 36 % hợp lệ, 1 351 range sai hơn 0,3 m vẫn mang cờ hợp lệ. Nguyên nhân: Kalman tĩnh với Q = 0,05 có hệ số K ổn định chỉ 0,002–0,031 (lại khởi tạo P = 1 mm², nên ngay từ đầu đã gần như không nhúc nhích); gate lại so mẫu mới với chính trạng thái đang trễ đó | `[MÔ HÌNH]` §6.4; `[CODE]` `tag_ranging.c:1589-1617`, `:672-704`, `:626-658` | §8.2 |
| **V02** | **Cao** | Hai bộ lọc C9 tái bắt khoá theo cách có thể **khoá vào mức NLOS**. Sau khi mất tín hiệu, mẫu đầu tiên được phát luôn như sự thật. Khi đang bám, 3 mẫu bị loại mà gần nhau là đủ để tái bắt, kể cả khi cả 3 đều NLOS. Mỗi mẫu được chấp nhận lại xoá sạch ứng viên. Kết quả trên 20 seed: `reacq_nlos` lỗi 5/20 (10 FA), `spikes_walk` 3/20, `spikes_hold` 1/20 | `[MÔ HÌNH]` §6.5; `[CODE]` `range_filter.c:338-360`, `:179-207`, `:477` | §8.1 (bản vá đã kiểm chứng) |
| **V03** | **Cao** | 8 anchor DS-TWR tuần tự mất 26 ms (39 Hz). Mục tiêu `TAG_CYCLE_MS` = 20 ms. Bài bay 24 s có 933 lần overrun, tức gần như mọi chu kỳ | `[MÔ HÌNH]` §7; `[CODE]` `tag_ranging.h:37` | §8.6 |
| V04 | Trung bình | Độ trễ CPU của anchor từ ngắt POLL tới lệnh TXDLYS chưa được đo, dù đây là **vách đứng**: với timing hiện tại, quá khoảng 917 µs thì cả A5–A8 đều trả RESP muộn và độ sẵn có tụt về 50 %. Anchor chỉ đếm `delayed_tx_late`; bộ đếm này không gửi về TAG, và STM32 không có UART/RTT | `[MÔ HÌNH]` §7.3; `[CODE]` `anchor_ranging.h:85-99`, `STM32_Anchor_5/prj.conf` | §8.5 |
| V05 | Trung bình | `CV_KALMAN_V2` dùng phương sai đo R sai lệch vì độ bù FPP −12,04 dB đẩy mọi mẫu vào lớp "yếu" (R = 40 000 mm², tức σ = 200 mm, trong khi σ thật khoảng 30 mm). Hậu quả: sau NLOS bộ lọc lố khoảng −120 mm trong 300 ms; `nlos_3s` lỗi p95 ở 2/20 seed. Đặt độ bù về 0 thì hết | `[MÔ HÌNH]` §6.8, §6.9; `[CODE]` `uwb_calibration.h:387-396`, `range_filter.c:24-31` | §8.3 |
| V06 | Trung bình | TX power mặc định `LEGACY` = `0x1E1E1E1E` (gain 30 dB), cao hơn giá trị tham chiếu `0x48484848` (14 dB) 16 dB. Có nguy cơ vượt mặt nạ phát xạ và bão hòa bộ thu ở cự ly gần. Reference tuning (LDE NTM = 13, XTAL trim) đang tắt | `[CODE]` `dw1000.c:50-52`, `uwb_calibration.h:341`, `:357`; `[NGOÀI]` | §8.9 |
| V07 | Trung bình | Calibration chỉ gắn với PARTID/profile **của TAG**. Không gắn với danh tính anchor, và anchor cũng không gửi PARTID của mình. Thay board anchor cùng ID thì bias sai mà không có cảnh báo | `[CODE]` `uwb_settings.c:228-231`, `uwb_frame.h:74-81` | §8.4 |
| V08 | Trung bình | Hai anchor có thể trùng ID: project nRF `Anchor_5..8` vẫn còn và có cùng ID 5–8 với `STM32_Anchor_5..8`. Lúc chạy không có cách nào phát hiện hai anchor trả lời cùng một ID | `[CODE]` `Firmware/Anchor_5..8`, `tools/verify_stm32_images.py` | §8.7 |
| V09 | Trung bình | Ảnh trong `anchor_8_dist/` nhúng git hash `0x08d28188` với cờ dirty = 1. Tức là build từ cây mã chưa commit, trước khi có `06f72de`. Mã chức năng thì tương đương `06f72de`. SHA-256 trong `manifest.csv` tính trên file **CRLF**, còn git lưu **LF**, nên trên máy Linux hoặc checkout `autocrlf=false` hash sẽ không khớp | `[BUILD]` §5.4 | §8.8 |
| V10 | Thấp | TAG DevKit và gateway đều chạy 115200 baud. `RANGE_MEAS`, đầu vào mà estimator trên host cần, đòi ít nhất 460800 nên đang bị tắt | `[CODE]` `Tag_DevKit/app.overlay:26`, `telemetry.h:70`, `gateway_config.h:11` | §8.10 |
| V11 | Thấp | Nhánh C9.1 (Adaptive Legacy) và C9.2 (motion controller) chỉ đạt 3/14 và 4/14 kịch bản. Bật C9.2 ACTIVE thì build `-Werror` thất bại vì `apply_outlier_gate` và `s_kf` không dùng | `[MÔ HÌNH]` §6.6; `[TEST]` `tag_ranging.c:507`, `:662` | §8.11 |
| V12 | Thấp | Lỗ hổng test: trước nhánh này, bộ lọc C9 không có test C nào (`test_tag_state` chỉ build được ở chế độ Legacy vì dùng `s_kf`). `verify_stm32_images.py` không nằm trong runner. 1 test GUI chỉ chạy được trên Windows, và các test GUI cần màn hình | `[TEST]` §5.2 | §8.12 |
| V13 | Thấp | Đường RESP→FINAL của TAG tốn 12 giao dịch SPI. Có thể dời phần đọc chẩn đoán sang lúc chờ REPORT để rút ngắn slot | `[MÔ HÌNH]` §7.4 | §8.6 |

### 1.3 Những gì v2 làm đúng (đã kiểm chứng)

- **Mã dùng chung** (`Firmware/common` + `uwb_node.cmake`). 11 project nRF và 4 project STM32 build từ cùng nguồn. Không còn nguy cơ các bản sao lệch nhau. `[CODE]`
- **Giao thức v2 có version và txn.**
  - Parser kiểm độ dài theo từng loại frame. `[CODE]` `uwb_frame.c:127-144`
  - RESP sai version bị loại là BADFRAME; RESP của transaction cũ bị bỏ qua. `[CODE]` `tag_ranging.c:2143-2160`
- **Anchor tính Da từ RMARKER thật:** `DX_TIME` giữ bit [39:9] rồi cộng TX antenna delay. `[CODE]` `anchor_ranging.c:216-219`. `[MÔ HÌNH]` xác nhận không có bias.
- **Delayed TX trễ được phát hiện ngay** qua HPDWARN hoặc TXPUTE. Driver đọc SYS_STATUS đủ 40 bit. `[CODE]` `dw1000.c:576-591`
- **Frame dài hơn buffer bị từ chối, không bị cắt.** `[CODE]` `dw1000.c:638-654`
- **Calibration fail-closed:**
  - `UWB_DS_CALIBRATED_MASK` = 0;
  - offset làm khoảng cách ≤ 0 trả về compute error; `[CODE]` `tag_ranging.c:1475-1500`
  - đổi calibration thì snapshot bị invalid ngay và hàng đợi đo bị xoá.
- **Recovery radio xoá hàng đợi và reset bộ lọc.** `[CODE]` `tag_ranging.c:2386-2410`
- **Legacy gate đã sửa:** mẫu bị loại không còn cập nhật Kalman và không còn được phát là hợp lệ. `[CODE]` `tag_ranging.c:1599-1614`
- **Toán DS-TWR bất đối xứng đúng** qua wrap 40-bit, lệch ±20 ppm và calibration từng cặp. `[MÔ HÌNH]` §6.4, cột `raw_b/sd`.
- **Sạch lỗi bộ nhớ và cảnh báo:**
  - ASan + UBSan: không lỗi;
  - `gcc -fanalyzer`: 0 cảnh báo;
  - clang-tidy (`clang-analyzer-*`, `bugprone-*`): không có lỗi thật;
  - 9 image Zephyr build 0 cảnh báo. `[TEST][BUILD]` §5
- **Ảnh STM32 vừa bộ nhớ:** 25,7 KB / 64 KB flash, 7,5 KB / 20 KB RAM. `[BUILD]`

### 1.4 Năm việc nên làm ngay

1. **Áp bản vá tái bắt khoá** (§8.1), sau đó **chuyển TAG sang `MEDIAN_GATE`** (§8.2). Cả hai làm được trên PC ngay hôm nay, và test đã có sẵn.
2. **Đo biên thời gian delayed TX của cả 8 anchor** bằng TLV sức khỏe (§8.5) trước khi đụng tới timing (§8.6).
3. **TLV danh tính anchor**, gắn calibration vào danh tính đó, và chặn trùng ID (§8.4, §8.7).
4. **Build lại ảnh phát hành từ một commit sạch**, thêm `.gitattributes` (§8.8).
5. **A/B TX power và reference tuning**, sau đó mới calibration (§8.9, T6, H3).

---

## 2. Phạm vi, phương pháp, giới hạn

### 2.1 Mã đã đọc

| Nhóm | File |
|---|---|
| Ranging | `common/src/ranging/tag_ranging.c` (2 868 dòng), `anchor_ranging.c`, `uwb_frame.c`, `common/include/uwb_frame.h`, `tag_ranging.h`, `anchor_ranging.h` |
| Driver | `common/src/drivers/dw1000.c`, `common/include/dw1000_hw.h`, `uwb_calibration.h` |
| Bộ lọc | `common/src/filters/range_filter.c`, `range_filter.h`, `range_filter_config.h`, `legacy_adaptive_tracking.h`, `motion_adaptive_range.h` |
| Ứng dụng | `main_tag.c`, `main_anchor.c`, `main_sniffer.c`, `uwb_cmd.c`, `uwb_settings.c`, `uwb_health.c` |
| Telemetry | `telemetry.c`, `telemetry_frame.c`, `uart_tx_zephyr.c` |
| Platform | `uwb_platform_zephyr.c`, `common/cmake/uwb_node.cmake` |
| Node | `Tag_DevKit`, `Anchor_1..8`, `STM32_Anchor_5..8` (`prj.conf`, `app.overlay`, `uwb_app_config.h`) |
| Khác | `ESP32C3_Gateway`, `Software/UWB_UART_GUI` (`telemetry_protocol.py`, `host_range_filter.py`, `uwb_uart_gui.py`), `tools/verify_stm32_images.py`, `scripts/*.ps1`, `CHANGELOG.md`, `DEPLOYMENT_MANIFEST.md`, `anchor_8_dist/` |

### 2.2 Công cụ

| Việc | Công cụ |
|---|---|
| Host test | gcc 13.3 (`-std=c11 -Wall -Wextra -Werror -Wshadow`), PowerShell 7.4.6 chạy `run_host_tests.ps1` |
| Sanitizer | gcc `-fsanitize=address,undefined -fno-sanitize-recover=all` |
| Phân tích tĩnh | `gcc -fanalyzer`, clang-tidy 18 (`clang-analyzer-*`, `bugprone-*`) |
| Build firmware | NCS v3.4.0 (sdk-zephyr `bf801e4e3d19`), Zephyr SDK 1.0.1 (GCC 14.3.0), hal_stm32 `39130f29` |
| Board | `decawave_dwm1001_dev/nrf52832` (TAG, A1–A4), `stm32_min_dev@blue/stm32f103xb` (A5–A8) |
| Mô phỏng | `test_tag_motion.c` + `tests/dw1000_sim.c` (1 chu kỳ CPU mô phỏng = 1 µs) |

### 2.3 Giới hạn

- **Không có phần cứng.** Thời gian CPU của anchor và TAG là giả định:
  - anchor nRF: 350 µs từ ngắt POLL tới lệnh TXDLYS;
  - anchor STM32: 500 µs cho cùng đoạn đó;
  - REPORT: 300 µs (nRF), 450 µs (STM32);
  - TAG: 350 µs từ RESP tới FINAL.

  §7 quét độ nhạy của kết quả theo các giá trị này.
- **Không có mã CubeIDE STM32** (`D:\Drone Project\UWB Drone\STM32_UWB`) trong repo. Các nhận định 23/9 về mã đó không kiểm lại được.
- **NLOS được mô hình hóa đơn giản:** cộng +0,6 hoặc +0,8 m vào đường truyền và giảm FPP 8 dB. Mô hình không tính CIR/LDE, bias theo mức RX, hay anten theo góc tới (P2, P5).
- **Mức FPP ở 1 m là tham số** (−62 dBm, kịch bản bay −58 dBm). §6.8 quét tới −90 dBm.
- **Build Zephyr bỏ module `nrf` của NCS**, vì môi trường thiếu nrfxlib/openthread. Image thiếu banner NCS nên nhỏ hơn ảnh dist 244–288 B. Mã ứng dụng không bị ảnh hưởng (§5.4).

---

## 3. Kiến trúc firmware v2 (để đối chiếu)

### 3.1 Các nút

| Nút | Phần cứng | Project | Flash (text+data) | RAM (data+bss) |
|---|---|---|---:|---:|
| TAG | DWM1001-DEV (nRF52832, 512 KB / 64 KB) | `Tag_DevKit` | 57,6 KB | 13,1 KB |
| A1–A4 | DWM1001C (nRF52832) | `Anchor_1..4` | 33,2 KB | 7,8 KB |
| A5–A8 | STM32F103C8 + DW1000 (64 KB / 20 KB) | `STM32_Anchor_5..8` | 25,7 KB | 7,5 KB |
| Sniffer | DWM1001-DEV | `Sniffer_DevKit` | — | — |
| Gateway | ESP32-C3 | `ESP32C3_Gateway` | — | — |

Số liệu `[BUILD]` từ `arm-zephyr-eabi-size`, build trong phiên này (KB = 1 000 B).

### 3.2 Một slot DS-TWR v2

```
TAG (đồng hồ T)                                  ANCHOR k (đồng hồ A)
  t1 |--- POLL  v2  15 B (ver, txn, flags) ------->| a2
     |                                             | DX  = (a2 + 1200 UUS) & 0xFFFFFFFE00
     |                                             | a3  = DX + TX_ANTD     (RMARKER thật)
  t4 |<-- RESP  v2 ≥20 B (ver, txn, Da, st, TLV) --| a3      Da = a3 - a2
  t5 |--- FINAL v2  14 B (ver, txn) -------------->| a6      Rb = a6 - a3
     |<-- REPORT v2 22 B (ver, txn, Rb, FP, RX) ---|
Ra = t4 - t1 ; Db = t5 - t4
ToF = (Ra·Rb − Da·Db) / (Ra + Rb + Da + Db)          mọi hiệu đều modulo 2^40
```

- Độ dài frame (đã tính 2 byte FCS): `uwb_frame.h:54-61` `[CODE]`.
- Airtime ở preamble 256, 6,8 Mbps `[MÔ HÌNH]`:
  - SHR (preamble + SFD) = 262,3 µs;
  - POLL / RESP / FINAL / REPORT = 303,3 / 308,5 / 302,3 / 310,5 µs.

### 3.3 Đường xử lý một khoảng cách trên TAG

```
timestamp -> ToF (DS) -> raw_mm
          -> corrected_mm = raw - bias(TAG, anchor)          [apply_offset_and_clamp]
          -> conditioner (theo UWB_RANGE_FILTER_MODE)        [publish_distance]
                 FPP đưa vào bộ lọc = FPP thật + UWB_FILTER_FPP_COMPAT_DB (−12,04 dB)
          -> snapshot RANGE (valid, filtered, age) + RANGE_MEAS (raw, corrected,
             filtered, flags, FP/RX, chẩn đoán)              [telemetry]
```

Chỗ cộng độ bù FPP: `tag_ranging.c:1552-1556` `[CODE]`. Telemetry phát FPP đã
sửa. Chỉ bộ lọc thấy thang FPP cũ.

### 3.4 Các chế độ bộ lọc

| Chế độ | Cờ | Mô tả | Trạng thái trong v2 |
|---|---|---|---|
| LEGACY | `UWB_RANGE_FILTER_MODE=0` (mặc định) | Median-3, gate động học (Vmax = 10 m/s, +100/−250 mm, snap sau 15 lần), Kalman tĩnh Q = 0,05, R = 50/200/1 000/10 000 theo FPP | Đang nạp. Comment: "radio noise needs its smoothing" (`uwb_calibration.h:153-156`) |
| MEDIAN_GATE | `=1` | Median-3, gate vật lý/động học, tái bắt khoá có kiểm soát, không Kalman | "candidate production" |
| CV_KALMAN_V2 | `=2` | Kalman vận tốc không đổi 1D, gate NIS 6,635, R theo 3 lớp FPP | "thử nghiệm" |
| C9.1 Adaptive Legacy | `UWB_LEGACY_ADAPTIVE_MODE=1/2` | Legacy có Q/R thích nghi | TAG DevKit ép OFF (`uwb_app_config.h:36`) |
| C9.2 Motion | `UWB_C9_2_MOTION_MODE=1/2` | Bộ điều khiển trạng thái chuyển động | OFF |

---

## 4. Đối chiếu bản rà soát 23/9

| Mục 23/9 | Kết luận 23/9 | Tình trạng ở v2 `06f72de` | Đánh giá | Bằng chứng |
|---|---|---|---|---|
| P0 (1) | TAG phát v2, STM32 CubeIDE trả v1. TAG đòi RESP cùng version/txn | RESP sai version bị loại là `TAG_ST_BADFRAME` | **Đúng về cơ chế.** Hết vấn đề nếu nạp `STM32_Anchor_5..8`: bản Zephyr dùng chung responder v2. Mã CubeIDE không có trong repo nên không kiểm lại được | `[CODE]` `tag_ranging.c:2143-2152`, `uwb_node.cmake` (role ANCHOR) |
| P0 (2) | STM32 A1–A4 trùng ID với DWM1001C | `STM32_Anchor_5..8` dùng ID 5–8, `verify_stm32_images.py` kiểm lại | **Đã sửa.** Nhưng xuất hiện rủi ro mới: project nRF `Anchor_5..8` cùng ID 5–8 (V08) | `[CODE]` `tools/verify_stm32_images.py` (`ANCHOR_IDS = range(5, 9)`) |
| P1 (1) | Legacy gate ép FPP = −100 nhưng vẫn phát `valid=1` | Mẫu bị loại: trả median về trạng thái cũ, đánh dấu reject, không cập nhật Kalman | **Đã sửa** | `[CODE]` `tag_ranging.c:1599-1614` |
| P1 (2) | Recovery chưa xoá filter/queue | `Tag_RecoverRadio` xoá queue, reset filter, invalid track | **Đã sửa** | `[CODE]` `tag_ranging.c:2386-2410` |
| P1 (3) | Q = 0,05, R = 50…10 000, K ≈ 0,0022: "dấu hiệu trễ động lớn, *chưa xác nhận*" | Test chuyển động đo được | **Đúng, nay đã định lượng.** 13–19 % hợp lệ ở 0,5–2 m/s, trễ 0,75–2,9 s. Hệ số K ổn định: 0,031 / 0,016 / 0,007 / 0,0022 | `[MÔ HÌNH]` §6.4, §6.9 |
| P1 (4) | STM32 dùng antenna delay HW = 0 và legacy offset = 1, khác TAG | STM32 Zephyr dùng chung `uwb_calibration.h`; `verify_stm32_images.py` kiểm các cờ PHY khớp nhau | **Đã giải quyết ở mức mã.** Calibration phần cứng vẫn phải làm (H3) | `[CODE]` `verify_stm32_images.py` (`PHY_DEFINES`) |
| P1 (5) | STM32 cắt frame dài; đọc SYS_STATUS 4 byte nên không thấy TXPUTE | Driver chung từ chối frame dài hơn buffer; đọc đủ 5 byte; kiểm cả HPDWARN lẫn TXPUTE | **Đã giải quyết** khi dùng image Zephyr | `[CODE]` `dw1000.c:638-654`, `:576-591` |
| P2 (1) | Đổi/huỷ DS calibration không invalid snapshot ngay; offset âm bị kẹp về 0 mà vẫn valid | Invalid ngay, xoá queue; khoảng cách ≤ 0 trả compute error | **Đã sửa** (có test trong `test_tag_state`) | `[CODE]` `tag_ranging.c:1475-1500`, `:2427-2470` |
| P2 (2) | Settings chỉ bind PARTID/profile của TAG | Vẫn như cũ | **Còn tồn tại** (V07) | `[CODE]` `uwb_settings.c:228-231` |
| P2 (3) | UART 115200; `RANGE_MEAS` cần ≥ 460800 | Vẫn như cũ | **Còn tồn tại** (V10) | `[CODE]` `Tag_DevKit/app.overlay:26`, `telemetry.h:70`, `telemetry.c:63-69`, `gateway_config.h:11` |
| 4.3 | Độ bù FPP −12,04 dB có thể chặn reacquire C9 (−97 dBm thành −109 dBm, dưới ngưỡng −105) | Cơ chế đúng. Ở mã hiện tại, tái bắt khoá vẫn xảy ra, qua đường stale reset (chậm hơn) | **Đúng.** Chính bản vá đầu tiên của báo cáo này đã dính lỗi đó (không bao giờ khởi tạo, 0 % hợp lệ). Lỗi đã sửa, và đã thêm kịch bản `weak_gap` để bắt nó | `[MÔ HÌNH]` §6.8 |
| 3.4 | 8 DS-TWR có thể vượt 20 ms; cân nhắc lịch 4+4 | 26 ms/chu kỳ, 39 Hz | **Đúng** | `[MÔ HÌNH]` §7 |
| 4.1 | Kích thước frame v2 và công thức Da | Khớp | **Đúng** | `[CODE]` `uwb_frame.h:54-61`, `anchor_ranging.c:216-219` |

---

## 5. Kiểm chứng trên PC: build, host test, sanitizer, phân tích tĩnh

### 5.1 Build Zephyr `[BUILD]`

| Target | Board | Kết quả | text / data / bss (B) |
|---|---|---|---|
| `Tag_DevKit` | `decawave_dwm1001_dev/nrf52832` | 0 cảnh báo | 57 200 / 436 / 12 692 |
| `Anchor_1..4` | `decawave_dwm1001_dev/nrf52832` | 0 cảnh báo | 32 824–32 828 / 396 / 7 453 |
| `STM32_Anchor_5..8` | `stm32_min_dev@blue/stm32f103xb` | 0 cảnh báo | 25 488 / 204 / 7 299 |
| `Tag_DevKit` + bản vá §8.1, `MEDIAN_GATE` | như trên | 0 cảnh báo | 58 620 / 436 / 13 268 |
| `Tag_DevKit` + bản vá §8.1, `CV_KALMAN_V2` | như trên | 0 cảnh báo | 59 632 / 436 / 13 268 |

Cờ biên dịch của node: `-Wall -Wextra -Wformat=2 -Wshadow`, không có `-Werror`
(`uwb_node.cmake`). Vì vậy lỗi C9.2 ở V11 chỉ hiện thành cảnh báo khi build
firmware, nhưng làm host test thất bại.

### 5.2 Host test `[TEST]`

`run_host_tests.ps1` (PowerShell 7.4.6, gcc 13.3):

| Bước | Kết quả |
|---|---|
| `test_driver`, `test_uwb_frame`, `test_tag_state`, `test_anchor_state`, `test_cmd_parser`, `test_cmd_executor`, `test_settings`, `test_telemetry_golden` | Đạt |
| `test_tag_motion` (Legacy, `--report`) | Chạy xong. 3/14 kịch bản trong ngưỡng. Chỉ ghi kết quả, không làm hỏng runner |
| `test_tag_motion_median_gate`, `test_tag_motion_cv_kalman` (strict) | Đạt: 13/14, còn 1 known issue (`reacq_nlos`) |
| `test_telemetry_golden.py`, `test_node_projects.py`, `test_sniffer_tool.py` | Đạt |
| `gui_python_tests` | 49/50 dưới `xvfb-run`. Test còn lại, `test_frozen_application_directory_is_executable_parent`, dùng đường dẫn Windows `D:/Demo/...` nên chỉ chạy được trên Windows. Không có màn hình thì thêm 10 lỗi Tk |
| `gateway_parser` | Đạt |

Khi áp bản vá §8.1 thì runner có **16/16 bước đạt**, gồm 2 bước unit test mới.
Hai biến thể motion strict đạt **14/14**.

### 5.3 Sanitizer và phân tích tĩnh `[TEST]`

- **ASan + UBSan**, `-fno-sanitize-recover=all`:
  - chạy cả 9 binary host test, cùng test chuyển động ở 3 chế độ;
  - chạy bản vá ở 2 chế độ, mỗi chế độ 3 seed;
  - **không có lỗi runtime nào**.
- **`gcc -fanalyzer`**: **0 cảnh báo** trên các file sau.
  - `tag_ranging.c`, `anchor_ranging.c`, `dw1000.c`, `uwb_frame.c`;
  - `range_filter.c` ở mode 1, mode 2, và bản đã vá;
  - `uwb_cmd.c`, `telemetry.c`, `telemetry_frame.c`, `uwb_settings.c`.
- **clang-tidy 18** (`clang-analyzer-*`, `bugprone-*`): 5 ghi chú, **không có lỗi thật**.
  - 4 dead store trong phép hoán vị của median-3: `range_filter.c:64-65`, `tag_ranging.c:609-610`. Vô hại.
  - 1 `bugprone-branch-clone` ở `anchor_ranging.c:460`: hai nhánh xử lý hai trạng thái khác nhau, cố ý như vậy.

### 5.4 Truy vết ảnh phát hành `[BUILD]`

Đối chiếu `anchor_8_dist/*.hex` với image tự build lại từ `06f72de`:

| Kiểm | Kết quả |
|---|---|
| Git hash nhúng trong ảnh dist | `0x08d28188` (commit baseline `08d2818`), dirty = 1. Tức là ảnh build từ cây mã chưa commit |
| Git hash nhúng trong image build lại | `0x06f72de8` |
| Chênh lệch nội dung | Chỉ khác ở banner NCS (244–288 B) và hằng số hash/dirty. Phần ứng dụng tương đương |
| SHA-256 trong `manifest.csv` | **Khớp bản CRLF** của file. Ví dụ A5: CRLF = `29b1fe52…` (khớp manifest); LF = `ddcfbf02…`, đúng bằng giá trị `verify_stm32_images.py` in ra. Cột `Bytes` = 73 087 cũng là kích thước CRLF; git lưu LF, 71 456 B |
| Lý do | Repo không có `.gitattributes`. Máy Windows để `autocrlf` đổi LF thành CRLF khi checkout. `package_8_anchors.ps1` gọi `Get-FileHash` trên bản CRLF đó |

---

## 6. Test chuyển động

### 6.1 Mục tiêu

Trả lời bằng số một câu hỏi: **phạm vi đo mà TAG phát ra khi drone đang bay
có dùng được không?** Cụ thể: bao nhiêu phần trăm mẫu hợp lệ, sai số, độ trễ,
số range sai mà vẫn được đánh dấu hợp lệ (gọi là *false accept*, FA), và thời
gian hồi phục. Tất cả đi qua **đúng mã TAG thật**, không viết lại thuật toán.

### 6.2 Mô hình `[MÔ HÌNH]`

| Thành phần | Mô hình | Nguồn tham số |
|---|---|---|
| Máy trạng thái TAG | `tag_ranging.c` gốc, được `#include`. Không có stub cho phần logic | `[CODE]` |
| Radio | `tests/dw1000_sim.c`: thanh ghi, IRQ, TX/RX timestamp, RX quality | Có sẵn trong repo |
| Airtime | Preamble `DW_PHY_PREAMBLE_SYMBOLS` × 0,99359 µs + SFD 8; PHR 19 bit ở 850 kb/s; data 6,8 Mb/s kèm Reed-Solomon | `[NGOÀI]` DW1000 UM |
| Đồng hồ | Hai bộ đếm 40-bit độc lập, lệch ppm riêng, có thể bắt đầu gần điểm wrap | Tham số của từng kịch bản |
| Luật reply của anchor | `DX = (a2 + ANCHOR_REPLY_DELAY_UUS·65536) & ~0x1FF`, RMARKER = DX + TX_ANTD, `Da = a3 − a2` | `[CODE]` `anchor_ranging.c:216-219` |
| Delayed TX trễ | Nếu `t_POLL_RX + đuôi frame + latency CPU + 10 µs + SHR > t_DX` thì anchor im lặng (tương đương HPDWARN/TXPUTE) | nRF 350 µs, STM32 500 µs |
| Nhiễu | Gauss trên mỗi RX timestamp, 10,4 tick, cho σ(DS-TWR) ≈ 30 mm | Có thể đổi bằng `MOTION_TS_NOISE_TICKS` |
| Bias cặp | 87, −43, 125, −12, 64, −95, 31, 150 mm. TAG được calibration đúng các giá trị này qua `Tag_SetDsCalibration`, nên kiểm được cả quy ước dấu | |
| FPP | `fpp_1m − 20·log10(d)`; NLOS trừ thêm 8 dB. `set_rx_quality` ghi RXPACC/F1–F3/CIR để TAG tự tính FPP bằng mã thật | −62 dBm @1 m (bay: −58) |
| Truth | Khoảng cách thật tại thời điểm RESP của **chính exchange đó** (ghép theo txn) | |

**Chỉ số:**

- **avail:** số range hợp lệ / số chu kỳ anchor còn hoạt động.
- **mean, p95, max:** sai số tuyệt đối của range hợp lệ.
- **lag:** trung bình (truth − published) chia vận tốc.
- **FA:** số range hợp lệ lệch quá 300 mm. Mẫu NLOS mà được phát là hợp lệ luôn tính là FA.
- **rec:** thời gian từ lúc hết nhiễu tới range hợp lệ đầu tiên lệch không quá 150 mm.
- **raw_b/sd:** bias và σ của range thô đã hiệu chỉnh.

### 6.3 Kịch bản và cổng nghiệm thu

| Kịch bản | Mô tả | Cổng: avail / p95 / lag / FA / rec |
|---|---|---|
| `static_3m` | Treo ở 3 m | ≥95 % / 60 mm / – / 0 / – |
| `drift_0.1` | Xuyên tâm 0,1 m/s | ≥95 % / 100 / 60 ms (sàn 10 mm) / 0 / – |
| `walk_0.5` | 0,5 m/s | ≥95 % / 120 / 60 ms / 0 / – |
| `run_1.0` | 1 m/s | ≥95 % / 150 / 60 ms / 0 / – |
| `fast_2.0` | 2 m/s | ≥95 % / 200 / 60 ms / 0 / – |
| `sine_1m_4s` | 4 m ±1 m, chu kỳ 4 s (đỉnh 1,6 m/s) | ≥95 % / 150 / – / 0 / – |
| `spikes_hold` | Treo, 10 % mẫu NLOS lẻ +0,8 m | ≥85 % / 60 / – / 0 / – |
| `spikes_walk` | 0,5 m/s, 10 % NLOS +0,8 m | ≥85 % / 150 / – / 0 / – |
| `dropout_1s` | 1 m/s, A1 im lặng 1 s | ≥85 % / 150 / – / 0 / ≤200 ms |
| `nlos_3s` | Treo, NLOS +0,6 m kéo dài 3 s | ≥85 % / 60 / – / 0 / ≤300 ms |
| `wrap_drift` | 1 m/s, ±20 ppm, wrap 40-bit tại 1 s | ≥95 % / 150 / 60 ms / 0 / – |
| `reacq_nlos` | 1 m/s, A1 im lặng 1 s, rồi 30 % NLOS +0,8 m trong 0,3 s | ≥85 % / 150 / – / 0 / ≤300 ms |
| `flight_8` | 8 anchor ở các góc phòng 10×8×3 m; bay hình số 8, tới 1,5 m/s; 2 % NLOS; TAG −5 ppm, anchor +8 ppm | ≥90 % / 150 / – / 0 / – |
| `weak_gap` | Treo ở 5 m, FPP −98 dBm (bộ lọc thấy −110 dBm), A1 im lặng 1 s | ≥85 % / 60 / – / 0 / ≤300 ms |

Mọi kịch bản còn phải đạt thêm: |bias thô| ≤ max(10 mm, 3·SE). Lỗi đổi dấu
calibration trên A1 sẽ cho bias 174 mm, nên ngưỡng này bắt được ngay.

Các cổng là **đề xuất**, cần xác nhận lại bằng yêu cầu bay thật. Cổng lag có
sàn 10 mm sai số trung bình: ở 0,1 m/s, một cổng 60 ms (tức 6 mm) chỉ còn đo
nhiễu thống kê.

### 6.4 Kết quả seed 0 `[MÔ HÌNH]`

Trích các kịch bản chính. Bảng đầy đủ ở Phụ lục B.

| Kịch bản | LEGACY (mặc định) | MEDIAN_GATE | CV_KALMAN_V2 |
|---|---|---|---|
| `static_3m` | 100 %, p95 45 mm | 100 %, p95 40 | 100 %, p95 25 |
| `drift_0.1` | **77,5 %, p95 393, lag 2 874 ms, 91 FA** | 100 %, 40, 31 ms | 100 %, 23, 30 ms |
| `walk_0.5` | **18,8 %, p95 2 835, lag 2 146 ms** | 100 %, 49, 23 ms | 100 %, 36, 23 ms |
| `run_1.0` | **15,4 %, p95 3 010** | 100 %, 56 | 100 %, 37 |
| `fast_2.0` | **13,5 %, p95 3 191** | 100 %, 85 | 100 %, 65 |
| `dropout_1s` | **6,9 %, không hồi phục** | 99,4 %, rec 42 ms | 99,4 %, rec 42 ms |
| `nlos_3s` | 85,1 %, **rec 1 042 ms** | 99,4 %, rec 42 | 98,3 %, rec 122 |
| `reacq_nlos` | 7,4 % | 97,4 %, **3 FA** | 97,1 %, **3 FA**, p95 103 |
| `flight_8` | **36,1 %, p95 2 610, 1 351 FA** | 99,8 %, p95 57, 0 FA | 99,8 %, p95 47, 0 FA |
| `weak_gap` | 99,4 % | 99,4 % | 99,4 % |
| **Trong cổng** | **3/14** | 13/14 + 1 known | 13/14 + 1 known |

Range thô (`raw_b/sd`) ở mọi kịch bản và mọi bộ lọc có bias −3…+3 mm, σ 29–31
mm, **kể cả qua wrap 40-bit và lệch ±20 ppm**. Toán DS-TWR, calibration và
wrap đều đúng.

### 6.5 Độ bền: 20 seed `[MÔ HÌNH]`

Số seed trượt cổng trên 20 seed. Cột **FA** là tổng số FA của 20 seed.

| Kịch bản | LEGACY | MEDIAN_GATE | CV_KALMAN_V2 | MEDIAN_GATE + vá | CV + vá | CV + vá + bù FPP = 0 |
|---|---|---|---|---|---|---|
| `static_3m` | 0/20 | 0/20 | 0/20 | 0/20 | 0/20 | 0/20 |
| `drift_0.1` | 20/20 | 0/20 | 0/20 | 0/20 | 0/20 | 0/20 |
| `walk_0.5` … `fast_2.0`, `sine` | 20/20 | 0/20 | 0/20 | 0/20 | 0/20 | 0/20 |
| `spikes_hold` | 2/20 (359 FA) | **1/20 (1 FA)** | **1/20 (1 FA)** | 0/20 | 0/20 | 0/20 |
| `spikes_walk` | 20/20 | **3/20 (5 FA)** | **3/20 (5 FA)** | 0/20 | 0/20 | 0/20 |
| `dropout_1s` | 20/20 | 0/20 (rec ≤ 42 ms) | 0/20 | 0/20 (rec ≤ 102 ms) | 0/20 | 0/20 |
| `nlos_3s` | 20/20 | 0/20 | **2/20 (p95 70)** | 0/20 | **2/20** | 0/20 |
| `wrap_drift` | 20/20 | 0/20 | 0/20 | 0/20 | 0/20 | 0/20 |
| `reacq_nlos` | 20/20 | **5/20 (10 FA)** | **5/20 (10 FA)** | 0/20 | 0/20 | 0/20 |
| `flight_8` | 20/20 (27 473 FA) | 0/20 | 0/20 | 0/20 | 0/20 | 0/20 |
| `weak_gap` | 1/20 | 0/20 | 0/20 | 0/20 | 0/20 | 0/20 |
| **Trong cổng** | 1/14 | 11/14 | 10/14 | **14/14** | 13/14 | **14/14** |

Quét mật độ NLOS sau khi mất tín hiệu (`reacq_nlos`, 20 seed, số seed trượt / tổng FA):

| Mật độ NLOS | MEDIAN_GATE | MEDIAN_GATE + vá | CV | CV + vá |
|---|---|---|---|---|
| 20 % | 4/20, 9 FA | **0/20, 0 FA** | 4/20, 9 FA | **0/20, 0 FA** |
| 30 % | 5/20, 10 FA | **0/20, 0 FA** | 5/20, 10 FA | **0/20, 0 FA** |
| 40 % | 7/20, 24 FA | 1/20, 2 FA | 7/20, 25 FA | 1/20, 3 FA |
| 60 % | 15/20, 109 FA | 10/20, 19 FA | 15/20, 113 FA | 10/20, 20 FA |

Ở 60 % NLOS kéo dài, bộ lọc một chiều theo range **không thể** tách NLOS khỏi
chuyển động thật. Muốn xử lý tới mức đó phải có thêm hai thứ:

- chỉ số NLOS từ `rx − fp` (H4);
- estimator 3D trên host (P7, §8.13).

### 6.6 C9.1 và C9.2 `[MÔ HÌNH]`

| Bộ lọc | Trong cổng | Nhận xét |
|---|---|---|
| LEGACY + C9.1 Adaptive ACTIVE | 3/14 | p95 248–315 mm khi chuyển động, lag 137–1 747 ms, FA 1–95 |
| LEGACY + C9.2 Motion ACTIVE | 4/14 | 0 FA (trừ `reacq_nlos`: 3), nhưng lag 85–314 ms, p95 92–220 mm khi chuyển động. Build `-Werror` thất bại (V11) |

Cả hai đều kém `MEDIAN_GATE` / `CV_KALMAN_V2` rõ rệt. Nên **bỏ hẳn** sau khi
chốt bộ lọc (khớp mục 5.4 của tài liệu Bitcraze).

### 6.7 Độ nhạy theo nhiễu `[MÔ HÌNH]`

5 seed cho mỗi mức nhiễu. p95 lấy giá trị tệ nhất; avail lấy giá trị thấp nhất.

| σ range | Bộ lọc | static p95 | walk 0,5 p95 | fast 2,0 p95 | flight_8 p95 | FA (`spikes_walk` + `flight_8`) |
|---|---|---|---|---|---|---|
| ≈ 60 mm | LEGACY | 88 | 2 806 (18,8 %) | 3 562 | 2 619 | 291 + 6 948 |
| | MEDIAN_GATE | 90 | 90 | 116 | 93 | 0 + 0 |
| | CV_KALMAN_V2 | **55** | **63** | **88** | **71** | 0 + 0 |
| ≈ 100 mm | LEGACY | 159 | 2 885 (17,2 %) | 3 622 | 2 620 | 289 + 6 993 |
| | MEDIAN_GATE | 150 | 149 | 169 | 146 | 4 + 6 |
| | CV_KALMAN_V2 | **93** | **97** | **116** | **106** | 0 + 0 |

Ghi chú: bảng này dùng 5 seed và cột p95 là p95 **tệ nhất**. Số liệu seed 0 ở
§6.4 (σ ≈ 30 mm) cho MEDIAN_GATE static p95 = 40 mm.

Kết luận:

- LEGACY hỏng khi chuyển động **ở mọi mức nhiễu**. Nguyên nhân là trễ, không phải nhiễu.
- Khi nhiễu lớn, median-3 đơn thuần không đủ làm mượt: ở σ ≈ 100 mm bắt đầu có FA.
- CV làm mượt tốt nhất, nhưng cần R đúng với nhiễu thật (§8.3). Nếu R quá nhỏ so với nhiễu thật, CV từ chối cả mẫu tốt (ví dụ: σ 100 mm với R lớp "mạnh" σ 50 mm cho 95 % hợp lệ khi treo).

### 6.8 Tín hiệu yếu và độ bù FPP `[MÔ HÌNH]`

Đặt FPP ở 1 m là −84 dBm cho mọi kịch bản, tức FPP thật −90…−110 dBm và bộ lọc
thấy −102…−122 dBm. Mỗi biến thể chạy 5 seed. Với −90 dBm @1 m, kết quả cũng
như vậy.

| Biến thể | MEDIAN_GATE | CV |
|---|---|---|
| Mã hiện tại | 13/14 (còn `reacq_nlos`) | 12/14 (thêm `nlos_3s` 1/5 seed) |
| **Bản vá đầu tiên** (cổng FPP áp cho cả lúc khởi tạo), chạy trên bộ 13 kịch bản lúc đó | **5/13. `static_3m`, `nlos_3s`, `flight_8` hợp lệ 0 %: không bao giờ khởi tạo** | 5/13 |
| Bản vá cuối (cổng FPP chỉ áp khi đang bám) | **14/14** | 13/14 (`nlos_3s` 1/5 seed) |

Bài học rút ra:

- Mọi cổng dựa trên FPP phải được test ở mức tín hiệu yếu nhưng vẫn dùng được.
- Kịch bản `weak_gap` đã được thêm để giữ đúng điều này về sau.
- Độ bù −12,04 dB làm mọi ngưỡng FPP của bộ lọc C9 bị lệch 12 dB so với con số trong `g_range_filter_config`.

### 6.9 Vì sao: cơ chế lỗi

**LEGACY (V01).** Kalman tĩnh với Q = 0,05 mm² mỗi mẫu. Ở trạng thái ổn định:

- a = (Q + √(Q² + 4QR)) / 2
- K = a / (a + R)

| Lớp FPP | R (mm²) | K |
|---|---:|---:|
| mạnh | 50 | 0,031 |
| trung bình | 200 | 0,016 |
| yếu | 1 000 | 0,007 |
| rất yếu | 10 000 | 0,0022 |

Với một đường dốc tốc độ v và chu kỳ T, bộ lọc hằng số K trễ khoảng v·T/K.
Ở 0,5 m/s và T = 25,6 ms: trễ khoảng 413 mm với R = 50, và khoảng 1 816 mm với R = 1 000.

Gate lại so mẫu mới với chính trạng thái đang trễ đó. Ngưỡng bên dương là
Vmax·dt + 100 = 356 mm, nên mẫu đúng bị loại. Khi bị loại, `dt` tăng dần,
ngưỡng nới ra, rồi đến lần thứ 15 thì snap về mẫu đo. Sau snap, trễ lại tích
lũy như cũ. Kết quả là răng cưa, chỉ 13–19 % mẫu hợp lệ.

Thêm một điểm nữa: lúc khởi tạo P = 1,0 mm² (`tag_ranging.c:1592`), nên ngay
từ đầu bộ lọc đã "tin" gần như tuyệt đối vào mẫu đầu tiên.

**C9 tái bắt khoá (V02).** Ba đường dẫn tới lỗi:

1. Sau một khoảng trống dài hơn 500 ms, bộ lọc khởi tạo **từ mẫu đầu tiên** và phát nó ngay (`range_filter.c:349-360`). Khi còn NLOS, mẫu đó sai +0,8 m.
2. Khi đang bám, 3 mẫu bị loại mà cách nhau không quá 250 mm là đủ để tái bắt khoá (`:179-207`). Ba mẫu NLOS liên tiếp thỏa điều kiện này.
3. Mỗi mẫu được chấp nhận xoá sạch ứng viên (`:477`). Khi đã khoá vào mức NLOS mà mẫu NLOS vẫn đến xen kẽ, các mẫu LOS không bao giờ tích đủ 3 ứng viên để kéo bộ lọc về.

**CV lố sau NLOS (V05).** Độ bù FPP đẩy các mẫu σ ≈ 30 mm vào lớp "yếu", với R = 40 000 mm² (σ 200 mm). Gate NIS vì thế rất lỏng: bước −0,6 m khi hết NLOS được nhận như một chuyển động thật. Ước lượng vận tốc bật lên, sai số xuống −120 mm và mất khoảng 300 ms mới về (vết ở `nlos_3s`, seed 3).

### 6.10 Chạy lại

Xem Phụ lục A. Tóm tắt:

- `run_host_tests.ps1 -Filter 'test_tag_motion*'`
- `test_tag_motion --seeds 20` để kiểm độ bền.

---

## 7. Thời gian slot, ngân sách anchor, SPI

### 7.1 Ngân sách delayed TX của anchor `[MÔ HÌNH]`

```
budget = REPLY_DELAY − đuôi frame POLL (sau RMARKER) − TX startup − SHR
       = 1 230,8 µs − 41,0 − 10 − 262,3 = 917 µs      (1 200 UUS, preamble 256)
```

Đây là **toàn bộ** thời gian anchor có, tính từ RMARKER của POLL: đọc frame,
parse, đọc timestamp, dựng RESP, ghi TX buffer, ghi DX_TIME, rồi phát lệnh TXDLYS.

### 7.2 Ma trận timing (MEDIAN_GATE, `flight_8`) `[MÔ HÌNH]`

| Preamble | Reply (UUS) | TAG RESP→FINAL | Ngân sách anchor | Chu kỳ/s | Chu kỳ dài nhất | Overrun (24 s) | Ghi chú |
|---:|---:|---:|---:|---:|---:|---:|---|
| 256 | 1 200 | 350 µs | 917 µs | **39,0** | 25,2 ms | 933 | Hiện tại |
| 256 | 1 200 | 150 µs | 917 µs | 41,5 | 23,6 ms | 995 | Tối ưu SPI trên TAG (V13) |
| 256 | 900 | 350 µs | 610 µs | 43,0 | 22,7 ms | 1 032 | |
| 256 | 900 | 150 µs | 610 µs | 46,2 | 21,1 ms | 1 108 | |
| 256 | 700 | 350 µs | 405 µs | (50,0) | 22,5 ms | 3 | **A5–A8 trễ hết, chỉ 50 % hợp lệ** (STM32 cần 500 µs); slot của chúng ngắn lại vì bỏ dở |
| 128 | 1 200 | 350 µs | 1 045 µs | 44,1 | 22,1 ms | 1 058 | |
| 128 | 900 | 350 µs | 737 µs | **49,6** | **19,7 ms** | **0** | Vừa 20 ms |
| 128 | 900 | 150 µs | 737 µs | 50,0 | 18,1 ms | 0 | |
| 128 | 700 | 350 µs | 532 µs | 50,0 | 18,0 ms | 0 | Biên STM32 rất mỏng |

Tổng số range mỗi giây: khoảng 312 hiện tại, khoảng 397 với preamble 128 +
reply 900. Muốn tăng gấp đôi phải dùng DS-TWR một-nhiều (S2).

### 7.3 Độ nhạy theo latency của STM32 `[MÔ HÌNH]`

Latency POLL→TXDLYS của A5–A8 được quét; bảng ghi độ sẵn có của cả hệ 8 anchor:

| Timing | 500 µs | 800 µs | 900 µs | 950 µs |
|---|---|---|---|---|
| 256 / 1 200 (hiện tại) | 99,8 % | 99,8 % | 99,8 % | **49,9 %** (965 RESP trễ) |
| 256 / 900 | 99,9 % | **49,9 %** | 50,0 % | 50,0 % |
| 128 / 900 | 99,9 % | **49,9 %** | 50,0 % | 50,0 % |
| 128 / 700 | 99,9 % | **49,9 %** | 50,0 % | 50,0 % |

Đây là một **vách đứng**: dưới ngưỡng thì mọi thứ ổn, vượt ngưỡng thì 4 anchor
STM32 trễ đồng loạt. Vì vậy **phải đo biên thời gian thật trước** (§8.5), rồi
mới giảm reply delay hay đổi preamble.

### 7.4 Số giao dịch SPI trên đường găng `[TEST]`

Đếm bằng bộ mô phỏng, mỗi giao dịch SPI = 1 lần gọi `uwb_platform_spi_xfer`:

| Đoạn | Số giao dịch / byte |
|---|---|
| Anchor: IRQ POLL → lệnh TXDLYS | 10 / 78 B (thêm 34 B TLV khi có yêu cầu info) |
| Anchor: FINAL → REPORT | 11 / 87 B (có đọc chẩn đoán, `UWB_ANCHOR_REPORT_DIAG=1`) |
| Anchor: frame lạc | 5 / 41 B |
| TAG: RESP RXFCG → FINAL TXSTRT (tính Db) | 12 |

Trên nRF52, SPI là loại ngắt theo từng byte (`uwb_platform_zephyr.c:196-201`),
nên mỗi giao dịch có chi phí cố định đáng kể. TAG có thể dời việc đọc chẩn đoán
RESP (FP, CIR, noise) sang lúc chờ REPORT (V13).

---

## 8. Những điểm cần sửa: sườn code và hướng làm

Thứ tự trong mục này là thứ tự nên làm. Mỗi mục ghi rõ: **tại sao**, **sửa ở
đâu**, **sườn code**, **test nghiệm thu**.

### 8.1 [V02] Tái bắt khoá có nhận biết NLOS: bản vá đã kiểm chứng

**Nguyên tắc.** NLOS chỉ làm range **dài ra**. Vì thế điểm bắt đầu phải là
**nhóm ngắn nhất** trong các mẫu gần đây, chứ không phải mẫu đầu tiên hay trung
bình của những gì đến trước. Cụ thể:

| Quy tắc | Lý do |
|---|---|
| Giữ tối đa 5 ứng viên gần nhất (bỏ ứng viên cũ hơn `stale_reset_ms`) | Có đủ bằng chứng mà vẫn ít tốn RAM (44 B mỗi anchor) |
| Nhóm = `need` mẫu **ngắn nhất** (`need` = `reacquire_min_samples` = 3), trải không quá `reacquire_cluster_mm` | Mẫu LOS luôn nằm ở đáy |
| Chưa bám (lúc khởi động hoặc sau mất tín hiệu): cần `need + 1` mẫu. **Không** áp cổng FPP | Muốn nhóm toàn NLOS thì mọi mẫu gần đây đều phải NLOS. Một liên kết yếu vẫn phải khởi động được |
| Đang bám mà nhóm **dài hơn** track: cần buffer đầy (5 mẫu) và cả 5 đều khớp nhau | Đây chính là hình dạng của NLOS |
| Mẫu được chấp nhận **giữ lại** các ứng viên ngắn hơn track − cluster | Cho phép thoát khỏi mức NLOS |
| Đầu vào không hợp lệ thì xoá nhóm | Giữ ngữ nghĩa cũ: cần các mẫu liên tiếp dùng được |

**Áp bản vá:**

```powershell
git apply Plan/patches/0001-range-filter-nlos-aware-reacquire.patch
powershell -ExecutionPolicy Bypass -File Firmware\tests\run_host_tests.ps1
```

Bản vá sửa và thêm các file sau:

| File | Nội dung |
|---|---|
| `common/include/range_filter.h` | `RANGE_FILTER_REACQUIRE_BUF` và 3 trường mới trong state |
| `common/src/filters/range_filter.c` | `update_reacquire_candidate()` viết lại, thêm `keep_lower_candidates()`, `clear_candidates()`; đường boot/gap/cv_predict dùng nhóm |
| `tests/test_range_filter.c` (**mới**) | 7 unit test cho các quy tắc trên. Mã hiện tại trượt 9 kiểm tra, bản vá đạt hết |
| `tests/run_host_tests.ps1` | Thêm `test_range_filter_median_gate` và `test_range_filter_cv_kalman` |
| `tests/test_tag_motion.c` | `reacq_nlos` đổi thành cổng bắt buộc (`known_issue = 0`) |

Hàm lõi (trích từ bản vá):

```c
static uint8_t update_reacquire_candidate(
    RangeFilterState_t *state, int32_t candidate_mm, float fpp_dbm,
    uint32_t now_ms, const RangeFilterConfig_t *config, int32_t *start_mm)
{
    int32_t sorted[RANGE_FILTER_REACQUIRE_BUF];
    uint8_t need = config->reacquire_min_samples;
    uint8_t evidence;
    uint8_t n = 0U;
    int64_t sum = 0;
    int32_t group_mm;

    if (need < 2U)
        need = 2U;
    if (need > RANGE_FILTER_REACQUIRE_BUF - 1U)
        need = (uint8_t)(RANGE_FILTER_REACQUIRE_BUF - 1U);
    evidence = state->initialized ? need : (uint8_t)(need + 1U);
    if (state->initialized && fpp_dbm < config->reacquire_min_fpp_dbm)
        return 0U;                      /* FPP chỉ chặn khi đang bám */

    /* Giữ ứng viên chưa hết hạn (cũ trước), rồi thêm mẫu này. */
    for (uint8_t i = 0U; i < state->candidate_buf_count; i++)
    {
        if ((uint32_t)(now_ms - state->candidate_buf_ms[i]) > config->stale_reset_ms)
            continue;
        state->candidate_buf_mm[n] = state->candidate_buf_mm[i];
        state->candidate_buf_ms[n] = state->candidate_buf_ms[i];
        n++;
    }
    if (n == RANGE_FILTER_REACQUIRE_BUF)
    {   /* buffer đầy: bỏ ứng viên cũ nhất */
        memmove(&state->candidate_buf_mm[0], &state->candidate_buf_mm[1],
                (RANGE_FILTER_REACQUIRE_BUF - 1U) * sizeof(state->candidate_buf_mm[0]));
        memmove(&state->candidate_buf_ms[0], &state->candidate_buf_ms[1],
                (RANGE_FILTER_REACQUIRE_BUF - 1U) * sizeof(state->candidate_buf_ms[0]));
        n--;
    }
    state->candidate_buf_mm[n] = candidate_mm;
    state->candidate_buf_ms[n] = now_ms;
    n++;
    state->candidate_buf_count = n;
    state->candidate_count = n;
    if (n < evidence)
        return 0U;

    memcpy(sorted, state->candidate_buf_mm, (size_t)n * sizeof(sorted[0]));
    /* insertion sort, n <= 5 (xem bản vá) */
    ...
    if ((int64_t)sorted[need - 1U] - (int64_t)sorted[0]
        > (int64_t)config->reacquire_cluster_mm)
        return 0U;                      /* còn mẫu ngắn hơn chưa khớp: chờ */
    for (uint8_t k = 0U; k < need; k++)
        sum += sorted[k];
    group_mm = (int32_t)(sum / (int64_t)need);
    if (state->initialized && (float)group_mm > state->distance_mm
        && (n < RANGE_FILTER_REACQUIRE_BUF
            || (int64_t)sorted[n - 1U] - (int64_t)sorted[0]
                > (int64_t)config->reacquire_cluster_mm))
        return 0U;                      /* nhóm dài hơn track: cần đủ 5 mẫu khớp */
    *start_mm = group_mm;
    state->candidate_mean_mm = (float)group_mm;
    return 1U;
}
```

**Đã kiểm chứng** `[TEST][BUILD]`:

| Kiểm | Kết quả |
|---|---|
| Runner đầy đủ trên cây đã vá | 16/16 bước đạt |
| Motion MEDIAN_GATE / CV, seed 0 | 14/14 / 14/14 |
| Motion, 20 seed | MEDIAN_GATE 14/14; CV 13/14 (còn `nlos_3s`, do V05), CV + bù FPP = 0 thì 14/14 |
| Unit test trên mã hiện tại | Trượt 9 kiểm tra, đúng như mong đợi: test phân biệt được hai phiên bản |
| ASan/UBSan | 0 lỗi |
| `gcc -fanalyzer` | 0 cảnh báo |
| Build Zephyr TAG (mode 1 và 2) | 0 cảnh báo |
| Cross-compile Cortex-M3 (STM32) | 0 cảnh báo |
| Chế độ LEGACY | Kết quả motion giống hệt trước khi vá (LEGACY không gọi `RangeFilter_Update`) |

**Cái giá phải trả** `[MÔ HÌNH]`:

- khởi động phát range hợp lệ đầu tiên sau 4 mẫu thay vì 1 (thêm khoảng 60–80 ms);
- hồi phục sau mất tín hiệu 102 ms thay vì 42 ms (cổng 200 ms);
- thêm khoảng 0,5 KB flash và 44 B RAM mỗi anchor.

Mã hiện tại có thể **phát ngay một mẫu NLOS sau khi mất tín hiệu**. Bản vá chấp
nhận thêm khoảng 60 ms trễ để không phát mẫu sai đó, trừ khi mọi mẫu gần đây
đều NLOS.

**Việc còn lại sau khi áp:** cập nhật `CHANGELOG.md`. Nếu GUI có dùng
`candidate_count` thì cần biết nghĩa mới của nó là "số ứng viên trong buffer".
Hiện `candidate_count` không được đọc ngoài `range_filter.c`.

### 8.2 [V01] Đổi bộ lọc mặc định của TAG

**Quyết định đề xuất:**

1. **Firmware TAG dùng `MEDIAN_GATE` + bản vá §8.1.** Việc của bộ lọc firmware là:
   - đặt cờ hợp lệ;
   - loại outlier/NLOS;
   - trễ tối thiểu.

   Nó **không** nên làm mượt thay estimator. Theo ADR-001 và mục 5.3 của tài liệu Bitcraze, estimator trên UP 7000 sẽ dùng `corrected_mm`. `MEDIAN_GATE` không có tham số mô hình nào cần fit, và đạt 14/14 trên 20 seed.
2. **`CV_KALMAN_V2` chỉ dùng khi cần một đầu ra đã làm mượt ngay trên firmware**, ví dụ khi đưa thẳng vào MAVLink mà chưa có estimator trên host. Điều kiện: phải fit R từ log trước (§8.3).
3. **`LEGACY` chỉ còn làm baseline trên bàn thử tĩnh.** Không dùng khi bay.

Quy tắc chọn theo σ đo được ở T2 (§10):

| σ range tĩnh đo được | Lựa chọn |
|---|---|
| ≤ 60 mm | `MEDIAN_GATE` + vá |
| > 60 mm, hoặc cần firmware làm mượt | `CV_KALMAN_V2` + vá + R fit từ log |
| > 150 mm | Xem lại phần RF trước (TX power, anten, NLOS). Bộ lọc không phải là chỗ sửa |

**Sửa** `Firmware/Tag_DevKit/include/uwb_app_config.h`. File này được include
trước `uwb_calibration.h`, và macro trong đó có guard `#ifndef`:

```c
/* Conditioner: MEDIAN_GATE + NLOS-aware reacquire (Plan/patches/0001).
 * Legacy median + static Kalman lags 0.75-2.9 s when the drone moves
 * (test_tag_motion.c, Plan/BAO_CAO_KIEM_TRA_FIRMWARE_V2_2026-09-24.md). The
 * host estimator uses corrected_mm; this filter only sets validity. */
#define UWB_RANGE_FILTER_MODE 1U   /* UWB_RANGE_FILTER_MEDIAN_GATE */
```

Và **`run_host_tests.ps1`**: sau khi đổi mặc định, bước `test_tag_motion` phải
chạy strict (bỏ `--report`). Giữ Legacy làm bước khảo sát riêng:

```powershell
    @{ Name = 'test_tag_motion'; Source = 'test_tag_motion'; Role = 'TAG'
       IncludeDir = 'tests\motion_cfg'
       Sources = @('sim', 'drivers\dw1000.c', 'ranging\uwb_frame.c', 'filters\range_filter.c') }
    @{ Name = 'test_tag_motion_legacy'; Source = 'test_tag_motion'; Role = 'TAG'
       IncludeDir = 'tests\motion_cfg'; Defines = @('UWB_RANGE_FILTER_MODE=0U')
       Sources = @('sim', 'drivers\dw1000.c', 'ranging\uwb_frame.c', 'filters\range_filter.c')
       Arguments = @('--report') }
```

`test_tag_state` đang dùng `s_kf` và chỉ build được ở chế độ Legacy. Hãy thêm
`Defines = @('UWB_RANGE_FILTER_MODE=0U')` cho nó, để nó không vỡ khi mặc định
đổi.

Đổi `uwb_app_config.h` sẽ làm đổi config hash nhúng trong `DEVICE_INFO`. Đây là
điều mong muốn, vì nhờ vậy truy vết được bản đang nạp.

**Nghiệm thu:**

- runner xanh;
- `test_tag_motion --seeds 20` đạt 14/14;
- trên phần cứng, A/B với Legacy ở T9 (§10): di chuyển theo quỹ đạo biết trước; chỉ số là tỉ lệ hợp lệ, p95, và FA đối chiếu với ground truth.

### 8.3 [V05] Độ bù FPP và phương sai đo R theo số đo thật

**Ngắn hạn.** Chỉ Legacy mới cần thang FPP cũ, nên để độ bù phụ thuộc bộ lọc
(`uwb_calibration.h`, đặt sau định nghĩa `UWB_RANGE_FILTER_MODE`):

```c
#ifndef UWB_FILTER_FPP_COMPAT_DB
#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN
/* Legacy thresholds were tuned while RXPACC was mis-read (-12.04 dB). */
#define UWB_FILTER_FPP_COMPAT_DB (-12.04f)
#else
/* C9 conditioners: corrected FPP. Their variance classes must be refit
 * on corrected-FPP logs (T2/T3) before CV_KALMAN_V2 is enabled. */
#define UWB_FILTER_FPP_COMPAT_DB 0.0f
#endif
#endif
```

`[MÔ HÌNH]`: CV + vá + bù 0 đạt 14/14 trên 20 seed, và `nlos_3s` có p95 35 mm
thay vì 70.

**Dài hạn: R lấy từ bảng σ(FPP) fit từ log.** Không dùng 3 lớp cứng nữa.

```c
/* range_filter.c — thay measurement_variance() */
typedef struct { float fpp_dbm; float sigma_mm; } RangeSigmaPoint_t;

/* TUNE_REQUIRED: fill from T2/T3 static logs, corrected FPP, one table per
 * board family if A5-A8 differ from A1-A4. Sorted by fpp_dbm ascending. */
static const RangeSigmaPoint_t k_sigma_by_fpp[] = {
    { -105.0f, 0.0f }, { -95.0f, 0.0f }, { -85.0f, 0.0f }, { -75.0f, 0.0f },
};

static float sigma_for_fpp(float fpp_dbm)
{
    const size_t n = sizeof(k_sigma_by_fpp) / sizeof(k_sigma_by_fpp[0]);
    if (fpp_dbm <= k_sigma_by_fpp[0].fpp_dbm)
        return k_sigma_by_fpp[0].sigma_mm;
    for (size_t i = 1U; i < n; i++)
    {
        if (fpp_dbm <= k_sigma_by_fpp[i].fpp_dbm)
        {
            const RangeSigmaPoint_t *a = &k_sigma_by_fpp[i - 1U];
            const RangeSigmaPoint_t *b = &k_sigma_by_fpp[i];
            const float w = (fpp_dbm - a->fpp_dbm) / (b->fpp_dbm - a->fpp_dbm);
            return a->sigma_mm + w * (b->sigma_mm - a->sigma_mm);
        }
    }
    return k_sigma_by_fpp[n - 1U].sigma_mm;
}

static float measurement_variance(float fpp_dbm, const RangeFilterConfig_t *config)
{
    const float sigma = sigma_for_fpp(fpp_dbm);
    return clamp_f32(sigma * sigma, config->variance_floor_mm2, config->variance_ceiling_mm2);
}
```

Script fit σ từ log tĩnh T2/T3 (Python, chạy trên PC; `truth_mm` đo bằng laser):

```python
"""Fit sigma(FPP) per anchor family from static RANGE_MEAS logs.

Input CSV (session_recorder export): anchor_id, fp_cdbm, corrected_mm, flags,
truth_mm. Robust sigma = 1.4826 * MAD, 2 dB FPP bins, >= 200 samples per bin.
"""
import csv, statistics, sys
from collections import defaultdict

CAL_OK = 0x0002
bins = defaultdict(list)
for row in csv.DictReader(open(sys.argv[1], newline="")):
    if not int(row["flags"]) & CAL_OK:
        continue
    fpp = int(row["fp_cdbm"]) / 100.0
    err = int(row["corrected_mm"]) - float(row["truth_mm"])
    family = "dwm1001" if int(row["anchor_id"]) <= 4 else "stm32"
    bins[(family, 2 * round(fpp / 2))].append(err)

for (family, fpp), errs in sorted(bins.items()):
    if len(errs) < 200:
        continue
    med = statistics.median(errs)
    mad = statistics.median(abs(e - med) for e in errs)
    print(f"{family:8s} {fpp:6.0f} dBm  n={len(errs):5d}  bias={med:6.1f} mm  "
          f"sigma={1.4826 * mad:5.1f} mm")
```

Nghiệm thu: sau khi fit, NIS trung bình trên log tĩnh nằm trong 0,8–1,2. NIS
quá cao nghĩa là R quá nhỏ; quá thấp nghĩa là R quá lớn.

### 8.4 [V07] TLV danh tính anchor và ràng buộc calibration

**Vì sao.** Bias DS-TWR là tính chất của **cặp phần cứng**: anten, antenna
delay, vỏ máy. Hiện TAG chỉ biết "anchor ID 5", không biết đó là board DW1000
nào. Đổi board cùng ID thì calibration cũ vẫn được áp.

**Giao thức** (`uwb_frame.h`). TLV còn trống 44 − 34 = 10 B, không đủ chỗ cho
cả danh tính lẫn sức khỏe. Vì vậy dùng một **trang info thứ hai**. Anchor cũ bỏ
qua bit trang và vẫn trả trang 0, nên tương thích ngược.

```c
#define UWB_POLL_FLAG_REQ_INFO       0x01U  /* ask the anchor for its info TLVs */
#define UWB_POLL_FLAG_INFO_PAGE1     0x02U  /* page 1: identity + health (§8.5) */

#define UWB_TLV_ANCHOR_IDENTITY      0x04U  /* u32 part_id, u32 lot_id (DW1000 OTP) */
#define UWB_TLV_ANCHOR_IDENTITY_LEN  8U
#define UWB_TLV_ANCHOR_HEALTH        0x05U  /* §8.5 */
#define UWB_TLV_ANCHOR_HEALTH_LEN    12U
```

**Anchor** (`anchor_ranging.c`, trong `handle_poll`, thay đoạn ở dòng 223-229):

```c
static uint8_t build_info_page1(uint8_t *tlv)
{
    uint8_t len = 0U;

    tlv[len++] = UWB_TLV_ANCHOR_IDENTITY;
    tlv[len++] = UWB_TLV_ANCHOR_IDENTITY_LEN;
    uwb_put_u32(&tlv[len], dw1000_otp.valid ? dw1000_otp.part_id : 0U);
    uwb_put_u32(&tlv[len + 4U], dw1000_otp.valid ? dw1000_otp.lot_id : 0U);
    len = (uint8_t)(len + UWB_TLV_ANCHOR_IDENTITY_LEN);
    len = (uint8_t)(len + build_health_tlv(&tlv[len]));   /* §8.5 */
    return len;                                           /* 24 B <= 44 B */
}

    if (poll->version >= UWB_FRAME_V2 && (poll->flags & UWB_POLL_FLAG_REQ_INFO) != 0U)
    {
        tlv_len = (poll->flags & UWB_POLL_FLAG_INFO_PAGE1) != 0U
            ? build_info_page1(tlv) : build_info_tlvs(tlv);
        anchor_stats.info_replies++;
    }
```

**TAG, phía xin** (`tag_ranging.c:1305` và lịch info ở dòng ~1983):

- Ngay sau khi boot, xin trang 1 của **từng** anchor ở chu kỳ kế tiếp, cho tới khi mọi anchor trong mask đã báo danh tính.
- Sau đó mới quay về nhịp xin info 1 lần mỗi 50 chu kỳ, luân phiên trang 0 và trang 1.

```c
        .flags = ((int8_t)s_current_anchor == s_info_request_idx)
            ? (uint8_t)(UWB_POLL_FLAG_REQ_INFO
                        | (s_info_page1_due ? UWB_POLL_FLAG_INFO_PAGE1 : 0U))
            : 0U,
```

**TAG, phía nhận** (`capture_anchor_info`, `tag_ranging.c:1349`): thêm vào
`TagAnchorInfo_t` các trường `part_id`, `lot_id`, `identity_valid` và
`identity_changes`.

```c
    const uint8_t *ident = uwb_frame_find_tlv(resp->tlv, resp->tlv_len,
                                              UWB_TLV_ANCHOR_IDENTITY,
                                              UWB_TLV_ANCHOR_IDENTITY_LEN);
    if (ident != NULL)
    {
        const uint32_t part = uwb_get_u32(&ident[0]);
        const uint32_t lot = uwb_get_u32(&ident[4]);
        if (info->identity_valid && (info->part_id != part || info->lot_id != lot))
            info->identity_changes++;        /* đổi board, hoặc 2 anchor trùng ID (§8.7) */
        info->part_id = part;
        info->lot_id = lot;
        info->identity_valid = (part != 0U) ? 1U : 0U;
        tag_check_pair_binding(idx, part, lot);   /* invalid calibration nếu không khớp */
    }
```

**Settings** (`uwb_settings.c`): nâng schema lên 3.
`_Static_assert(sizeof(SettingsBlobV2_t) == 80U)` sẽ nhắc bước này.

```c
typedef struct __attribute__((packed)) {
    int32_t  bias_um;
    uint8_t  calibrated;
    uint32_t anchor_part_id;   /* 0 = chưa gắn: fail-closed */
    uint32_t anchor_lot_id;
} CalEntryV3_t;                /* 13 B x 8 = 104 B */

/* Migration v2 -> v3: giữ bias_um nhưng calibrated = 0 và part/lot = 0.
 * Chỉ lệnh CAL_BIND (commissioning) mới ghi part/lot đang thấy qua TLV
 * sau khi kiểm range ở cự ly đã biết. Không tự "tin lần đầu" (TOFU). */
```

**Quy tắc lúc chạy** (`tag_check_pair_binding`):

| Tình huống | Hành động |
|---|---|
| Entry `calibrated = 1` và `(part, lot)` khớp | Áp bias. Cờ `CAL_OK` |
| `(part, lot)` khác | Invalid calibration **của anchor đó**. Cờ `CAL_MISSING`, và thêm status mới `TAG_ST_CAL_IDENTITY` |
| Chưa nhận được danh tính kể từ boot | Coi là `CAL_MISSING` (fail-closed). Tối đa vài chu kỳ vì trang 1 được xin ưu tiên |
| Anchor là firmware cũ, không có TLV danh tính | Như trên. Phải nâng anchor |

Phía host (Python): thêm `part_id`, `lot_id` vào `AnchorInfoMessage`, và cập
nhật golden (`test_telemetry_golden.c/.py`).

Nghiệm thu:

- unit test: đổi part_id thì calibration bị invalid; migration v2→v3 là fail-closed;
- trên phần cứng: đổi hai board A5↔A6 thì range của cả hai chuyển thành `CAL_MISSING`.

### 8.5 [V04] TLV sức khỏe anchor và đo biên thời gian delayed TX

**Cách đo đúng.** Không đo bằng đồng hồ MCU, mà đo bằng chính DW1000. Ngay sau
lệnh TXDLYS, đọc SYS_TIME: biên = `DX_TIME − SYS_TIME`. Lần đọc này nằm **sau**
điểm găng, không làm chậm RESP. Muốn RESP không trễ thì biên phải lớn hơn
SHR + khởi động TX (khoảng 272 µs với preamble 256).

```c
/* anchor_ranging.c — trong handle_poll(), ngay sau DW1000_StartTxDelayedEx() thành công */
#if UWB_ANCHOR_MEASURE_MARGIN
    {
        uint8_t now[5];
        DW1000_ReadSysTime(now);                          /* dw1000.c:666 */
        const uint64_t margin = (resp_tx - ts_to_u64(now)) & UWB_TS40_MASK;
        const uint32_t margin_us = (uint32_t)(margin / 63898U);   /* 63 897,6 tick/µs */
        if (margin_us < s_margin_min_us)
            s_margin_min_us = margin_us;
        if (margin_us < ANCHOR_MARGIN_ALARM_US)            /* ví dụ SHR + 150 = 420 µs */
            anchor_stats.margin_low++;
    }
#endif
```

Nếu biên âm, hoặc thấp hơn SHR, thì TX thực tế đã trễ và HPDWARN/TXPUTE phải
đã báo. Hai số liệu này phải khớp nhau.

**TLV sức khỏe** (trang 1, 12 B):

```c
static uint8_t build_health_tlv(uint8_t *tlv)
{
    tlv[0] = UWB_TLV_ANCHOR_HEALTH;
    tlv[1] = UWB_TLV_ANCHOR_HEALTH_LEN;
    uwb_put_u16(&tlv[2],  sat16(anchor_stats.delayed_tx_late));
    uwb_put_u16(&tlv[4],  sat16(anchor_stats.rx_errors));
    uwb_put_u16(&tlv[6],  sat16(anchor_stats.idle_rx_restarts));
    uwb_put_u16(&tlv[8],  sat16(s_margin_min_us));      /* 0xFFFF = chưa đo */
    uwb_put_u16(&tlv[10], sat16(anchor_stats.margin_low));
    uwb_put_u16(&tlv[12], sat16(anchor_stats.txn_mismatch));
    return (uint8_t)(2U + UWB_TLV_ANCHOR_HEALTH_LEN);
}
```

Phía TAG: đưa các trường này vào `TagAnchorInfo_t` và vào gói `ANCHOR_INFO`.
GUI hiện thêm cột "biên TX (µs)" và "trễ TX".

**Vì sao cần gấp:** A5–A8 không có UART, không có RTT
(`STM32_Anchor_5/prj.conf`), nên hiện chỉ đọc được bộ đếm qua SWD. Với TLV này,
số liệu đi theo đường vô tuyến tới GUI.

Nghiệm thu:

- bài 10 phút, 8 anchor, cấu hình hiện tại;
- biên nhỏ nhất ≥ SHR + 150 µs trên mọi anchor;
- `delayed_tx_late` = 0;
- biên của A5–A8 là dữ liệu đầu vào cho §8.6.

### 8.6 [V03, V13] Lịch đo 8 anchor

Chỉ làm **sau** §8.5. Các phương án, xếp theo rủi ro tăng dần:

| Phương án | Sửa | Lợi `[MÔ HÌNH]` | Điều kiện |
|---|---|---|---|
| **A. Rút đường Db của TAG** (V13) | Dời việc đọc chẩn đoán RESP (FP, CIR, noise, CI) ra sau lệnh FINAL, làm trong lúc chờ REPORT. Giảm từ 12 xuống khoảng 6 giao dịch SPI | 39 → 41,5 Hz khi TAG còn 150 µs | Không đổi RF. Db ngắn hơn không làm tăng sai số DS bất đối xứng `[NGOÀI]` |
| **B. Reply delay riêng từng anchor** | `ANCHOR_REPLY_DELAY_UUS` hiện là `#define` cứng (`anchor_ranging.h:32`); đổi thành `#ifndef` để từng node tự đặt. nRF: 900; STM32: giữ 1 200, hoặc theo biên đo được | Tối đa 43 Hz khi mọi anchor 900 | TAG không cần biết giá trị này vì Da đi trong RESP. Mỗi anchor phải có biên ≥ SHR + 150 µs |
| **C. Preamble 128, PAC 8** (S4) | Hồ sơ PHY mới cho **mọi** nút (bảng dưới). Profile ID mới làm mọi calibration cũ bị invalid, đây là điều cố ý | 49,6 Hz, 0 overrun (kèm reply 900) | A/B σ và tầm (T6-like) với ≥ 1 000 mẫu mỗi điểm. Chỉ giữ nếu σ không tăng |
| **D. Lịch 4+4** (S1) | Mỗi chu kỳ chỉ đo một nhóm 4 anchor không đồng phẳng | Chu kỳ chắc chắn dưới 20 ms. **Không** tăng số range mỗi giây | Estimator phải xử lý range theo đúng thời điểm đo |
| **E. DS-TWR một-nhiều** (S2/GĐ5) | POLL/FINAL broadcast; RESP/REPORT theo slot | Khoảng gấp đôi số range mỗi giây | Viết lại cả TAG và anchor |

Sườn cho B (`anchor_ranging.h`):

```c
#ifndef ANCHOR_REPLY_DELAY_UUS
#define ANCHOR_REPLY_DELAY_UUS  1200UL
#endif
_Static_assert(ANCHOR_REPLY_DELAY_UUS >= 700UL && ANCHOR_REPLY_DELAY_UUS <= 2000UL,
               "reply delay outside the validated range");
```

Trong `Anchor_1/include/uwb_app_config.h`:

```c
/* Measured margin (TLV health) >= SHR + 150 us at 900 UUS on this board. */
#define ANCHOR_REPLY_DELAY_UUS 900UL
```

Sườn cho C: bảng hồ sơ PHY. Các giá trị thanh ghi phải **đối chiếu DW1000 User
Manual** trước khi ghi `[NGOÀI]`.

```c
typedef struct {
    uint16_t preamble_symbols;
    uint8_t  pac_symbols;
    uint16_t sfd_timeout;        /* preamble + 1 + SFD(8) - PAC */
    uint32_t tx_fctrl_upper;     /* ((PLEN | PRF16) << 16) | TR | TXBR 6M8 */
    uint32_t drx_tune2;          /* PAC x PRF16 */
    uint8_t  profile_id;         /* đi vào calibration profile hash */
} DwPhyProfile_t;

static const DwPhyProfile_t k_phy_fast256 = { 256U, 16U, 249U, 0x0025C000UL, 0x331A0052UL, 2U };
static const DwPhyProfile_t k_phy_fast128 = { 128U,  8U, 129U, 0x0015C000UL, 0x311A002DUL, 3U };
/* DRX_TUNE1b (0x0020) và DRX_TUNE4H (0x0028) giữ nguyên với preamble 128 ở 6,8 Mb/s. */
```

Quyết định nên làm theo thứ tự: A, rồi B (theo biên đo được), rồi A/B cho C. D
và E chỉ làm khi estimator trên host thật sự cần nhiều range hơn.

### 8.7 [V08] Chống trùng ID anchor

**Lúc build: test cấu hình triển khai.** Thêm `Firmware/deployment.json`, liệt
kê image nào nạp vào nhãn board nào, rồi thêm test sau:

```python
"""tests/test_deployment_ids.py: the deployed set is exactly A1..A8, one image each."""
import json, re, sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
ADDR = re.compile(r"#define\s+ANCHOR_ADDR\s+\(\(uint16_t\)(\w+)\)")
ANCHOR_ID = re.compile(r"#define\s+UWB_ANCHOR_ID\s+(\d+)U?")

def anchor_id(project: str) -> int:
    text = (FW / project / "include" / "uwb_app_config.h").read_text(encoding="utf-8")
    value = ADDR.search(text).group(1)              # "5U" (nRF) or "UWB_ANCHOR_ID" (STM32)
    if value == "UWB_ANCHOR_ID":
        return int(ANCHOR_ID.search(text).group(1))
    return int(value.rstrip("uU"))

deployment = json.loads((FW / "deployment.json").read_text(encoding="utf-8"))
# {"A1": "Anchor_1", ..., "A5": "STM32_Anchor_5", ..., "A8": "STM32_Anchor_8"}
ids = {label: anchor_id(project) for label, project in deployment.items()}
assert sorted(ids.values()) == list(range(1, 9)), f"anchor IDs not 1..8 unique: {ids}"
for label, value in ids.items():
    assert label == f"A{value}", f"{label} flashes ID {value}"
spare = [p.name for p in FW.glob("Anchor_*") if p.name not in deployment.values()]
print(f"deployment IDs ok; spare projects not to flash: {spare}")
```

Với `Firmware/Anchor_5..8` (nRF), chọn một trong hai:

- đổi tên thành `Anchor_5_nrf_spare` …; hoặc
- thêm `#error` khi build mà không có cờ `-DUWB_ALLOW_SPARE_NRF_ANCHOR`.

Như vậy không thể vô tình nạp chúng khi A5–A8 STM32 đang chạy.

**Lúc chạy.** Dựa vào TLV danh tính (§8.4): cùng một ID mà `part_id` đổi qua
lại nhiều lần trong thời gian ngắn thì nghi có trùng ID. Khi đó:

- tăng `identity_changes`;
- gắn status `TAG_ST_DUP_ID`;
- invalid range của ID đó.

Kèm một dấu hiệu phụ: range hai đỉnh (bimodal) với cùng ID. Có thể phát hiện
trên host.

### 8.8 [V09] Truy vết bản phát hành

1. **`.gitattributes`** ở gốc repo, để git lưu đúng từng byte và hash không phụ thuộc máy:

   ```gitattributes
   *.hex  -text
   *.bin  -text
   *.elf  -text
   *.map  -text
   ```

2. **Chặn build phát hành từ cây mã bẩn** (`common/cmake/uwb_node.cmake`, trong `uwb_node_setup`):

   ```cmake
   option(UWB_RELEASE_BUILD "Refuse a release image from an uncommitted tree" OFF)
   if(UWB_RELEASE_BUILD AND NOT build_dirty EQUAL 0)
     message(FATAL_ERROR "UWB_RELEASE_BUILD: tree is dirty (git status not clean); "
                         "commit or stash before building release images")
   endif()
   ```

   `build_all.ps1` và `build_stm32_anchors.ps1` truyền `-DUWB_RELEASE_BUILD=ON` khi có tham số `-Release`.

3. **`package_8_anchors.ps1`** ghi thêm vào `manifest.csv`:
   - `GitHash` (`git rev-parse --short=8 HEAD`), `Dirty`, `ConfigHash`;
   - SHA-256 tính trên **đúng file sẽ nạp**.

   Script kiểm hash nhúng trong image bằng đoạn Python dưới đây, và **dừng** nếu không khớp:

   ```python
   """Check that an image embeds the expected build hash (little-endian u32)."""
   import sys
   from intelhex import IntelHex          # pip install intelhex
   image, expected = sys.argv[1], int(sys.argv[2], 16)
   data = IntelHex(image).tobinstr()
   if data.count(expected.to_bytes(4, "little")) != 1:
       sys.exit(f"{image}: build hash 0x{expected:08x} not embedded exactly once")
   ```

4. **Build lại** 9 image từ commit đã tag, rồi thay `anchor_8_dist/` và `stm32_anchor/dist/`.

Nghiệm thu: trên một clone mới ở **cả Windows lẫn Linux**, `Get-FileHash`
(hoặc `sha256sum`) khớp manifest, và hash nhúng trong image bằng tag.

### 8.9 [V06] TX power và reference tuning

- `LEGACY` = `0x1E1E1E1E`: coarse 15 dB + fine 15 dB = **30 dB**. `REFERENCE` = `0x48484848`: 10 + 4 = **14 dB**, là giá trị tham chiếu cho Ch5/PRF16 trong User Manual. Hai giá trị lệch nhau **16 dB** `[CODE][NGOÀI]`.
- Giá trị tham chiếu được hiệu chỉnh cho board mẫu, sao cho phổ nằm sát giới hạn −41,3 dBm/MHz. Vì vậy LEGACY **rất có thể vượt giới hạn phát xạ** `[NGOÀI][GIẢ THUYẾT]`. Mức thu cao ở cự ly gần cũng làm tăng bias phụ thuộc mức RX (P2, P8).
- `UWB_DW_REFERENCE_TUNING` = 0 (`uwb_calibration.h:357`), nên LDE NTM = 13, LDOTUNE và XTAL trim từ OTP **không được áp**. Tắt XTAL trim thì lệch tần số giữa các nút lớn hơn. DS-TWR chịu được, nhưng đường SS fallback và bù CI thì không.

**Hướng làm:**

- A/B theo T6 của tài liệu Bitcraze.
  - TAG: đổi được lúc chạy qua settings/lệnh (`tx_power_mode` nằm trong blob settings).
  - Anchor: phải build lại (cờ build-time `UWB_TX_POWER_MODE`).
- Thứ tự A/B: `LEGACY` → `REFERENCE` → `SMART`; sau đó bật `UWB_DW_REFERENCE_TUNING=1`.
- Chỉ số: bias và σ ở 1 / 3 / 10 m, tỉ lệ exchange thành công ở cự ly xa nhất của vùng bay, phổ phát xạ nếu có máy.
- **Chốt profile xong rồi mới calibration (H3)**, vì mọi thay đổi RF đều làm dịch bias.

### 8.10 [V10] UART cho `RANGE_MEAS`

- `Firmware/Tag_DevKit/app.overlay:26`: đổi `current-speed = <921600>;`. VCOM J-Link của DWM1001-DEV đã chạy 1 Mbaud với `Sniffer_DevKit`. `telemetry.c:47-50` có `_Static_assert` kiểm ngân sách băng thông.
- `ESP32C3_Gateway/main/gateway_config.h:11`: đổi `#define GATEWAY_DWM_BAUD 921600`, và tăng buffer RX UART của ESP32-C3 cho phù hợp.
- GUI và `uwb_command.py`: tham số baud mặc định.
- Khi bay nên nối UART của TAG **thẳng vào UP 7000**, không qua ESP32 (mục 5.3 của tài liệu Bitcraze).

Nghiệm thu: `RANGE_MEAS` bật được mặc định; 10 phút không rớt frame (bộ đếm
parser); `uart_tx_high_water` < 50 %.

### 8.11 [V11] Nhánh C9.1 / C9.2

**Ngắn hạn** (để build `-Werror` với C9.2 ACTIVE):

```c
/* tag_ranging.c:661 */
#if UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_ACTIVE && \
    UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_ACTIVE
static uint8_t apply_outlier_gate(KalmanFilter_t* kf, double meas) { ... }
#endif
```

Và bọc `s_kf` / `s_kf_initialized` (dòng 507-508) cùng mọi chỗ dùng chúng trong
điều kiện `UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_ACTIVE`. Trình biên dịch sẽ
chỉ ra hết các chỗ đó.

**Dài hạn (nên làm):** khi §8.2 đã được nghiệm thu trên phần cứng, **xoá hẳn**
C9.1 và C9.2, gồm:

- `legacy_adaptive_tracking.h` (394 dòng);
- `motion_adaptive_range.h` (514 dòng);
- các nhánh trong `tag_ranging.c` và các cờ trong `uwb_calibration.h`.

Lý do: cả hai đều kém hơn C9 (§6.6), mà lại làm `tag_ranging.c` (2 868 dòng)
khó kiểm chứng.

### 8.12 [V12] Lỗ hổng test

1. **Unit test cho bộ lọc C9:** đã có trong bản vá (`tests/test_range_filter.c`).
2. **STM32 vào runner.** Thêm vào `run_host_tests.ps1`:

   ```powershell
   Invoke-Step -Name 'verify_stm32_images.py' -Body {
       Invoke-Python -Arguments @((Join-Path $firmwareRoot 'tools\verify_stm32_images.py'),
                                  '--source-only') `
                     -Message 'STM32 anchor source checks failed'
   }
   ```

   Và thêm một biến thể `test_anchor_state` với `Config = 'STM32_Anchor_5'`. Test này đã đạt khi chạy bằng tay trong phiên này.
3. **Test GUI chỉ chạy trên Windows** (`Software/UWB_UART_GUI/tests/test_gui_state.py:39`):

   ```python
   @unittest.skipUnless(os.name == "nt", "Windows drive-letter path semantics")
   def test_frozen_application_directory_is_executable_parent(self):
   ```

   Các test Tk thì bỏ qua khi không có display, hoặc chạy dưới `xvfb-run` trên CI Linux.
4. **Chạy độ bền định kỳ:** `test_tag_motion --seeds 20` (khoảng 8 s) trước mỗi lần phát hành.

### 8.13 Estimator trên host (UP 7000): sườn EKF dùng `corrected_mm`

Đây là điểm đến của kiến trúc (ADR-001, P7): **TAG chỉ đo, host ước lượng**.
Sườn Python dưới đây khớp đúng `telemetry_protocol.RangeMeasMessage` và
`AnchorInfoMessage` hiện có.

```python
"""3-D position/velocity EKF on the host, one range update per RANGE_MEAS.

Never feed filtered_mm: it is already time-correlated (double filtering).
Anchor positions come from AnchorInfoMessage.position_mm (TLV) or a survey.
"""
import numpy as np
from telemetry_protocol import MEAS_FLAG_CAL_OK, RangeMeasMessage


class RangeEkf:
    def __init__(self, accel_sigma_m_s2: float = 3.0):   # TUNE on flight logs
        self.x = None                                      # px py pz vx vy vz (m, m/s)
        self.P = np.diag([4.0, 4.0, 4.0, 1.0, 1.0, 1.0])
        self.q = accel_sigma_m_s2 ** 2
        self.t_us = None
        self.boot_id = None
        self.anchors: dict[int, np.ndarray] = {}

    def set_anchor(self, anchor_id: int, position_mm) -> None:
        self.anchors[anchor_id] = np.asarray(position_mm, dtype=float) / 1000.0

    def init_from_fix(self, position_m, t_us: int) -> None:
        """Seed from a multilateration fix (Gauss-Newton on >= 4 recent ranges)."""
        self.x = np.concatenate([np.asarray(position_m, dtype=float), np.zeros(3)])
        self.P = np.diag([0.25, 0.25, 0.25, 1.0, 1.0, 1.0])
        self.t_us = t_us

    def _predict(self, t_us: int) -> None:
        dt = (t_us - self.t_us) * 1e-6
        if dt <= 0.0:
            return
        F = np.eye(6)
        F[0:3, 3:6] = dt * np.eye(3)
        G = np.vstack([0.5 * dt * dt * np.eye(3), dt * np.eye(3)])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.q * (G @ G.T)
        self.t_us = t_us

    def sigma_m(self, m: RangeMeasMessage) -> float:
        sigma = 0.05                                   # TUNE: sigma(FPP) of §8.3
        nlos_db = m.nlos_indicator_db                  # rx - fp (APS006)
        if nlos_db is not None and nlos_db > 6.0:      # TUNE with T4
            sigma *= 1.0 + (nlos_db - 6.0)             # soft down-weighting
        return sigma

    def update(self, m: RangeMeasMessage, nis_gate: float = 9.0) -> bool:
        if self.boot_id is not None and m.boot_id != self.boot_id:
            self.x = None                              # TAG rebooted: restart
        self.boot_id = m.boot_id
        anchor = self.anchors.get(m.anchor_id)
        if anchor is None or not (m.flags & MEAS_FLAG_CAL_OK):
            return False
        if self.x is None:
            return False                               # call init_from_fix() first
        self._predict(m.meas_time_us)
        d = self.x[0:3] - anchor
        r = float(np.linalg.norm(d))
        if r < 1e-3:
            return False
        H = np.zeros((1, 6))
        H[0, 0:3] = d / r
        y = m.corrected_mm / 1000.0 - r
        S = (H @ self.P @ H.T).item() + self.sigma_m(m) ** 2
        if y * y / S > nis_gate:                       # 3-D gate: geometry sees NLOS
            return False
        K = (self.P @ H.T) / S
        self.x = self.x + K[:, 0] * y
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + (K @ K.T) * self.sigma_m(m) ** 2   # Joseph
        return True
```

Khởi tạo `x` bằng bình phương tối thiểu (Gauss-Newton) trên ≥ 4 range gần
nhau về thời gian, rồi gọi `init_from_fix()`. Sau đó xuất
`VISION_POSITION_ESTIMATE` / `ODOMETRY` sang PX4, kèm độ trễ đo được
(`EKF2_EV_DELAY`). `meas_time_us` được TAG lấy ở điểm giữa exchange
(`tag_ranging.h:397`).

Đã chạy thử sườn này trên dữ liệu tổng hợp `[MÔ HÌNH]`:

- phòng 8 anchor như `flight_8`, bay hình số 8;
- range σ 30 mm, 312 range/s.

Kết quả: sai số vị trí p50 22 mm, p95 38 mm. Không có cảnh báo NumPy nào.

Nghiệm thu: trên quỹ đạo biết trước (T9, §10), sai số vị trí p95 và độ trễ đo
so với ground truth.

### 8.14 Bù bias theo mức RX (H2): làm trên host trước

```python
"""Range bias vs received level, per board family (fitted from T3).
Values are placeholders: DO NOT use before T3 has been measured."""
import numpy as np

RX_DBM = np.array([-95.0, -90.0, -85.0, -80.0, -75.0])   # corrected RX level
BIAS_MM = {"dwm1001": np.zeros(5), "stm32": np.zeros(5)}  # TUNE_REQUIRED from T3

def corrected_for_rx_level(corrected_mm: int, rx_cdbm: int, family: str) -> float:
    rx = rx_cdbm / 100.0
    return corrected_mm - float(np.interp(rx, RX_DBM, BIAS_MM[family]))
```

Bảng phải gắn với profile radio (PHY, TX power, antenna delay). Đổi profile
thì phải đo lại. Chỉ đưa vào firmware khi bảng đã ổn định qua nhiều phiên đo.

---

## 9. Lộ trình và tiêu chí nghiệm thu

| Giai đoạn | Việc | Nghiệm thu |
|---|---|---|
| **G0: PC, 1–2 ngày** | §8.1 bản vá; §8.2 đổi mặc định sang `MEDIAN_GATE`; §8.12 test; §8.8 `.gitattributes` + chặn build bẩn; §8.7 test ID triển khai + cô lập `Anchor_5..8` nRF; §8.10 UART 921600 | Runner xanh trên Windows. `test_tag_motion --seeds 20` đạt 14/14. Image có dirty = 0 và hash nhúng bằng tag. Hash manifest khớp trên clone mới |
| **G1: bàn thử, đo** | §8.5 TLV sức khỏe + biên TX; T1 (sniffer timeline); T2 (σ tĩnh từng anchor, ≥ 1 000 mẫu mỗi điểm) | Biên nhỏ nhất ≥ SHR + 150 µs trên cả 8 anchor trong 10 phút; `delayed_tx_late` = 0; biết σ(FPP) từng họ board |
| **G2: RF A/B + calibration** | §8.9 (T6, reference tuning); §8.4 TLV danh tính + settings v3 + CAL_BIND; H3 calibration từng cặp | Đạt `HARDWARE_AB_CHECKLIST.md`. Calibration gắn với (TAG part, anchor part, profile). Thử đổi board thì ra `CAL_MISSING` |
| **G3: timing** | §8.6 A → B (theo biên G1) → A/B cho C | Mỗi anchor ≥ 45 Hz, overrun < 1 %, σ không xấu hơn baseline (A/B ≥ 1 000 mẫu) |
| **G4: estimator** | §8.13 EKF trên UP 7000; H4 chỉ số NLOS; §8.14 bảng bias theo RX; §8.3 R theo σ(FPP) | Quỹ đạo biết trước: p95 vị trí và độ trễ đạt mục tiêu bay (chủ dự án đặt) |
| **G5: dọn mã** | §8.11 xoá C9.1/C9.2; tách `tag_ranging.c` (lịch, exchange, tính toán, publish) | Runner xanh; motion 14/14; `tag_ranging.c` ngắn hơn khoảng 40 % |

Không đưa range vào vòng điều khiển bay tự động trước khi xong G2. Điều kiện
này giống điều kiện kết thúc của bản rà soát 23/9.

---

## 10. Thí nghiệm phần cứng cần làm

Dùng lại T1–T8 của tài liệu Bitcraze, bổ sung các điểm sau:

| ID | Thí nghiệm | Chỉ số | Xác nhận cho |
|---|---|---|---|
| T1+ | Sniffer + TLV sức khỏe, 10 phút, 8 anchor | p50/p99 slot và chu kỳ; biên TX nhỏ nhất theo từng anchor | §7.2, §7.3 (mô hình 26 ms, 39 Hz), §8.5 |
| T2+ | Tĩnh ở 1 / 3 / 5 / 10 m, từng anchor, ≥ 1 000 mẫu | σ theo FPP (đã sửa); bias thô | §8.2 (chọn bộ lọc), §8.3 (bảng σ) |
| T9 | **Bay/di chuyển có ground truth** (camera, motion capture, hoặc ray trượt có encoder) ở 0,5 / 1 / 2 m/s; A/B Legacy ↔ MEDIAN_GATE + vá | Tỉ lệ hợp lệ, p95, trễ, FA | V01, V02: xác nhận §6.4 trên phần cứng |
| T10 | Vật cản người/kim loại chắn 1–3 s trong lúc di chuyển; rút rồi cắm lại 1 anchor | Thời gian hồi phục; FA sau khi mất tín hiệu | V02, §8.1 (`nlos_3s`, `reacq_nlos`) |
| T11 | Đổi hai board A5↔A6 (giữ nguyên ID) | Range phải chuyển thành `CAL_MISSING` | §8.4 |
| T12 | Nạp đồng thời 2 anchor cùng ID (chỉ trên bàn thử) | `identity_changes`, range hai đỉnh | §8.7 |

---

## 11. Rủi ro còn lại và câu hỏi mở

1. **Mô hình NLOS và nhiễu mới là giả định.** §6.7–§6.8 cho thấy kết luận về LEGACY và tái bắt khoá không đổi khi σ đi từ 30 tới 100 mm và FPP từ −62 tới −90 dBm @1 m. Nhưng NLOS thật có phân bố đuôi dài, và có cả lỗi phát hiện first path sớm (early FP). T9/T10 sẽ trả lời.
2. **Latency CPU thật của A5–A8** là con số quyết định mọi thay đổi timing (§7.3). Chưa có số đo.
3. **Ngưỡng NLOS dựa trên `rx − fp`** (H4) cho board STM32 + DW1000 có thể khác DWM1001, do anten và đường RF khác nhau.
4. **Ở 60 % NLOS kéo dài**, không bộ lọc 1D nào đủ (§6.5). Phải có estimator 3D và chỉ số NLOS.
5. Câu hỏi cho chủ dự án:
   - vận tốc và gia tốc tối đa khi bay;
   - p95 vị trí mục tiêu;
   - tần số cập nhật tối thiểu mà PX4 cần;
   - có dùng MAVLink trực tiếp từ TAG (không qua host) hay không.

   Các con số này quyết định cổng nghiệm thu ở §6.3 và lựa chọn ở §8.2 / §8.6.

---

## Phụ lục A. Lệnh tái lập

**Toàn bộ host test** (Windows, từ gốc repo):

```powershell
powershell -ExecutionPolicy Bypass -File Firmware\tests\run_host_tests.ps1
powershell -ExecutionPolicy Bypass -File Firmware\tests\run_host_tests.ps1 -Filter 'test_tag_motion*'
```

**Test chuyển động chạy riêng** (MinGW hoặc Linux; thay `1U` bằng `0U` cho Legacy, `2U` cho CV):

```bash
gcc -std=c11 -Wall -Wextra -Werror -Wshadow -O1 -DUWB_ROLE_TAG -DUWB_RANGE_FILTER_MODE=1U \
    -I Firmware/common/include -I Firmware/tests/motion_cfg -I Firmware/tests \
    Firmware/tests/test_tag_motion.c Firmware/tests/dw1000_sim.c \
    Firmware/common/src/drivers/dw1000.c Firmware/common/src/ranging/uwb_frame.c \
    Firmware/common/src/filters/range_filter.c -lm -o motion
./motion                 # seed 0, strict
./motion --seeds 20      # độ bền
./motion --report        # chỉ in, luôn exit 0
```

**Các núm mô hình** (thêm `-D`):

| Cờ | Ý nghĩa | Ví dụ |
|---|---|---|
| `MOTION_PREAMBLE_SYMBOLS` | Preamble dùng để tính airtime | `128` |
| `MOTION_REPLY_DELAY_UUS` | Reply delay của anchor | `900` |
| `MOTION_TAG_RESP_US` | CPU của TAG từ RESP tới FINAL | `150.0` |
| `MOTION_TS_NOISE_TICKS` | Nhiễu timestamp: 10,4 cho σ 30 mm; 20,8 cho 60 mm; 34,7 cho 100 mm | `20.8` |
| `UWB_FILTER_FPP_COMPAT_DB` | Độ bù FPP của bộ lọc | `0.0f` |
| `MOTION_LEGACY_ADAPTIVE_MODE` | C9.1: 1 = shadow, 2 = active (kèm `-DUWB_RANGE_FILTER_MODE=0U`, vì C9.1 chỉ chạy trên đường Legacy) | `2U` |
| `UWB_C9_2_MOTION_MODE` | C9.2 (kèm `-DUWB_RANGE_FILTER_MODE=0U`, cùng `-Wno-error=unused-function -Wno-error=unused-variable`) | `2U` |

**Áp và kiểm bản vá:**

```bash
git apply --check Plan/patches/0001-range-filter-nlos-aware-reacquire.patch
git apply Plan/patches/0001-range-filter-nlos-aware-reacquire.patch
```

Trên Windows đang bật `core.autocrlf=true`, nếu `git apply` báo lệch khoảng
trắng hoặc xuống dòng thì thêm `--ignore-whitespace`. Các dòng chỉ gồm một dấu
cách trong file `.patch` là dòng ngữ cảnh rỗng của định dạng diff. Không được
xoá chúng.

**Sanitizer** (ví dụ):

```bash
gcc ... -fsanitize=address,undefined -fno-sanitize-recover=all -g -O1 ...
```

**Build Zephyr:** dùng `Firmware/scripts/build_all.ps1` và
`build_stm32_anchors.ps1` như bình thường. Muốn build TAG ở chế độ C9 mà không
sửa file:

```powershell
west build -b decawave_dwm1001_dev/nrf52832 Firmware/Tag_DevKit -- -DEXTRA_CFLAGS=-DUWB_RANGE_FILTER_MODE=1U
```

**Kiểm hash dist:**

```bash
python3 Firmware/tools/verify_stm32_images.py
sha256sum Firmware/anchor_8_dist/*.hex
```

---

## Phụ lục B. Bảng kết quả đầy đủ

Seed 0, mô hình mặc định: preamble 256, reply 1 200 UUS, σ ≈ 30 mm, FPP −62 dBm
@1 m, độ bù FPP −12,04 dB. Đơn vị mm và ms; `rec` = −1 nghĩa là không hồi phục.

**LEGACY (mặc định hiện tại)**

| Kịch bản | avail | mean | p95 | max | lag | FA | rec | raw b/sd | Kết quả |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| static_3m | 100,0 % | 20 | 45 | 50 | – | 0 | – | −0/30 | đạt |
| drift_0.1 | 77,5 % | 287 | 393 | 412 | 2 874 | 91 | – | −1/29 | TRƯỢT |
| walk_0.5 | 18,8 % | 1 073 | 2 835 | 3 239 | 2 146 | 59 | – | −2/30 | TRƯỢT |
| run_1.0 | 15,4 % | 1 283 | 3 010 | 3 289 | 1 283 | 53 | – | 1/31 | TRƯỢT |
| fast_2.0 | 13,5 % | 1 508 | 3 191 | 3 538 | 754 | 27 | – | −3/31 | TRƯỢT |
| sine_1m_4s | 48,8 % | 445 | 970 | 1 019 | – | 116 | – | 3/31 | TRƯỢT |
| spikes_hold | 98,4 % | 2 | 6 | 7 | – | 0 | – | −1/29 | đạt |
| spikes_walk | 20,0 % | 1 024 | 2 649 | 3 170 | 2 047 | 60 | – | −1/31 | TRƯỢT |
| dropout_1s | 6,9 % | 1 304 | 2 997 | 3 289 | 1 304 | 24 | −1 | 1/29 | TRƯỢT |
| nlos_3s | 85,1 % | 48 | 136 | 150 | – | 0 | 1 042 | 0/29 | TRƯỢT |
| wrap_drift | 17,5 % | 1 182 | 2 580 | 3 087 | 1 182 | 47 | – | −3/30 | TRƯỢT |
| reacq_nlos | 7,4 % | 1 164 | 3 067 | 3 358 | 1 164 | 25 | −1 | −1/31 | TRƯỢT (known) |
| flight_8 | 36,1 % | 687 | 2 610 | 3 710 | – | 1 351 | – | 1/30 | TRƯỢT |
| weak_gap | 99,4 % | 32 | 39 | 39 | – | 0 | 42 | −3/31 | đạt |

**MEDIAN_GATE (mã hiện tại / + bản vá)**

| Kịch bản | avail | p95 | max | lag | FA | rec | Mã hiện tại | + vá |
|---|---:|---:|---:|---:|---:|---:|---|---|
| static_3m | 100,0 % | 40 | 58 | – | 0 | – | đạt | đạt |
| drift_0.1 | 100,0 % | 40 | 65 | 31 | 0 | – | đạt | đạt |
| walk_0.5 | 100,0 % | 49 | 69 | 23 | 0 | – | đạt | đạt |
| run_1.0 | 100,0 % | 56 | 83 | 19 | 0 | – | đạt | đạt |
| fast_2.0 | 100,0 % | 85 | 101 | 21 | 0 | – | đạt | đạt |
| sine_1m_4s | 100,0 % | 60 | 99 | – | 0 | – | đạt | đạt |
| spikes_hold | 98,4 % | 43 | 62 | – | 0 | – | đạt | đạt |
| spikes_walk | 98,0 % | 50 | 83 | 11 | 0 | – | đạt | đạt |
| dropout_1s | 99,4 % / 98,6 % | 55 | 78 | 20 | 0 | 42 / 102 | đạt | đạt |
| nlos_3s | 99,4 % | 39 | 48 | – | 0 | 42 | đạt | đạt |
| wrap_drift | 100,0 % | 57 | 90 | 22 | 0 | – | đạt | đạt |
| reacq_nlos | 97,4 % / 97,7 % | 55 | 70 | 21 | **3 / 0** | 162 / 142 | known | **đạt** |
| flight_8 | 99,8 % | 57 | 115 | – | 0 | – | đạt | đạt |
| weak_gap | 99,4 % / 98,6 % | 42 | 74 | – | 0 | 42 / 102 | đạt | đạt |

**CV_KALMAN_V2 (mã hiện tại / + bản vá / + bản vá + bù FPP = 0)**

| Kịch bản | avail | p95 | FA | rec | Kết quả |
|---|---:|---:|---:|---:|---|
| static_3m | 100 % | 25 / 25 / 33 | 0 | – | đạt |
| drift_0.1 | 100 % | 23 / 23 / 31 | 0 | – | đạt |
| walk_0.5 | 100 % | 36 / 36 / 39 | 0 | – | đạt |
| run_1.0 | 100 % | 37 / 37 / 39 | 0 | – | đạt |
| fast_2.0 | 100 % | 65 / 65 / 67 | 0 | – | đạt |
| sine_1m_4s | 100 % | 74 / 74 / 59 | 0 | – | đạt |
| spikes_hold | 98,4 % | 26 / 26 / 32 | 0 | – | đạt |
| spikes_walk | 98,0 % | 29 / 29 / 34 | 0 | – | đạt |
| dropout_1s | 99,4 / 98,6 / 98,6 % | 43 / 44 / 43 | 0 | 42 / 102 / 102 | đạt |
| nlos_3s | 98,3 / 98,3 / 99,1 % | 39 / 39 / 31 | 0 | 122 / 122 / 62 | đạt (seed 0) |
| wrap_drift | 100 % | 38 / 38 / 41 | 0 | – | đạt |
| reacq_nlos | 97,1 / 97,7 / 97,7 % | 103 / 41 / 43 | **3** / 0 / 0 | 162 / 142 / 142 | known / **đạt** / **đạt** |
| flight_8 | 99,8 % | 47 / 47 / 50 | 0 | – | đạt |
| weak_gap | 99,4 / 98,6 / 98,6 % | 28 / 27 / 27 | 0 | 42 / 102 / 102 | đạt |

---

## Phụ lục C. File trong nhánh này

| File | Nội dung |
|---|---|
| `Firmware/tests/test_tag_motion.c` | Test chuyển động end-to-end: 14 kịch bản, `--report`, `--seeds N` |
| `Firmware/tests/motion_cfg/uwb_app_config.h` | Dùng đúng cấu hình TAG DevKit; cho phép bật C9.1 để khảo sát |
| `Firmware/tests/run_host_tests.ps1` | Hỗ trợ khoá `Source`, `IncludeDir`, `Defines`; thêm 3 bước motion (Legacy chỉ ghi kết quả; MEDIAN_GATE và CV bắt buộc) |
| `Plan/patches/0001-range-filter-nlos-aware-reacquire.patch` | Bản vá §8.1 kèm unit test. **Đã áp** trên nhánh này; giữ file để đối chiếu, không `git apply` lại |
| `Plan/BAO_CAO_KIEM_TRA_FIRMWARE_V2_2026-09-24.md` | Báo cáo này |
