# Đánh giá so sánh với Bitcraze Crazyflie và hướng giảm sai số UWB

- **Ngày:** 2026-09-24
- **Người viết:** Claude (Opus 5.5), theo yêu cầu của chủ dự án
- **Mục đích:** ghi lại các vấn đề tìm ra khi so sánh firmware DWM1001 với
  `bitcraze/crazyflie-firmware`, và hướng giải quyết để **giảm sai số tối đa**.
  Tài liệu viết để một model/reviewer khác kiểm chứng lại từng khẳng định.
- **Phạm vi mã đã đọc:**
  - Repo này: nhánh `feature/firmware-upgrade-v2` (HEAD `08d2818`, có thay đổi
    chưa commit), thư mục `Firmware/common`, `Software/UWB_UART_GUI`, `Plan/`.
  - Bitcraze: `crazyflie-firmware` nhánh `master`, commit
    `16342433a0b17788321463fafbd1c950dfe71705`. Các file đã đọc:
    `src/deck/drivers/src/locodeck.c`, `lpsTwrTag.c`, `lpsTdoa2Tag.c`,
    `lpsTdoa3Tag.c`, `src/deck/drivers/interface/locodeck.h`, `lpsTwrTag.h`,
    `src/utils/src/tdoa/tdoaEngine.c`, `tdoaStorage.c`,
    `src/modules/src/kalman_core/mm_distance.c`, `mm_distance_robust.c`,
    `mm_tdoa.c`, `mm_tdoa_robust.c`,
    `src/modules/src/outlierfilter/outlierFilterTdoa.c`.
  - **Chưa đọc:** thư viện `libdw1000` (định nghĩa `MODE_SHORTDATA_*`) và repo
    anchor `lps-node-firmware`. Kế hoạch `KE_HOACH_NANG_CAP_TOAN_DIEN_FIRMWARE_DWM1001.md`
    đã so sánh riêng với repo anchor này.

## 0. Quy ước nhãn cho reviewer

Mỗi khẳng định có một nhãn:

| Nhãn | Ý nghĩa | Cách kiểm chứng |
|---|---|---|
| `[CODE]` | Đã đọc trực tiếp trong mã, có trích file:dòng | Mở đúng file:dòng |
| `[ƯỚC TÍNH]` | Tính toán giấy bút, chưa đo | Kiểm lại công thức, rồi đo bằng sniffer hoặc bench |
| `[NGOÀI]` | Kiến thức từ tài liệu Decawave/Qorvo hoặc tài liệu khác, **không** mở tài liệu trong phiên này | Đối chiếu tài liệu gốc (APS011, APS006, APS014, DW1000 User Manual) |
| `[GIẢ THUYẾT]` | Nhận định kỹ thuật cần thí nghiệm để xác nhận | Làm thí nghiệm A/B ghi trong mục 6 |

Đường dẫn trong tài liệu tính tương đối từ thư mục `UWB DW1001/`.

---

## 1. Tóm tắt kết luận

1. Nếu dự án **chỉ có một drone** và mục tiêu là **sai số nhỏ nhất**, **nên giữ
   DS-TWR**. Không nên quay về SS-TWR, cũng không nên chuyển sang TDoA3.
   `[GIẢ THUYẾT]`, lập luận ở mục 4.
2. Vấn đề 8 anchor không vừa chu kỳ 20 ms giải được mà **vẫn giữ DS-TWR**. Có hai
   cách: chia 2 nhóm anchor, hoặc làm DS-TWR một-nhiều (GĐ5 trong kế hoạch
   toàn diện). Xem mục 5.1.
3. Các nguồn sai số **lớn hơn chuyện chọn kiến trúc** gồm:
   - vị trí anchor đo sai,
   - bias thay đổi theo mức tín hiệu RX (**chưa có bù trong code**),
   - antenna delay riêng từng board,
   - hướng anten,
   - NLOS và đa đường,
   - hình học đặt anchor.

   Xem mục 3 và 5.2.
4. TDoA3 chỉ đáng làm khi cần **nhiều drone** bay cùng lúc.

---

## 2. So sánh với Crazyflie: các điểm đã xác minh

### 2.1 Crazyflie (phía TAG và estimator)

| # | Khẳng định | Nhãn | Bằng chứng |
|---|---|---|---|
| B1 | Hỗ trợ TWR, TDoA2, TDoA3 và chế độ tự dò (`lpsMode_auto`) | `[CODE]` | `locodeck.c:98-120, 304-401` |
| B2 | PHY: Channel 2, preamble code 9 (PRF 64 MHz), `MODE_SHORTDATA_FAST_ACCURACY` (hoặc `MID_ACCURACY` khi bật `CONFIG_DECK_LOCO_LONGER_RANGE`), smart power bật theo mặc định | `[CODE]` | `locodeck.c:592-606` |
| B2a | `FAST_ACCURACY` = 6.8 Mbps, preamble 128 | `[NGOÀI]` **chưa kiểm** | Cần mở `libdw1000` |
| B3 | Antenna delay là **một hằng số chung**: `LOCODECK_ANTENNA_OFFSET = 154.6 m`, đổi ra tick. Bù trong phần mềm bằng `±antennaDelay/2` trên timestamp TX/RX, còn thanh ghi antenna delay của chip đặt về 0 | `[CODE]` | `locodeck.h:47-48`, `lpsTwrTag.c:145, 207`, `locodeck.c:580-581` |
| B4 | DS-TWR dùng công thức bất đối xứng 4 số hạng trên `low32` | `[CODE]` | `lpsTwrTag.c:230-238` |
| B5 | Lọc outlier TWR: lịch sử 32 mẫu, loại nếu lệch khỏi trung bình quá 4σ. σ đo đưa vào EKF **cố định 0,25 m** | `[CODE]` | `lpsTwrTag.c:102-103, 240-262` |
| B6 | Khoảng cách chỉ đưa vào EKF khi đã biết vị trí anchor. Vị trí anchor có thể nhận qua LPP | `[CODE]` | `lpsTwrTag.c:254-262, 446-452` |
| B7 | Nhiều tag TWR dùng TDMA, slot suy ra từ địa chỉ radio | `[CODE]` | `lpsTwrTag.c:293-311, 461-465` |
| B8 | Engine TDoA có bù clock, ghép cặp anchor ngẫu nhiên hoặc mới nhất, và loại mẫu khi `|Δd| / khoảng cách giữa 2 anchor` vượt ngưỡng | `[CODE]` | `tdoaEngine.c` (hàm `updateClockCorrection`, `isGeometryGoodEnough`, `findSuitableAnchor`) |
| B9 | Lọc outlier TDoA kiểu integrator: bộ lọc tự mở cho mọi mẫu đi qua khi EKF có thể đã phân kỳ, rồi đóng lại khi hội tụ | `[CODE]` | `outlierFilterTdoa.c` (hằng `INTEGRATOR_FORCE_OPEN_LEVEL`, `INTEGRATOR_RESUME_ACTION_LEVEL`) |
| B10 | Có bản cập nhật robust cho EKF dùng trọng số Geman-McClure (`GM_UWB`, sigma = 1.5) | `[CODE]` | `mm_distance_robust.c:54-55` |
| B11 | TDoA3 có chế độ hybrid: TAG chen thêm TWR, có `twrStdDev` riêng | `[CODE]` | `lpsTdoa3Tag.c:87, 162, 186, 279, 328-359` |
| B12 | Không dùng FPP hoặc CIR để đánh trọng số hay phát hiện NLOS trong luồng TWR | `[CODE]` (theo các file đã đọc) | `lpsTwrTag.c` không đọc chẩn đoán tín hiệu |

### 2.2 Firmware DWM1001 (repo này)

| # | Khẳng định | Nhãn | Bằng chứng |
|---|---|---|---|
| U1 | DS-TWR 4 số hạng dùng timestamp **40-bit** (mặt nạ `0xFFFFFFFFFF`) | `[CODE]` | `Firmware/common/src/ranging/tag_ranging.c:1796-1830` |
| U2 | PHY: Ch5, PRF16, preamble 256, PAC16, 6.8 Mbps, SFD chuẩn | `[CODE]` | `Firmware/common/include/dw1000_hw.h:204-208`, `Firmware/common/src/drivers/dw1000.c:6-7` |
| U3 | TX power mặc định là `UWB_TX_POWER_LEGACY` = `0x1E1E1E1E`. Có sẵn profile `REFERENCE` (`0x48484848`) và `SMART` | `[CODE]` | `Firmware/common/include/uwb_calibration.h:340-341`, `Firmware/common/src/drivers/dw1000.c:50-52` |
| U4 | Antenna delay TX/RX riêng (mặc định 16436), ghi vào thanh ghi chip; có thêm bias DS riêng từng anchor | `[CODE]` | `Firmware/common/include/uwb_calibration.h:314-318`, `tag_ranging.c:141` (`ds_calibration_offset_for_index`), `tag_ranging.c:2427` (`Tag_SetDsCalibration`) |
| U5 | Mỗi phép đo có đủ chẩn đoán: `raw_mm`, `corrected_mm`, `filtered_mm`, `fp_cdbm`, `rx_cdbm`, `anchor_fp_cdbm`, `anchor_rx_cdbm`, `std_noise`, `fp_index`, `ci_ppm_x100`, `slot_us` | `[CODE]` | `Firmware/common/include/tag_ranging.h:396-415` |
| U6 | Anchor gửi vị trí của mình qua sóng (TLV trong RESP v2), TAG gom vào `TagAnchorInfo_t.pos_mm` | `[CODE]` | `Firmware/common/include/uwb_frame.h:75, 79`, `tag_ranging.h:421-436` |
| U7 | SS-TWR và SS fallback có bù lệch clock bằng carrier integrator, làm mượt bằng EMA 0,95/0,05 | `[CODE]` | `tag_ranging.c:1403-1456`, `uwb_calibration.h:381` |
| U8 | Bộ lọc trong firmware: median3, Kalman 1D, gate NIS 6.635, R theo FPP, reacquire | `[CODE]` | `Firmware/common/src/filters/range_filter.c:10-34` |
| U9 | Bộ lọc host trong GUI lấy **`raw_mm`** làm đầu vào, không lọc lại `filtered_mm` | `[CODE]` | `Software/UWB_UART_GUI/uwb_uart_gui.py:891-896` |
| U10 | Thời gian đo: reply delay của anchor 1200 µs, RESP timeout 2600 µs, guard 150 µs, recovery cuối chu kỳ 500 µs, REPORT timeout 3000 µs, chu kỳ đích 20 ms | `[CODE]` | `Firmware/common/include/anchor_ranging.h:32`, `tag_ranging.h:37-79` |
| U11 | `tag_ranging.c` dài 2868 dòng. Hai nhánh `LEGACY_ADAPTIVE` và `C9_2_MOTION` **mặc định tắt** nhưng vẫn nằm trong mã | `[CODE]` | `wc -l`, `uwb_calibration.h:202, 250` |
| U12 | **Chưa có** bù bias theo mức tín hiệu RX (range bias vs RSL) | `[CODE]` | `grep -i "range.bias\|bias_table\|APS011"` trên `Firmware/common` không có kết quả |
| U13 | **Chưa có** cờ hoặc bộ phát hiện NLOS từ chênh lệch RX power và FPP | `[CODE]` | Tìm trong `tag_ranging.c` và `range_filter.c`: FPP chỉ dùng để chọn R |
| U14 | Estimator vị trí trên UP 7000 (IEKF) **mới là kế hoạch, chưa có mã** | `[CODE]` | `Plan/KE_HOACH_TRIEN_KHAI_UWB_DRONE_UP7000_PIXHAWK6C.md:465`. Trong repo chỉ có `Software/UWB_UART_GUI/host_range_filter.py`, là bộ lọc 1D |
| U15 | Chưa có kết quả đo A/B trên phần cứng cho các thay đổi RF của GĐ0–GĐ4 | Theo tài liệu | `Firmware/HARDWARE_AB_CHECKLIST.md`, `Firmware/CHANGELOG.md` |

### 2.3 Đính chính các nhận định sai trong trao đổi trước

Reviewer không cần kiểm lại hai điểm này, chỉ ghi lại cho minh bạch:

- Trong trao đổi đầu tiên, trợ lý ghi *"vị trí anchor chỉ cấu hình trên host"*.
  **Sai.** U6 cho thấy anchor đã gửi vị trí qua TLV.
- Trợ lý từng cảnh báo *"lọc hai lần"* như một lỗi đang tồn tại. **Hiện tại không
  đúng**, vì GUI lấy `raw_mm` (U9). Đây chỉ là rủi ro cần tránh khi viết IEKF (mục 5.3).

---

## 3. Danh sách vấn đề

Xếp theo mức ảnh hưởng tới sai số vị trí cuối cùng. Thứ tự là nhận định
`[GIẢ THUYẾT]`: chưa có số đo để xếp hạng chắc chắn.

| ID | Vấn đề | Ảnh hưởng | Nhãn |
|---|---|---|---|
| P1 | Vị trí anchor nhập tay, chưa có quy trình đo chính xác hay tự hiệu chỉnh | Sai số vị trí anchor đi thẳng vào sai số vị trí drone, **không lọc được** | `[GIẢ THUYẾT]` |
| P2 | Chưa bù bias theo mức RX (U12) | DW1000 có bias phụ thuộc mức tín hiệu thu, cỡ vài cm tới hơn 10 cm trên dải công suất. Calibration ở một cự ly không bù được cho cự ly khác | `[NGOÀI]`, APS011 |
| P3 | Antenna delay và bias từng anchor chưa được calibration trên phần cứng (U4, U15). A5–A8 là STM32 + DW1000 với anten khác DWM1001 | Bias cố định từng cặp TAG-anchor | `[CODE]` và `[GIẢ THUYẾT]` |
| P4 | Chưa phát hiện NLOS (U13) | Mẫu đi đường vòng luôn **dài hơn** thật, và bộ lọc 1D không phân biệt được với chuyển động thật | `[NGOÀI]`, APS006 |
| P5 | Delay của anten chip DWM1001 thay đổi theo góc tới | Sai số phụ thuộc tư thế drone | `[NGOÀI]`, cần đo |
| P6 | 8 anchor đo DS-TWR lần lượt, ước tính 25–33 ms, vượt chu kỳ 20 ms | Mỗi anchor cập nhật chậm hơn, ít mẫu hơn để lấy trung bình | `[ƯỚC TÍNH]`, từ `Plan/KE_HOACH_NANG_CAP_TOAN_DIEN_FIRMWARE_DWM1001.md:48`, chưa đo sniffer |
| P7 | Chưa có estimator 3D (U14) | Không loại outlier theo hình học, không hợp nhất IMU | `[CODE]` |
| P8 | TX power mặc định `0x1E1E1E1E`, smart power tắt (U3) | Có thể làm bão hòa bộ thu ở cự ly gần, và đổi TX power sẽ làm lệch bias | `[GIẢ THUYẾT]` |
| P9 | Mã phức tạp (U11) | Rủi ro bảo trì, khó kiểm chứng tính đúng | `[CODE]` |
| P10 | Chuỗi trễ UART → ESP32 → PC/UP → MAVLink | Trễ và jitter đưa vào EKF2 | `[GIẢ THUYẾT]` |
| P11 | Mỗi lúc chỉ một TAG | Không mở rộng sang nhiều drone. **Không** ảnh hưởng sai số của một drone | `[CODE]` |

---

## 4. Vì sao không chuyển sang SS-TWR hay TDoA3 để giảm sai số

### 4.1 SS-TWR

Sai số ToF do lệch clock ở SS-TWR xấp xỉ `½ · T_reply · e`, với `e` là độ lệch
tần số giữa hai clock. `[NGOÀI]`, công thức chuẩn.

- Với `T_reply = 1,2 ms` (U10) và `e = 10 ppm`: `0,5 × 1,2e-3 × 10e-6 = 6 ns`,
  tức khoảng **1,8 m**. `[ƯỚC TÍNH]`
- Nếu carrier integrator bù còn dư khoảng 0,1 ppm: khoảng **1,8 cm**.
  `[ƯỚC TÍNH]`. Mức dư 0,1 ppm là giả định, chưa đo.
- DS-TWR bất đối xứng triệt tiêu bậc nhất của lệch clock. Phần dư tỉ lệ với
  `ToF × e`, không đáng kể. `[NGOÀI]`

Kết luận: SS-TWR có bù CI dùng được khi cần nhanh, nhưng **không thể chính xác
hơn** DS-TWR. Chủ dự án cũng đã xác nhận SS-TWR là kiến trúc cũ.

### 4.2 TDoA3

`[GIẢ THUYẾT]`, dựa trên cấu trúc thuật toán ở B8 và B9:

1. Mỗi giá trị TDoA cộng dồn nhiều nguồn nhiễu hơn một range DS-TWR:
   - hai timestamp RX tại TAG,
   - hệ số bù clock ước lượng,
   - ToF giữa hai anchor (bản thân là một phép đo),
   - antenna delay TX của anchor này và RX của anchor kia.
2. Định vị theo hyperbol thường có GDOP lớn hơn định vị theo mặt cầu, nhất là
   trục Z và ở ngoài vùng bao các anchor.
3. Phải viết lại toàn bộ firmware anchor, trên cả DWM1001 lẫn STM32: anchor
   phải tự phát định kỳ và đo lẫn nhau. Phải viết engine TDoA trên host và
   làm lại calibration. Kế hoạch `Plan/KE_HOACH_NANG_CAP_FIRMWARE_DW1001.md`
   mục P4 cũng xếp đây là mức rủi ro cao nhất và chỉ dành cho nhu cầu nhiều drone.
4. Giấy phép: `crazyflie-firmware` là GPL-3.0. Chép mã vào firmware rồi phân phối
   thì firmware cũng phải theo GPL. `[NGOÀI]`, cần xác nhận file LICENSE.

TDoA3 vẫn phù hợp khi cần nhiều drone, cần TAG tiết kiệm pin, hoặc cần rất nhiều
anchor. Khi đó có thể cân nhắc chế độ lai giống B11.

**Thí nghiệm để bác bỏ hay xác nhận mục 4.2:** xem T7 ở mục 6.

---

## 5. Hướng giải quyết

### 5.1 Ngân sách thời gian 8 anchor mà vẫn giữ DS-TWR (P6)

Ước tính thời gian phát một frame (`[ƯỚC TÍNH]`, `[NGOÀI]`: độ dài symbol
lấy theo DW1000 User Manual, cần kiểm):

- Preamble 256 symbol × khoảng 993,6 ns ≈ 254 µs
- SFD 8 symbol ≈ 8 µs
- PHR 21 bit ở 850 kbps ≈ 21,5 µs
- Payload khoảng 20 byte ở 6,8 Mbps, tính cả Reed-Solomon ≈ 27 µs
- **Tổng khoảng 310 µs mỗi frame.** Preamble 128 tiết kiệm khoảng 127 µs mỗi frame.

| Phương án | Mô tả | Ước tính chu kỳ | Ghi chú |
|---|---|---|---|
| S1 | Chia 2 nhóm 4 anchor không đồng phẳng, mỗi nhóm 25 Hz | 13–16 ms mỗi nhóm | Vẫn DS-TWR. Estimator phải xử lý range theo đúng thời điểm đo (`slot_us` ở U5). Ý tưởng đã có ở kế hoạch UP7000 dòng 564-575 |
| S2 | DS-TWR một-nhiều: POLL/FINAL broadcast, RESP/REPORT theo slot, FINAL mang bitmap các anchor đã nhận RESP | 9–15 ms | GĐ5, `Plan/KE_HOACH_NANG_CAP_TOAN_DIEN_FIRMWARE_DWM1001.md:358-366, 502`. Mỗi giây có gấp khoảng 2 lần số range |
| S3 | Giảm `ANCHOR_REPLY_DELAY_UUS` từ 1200 xuống 600–800 µs | Tiết kiệm 3–5 ms | Phụ thuộc thời gian SPI/ISR trên cả nRF52 lẫn STM32. **Đo bằng sniffer trước khi đổi** |
| S4 | Preamble 256 → 128 | Tiết kiệm khoảng 4 ms | **Có thể làm tăng sai số** và giảm tầm. Chỉ làm khi A/B cho thấy σ không tăng |

Nên làm **S1 trước**, sau đó S2. S3 và S4 chỉ làm khi có dữ liệu sniffer và A/B.

### 5.2 Giảm sai số đo (P1–P5, P8)

| ID | Việc cần làm | Chi tiết | Liên quan |
|---|---|---|---|
| H1 | Đo vị trí anchor chính xác | Dùng máy toàn đạc hoặc laser. Hoặc thêm chế độ anchor đo DS-TWR lẫn nhau, rồi giải bình phương tối thiểu hoặc MDS để tự hiệu chỉnh; cần tối thiểu một số điểm mốc đã biết để cố định hệ trục | P1 |
| H2 | Bù bias theo mức RX | Đo tĩnh ở nhiều cự ly và mức công suất cho từng profile PHY/TX power, lập bảng `bias(rx_cdbm)` rồi nội suy. **Làm trên host trước** (dữ liệu `rx_cdbm` đã có ở U5), ổn định rồi mới đưa vào firmware. Bảng gắn với identity profile radio (`dw1000_hw.h:351`) | P2, P8 |
| H3 | Calibration antenna delay và bias từng anchor | Làm theo `Firmware/HARDWARE_AB_CHECKLIST.md`: A/B RF xong mới calibration chính thức. Calibration riêng A1–A4 (DWM1001) và A5–A8 (STM32 + DW1000) | P3 |
| H4 | Phát hiện NLOS | Chỉ số `Δ = rx_cdbm − fp_cdbm`. Ngưỡng khởi điểm để tinh chỉnh: Δ < 6 dB coi là LOS, Δ > 10 dB nhiều khả năng NLOS (`[NGOÀI]`, APS006, **cần đối chiếu**). Dùng Δ để tăng R hoặc loại mẫu, và xuất cờ ra telemetry | P4 |
| H5 | Hướng anten | Gắn anchor và TAG theo hướng cố định, ghi rõ trong `ANCHOR_DEPLOYMENT.md`. Calibration đúng tư thế lắp khi bay. Đo thêm sai lệch theo góc quay TAG (T5) | P5 |
| H6 | TX power | A/B `LEGACY` so với `REFERENCE` và `SMART` ở cự ly gần và xa. Đổi profile thì **phải calibration lại** | P8 |
| H7 | Hình học anchor | Đặt anchor ở nhiều độ cao khác nhau, tránh đồng phẳng. Giữ vùng bay nằm trong vùng bao các anchor | P1, P7 |

### 5.3 Estimator trên UP 7000 (P7, P10)

1. Đầu vào là **`corrected_mm`**, không dùng `filtered_mm`. Lọc trước bằng bộ 1D
   làm sai số tương quan theo thời gian và trễ thêm, vi phạm giả định nhiễu trắng
   của EKF. Bộ lọc firmware chỉ giữ để hiển thị và chẩn đoán.
2. IEKF cập nhật từng range không đồng bộ, theo kế hoạch UP7000 dòng 465.
   R lấy theo FPP và cờ NLOS (H4).
3. Loại outlier trong không gian 3D:
   - gate NIS,
   - trọng số robust kiểu Geman-McClure (tham khảo B10),
   - cơ chế tự mở cổng khi estimator phân kỳ (tham khảo B9).
4. Giảm trễ:
   - Khi bay, nối UART của TAG thẳng vào UP 7000, không qua ESP32.
   - Baud từ 921600 trở lên.
   - Gắn thời điểm đo theo đồng hồ TAG, đồng bộ bằng heartbeat.
   - Chỉnh `EKF2_EV_DELAY` theo innovation (kế hoạch UP7000 dòng 251).

### 5.4 Dọn mã (P9)

Sau khi A/B xong:
- Chốt lấy một chế độ lọc và xóa hẳn nhánh `LEGACY_ADAPTIVE` và `C9_2_MOTION`
  cùng các cờ đi kèm.
- Tách `tag_ranging.c` thành 4 phần: lịch anchor, máy trạng thái trao đổi bản
  tin, tính khoảng cách, và publish.
- Test trong `Firmware/tests` (mô phỏng DW1000) là lưới an toàn khi refactor.

### 5.5 Nhiều drone (P11), chỉ khi có nhu cầu

- **Bậc 1:** TDMA cho TWR theo kiểu B7.
- **Bậc 2:** TDoA3. `Sniffer_DevKit` đã đẩy mọi frame kèm timestamp DW1000, có
  thể làm nền cho TAG thụ động. Engine TDoA và bù clock đặt trên UP 7000, vẫn
  giữ nguyên tắc ADR-001 "TAG chỉ đo".

---

## 6. Thí nghiệm kiểm chứng đề xuất

| ID | Thí nghiệm | Chỉ số | Để xác nhận |
|---|---|---|---|
| T1 | Dùng sniffer ghi timeline 8 anchor, cấu hình hiện tại | p50/p99 thời gian chu kỳ, thời gian từng slot | P6; ước tính 25–33 ms và 310 µs mỗi frame |
| T2 | Đo tĩnh 3–5 cự ly cho từng anchor, mỗi điểm ≥ 1000 mẫu, DS-TWR | Bias trung bình, σ | P3, H3 |
| T3 | Như T2 nhưng thay đổi suy hao (cự ly hoặc bộ suy hao) để quét mức RX | Đường `bias(rx_cdbm)` | P2, H2 |
| T4 | Đặt vật cản (người, tường, kim loại) chắn đường truyền | Δ = rx − fp, sai lệch range | P4, ngưỡng H4 |
| T5 | Quay TAG theo góc 0–360°, cố định cự ly | Sai lệch range theo góc | P5 |
| T6 | A/B TX power `LEGACY`, `REFERENCE`, `SMART` | Bias và σ ở cự ly gần và xa, tỉ lệ đo thành công | P8, H6 |
| T7 | So σ của SS-TWR có bù CI với DS-TWR trên cùng dữ liệu. Nếu có thể, so với mô phỏng TDoA dùng dữ liệu sniffer | σ, bias | Mục 4.1, 4.2 |
| T8 | So S1 (2 nhóm) với cấu hình hiện tại | Số range hợp lệ mỗi giây cho mỗi anchor, sai số vị trí trên quỹ đạo đã biết | 5.1 |

---

## 7. Lộ trình đề xuất

1. T1–T6: A/B RF, sau đó calibration antenna delay (H3, H6).
2. H2 + H4: bù bias theo mức RX và cờ NLOS, làm trên host trước.
3. H1 + H7: đo lại vị trí anchor, hoặc làm chế độ tự hiệu chỉnh.
4. S1, sau đó S2.
5. 5.3: IEKF trên UP 7000.
6. 5.4: dọn mã.
7. 5.5: chỉ khi cần nhiều drone.

---

## 8. Câu hỏi mở cho reviewer

1. Dải bias theo mức RX của DW1000 ở Ch5/PRF16 theo APS011 cụ thể là bao nhiêu?
   Có cần bảng riêng cho từng loại board (DWM1001 và STM32 + DW1000) không?
2. Ngưỡng NLOS dựa trên `rx − fp` theo APS006 là bao nhiêu? Công thức RX power
   và FPP trong `dw1000.c` (chỉnh RXPACC, xem `telemetry.h:57`) có khớp với
   User Manual mục 4.7 không?
3. Độ dư của phép bù carrier integrator trên phần cứng này là bao nhiêu? Con số
   0,1 ppm ở mục 4.1 chỉ là giả định.
4. Ở cùng số anchor và cùng hình học, sai số vị trí của TDoA3 so với DS-TWR đã
   hiệu chỉnh có thực sự kém hơn không? Có tài liệu hay số liệu thực nghiệm nào
   để so sánh?
5. Chế độ `MODE_SHORTDATA_FAST_ACCURACY` và `MID_ACCURACY` của `libdw1000` cụ
   thể dùng preamble và data rate nào (B2a)?
