# Rà soát và kế hoạch nâng cấp TAG DevKit + 8 anchor hỗn hợp

**Ngày:** 2026-09-23  
**Trạng thái:** rà mã và test trên PC; chưa xác nhận bằng log RF/flash mới  
**Nguồn đang phát triển:** `D:\Drone Project\UWB DW1001\Firmware`  
**Nguồn STM32:** `D:\Drone Project\UWB Drone\STM32_UWB`  
**Đọc cùng:** `Plan/KE_HOACH_NANG_CAP_TOAN_DIEN_FIRMWARE_DWM1001.md`, `Firmware/HARDWARE_AB_CHECKLIST.md`.

Lúc rà soát, repo DW1001 ở nhánh `feature/firmware-upgrade-v2` với nhiều thay
đổi staged/unstaged và file mới chưa commit từ trước lượt này. Phiên triển khai
tiếp theo phải kiểm `git status` và giữ nguyên công việc đó; không reset tree về
HEAD để “lấy baseline”. Thư mục `UWB Drone` chứa project STM32 độc lập.

## 1. Cấu hình phần cứng đã được chủ dự án xác nhận

| ID vật lý | MCU/UWB | Mã tương ứng | Tình trạng |
|---|---|---|---|
| A1–A4 | DWM1001C (nRF52832 + DW1000) | `UWB DW1001/Firmware/Anchor_1..Anchor_4` | Đang đo được với TAG DevKit; chưa lưu log |
| A5–A8 | STM32F103 + radio DW1000 theo mã hiện có | `UWB Drone/STM32_UWB/Anchor`, `Anchor_2`, `Anchor_3`, `Anchor_4` | Chưa nạp bản firmware mới |
| TAG | DWM1001-DEV | `UWB DW1001/Firmware/Tag_DevKit` | Đang chạy firmware hiện tại của project |

**Lưu ý đặc biệt:** bốn project STM32 hiện *vẫn khai báo địa chỉ A1–A4*. Các project `UWB DW1001/Firmware/Anchor_5..Anchor_8` là target **nRF52832/Zephyr**, không tạo được binary cho STM32F103. Không nạp chúng vào bốn board A5–A8. Cần gán lại `STM32_UWB/Anchor → A5`, `Anchor_2 → A6`, `Anchor_3 → A7`, `Anchor_4 → A8`, rồi kiểm tra từng nhãn board trước khi flash.

## 2. Kết quả rà soát ưu tiên

| Mức | Phát hiện và bằng chứng | Hành động |
|---|---|---|
| P0 | TAG mặc định phát frame v2 (`Firmware/common/include/tag_ranging.h`, `UWB_TAG_FRAME_VERSION`); STM32 tạo RESP/REPORT v1 14 byte (`STM32_UWB/Anchor/Core/Src/anchor_ranging.c`, `build_resp`/`send_report`). TAG yêu cầu RESP cùng version/txn. | Port responder STM32 sang v2; chỉ dùng v1 như phép thử trên bàn nếu cần, không làm cấu hình sản xuất hỗn hợp. |
| P0 | STM32 source A1–A4 trùng ID của DWM1001C vật lý (`STM32_UWB/Anchor*/Core/Inc/dw1000_hw.h`, `ANCHOR_ADDR`). | Gán A5–A8, thêm kiểm thử tĩnh phát hiện địa chỉ trùng. |
| P1 | TAG legacy median-3 + scalar Kalman mặc định: gate gọi là reject nhưng chỉ ép FPP=-100/R=10000, vẫn cập nhật và phát `valid=1` (`tag_ranging.c`, `publish_distance`). | Khi gate reject, giữ last-good, đánh dấu range mới invalid và xuất raw ở kênh chẩn đoán. Thêm test spike/reacquire. |
| P1 | Sau `Tag_RecoverRadio`, snapshot được invalid nhưng bộ lọc và hàng đợi đo cũ chưa được xóa ở baseline rà soát (`tag_ranging.c`). | Reset toàn bộ trạng thái thời gian/filter/queue, test fault giữa exchange. |
| P1 | Scalar Kalman `Q=0.05` mm²/mẫu và `R=50/200/1000/10000` mm²; nhánh tín hiệu yếu có gain ổn định khoảng 0,0022 ở 50 Hz. Đây là dấu hiệu trễ động lớn, *chưa phải sai số đo đã xác nhận*. | Đo replay và A/B theo tốc độ thực trước khi đổi mặc định; không tăng số filter nối tiếp một cách mù quáng. |
| P1 | STM32 dùng HW antenna delay=0, legacy offset=1 (`STM32_UWB/Anchor/Core/Inc/uwb_calibration.h`); TAG DevKit dùng HW delay=1, legacy offset=0, DS calibrated mask=0. | Chốt một profile RF/antenna delay theo từng board; hiệu chuẩn lại A5–A8 và từng cặp với TAG. Không dùng offset STM cũ như calibration của DWM. |
| P1 | STM32 `DW1000_ReadRxData` cắt frame dài về 20 byte nhưng trả như frame đầy đủ; driver đọc SYS_STATUS 4 byte nên không thấy TXPUTE (bit 34); delayed TX mới kiểm HPDWARN (`dw1000_hw.c`). | Nâng RX buffer/kiểm độ dài trước copy, đọc status 40 bit, kiểm cả HPDWARN/TXPUTE. |
| P2 | Đổi/hủy DS calibration trên TAG từng để `s_track.valid=1` tới slot kế tiếp; offset DS âm quá mức bị kẹp thành 0 mm rồi vẫn có thể phát valid. | Invalidate snapshot tức thì khi calibration đổi; coi khoảng cách đã hiệu chỉnh <0 hoặc ngoài vùng vận hành là lỗi, không gán 0 valid. |
| P2 | Settings chỉ bind calibration với PARTID/profile của TAG, chưa bind danh tính và RF profile của anchor; TLV hiện chỉ báo build/config/antenna delay. | Thêm fingerprint anchor bền vững và fail-closed khi thay module/PHY/delay. |
| P2 | `Tag_DevKit/app.overlay` mặc định UART 115200; `RANGE_MEAS` chi tiết mặc định tắt, cần ≥460800 để thu từng exchange. Chưa có log thực tế. | Thu log ở baud phù hợp, theo dõi UART overflow và slot/cycle timing, rồi mới chốt filter/scheduler. |

Rà riêng `Firmware/Anchor_1..Anchor_8` cho thấy 8 project Zephyr dùng cùng `Firmware/common` và khác ID/config. Responder nRF hiện có parse v1/v2, echo txn, tính Da/Rb từ timestamp 40-bit, kiểm HPDWARN/TXPUTE và WAIT4RESP. Chưa tìm thấy lỗi công thức chắc chắn trong đường này; cần test RF thật. Các bản STM32 cũng dùng PHY gần tương ứng Ch5/PRF16/preamble256/6,8 Mbps theo source, nhưng profile timing/calibration khác và chưa thể coi là tương thích hoàn toàn.

### Các sửa chữa đã thực hiện trong lượt rà soát này

`Firmware/common/src/ranging/tag_ranging.c` đã được sửa để legacy gate không cập nhật Kalman hoặc phát valid khi từ chối outlier; recovery xóa hàng đợi và reset filter; thay/hủy DS calibration invalid snapshot ngay **và xóa các `RANGE_MEAS`/cycle snapshot đang chờ theo offset cũ**; DS/SS offset tạo khoảng cách âm, 0 hoặc làm tròn về 0 trả compute error. `Firmware/tests/test_tag_state.c` thêm test cho các tình huống đó. Không đổi `Q/R`, mode filter mặc định, PHY hoặc calibration mask. Đường compute-error hiện giữ raw trước offset trong `s_meas` nội bộ nhưng chưa đẩy nó vào `RANGE_MEAS`; nếu cần phân tích lỗi hiệu chuẩn trên log, bổ sung telemetry chẩn đoán cho case này trong phiên tiếp theo.

Bộ host test đầy đủ đã đạt **13/13 nhóm**, gồm 50 Python GUI tests và gateway parser. Host test TAG biên dịch riêng với config `Tag_DevKit` cũng đạt. Build Zephyr của TAG DevKit và tám **target nRF** đạt; build TAG được chạy lại sau sửa code. Kết quả này chưa xác nhận bốn board STM32, thời gian thực, range bias hay chuyển động trên phần cứng.

## 3. Quyết định kiến trúc

1. Giữ TAG dùng **v2** và port bốn responder STM32 sang v2. Không hạ cả mạng về v1 chỉ để bốn board mới trả lời: v1 không có transaction ID nên tăng rủi ro ghép nhầm frame muộn.
2. Dùng `Firmware/common/include/uwb_frame.h` và `Firmware/common/src/ranging/uwb_frame.c` làm đặc tả wire format. Hai file thuần C; đưa vào CubeIDE qua bản copy có checksum/kiểm thử parity, hoặc một package chung được build ở cả hai project. Không để bốn bản STM32 tự định nghĩa offset khác nhau.
3. Firmware TAG phát range thô, trạng thái, chất lượng và timestamp trung thực. Bộ ước lượng vị trí/IMU cho drone chạy trên host. Bộ lọc trên TAG chỉ làm điều hòa range và phải giữ đường raw để replay.
4. Ba mục tiêu ổn định, tốc độ và độ trễ cùng được đo. Nếu tám DS-TWR tuần tự vượt chu kỳ 20 ms, lịch polling có thể chia 4+4 giữa hai chu kỳ và host dự đoán bằng IMU ở nhịp điều khiển. Chỉ chọn sau khi có phân bố thời gian slot thực; không coi 50 Hz/anchor là đã đạt chỉ vì `TAG_CYCLE_MS=20`.

## 4. Sườn code cho phiên triển khai tiếp theo

### 4.1. Danh tính A5–A8 và cổng giao thức

**Sửa:** `STM32_UWB/Anchor*/Core/Inc/dw1000_hw.h` và mọi giá trị ID/nhãn liên quan. Đề xuất tạo một `node_identity.h` cho từng project, chỉ khác địa chỉ và tên board:

```c
/* Anchor -> 5; Anchor_2 -> 6; Anchor_3 -> 7; Anchor_4 -> 8. */
#define TAG_ADDR       ((uint16_t)0U)
#define ANCHOR_ADDR    ((uint16_t)5U)  /* ví dụ project Anchor */
#define DW_PAN_ID      ((uint16_t)0xDECAU)
_Static_assert(ANCHOR_ADDR >= 5U && ANCHOR_ADDR <= 8U,
               "STM32 anchor phải dùng ID A5..A8");
```

Kiểm `CMakeLists.txt`/CubeIDE project mapping và test script phải quét 4 header, so với A1–A4 của Zephyr, xác nhận tập ID chính xác `{1,2,3,4,5,6,7,8}`. Tên project STM có thể được đổi thành `Anchor_5..8` *sau* khi sửa linked resources/build; không đổi thư mục vội nếu CubeIDE trỏ đường dẫn cũ.

**Wire format v2 chính xác** (độ dài RX gồm 2 byte FCS do DW1000 thêm): header `[0..9]` = `41 88 | seq | PAN16 | dst16 | src16 | func`; POLL 15 B gồm `[10]=2,[11]=txn,[12]=flags`; RESP tối thiểu 20 B gồm `ver,txn,Da32,status,tlv_len`; FINAL 14 B gồm `ver,txn`; REPORT 22 B gồm `ver,txn,Rb32,FP_cdbm16,RX_cdbm16`. Đây là các macro trong `uwb_frame.h`; không dùng kích thước `14`/`20` của source STM cũ làm RX capacity.

```c
/* Trong STM32 anchor_ranging.c: dùng cùng parser/builder với TAG. */
static uint8_t rx_buf[UWB_FRAME_MAX_RX_LEN]; /* 64, gồm FCS */
static uint8_t active_version, active_txn;
static uint16_t active_tag;

static bool parse_poll_for_me(const uint8_t *frame, uint16_t rx_len,
                              UwbFrameHeader_t *hdr, UwbPoll_t *poll)
{
    return uwb_frame_parse_header(frame, rx_len, DW_PAN_ID, hdr)
        && hdr->func == 0x21U && hdr->dst == ANCHOR_ADDR
        && hdr->src == TAG_ADDR
        && uwb_frame_parse_poll(frame, rx_len, poll);
}

static bool parse_open_final(const uint8_t *frame, uint16_t rx_len)
{
    UwbFrameHeader_t hdr;
    UwbFinal_t fin;
    return uwb_frame_parse_header(frame, rx_len, DW_PAN_ID, &hdr)
        && hdr.func == 0x23U && hdr.dst == ANCHOR_ADDR
        && hdr.src == active_tag
        && uwb_frame_parse_final(frame, rx_len, &fin)
        && fin.version == active_version
        && (fin.version == UWB_FRAME_V1 || fin.txn == active_txn);
}
```

Trong `handle_poll`: đọc `poll_rx_ts[5]`, giữ `active_version/txn/tag`, tính delayed `resp_tx = (poll_rx + REPLY_DELAY_TICKS) & ~0x1FFULL`; `Da` phải là `(resp_tx + active_tx_ant_delay - poll_rx) & ((1ULL<<40)-1)` để dùng RMARKER thật. Build `UwbResp_t` và gọi `uwb_frame_build_resp`, ghi `len + 2` vào TX_FCTRL. Sau `StartTxDelayedEx(wait4resp=1)`, lỗi HPDWARN **hoặc TXPUTE** phải hủy slot và trở về RX; không chờ wrap 40-bit. Sau TXFRS, chỉ clear TX status, giữ RX event/IRQ có thể đến sớm. Trong `send_report`, đọc timestamp RESP TX và FINAL RX thật để tính `Rb` modulo 40-bit, build `UwbReport_t` theo `active_version/txn`, đưa FP/RX power hoặc `INT16_MIN` khi chưa đo được.

**Driver STM32:** sửa `DW1000_ReadRxData(buf, cap)` trả lỗi nếu `RX_FINFO.length > cap` hay >64; không cắt/trả thành công. Đọc SYS_STATUS ít nhất 5 byte vào `uint64_t`, clear W1C chính xác. Hàm start delayed TX trả mã lỗi riêng cho HPDWARN, TXPUTE, timeout. `DW1000_Configure()` trả lỗi nếu PLL không lock, main phải recovery/fault thay vì chỉ log. Đối chiếu thanh ghi/bit từ header đang dùng và DW1000 user manual trước khi ghi.

### 4.2. Calibration và profile

Mỗi A5–A8 phải được hiệu chuẩn **sau** khi chốt antenna delay, TX power, PHY và vỏ/ăng-ten. Giữ `UWB_DS_CALIBRATED_MASK=0`; raw range chỉ là chẩn đoán cho tới khi đủ số mẫu chuẩn. Mục tiêu là mỗi entry calibration gắn với cả hai đầu radio:

```c
typedef struct {
    uint16_t anchor_id;
    uint32_t tag_part_id;
    uint32_t tag_rf_profile_id;
    uint32_t anchor_part_id;       /* thêm qua TLV định danh/commissioning */
    uint32_t anchor_rf_profile_id; /* PHY, TX power, TX/RX antenna delay */
    int32_t  ds_bias_um;           /* corrected_mm = measured_mm - bias_um/1000 */
    uint8_t  calibrated;
} PairCalibration;

bool pair_calibration_valid(const PairCalibration *c,
                            const DeviceInfo *tag, const AnchorInfo *anchor)
{
    return c->calibrated && c->tag_part_id == tag->part_id
        && c->tag_rf_profile_id == tag->rf_profile_id
        && c->anchor_part_id == anchor->part_id
        && c->anchor_rf_profile_id == anchor->rf_profile_id;
}
```

Đây là **sườn API**, chưa phải drop-in code: `AnchorInfo`/TLV và NVS schema hiện chưa có PARTID anchor. Tăng schema version, đọc schema cũ fail-closed, chỉ đánh dấu calibrated sau khi fingerprint từ A5–A8 khớp. Không đưa `boot_count` hoặc git dirty bit vào RF profile vì chúng đổi mà bias vật lý không đổi. Khi mất thông tin danh tính anchor, thay board cùng ID, thay profile, hoặc lệnh `SET_DS_CAL` thay giá trị: invalid snapshot ngay và reset filter. Sau offset, `corrected < 0` phải là compute/calibration error; không biến thành 0 mm valid. Bật mask từng anchor sau thử đa cự ly và reboot/NVS roundtrip.

### 4.3. Bộ lọc có độ trễ đo được

Giữ đường legacy làm baseline; bổ sung shadow trên host hoặc sau cờ build. Input bắt buộc là raw corrected range, thời gian đo thật, `valid/status`, FPP/RX power/noise/FP index, CI và anchor ID. Tuyệt đối không dùng filtered range làm measurement của filter thứ hai. Dùng constant-velocity 1D **theo từng anchor** trước, sau đó so sánh với estimator vị trí ở host:

```c
/* Giả mã cho một anchor; float đủ cho Cortex-M4F, guard mọi finite/overflow. */
dt = (t_meas_us - state.last_us) * 1e-6f;
if (!state.ready || dt <= 0 || dt > stale_reset_s) {
    seed_from_current_good_measurement();  /* không giữ state từ trước dropout */
} else {
    F = [[1, dt], [0, 1]];
    Q = sigma_a2 * [[dt4/4, dt3/2], [dt3/2, dt2]];
    x_pred = F * x;
    P_pred = F * P * transpose(F) + Q;
    R = calibrated_range_variance(signal_quality, anchor_id);
    innovation = raw_mm - x_pred.range_mm;
    S = P_pred[0][0] + R;
    nis = innovation * innovation / S;
    if (!finite(S) || S <= 0 || nis > measured_nis_gate) {
        publish_invalid_raw_diagnostic();  /* không update x bằng outlier */
        consider_clustered_reacquire_candidate();
    } else {
        kalman_update();
        publish_valid_filtered_with_age_and_variance();
    }
}
```

`sigma_a2`, R theo FPP/NLOS và `measured_nis_gate` phải fit bằng replay, không lấy một ngưỡng tùy ý làm mặc định. Với C9 sẵn có, kiểm lại thang FPP: `UWB_FILTER_FPP_COMPAT_DB=-12.04` có thể đẩy tín hiệu thật -97 dBm xuống -109 dBm nội bộ, dưới `reacquire_min_fpp_dbm=-105`; test riêng case này trước khi bật C9. Tránh thêm median sau một median khác vì sẽ cộng trễ. Giữ snapshot last-good và age riêng; mọi reject hoặc stale phải có status rõ ràng.

### 4.4. Scheduler và telemetry

Ghi phân bố `POLL→RESP→FINAL→REPORT`, tổng slot, thời gian filter, chu kỳ, timeout, retry và UART queue high-water theo từng anchor. Mặc định 8 slot/20 ms hiện là mục tiêu thiết kế, chưa phải tốc độ thực. Nếu p99 tổng slot >20 ms hoặc có overrun: thử lịch 4 anchor/chặng 20 ms luân phiên (A1–A4 rồi A5–A8), offline probe giới hạn một slot, gửi timestamp/age cho host. So sánh với lịch 8/40 ms và chỉ chọn theo p95/p99 latency tới estimator, độ sẵn có của từng anchor và overrun. Host có thể cập nhật dự đoán IMU nhanh hơn UWB; không nội suy range cũ thành phép đo mới.

Để thu từng exchange: tạm dùng UART VCOM của TAG DevKit ở **ít nhất 460800 baud**, bật `RANGE_MEAS`, cùng baud trên host/parser; kiểm `uart_tx_high_water`, dropped frames và thời gian slot. Sau khi lấy log, trả profile vận hành đã được nghiệm thu. Không bật telemetry dày khi UART 115200 rồi đánh giá mất gói như lỗi RF.

## 5. Kiểm thử và thứ tự triển khai

1. **Chốt baseline:** lưu binary/hex hiện có, git hash/config hash/dirty và PARTID từng board, nhãn A1–A8, profile PHY/TX power/delay. Nếu không đọc được fingerprint A5–A8, để `valid=0` cho chúng.
2. **PC:** `Firmware/tests/run_host_tests.ps1`; build TAG DevKit và nRF A1–A4. Thêm host test STM32 chạy chính `uwb_frame.c` + responder state trên stub HAL/DW1000. Test v2 golden bytes, txn sai, frame dài, PAN/src/dst sai, FCS length, 40-bit wrap, delayed TX late, TXPUTE, RX timeout/rearm và v1 (nếu vẫn hỗ trợ). Test bộ lọc: spike, ramp 0.5/1/2 m/s, dropout, recovery, đổi calibration, offset âm. Build cả bốn CubeIDE target A5–A8 và xác minh map/size/ID từ ELF.
3. **RF một board:** flash A5 **sau** khi ID=5 và v2 host test pass; giữ A6–A8 tắt. Kiểm sniffer thấy đúng POLL/RESP/FINAL/REPORT, txn, Da/Rb, slot timing; range vẫn `CAL_MISSING`. Lặp A6, A7, A8 từng chiếc, tránh hai board cùng ID.
4. **Log nghiệm thu:** 1/3/5/10 m LOS, tối thiểu 2000 mẫu mỗi anchor/cự ly; nhiều hướng ăng-ten, 5 phút tĩnh ở 5 m; di chuyển 0.5/1/2 m/s với ground truth; thử một vật cản NLOS, rút/cắm lại từng anchor và restart TAG. Thu cả raw/filtered/status/timestamp/quality/slot/CPU/overflow, với cấu hình 4 rồi 8 anchor.
5. **Báo cáo A/B:** per-anchor bias, độ lệch chuẩn, p50/p95/p99 lỗi, độ trễ động, false reject/false accept, availability, reacquire time, p99 cycle/slot và UART loss. Đề xuất gate ban đầu: không mất valid availability so với baseline, không có overrun trong bài 10 phút, độ trễ động p95 giảm có ý nghĩa và static p95 không xấu đi; chốt ngưỡng số sau khi xem baseline/điều kiện bay.
6. **Calibration:** theo `HARDWARE_AB_CHECKLIST.md`, riêng A1–A4 và A5–A8, ít nhất 2 cự ly xác nhận sau khi fit; lưu NVS, reboot và đối chiếu fingerprint; bật từng bit mask chỉ sau khi pass. Kiểm thay một anchor cùng ID hoặc đổi delay khiến range thành invalid.
7. **Rollback:** giữ binary và settings snapshot trước mỗi lượt; nếu lỗi giao thức hoặc bias xấu, trả TAG và anchor về cặp version tương thích, xóa bit calibration của board vừa đổi. Không dùng một ảnh TAG v2 với anchor STM v1 để kết luận mất sóng RF.

**Điều kiện kết thúc:** cả tám ID duy nhất, đủ bốn frame v2/slot, không có range valid chưa hiệu chuẩn, test PC/build pass, và log A/B chứng minh các mục tiêu ổn định/tốc độ/độ trễ trên đúng phần cứng. Cho tới lúc đó không đưa range vào vòng điều khiển bay tự động.

## 6. Dữ liệu còn cần từ lần thử board

- Log UART/GUI hoặc `RANGE_MEAS` của 4 anchor đang chạy, rồi log sau khi thêm từng STM32.
- Sơ đồ bố trí A1–A8, kích thước vùng bay, tốc độ/ gia tốc tối đa, khoảng cách hoạt động và nhu cầu cập nhật vị trí thật của bộ điều khiển.
- Loại module DW1000/ăng-ten trên bốn board STM32, nhãn serial từng board, độ dài dây SPI/nguồn và profile TX power/antenna delay thực tế.
- Ground truth cho thử động (camera/motion capture/thước mốc theo thời gian); nếu chưa có thì chỉ có thể xác nhận tĩnh và giao thức, chưa tối ưu độ trễ động đáng tin cậy.

**Tham chiếu RF:** [Qorvo DW1000](https://www.qorvo.com/products/p/DW1000) dẫn tài liệu APS014 về hiệu chuẩn antenna delay và APS006 về chất lượng tín hiệu/NLOS; [APS013](https://forum.qorvo.com/uploads/short-url/x34DrF7EW5fQP9wY3aNESqPKz8z.pdf) mô tả DS-TWR bất đối xứng và công thức ToF. Quyết định code ở trên dựa trên source của hai project; các ngưỡng RF cần số đo thật.
