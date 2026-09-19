# Rà soát firmware — 2026-09-15

## 1. Phạm vi và baseline thực tế

Rà soát nguồn đang phát triển trong `Firmware/Tag`, `Anchor_1`–`Anchor_8`, `ESP32C3_Gateway`, các test và giao diện calibration với GUI Python. Không lấy `Firmware Code Base` làm baseline đang chạy.

- TAG hiện đã cấu hình **8 anchor**, DS-TWR tuần tự, chu kỳ mục tiêu 20 ms, UART 115200.
- A1–A8 có địa chỉ riêng; kiểm tra source parity của các file chung và topology đã PASS.
- Calibration DS mask = 0, offset A1–A8 = 0; TX/RX antenna delay mặc định 16436. Đây là cấu hình bring-up, chưa phải bộ hiệu chuẩn đo trên từng module.
- Filter xuất hiện tại là Legacy Kalman; Legacy Adaptive ở SHADOW, C9.2 Motion OFF. Có code thử nghiệm không đồng nghĩa thuật toán đó đang là output chính.
- README GUI và một số ghi chú review cũ còn nói 4 anchor; nguồn firmware đã thay đổi ngày 14/09. Chưa xác minh firmware thực sự đã nạp trên từng board.

## 2. Các vấn đề ưu tiên cao

### F1 — Thăm dò offline có thể bỏ đói anchor cuối danh sách

**Mức: P1; đã tái hiện bằng mã C thật.**

Nguồn: `Tag/src/ranging/tag_ranging.c:1771`, `:1791`; `Tag/include/tag_ranging.h`.

TAG chỉ cho một offline probe mỗi chu kỳ, mỗi anchor được hẹn lại sau 5 chu kỳ, nhưng luôn duyệt từ A1 lên A8. Khi năm anchor đầu liên tục mất kết nối, chúng chiếm hết lượt probe; các anchor cuối không có cơ hội hồi phục.

Kiểm tra 500 chu kỳ, khởi tạo cả tám anchor đã đạt ngưỡng offline:

| Anchor | Số lần probe |
|---|---:|
| A1–A5 | 100 lần mỗi anchor |
| A6–A8 | 0 |

Nếu A6–A8 được bật lại trong trạng thái này, TAG vẫn không gửi POLL đến chúng. Đây là lỗi phục hồi kết nối, không chỉ là giảm tốc độ cập nhật.

**Sửa:** round-robin riêng cho offline probe hoặc chọn deadline quá hạn lâu nhất; bảo đảm mỗi anchor có giới hạn thời gian chờ. Test tất cả offline, một anchor online, các tập offline khác nhau và bật lại anchor ID lớn.

### F2 — Lỗi SPI dẫn đến ngừng đo vô thời hạn dù có watchdog

**Mức: P1 cho vận hành tự phục hồi; xác nhận bằng luồng code.**

Nguồn: `Tag/src/main.c:59`, `:92`; `Anchor_1/src/main.c:57`, `:82` và các bản Anchor tương ứng.

Khi counter SPI tăng, vòng ranging trả lỗi. `fatal_blink()` tiếp tục gọi `watchdog_feed()` vô hạn. Vì vậy watchdog không tự reset trong đường lỗi này; một lỗi SPI thoáng qua có thể làm node dừng đo cho đến khi can thiệp.

Ngừng phát range sau SPI lỗi là hành vi bảo vệ đúng; điểm thiếu là chính sách phục hồi và báo lỗi.

**Sửa:** trạng thái FAULT/RECOVERING, invalid toàn bộ phép đo, lưu nguyên nhân, thử reset/reinit DW1000 có giới hạn, sau đó reset MCU hoặc giữ fault theo chính sách rõ ràng. Tránh reboot loop bằng backoff và bộ đếm. Trước khi tái sử dụng `Tag_Init()` cho recovery cần reset đầy đủ state machine, filter chính, median, timeout/probe state và timestamp; hàm hiện được viết chủ yếu cho khởi động lạnh.

### F3 — Nhánh nhận frame chưa ưu tiên cờ lỗi RX đi kèm RX_GOOD

**Mức: P1 hardening; đã tái hiện với trạng thái thanh ghi mô phỏng.**

Nguồn: `Tag/src/ranging/tag_ranging.c:2039`, `:2266`; `Anchor_1/src/ranging/anchor_ranging.c:289`, `:384`.

Điều kiện thành công kiểm tra đủ `DW_ALL_RX_GOOD`, nhưng không loại các cờ như LDEERR/RXOVRR nếu cùng xuất hiện. Probe đặt `DW_ALL_RX_GOOD | DW_LDEERR_BIT`: TAG vẫn nhận RESP và chuyển sang TX_FINAL.

Đây là bằng chứng ở cấp logic xử lý bit; chưa khẳng định tần suất tổ hợp này trên phần cứng. Cần phân loại trạng thái lỗi/mâu thuẫn trước khi dùng timestamp.

**Sửa:** định nghĩa rõ những cờ làm phép đo không đáng tin, từ chối mẫu khi các cờ đó xuất hiện, ghi nguyên nhân và kiểm thử recovery tương ứng trên board.

## 3. Giao thức đo và calibration cần sửa/phát triển

### F4 — Chưa liên kết bốn frame vào một transaction

Nguồn: TAG `send_poll`, `build_and_send_final`, nhánh WAIT_RESP/WAIT_REPORT; Anchor `handle_poll`, WAIT_FINAL.

Sequence của bên TAG và Anchor tự tăng riêng; receiver kiểm tra header/PAN/address/function/length nhưng không đối chiếu sequence giao dịch. Probe gửi RESP sequence=200 trong khi bộ đếm TAG=7 vẫn được nhận và chuyển sang TX_FINAL.

Điều này chứng minh không có ràng buộc transaction, chưa chứng minh một range sai đã xảy ra trên board. Frame cũ hoặc nhầm chu kỳ từ cùng địa chỉ không được phân biệt bằng transaction ID. Hiện thiết kế chỉ nhắm một TAG.

**Phát triển:** transaction ID được echo xuyên POLL/RESP/FINAL/REPORT, quản lý duplicate/stale, deadline theo transaction. Nếu thay wire format phải đổi đồng bộ TAG và cả tám anchor, có version và test tương thích.

### F5 — Dấu offset GUI và firmware ngược nhau

Nguồn: `Tag/src/ranging/tag_ranging.c:1372`, `:1399`, `:1408`; `Software/UWB_UART_GUI/README.md`.

- Firmware: `corrected = measured - firmware_offset`.
- GUI: `gui_offset = reference - mean_raw`.

Ví dụ đo 1.20 m tại khoảng cách thật 1.00 m: GUI xuất −0.20 m. Chép trực tiếp −0.20 vào `UWB_DS_OFFSET_Ax_M` sẽ cho 1.40 m. Firmware cần +0.20 m trong ví dụ này.

Hiện chưa có đường import GUI→firmware tự động, nên đây là rủi ro tích hợp/thao tác thủ công, không phải một phép chuyển tự động đã chạy sai.

**Sửa:** quy ước tên và dấu duy nhất, hoặc exporter đổi dấu có kiểm thử. Tách `bias_to_subtract` và `correction_to_add`; ghi đơn vị và phiên bản trong profile.

### F6 — Calibration chưa có cơ chế cấu hình bền vững và validation

Nguồn: `Tag/include/uwb_app_config.h`, `Tag/include/uwb_calibration.h`, `apply_offset_and_clamp`.

- DS calibration là macro build-time; chưa có command cấu hình và lưu flash/NVS.
- `auto_calibrate_actual_m` chỉ tác động nhánh SS/SS_FALLBACK, sử dụng một phép đo rồi bật bit SS calibrated; không phải quy trình auto-calibration DS.
- INFO luôn gửi `uwb_ds_calibrated_mask_build`; trong cấu hình SS hoặc SS fallback, nó không mô tả đầy đủ mask runtime SS.
- Chưa có mapping serial module ↔ anchor ID ↔ calibration profile, version/hash, kiểm chứng nhiều cự ly, commit/rollback.

**Phát triển:** capture nhiều mẫu và nhiều cự ly, quality gate, validation độc lập; lưu mode/PHY/antenna-delay/serial/version/CRC; cập nhật atomic và rollback. Giữ mask=0 cho anchor chưa được xác nhận. Không bật mask hàng loạt chỉ để GUI hiện valid.

### F7 — raw và timestamp chưa đủ rõ cho estimator/replay

Nguồn: `Tag/include/tag_ranging.h` (`TagAnchorSample_t`), `mark_anchor_result`, `mark_anchor_rejected_measurement`, `Tag_GetSnapshot`.

- `raw_mm` khi đã calibrated là khoảng cách sau software offset; khi CAL_MISSING lại là khoảng cách trước offset.
- Chỉ có timestamp snapshot cả chu kỳ và age kể từ lần valid cuối. Tám anchor được đo tuần tự nên không cùng thời điểm.
- Với diagnostic chưa calibrated, age không phản ánh tuổi của phép đo diagnostic vừa nhận.

**Phát triển:** tách uncorrected/corrected/filtered range, sample timestamp hoặc time offset từng anchor, measurement mode, radio-valid/calibration-valid/filter-valid. Thêm boot ID và calibration ID để host xử lý reboot, replay và đổi cấu hình chính xác.

## 4. Timing và observability

### F8 — 8 anchor × 50 Hz hiện là mục tiêu, chưa phải hiệu năng được chứng minh

Nguồn: `tag_ranging.h`, `finish_full_cycle`, `note_anchor_response_received`, WAIT_REPORT.

- Cấu hình 20 ms và static assertion trong test chỉ xác nhận hằng số; không đo được tần số thực.
- Mỗi chu kỳ có tám trao đổi DS và bảy guard 150 µs; thêm xử lý SPI, filter, timeout và UART ISR.
- Backoff hiện chỉ dựa trên timeout RESP. Anchor trả RESP nhưng liên tục mất REPORT được xóa timeout streak trước khi DS hoàn tất, vẫn tốn REPORT timeout mỗi chu kỳ.
- `cycle_overrun_count` phát hiện muộn sau khi hết chu kỳ; chưa có scheduler theo ngân sách thời gian hoặc phân nhóm linh hoạt.

**Phát triển:** đo cycle/slot p50/p95/p99/max, actual per-anchor update rate và recovery latency; test 1/4/8 anchor, mất RESP, mất REPORT, NLOS và UART tải cao. Tách radio response health khỏi DS completion health. Chọn tần số/slot policy theo kết quả, chỉ đổi sang lịch nhóm/bất đồng bộ khi có bằng chứng cần thiết.

### F9 — Diagnostics cần cho GUI đang chỉ nằm trong debugger

Nguồn: `Tag/include/tag_ranging.h`; `Tag/src/telemetry/telemetry.c:277`.

Đã có per-anchor timeout, poll skip, slot duration, DS fallback/report lỗi, filter diagnostics, nhưng STATS mới truyền tổng poll/ok/timeout/rxerr/overrun/UART overflow và rate.

**Phát triển:** DIAG packet có version, tốc độ thấp; chứa per-anchor counters, slot/cycle timing, SPI faults, reset reason, boot/build ID, calibration identities. Phân biệt skip do backoff với timeout thật trên wire. Bổ sung signal diagnostics phù hợp cho phân tích chất lượng/NLOS ở host; chưa cần đưa thuật toán nặng vào nRF.

UART: RANGE tám anchor = 145 byte, 50 Hz = 7.250 byte/s, khoảng 63% trần 115200 8N1 trước INFO/STATS. Đây là tính băng thông từ format, không phải xác nhận quá tải hiện tại. Khi mở rộng diagnostics cần tính lại tổng lưu lượng, queue high-water/drop; lựa chọn baud đồng bộ TAG/gateway/host sau kiểm chứng adapter.

### F10 — ESP gateway chưa có health/command path đầy đủ

Nguồn: `ESP32C3_Gateway/main/main.c`, `gateway_config.h`.

- Hiện là UART → parser → USB; không có đường command host→TAG hoặc Wi-Fi backend.
- USB forwarded/drop và parser counters chưa được xuất thành health telemetry.
- UART driver không dùng event queue để ghi nhận riêng overflow/frame/parity error.
- USB write chỉ thử một lần rồi đếm drop nếu số byte ghi không đủ; cần kiểm thử hành vi short-write/disconnect và tránh để frame dở dang bị hiểu như drop nguyên frame. Chưa xác minh API driver thực tế hoặc board trong lần review này.
- Chân RDY được cấu hình input nhưng chưa tham gia xử lý.

**Phát triển:** health snapshot, kiểm thử backpressure và USB reconnect, quy định drop/queue rõ ràng. Chỉ bổ sung command có ID/ACK/NACK/timeout khi firmware TAG có command handler tương ứng.

## 5. Kiểm thử, build và bảo trì

Đã chạy `tests/run_host_tests.ps1` thành công:

- driver failure/truncation;
- TAG timeout/IRQ/deadline;
- compile telemetry và Anchor responder với `-Wall -Wextra -Werror`;
- gateway parser;
- A1–A8 identity/source parity/topology.

Đã chạy thêm probe liên kết mã TAG/driver thật với SPI/time mô phỏng: starvation 500 chu kỳ, sequence không liên kết, RX_GOOD kèm LDEERR.

**Khoảng trống test:** DS math và timestamp wrap bằng vector chuẩn; toàn bộ state machine Anchor bằng mô phỏng; TX FINAL/REPORT loss; scheduler fairness/reconnect; calibration sign/mode; mixed status; UART ring/ISR; restart/reset; full protocol TAG→gateway→Python; timing và soak trên board. Việc các test cũ PASS không bao phủ các lỗi mới tìm thấy.

Nguồn Anchor hiện được sao chép tám nơi. Test parity giúp phát hiện lệch nhưng vẫn cần một module chung hoặc quy trình sinh project để tránh sửa thiếu node. Header/comment còn mô tả SS default, chân STM32 và timeout cũ; cần đồng bộ tài liệu với DWM1001/DS/8-anchor.

Không chạy full NCS/ESP-IDF build và không flash trong lần này. Không thấy artifact ở các đường chuẩn `Tag/build/zephyr/zephyr.hex` và `Anchor_*/build/zephyr/zephyr.hex` khi kiểm tra; có thể bản build nằm nơi khác. Chưa thể xác nhận binary nào đang trên board. Release cần manifest hash nguồn/config/binary và serial của từng board được nạp.

## 6. Thứ tự triển khai đề xuất

1. **Sửa trước:** offline probe fairness, fault/recovery/watchdog, RX status validation; thêm regression test tương ứng.
2. **Chuẩn hóa phép đo:** transaction ID, calibration sign/mode/profile, raw/corrected và timestamp từng sample.
3. **Làm cho lỗi quan sát được:** DIAG telemetry, per-anchor health, boot/reset/build metadata, gateway health.
4. **Kiểm chứng trên phần cứng:** 1/4/8 anchor, loss/reconnect, calibration/ground truth, timing/long-run; chốt 50 Hz theo số đo.
5. **Mở rộng sau baseline:** cấu hình runtime, commissioning, calibration storage; BLE/multi-TAG chỉ khi có nhu cầu và timing test tương ứng.

Ưu tiên hoàn thành scheduler và chuỗi dữ liệu đo trước khi thêm tính năng điều khiển drone vào GUI. Phần estimator và tích hợp PX4 thuộc backend theo kiến trúc dự án; firmware nRF nên cung cấp phép đo, thời gian, chất lượng và trạng thái lỗi đáng tin cậy.
