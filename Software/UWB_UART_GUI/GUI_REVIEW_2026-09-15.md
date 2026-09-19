# Đánh giá GUI Python — 2026-09-15

## Phạm vi và kết luận

Đã đọc sáu module Python của GUI, README, bộ test, telemetry/config của Firmware/Tag và đối chiếu mục 11.3 trong Plan/KE_HOACH_TRIEN_KHAI_UWB_DRONE_UP7000_PIXHAWK6C.md. Chưa sửa mã ứng dụng, chưa kiểm thử UART với board thật hoặc xác minh nội dung binary EXE.

GUI hiện là công cụ chẩn đoán UWB khá đầy đủ ở phần nhận dữ liệu, nhưng chưa hoàn thành quy trình hiệu chuẩn, phân tích offline và đánh giá độ tin cậy vị trí. Những chức năng tích hợp drone trong kế hoạch cũng chưa có ở Software này.

## Chức năng đã có

- Serial UART, parser binary/CRC/resync, INFO/RANGE/STATS, demo không cần board.
- Bảng A1–A8 với valid/status/age/FPP; plot Raw/Host/Firmware Filter.
- Ghi CSV/JSONL/events/binary bằng writer thread, metadata Host Filter và snapshot anchor layout.
- Nhập/xuất tọa độ ENU, preset 4/8 anchor, bản đồ 2D/3D, trail, solver robust, residual.
- Thống kê theo cửa sổ, xuất CSV; capture calibration nhiều khoảng cách và xuất CSV/JSON.

Cập nhật sau khi rà soát firmware ngày 15/09: nguồn Tag hiện đã đặt TAG_NUM_ANCHORS=8 và có project Anchor_1–Anchor_8; đoạn README GUI còn nói firmware có thể chỉ gửi A1–A4 đã cũ. Chưa xác minh binary được nạp hoặc khả năng đo đủ tám anchor trên phần cứng. Cấu hình Tag đang đặt UWB_DS_CALIBRATED_MASK=0; dữ liệu CAL_MISSING được dành cho chẩn đoán/hiệu chuẩn và bị loại khỏi solver vị trí.

## A. Lỗi và hạn chế cần sửa trước

### A1. Calibration có thể đánh giá GOOD với dữ liệu không đủ hoặc đã cũ

Vị trí: gui_views.py:576–646.

- ingest chỉ kiểm tra raw dương và valid/diagnostic, không loại mẫu valid có age quá lớn.
- Nút kết thúc capture có thể gọi finish_capture trước khi đủ số mẫu. finish_capture không áp dụng mức tối thiểu 20 mẫu mà toggle_capture yêu cầu khi bắt đầu.
- Đã tái hiện: một mẫu valid có age=5000 ms được nhận; kết thúc capture tại reference phù hợp cho kết quả samples=1, quality=GOOD.
- Cần kiểm tra freshness theo loại mẫu, số mẫu tối thiểu, thời lượng, độ ổn định; tách trạng thái INCOMPLETE khỏi GOOD. Không áp dụng máy móc age của range valid cho CAL_MISSING vì age diagnostic có ngữ nghĩa riêng.

### A2. Đổi lựa chọn Analysis không tính lại dữ liệu

Vị trí: gui_views.py:386–387, 415–449.

- Combobox anchor/plot/cửa sổ chỉ gọi draw; cached_points và latest_metrics chỉ được đổi trong refresh.
- Đã tái hiện: A1=1000 mm, A2=2000 mm; đổi lựa chọn sang A2 rồi draw vẫn dùng raw=1000 mm.
- Khi UART đang chạy, refresh tiếp theo thường sửa lại sau khoảng 200 ms; khi ngắt kết nối thì dữ liệu lựa chọn có thể sai mãi. Đổi cửa sổ rồi xuất CSV cũng có thể xuất thống kê của cửa sổ trước đó.
- Cần tính lại cache theo lựa chọn ngay cả khi không có frame mới.

### A3. Chỉ số geometry chưa phản ánh độ suy biến

Vị trí: gui_analysis.py:291 và _fit_position; gui_views.py:227–254.

- _matrix_condition_diagonal chỉ lấy tỷ số phần tử lớn/nhỏ trên đường chéo, bỏ qua các phần tử ngoài đường chéo.
- Đã tái hiện: ma trận suy biến [[1,1],[1,1]] nhận chỉ số 1.0. Đây là kiểm tra trực tiếp helper; solver có kiểm tra pivot riêng nên không đồng nghĩa mọi ma trận suy biến đều tạo được nghiệm.
- Cần condition number thực hoặc kiểm tra trị riêng/singular values, covariance/GDOP và trạng thái chất lượng có ngưỡng rõ ràng.
- Bản đồ hiện thêm mọi nghiệm khác None vào trail; chưa chặn theo RMS, residual, condition hay hội tụ. _fit_position có giới hạn 20 vòng nhưng chưa trả cờ hội tụ.

### A4. “Toàn phiên” thực tế chỉ là phần lịch sử còn trong RAM

Vị trí: uwb_uart_gui.py:53, 200–203; gui_views.py:412–423, 451.

- detailed_history/frame_times bị giới hạn 15.000 phần tử. Ở 50 Hz, tương đương khoảng năm phút.
- Plot Analysis chỉ vẽ 1.200 mẫu cuối, khoảng 24 giây ở 50 Hz, kể cả chọn cửa sổ dài hơn. Trục ngang là chỉ số mẫu, không phải thời gian thực.
- Cần đổi nhãn đúng ý nghĩa hoặc dùng thống kê tích lũy/log trên đĩa; plot giảm mẫu nhưng phủ đủ khoảng thời gian và biểu diễn khoảng mất dữ liệu.

### A5. Hiệu chuẩn chưa gắn chặt với cấu hình đo

Vị trí: gui_views.py:519–677; uwb_uart_gui.py:684–693.

- Capture/export chưa chứa SS/DS mode, PHY, firmware/calibration identity; lời nhắc không trộn SS-TWR và DS-TWR chưa được kiểm tra bằng code.
- Offset tổng hợp lấy trung bình theo số mẫu của mọi capture, kể cả capture bị đánh dấu CHECK.
- Chưa có chọn/bỏ capture lỗi, tập validation riêng, profile hiệu chuẩn có version, import/apply/rollback.
- INFO có active_offsets_um, motion_mode, global_motion_state và flags nhưng UI mới hiển thị một phần.
- Cần lưu metadata lúc capture, ngăn trộn cấu hình, giữ kết quả chưa đạt khỏi profile được chấp nhận và hiển thị offset đang thực sự áp dụng.

## B. Chức năng còn thiếu cho công cụ đo thực tế

| Ưu tiên | Nâng cấp | Kết quả cần đạt |
|---|---|---|
| Cao | Mở phiên đo và replay | Load binary/CSV + layout, play/pause/seek/tốc độ, timestamp gốc, reset filter đúng khi seek/reboot |
| Cao | Calibration hoàn chỉnh | Lưu raw của từng capture, loại capture, fit/validation, so sánh trước/sau, profile version; tách correction trên host khỏi calibration firmware |
| Cao | Log có thể tái lập | Ghi byte UART trước parser để giữ cả CRC lỗi/noise, snapshot INFO khi bắt đầu ghi, mode/layout/calibration/filter/hash và sự kiện đổi cấu hình |
| Cao | Position có đánh giá chất lượng | Chọn nguồn range rõ ràng; trạng thái chờ/tracking/degraded/lost, residual từng anchor, covariance và log position |
| Vừa | Phục hồi kết nối | Reconnect có giới hạn, nhận diện adapter bằng serial, phân biệt kết nối COM với đang nhận telemetry |
| Vừa | Giới hạn tải GUI | Queue nhận có giới hạn và bộ đếm drop/backlog; giảm tải phân tích dài; không để modal dialog làm backlog tăng vô hạn |
| Vừa | Workflow thực địa | Profile COM/baud/layout, đánh dấu sự kiện đo, tìm/lọc log, so sánh nhiều anchor và xuất báo cáo |
| Vừa | Phân tích chất lượng đầy đủ | Phân bố timeout/reject/calibration_missing, tỷ lệ mất frame, khoảng ngắt, sai số so ground truth; tách noise tĩnh khỏi biến thiên khi chuyển động |

raw_telemetry.bin hiện chứa frame đã qua parser, được encode lại từ trường frame, không phải toàn bộ byte UART đầu vào. File hữu ích cho replay dữ liệu hợp lệ nhưng chưa đủ tái hiện lỗi truyền thông.

Nguồn vị trí hiện ưu tiên Host Filter rồi Firmware Filter rồi raw (gui_views.py:232–234). Cần hiển thị/chọn nguồn và đo độ trễ trước khi dùng kết quả cho hệ thống định vị drone.

## C. Các phần phụ thuộc firmware/backend theo kế hoạch drone

Chưa thấy trong Software hiện tại:

- Kết nối UP 7000 qua REST/WebSocket, telemetry health của companion.
- PX4 armed/mode/battery/EKF/failsafe; optical flow/rangefinder.
- RX power, NLOS score, timestamp từng phép đo, cấu hình/calibration identity đầy đủ.
- Command cấu hình thiết bị có ACK/NACK; inventory/commissioning.
- Auto-calibration tọa độ anchor và validation/commit/rollback.
- Mission/target/geofence/trajectory và command có kiểm tra trạng thái.

Các mục này cần protocol hoặc backend tương ứng; chỉ thêm nút GUI không hoàn thành chức năng. Theo kế hoạch hiện có, phần command được triển khai sau khi đạt kiểm chứng flight hold, qua mission manager.

## D. Kiểm chứng và thứ tự thực hiện

- Chạy Python 3.12 unittest discover: 28/28 PASS. Lần chạy sandbox đầu có một lỗi quyền thư mục tạm; chạy lại ngoài sandbox đạt toàn bộ.
- Kiểm tra bổ sung bằng Tk ẩn tái hiện A1/A2; helper test tái hiện A3.
- Các thiếu sót còn lại được xác định bằng đọc mã, chưa benchmark tải dài hạn hoặc thử phần cứng.

Thứ tự đề xuất:

1. Sửa A1–A5 và thêm regression test cho các tình huống đã tái hiện.
2. Hoàn thiện replay, logging có thể tái lập và calibration profile/validation.
3. Hoàn thiện chất lượng position, biểu đồ/thống kê và reconnect.
4. Tích hợp backend UP/PX4 theo các giai đoạn trong kế hoạch hệ thống.

Ưu tiên hoàn thành ba bước đầu để GUI trở thành công cụ đo và kiểm chứng đáng tin cậy.
