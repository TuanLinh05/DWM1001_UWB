# UP4000 — sửa truyền trạng thái và kiểm thử thực tế 07/11/2026

## Đã thực hiện

- Khôi phục adapter SEGGER J-Link USB/VCOM: trước reset, đúng baud 460800 và đọc độc quyền vẫn không có frame hợp lệ; sau reset nhận 2.244 frame trong 8 giây, không lỗi CRC/độ dài. Không xác định được nguyên nhân adapter rơi vào trạng thái lỗi.
- Giới hạn toàn bộ UDP trạng thái ở 1.200 byte, bao gồm JSON envelope và số thứ tự/timestamp. Chia dictionary theo đường dẫn; giá trị nguyên khối quá dài được chia fragment và chỉ cập nhật khi nhận đủ. Test có Unicode và giới hạn 400 byte.
- GUI theo dõi thời gian nhận riêng cho từng trường, cộng tuổi dữ liệu tại nguồn. Sau 2 giây thiếu cập nhật, ẩn các giá trị PX4 cũ, cảnh báo STALE theo message và đánh dấu cây dữ liệu. Các nhóm host/session/logger có cảnh báo riêng. Không hiển thị disarmed khi HEARTBEAT đã cũ.
- Đổi CRC-16/CCITT-FALSE sang `binascii.crc_hqx`, đồng bộ parser vendored. Phép đo cục bộ 1.000 lần CRC trên 1.024 byte: 0,890 s → 0,00229 s. Không xác nhận giảm CPU tổng của relay: trung bình hai đợt khoảng 57.52% và 58.03% một nhân; thời lượng/điều kiện hai đợt khác nhau.
- Thêm `scripts/reset_tag_usb.py`: chọn đúng SEGGER J-Link, từ chối khi nhiều thiết bị mà chưa chọn serial hoặc cổng vẫn bị ứng dụng khác dùng; tạm dừng logger và khởi động lại trong finally nếu trước đó service chạy. Công cụ đã được chép lên drone, không tự reset theo watchdog.
- Đã sao lưu các file cũ, xác minh SHA-256, triển khai sender và parser lên UP4000; service được nạp lại và vẫn active sau test.

## Phép đo

PC 192.168.10.7 nhận trực tiếp WiFi từ UP4000 192.168.10.23, PIF_CLUB_5G, power save off. SSH qua Tailscale chỉ dùng chẩn đoán. Bộ đo đăng ký địa chỉ ground bằng UDP rỗng, không gửi lệnh điều khiển PX4/TAG. UWB, PX4 và panel/parser GUI chạy cùng lúc; đây là thử nghiệm tại bàn, không phải chuyến bay.

| Đợt | Thời lượng | Kết quả |
|---|---:|---|
| Sau chia gói/reset VCOM, trước CRC tối ưu | 600 s | Bản bắt gói thu được 16,059 gói trạng thái gửi, khớp 16,054 gói nhận; chênh 5 (~0,031%). Không có thống kê drop của socket bắt gói ở đợt này, nên chỉ coi đây là số liệu từ bản capture, không chứng minh capture đầy đủ. UWB nhận 175,784 Burst. |
| Sau triển khai cuối | 600 s | Log nhận kéo dài 599.89 s; nhận 15,337 gói, đủ 14 loại PX4; gói lớn nhất 1121 byte. Có 38 chu kỳ seq không xuất hiện tại PC. Chuyển metadata gửi qua SSH bị timeout; không suy ra tỷ lệ mất từng gói của đợt này. Log nhận đã được giữ lại. |
| Đối chiếu cuối, metadata lưu nén trên drone | 120 s | Trạng thái 3,468/3,468, UWB 10,062/10,062: không mất UDP trong mẫu này. Socket bắt gói đọc 70,134 packet, drop = 0. Không thấy đảo thứ tự UWB. |

Đợt đối chiếu cuối dùng SHA-256 của từng payload ở đầu gửi/nhận; loại các seq biên chưa trọn chu kỳ. UWB được căn biên bằng hash payload đầu/cuối. Metadata được lưu trước khi tải. Kết quả 0% trong 120 giây không bảo đảm 0% khi điều kiện mạng thay đổi.

## Số liệu cuối và phần còn hạn chế

- Trạng thái: 478/478 chu kỳ nhận đầy đủ; đủ 14 loại PX4, khoảng 4 Hz. Khoảng cách lớn nhất giữa cập nhật message là 0.485 s. Parser trạng thái không lỗi.
- Payload UDP trạng thái: 379–1118 byte trong 120 s cuối; không gói vượt 1.200. Trong lượt nhận 10 phút sau triển khai, lớn nhất 1121 byte.
- UWB cuối: 38,716 Burst, 322.63 Burst/s; 31 khoảng hụt cycle seq, không có cycle trùng. Dù UDP không mất trong mẫu này, parser trên UP4000 tăng 45 lỗi CRC và 2 lỗi độ dài; parser PC có 46 CRC và 2 lỗi độ dài. Điều này chứng minh vẫn có lỗi trước đường WiFi. Chưa xác định được lỗi phát sinh trong UART, adapter hay firmware; không coi luồng UWB là đầy đủ tuyệt đối.
- ULog cuối: thêm 21,201 chunk, không tăng mất chunk/dropout trong 120 s. Tuy nhiên lượt 10 phút sau triển khai đã tăng **953 dropped chunk và 15 dropout event**, trên 104.230 chunk nhận. Cần giữ log SD của Pixhawk làm bản đối chiếu và tiếp tục xác định nguyên nhân; chưa sửa được dropout này.
- Mất dữ liệu WiFi có biến động giữa các đợt; cảnh báo stale giúp nhận biết dữ liệu cũ, không khôi phục gói đã mất.

## Kiểm thử phần mềm và hiển thị

- 116 test đạt, 20 subtest đạt, 2 test bỏ qua. Test bao gồm New_GUI, logger, protocol v1/v2, giới hạn toàn envelope, ghép fragment đảo thứ tự/mất phần, tuổi nguồn, mất chọn lọc message, im lặng toàn luồng, nhận diện USB và khởi động lại service khi ioctl reset lỗi.
- File golden thực do firmware tạo: 12 frame được giải mã, 0 lỗi CRC/độ dài/version/decode.
- Đã render và kiểm tra cảnh báo STALE bằng dữ liệu giả với font Windows. Ảnh `stale_preview.png` là tình huống mô phỏng. `up4000_panel.png` là render lại từ dữ liệu thật đã ghi; không phải ảnh cửa sổ người dùng đang mở.

## File và sử dụng

- Sender: `UP4000/ros2_ws/src/uwb_px4_logger/uwb_px4_logger/status_reporter.py`.
- GUI: `Software/New_GUI/up4000_panel.py`; mở lại New_GUI để nạp thay đổi.
- Parser nguồn: `Software/UWB_UART_GUI/telemetry_protocol.py`, vendor đã đồng bộ bằng `UP4000/scripts/sync_vendor.py`.
- Khôi phục khi VCOM lại có nhiều byte nhưng gần như không frame hợp lệ: `sudo python3 ~/UP4000/scripts/reset_tag_usb.py` (ngắt logger ngắn, tạo phiên mới). Đây không phải cách chữa mọi CRC lẻ tẻ.
- Bằng chứng gọn: `baseline_summary.json`, `final_10min_receive_summary.json`, `final_summary.json`, `uwb_order.json`.
- Log thô/bộ đo nằm ở `.tmp_up4000_test/results_long`, `results_final`, `results_check` trong repo. Thư mục chuyến bay cuối trên drone: `/home/dronepif/flight_logs/20261107_131656`.
