# Ghi ROS 2 bag (MCAP) từ New_GUI

Ở thanh trên cùng, chọn **CSV + MCAP**, rồi bấm **Log: OFF** để bắt đầu.
Khi nút hiện **Log: ON**, GUI ghi cả UWB và PX4 đang nhận. Bấm lại để dừng
và hoàn tất file. Có thể dùng UART, UDP từ UP7000 hoặc Demo; phiên Demo được
đánh dấu `simulated` trong topic `/session/metadata` và không trộn PX4 thật
vào bag mô phỏng. Dừng log trước khi chuyển giữa Demo và phần cứng.

## Một nút ghi cả laptop và UP7000

Khi chọn **WiFi UDP :15555 (UP7000 relay)**, bấm nút log sẽ tự mở luồng nhận
UDP nếu chưa kết nối và yêu cầu UP7000 bắt đầu một phiên HDD mới:

- Laptop ghi CSV và MCAP theo lựa chọn định dạng ở thanh trên.
- UP7000 bảo đảm logger UWB/PX4 chạy, khởi động ESKF + recorder và ghi native
  MCAP trên HDD. Có IMU, UWB, odometry PX4, snapshot 14 loại message PX4,
  trạng thái ESKF và odometry ESKF khi bộ lọc có nghiệm hợp lệ.
- GUI gửi RESUME và bật các luồng telemetry Range/RxQuality/diagnostics hiện có
  của TAG. Bố trí anchor, hiệu chuẩn và burst preset không bị thay đổi.
- **Log: STARTING**: đang đợi UP7000 xác nhận. **Log: ON** cùng dòng trạng thái
  **Recording: laptop + UP7000 HDD**: đã xác nhận cả hai phía.
- **Log: PARTIAL**: chưa xác nhận được UP7000 hoặc đã mất liên lạc với phiên
  đang ghi. Lỗi hiển thị ở thanh trạng thái; dữ liệu local vẫn được giữ.

Bấm dừng/lưu: GUI hiện **Log: SAVING**, đóng file local, đợi native MCAP trên
HDD hoàn tất rồi cho UP7000 chuyển sang **phiên ghi nền mới**. Khi đóng cửa sổ
trong lúc đang ghi, GUI cũng hoàn tất quy trình này. Nếu mất kết nối mạng,
UP7000 tiếp tục ghi dữ liệu của nó; GUI không báo đã lưu thành công khi chưa
có phản hồi xác nhận.

File `onboard_recording.json` trong thư mục log laptop lưu mã phiên chung,
đường dẫn bag trên HDD và kết quả xác nhận. Lựa chọn **CSV only** chỉ điều
khiển định dạng trên laptop; phiên UP7000 vẫn ghi MCAP. Demo/UART local không
tự điều khiển service UP7000. Ghi hai phía dùng Tailscale và kênh UDP 15558
từ laptop đã cấu hình, không cần nhập mật khẩu SSH mỗi lần.

ESKF hiện vẫn báo **WAIT_CONFIGURATION** vì hình học anchor, lever arm và
hướng ban đầu chưa được xác nhận. Node được chạy và đầu vào được ghi; vị trí
ESKF chỉ xuất khi có cấu hình và dữ liệu hợp lệ.

## Gửi cấu hình GUI xuống ESKF khi bấm Log

1. Mở **3. Rig Layout & Diag**, nhập tọa độ **X/Y/Z (m)** và bật những anchor
   thực sự sử dụng trong bảng rig.
2. Trong **UP7000 ESKF CONFIGURATION**, nhập **Offset X/Y/Z (m)** bằng vị trí
   TAG trừ vị trí IMU Pixhawk: X hướng trước, Y hướng trái, Z hướng lên.
   **Initial yaw (°)** là hướng mũi drone so với trục +X của rig, dương quay
   về +Y khi nhìn từ trên xuống. Đây không phải yaw compass Bắc của PX4.
3. Chỉ chọn **Use measured rig, TAG–IMU offset and heading for ESKF** sau khi
   đã kiểm tra các số đo. Các ô bên dưới cho phép chỉnh nhiễu range, số anchor
   tối thiểu, thời gian khởi tạo và ngưỡng quá hạn. Có thể giữ mặc định trước.
4. Chọn nguồn **WiFi UDP :15555 (UP7000 relay)** rồi bấm **Log: OFF**.
   GUI gửi cấu hình, UP7000 lưu phiên bản trên HDD và khởi động lại ESKF với
   cấu hình đó. Chờ **Log: ON** và dòng **GUI config verified**: GUI đã đối chiếu
   SHA-256 với chính node ESKF đang chạy. Không cần SSH hay sửa YAML thủ công.

ESKF trên UP7000 là bộ ước lượng 3D riêng. Các lựa chọn bộ giải 2D/3D và
lọc vị trí ở sidebar vẫn điều khiển bộ giải trên laptop. Cột **Bias/σ** từng
anchor được giữ trong bản cấu hình để truy vết; ESKF dùng range đã hiệu chuẩn
từ TAG và **Range σ (m)** chung, không trừ host bias thêm lần nữa.

GUI khóa bảng rig và các ô ESKF trong lúc ghi để giữ đúng cấu hình của phiên.
Muốn thay đổi, dừng/lưu log, sửa rồi bắt đầu phiên mới. Thay tọa độ, anchor
bật/tắt, offset hoặc yaw sẽ bỏ dấu xác nhận số đo để người dùng kiểm tra lại.
Cấu hình được nhớ khi mở lại GUI; trạng thái xác nhận chỉ được khôi phục nếu
hình học rig vẫn khớp. Khi dừng, phiên ghi nền tiếp tục dùng cấu hình vừa áp
dụng; khởi động lại UP7000 cũng dùng phiên bản cuối đã nhận.

Không chọn ô xác nhận vẫn ghi IMU/UWB/PX4 đầy đủ và ESKF báo
**WAIT_CONFIGURATION**. Cấu hình đã xác nhận phải có đủ anchor đang bật
(mặc định 6) và không đồng phẳng. Node còn kiểm tra tập anchor thực sự nhận
được, dữ liệu hợp lệ và trạng thái đứng yên trước khi khởi tạo. **Log: ON**
xác nhận phiên ghi/cấu hình; chưa đồng nghĩa bộ lọc đã có nghiệm vị trí.

`onboard_recording.json` trên laptop giữ toàn bộ cấu hình đã gửi và SHA-256.
Phiên HDD giữ `config/gui_config.json`, `rig_onboard.txt`, `eskf.yaml`,
`physical_setup.json` và mã cấu hình trong `session.json` cùng topic
`/uwb_eskf/status`. Gói điều khiển được chia phần tối đa 1.200 byte, tự gửi
lại và đối chiếu checksum khi mất gói. Cấu hình sai bị từ chối trước khi
thay phiên đang chạy; lỗi hoặc mã cấu hình không khớp được báo **PARTIAL**.

Mỗi phiên nằm trong `Software/New_GUI/logs/uwb_fast_<thời gian>_<nguồn>/`:

```text
uwb_fast_log.csv             # dữ liệu hiện có để replay trên GUI
session_metadata.json        # cấu hình rig, bộ lọc, hiệu chuẩn
onboard_recording.json       # liên kết phiên HDD khi dùng UP7000 qua UDP
rx_quality.jsonl
diag.jsonl
rosbag2/
  data_0.mcap                # message ROS 2, mã hóa CDR
  metadata.yaml              # thông tin rosbag2
  recording_status.json     # trạng thái hoàn tất, lỗi, số event bị rơi
  topics.json                # topic, kiểu message và số message
  README_ROS2.md             # lệnh sử dụng bag này
```

Trang **4. Log Replay & Analysis** có **Log Folder** để mở thư mục phiên vừa
ghi, và **Plot MCAP...** để chọn file MCAP mở bằng PlotJuggler. Trên laptop
Windows hiện tại, nút này sử dụng ROS 2 Humble/PlotJuggler đã cài trong WSL
`Ubuntu-22.04`. Dừng ghi trước khi mở MCAP. Nếu PlotJuggler hỏi loader/topic,
chọn ROS 2/MCAP và các topic cần vẽ, rồi kéo trường số vào ô đồ thị.

## Các topic dùng để plot

| Dữ liệu | Topic / trường |
| --- | --- |
| Khoảng cách hợp lệ, đã hiệu chuẩn của anchor 1 | `/uwb/anchor_1/range_m` → `data` (m) |
| Khoảng cách chẩn đoán, kể cả chưa hiệu chuẩn | `/uwb/anchor_1/diagnostic_range_m` → `data` (m) |
| XYZ từ bộ giải của GUI | `/uwb/position` → `point.x`, `point.y`, `point.z` (m) |
| Vận tốc UWB | `/uwb/velocity` → `vector.x`, `vector.y`, `vector.z` (m/s) |
| Sai số dư | `/uwb/rms_m` → `data` (m) |
| Chu kỳ và thời gian của TAG | `/uwb/cycle_seq`, `/uwb/tag_time_us`, `/uwb/boot_id`, `/uwb/period_us` |
| Góc PX4 | `/px4/attitude/roll`, `/pitch`, `/yaw` → `data` (rad) |
| Vị trí PX4 | `/px4/local_position_ned/x`, `/y`, `/z` → `data` (m, trục NED) |
| IMU PX4 | `/px4/highres_imu/xacc`, `/yacc`, `/zacc`, `/xgyro`, … → `data` |
| Các trường PX4 khác | `/px4/<tên message chữ thường>/<tên trường>` |

Anchor 2–8 có cùng cách đặt tên. Bag chỉ dùng message ROS chuẩn
`std_msgs` và `geometry_msgs`, không cần biên dịch package message riêng.
Trường dạng mảng dùng `Float64MultiArray`, dữ liệu đầy đủ còn được giữ
trong các topic JSON `String` `/uwb/telemetry/*`, `/px4/*/message` và
`/up7000/status_parts`. Các con số PX4 giữ nguyên đơn vị MAVLink; không tự
chuyển NED/FRD sang ENU/FLU. `uwb_map` sử dụng các trục rig của GUI.

Không có phép đo hợp lệ thì không tạo khoảng cách bằng 0. Không có nghiệm
vị trí thì không tạo topic XYZ giả. Tên topic chỉ xuất hiện sau mẫu đầu tiên.

## Đọc và phát lại bằng ROS 2

Trong WSL Ubuntu-22.04:

```bash
source /opt/ros/humble/setup.bash
ros2 bag info '/mnt/d/Drone Project/Github Clone UWB Test/Software/New_GUI/logs/<phiên>/rosbag2'
ros2 bag play '/mnt/d/Drone Project/Github Clone UWB Test/Software/New_GUI/logs/<phiên>/rosbag2'
```

Trên máy ROS 2 khác cần plugin MCAP tương ứng, ví dụ với Humble:

```bash
sudo apt install ros-humble-rosbag2-storage-mcap
```

Khi đang phát bag, có thể dùng `ros2 topic echo`, `rqt_plot` (nếu đã cài),
hoặc PlotJuggler. PlotJuggler cũng mở trực tiếp MCAP, không cần chạy `bag play`.

## Thời gian và tính đầy đủ

- Timestamp của bag/header là thời điểm laptop nhận dữ liệu, dùng một mốc
  wall-clock + monotonic cho cả phiên. Thời gian boot của TAG/PX4 được giữ
  riêng trong payload; không coi các đồng hồ thiết bị là đã đồng bộ.
- UWB được ghi theo từng message GUI nhận, không giảm xuống tần số vẽ màn hình.
- PX4 qua tab Onboard là các snapshot khoảng **4 Hz**, không phải toàn bộ
  MAVLink tốc độ gốc. Các payload không đổi được loại trùng; snapshot quá
  hạn 2 giây không được ghi thành dữ liệu số mới. Các phần status gốc vẫn
  được giữ để kiểm tra mất phần/gói.
- Đây là bản ghi trên laptop. Gói mất trên đường truyền không thể được GUI
  khôi phục. Các log gốc trên HDD UP7000 tiếp tục chạy độc lập.
- Luồng ghi MCAP chạy nền, có hàng đợi giới hạn. Nút log báo **WARN/ERROR**
  khi rơi event hoặc lỗi đĩa. Kiểm tra `recording_status.json`: `closed: true`,
  `error` rỗng và `dropped_events: 0` trước khi dùng như phiên hoàn chỉnh.

Thư viện ghi sử dụng [MCAP ROS 2 Python](https://mcap.dev/docs/python/ros2_noenv_example),
với schema ROS 2 và mã hóa CDR; cách đọc/phát dùng
[rosbag2 MCAP storage](https://github.com/ros-tooling/rosbag2_storage_mcap).
