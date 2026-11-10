# ESKF và MCAP trên UP7000

**Cập nhật cách lưu trữ:** ROS/logger/ESKF hiện chạy từ eMMC; log, MCAP,
cache và trạng thái phiên vẫn trên HDD USB 2.0. SSD chưa được dùng ghi log
do lỗi đọc lại dữ liệu. Xem [báo cáo tách phần chạy và dữ liệu](../migration/20261108_eskf/EMMC_RUNTIME_REPORT.md).

Đã chuẩn bị [gói thẻ dữ liệu 64 GB](../card64_kit/README_VI.md) tại
`/opt/up7000-card64-kit` trên UP7000. Thẻ ext4 sẽ thay HDD sau khi kiểm tra
ghi–đọc và kích hoạt; hiện vẫn ghi HDD. [Kết quả chuẩn bị](../migration/20261108_eskf/CARD64_PREPARATION_REPORT.md).

Hệ thống đã được cài thành service `uwb-eskf-recorder`, tự khởi động cùng
`uwb-px4-logger` khi HDD có mặt. Node nhận IMU thật và UWB ngay trên UP7000;
GUI/laptop không phải thành phần của luồng ước lượng hoặc ghi bag này.

GUI nay có thể điều khiển phiên ghi: với đầu vào WiFi UDP, nút log khởi động
logger/ESKF/recorder và tạo phiên HDD có cùng mã với log laptop. Dừng/lưu trên
GUI đóng phiên đó rồi bắt đầu một phiên tự ghi nền mới, theo lựa chọn của
người dùng. Các bản ghi không phụ thuộc GUI tiếp tục hoạt động khi laptop
không kết nối. Thao tác trên GUI có phản hồi xác nhận và đánh dấu PARTIAL nếu lỗi.

Mỗi lần bắt đầu log qua UDP, GUI gửi snapshot tọa độ anchor bật/tắt, offset
TAG–IMU theo body FLU, yaw ban đầu và các tham số ESKF. Helper kiểm tra schema,
giới hạn số, hình học 3D và SHA-256 rồi lưu phiên bản bất biến trên HDD. Phiên
ESKF mới chỉ được xác nhận khi chính node C++ báo đúng mã cấu hình. Sửa cấu
hình tại **3. Rig Layout & Diag → UP7000 ESKF CONFIGURATION**, không cần SSH.

**Trạng thái hiện tại: `WAIT_CONFIGURATION`.** Người dùng chưa xác nhận
tọa độ anchor. Thông tin TAG ở sau tâm drone 76,3 mm và cao 50,5 cm so với
mặt đất được lưu nguyên văn ý nghĩa; chưa dùng các số đó làm lever arm TAG–IMU.
Pixhawk được xác nhận có mũi tên hướng về phía trước. IMU và khoảng cách UWB
vẫn được ghi để xử lý lại sau khi có cấu hình đúng.

## Luồng dữ liệu

```text
TAG ────── uwb_relay ── /uwb/anchor_N/range ──┐
                                            ├─ uwb_eskf ── /uwb_eskf/odom
Pixhawk ── px4_logger ── /px4/imu_frd ────────┘                  │
             │                         tất cả đầu vào/trạng thái ├─ MCAP trên HDD
             └─ tlog/ULog hiện có trên HDD                       │
```

- Logger hiện có tiếp tục là tiến trình đọc cổng Pixhawk. `HIGHRES_IMU` được
  chuyển thành `sensor_msgs/Imu` với trục FRD tại `/px4/imu_frd`; khoảng 50 Hz
  theo stream PX4 hiện tại. Orientation không đo ở topic này, đánh dấu bằng
  `orientation_covariance[0] = -1`.
- ESKF dùng `imu:=mavlink` và chuyển FRD sang FLU đúng một lần. Header IMU
  giữ khoảng thời gian mẫu thiết bị, neo vào thời gian nhận trên UP7000.
  `/px4/imu_device_time_us` và log TIMESYNC gốc được giữ để căn thời gian offline.
  Đây chưa phải đồng bộ timestamp phần cứng.
- Range đầu vào ESKF là DS calibrated+valid từ relay. `ignore_rig_bias: true`
  tránh trừ bias cũ thêm lần nữa. Không đưa diagnostic range chưa đạt điều kiện
  thành range hợp lệ.
- Đầu ra là `nav_msgs/Odometry` trong `uwb_rig`, body FLU `px4_imu_flu`, và
  JSON trạng thái tại `/uwb_eskf/status`. Chúng được dùng cho ghi log/đánh giá.
- IMU quá hạn 0,2 s hoặc range được chấp nhận quá hạn 0,5 s làm dừng xuất
  odometry và khởi tạo lại khi đủ dữ liệu. Dữ liệu NaN, timestamp cũ/đảo và
  nghiệm khởi tạo không hữu hạn bị loại.
- MCAP cũng lưu `/px4/snapshot` để giữ đủ các message PX4 ở tần số snapshot.
  Mavlink tlog/ULog gốc tiếp tục được ghi độc lập bởi logger hiện có.

## Nơi lưu

HDD yêu cầu UUID `896b17fa-8fe0-4111-853b-57b93ec87c60` tại `/mnt/droneup`.

Từ 08/11/2026, cổng HDD hiện tại được giới hạn **USB2 480 Mb/s** bằng
`up7000-hdd-usb2.service` để thử giảm nhiễu GPS từ USB3/HDD. Chính sách chạy
trước fsck và mount, tự áp dụng khi khởi động; chỉ áp dụng cho cổng vật lý đã
kiểm tra (controller `0000:00:14.0`, port 1). Đổi cổng HDD cần kiểm tra lại.
Kiểm thử đọc/ghi checksum đạt; GPS chưa được đo lại vì Pixhawk chưa kết nối.
Chi tiết: `../migration/20261108_eskf/USB2_REPORT.md`.

Lần thử thay HDD bằng SSD ngày 08/11 chưa được áp dụng: image ghi lên SSD đọc
lại sai dữ liệu. Hệ thống đã khôi phục chạy trên HDD; cả 3 file SSD có sẵn đã
được đối chiếu SHA-256 và giữ nguyên. SSD đã unmount. Xem
[báo cáo chuyển SSD](../migration/20261108_eskf/SSD_MIGRATION_REPORT.md).

```text
/mnt/droneup/onboard_20261107/thoa/flight_logs/eskf/<UTC_timestamp>_<PID>/
  bag/                         # MCAP + metadata.yaml của rosbag2
  config/                      # tham số, rig, thông tin lắp đặt, DDS, supervisor
  session.json                 # hash, lệnh chạy, trạng thái ESKF, tiến trình
  estimator.log
  recorder.log
```

Phiên hiện tại luôn được trỏ bởi `/home/thoa/drone_tools/eskf/latest_session`.
`session.json.recording_context` cho biết phiên `gui` hoặc `autonomous`, mã
phiên và tên thư mục trên laptop. `onboard_recording.json` phía laptop lưu
đường dẫn phiên HDD tương ứng.
Mỗi phiên sao chép cấu hình và lưu SHA-256. Native rosbag2 tách file sau 10 phút
hoặc 1 GiB. Toàn bộ dữ liệu, cache ROS/Python và log ứng dụng nằm trên HDD;
file unit systemd nhỏ nằm trong `/etc/systemd/system`.

Service kiểm tra đúng HDD/UUID và tối thiểu 2 GiB trống. Giám sát cũng xác nhận
recorder có subscription IMU, trạng thái và odometry, cùng heartbeat của ESKF.
Fast DDS được giữ ở UDP với peer localhost; `maxInitialPeersRange=32` giúp
các tiến trình khám phá đủ nhau. Tham số này được mô tả trong
[tài liệu Fast DDS 2.6](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html).

Kênh điều khiển riêng chạy qua `uwb-recording-control.service`, UDP 15558;
hiện chỉ nhận từ laptop Tailscale `100.77.150.59` và loopback. Thao tác service
dùng helper root-owned `/usr/local/libexec/up7000-recording-action` với sudoers
giới hạn vào helper đó. Nó chỉ nhận `start`/`finish` và mã phiên, điều khiển hai
unit cố định và cấu hình ESKF theo schema cho phép; không nhận lệnh shell hay
đường dẫn tùy ý. Luồng điều khiển chạy riêng với luồng USB/telemetry. Lệnh lặp
dùng cùng request ID không tạo thêm phiên. Payload cấu hình lớn được chia
thành gói ≤1.200 byte, có checksum, TTL, giới hạn bộ đệm và gửi lại.

## Kiểm tra và lấy bag

SSH vào UP7000:

```bash
systemctl status uwb-eskf-recorder --no-pager
cat ~/drone_tools/eskf/latest_session/session.json
```

Để đóng phiên hiện tại cho việc tải/xem, đồng thời bắt đầu phiên mới:

```bash
sudo systemctl restart uwb-eskf-recorder
```

Chọn thư mục phiên **đã đóng**, có `closed: true` và `error` rỗng trong
`session.json`. Sau khi source ROS:

```bash
source ~/onboard_env.sh
source ~/ros2_px4_v2/install/setup.bash
ros2 bag info /mnt/droneup/onboard_20261107/thoa/flight_logs/eskf/<phiên>/bag
```

Bag có thể được sao chép về laptop rồi phát lại hoặc mở trong PlotJuggler.
Giữ cả thư mục `bag/` cùng `config/` và `session.json` để biết chính xác
cấu hình tạo ra kết quả. Các bag dưới `drone_tools/eskf/tests/synthetic_*`
là dữ liệu mô phỏng kiểm thử, tách bằng ROS_DOMAIN_ID=187.

## Thông tin còn cần để có vị trí thật

1. Tọa độ XYZ của từng anchor đang sử dụng, cùng đơn vị và trục rig.
2. Vị trí TAG trừ vị trí IMU trong trục body FLU (x trước, y trái, z lên).
   Khoảng cách từ TAG đến tâm drone và độ cao TAG so với mặt đất chưa đủ.
3. Góc hướng ban đầu của drone so với trục X rig (`yaw0_deg`).

Cấu hình ban đầu nằm trên HDD tại `~/drone_tools/eskf/config/`:

- `rig_onboard.txt`: bản tham chiếu từ GUI, được ghi rõ chưa xác nhận.
- `physical_setup.json`: số đo người dùng đã cung cấp và những mục còn thiếu.
- `eskf.yaml`: `configuration_confirmed: false`, lever tạm bằng 0 chưa được dùng
  để xuất vị trí, bộ lọc IMU và ngưỡng kiểm tra.

Sau khi có số đo đúng, cập nhật trong GUI rồi chọn **Use measured rig, TAG–IMU
offset and heading for ESKF** và bấm Log. GUI khóa các ô trong lúc ghi. Cấu
hình đã xác nhận phải có đủ anchor bật và không đồng phẳng; tập anchor thực
sự nhận được cũng phải đạt điều kiện hình học. Mặc định cần 6 anchor hợp lệ
trong cửa sổ 50 ms để khởi tạo; 2 anchor giữ trạng thái chờ. Không chọn ô xác
nhận vẫn ghi đầu vào, không xuất odometry ESKF.

Các phiên bản GUI được ghi tại `~/drone_tools/eskf/config_versions/<SHA256>/`:
`gui_config.json`, `rig_onboard.txt`, `eskf.yaml`, `physical_setup.json`.
`active_config.json` chọn phiên bản đang dùng bằng thay thế nguyên tử.
Supervisor sao chép bản cấu hình vào từng phiên HDD; `session.json` và topic
`/uwb_eskf/status` giữ SHA-256. Laptop giữ bản đã gửi trong
`onboard_recording.json`. Cấu hình sai bị từ chối trước khi đổi phiên đang chạy.
Phiên tự ghi nền và lần khởi động sau tiếp tục dùng cấu hình cuối đã áp dụng.
Cột host Bias/σ được giữ trong JSON; ESKF dùng range đã hiệu chuẩn từ TAG và
nhiễu `sig_r` chung, không áp dụng host bias thêm lần nữa.

## Build lại từ nguồn

`uwb_eskf/` trong thư mục này là nguồn ESKF đã triển khai. Trên UP7000 nó nằm
ở `~/ros2_px4_v2/src/uwb_eskf`; phần IMU logger nằm trong package
`~/UP4000/ros2_ws/src/uwb_px4_logger`. Các bản trước thay đổi nằm trên HDD tại
`~/drone_tools/eskf/backups/`.
