# UP7000: phần chạy trên eMMC, dữ liệu ghi trên ổ ngoài

Đã tách phần mềm chạy về eMMC theo yêu cầu người dùng. Log và cache hiện
vẫn ghi trên HDD USB 2.0, chờ SSD đạt kiểm tra ghi–đọc. Không ghi thêm lên
SSD đã phát hiện sai dữ liệu.

## Cách bố trí

| Nơi lưu | Nội dung |
|---|---|
| eMMC | Ubuntu, ROS 2 Humble, mã logger UWB/PX4, workspace và binary ESKF, thư viện Python, script khởi động |
| Ổ ghi bên ngoài | Flight log, MCAP, session/config snapshot, ROS log, cache Python, dữ liệu tạm và cấu hình phiên thay đổi |

Các file phần mềm chủ yếu được đọc khi chạy. Việc cài/cập nhật có ghi vào
eMMC, nhưng dòng ghi log thường xuyên được đưa ra ổ ngoài. Hiện eMMC còn
khoảng **15,2 GB trống** sau khi đưa khoảng 0,5 GB phần mềm về.

SSD chỉ lưu file log/MCAP có thể dùng exFAT; không cần image ext4 32 GiB để
chứa toàn bộ ROS như phương án trước. Cần tách các pointer/symlink/cấu hình
Linux về eMMC hoặc RAM khi áp dụng backend SSD thực tế. Ổ SSD hiện tại chưa
đạt kiểm tra dữ liệu, nên backend ghi vẫn là HDD; không thể tháo HDD lúc này.

## Đã chuyển

Sáu đường dẫn hiện là file/thư mục thật trên eMMC:

```text
/opt/ros/humble
/home/thoa/UP4000
/home/thoa/ros2_px4_v2
/home/thoa/drone_tools
/home/thoa/onboard_env.sh
/home/thoa/drone_env.sh
```

Đã đối chiếu checksum bản sao trước khi đổi symlink và sửa bảy file prefix
sinh bởi colcon/egg-link để import từ đường dẫn eMMC ổn định. Các bản gốc
HDD được giữ nguyên. Các thư mục trạng thái ESKF, con trỏ phiên, cấu hình
GUI và build log được liên kết về HDD để tránh ghi thường xuyên vào eMMC.

`ROS_HOME`, `ROS_LOG_DIR`, `PYTHONPYCACHEPREFIX`, `TMPDIR` và `~/flight_logs`
đều trỏ về HDD. Guard đúng UUID và phụ thuộc mount vẫn được giữ: không tự
ghi log thay thế vào eMMC khi ổ ghi ngoài vắng mặt. Backup sáu symlink cũ
nằm tại `/etc/up7000-emmc-runtime-20261108/`.

## Kiểm tra đạt

- Binary ESKF đang chạy trong `/proc/<PID>/exe` thực sự nằm trên eMMC.
- `rclpy`, `rosbag2_py`, `pymavlink`, `uwb_px4_logger` import từ eMMC;
  `ldd` sau khi source môi trường không có thư viện thiếu.
- Cả ba service logger, ESKF/MCAP và điều khiển GUI đều active.
- Thử đúng giao thức GUI Log → Save: cấu hình được xác nhận cùng SHA-256,
  bag đóng không lỗi và một phiên ghi nền mới tiếp tục trên HDD.
- Native ROS 2 đọc MCAP thử, giải mã CDR đủ **47 message**. Không có Pixhawk
  hoặc TAG kết nối, nên đây là kiểm tra đường chạy/ghi và message trạng thái;
  chưa phải kiểm thử IMU/UWB/GPS thật. ESKF đang `WAIT_CONFIGURATION`.

Bằng chứng: `emmc_runtime_result.json`, `emmc_runtime_validation.json`,
`emmc_control_bag_validation.json`. Báo cáo và bằng chứng cũng lưu tại
`~/flight_logs/emmc_runtime_20261108/` trên UP7000.

## Giới hạn SSD còn lại

Bản image nguồn HDD có SHA-256 đúng, nhưng dữ liệu đọc lại từ SSD sai ở nhiều
vùng; đọc block và READ16 cũng cho cùng dữ liệu sai. Đổi vị trí chạy phần mềm
không giải quyết được lỗi toàn vẹn dữ liệu của đường ghi đó. Cần kiểm tra
SSD/controller/cáp/nguồn/driver và ghi–đọc checksum đạt trước khi dùng lưu log.
Ba file có sẵn trên SSD vẫn giữ nguyên SHA-256 và đã có backup HDD.
