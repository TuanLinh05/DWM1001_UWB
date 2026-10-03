# PlotJuggler + ROS 2 để kiểm tra UWB trên Windows/WSL2

## Phần mềm đã cài ngày 2026-10-02

- WSL distro **Ubuntu-22.04**, user `bling`, Ubuntu 22.04.5, WSL2 + WSLg.
- ROS 2 **Humble Base**; trước khi cài chưa tìm thấy ROS 2 trong /opt/ros hoặc workspace của Ubuntu 22.04/20.04.
- PlotJuggler **3.17.2**, ROS plugins **2.3.1**, rosbag2 MCAP storage **0.15.17**.
- Nguồn apt ROS 2 chính thức qua ros2-apt-source 1.3.0; SHA-256 của gói nguồn khớp release chính thức. Cài 361 gói mới, cập nhật năm dependency, không gỡ gói; dung lượng thêm khoảng 410 MB.
- Đã kiểm tra executable, version, import rclpy/pyserial, và `ros2 bag info` đọc đủ 50.606 message của file tester.
- Đã khởi chạy PlotJuggler qua WSLg cùng đường dẫn MCAP. Log khởi động xác nhận ROS2 DataLoader/DataStreamer và MCAP DataLoader được nạp. Việc chọn topic/đường plot trong giao diện do người dùng thực hiện.

Hướng cài/khởi chạy được công bố tại [ROS plugins của PlotJuggler](https://github.com/PlotJuggler/plotjuggler-ros-plugins). ROS 2 Humble dùng Ubuntu 22.04 theo [hướng dẫn ROS chính thức](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst).

## 1. Mở PlotJuggler

Mở Ubuntu 22.04 từ Windows Terminal hoặc PowerShell:

```powershell
wsl -d Ubuntu-22.04
```

Trong terminal Ubuntu:

```bash
source /opt/ros/humble/setup.bash
ros2 run plotjuggler plotjuggler
```

Tên đúng là **plotjuggler**, không phải `plotjugger`. Lệnh source cần chạy ở mỗi terminal Ubuntu mới. Không sửa .bashrc trong lượt cài này.

Có thể mở từ PowerShell bằng script đã lưu trong repo:

```powershell
wsl -d Ubuntu-22.04 -- bash '/mnt/d/Drone Project/Github Clone UWB Test/Software/UWB_UART_GUI/ros2/start_plotjuggler.sh'
```

WSLg hiển thị cửa sổ Linux trên desktop Windows. [Tài liệu Microsoft](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

## 2. Xem lại file tester trước

Lệnh PowerShell:

```powershell
wsl -d Ubuntu-22.04 -- bash '/mnt/d/Drone Project/Github Clone UWB Test/Software/UWB_UART_GUI/ros2/start_plotjuggler.sh' --datafile '/mnt/c/Users/vutua/Downloads/uwb_20261001_200355/uwb_20261001_200355/uwb_20261001_200355_0.mcap'
```

Nếu xuất hiện hộp chọn loader, chọn loader có hỗ trợ MCAP/ROS2. Nếu xuất hiện danh sách topic, chọn `/uwb/anchor_2/range` đến `/uwb/anchor_7/range`, rồi xác nhận tải. Tại danh sách series bên trái, tìm các trường dạng `/uwb/anchor_*/range/range` và kéo chúng vào ô plot. Có thể kéo nhiều series vào cùng ô để so sánh. Tên hiển thị có thể được thêm prefix tùy lựa chọn khi import.

Zoom một đoạn 1–2 giây và xem từng điểm dữ liệu để quan sát khoảng cách timestamp. File tester có median interval khoảng 200 ms và trung bình 4,62–4,69 Hz mỗi anchor. Đường nối điểm không phải mẫu đo bổ sung.

Kiểm tra thống kê bằng ROS:

```bash
source /opt/ros/humble/setup.bash
ros2 bag info /mnt/c/Users/vutua/Downloads/uwb_20261001_200355/uwb_20261001_200355
```

Có thể replay bag khi cần học thao tác stream, nhưng đây là phát lại dữ liệu cũ. Với tốc độ replay 1×, topic khoảng 5 Hz; tăng tốc replay sẽ tăng message/s trên PC mà không tạo phép đo UWB mới. Không dùng replay để xác nhận 100/200 Hz của phần cứng.

## 3. Đường đo trực tiếp đã chuẩn bị

```text
TAG -> UART COM14 trên Windows -> uart_udp_relay.py
    -> UDP trong máy tới WSL -> uwb_ros_bridge.py
    -> topic ROS -> ros2 topic hz / rosbag2 / PlotJuggler
```

Đường này dùng COM trên Windows nên chưa cần chuyển USB sang WSL. Hiện máy đã có usbipd nhưng chưa thấy TAG/COM14; chưa thực hiện attach USB, nạp firmware hay đo trên phần cứng.

- Relay gửi một frame TAG hợp lệ cho mỗi datagram, giữ nguyên type/sequence/timestamp/payload và ghi lại raw frame. Parser kiểm tra CRC. Nó gửi PING, GET_DEVICE_INFO và SET_TELEMETRY=0x07 cho phiên; **không thay timing ranging**, không lưu NVS, không nạp firmware.
- Bridge publish một lần khi nhận kết quả DS mới trong RANGE_BURST 0x18. Deduplicate theo boot_id/cycle_seq; không có timer tạo phép đo. Chỉ log trạng thái dùng nhịp một giây.
- RANGE snapshot 0x01 có topic riêng, nên có thể đo đồng thời nhịp burst và snapshot trên cùng phiên.
- `Range.header.stamp` là thời điểm bridge nhận dữ liệu trên PC. Timestamp TAG được giữ riêng trong `/uwb/tag_time_us` và metadata; không trình bày clock TAG như clock epoch đã đồng bộ.
- Topic diagnostic giữ giá trị range trong payload: raw khi CAL_OK=0, corrected khi CAL_OK=1. Topic `/range` cho sử dụng thực tế chỉ publish khi DS, RADIO_OK, CAL_OK và VALID.
- sensor_msgs/Range không có enum radiation cho UWB; adapter này chỉ dùng header/range để đối chiếu. Cờ và danh tính chu kỳ nằm trong topic metadata.

## 4. Trình tự tự đo 100 Hz

### A. Chuẩn bị phần cứng và timing

1. Cắm TAG, bật các anchor, kiểm tra đúng COM. Đóng GUI UART và các chương trình đang giữ COM14.
2. Đặt TAG đủ xa các anchor để tránh trường hợp ToF sát 0 như lần A1 trước.
3. Từ PowerShell ở root repo, đặt profile 100 Hz đã có log hoạt động đủ tám anchor:

```powershell
Set-Location 'D:\Drone Project\Github Clone UWB Test'
python '.\Software\UWB_UART_GUI\uwb_command.py' --port COM14 set-burst --profile hz100 --margin 1200
```

Không reset TAG sau lệnh này: timing nằm trong RAM và reset có thể trả về profile nhanh mặc định của image. 100 Hz là bước so sánh hai luồng; mục tiêu 200 Hz vẫn cần tối ưu/đo riêng.

### B. Terminal Ubuntu số 1: nhận UART relay và publish ROS

```bash
source /opt/ros/humble/setup.bash
cd '/mnt/d/Drone Project/Github Clone UWB Test/Software/UWB_UART_GUI/ros2'
python3 uwb_ros_bridge.py
```

Terminal sẽ báo đang nghe UDP 15555. Chưa có frame thì count vẫn bằng 0, không tự tạo dữ liệu.

### C. PowerShell: đọc COM và gửi tới WSL

```powershell
Set-Location 'D:\Drone Project\Github Clone UWB Test'
$uwbWslIp = ((wsl -d Ubuntu-22.04 -- hostname -I).Trim() -split '\s+')[0]
python '.\Software\UWB_UART_GUI\ros2\uart_udp_relay.py' --port COM14 --host $uwbWslIp
```

WSL phải đang chạy bước B trước khi lấy IP. Relay in đường dẫn capture trong `Software/UWB_UART_GUI/Output/ros2_logs/`, firmware identity, ACK SET_TELEMETRY và số burst/snapshot đã nhận. Nếu không có burst, cần kiểm tra ACK/features và dữ liệu TAG trước khi kết luận về tần số.

### D. Terminal Ubuntu số 2: đo tốc độ topic

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /uwb/anchor_8/diagnostic_range
```

Một terminal Ubuntu khác:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /uwb/snapshot/anchor_8/diagnostic_range
```

Kỳ vọng cần được đo thực tế: topic DS diagnostic theo nhịp burst khoảng 100 Hz trừ mẫu lỗi; topic snapshot khoảng 5 Hz. Nếu anchor không có DS, topic diagnostic của anchor đó chưa xuất hiện. Có thể đổi A8 thành A2/A5 khi cần kiểm tra anchor khác.

Khi chưa calibration, `/uwb/anchor_8/range` không xuất hiện là đúng chính sách VALID. Dùng `diagnostic_range` để đánh giá tần số DS raw.

### E. PlotJuggler: stream topic

Mở terminal Ubuntu khác, source ROS rồi chạy PlotJuggler. Trong phần **Streaming**, chọn plugin ROS2 đã cài, Start và chọn các topic diagnostic/cycle cần xem. Kéo trường `range` của `/uwb/anchor_*/diagnostic_range` vào plot. Có thể thêm snapshot vào plot riêng để thấy hai nhịp cập nhật khác nhau.

Các terminal cùng ROS_DOMAIN_ID; mặc định cài đặt là domain 0. Dừng replay bag cũ trước phép thử live để tránh trộn các nguồn phát cùng tên.

### F. Ghi ROS bag cùng raw UART

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/uwb_ros_runs
ros2 bag record -s mcap -o ~/uwb_ros_runs/uwb_live_100hz_01 --regex '^/uwb/'
```

Đổi tên output ở lần sau nếu thư mục đã tồn tại. Thu tối thiểu 60 s khi hệ thống đã ổn định. Bấm Ctrl+C ở recorder trước, rồi relay và bridge. Giữ cả thư mục raw capture Windows và bag WSL của cùng phiên để đối chiếu.

## 5. Các topic và chỉ số cần đối chiếu

| Topic | Ý nghĩa |
|---|---|
| /uwb/anchor_N/diagnostic_range | Kết quả DS mới; raw hoặc corrected tùy CAL_OK |
| /uwb/anchor_N/range | DS có calibration và VALID cho sử dụng thực tế |
| /uwb/snapshot/anchor_N/diagnostic_range | Snapshot 0x01 khoảng 5 Hz |
| /uwb/cycle_seq | Sequence chu kỳ của TAG, không phải counter timer PC |
| /uwb/tag_time_us | Timestamp POLL do TAG cấp |
| /uwb/period_us | Period đo được trong payload burst |
| /uwb/burst_metadata | Boot/cycle/measurement sequence, mode, status, flags, range của từng anchor |
| /uwb/device_info | Firmware config, telemetry features, calibration mask |

Sau phép thử phải tính riêng: tốc độ tạo cycle, số DS mới mỗi anchor, số bộ đủ tám DS, tốc độ topic ROS và số frame mất/trùng. Hai clock có thể lệch; giữ cả timestamp TAG và host. Tăng timer publish hoặc tốc độ replay không chứng minh tăng ranging.

## Trạng thái kiểm tra bridge

Đã kiểm tra cú pháp/import/CLI trong Windows và WSL. Chưa chạy bridge với TAG trong lượt cài này vì TAG chưa kết nối; chưa xác nhận đường UART→UDP→ROS đạt tốc độ mục tiêu. Đây là node kiểm tra mới, không phải source node mà tester đã dùng.
