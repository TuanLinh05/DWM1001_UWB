# Gói tester UART / ImGui đầy đủ — 25/10/2026

Firmware TAG đi kèm config **b5f8839d**, tương thích anchor300/PHY64 đã nạp.
Giữ lịch ranging, thêm chất lượng raw10 Hz và môi trường TAG khoảng1 Hz.
Không tự hiệu chuẩn hoặc biến CAL_MISSING thành VALID.

## Dùng trên Windows

Nạp riêng TAG bằng script trong repo gốc:

```powershell
& 'D:\Drone Project\Github Clone UWB Test\Firmware\scripts\flash_highrate.ps1' -Node TAG -Rate 300
```

Mở `DWM1001_UWB_Ground_Control_quality.exe`, xem tab **RX chất lượng**.
Ghi log sẽ có rx_quality.csv; raw chất lượng có cycle/t_us riêng. Các
ô chưa có mẫu là unknown; không lấy số0 hoặc dữ liệu cũ làm mẫu mới.

## Dùng trên Ubuntu / GUI tester

Giải nén `UWB_tester_quality_20261025.zip`, cd vào thư mục gói. Cần ROS2
Humble cùng dependency của GUI gốc, Python pyserial và numpy. CMake sẽ tải
đúng các phiên bản ImGui/ImPlot/GLFW mà repository gốc ghim.

```bash
source /opt/ros/humble/setup.bash
cmake -S IMGUI_UWB_CALIB/uwb_gui -B build_gui -DCMAKE_BUILD_TYPE=Release
cmake --build build_gui -j4
./build_gui/uwb_gui
```

Nhập cổng TAG, ví dụ `/dev/ttyACM0`, rồi chọn **Start UART telemetry**.
Nút này chạy adapter kèm gói, đổi line coding115200→460800 và yêu cầu
telemetry0x07. Nó không đổi timing, calibration, NVS hoặc flash firmware.
Chỉ một chương trình được giữ cổng serial trong phiên.

Các cột RX, RX−FP, CI và nhiệt độ/điện áp nằm ở Link. Mở cây **Raw RESP
quality** để xem F1/F2/F3/CIR/RXPACC/noise/FP index/CI raw/power chính xác hơn.
Raw tối đa10 Hz; range và compact FP/RX/CI vẫn theo mọi burst.

Nút cũ **Start vcom_node + uwb_pos** thuộc full drone stack của tester,
cần cấu hình DRONE_DIR và cập nhật node UART trong launch đó. Adapter
kèm gói xuất range/metadata, không tự giải vị trí. Topic `uwb/pos` vẫn cần
node định vị cùng geometry/calibration của host.

Có thể chạy adapter thủ công thay vì nút UART:

```bash
python3 uart_bridge/ros2/vcom_node.py --port /dev/ttyACM0 --baud 460800
```

GUI đang dùng `/uwb/burst_metadata` phải có cùng ROS_DOMAIN_ID với adapter.
Không chạy đồng thời adapter mới và node UART cũ trên cùng cổng/topic.

## Nếu dùng COM trên Windows → UDP → WSL

Ubuntu:

```bash
source /opt/ros/humble/setup.bash
python3 uart_bridge/ros2/uwb_ros_bridge.py
```

Windows, từ thư mục gói:

```powershell
$uwbWslIp = ((wsl -d Ubuntu-22.04 -- hostname -I).Trim() -split '\s+')[0]
python .\uart_bridge\ros2\uart_udp_relay.py --port COM14 --host $uwbWslIp
```

Chạy GUI riêng để nghe ROS. Không chọn Start UART khi đang dùng relay.

## Áp patch vào checkout tester đang có

Patch đối chiếu commit `1b259c08fe42791e802c3b77c7c62ec18025209f`:

```bash
cd /path/to/IMGUI_UWB_CALIB
git apply --check /path/to/imgui_quality_20261025.patch
git apply /path/to/imgui_quality_20261025.patch
```

Nếu adapter không nằm theo layout của gói, cấu hình CMake thêm
`-DUART_BRIDGE=/absolute/path/to/vcom_node.py`. Nếu giữ full bringup cũ,
hãy thay node đọc UART bằng adapter/decoder mới để JSON có đủ trường.
Repo tester gốc không chứa producer `vcom_node.py`; sửa bảng GUI không
thể tự tạo các trường JSON chưa được producer gửi.

## Kiểm tra không cần phần cứng

Gói có C encoder golden và JSON mẫu do ROS bridge thật tạo. Chúng là
dữ liệu test tổng hợp, không phải log đo board mới. Build GUI với
`-DGUI_HEADLESS=ON` rồi dùng:

```bash
./build_gui/uwb_gui --self-test validation/tester_metadata_example_20261025.json
python3 IMGUI_UWB_CALIB/uwb_calib.py --test
ROS_DOMAIN_ID=141 python3 uart_bridge/ros2/test_quality_bridge.py validation/quality_golden_20261025.bin
```

## Chi phí

Thêm tối đa2.700 B/s raw quality và12 B/s DIAG. Tải UART dựa log cũ khoảng
65,3%→71,2% tại460800 baud. Giao tiếp TX không blocking, nên không đồng nghĩa
giảm6% tốc độ ranging. Chưa có số đo Hz sau bổ sung; cần log board60–120 s.
Đặc tả wire, validity và unit đầy đủ ở docs/TELEMETRY_QUALITY_20261025.md.
