# UP4000 — ghi log UWB + PX4 trên drone, xem UWB qua WiFi

```
TAG DevKit ──micro-USB──┐                       WiFi (cùng mạng)
                        ├── UP4000 (Ubuntu 22.04, ROS 2 Humble) ───UDP 15555──► New_GUI (Windows)
Pixhawk 6C ──USB-C──────┘   ~/flight_logs/<ngày_giờ>/           ───UDP 14550──► QGroundControl
```

Package ROS 2 `uwb_px4_logger` có hai node, cùng ghi vào một thư mục chuyến bay:

| Node | Việc làm |
|---|---|
| `uwb_relay` | Đọc TAG (460800 baud), ghi **mọi** frame vào `uwb_frames.bin`, gửi qua WiFi cho New_GUI, chuyển lệnh từ GUI (calib, burst rate…) xuống TAG, publish `/uwb/...` |
| `px4_logger` | Ghi toàn bộ MAVLink vào `px4.tlog`, **ULog đầy đủ** qua MAVLink vào `px4_stream_N.ulg`, đồng bộ thời gian vào `timesync.csv`, chuyển tiếp MAVLink cho QGroundControl, publish `/px4/odometry` |
| `up4000_status` | Gửi về tab **5. UP4000 Onboard** của New_GUI (UDP 15557, ~4 lần/giây): CPU, nhiệt độ, RAM, ổ đĩa, WiFi, bộ đếm hai node trên, file của phiên, giá trị PX4/EKF mới nhất và STATUSTEXT. Chỉ đọc |

Dữ liệu EKF được lưu trên UP4000; UWB được ghi đầy đủ trên UP4000 và đồng thời xem trực tiếp trên GUI.

## File trong mỗi chuyến bay (`~/flight_logs/YYYYmmdd_HHMMSS/`)

| File | Nội dung |
|---|---|
| `uwb_frames.bin` | Mọi frame TAG (Burst, RX_QUALITY, DIAG…), kèm thời gian UP4000 (realtime + monotonic) |
| `uwb_status.json`, `uwb_commands.jsonl` | Bộ đếm relay; lệnh đã gửi/nhận với TAG |
| `px4_stream_1.ulg` | ULog PX4 stream qua USB (file mới sau mỗi lần cắm lại USB) |
| `px4.tlog` | Mọi gói MAVLink nhận được (mở bằng QGC/MAVExplorer) |
| `timesync.csv` | Ánh xạ đồng hồ PX4 ↔ UP4000 để ghép ULog với UWB |
| `px4_status.json`, `*_config.json` | Trạng thái và tham số đã dùng |

## 1. Cài trên UP4000

Chép cả thư mục `UP4000` lên UP4000 (git clone hoặc scp), rồi:

```bash
bash UP4000/scripts/install.sh
```

Script cài `pyserial`, `pymavlink`, udev rule (`/dev/uwb_tag`, `/dev/pixhawk`, ModemManager bỏ qua hai thiết bị), build package và tạo service `uwb-px4-logger` (chưa bật). Đăng xuất/đăng nhập lại một lần để nhóm `dialout` có hiệu lực. Kiểm tra thiết bị:

```bash
bash UP4000/scripts/list_devices.sh
```

Chạy thử (Ctrl+C dừng, file được đóng sạch):

```bash
bash UP4000/scripts/run.sh
```

Tự chạy khi bật máy:

```bash
sudo systemctl enable --now uwb-px4-logger
```

Xem output: `journalctl -u uwb-px4-logger -f`. Mỗi lần service khởi động là một thư mục chuyến bay mới.

Mỗi node chỉ chạy **một bản** (khóa `/tmp/uwb_px4_logger.<node>.lock` và cổng UDP riêng). Khi service đang chạy, `run.sh` sẽ báo `already running` / `UDP port ... is busy` rồi thoát. Sau khi cập nhật code, khởi động lại service để nạp bản mới. `update_service.sh` build lại, cập nhật file unit systemd và khởi động lại service:

```bash
bash ~/UP4000/scripts/update_service.sh
```

## 2. Tham số PX4 (QGroundControl → Parameters)

| Tham số | Giá trị | Lý do |
|---|---|---|
| `SDLOG_BACKEND` | 3 (SD + MAVLink) | Cho phép stream ULog qua USB; vẫn ghi thẻ SD |
| `SDLOG_PROFILE` | bật thêm bit **Estimator replay (EKF2)** | ULog có đủ dữ liệu vào/ra của EKF2 để phân tích, replay |
| `SDLOG_MODE` | 2 (từ boot đến tắt) | Log cả lúc chưa arm |
| `CBRK_USB_CHK` | 197848 | **Chỉ khi bay với USB cắm vào Pixhawk.** PX4 mặc định từ chối arm khi có USB |

Về `CBRK_USB_CHK`: tắt kiểm tra này là chủ ý chấp nhận rủi ro dây USB lỏng khi bay. Cách bền hơn là nối UP4000 vào cổng **TELEM2** (MAV_1_CONFIG = TELEM2, MAV_1_MODE = Onboard, SER_TEL2_BAUD = 921600) qua adapter USB-UART, rồi đặt `px4_port` thành cổng đó. Node chạy giống hệt.

Nếu sau vài giây không có file `.ulg`, node ghi cảnh báo `No ULog stream ... SDLOG_BACKEND`; khi đó kiểm tra lại bảng trên.

## 3. Máy mặt đất (Windows)

1. Máy tính và UP4000 cùng mạng WiFi.
2. Mở New_GUI, ở ô cổng chọn **WiFi UDP :15555 (UP4000 relay)**, bấm Connect.
3. UP4000 phát beacon broadcast mỗi giây; GUI trả lời PING và relay bắt đầu gửi riêng cho máy này. Mọi chức năng (bản đồ, calib, Write to TAG, burst rate, Diagnostics, ghi log CSV) dùng như khi cắm COM.
4. QGroundControl tự nhận xe qua UDP 14550.
5. Tab **5. UP4000 Onboard** hiện sức khỏe UP4000, trạng thái logger và PX4/EKF trực tiếp (chế độ bay, arm, pin, vị trí, vận tốc, góc, test ratio, cờ EKF, rung, thông báo PX4), kèm cây "All values received" chứa mọi giá trị. Tab nhận broadcast 1 Hz ngay cả khi chưa kết nối UWB; khi relay đã biết máy này thì nhận 4 lần/giây. Không gửi lệnh nào lên drone.

Lần đầu Windows hỏi tường lửa: cho phép Python (New_GUI) và QGC nhận kết nối trong mạng Private. Nếu router chặn broadcast, đặt IP máy mặt đất cố định:

```bash
echo 'LAUNCH_ARGS="ground_host:=192.168.1.20"' | sudo tee /etc/default/uwb-px4-logger
sudo systemctl restart uwb-px4-logger
```

Có thể sửa thêm `UP4000/ros2_ws/src/uwb_px4_logger/config/logger.yaml` (cổng, tần số MAVLink, tắt ULog…) rồi chạy lại `install.sh` hoặc `colcon build`.

### WiFi power save của USB WiFi (rtl8821cu)

Ubuntu bật power save cho WiFi (`/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf`, `wifi.powersave = 3`). Đo ngày 07/11 (UP4000 → PC, cùng AP 5 GHz, -48 dBm): lúc tệ chỉ 72% gói UDP 1,1 kB tới nơi nếu gửi dồn 8 gói, 85% nếu cách nhau 5 ms; gói đầu tiên sau 250 ms nghỉ chỉ tới 38–48%. Lúc khác cùng phép đo lại đạt 99,9%. Luồng UWB lên GUI cùng lúc thiếu 16% chu kỳ. File trên UP4000 không bị ảnh hưởng. Tab UP4000 hiện `power save ON` ở thẻ WiFi khi đang bật. Tắt (cần sudo):

```bash
sudo iw dev wlx30169df9a262 set power_save off
printf '[connection]\nwifi.powersave = 2\n' | sudo tee /etc/NetworkManager/conf.d/zz-wifi-powersave-off.conf
```

Lệnh đầu có hiệu lực ngay; file thứ hai (đọc sau file mặc định nên ghi đè nó) giữ cấu hình sau khi khởi động lại. `up4000_status` cũng giãn các phần của một chu kỳ trạng thái 2 ms (`part_gap_ms`).

### UWB có byte nhưng không có frame hợp lệ

Kiểm tra `uwb_status.json`: nếu `parser.bytes_received` tăng nhưng `valid_frames` gần như bằng 0,
`crc_errors`/`length_errors` tăng nhanh thì lỗi đã xuất hiện trước WiFi. Đừng mở thêm chương trình
đọc cùng cổng để thử. Ngày 07/11, đúng baud 460800 và đọc độc quyền vẫn lỗi; reset riêng USB
SEGGER J-Link đã khôi phục 2.244 frame/8 giây với 0 lỗi CRC/độ dài. Không cần flash TAG hay reset PX4.

Khi gặp lại tình trạng này, có thể khôi phục adapter:

```bash
sudo python3 ~/UP4000/scripts/reset_tag_usb.py
```

Script xác minh đúng thiết bị SEGGER J-Link, tạm dừng logger, reset USB và khởi động lại logger
trong `finally` nếu service đang chạy trước đó. Log sẽ sang phiên mới. Có nhiều J-Link thì thêm
`--serial <USB-serial>`. Script không tự chạy theo watchdog. Các lỗi CRC lẻ tẻ khi vẫn nhận đều
frame cần đo riêng; kết quả reset trên không có nghĩa đường UART sẽ luôn sạch lỗi.

Các gói trạng thái được giới hạn **1.200 byte tính cả JSON envelope**. Giá trị nguyên khối quá
dài được chia fragment; GUI chỉ dùng khi nhận đủ fragment, và báo STALE theo từng message sau 2 giây.

## 4. Sau chuyến bay

Chép log về máy tính (ví dụ `scp -r user@<ip-up4000>:~/flight_logs/20261107_150000 .`).

- **UWB đầy đủ trong New_GUI** (log GUI ghi qua WiFi có thể hụt khi sóng yếu, file trên UP4000 thì không):
  ```powershell
  python Software\New_GUI\tools\uwb_frames_to_session.py <thư mục chuyến bay> --anchors Software\New_GUI\last_rig_config.json
  ```
  Tạo một phiên trong `Software/New_GUI/logs/` để mở bằng trang Replay và chạy `tools/diag_summary.py`.
- **EKF**: mở `px4_stream_1.ulg` bằng PlotJuggler, Flight Review hoặc `pyulog`. File trên thẻ SD của Pixhawk là bản dự phòng.
- **Ghép thời gian UWB ↔ EKF**: ULog dùng thời gian PX4 kể từ boot. `timesync.csv` có cột `px4_minus_host_monotonic_ns`; thời gian PX4 = thời gian monotonic của UP4000 (cột thứ hai trong `uwb_frames.bin`) + giá trị đó. Chỉ dùng các mẫu có `rtt_ns` nhỏ (ví dụ < 5 ms): sai số offset ≤ RTT/2, và RTT có lúc lên hàng trăm ms khi máy bận.
- **ULog stream có thể có dropout** (message `O`) khi USB/CPU không theo kịp lúc PX4 dồn dữ liệu; `px4_status.json` ghi số chunk mất. File trên thẻ SD của Pixhawk (`SDLOG_BACKEND` = 3) là bản đầy đủ.

Kết quả chạy thật ngày 07/11/2026 (60 s mỗi lần, UP4000 đang tải nặng vì biên dịch C++ và `drone_gcs`):

| Hạng mục | Kết quả |
|---|---|
| UWB về máy mặt đất | ~320 Burst/s, 4 anchor ~290 Hz mỗi anchor, thiếu 12–20 cycle/phút (mất byte trên serial khi máy quá tải) |
| ULog | 89 topic gồm toàn bộ `estimator_*`; mất 0,06% chunk khi UWB rảnh, ~2,6% khi máy tải 4,4 trên 2 nhân |
| tlog | ODOMETRY/ATTITUDE/LOCAL_POSITION ~49 Hz, ESTIMATOR_STATUS ~10 Hz |

**Cấu hình cổng đã chọn: USB của Pixhawk dành cho MAVLink (logger).** Một cổng serial chỉ dùng được cho một chương trình. Không chạy `MicroXRCEAgent`, MAVROS hay QGroundControl (qua USB) trên `/dev/pixhawk` cùng lúc với logger: dữ liệu bị chia đôi, log ghi `multiple access on port`. Khi mở cổng, logger in `WARNING: ... port ... is also named by pid ...` nếu thấy tiến trình khác nhắc tới cổng đó. Cần uXRCE-DDS thì chạy nó trên TELEM2 (`UXRCE_DDS_CFG` = TELEM2, adapter USB-UART, ví dụ `MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600`).

Lần chạy 10 (USB chỉ cho MAVLink, máy không biên dịch): ULog mất 4/10.512 chunk (0,04%); tlog ODOMETRY/ATTITUDE/LOCAL_POSITION/HIGHRES_IMU 49,4 Hz; TIMESYNC RTT trung vị 0,82 ms; UWB về máy mặt đất 318 Burst/s; CPU `uwb_relay` ~60%, `px4_logger` ~24% một nhân.

Topic UWB trên ROS được giới hạn `ros_publish_hz` (mặc định 50 Hz) vì publish đủ 320 Burst/s bằng rclpy làm nghẽn CPU của UP4000; file `uwb_frames.bin` và luồng WiFi luôn giữ đủ tốc độ.

## 5. Topic ROS 2

| Topic | Kiểu |
|---|---|
| `/uwb/anchor_N/range` | `sensor_msgs/Range` (DS đã calib và hợp lệ) |
| `/uwb/anchor_N/diagnostic_range` | `sensor_msgs/Range` (mọi DS) |
| `/uwb/burst_metadata`, `/uwb/device_info` | `std_msgs/String` JSON |
| `/uwb/cycle_seq`, `/uwb/tag_time_us` | `UInt32`, `UInt64` |
| `/px4/odometry` | `nav_msgs/Odometry` (EKF2 của PX4, khung NED/FRD như PX4 gửi) |
| `/uwb_relay/status`, `/px4/status` | `std_msgs/String` JSON 1 Hz |

## 6. Kiểm thử và giới hạn

Phần lõi không phụ thuộc ROS và có test chạy được trên Windows:

```bash
cd UP4000/ros2_ws/src/uwb_px4_logger
python -m pytest test
```

Các test tự động dùng TAG giả, PX4 giả qua MAVLink (pymavlink) và ULog tổng hợp có mất gói. Test xuyên suốt UP4000 relay → New_GUI nằm ở `Software/New_GUI/tests/test_udp_relay_input.py`. Đã chạy thêm trên UP4000, Pixhawk 6C và TAG thật với ba node ROS 2: xem [báo cáo WiFi 07/11/2026](test_results/20261107_wifi/REPORT.md). Khi chuyển sang phần cứng/mạng khác, chạy `run.sh`, mở thư mục log và kiểm tra lại `uwb_status.json` / `px4_status.json`.

Bộ parser TAG được chép từ `Software/UWB_UART_GUI` vào `uwb_px4_logger/vendor/`. Sau khi sửa parser ở đó, chạy `python UP4000/scripts/sync_vendor.py`.
