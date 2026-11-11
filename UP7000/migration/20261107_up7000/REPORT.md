> Cập nhật lưu trữ 07/11/2026: các đường dẫn onboard trong báo cáo này hiện là symlink tới HDD `/mnt/droneup/onboard_20261107`; xem `~/UP7000_HDD_STORAGE.md`. Logger và APT cần HDD được gắn.

# UP7000 thay UP4000 — bàn giao phần mềm 07/11/2026

UP7000 đã được cài và cấu hình để đảm nhiệm bộ logger UWB/PX4 cùng workspace ROS của UP4000. Service thật **active + enabled**, tự chờ và nhận lại thiết bị khi cắm USB. Chưa xác nhận hoạt động trên TAG/Pixhawk/USB WiFi thật vì UP7000 hiện không cắm thiết bị nào trong số đó. Đây chưa phải kết quả thử bay hay thử phần cứng.

## Máy và đường dẫn

- SSH: `ssh thoa@100.123.216.14`; hostname giữ nguyên `thoa`.
- Ubuntu 22.04.5, kernel `6.8.0-138-generic`, RAM khoảng 4 GB. Sau cài/build còn khoảng 14 GB trên ổ trong.
- Ethernet hiện tại: `192.168.1.207`; PC kiểm thử: `192.168.1.205`.

| Thành phần | Vị trí trên UP7000 |
|---|---|
| Logger UWB/PX4, source và scripts | `/home/thoa/UP4000` |
| Tên truy cập thuận tiện | `/home/thoa/onboard_logger` → thư mục trên |
| 7 package ROS PX4/UWB | `/home/thoa/ros2_px4_v2` |
| Python dependency độc lập ổ trong | `/home/thoa/drone_tools/python` |
| Agent đúng bản UP4000 v3.0.2 | `/home/thoa/drone_tools/microxrce-3.0.2` |
| Lệnh Agent, có wrapper thư viện riêng | `/home/thoa/.local/bin/MicroXRCEAgent` |
| ONNX Runtime CPU 1.20.1 | `/home/thoa/lib/onnxruntime-linux-x64-1.20.1` |
| Dependency GUI đã cache từ UP4000 | `/home/thoa/drone_tools/gui_deps` |
| Log khi chạy thật | `/home/thoa/flight_logs/<session>` |
| Gói chuyển, manifest và kết quả mô phỏng | `/home/thoa/up4000_migration_20261107` |

Đã chuyển **1.215 file nguồn** từ UP4000 và đối chiếu SHA-256 từng file, không thiếu file. Bao gồm source logger, script phục hồi J-Link, cấu hình, rig/bias/URDF, model ONNX, workspace PX4, source Agent và source driver WiFi cũ. Build/install/cache cũ không dùng lại; workspace được build mới cho UP7000.

Sau đối chiếu có ba file được sửa có chủ đích: `scripts/install.sh`, `scripts/uwb-px4-logger.service` và test `test_status.py`; thêm `test_usb_recovery.py`. Bản runtime logger/protocol từ UP4000 được giữ nguyên.

## Cấu hình đã áp dụng

- ROS 2 Humble ROS Base, công cụ colcon/compiler và các dependency sensor/nav/launch/URDF/MCAP/RealSense. Repository được thiết lập theo [hướng dẫn chính thức ROS](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/_Apt-Repositories.rst).
- Bộ Python riêng có `pymavlink==2.4.50`, `fastcrc==0.5.0`, `lxml==6.1.3`; pyserial hệ thống 3.5. `~/.local/lib` và pip cache của máy đang trỏ vào ổ `/mnt/droneup` chưa mount, nên logger dùng prefix riêng trên ổ trong. Không sửa các symlink cũ.
- Template service và installer đã được sửa để các lần cập nhật tiếp theo vẫn dùng prefix Python đó. Unit được `systemd-analyze verify` và khởi động lại thành công. Chưa reboot máy để thử boot thật; trạng thái systemd đã là enabled.
- Service `uwb-px4-logger` chỉ tự chạy ba node `uwb_relay`, `px4_logger`, `up4000_status`. Không bật tự động planner, controller, Agent hay lệnh bay.
- Alias USB `/dev/uwb_tag` cho SEGGER J-Link và `/dev/pixhawk` cho các vendor PX4 tương ứng đã cài; user `thoa` có nhóm dialout. Chưa kiểm tra được alias thực tế vì chưa cắm USB.
- TAG 460800 baud; PX4 logger giữ cấu hình MAVLink/ULog/rates/timesync như nguồn. Các cổng giữ nguyên: UWB về PC 15555, nhận đăng ký/lệnh tại UP7000 15556, trạng thái về PC 15557, MAVLink về QGC 14550, cổng MAVLink tại UP7000 14555.
- Profile `PIF_CLUB_5G` được chuyển qua SSH, bỏ ràng buộc MAC/interface cũ, bật tự kết nối và tắt power save. Mật khẩu không nằm trong báo cáo hay log chuyển; profile hệ thống có quyền 0600.
- Kernel có sẵn driver `rtw88_8821cu` hỗ trợ USB ID 0bda:c811 của adapter trên UP4000; đã nạp module thành công. Không cài bản DKMS cũ của kernel 5.15 lên kernel 6.8. Chưa thử kết nối WiFi thật khi chưa có adapter.
- NetworkManager chỉ quản lý WiFi; Ethernet, Tailscale và interface Docker để unmanaged, tiếp tục do cơ chế cũ quản lý. Ethernet/Tailscale đã được kiểm tra còn hoạt động.
- Phát hiện cấu hình Fast DDS mặc định thấy endpoint nhưng dữ liệu giữa các node không cập nhật ổn định. Đã dùng transport UDPv4 và thêm peer loopback tại `/home/thoa/drone_tools/fastdds-udp.xml`, áp dụng qua service drop-in `20-dds.conf` và helper môi trường. Không đặt whitelist giới hạn interface, không đổi domain ROS của service. Các node đã truyền địa chỉ ground và status được với cấu hình cuối. Cách thêm initial peer dựa trên [tài liệu Fast DDS](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/discovery/simple.html).
- Dependency ImGui/ImPlot/GLFW/MAVLink được lấy đúng bản đã build trên UP4000, cache offline và chỉ định trong `ros2_px4_v2/colcon.meta`. Không phụ thuộc DNS/GitHub khi build lại GUI.

## Kiểm chứng

| Kiểm tra | Kết quả |
|---|---|
| Logger trên Linux UP7000 | **25 test đạt, 1 test bỏ qua**: kiểm tra vendor-sync không áp dụng cho source standalone không có cây Software bên cạnh; file protocol đã được đối chiếu SHA-256 |
| Build logger | 1 package thành công |
| Build workspace cũ | Cả 7 package thành công: data_collector_gui, px4_learned_planner, px4_msgs, px4_test_bridge, px4ctrl_dds, uav_core, uwb_eskf |
| Shared library | Agent và GUI đủ thư viện khi source đúng môi trường ROS; Agent chạy được lệnh trợ giúp |
| ESKF/controller/planner | Khởi động rồi dừng sạch trong domain 77, localhost-only; planner nạp model K=27, ảnh 80×48, state=8, dry-run |
| Selftest ESKF tham chiếu | `tools/uwb_eskf.py --selftest`: OK |
| ROS tích hợp cấu hình cuối | TAG giả qua PTY, PX4 giả qua UDP loopback, 3 node ROS thật, domain 77, cổng test riêng; launch thoát sạch |
| UWB giả lập cuối | UP7000 parse 17,302 Burst; PC nhận 17,302, 2 anchor giả ID 1 và 2; 0 CRC/length/decode error |
| PX4 giả lập cuối | 1,335 MAVLink message; 108 mẫu timesync; file tlog/ULog/raw UWB được tạo |
| ULog giả lập cuối | 191 chunk, 47,393 byte; 0 dropped chunk/dropout |
| UDP trạng thái mô phỏng | PC nhận 696 datagram; lớn nhất 1181 byte, dưới 1.200 |
| Trạng thái service thật | PC nhận 154 datagram, 0 lỗi parser; hostname thoa; PX4 connected=false đúng thực tế |
| UDP hai chiều service thật | Relay nhận CMD_PING từ PC và học `192.168.1.205:15555`; địa chỉ truyền sang logger PX4; unicast về GUI hoạt động |

Lượt mô phỏng tạo dữ liệu từ trước lúc node khởi động; số frame viết vào PTY lúc startup không dùng làm mẫu số tính mất gói. Số Burst nhận tại PC khớp số đã parse/ghi trên UP7000 trong phiên này.

Source C++ chuyển sang không có bộ test C++: `colcon test` báo 0 test; vì vậy không coi đó là kiểm chứng thuật toán. Đã dùng kiểm tra khởi động/nạp model cùng selftest Python nêu trên. GUI `drone_gcs` được build và kiểm tra shared library; chưa mở cửa sổ GUI thật trên UP7000 do máy hiện chạy headless. Build có cảnh báo enum của ImPlot, không có lỗi build.

## Dùng hằng ngày

```bash
# Xem service và log
systemctl status uwb-px4-logger
journalctl -u uwb-px4-logger -f

# Môi trường dùng CLI và các package bổ sung
source ~/drone_env.sh
ros2 topic list

# Sau khi chép source logger mới
bash ~/UP4000/scripts/update_service.sh

# Chỉ khi adapter J-Link lại có byte nhưng gần như không có frame hợp lệ
sudo python3 ~/UP4000/scripts/reset_tag_usb.py
```

Tab **UP4000 Onboard** trong New_GUI vẫn dùng được; tên tab/namespace giữ để tương thích. Máy mới hiện hostname **thoa**. Trong New_GUI vẫn chọn WiFi UDP :15555 để xem UWB khi TAG đã cắm. Trạng thái máy hoạt động cả khi chưa có TAG/Pixhawk.

Log nằm trên ổ trong; tên session dùng giờ UTC theo timezone hiện có của hệ điều hành, giống cách chạy trước trên UP4000.

## Phần chờ thiết bị và chuyển nhiệm vụ

- Hiện `lsusb` chỉ có root hub; chưa có TAG, Pixhawk, camera hay USB WiFi để kiểm thử trên UP7000. Khi có thiết bị, cần xác nhận alias USB, frame UWB thật với 2 anchor, MAVLink/ULog/timesync thật, handshake DDS nếu dùng Agent, và WiFi thật. Không suy ra lỗi CRC/dropout trước đây đã hết chỉ từ mô phỏng.
- USB Pixhawk dành cho MAVLink logger. Không mở đồng thời Agent, Companion, MAVROS hoặc một reader khác trên cùng cổng. Nếu dùng DDS Agent, dùng đường serial riêng như TELEM2 theo cấu hình nguồn; không tự đổi tham số PX4 trong lần chuyển này.
- Giữ nguyên UP4000 đang chạy để làm nguồn/backup, chưa tắt service cũ khi UP7000 chưa có phần cứng. Khi chuyển nhiệm vụ thật và hai máy cùng mạng, dừng service cũ (`sudo systemctl stop uwb-px4-logger` trên UP4000) để tránh hai nguồn cùng gửi vào tab/UDP của GUI.
- Giữ nguyên các dự án Companion, LiDAR/Docker, dữ liệu và cấu hình cũ của UP7000. Ổ `/mnt/droneup` đang thiếu là tình trạng có sẵn; bộ logger mới không phụ thuộc ổ đó.
- Lịch sử `flight_logs`/`bags` của UP4000 vẫn nằm trên máy nguồn; không chép dữ liệu lịch sử không cần cho runtime trong lần chuyển này.

## Bằng chứng và backup

Trong thư mục báo cáo trên PC có `final_inventory.json`, `source_manifest.json`, `archive_sha256.json`, `status_check.json`, `ros_integration.json`, `cpp_smoke.json`, `fastdds-udp.xml`. Gói source, Agent và GUI dependency cũng lưu tại `/home/thoa/up4000_migration_20261107` trên UP7000; dữ liệu mô phỏng được đánh dấu `integration/migration_sim_*`, không để lẫn vào log thật. Báo cáo này được chép lên `/home/thoa/UP7000_MIGRATION.md`.
