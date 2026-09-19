# Kế hoạch triển khai hệ thống định vị UWB cho drone với UP 7000, Pixhawk 6C và MTF-01

**Phiên bản:** 1.1  
**Ngày lập:** 2026-09-12  
**Cập nhật cấu hình thực tế:** UP 7000 N100/4 GB RAM, lưu trữ ngoài qua USB box, Pixhawk 6C nối trực tiếp với UP 7000 bằng USB-C thay vì TELEM2  
**Trạng thái:** Kế hoạch thực thi; mọi ngưỡng hiệu năng phải được xác nhận bằng log và ground truth trước khi cho phép bay tự động  
**Phạm vi:** một drone, một TAG UWB, giai đoạn đầu 4 anchor/2D, giai đoạn sau 8 anchor/3D, UP 7000 là companion computer, Pixhawk 6C chạy PX4

---

## 1. Kết luận kiến trúc

Kiến trúc nên triển khai là kiến trúc phân tầng, không chuyển quyền điều khiển ổn định bay sang Linux:

1. **TAG/DWM1001C** chỉ làm phần thời gian thực sát phần cứng: DS-TWR, đọc chẩn đoán DW1000, đóng dấu thời gian, kiểm tra frame/CRC, watchdog và truyền phép đo thô sang UP 7000.
2. **UP 7000** làm phần tính toán nặng: hiệu chỉnh range, nhận biết LOS/NLOS, multilateration có trọng số, bộ ước lượng động học, kiểm tra integrity, factor graph/smoother, auto-calibration, ghi log, web backend, GUI và tạo lệnh quỹ đạo.
3. **Pixhawk 6C/PX4** vẫn là lớp an toàn và điều khiển bay cuối cùng: đọc IMU, MTF-01/rangefinder, EKF2, attitude/rate/position controller, arming check và failsafe. Pixhawk 6C có STM32H743 480 MHz, IMU dự phòng và được đội PX4 hỗ trợ kiểm thử, vì vậy phù hợp giữ nhiệm vụ điều khiển cứng thời gian thực.[^1]
4. **Wi-Fi và GUI không nằm trong vòng ổn định bay.** Mất trình duyệt hoặc mất Wi-Fi không được làm mất luồng setpoint nội bộ giữa UP 7000 và PX4. Nếu UP 7000 chết hoặc ngừng gửi heartbeat, PX4 phải tự chuyển sang hành động failsafe đã cấu hình.
5. **Không dùng trực tiếp range đã làm mượt mạnh trên TAG để điều khiển.** UP 7000 phải nhận `raw_mm`, timestamp và quality; range lọc trên TAG chỉ được dùng để quan sát/so sánh. Cách này tránh lặp lại lỗi step, độ trễ và giới hạn giả đã gặp trong các thử nghiệm trước.

UP 7000 N100/4 GB hiện có **đủ sức cho toàn bộ chuỗi UWB, PX4 bridge, telemetry và web backend** nếu chạy headless, dùng queue/cửa sổ tối ưu hữu hạn và không chạy desktop/browser/VIO nặng trên board. Không cần đổi UP 7000 chỉ vì RAM 4 GB. Nâng cấp phần cứng chỉ trở nên đáng cân nhắc khi bổ sung camera VIO, AI hoặc mapping dày. Trang phần cứng công bố N50/N97/N100, tối đa bốn nhân, LPDDR5 4/8 GB, eMMC 32/64 GB, ba cổng USB 3.2, hai USB 2.0 nội bộ, một HSUART, Ethernet 1 Gb/s và Wi-Fi/Bluetooth tùy chọn.[^2] Linux trên UP 7000 không phải hard-real-time; điều này chấp nhận được vì hard-real-time vẫn ở Pixhawk.

### Quyết định về hệ điều hành

- **Có thể dùng ngay Ubuntu 22.04 + ROS 2 Humble cho prototype**, vì đây là cấu hình hệ điều hành/kernel được UP công bố hỗ trợ.[^2]
- **Nên chuẩn bị chuyển sang Ubuntu 24.04 + ROS 2 Jazzy trước khi đóng băng hệ thống bay.** Humble và giai đoạn bảo trì chuẩn của Ubuntu 22.04 đều kết thúc vào khoảng tháng 5/2027, trong khi Jazzy và Ubuntu 24.04 kéo dài tới 2029.[^3][^4][^5]
- Không nâng cấp trực tiếp eMMC đang chạy. Tạo image 24.04/Jazzy riêng, kiểm tra đầy đủ USB, HSUART, Wi-Fi, watchdog, nhiệt độ và tải CPU; chỉ chuyển khi đạt cổng nghiệm thu. Không chọn Ubuntu 26.04/ROS 2 Lyrical ở giai đoạn đầu vì còn mới và chưa nằm trong danh sách OS chính thức của UP 7000.

### Điều cần hiểu về mục tiêu “3D hoàn hảo”

Không có hệ UWB vô tuyến nào đảm bảo tuyệt đối trong mọi NLOS/multipath. DWM1001 công bố ranging trong khoảng 10 cm trong điều kiện phù hợp, còn vật cản, hướng antenna và hình học anchor có thể làm sai số tăng mạnh.[^6][^7] Mục tiêu đúng phải là: sai số có thống kê, covariance trung thực, phát hiện khi nghiệm không đáng tin, suy giảm có kiểm soát và PX4 chuyển chế độ an toàn. Kế hoạch này dùng các cổng nghiệm thu định lượng thay cho từ “hoàn hảo”.

---

## 2. Giả định, phạm vi và các quyết định phải khóa

### 2.1 Giả định làm việc

- Một drone và một TAG trong phiên bản đầu.
- Tiếp tục dùng DWM1001C/DW1000 làm UWB front-end cho đến khi benchmark chỉ ra giới hạn vật lý không đạt yêu cầu.
- TAG giao tiếp với UP 7000 qua CP2102/CP2102N USB-UART.
- UP 7000 thực tế là bản N100/4 GB; USB box dùng cho lưu trữ log/archive.
- UP 7000 nối trực tiếp cổng USB-C của Pixhawk 6C; link chính là MAVLink 2 qua USB CDC ACM, không dùng TELEM2 trong cấu hình ban đầu.
- UP 7000 và Pixhawk 6C cùng nằm trên drone.
- MTF-01 nối trực tiếp với Pixhawk, không đi vòng qua web server.
- Bản đồ làm việc của anchor dùng **ENU** cho người vận hành/GUI; trước khi gửi PX4 phải đổi đúng một lần sang **NED**.
- Một TAG chỉ cung cấp vị trí, không quan sát trực tiếp yaw/attitude. Yaw và attitude tiếp tục do PX4 ước lượng.
- BLE là kênh commissioning/bảo trì; không là kênh điều khiển bay bắt buộc.

### 2.2 Các quyết định phải ghi thành ADR

Tạo thư mục `docs/adr/` và ghi ít nhất các Architecture Decision Record sau:

| ADR | Quyết định cần khóa | Khuyến nghị ban đầu |
|---|---|---|
| ADR-001 | Phân chia TAG/UP/PX4 | TAG đo, UP định vị, PX4 điều khiển |
| ADR-002 | OS/ROS | Prototype 22.04/Humble; qualify 24.04/Jazzy |
| ADR-003 | Link PX4 | MAVLink 2 qua USB CDC; `mavlink-routerd` fan-out tới MAVSDK/QGC |
| ADR-004 | Hệ tọa độ | Anchor map ENU, bridge duy nhất ENU→NED |
| ADR-005 | Estimator | robust WNLS + IEKF thời gian thực; smoother riêng |
| ADR-006 | Nguồn độ cao 2D | MTF-01/rangefinder qua PX4; UWB chỉ XY |
| ADR-007 | Lịch 8 anchor | bất đồng bộ, hai nhóm tứ diện xen kẽ |
| ADR-008 | Auto-calibration | offline/commissioning, validation rồi mới commit |
| ADR-009 | BLE | provisioning, mặc định không tham gia khi armed |
| ADR-010 | Safety | GUI không được gửi motor command trực tiếp |

---

## 3. Baseline thực tế của project hiện tại

Trước khi mở nhánh phát triển mới phải lưu lại baseline vì nó là mốc so sánh bắt buộc.

### 3.1 Những gì đã có trong firmware

- `TAG_NUM_ANCHORS = 4`, `TAG_CYCLE_MS = 20`, tức mục tiêu 50 chu kỳ/s.
- DS-TWR đang bật và dùng chuỗi `POLL → RESP → FINAL → REPORT` cho từng anchor.
- Anchor phản hồi delayed-TX ở 1200 UUS; TAG có response timeout 2600 µs, report timeout 3000 µs và guard 150 µs giữa hai anchor.
- Telemetry nhị phân hiện tại có header 14 byte, một byte số record, mỗi anchor 16 byte và CRC 2 byte. Gói bốn anchor là 81 byte; nếu mở rộng cơ học lên tám anchor sẽ là 145 byte.
- Ở 50 Hz, tám anchor theo schema hiện tại tạo 7.250 byte/s. UART 115200 8N1 có trần lý thuyết 11.520 byte/s, tức đã dùng khoảng 63% trước INFO/STATS/command/jitter. Vì vậy production phải chuyển lên 921600 baud.
- CP2102N hỗ trợ 300 baud tới 3 Mbaud và có buffer RX/TX 512 byte, nên 921600 nằm trong khả năng phần cứng; nếu adapter thực tế là CP2102 đời cũ thì phải xác minh part number và thử lỗi baud riêng.[^8]
- Firmware đang có CRC, status, raw/filtered range, age và FPP; đây là nền tốt nhưng chưa đủ cho estimator mạnh vì còn thiếu nhiều DW1000 diagnostics và timestamp có độ phân giải cao.
- Cấu hình build hiện tại chưa bật BLE.
- `UWB_DS_CALIBRATED_MASK = 0`, nghĩa là chưa anchor nào được xác nhận hiệu chuẩn DS-TWR cho production.

### 3.2 Baseline log mới nhất ngày 2026-09-12

Log nội bộ được kiểm tra trong phiên đo ngày 2026-09-12. File log thô là dữ liệu
cục bộ và không được commit vào repository.

Kết quả quan sát:

- 19,578 giây, 981 range frame, xấp xỉ 50 Hz, `queue_drops = 0`.
- Toàn bộ record có `valid = 0`.
- A1 chủ yếu trả kết quả diagnostic `CAL_MISSING`, có thêm timeout/compute error/fallback.
- A2, A3 và A4 timeout ở cả 981 frame.

Điều này không phủ nhận việc đường host-filter nhìn mượt, nhưng **chưa phải baseline cho bay 4-anchor**. Cổng đầu tiên của kế hoạch là làm cho cả bốn anchor đồng thời valid, đã hiệu chuẩn và có tỷ lệ thành công đạt yêu cầu. Không được phát triển flight control dựa trên một đường cong đẹp nhưng status không hợp lệ.

### 3.3 Việc cần đóng băng ngay

1. Tag firmware hash, anchor firmware hash, GUI hash.
2. Toàn bộ header/calibration/timing hiện tại.
3. Log tĩnh ở nhiều khoảng cách và log di chuyển.
4. Ảnh/sơ đồ vị trí anchor, hướng antenna, nguồn cấp và vật cản.
5. Script replay để cùng một log luôn tạo đúng cùng một output.

---

## 4. Kiến trúc phần cứng và luồng dữ liệu mục tiêu

```text
 DWM1001C TAG
 [DW1000 + nRF52832]
       │ UART 3.3 V, 921600, binary + CRC, source timestamp
       ▼
 CP2102N ── USB ──> UP 7000
                     ├─ uwb_serial_driver
                     ├─ range correction / NLOS scoring
                     ├─ robust WNLS + real-time IEKF
                     ├─ integrity monitor
                     ├─ calibration / fixed-lag smoother
                     ├─ logger / replay
                     ├─ web backend + GUI
                     └─ mission manager / trajectory generator
                              │
                              │ MAVLink 2 qua USB CDC ACM
                              │ ODOMETRY + telemetry + command/offboard
                              ▼
                       USB-C ─ Pixhawk 6C / PX4
                     [EKF2 + flight controllers]
                        ▲               │
                        │               └─ motors/ESC
              MTF-01 ───┘
        flow + range, UART 115200

 Pixhawk USB-C ⇄ UP mavlink-router ⇄ MAVSDK bridge/mission manager
                         └───────── Wi-Fi/UDP ── QGC/ground PC
 Browser/ground GUI ── HTTPS/WSS ───────────────┘
```

### 4.1 Phân công tính toán

| Chức năng | TAG | UP 7000 | Pixhawk/PX4 | Ground PC |
|---|---:|---:|---:|---:|
| DW1000 IRQ, timestamp TX/RX, DS-TWR | **Chính** | Không | Không | Không |
| Antenna-delay register, PHY profile | **Chính** | Cấu hình/kiểm tra | Không | Wizard |
| Range thô và DW1000 diagnostics | **Thu thập** | **Xử lý** | Không | Hiển thị |
| NLOS score, adaptive variance | Tối thiểu | **Chính** | Không | Phân tích |
| Multilateration 2D/3D | Không | **Chính** | Không | Replay |
| IEKF/fixed-lag factor graph | Không | **Chính** | EKF2 riêng | Replay |
| IMU/attitude/rate control | Không | Có thể đọc | **Chính** | Hiển thị |
| Optical flow + rangefinder fusion | Không | Không ở Phase 1 | **Chính** | Hiển thị |
| Auto-calibration | Chế độ đo | **Tối ưu chính** | Cấp attitude/z | Wizard |
| Offboard trajectory | Không | **Tạo setpoint** | **Thực thi + failsafe** | Ra yêu cầu |
| Web/telemetry | Không | **Server** | Cấp telemetry | Client |
| BLE | Provisioning | Optional central | Không bắt buộc | Điện thoại kỹ thuật |

### 4.2 Gán cổng đề xuất

| Đường kết nối | Cổng | Giao thức | Ghi chú |
|---|---|---|---|
| TAG → UP | UP USB #1 + CP2102N | UART 921600 8N1 | Tên cố định `/dev/uwb_tag` |
| UP ↔ PX4 control/estimation/telemetry | UP USB #2 → Pixhawk USB-C | MAVLink 2 qua USB CDC ACM | Udev symlink `/dev/pixhawk`; một link hai chiều |
| Log/archive | UP USB #3 → USB box | filesystem có quota/rotation | SSD phù hợp bay hơn HDD cơ |
| MTF-01 → PX4 | TELEM3 hoặc serial spare | MAVLink_PX4 115200 | 5 V supply, logic 3.3 V |
| TELEM2 | Để trống ở cấu hình đầu | dự phòng | Có thể dùng link fallback/thiết bị khác sau này |
| UP ↔ ground PC | Wi-Fi 5 GHz | HTTPS/WSS + MAVLink UDP tùy chọn | Không là flight-critical link |

PX4 cấu hình mặc định cổng USB-C thành MAVLink với profile `Onboard` cho companion; Linux nhận Pixhawk nối USB dưới dạng `/dev/ttyACM0`.[^29][^9] Vì vậy yêu cầu nối thẳng USB là khả thi và đơn giản hơn việc dành TELEM2 cho DDS. `mavlink-routerd` phải là tiến trình duy nhất sở hữu `/dev/pixhawk`, sau đó chuyển tiếp MAVLink tới MAVSDK cục bộ và QGroundControl qua Wi-Fi. Không cho nhiều process cùng mở thiết bị ACM.

### 4.3 Nguồn, cơ khí và EMI

UP 7000 yêu cầu đầu vào 12 V DC; trang sản phẩm ghi nguồn 12 V/5 A, fanless và giới hạn môi trường cần airflow.[^2] Vì vậy:

- Dùng DC-DC/BEC 12 V riêng, dải đầu vào phù hợp số cell LiPo, có fuse, TVS, LC filter và headroom dòng.
- Không cấp UP 7000 từ rail 5 V của Pixhawk. PX4 cảnh báo companion/peripheral công suất lớn cần nguồn riêng.[^9]
- Đo brownout khi motor thay đổi tải, Wi-Fi phát và CPU stress đồng thời.
- Dùng active cooling/airflow; ghi CPU temperature và thermal throttling vào telemetry. Nghiệm thu theo “không throttle ở ambient dự kiến”, không chỉ theo thử bàn lạnh.
- CP2102 breakout và USB-A cần giá đỡ, strain relief và đầu nối khóa; USB lỏng trên drone là lỗi an toàn.
- Cáp USB Pixhawk phải ngắn, có chống tuột/strain relief và được test rung. Kiểm tra riêng hiện tượng back-power qua VBUS khi Pixhawk và UP có nguồn riêng; không giả định USB sẽ cấp nguồn chính cho flight controller.
- Ba cổng USB ngoài sẽ bị chiếm bởi TAG, Pixhawk và storage. Ưu tiên Wi-Fi qua đầu 10-pin nội bộ; tránh thêm hub không nguồn vào đường bay. Nếu buộc dùng hub, chọn hub công nghiệp có nguồn riêng và fault-injection đầy đủ.
- Nếu USB box chứa **HDD cơ**, chỉ dùng trên bàn hoặc làm archive sau chuyến bay: rung, dòng spin-up và độ trễ có thể gây reset/mất mount. Trên drone nên thay bằng USB SSD; dữ liệu flight-critical hiện hành ghi vào eMMC theo ring buffer rồi đồng bộ bất đồng bộ sang USB storage sau khi disarm.
- UART dùng dây ngắn, xoắn TX-GND/RX-GND nếu phù hợp, đi xa ESC/motor lead.
- Chỉ nối GND/TX/RX giữa CP2102 và TAG, không nối chân 5 V của adapter vào DWM1001C nếu chưa có thiết kế cấp nguồn đã kiểm chứng. DWM1001 hoạt động 2,8–3,6 V.[^6]
- Đặt antenna UWB ngoài vùng che của pin, carbon, UP board, heatsink và Wi-Fi antenna; cố định hướng lắp để hiệu chuẩn có ý nghĩa.
- Tách antenna Wi-Fi/BLE khỏi UWB và dây công suất; ưu tiên Wi-Fi 5 GHz để giảm chen lấn 2,4 GHz với BLE/RC 2,4 GHz.

### 4.4 Khuyến nghị cấu hình UP 7000

- Giữ UP 7000 N100/4 GB hiện tại; không cần nâng RAM để đạt 2D/3D UWB.
- Chạy Ubuntu Server headless; không cài desktop environment và không chạy browser trên UP. GUI render trên máy tính mặt đất.
- Đặt memory limit riêng: nhóm realtime 512–768 MB, ROS 2/internal bus 512–900 MB, web/telemetry 384–640 MB; luôn giữ tối thiểu khoảng 500 MB available trong stress test. Đây là ngân sách ban đầu và phải điều chỉnh theo đo đạc.
- Dùng cửa sổ fixed-lag hữu hạn, bounded queue và log streaming; không nạp toàn bộ rosbag/dataset vào RAM.
- Có thể bật zram 1–2 GB cho đột biến không quan trọng; không dùng USB HDD làm swap trong khi bay vì I/O stall có thể làm trễ estimator.
- Ghi ring buffer hiện hành và crash data lên eMMC; USB storage/SSD dùng log dài hạn với quota/rotation và đồng bộ sau disarm. Nếu box là HDD cơ, thay SSD trước flight release.
- Bật hardware watchdog nếu BIOS/driver hỗ trợ; systemd watchdog cho từng service.
- Tắt sleep/suspend/USB autosuspend trên các thiết bị flight-critical sau khi đã xác minh ảnh hưởng điện năng.
- BIOS fixed-performance vừa đủ, không chạy max turbo nếu gây throttle; ưu tiên timing ổn định.

---

## 5. Giao tiếp TAG ↔ UP 7000 qua CP2102N

### 5.1 Trình tự đưa link lên hoạt động

1. Giữ 115200 với protocol hiện tại để xác nhận dây, driver `cp210x` và parser.
2. Đọc USB VID/PID/serial; nếu adapter không có serial duy nhất thì program serial hoặc nhận dạng theo physical port.
3. Tạo udev rule và symlink ổn định `/dev/uwb_tag`; tuyệt đối không hard-code `/dev/ttyUSB0`.
4. Chạy soak test 24 giờ ở 115200: CRC error, sequence gap, reconnect và CPU load.
5. Chuyển TAG và host đồng thời lên 921600; lặp soak test, EMI test có motor và Wi-Fi.
6. Sau khi protocol v2 hoàn tất, khóa schema và thêm compatibility test v1/v2.

### 5.2 Protocol telemetry v2 đề xuất

Mỗi frame nên có:

| Trường | Mục đích |
|---|---|
| `protocol_version`, `message_type`, `payload_length` | tương thích phiên bản |
| `boot_id`, `sequence` | phát hiện reboot, gap, duplicate |
| `tag_time_us` 64-bit | thời gian monotonic tại nguồn |
| `measurement_time_us` | thời điểm đại diện của phép đo, không phải lúc UART gửi |
| `anchor_id`, `exchange_id`, `ranging_mode` | định danh phép đo |
| `raw_range_mm` | input chính của UP |
| `tag_filtered_mm` | chỉ diagnostic/A-B comparison |
| `valid`, `status`, `age_us` | freshness và lỗi |
| `fp_power`, `rx_power`, `cir_power`, `noise_std` | LOS/NLOS và variance |
| `fp_index`, `peak_index`, `pacc_count` | chất lượng first path |
| `carrier_integrator` | clock correction/health |
| `temperature`, `voltage` nếu có | drift và nguồn |
| `slot_duration_us`, timeout counters | timing health |
| CRC | phát hiện hỏng frame |

DW1000 cung cấp các chỉ báo first path/CIR/noise có thể dùng để ước lượng NLOS; Qorvo dành riêng APS006 Part 3 cho các metrics này.[^10] Không gửi toàn bộ CIR ở 50 Hz trong chế độ bay. CIR chỉ bật theo burst 1–5 giây khi commissioning, khi integrity monitor phát hiện bất thường hoặc trong test chuyên dụng.

Khung nên dùng một trong hai phương án và khóa bằng benchmark:

- **Ít thay đổi:** SOF + length + CRC16-CCITT như hiện tại, parser có resync và fuzz test.
- **Khuyến nghị production:** COBS framing + CRC32C, giữ payload versioned/TLV. COBS giúp tìm lại biên frame sau byte lỗi; CRC32C tăng bảo vệ cho frame diagnostic dài.

Mọi command từ UP xuống TAG phải có `command_id`, ACK/NACK, timeout, retry hữu hạn và tính idempotent. Không cho phép lệnh thay calibration khi PX4 đang armed.

### 5.3 Đồng bộ thời gian

Không dùng `host_receive_time` làm thời điểm đo vì USB/UART có jitter. Thực hiện:

1. TAG dùng clock monotonic độ phân giải microsecond, 64-bit.
2. UP gửi gói timesync chứa `t1`; TAG đóng dấu `t2` lúc nhận và `t3` lúc gửi; UP đóng dấu `t4` lúc nhận.
3. Loại sample có round-trip cao; fit mô hình affine `t_up = a × t_tag + b` trên cửa sổ trượt để ước lượng offset và drift.
4. Mỗi UWB range được đổi sang `CLOCK_MONOTONIC_RAW`/ROS time bằng mô hình trên.
5. PX4 bridge duy trì ước lượng lệch clock qua MAVLink `TIMESYNC`; kiểm chứng time alignment trong ULog và tune `EKF2_EV_DELAY` bằng innovation thay vì dùng thời gian nhận USB làm thời gian đo.
6. Log cả source time và receive time để đo end-to-end latency.

### 5.4 Driver trên UP

`uwb_serial_driver` viết C++17/20, non-blocking, một read thread và ring buffer bounded:

- Không malloc trong đường hot sau khởi tạo.
- Parser chịu được frame phân mảnh, frame ghép, byte rác, CRC sai và reboot giữa frame.
- Publish từng phép đo ngay khi parse xong; không chờ đủ tám anchor thành một snapshot.
- Counters: bytes, frames, CRC error, parse error, sequence gap, late frame, reconnect, queue overflow.
- Khi mất USB: đóng descriptor, backoff có giới hạn, reconnect theo symlink, phát health `INVALID`; không reuse sample cũ.
- systemd service dùng `Restart=on-failure`, watchdog và dependency vào device unit.
- Property/fuzz test parser; replay raw byte stream từ log.

### 5.5 Cổng nghiệm thu link

- 24 giờ liên tục, không crash và không leak memory.
- CRC/parse error bằng 0 trong bench sạch; khi fault injection phải phát hiện 100% frame cố ý làm hỏng.
- Sequence gap < 0,01% ở tải bình thường; mọi gap đều được đếm.
- Rút/cắm USB 100 lần, service tự phục hồi và không cần reboot.
- p99 từ `measurement_time` đến ROS publish được đo và lưu; mục tiêu ban đầu < 10 ms trên bench.
- Có stress đồng thời CPU, disk log và Wi-Fi; không overflow ring buffer.

---

## 6. Nền tảng phần mềm trên UP 7000

### 6.1 Stack khuyến nghị

- Ubuntu 24.04 LTS + ROS 2 Jazzy sau khi qualify; Ubuntu 22.04 + Humble trong prototype.
- C++ cho serial driver, estimator, integrity, PX4 bridge và mission manager.
- Python chỉ cho notebook, offline calibration, log analysis và tooling.
- Eigen cho đại số; Ceres Solver cho batch calibration/robust nonlinear least squares. Ceres hỗ trợ robust loss để giảm ảnh hưởng outlier.[^12]
- GTSAM/iSAM2 được đánh giá cho backend incremental smoother; iSAM2 dùng tái tuyến tính hóa và sắp thứ tự biến tăng dần để tránh batch toàn bộ mỗi lần.[^13]
- FastAPI hoặc tương đương cho REST/WebSocket; frontend TypeScript + React, Three.js/Plotly.
- `mavlink-routerd` sở hữu `/dev/pixhawk` và fan-out MAVLink tới MAVSDK cục bộ/QGroundControl.
- MAVSDK C++ cho telemetry, action/offboard và `Mocap::set_odometry()` để đưa external odometry vào PX4.[^31]
- rosbag2 + raw telemetry + PX4 ULog cho logging.

Không đưa web frontend/backend vào cùng process với estimator. Chia systemd services/cgroups để web hoặc ghi log không thể làm starve localization.

Với RAM 4 GB, ROS 2 chỉ là bus nội bộ tùy chọn. Không cần uXRCE-DDS/Micro XRCE-DDS Agent trên link Pixhawk USB. Nếu đo cho thấy ROS 2 tạo footprint hoặc jitter không cần thiết, giữ cùng schema message nhưng chuyển các node flight-critical sang C++ process/in-process queue; API web không thay đổi.

### 6.2 Cấu trúc repository mục tiêu

```text
companion/
  ros2_ws/src/
    uwb_msgs/
    uwb_serial_driver/
    uwb_range_model/
    uwb_localization/
    uwb_integrity/
    uwb_calibration/
    px4_bridge/
    mission_manager/
    telemetry_gateway/
  web/
    backend/
    frontend/
  config/
    anchors.yaml
    tag.yaml
    calibration.yaml
    px4.yaml
  deploy/
    systemd/
    udev/
    nftables/
  tools/
    replay/
    benchmark/
    calibration/
  tests/
    unit/
    integration/
    sitl/
    hil/
firmware/
  tag/
  anchors/
docs/
  adr/
  wiring/
  calibration/
  flight_test/
```

### 6.3 ROS 2 interface nội bộ

| Topic | Nội dung | Tần số |
|---|---|---:|
| `/uwb/range_raw` | một scalar range + timestamp + diagnostics | theo từng phép đo |
| `/uwb/range_corrected` | range đã sửa bias + variance + LOS score | theo từng phép đo |
| `/uwb/pose_realtime` | pose/velocity low-latency | 50–100 Hz |
| `/uwb/pose_smoothed` | nghiệm fixed-lag, không dùng trực tiếp cho control | 10–30 Hz |
| `/uwb/integrity` | state, covariance, residual, geometry, anchor health | 10–20 Hz |
| `/uwb/anchors` | map/version/calibration hash | khi đổi + 1 Hz |
| `/uwb/calibration/status` | progress/quality/commit state | 1–10 Hz |
| `/system/health` | CPU, RAM, temperature, disk, links | 1 Hz |

Các message phải chứa `timestamp_sample`, frame ID, `map_version`, `calibration_version` và covariance. Không cho node tự đoán đơn vị; range dùng mét trong ROS, millimet chỉ ở wire protocol.

### 6.4 Tài nguyên và ưu tiên tiến trình

- PX4 bridge, serial driver, realtime estimator: ưu tiên cao nhất, C++.
- Integrity monitor và mission manager: ưu tiên kế tiếp.
- Logger: bounded queue; được phép drop dữ liệu visualization trước, không drop raw flight-critical mà không báo counter.
- Smoother/calibration: background core, có deadline budget; tự giảm tần số nếu CPU nóng.
- Web/backend/frontend: cgroup riêng, CPU/memory limit.
- Chỉ dùng `SCHED_FIFO`, CPU isolation hoặc `mlockall` sau khi đo chứng minh cần thiết; cấu hình sai real-time Linux có thể làm cả hệ thống treo.

Mục tiêu tải trong flight profile:

- p99 realtime estimator < 10 ms/update.
- p99 end-to-end UWB measurement → PX4 input < 50 ms.
- Không thermal throttle trong 30 phút stress ở nhiệt độ môi trường dự kiến.
- RAM ổn định; disk logger có quota và cảnh báo trước khi đầy.

---

## 7. Chuỗi thuật toán UWB khuyến nghị

Không có một “filter mạnh nhất” cho mọi trường hợp. Pipeline bền vững là nhiều lớp: calibration vật lý → correction → quality/NLOS → robust spatial solve → dynamic estimator → integrity monitor → PX4 EKF2. Factor graph là backend tăng độ chính xác và calibration, không thay output low-latency một cách mù quáng.

### 7.1 Mô hình phép đo

Với anchor `i` tại `a_i`, TAG tại `p_k`:

```text
z_i,k = ||p_k - a_i|| + b_tag + b_anchor_i + b_power_i + b_NLOS_i,k + n_i,k
```

Trong đó:

- `b_tag`, `b_anchor_i`: antenna delay/electronic bias.
- `b_power_i`: bias phụ thuộc received power/PHY.
- `b_NLOS`: bias dương, biến thiên và không Gaussian.
- `n`: nhiễu LOS gần Gaussian sau calibration.

Qorvo APS011 liệt kê clock, antenna delay, signal-power bias và channel effects là các nguồn lỗi TWR; vì vậy chỉ dùng một Kalman 1D trên distance không giải quyết được nguồn gốc sai số.[^7]

### 7.2 Lớp 0 — ranging trên TAG

- Dùng asymmetric DS-TWR theo công thức chuẩn của DW1000; phương pháp này giảm sai số do lệch clock hơn SS-TWR.[^14]
- Kiểm tra địa chỉ, PAN, sequence, function, độ dài và CRC/FCS.
- Đọc timestamp DW1000 và carrier integrator đúng thời điểm.
- Không clamp khoảng đo về một maximum giả; invalid phải biểu diễn bằng status/NaN ở host.
- Giữ state machine non-blocking, deadline rõ ràng, watchdog.
- Gửi raw result ngay sau mỗi anchor, không đợi kết thúc full cycle nếu protocol cho phép.

### 7.3 Lớp 1 — hiệu chỉnh range

Áp dụng theo thứ tự và version hóa:

1. Antenna delay per device.
2. Residual bias per anchor/pair.
3. LUT hoặc spline range-bias theo RX power từ dữ liệu local.
4. Temperature coefficient nếu chamber/flight log chứng minh có ý nghĩa.
5. Lever-arm từ UWB antenna tới tâm IMU/PX4.

Không fit polynomial bậc cao ngoài miền dữ liệu. Mọi profile có `valid_distance_min/max`, PHY ID, channel, PRF, preamble, firmware hash và temperature range.

### 7.4 Lớp 2 — chất lượng và LOS/NLOS

Tạo `los_probability` hoặc `quality_score` từ:

- chênh lệch RX power và first-path power;
- first-path index so với peak index;
- CIR power/noise/PACC;
- carrier integrator;
- residual lịch sử của anchor;
- tuổi phép đo, timeout streak;
- consistency với giới hạn vận tốc/gia tốc và các anchor khác.

Chính sách:

- LOS tốt: variance nhỏ theo calibration.
- Tín hiệu yếu/khả nghi: tăng variance liên tục, không hard-reject ngay.
- Innovation vật lý không thể hoặc frame lỗi: reject.
- Anchor liên tục NLOS: state `DEGRADED/OFFLINE`, probe chậm.
- Khi hồi phục: probation nhiều sample rồi mới trả full weight.

Machine-learning NLOS chỉ là Phase tùy chọn. Mô hình phải train trên đúng DWM1001, đúng antenna/enclosure và môi trường triển khai; đánh giá calibration của xác suất và chạy shadow mode trước. Không cho black-box model là điều kiện duy nhất để nhận/reject phép đo.

### 7.5 Lớp 3 — robust multilateration tức thời

Mỗi epoch hoặc cửa sổ ngắn giải:

```text
min_p Σ_i ρ( (||p-a_i|| - r_i)² / σ_i² ) + prior_z + prior_motion
```

Khuyến nghị:

- Levenberg-Marquardt/Gauss-Newton, warm start từ state trước.
- Weight `1/σ_i²` động theo quality/NLOS/age.
- Huber loss làm mặc định ban đầu; so sánh Cauchy/Tukey trên replay NLOS.
- Lost initialization: closed-form/linear seed rồi multi-start hữu hạn trong flight volume.
- Bound nghiệm trong geofence mở rộng; nghiệm chạm bound phải báo degraded, không im lặng clamp output.
- Tính Jacobian, `JᵀWJ`, condition number và covariance gần đúng.
- Với đủ redundancy, chạy leave-one-anchor-out/solution separation để tìm anchor gây sai.

#### Phase 2D

Không ép tất cả anchor về một mặt phẳng toán học rồi bỏ qua độ cao. Giải `x,y` với `z_tag` là measurement/prior lấy từ vertical estimate của PX4 (MTF-01 + IMU) và lever-arm/attitude. Ở Phase 2D chỉ gửi XY UWB vào PX4; Z tiếp tục từ rangefinder, nên tránh vòng lặp fusion không cần thiết.

#### Phase 3D

Giải XYZ với tối thiểu bốn anchor không đồng phẳng, nhưng chỉ cho trạng thái `TRACKING` khi hình học, residual và covariance đạt gate. Dùng 6–8 anchor gần đây để có redundancy, không yêu cầu chúng có cùng timestamp.

### 7.6 Lớp 4 — bộ ước lượng realtime

Khuyến nghị một **Iterated EKF (IEKF) trên scalar range bất đồng bộ**:

- State tối thiểu: `p, v`; có thể thêm acceleration bias/anchor bias chậm nếu quan sát được.
- Predict bằng constant-acceleration/white-noise model giữa các measurement.
- Update ngay từng scalar range tại source timestamp.
- Re-iterate 1–3 lần khi phi tuyến lớn.
- NIS/chi-square gate, adaptive measurement covariance.
- Publish predicted state ở 50–100 Hz tại thời gian hiện tại, dù mỗi anchor chỉ 25 Hz.
- Reset/relocalize state machine rõ ràng, không snap vô điều kiện.

Alternative để benchmark là UKF, nhưng không tự động xem UKF là tốt hơn. Chọn bằng Monte Carlo + log replay theo accuracy, consistency, CPU và recovery.

### 7.7 Lớp 5 — fixed-lag smoother/factor graph

Chạy cửa sổ 1–3 giây trên UP 7000, gồm:

- UWB scalar range factors.
- motion/velocity factors.
- rangefinder height factor.
- optical-flow velocity hoặc PX4 local velocity factor khi được phép.
- optional IMU preintegration ở Phase nâng cao.
- robust kernels/switchable constraints cho NLOS.

Nghiệm smoother dùng để:

- phân tích và hiệu chỉnh;
- ước lượng bias chậm;
- tạo trajectory refined cho log/GUI;
- so sánh shadow với realtime estimator.

Không gửi pose đã trễ 1–3 giây thẳng vào controller. Nếu muốn tận dụng smoother cho flight, phải lấy state hiện tại được forward-propagate và chứng minh p99 latency/consistency trong HIL.

Tightly-coupled UWB/INS factor graph đã cho thấy lợi ích trong UAV/NLOS ở nghiên cứu, nhưng độ phức tạp và correlation cao hơn, nên chỉ đưa lên flight sau khi pipeline phân tầng đã đạt chuẩn.[^15][^16]

### 7.8 Integrity monitor — bắt buộc trước khi bay

State machine:

```text
INIT → TRACKING → DEGRADED → INVALID
             ↘ RECOVERING ↗
```

Input:

- số anchor valid và age;
- GDOP/condition number;
- residual RMS/max và normalized residual;
- NIS/NEES trên test có ground truth;
- covariance, horizontal/vertical protection level bảo thủ;
- leave-one-out solution separation;
- estimator deadline, source timestamp age;
- calibration/map/version match.

Output:

- `pose_valid`, `velocity_valid`;
- `tracking_state`, `reason_bits`;
- covariance đã inflate;
- danh sách anchor suspect;
- cho phép/không cho phép arm, position hold, nhận target mới.

Quy tắc an toàn ban đầu:

- Không nhận target mới khi `DEGRADED`.
- Không gửi measurement cũ khi `INVALID`; dừng external odometry để PX4 tự nhận biết mất nguồn.
- Nếu mất một anchor nhưng geometry còn tốt, tiếp tục với covariance tăng.
- Nếu không còn geometry, PX4 giữ ngắn bằng flow/IMU rồi chuyển failsafe theo thời gian đã thử nghiệm.

---

## 8. Lịch radio từ 4 lên 8 anchor

### 8.1 Vì sao không mở rộng cơ học 4 → 8

Hiện tại mỗi anchor dùng bốn UWB frame và có delayed response khoảng 1,23 ms cùng nhiều timeout/guard. Tám giao dịch tuần tự trong 20 ms gần như không còn margin, đặc biệt khi có một anchor mất. Tăng STM32 hay ESP32 không làm giảm airtime UWB.

Trước khi đổi lịch, lấy p50/p95/p99 từ các counter `anchor_slot_duration_us` khi **cả bốn anchor khỏe**. Tính:

```text
N_max = floor((cycle_budget - safety_margin) / slot_duration_p99)
```

Chỉ chọn 8 × 50 Hz nếu phép đo thật chứng minh có margin ít nhất 25–30%; không chọn theo lý thuyết bit-rate của PHY.

### 8.2 Lịch khuyến nghị cho 8 góc hình hộp

Đánh số:

```text
A1=(0,0,0)   A2=(L,0,0)   A3=(0,W,0)   A4=(L,W,0)
A5=(0,0,H)   A6=(L,0,H)   A7=(0,W,H)   A8=(L,W,H)
```

Hai nhóm xen kẽ 20 ms:

- Nhóm T0: **A1, A4, A6, A7**.
- Nhóm T1: **A2, A3, A5, A8**.

Mỗi nhóm tạo một tứ diện không đồng phẳng; mỗi anchor 25 Hz, còn estimator update bất đồng bộ và publish 50–100 Hz. Nếu một anchor lỗi, scheduler chèn anchor tốt từ nhóm kia dựa trên freshness và information gain.

### 8.3 Các phương án protocol cần benchmark

| Phương án | Ưu điểm | Nhược điểm | Quyết định |
|---|---|---|---|
| 4-message DS-TWR tuần tự hiện tại | Ít thay đổi, dễ debug | Airtime lớn | Phase 4-anchor |
| Hai nhóm 4 anchor xen kẽ | Giữ protocol, geometry 3D tốt | Mỗi anchor 25 Hz | Baseline Phase 8-anchor |
| Broadcast/multi-anchor asymmetric DS-TWR | Giảm số frame trùng lặp | Firmware phức tạp, collision/timestamp payload | R&D sau khi 3D baseline đạt |
| TDoA | Tag rate cao | Đồng bộ clock anchor/infrastructure phức tạp | Không dùng ban đầu |

Mọi scheduler phải lưu `measurement_time`, vì các range trong một nhóm vẫn được đo ở thời điểm khác nhau. Estimator scalar-range xử lý đúng thời gian thay vì giả định một snapshot tức thời.

---

## 9. Tích hợp Pixhawk 6C, PX4 và MTF-01

### 9.1 Phiên bản và link PX4

- Pin PX4 theo release stable đã HIL-test; tại thời điểm lập kế hoạch, v1.17.0 là stable.[^17]
- Dùng **MAVLink 2 qua Pixhawk USB-C** làm link companion chính. PX4 ghi rõ USB-C mặc định là MAVLink profile `Onboard`, cấu hình bởi `SYS_USB_AUTO`/`USB_MAV_MODE` và không dùng nhóm `MAV_X_CONFIG` như TELEM.[^29]
- Linux thường nhận thiết bị là `/dev/ttyACM0`; tạo udev rule thành `/dev/pixhawk`, cấp quyền qua group và tắt/disable ModemManager để nó không chiếm cổng.[^9]
- `mavlink-routerd` là owner duy nhất của USB. Nó tạo endpoint loopback cho MAVSDK và UDP endpoint có allowlist cho QGroundControl; TELEM2 được để trống/dự phòng.
- `UXRCE_DDS_CFG` giữ `Disabled` ở kiến trúc này. Không dựa vào uXRCE-DDS qua USB vì đây không phải cấu hình USB CDC chuẩn được tài liệu PX4 hướng dẫn.
- Tạo systemd service và health check cho router/bridge: Pixhawk heartbeat, reconnect counter, RX/TX rate, ODOMETRY send rate, command ACK và clock offset.

Đường gửi UWB:

1. `px4_bridge` dùng MAVSDK C++ `Mocap::set_odometry()` để gửi MAVLink `ODOMETRY`, `frame_id=MAV_FRAME_LOCAL_FRD`, kèm position, velocity đã validate, pose/velocity covariance, reset counter và quality.[^31]
2. PX4 ánh xạ `ODOMETRY` ở `MAV_FRAME_LOCAL_FRD` vào `vehicle_visual_odometry`; đây cũng là message external-position duy nhất trong nhóm này có thể mang linear velocity tới EKF2.[^30]
3. Stream 30–50 Hz; PX4 cảnh báo rate thấp hơn có thể khiến EKF2 không fuse external vision. Luôn dùng thời điểm phép đo đã time-align, không dùng thời gian publish.[^30]
4. Telemetry/action/offboard dùng MAVSDK; QGC nhận cùng dòng MAVLink đã fan-out, không cần TELEM1/TELEM2 cho cấu hình ban đầu.

### 9.2 Hệ tọa độ

Quy ước duy nhất:

- Anchor map/GUI: ENU, mét.
- PX4: NED/FRD.
- Với cùng origin và ENU chuẩn: `p_NED = [y_ENU, x_ENU, -z_ENU]`.
- Body PX4 là FRD; ROS thường dùng FLU. Không được chỉ đổi dấu vị trí mà quên velocity/quaternion/covariance. PX4 cảnh báo rõ hai hệ quy chiếu khác nhau và không tự chuyển đổi cho ứng dụng.[^19]

Viết library `frame_conversions` duy nhất và test:

- basis vectors;
- round-trip ENU↔NED;
- covariance rotation;
- quaternion/lever-arm;
- north/east/up sign trong SITL.

### 9.3 Chiến lược fusion theo giai đoạn

#### Phase 2D an toàn

- MTF-01/rangefinder → PX4 EKF2.
- UWB UP → PX4: XY position; velocity XY chỉ bật sau validation; không gửi Z/yaw/attitude.
- `EKF2_EV_CTRL`: bật horizontal position; thêm velocity sau cổng nghiệm thu.
- PX4 Z reference: range sensor.
- PX4 attitude/yaw: IMU/magnetometer cấu hình hiện có.

#### Phase 3D

- UWB gửi XYZ + velocity + covariance động.
- Rangefinder vẫn hỗ trợ height ở vùng làm việc; không bật hai nguồn Z với covariance quá lạc quan.
- Thực hiện A/B: UWB Z only, range Z only, fusion cả hai.

#### Tightly-coupled nâng cao

Nếu UP trực tiếp fuse raw IMU/flow/range/UWB rồi gửi odometry đã fusion, phải tránh PX4 lại fuse cùng raw flow/range như measurement độc lập. Hai estimator dùng chung measurement nhưng giả định độc lập sẽ trở nên over-confident. Chỉ chuyển kiến trúc sau khi viết rõ correlation strategy hoặc tắt fusion trùng ở PX4.

### 9.4 MTF-01

MTF-01 công bố UART 115200 3,3 V LVTTL, output 100 Hz, optical-flow FOV 42°, ToF tối đa 8 m trong điều kiện phản xạ/ánh sáng chỉ định, độ chính xác range 4 cm dưới 2 m và optical flow cần đủ ánh sáng/nền.[^22]

Checklist:

1. Update MTF-01 firmware và lưu version.
2. Chọn `Mavlink_PX4`; với hướng dẫn MicoAir cho PX4 1.17 trở lên, kiểm tra thêm yêu cầu `MAV_PROTO_VER=1` trên đúng MAVLink instance.[^22]
3. Chọn serial Pixhawk riêng cho MTF-01; TELEM2 đang trống nên có thể dùng nếu pinout/nguồn/instance được cấu hình đúng.
4. Đặt rotation đúng bằng `SENS_FLOW_ROT`.
5. Đo chính xác offset camera so với IMU và cấu hình `EKF2_OF_POS_X/Y/Z`.
6. Bật `EKF2_OF_CTRL`, range control/height reference phù hợp bản PX4.
7. Kiểm tra flow quality, scale X/Y và range ở nhiều mặt sàn/ánh sáng/độ cao/tilt.
8. Test nền trơn, bóng, tối, ánh sáng mặt trời và vibration.

PX4 chỉ fuse optical flow khi có rangefinder hợp lệ, flow được bật và quality vượt ngưỡng; covariance/quality phải được quan sát trong log.[^23]

### 9.5 Offboard và failsafe

PX4 yêu cầu proof-of-life Offboard liên tục trên 2 Hz và sẽ thoát Offboard theo `COM_OF_LOSS_T`/`COM_OBL_RC_ACT` nếu mất luồng.[^24]

Thực thi:

- Mission manager trên UP publish setpoint 10–20 Hz, không phải browser.
- Dùng trajectory jerk-limited, giới hạn velocity/acceleration/jerk.
- Trước khi arm/offboard: UWB tracking, PX4 local position valid, flow/range healthy, map/calibration match, Wi-Fi không bắt buộc.
- RC/manual takeover luôn sẵn sàng trong flight test.
- Thiết lập offboard-loss, position-loss, RC-loss và low-battery failsafe trong QGC; test bằng fault injection có cánh tháo trước, sau đó tether.
- Browser chỉ gửi “mục tiêu mong muốn”; mission manager xác nhận geofence/height/health rồi mới sinh trajectory.
- Không expose motor/actuator topic qua web API.

---

## 10. Hiệu chuẩn UWB nhiều lớp

### 10.1 Cấp 1 — antenna delay từng thiết bị

Đây là hiệu chuẩn bắt buộc, không phải smoothing. Qorvo nêu antenna delay khác giữa PCB/module và hiệu chuẩn riêng cho từng thiết bị cho độ chính xác cao hơn.[^25]

Quy trình:

1. Tạo một hoặc nhiều reference device đã hiệu chuẩn.
2. Đặt LOS, antenna cùng hướng, khoảng cách biết chính xác, tránh near-field/vật kim loại/người.
3. Thu nhiều khoảng cách, không chỉ một điểm.
4. Ước lượng TX/RX antenna delay theo APS014, lưu theo hardware serial.
5. Cross-check tam giác/đa thiết bị để tránh đẩy toàn bộ lỗi vào một đầu.
6. Ghi nhiệt độ, PHY profile, channel, PRF, preamble.
7. Flash/store profile có CRC/version; readback verify.

Một phép đo cặp chỉ quan sát tổng delay của hai endpoint; không thể tự động tách hoàn hảo `tag delay` và `anchor delay` nếu không có reference/ràng buộc. Với một TAG, có thể thêm per-pair residual, nhưng không được gọi đó là delay vật lý duy nhất của anchor.

### 10.2 Cấp 2 — bias theo power/khoảng cách/hướng

- Tạo jig/grid khoảng cách ground truth.
- Thu LOS ở nhiều khoảng cách, bốn hướng yaw và vài pitch/roll đại diện.
- Fit LUT 1D/2D có regularization; giữ holdout set.
- Không dùng sample NLOS để fit LOS bias.
- Nếu enclosure/drone frame thay đổi, profile hết hiệu lực và phải calibrate lại.

### 10.3 Cấp 3 — tọa độ anchor 2D/3D

#### Phương án chính xác nhất

Survey bằng laser/total station, sau đó dùng auto-calibration để refine. Đây là baseline để biết thuật toán auto-calibration có thật sự đúng.

#### Anchor-to-anchor self-calibration

Firmware anchor cần calibration mode cho phép đổi initiator/responder và đo các cạnh graph:

1. Thu pairwise distance matrix, median/MAD và LOS score cho từng cạnh.
2. Loại graph kém connected/cạnh NLOS.
3. Khởi tạo bằng metric MDS/LMDS.
4. Khóa gauge và align vào ENU bằng Procrustes/rigid transform.
5. Refine nonlinear least squares/factor graph với robust loss.

MDS đã được triển khai thực tế cho self-localization anchor, nhưng vẫn cần cơ chế thu range và ràng buộc map.[^26] Nghiên cứu UAV gần đây cũng dùng LMDS + Procrustes với một số anchor biết tọa độ rồi mới factor-graph fusion.[^27]

Gauge bắt buộc:

- 2D: cố định A1 tại origin, A2 trên +X, A3 chọn dấu +Y; tốt hơn là ba anchor survey không thẳng hàng.
- 3D: cố định A1, A2 trên +X, A3 trong XY và dùng A4 không đồng phẳng/chiều gravity để loại reflection. Khuyến nghị survey ít nhất bốn anchor không đồng phẳng.
- Pairwise range có đơn vị mét nên cung cấp scale, nhưng translation/rotation/reflection vẫn không tự biến mất.

### 10.4 Cấp 4 — bundle adjustment với TAG di động

Biến tối ưu:

- `anchor_position_i`;
- `tag_pose_k` hoặc `tag_position_k`;
- per-anchor/per-pair bias;
- optional temperature coefficient và flow scale.

Factors:

- corrected UWB ranges;
- MTF-01/PX4 height;
- optical-flow/PX4 velocity;
- PX4 attitude cho lever-arm;
- survey priors/baseline constraints;
- motion smoothness/IMU preintegration ở Phase nâng cao.

Data collection phải kích thích đủ mọi trục:

- 2D: đi quanh perimeter, đường chéo, gần/xa từng anchor.
- 3D: nhiều lớp độ cao, đường chéo không gian, lên/xuống và yaw khác nhau.
- Một vòng tròn phẳng không đủ quan sát tốt tọa độ Z của anchor.

### 10.5 Commit/rollback calibration

Auto-calibration không được ghi đè trực tiếp file active.

```text
DRAFT → OPTIMIZED → VALIDATED_ON_HOLDOUT → APPROVED → ACTIVE
                                              ↘ ROLLBACK
```

Gate commit:

- cost giảm trên train và holdout;
- anchor displacement nằm trong bound vật lý;
- covariance/condition number đạt ngưỡng;
- không có reflection/axis swap;
- replay cũ không regression;
- người vận hành nhìn 3D map và xác nhận;
- file có version, hash, timestamp, source run IDs.

Online flight chỉ được cập nhật **bias chậm có bound** khi LOS tốt và motion đủ kích thích. Không cho optimizer tự di chuyển vị trí anchor trong lúc bay production.

---

## 11. Wi-Fi, telemetry và GUI mặt đất

### 11.1 Wi-Fi

UP 7000 không có Wi-Fi mặc định; nhà sản xuất ghi Wi-Fi/Bluetooth là tùy chọn qua đầu 10-pin và không có expansion slot chung.[^2]

Ưu tiên:

1. Module/kit Wi-Fi được UP hỗ trợ qua 10-pin, có external antenna.
2. Nếu không có, USB Wi-Fi industrial có driver ổn định trên kernel đã pin.
3. Drone kết nối như client vào router 5 GHz riêng; AP mode trên drone là fallback commissioning.
4. Static DHCP reservation, không hard-code interface name.
5. WPA2/WPA3, SSH key, tắt password login, firewall, TLS; remote access ngoài LAN qua WireGuard.
6. QoS ưu tiên command/telemetry; video/log download bị giới hạn bandwidth.

Mất Wi-Fi phải chỉ làm GUI stale. Drone xử lý theo mission/offboard nội bộ hoặc PX4 failsafe, không phụ thuộc browser heartbeat.

### 11.2 Các luồng telemetry trên cùng link Pixhawk USB

- **QGroundControl:** MAVLink2 từ Pixhawk USB → `mavlink-routerd` → UDP 14550 tới ground PC. Giữ QGC cho firmware, parameter, calibration, flight mode và ULog.
- **MAVSDK/mission manager:** endpoint loopback riêng từ router; không phụ thuộc Wi-Fi.
- **Custom GUI:** REST cho cấu hình/history, WebSocket cho live state. Không thay QGC trong Phase đầu.

### 11.3 Tính năng GUI theo thứ tự

#### MVP quan sát — trước khi có command

- 2D/3D scene anchor và drone.
- Raw/corrected range từng anchor.
- valid/status/age/FPP/RX power/NLOS score.
- residual, covariance ellipse/ellipsoid, GDOP, tracking state.
- PX4 armed/mode/battery/local position/EKF health/failsafe.
- Flow quality, rangefinder, CPU temperature, link rates.
- Start/stop/annotate log; replay timeline.

#### Calibration wizard

- Inventory serial/firmware/antenna-delay.
- Hướng dẫn đặt khoảng cách/góc/anchor.
- Thu sample với progress và quality gate.
- Plot residual/holdout.
- Preview map trước commit; version/rollback.

#### Command — chỉ mở sau flight hold đạt chuẩn

- Click/nhập target ENU x-y-z-yaw tùy chọn.
- Hiển thị trajectory preview và geofence.
- Hai bước xác nhận cho arm/takeoff/land.
- Speed/acceleration/jerk limit.
- Pause/hold/return/land; manual override status.
- Reject command nếu pose stale, integrity degraded, target ngoài volume hoặc PX4 không ready.

### 11.4 API an toàn

- Authentication + role `viewer/operator/admin`.
- CSRF protection, origin allowlist, TLS.
- Mỗi command có UUID, TTL, operator, timestamp, ACK và audit log.
- Rate limit; không cho replay command cũ.
- Server-side geofence, không tin validation frontend.
- Web process không có quyền truy cập actuator; chỉ gọi mission-manager API nhỏ và typed.

---

## 12. Kế hoạch thực thi theo giai đoạn

Ước lượng dưới đây là **engineering effort cho một người**, không phải cam kết lịch. Tổng tuần của các phase là khoảng **33–54 person-weeks** tùy mức sửa radio protocol, hardware và ground-truth equipment. Việc chuyển sang MAVLink qua USB giảm một phần công tích hợp DDS/TELEM2, nhưng phải dành lại thời gian cho chống tuột cáp, reconnect, back-power và fault injection nên tổng lịch không thay đổi đáng kể.

| Mốc có thể sử dụng | Phạm vi hoàn tất | Một người full-time, cộng dồn |
|---|---|---:|
| Localization 4-anchor trên mặt đất đã định lượng | Phase 0–4 | 11–18 tuần |
| Chuyến bay 2D an toàn đầu tiên, MTF-01 giữ Z | Phase 0–6 | 16–27 tuần |
| Hệ 2D dùng được với Wi-Fi/GUI/mission manager | Phase 0–8 | 22–36 tuần |
| Auto-calibration + 8-anchor 3D + hardening | Phase 0–11 | 33–54 tuần |

Để lập lịch thực tế, nên dự trù **9–15 tháng nếu một người làm full-time**; **18–30 tháng nếu làm part-time khoảng 15–20 giờ/tuần**. Hai kỹ sư mạnh chia embedded/UWB và PX4/companion/web có thể hạ thời gian lịch xuống khoảng **6–9 tháng**, nhưng các gate bay vẫn phải đi tuần tự. Đường găng là thu ground truth, hiệu chuẩn, phân tích ULog và lặp flight test, không phải số dòng code.

### Phase 0 — Đóng băng baseline và yêu cầu (1–2 tuần)

**Công việc**

1. Tạo monorepo/branch release và ADR.
2. Lưu firmware hashes, wiring, anchor IDs, calibration mask.
3. Chuẩn hóa run ID và metadata.
4. Sửa trạng thái A2–A4 timeout; calibrate đủ A1–A4.
5. Thu bộ log LOS tĩnh/động có ground truth.
6. Viết replay test cho protocol và host filter hiện tại.
7. Chốt flight volume, tốc độ tối đa, độ cao, vật liệu sàn, ánh sáng và môi trường NLOS.

**Deliverable**

- `BASELINE_4ANCHOR.md`.
- Bộ raw log/reference trajectory.
- Báo cáo link success/latency/range error từng anchor.

**Gate**

- 4/4 anchor valid đồng thời.
- Calibration mask đúng 4 anchor.
- Không queue drop/cycle overrun trong soak test.
- Không bắt đầu position solver nếu gate này chưa đạt.

### Phase 1 — Hardware companion, nguồn và OS (2–3 tuần)

**Công việc**

1. Ghi nhận cấu hình N100/4 GB, xác nhận dung lượng eMMC, loại media trong USB box (HDD hay SSD) và Wi-Fi option.
2. Thiết kế power budget/BEC/cooling/mount.
3. Clone Ubuntu 22.04 image.
4. Dựng Ubuntu 24.04/Jazzy image thử riêng.
5. Test đồng thời ba nhánh USB: CP2102 TAG, Pixhawk CDC ACM và storage; sau đó test Wi-Fi, Ethernet, watchdog, thermal.
6. Cài CI/build dependencies và pin versions.

**Gate 24.04**

- Boot 100 lần không lỗi.
- TAG USB và Pixhawk USB reconnect độc lập, tự trở lại đúng symlink; UART TAG 921600 ổn định.
- Wi-Fi driver ổn định; storage bị rút hoặc treo I/O không được chặn estimator/PX4 bridge.
- 24 giờ CPU+disk+network stress không throttle/brownout/crash; RAM available không thấp hơn budget và không swap I/O trên USB khi chạy flight profile.

Nếu fail, dùng 22.04/Humble cho prototype và mở issue migrate trước 2027; không nâng cấp nửa chừng hệ bay.

### Phase 2 — TAG ↔ CP2102N ↔ UP (2–3 tuần)

**Công việc**

1. Udev stable naming.
2. C++ serial driver + parser v1.
3. Protocol v2 + timestamp/timesync/diagnostics.
4. Firmware command/ACK, reconnect state machine.
5. 115200 → 921600 migration.
6. Fuzz/property/soak/EMI tests.

**Gate**

- Đạt toàn bộ mục 5.5.
- Raw log replay bit-for-bit deterministic.
- Tắt host filter vẫn nhìn được range thô không clamp sai.

### Phase 3 — Calibration vật lý và range-quality model (3–5 tuần)

**Công việc**

1. Antenna delay per device.
2. Residual bias/range-power dataset.
3. Bổ sung DW1000 diagnostics.
4. LOS/NLOS scoring baseline deterministic.
5. Calibration file schema/version/rollback.
6. Test antenna orientation, drone electronics on/off, motor on/off.

**Gate**

- LOS per-range median absolute error mục tiêu ≤ 5 cm.
- p95 absolute error mục tiêu ≤ 10–15 cm trong miền đã calibrate.
- Không có vùng clamp/plateau giả.
- NLOS làm covariance tăng hoặc state degraded; không âm thầm coi là chính xác.

### Phase 4 — Localization 4 anchor/2D trên mặt đất (3–5 tuần)

**Công việc**

1. Anchor map ENU + survey.
2. Robust constrained WNLS x-y với z prior.
3. IEKF scalar range bất đồng bộ.
4. Integrity state machine và covariance.
5. Monte Carlo synthetic; log replay; trolley/hand-carried paths.
6. So sánh Huber/Cauchy, gate/adaptive variance.

**Gate**

- Static horizontal RMSE mục tiêu ≤ 10 cm, p95 ≤ 20 cm.
- Dynamic 0–2 m/s RMSE mục tiêu ≤ 20 cm, p95 ≤ 35 cm.
- Output ≥ 50 Hz; p99 estimator < 10 ms; p99 end-to-end < 50 ms.
- Recovery sau mất một anchor có trạng thái/covariance đúng, không teleport.
- Chỉ khi ground truth xác nhận mới chuyển sang Pixhawk.

### Phase 5 — PX4/MAVLink-USB và MTF-01, chưa UWB control (2–4 tuần)

**Công việc**

1. Pin PX4 stable, MAVSDK và MAVLink dialect/version.
2. Pixhawk USB MAVLink 2 + `mavlink-routerd`; MAVSDK loopback endpoint; MTF-01 trên serial Pixhawk riêng; TELEM2 để dự phòng.
3. SITL kiểm tra frames, MAVLink `ODOMETRY`, covariance, reconnect và timesync.
4. MTF-01 orientation/scale/offset/quality/range.
5. Hover bằng optical flow + rangefinder trước, không UWB.
6. Fault injection flow tối/texture kém/range invalid và rút Pixhawk USB.

**Gate**

- PX4 hold XY/Z bằng MTF-01 trong envelope đã quy định.
- Timestamps và ENU→NED test đúng.
- ULog xác nhận EKF2 fuse flow/range, innovation hợp lý.
- Router/MAVSDK tự reconnect sau Pixhawk reboot; mất USB đưa mission manager về trạng thái không được arm/offboard.
- Manual takeover/failsafe đã thử.

### Phase 6 — UWB external position vào PX4 và bay 2D (3–5 tuần)

**Công việc**

1. Shadow: gửi/ghi UWB nhưng chưa fuse.
2. Fuse UWB XY position với covariance lớn bảo thủ.
3. Tune delay/noise/gates bằng ULog.
4. Thử props-off, restrained rig, tethered hover.
5. Free hover nhỏ; step target chậm; mở rộng envelope tuần tự.
6. Sau đó mới thêm UWB velocity nếu đo chứng minh tốt.

**Gate**

- Tether/free hover 60 s không position jump/failsafe ngoài dự kiến.
- Horizontal hover RMS mục tiêu ≤ 20–30 cm trong LOS.
- Mất một anchor: không mất control nếu geometry còn đủ.
- Mất UWB: PX4 chuyển đúng flow-only/failsafe.
- Mất UP/USB/MAVLink heartbeat: offboard-loss đúng cấu hình.

### Phase 7 — Wi-Fi, telemetry và GUI MVP (3–5 tuần)

**Công việc**

1. Wi-Fi 5 GHz, firewall, SSH keys, TLS.
2. Mở endpoint MAVLink router tới QGC qua Wi-Fi, có firewall/allowlist.
3. Backend/WebSocket + live 2D/3D visualization.
4. Integrity/range/flow/PX4/system health panels.
5. Log control, annotation và replay.
6. Network soak, packet loss, reconnect và security tests.

**Gate**

- Mất Wi-Fi không ảnh hưởng link MAVLink-USB cục bộ/localization/control.
- GUI stale indicator đúng và không hiển thị data cũ như live.
- 24 giờ telemetry không memory leak.
- QGC và custom GUI cùng hoạt động, không saturate link.

### Phase 8 — GUI command và mission manager (3–4 tuần)

**Công việc**

1. Typed command API/audit/auth.
2. Geofence + trajectory jerk-limited.
3. Offboard heartbeat local 10–20 Hz.
4. Target preview, confirm, cancel/hold/land.
5. SITL scenario suite; HIL; tether flight.

**Gate**

- Browser không thể bypass safety gate.
- Wi-Fi disconnect giữa mission cho hành vi đúng requirement.
- Stale/degraded pose bị reject target.
- Offboard loss, position loss, RC takeover đều qua test matrix.

### Phase 9 — Auto-calibration 4 anchor (3–5 tuần)

**Công việc**

1. Anchor calibration mode và pairwise measurement graph.
2. MDS/LMDS initialization.
3. Survey alignment/Procrustes.
4. Bundle adjustment robust với mobile TAG.
5. Wizard preview/holdout/commit/rollback.
6. Online bias estimator chạy shadow.

**Gate**

- Auto map so với survey đạt RMSE mục tiêu ≤ 5–10 cm tùy không gian.
- Localization holdout không tệ hơn manual survey.
- Không commit khi graph degenerate/NLOS/reflection.
- Rollback khôi phục bit-for-bit cấu hình trước.

### Phase 10 — 8 anchor/3D trên mặt đất (4–7 tuần)

**Công việc**

1. Lắp tám góc hình hộp, survey, antenna orientation.
2. Calibrate A5–A8 và toàn mạng.
3. Triển khai hai nhóm tứ diện xen kẽ.
4. IEKF XYZ bất đồng bộ + 8-anchor integrity.
5. Fixed-lag smoother shadow.
6. Test nhiều tầng độ cao, đường chéo 3D, NLOS/fault injection.
7. Benchmark broadcast multi-anchor DS-TWR chỉ sau baseline.

**Gate**

- 3D static RMSE mục tiêu ≤ 10–15 cm, p95 ≤ 25 cm.
- Dynamic 3D RMSE mục tiêu ≤ 20–30 cm trong envelope.
- Vertical p95 mục tiêu ≤ 20–30 cm hoặc tốt hơn sau fusion rangefinder.
- Availability ≥ 99% trong test LOS quy định.
- Anchor fail/recover không tạo jump; covariance/integrity phản ánh đúng.

### Phase 11 — Bay 3D và hardening (4–6 tuần)

**Công việc**

1. Shadow UWB Z, rồi fuse với covariance bảo thủ.
2. Tethered climb/descend/diagonal; free flight envelope tăng dần.
3. NLOS người đi qua, anchor mất nguồn, TAG USB/Pixhawk USB/storage USB/Wi-Fi loss độc lập, CPU overload.
4. Thermal/vibration/EMI/long-duration flight.
5. Golden image, recovery image, runbooks và release checklist.
6. Đánh giá optional tightly-coupled FGO hoặc DWM3000 migration.

**Gate release**

- Test matrix 100% pass hoặc waiver có lý do/risk owner.
- Không known P0/P1 defect.
- Golden image và rollback được thử trên hardware thật.
- Flight logs có thể truy vết firmware/config/calibration/run ID.
- Người vận hành có checklist và emergency procedure.

---

## 13. Ma trận kiểm thử bắt buộc

### 13.1 Unit/simulation

- DS-TWR math, wrap timestamp, unit conversions.
- ENU/NED/FRD/FLU, quaternion và covariance.
- Parser fuzz/CRC/resync.
- WNLS Jacobian numerical check.
- IEKF covariance consistency với Monte Carlo.
- Calibration gauge/degenerate geometry/reflection.
- Synthetic LOS Gaussian, positive NLOS bias, bursts, dropout, delayed packet.
- PX4 SITL: takeoff/hold/target/land/offboard loss.

### 13.2 Bench/HIL

- CP2102 disconnect/reconnect.
- TAG reboot/boot_id/sequence reset.
- UP reboot và process crash.
- TAG USB loss, Pixhawk USB/MAVLink loss, storage loss và Wi-Fi loss độc lập.
- Một/hai anchor mất; bad calibration version; map mismatch.
- Disk full/log rotation.
- CPU 100%, memory pressure, thermal throttle.
- ESC/motor EMI với prop tháo.
- MTF-01 nền tối/trơn/range out-of-range.

### 13.3 Flight test progression

```text
SITL → props-off → restrained rig → tethered hover
     → free hover nhỏ → slow XY path → target steps
     → full 2D envelope → tethered 3D → full 3D envelope
```

Không bỏ cấp. Mỗi cấp có pre-flight checklist, abort condition, safety pilot và log review trước cấp tiếp.

### 13.4 Ground truth

Thứ tự ưu tiên:

1. Motion capture/total station cho validation chính xác cao.
2. AprilTag/VIO camera đã calibrate, time-aligned.
3. Laser-survey grid và trolley path.
4. Đo thủ công chỉ cho smoke test, không dùng để tuyên bố RMSE động.

Mỗi báo cáo phải có median, RMSE, p95, p99, max, availability, latency p50/p95/p99 và phân tách LOS/NLOS. Không chỉ dùng “đường nhìn mượt”.

---

## 14. Cấu hình, logging và truy vết

### 14.1 Một run phải chứa

- `run_id`, UTC/local time, operator, test case.
- TAG/anchor/UP/PX4/GUI git hashes.
- PX4 parameter export và airframe.
- Anchor map/calibration/tag profile hashes.
- UWB PHY/scheduler/protocol version.
- Raw UART bytes, parsed ranges, corrected ranges, realtime/smoothed poses.
- Integrity state, system health, ROS diagnostics.
- PX4 ULog và MTF-01 status.
- Ground truth và time-alignment metadata.
- Annotation sự kiện: người chắn, mất anchor, target command, mode switch.

### 14.2 Chính sách cấu hình

- YAML có JSON Schema hoặc equivalent validation.
- Mọi length/time/noise có unit trong tên hoặc typed structure.
- Không magic number runtime trong source.
- Calibration immutable theo version; active chỉ là symlink/reference.
- Config change khi armed bị từ chối.
- Startup fail-closed nếu map, anchor IDs hoặc calibration hash không khớp.

---

## 15. Rủi ro chính và biện pháp

| Rủi ro | Hậu quả | Phát hiện | Giảm thiểu |
|---|---|---|---|
| NLOS/multipath | bias dương, nhảy pose | diagnostics/residual/solution separation | adaptive variance, robust loss, geometry, failsafe |
| 8 anchor quá airtime | overrun/dropout | slot p99/cycle overrun | hai nhóm tứ diện, async update, protocol R&D |
| Calibration sai/unobservable | map đẹp giả nhưng lệch | holdout/survey/condition number | gauge cố định, priors, approval/rollback |
| Timestamp/latency sai | EKF innovation/dao động | source vs receive latency | two-way sync, sample timestamp, delay tuning |
| Double fusion | covariance quá tự tin | innovation/NEES | một owner mỗi raw sensor, kiến trúc phân tầng |
| Flow mất texture/ánh sáng | drift XY | flow quality/PX4 flags | UWB global position, lighting/envelope, failsafe |
| Rangefinder ngoài miền | mất Z | range validity | giới hạn độ cao, UWB Z Phase 3D, failsafe |
| UP brownout/thermal | mất companion/offboard | voltage/temp/watchdog | BEC riêng, cooling, stress test |
| USB rung/lỏng | mất TAG/PX4 link | reconnect counter | cáp ngắn, khóa cơ/strain relief, spare cable, test rung và offboard-loss |
| HDD cơ rung/spin-up hoặc USB I/O stall | mất log, brownout, trễ estimator | I/O latency, mount/power events | ring buffer eMMC, async copy sau disarm, đổi USB SSD trước release |
| RAM 4 GB bị áp lực bởi web/ROS/log | OOM hoặc jitter | memory.current, PSI, OOM counter | headless, cgroup limit, bounded queue/window, zram không flight-critical |
| Wi-Fi mất | mất GUI/GCS | heartbeat/RTT | local autonomy, QGC fallback, không flight-critical |
| MAVLink route/version/frame sai | PX4 không fuse hoặc command sai | Inspector, ACK, ULog innovation | pin versions, ODOMETRY conformance test, SITL/HIL |
| BLE làm tăng jitter | mất UWB deadline | A/B timing/drop test | disabled while armed, low-duty commissioning |

---

## 16. BLE nên dùng thế nào

DWM1001 tích hợp nRF52832, antenna BLE và DW1000/UWB riêng.[^6] BLE có giá trị nhưng không cần nằm trong flight data path:

- Cấu hình anchor ID/role/PHY/calibration version tại hiện trường.
- Điện thoại kỹ thuật đọc health, voltage, temperature, firmware và last-error.
- Anchor placement wizard: scan node gần, bật LED, xác nhận đúng serial.
- Kích hoạt calibration mode hoặc diagnostic CIR burst khi disarmed.
- Firmware update chỉ khi có bootloader bảo vệ, signed image và rollback; nếu chưa có thì không mở OTA.
- Beacon để inventory/tìm anchor, không làm nguồn vị trí chính.

Trình tự:

1. Hoàn tất 4-anchor UWB timing ở BLE-off.
2. Bật BLE advertising interval dài, không connection; A/B slot p99 và range error.
3. Thử connected provisioning; đo UWB IRQ latency/drop.
4. Production ban đầu: tự tắt BLE khi armed.
5. Chỉ cho BLE hoạt động khi bay nếu benchmark chứng minh không regression và use case thật sự cần.

UP 7000 cũng có Bluetooth tùy chọn, nhưng điện thoại có thể nói trực tiếp với DWM1001. Không nên thêm UP-BLE nếu không có use case rõ ràng.

---

## 17. Điểm quyết định đổi phần cứng UWB/TAG

Không đổi sang STM32F4 + ESP32 chỉ vì muốn “nhiều tính toán hơn”; tính toán nặng đã đặt ở UP 7000, còn bottleneck chính có thể là airtime, NLOS và calibration.

Giữ DWM1001C nếu:

- DS-TWR timing ổn định;
- diagnostics cần thiết đọc được;
- 4-anchor 2D và 8-anchor schedule đạt latency/availability;
- BLE không gây deadline regression;
- error targets đạt sau calibration/fusion.

Mở gate migration sang MCU/UWB mới nếu một trong các điều sau lặp lại sau tối ưu:

- không đủ airtime 8-anchor với margin;
- DW1000 diagnostics/timestamp/scheduler không đủ;
- range accuracy/NLOS không đạt cổng dù survey/calibration tốt;
- cần AoA/security/802.15.4z hoặc nhiều TAG hơn;
- firmware nRF không thể đồng thời đáp ứng UWB + BLE timing.

DWM3000 là ứng viên R&D vì hỗ trợ IEEE 802.15.4z và channel 5/9; channel 5 có đường tương thích với hệ DWM1000 ở mức RF, nhưng migration vẫn cần đánh giá protocol/driver/calibration đầy đủ.[^28]

---

## 18. Definition of Done toàn hệ thống

Hệ thống chỉ được coi là hoàn thành khi:

1. Bốn anchor/2D đạt chỉ tiêu và bay hold ổn định với MTF-01/rangefinder.
2. Tám anchor/3D đạt chỉ tiêu static/dynamic, geometry và availability.
3. Covariance/integrity phản ánh đúng lỗi, đặc biệt NLOS và mất anchor.
4. UP/PX4 link có timestamp đúng, p99 latency trong budget.
5. Mất TAG, anchor, Pixhawk USB/MAVLink, UP, storage hoặc Wi-Fi đều có hành vi đã thử và ghi lại.
6. GUI quan sát, calibration và command có authentication/audit/safety gates.
7. Auto-calibration không thể tự commit nghiệm degenerate và có rollback.
8. Golden image/firmware/config/calibration có version và tái tạo được.
9. Test report có ground truth và thống kê, không chỉ đánh giá bằng mắt.
10. Có manual takeover, pre-flight checklist, abort criteria và safety pilot procedure.

---

## 19. Việc nên thực hiện ngay trong sprint đầu

Theo đúng thứ tự:

1. Khắc phục A2–A4 timeout và hiệu chuẩn DS cho đủ bốn anchor; tạo baseline 4-anchor valid.
2. Ghi nhận N100/4 GB, kiểm kê eMMC, xác định USB box là HDD hay SSD, đo dòng khởi động và chốt Wi-Fi nội bộ 10-pin.
3. Mua/xác nhận CP2102N có serial duy nhất; dựng link 115200 rồi 921600.
4. Tạo `uwb_serial_driver` + raw logger trên UP; chưa làm GUI command.
5. Bổ sung source timestamp và DW1000 diagnostics vào telemetry.
6. Thu dataset range LOS/NLOS có ground truth; hiệu chuẩn antenna delay.
7. Viết robust WNLS + replay benchmark 4-anchor trên PC/UP.
8. Nối Pixhawk 6C trực tiếp USB, dựng `/dev/pixhawk` + `mavlink-routerd` + MAVSDK; test 100 lần reboot/rút-cắm và fault injection nguồn.
9. Dựng MTF-01 và chứng minh flow/range hover độc lập; sau đó tích hợp UWB XY bằng MAVLink `ODOMETRY` ở shadow mode rồi mới fuse.
10. Chỉ sau 2D flight gate mới xây command GUI, auto-calibration và 8-anchor 3D.

---

## Sources

[^1]: PX4, “Holybro Pixhawk 6C,” thông số STM32H743, sensor và interfaces. <https://docs.px4.io/main/en/flight_controller/pixhawk6c>
[^2]: UP, “UP 7000,” CPU/RAM/eMMC/I/O/OS/Wi-Fi/power/environment. <https://up-board.org/up-7000/>
[^3]: ROS 2, “Distributions,” ngày EOL của Humble và Jazzy. <https://docs.ros.org/en/humble/Releases.html>
[^4]: ROS 2 Jazzy, “Ubuntu (binary),” hỗ trợ Ubuntu 24.04 amd64/arm64. <https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html>
[^5]: Ubuntu, “Ubuntu release cycle,” vòng đời 22.04/24.04 LTS. <https://ubuntu.com/about/release-cycle>
[^6]: Qorvo/Decawave, “DWM1001 Datasheet,” DWM1001C, DW1000, nRF52832, BLE, nguồn và ranging accuracy. <https://store.qorvo.com/datasheets/qorvo/dwm1001datasheet.pdf>
[^7]: Qorvo/Decawave, “APS011: Sources of Error in DW1000-Based Two-Way Ranging Schemes.” <https://forum.qorvo.com/uploads/default/original/1X/efec77081294e632d9dd4a0f4fe9970c0a762c8c.pdf>
[^8]: Silicon Labs, “CP2102N Data Sheet,” UART 300 baud–3 Mbaud và buffer. <https://www.silabs.com/documents/public/data-sheets/cp2102n-datasheet.pdf>
[^9]: PX4, “Pixhawk + Companion Setup,” ví dụ companion kết nối USB qua `/dev/ttyACM0`, cấu hình link và lưu ý cấp nguồn. <https://docs.px4.io/main/en/companion_computer/pixhawk_rpi>
[^10]: Qorvo/Decawave, “APS006 Part 3: DW1000 Metrics for Estimation of NLOS Operating Conditions.” <https://forum.qorvo.com/uploads/short-url/v27pcxSy7qGzGqXiucYgtUNnCtF.pdf>
[^11]: PX4 v1.17, “uXRCE-DDS,” Agent/client, serial 921600, port parameters và time synchronization. <https://docs.px4.io/v1.17/en/middleware/uxrce_dds>
[^12]: Ceres Solver, “Modeling Non-linear Least Squares,” robust loss functions. <https://ceres-solver.readthedocs.io/latest/nnls_modeling.html>
[^13]: Kaess et al., “iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree,” IJRR 2012. <https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf>
[^14]: Qorvo/Decawave, “APS013: The Implementation of Two-Way Ranging with the DW1000.” <https://forum.qorvo.com/uploads/short-url/x34DrF7EW5fQP9wY3aNESqPKz8z.pdf>
[^15]: Liu et al., “Tightly Coupled Integrated Navigation System via Factor Graph for UAV Indoor Localization,” Aerospace Science and Technology, 2021. <https://www.sciencedirect.com/science/article/pii/S127096382031052X>
[^16]: “Resilient Tightly Coupled INS/UWB Integration Method for Indoor UAV Navigation under Challenging Scenarios,” Defence Technology, 2023. <https://www.sciencedirect.com/science/article/pii/S2214914722002884>
[^17]: PX4, “PX4-Autopilot Releases,” v1.17.0 stable tại thời điểm nghiên cứu. <https://github.com/PX4/PX4-Autopilot/releases>
[^18]: PX4, `px4_msgs`, yêu cầu message definitions khớp firmware. <https://github.com/PX4/px4_msgs>
[^19]: PX4, “ROS 2 User Guide,” DDS recommendation và frame conventions. <https://docs.px4.io/main/en/ros2/user_guide>
[^20]: PX4 v1.17, “PX4 ROS 2 Navigation Interface,” local position measurement và variance. <https://docs.px4.io/v1.17/en/ros2/px4_ros2_navigation_interface>
[^21]: PX4 v1.17, `dds_topics.yaml`, `/fmu/in/vehicle_visual_odometry` và topic mapping. <https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/uxrce_dds_client/dds_topics.yaml>
[^22]: MicoAir, “MTF-01 Optical & Range Sensor,” interface, tần số, range/flow limits và PX4 setup. <https://micoair.com/optical_range_sensor_mtf-01/>
[^23]: PX4, “Using PX4’s Navigation Filter (EKF2),” optical flow/rangefinder và external vision covariance. <https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf>
[^24]: PX4, “Offboard Mode,” proof-of-life >2 Hz và offboard-loss failsafe. <https://docs.px4.io/main/en/flight_modes/offboard>
[^25]: Qorvo, “DW1000” resources, APS014 antenna-delay calibration và APS006/APS011. <https://www.qorvo.com/products/p/DW1000>
[^26]: Corbalán et al., “Self-Localization of Ultra-Wideband Anchors: From Theory to Practice,” Proceedings of the ACM on Interactive, Mobile, Wearable and Ubiquitous Technologies, 2023. <https://disi.unitn.it/~picco/papers/access23.pdf>
[^27]: “Indoor UAV Localization via Multi-Anchor One-Shot Calibration and Factor Graph Fusion,” Remote Sensing, 2026. <https://www.mdpi.com/2072-4292/18/9/1407>
[^28]: Qorvo, “DWM3000,” UWB 802.15.4z, channel 5/9. <https://www.qorvo.com/products/p/DWM3000>
[^29]: PX4, “Serial Port Configuration,” USB-C mặc định là MAVLink với Onboard profile; `SYS_USB_AUTO` và `USB_MAV_MODE`. <https://docs.px4.io/main/en/peripherals/serial_configuration>
[^30]: PX4, “Using Vision or Motion Capture Systems for Position Estimation,” ánh xạ MAVLink `ODOMETRY` vào `vehicle_visual_odometry`, velocity và yêu cầu stream 30–50 Hz. <https://docs.px4.io/main/en/ros/external_position_estimation>
[^31]: MAVSDK, “Mocap Class Reference,” API `Mocap::set_odometry()` và covariance/quality fields. <https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_mocap.html>
