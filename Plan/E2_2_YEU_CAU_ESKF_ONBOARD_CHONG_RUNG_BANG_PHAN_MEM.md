# E2.2 — ESKF onboard chống rung bằng phần mềm (UP7000)

**Ngày:** 10/11/2026 · **Code:** Codex · **Review:** Claude
**Ràng buộc của dự án:** phần cứng drone đã khóa, không đổi cơ khí, giảm chấn hay tham số PX4. Mọi biện pháp đều nằm trên UP7000 (bridge MAVLink, node ESKF, cấu hình GUI).

## Điều kiện trước khi bắt đầu (bắt buộc)

1. **E2.1 đã được nghiệm thu.** E2.2 xây trên `eskf_replay.py` (v2) và `eskf_tuning.py` của E2.1.
2. **Người dùng đã commit phần onboard đang dở**, gồm:
   - toàn bộ `UP4000/` (hiện chưa nằm trong git);
   - trong `Software/New_GUI`: `eskf_config.py`, `onboard_eskf_settings.json`, `onboard_recording.py`, `rosbag_logger.py`, các thay đổi của `main_gui.py`, `up4000_panel.py`…, cùng test của chúng.

   E2.2 sửa đúng những file này, nên cần lịch sử git để review diff.
3. **Chép 3 file hợp đồng từ `Plan/E2_2_hop_dong/` vào đúng chỗ**, giữ nguyên đường dẫn tương đối:
   - `Software/New_GUI/tests/test_e22_onboard_core.py` (5 test);
   - `Software/New_GUI/tests/test_e22_eskf_config.py` (3 test);
   - `UP4000/ros2_ws/src/uwb_px4_logger/test/test_e22_attitude_bridge.py` (4 test).

   Các file này cố ý chưa nằm trong `tests/`, để không làm đỏ bộ test khi Codex còn đang làm E2.1.

**Hợp đồng:** 12 test trên, hiện đỏ trên code trước E2.2 và xanh trên bản tham chiếu của người review. **Không sửa chúng.** Các hợp đồng E2.0/E2.1 cũng phải giữ xanh, không sửa.

---

## 0. Tóm tắt

| Gói | Việc | Kết quả mong đợi |
|---|---|---|
| **E2.2-A** | Bản tham chiếu Python (`eskf_replay.py`): lấy trung bình IMU trực tuyến, khởi tạo tư thế từ PX4, luật đứng yên của node, phục hồi sau khi mất IMU, đổi tư thế PX4 sang khung rig | Một đặc tả chạy được, tất định, để C++ khớp theo |
| **E2.2-B** | Lõi C++ không phụ thuộc ROS (`estimator.hpp`), cùng `attitude_update` trong `filter.hpp`; bộ kiểm parity C++ ↔ Python | Lõi C++ khớp Python tới sai số số học, trên dữ liệu giả lập và Test_1 |
| **E2.2-C** | Bridge MAVLink: HIGHRES_IMU 200 Hz; topic tư thế `/px4/attitude_frd` có tem thời gian theo đồng hồ PX4; xin thêm stream flow và khoảng cách để ghi (chưa fuse) | Node có IMU đủ nhanh để lấy trung bình và có tư thế PX4 |
| **E2.2-D** | Cấu hình GUI → onboard: thêm tham số mới, ô nhập trên GUI | Chỉnh được từ GUI; giá trị mặc định an toàn |
| **E2.2-E** | Node ROS thành lớp vỏ mỏng quanh lõi; smoke test bằng dữ liệu giả lập trên UP7000 | Build và chạy được trên UP7000 mà không cần bay |

Ngoài phạm vi:
- fuse optical flow;
- gửi vị trí UWB cho PX4 (`EKF2_EV_CTRL` giữ nguyên 0);
- công cụ phân tích chuyến bay shadow-mode (E2.3).

---

## 1. Bằng chứng (người review đã tự kiểm trên Test_1)

1. **Node hiện tại không hề chạy trong suốt chuyến Test_1 nếu được khởi động sau khi ARM.**
   - Luồng HIGHRES_IMU 50 Hz lấy mẫu thưa, cộng luật đứng yên của node (\|‖a‖ − 9,81\| ≤ 0,8 và ‖ω‖ ≤ 0,15 liên tục trong 1 s), thì tư thế chỉ khởi tạo được ở **310,06 s, tức sau khi hạ cánh**.
   - Với luồng gốc 221 Hz, chỉ 6,9% số mẫu đạt luật này; chuỗi đạt dài nhất là 0,75 s.
2. **Lấy trung bình 20 ms giúp được lúc đứng dưới đất, nhưng không giúp được giữa chuyến bay.**
   - Ở 57–60 s (đã ARM, motor quay không tải), 99% mẫu đạt và có chuỗi đạt dài 2,3 s.
   - Khi bay, chuỗi đạt dài nhất chỉ 0,42 s.
   - Vì vậy sau `IMU_GAP` giữa chuyến (mất IMU > 0,2 s, `reset_attitude` đòi đứng yên lại), node **không bao giờ hồi phục**. Phải lấy tư thế từ PX4.
3. **Cấu hình kiểu onboard mới** chạy bằng bản tham chiếu trên Test_1:
   - cấu hình: IMU gốc + `imu_average_s` = 0,02 + khởi tạo và cập nhật tư thế từ PX4 + luật đứng yên + `imu_gap_reset_s` = 0,2 + Q (0,293 / 0,0107) + giới hạn 0,02 s mỗi anchor;
   - kết quả:
     - khởi tạo ở 57,74 s, ngay lúc có dữ liệu;
     - không khởi tạo lại; nhận 100% range và 100% tư thế;
     - XY RMSE **10,9 / 16,3 / 23,0 cm**, Z p95 **12,0 / 12,2 / 11,1 cm**;
     - \|dz\|<2σ đạt **0,953**, `rms_normalized` **0,95**.

   Kết quả này ngang với biến thể mean50 + tư thế của E2.1.
4. Các số khác (rung, mật độ nhiễu, độ trôi yaw của pick50, độ nhạy Q, tương quan của range) xem mục 1 của `E2_1_YEU_CAU_TINH_CHINH_ESKF_THEO_RUNG_KHI_BAY.md`.

---

## 2. Phạm vi file

**Sửa:**
- `Software/New_GUI/eskf_replay.py` (E2.2-A). `REPLAY_VERSION` **giữ nguyên `"eskf-a-replay-v2"`**, vì test E2.1 ghim giá trị này;
- `UP4000/estimator/uwb_eskf/include/uwb_eskf/filter.hpp`: thêm `attitude_update` và `log_so3`, không đổi giá trị mặc định của `Params`;
- `UP4000/estimator/uwb_eskf/src/uwb_eskf_node.cpp`, `CMakeLists.txt`, `package.xml`;
- `UP4000/ros2_ws/src/uwb_px4_logger/`: `imu_bridge.py`, `px4_logger_node.py`, `px4_link.py` (chỉ `DEFAULT_RATES`), `config/logger.yaml`, `package.xml`;
- `UP4000/estimator/run_onboard.py`: chỉ thêm `/px4/attitude_frd` vào `TOPICS`;
- `Software/New_GUI/eskf_config.py`, `main_gui.py` (khung cấu hình ESKF).

**Mới:**
- `UP4000/estimator/uwb_eskf/include/uwb_eskf/estimator.hpp`;
- `UP4000/estimator/uwb_eskf/test/` (gtest và dữ liệu parity);
- `Software/New_GUI/tools/export_core_events.py`, `Software/New_GUI/tools/check_core_parity.py`;
- `UP4000/estimator/smoke_test_eskf.py`;
- `E22_CHANGE_DESCRIPTION.md`.

**Không sửa:**
- các hợp đồng E2.0/E2.1/E2.2;
- `last_rig_config.json`;
- tham số PX4 và firmware;
- dữ liệu trong `Logs/`.

---

## 3. E2.2-A — Bản tham chiếu Python (`eskf_replay.py`)

### 3.1 Thêm vào `EskfParams` (giá trị mặc định giữ nguyên hành vi E2.0/E2.1)

```python
imu_average_s: float = 0.0        # >0: lấy trung bình IMU gốc theo khung trước khi vào bộ lọc
attitude_init: str = "static"     # "static" (gia tốc + yaw0) | "external" (AttitudeSample mới nhất)
static_check: bool = False        # luật đứng yên của node
static_acc_tol: float = 0.8
static_gyro_tol: float = 0.15
imu_gap_reset_s: float = math.inf # khe IMU gốc lớn hơn giá trị này -> bỏ bộ lọc và tư thế
```

Thêm vào `ReplayResult` (có giá trị mặc định): `imu_gap_resets: int = 0`, `attitude_inits: int = 0`.

Báo `ValueError` khi:
- `imu_average_s` âm hoặc không hữu hạn;
- `attitude_init` không thuộc hai giá trị trên;
- `imu_gap_reset_s ≤ 0`.

### 3.2 Xử lý một mẫu IMU gốc, đúng thứ tự

Sự kiện vẫn được sắp như E2.1, rồi xử lý lần lượt.

1. **Khe IMU.** Nếu `t − t_IMU_gốc_trước > imu_gap_reset_s`:
   - bỏ bộ lọc và tư thế đang giữ (`carry`), đặt `attitude_ready = False`;
   - xóa tích lũy đứng yên, `previous_imu`, bộ lấy trung bình, bộ đệm khởi tạo và chuỗi từ chối;
   - tăng `imu_gap_resets` thêm 1.

   Tư thế PX4 mới nhất vẫn được giữ, nhưng phải có **một mẫu tư thế mới** thì mới đặt lại `attitude_ready`.
2. **Lấy trung bình** (khi `imu_average_s > 0`):
   - chỉ số khung `k = floor((t − t0) / imu_average_s + 1e-9)`, với `t0` là mẫu gốc đầu tiên kể từ lúc bắt đầu hoặc lúc đặt lại;
   - khi có mẫu thuộc khung mới, khung cũ được đóng và đưa **trung bình của t, acc, gyro** vào bước 3;
   - khung đang mở lúc hết dữ liệu không được đưa ra.

   Thời điểm xử lý là lúc mẫu đóng khung tới. Thời điểm của mẫu ra (dùng cho dt và dòng đầu ra) là t trung bình của khung.
3. **Mẫu vào bộ lọc:**
   - chưa sẵn sàng tư thế và chế độ `"static"`:
     - nếu `static_check` bật và mẫu không đạt luật thì xóa tích lũy;
     - ngược lại cộng dồn; đủ `init_static_s` thì R0 lấy từ gia tốc và `yaw0`, bg0 là trung bình gyro, `attitude_inits` tăng 1;
   - còn lại: giữ tư thế khi chờ khởi tạo lại, hoặc predict, như E2.1.

   Luật đứng yên áp lên mẫu **đã lấy trung bình** khi bật lấy trung bình.

### 3.3 Mẫu tư thế (`AttitudeSample`, khung body FLU → rig)

- Chế độ `"external"`: lưu làm tư thế mới nhất. Nếu chưa sẵn sàng và đã có ít nhất một mẫu IMU gốc, thì đặt sẵn sàng và tăng `attitude_inits`.
- Sau đó là cập nhật tư thế như E2.1, có gate và khoảng thời gian tối thiểu, ở cả hai chế độ.

### 3.4 Khởi tạo vị trí (Gauss–Newton) khi ở chế độ `"external"`

`R` lấy từ tư thế PX4 mới nhất (kể cả khi khởi tạo lại sau chuỗi từ chối). `bg` lấy từ tư thế đang giữ nếu có, ngược lại bằng 0. Chế độ `"static"` giữ như E2.1.

### 3.5 Hàm đổi khung

```python
quat_to_matrix(q_wxyz)                      # Hamilton, chuẩn hóa; ValueError nếu không hữu hạn hoặc |q| ∉ (0,5; 2)
px4_yaw_ned(q_wxyz)                          # atan2(R[1,0], R[0,0])
px4_attitude_to_rig(q_wxyz, rig_yaw_offset)  # Rz(offset − π/2) · T_ned→enu · R_ned_frd · diag(1,−1,−1)
rig_yaw_offset_from_yaw0(q_wxyz, yaw0)       # yaw0 + yaw_ned, để yaw_rig = yaw0 tại mẫu đó
```

Đây cũng là công thức E2.1 dùng trong `flight_log_io`. Node dùng `rig_yaw_offset_from_yaw0` ở **mẫu tư thế đầu tiên** của phiên. Vì vậy ô "Initial yaw" trên GUI giờ còn quyết định độ lệch giữa yaw PX4 và rig. Lệch vài độ chỉ gây sai số TAG cỡ milimét, vì cánh tay đòn ngang chỉ 9 cm.

---

## 4. E2.2-B — Lõi C++ và parity

1. **`estimator.hpp`** (không dùng ROS), `class uwb::Estimator`:
   - đầu vào: `Params` gồm mọi trường của `EskfParams` (E2.0 → E2.2), anchor, lever và `yaw0`;
   - phương thức: `on_imu(t, a_flu, w_flu)`, `on_range(id, t, r)`, `on_attitude(t, R_rig_flu)`, `reset_attitude()`;
   - truy cập: trạng thái, P, các bộ đếm (khởi tạo lại, `skipped_ranges`, tư thế nhận/từ chối, `imu_gap_resets`, `attitude_inits`) và thông tin lần cập nhật gần nhất.
   - Ngữ nghĩa **đúng như mục 3 và E2.1**, gồm giới hạn tốc độ mỗi anchor (`t − t_giữ_cuối[id] < khoảng − 1e-9` thì bỏ), giữ tư thế, cập nhật tư thế, kiểm tra hình học và Gauss–Newton.
   - Lõi xử lý sự kiện **theo thứ tự được đưa vào**, không tự sắp xếp: node đưa theo thứ tự nhận, bộ kiểm parity đưa theo thứ tự Python đã sắp.
2. **`filter.hpp`:**
   - thêm `log_so3` và `ESKF::attitude_update(R_meas, sig_deg, gate, nis&)`: phần dư `log(R̂ᵀR)`, `H = [0 0 I 0 0]`, Joseph, tiêm cả 15 trạng thái;
   - giữ nguyên `Params` mặc định, để harness E2.0 vẫn khớp.
3. **Bộ kiểm parity:**
   - `tools/export_core_events.py` ghi file sự kiện đã sắp, cùng kết quả Python;
   - `test/estimator_parity.cpp` biên dịch bằng g++ + Eigen, không cần ROS, cũng là gtest khi chạy `colcon test`;
   - `tools/check_core_parity.py` so sánh hai bên.
   - Kịch bản:
     - 5 kịch bản giả lập của `test_e22_onboard_core.py`;
     - kịch bản giữ tư thế của E2.1;
     - Test_1 với cấu hình ở mục 1.3.
   - Tiêu chí:
     - cờ nhận/từ chối của **mọi** cập nhật range và tư thế trùng nhau;
     - các bộ đếm trùng nhau;
     - `max|Δp| ≤ 1e-6 m`, góc lệch R ≤ 1e-6 rad, `max|ΔP| / max|P| ≤ 1e-6`.

---

## 5. E2.2-C — Bridge MAVLink (`uwb_px4_logger`)

1. **`ImuSampleClock.to_stamp(device_us)`:** đổi một thời điểm PX4 khác (cùng đồng hồ boot) sang ns, dùng offset của IMU. Không thay đổi trạng thái đồng hồ. Trả về `None` khi chưa có offset hoặc giá trị không hợp lệ.
2. **`odometry_attitude(msg, clock)`:**
   - đầu vào là `ODOMETRY.q` (w, x, y, z; body FRD → local NED) và `time_usec`;
   - trả về `(stamp_ns, q chuẩn hóa)`, hoặc `None` nếu q không hữu hạn hay \|q\| ∉ (0,5; 2), hoặc chưa có đồng hồ.
3. **`px4_logger_node`:** phát `/px4/attitude_frd` (`geometry_msgs/QuaternionStamped`, `frame_id = "px4_local_ned"`, ghi chú "q: body FRD → local NED"); bổ sung số mẫu và số mẫu bị bỏ vào `/px4/status`. Không đổi `/px4/odometry` và `/px4/imu_frd`.
4. **Tần số stream** (`DEFAULT_RATES` và `config/logger.yaml`):
   - `HIGHRES_IMU:200`;
   - thêm `OPTICAL_FLOW_RAD:50`, `DISTANCE_SENSOR:50`. Hai stream này chỉ được ghi vào tlog để nghiên cứu sau, **chưa fuse**;
   - các stream cũ giữ nguyên.

   Kết nối là USB CDC. Sau khi chạy thật, kiểm `rates_hz` trong status: HIGHRES_IMU phải ≥ 180 Hz.
5. `package.xml` thêm `geometry_msgs`. `run_onboard.py` thêm `/px4/attitude_frd` vào `TOPICS`.

---

## 6. E2.2-D — Cấu hình GUI → onboard

1. Thêm vào `eskf_config.PARAMETERS` (giá trị mặc định, thấp nhất, cao nhất):

| Khóa | Mặc định | Khoảng | Ghi chú |
|---|---|---|---|
| `na` | 0,45 | 1e-4 – 20 | khoảng 1,5 × mean50 của Test_1. Q cao hơn thực tế thì an toàn; đo lại sau chuyến shadow |
| `ng` | 0,016 | 1e-5 – 1 | như trên |
| `nba`, `nbg` | 3e-3, 1e-4 | 0 – 1, 0 – 0,1 | như `filter.hpp` |
| `min_anchor_interval_s` | 0,02 | 0 – 0,5 | 0 = dùng mọi range |
| `imu_average_s` | 0,02 | 0 – 0,1 | 0 = predict mỗi mẫu |
| `use_px4_attitude` | 1 | 0 – 1, **số nguyên** | 1 = khởi tạo, cập nhật và phục hồi tư thế bằng PX4; 0 = cách cũ (đứng yên) |
| `att_sig_tilt_deg`, `att_sig_yaw_deg`, `att_gate` | 2, 5, 25 | 0,1 – 30, 0,1 – 90, 1 – 1000 | nâng cao |

2. Kiểm số nguyên áp cho cả `min_init_anchors` và `use_px4_attitude`. `render_files` ghi mọi tham số, thêm `attitude_topic: /px4/attitude_frd`, các khóa cũ giữ nguyên.
3. **GUI (`main_gui.py`):**
   - Thêm ô cho `na`, `ng`, `min_anchor_interval_s`, `imu_average_s` và một checkbox `use_px4_attitude`.
   - Payload chỉ lấy những khóa có ô trên GUI, phần còn lại dùng giá trị mặc định. Hiện `{key: self.eskf_inputs[key] for key in ESKF_PARAMETERS}` sẽ lỗi `KeyError`.
   - Thêm tooltip cho "Initial yaw": khi dùng tư thế PX4, đây là hướng mũi so với trục +X của rig **lúc node khởi động**.
4. `test_eskf_config.py`, `test_onboard_recording.py` và các test GUI hiện có phải vẫn xanh.

---

## 7. E2.2-E — Node ROS và kiểm tra không cần bay

1. **Node thành lớp vỏ mỏng:**
   - đọc ROS param, ánh xạ sang `Estimator::Params`;
   - khi `use_px4_attitude = 1`: đăng ký `/px4/attitude_frd`, đổi q sang `R_rig` bằng offset tính ở mẫu đầu tiên của phiên (giữ nguyên qua các lần đặt lại);
   - giữ các kiểm tra trực tiếp hiện có: tem thời gian còn mới, timeout, cấu hình đã xác nhận;
   - `imu_gap_reset_s = imu_timeout_s`;
   - `IMU_TIMEOUT` theo đồng hồ thật thì gọi `reset_attitude()`; ở chế độ PX4 thì tự khởi tạo lại từ mẫu tư thế kế tiếp.
   - Status JSON thêm: các tham số đang dùng, `attitude_mode`, `rig_yaw_offset_deg`, số cập nhật tư thế nhận/từ chối, `skipped_ranges`, `imu_gap_resets`, `attitude_inits`.
2. **Build trên UP7000.** Người dùng tự chạy; Codex ghi đúng lệnh vào `E22_CHANGE_DESCRIPTION.md`. `colcon build` và `colcon test` cho `uwb_eskf` (gtest parity) và `uwb_px4_logger` (pytest).
3. **`smoke_test_eskf.py`** (numpy + rclpy, chạy trên UP7000, không cần drone):
   - Phát một chuyến bay giả lập 30 s theo thời gian thực:
     - hover rồi bay vòng, có rung giống Test_1;
     - `/px4/imu_frd` 200 Hz (FRD), `/px4/attitude_frd` 50 Hz, `/uwb/anchor_i/range` 8 × 100 Hz.
   - Đọc `/uwb_eskf/odom` và `/uwb_eskf/status`.
   - Đạt khi:
     - trạng thái `RUNNING` trong vòng 1 s;
     - không khởi tạo lại;
     - RMS vị trí so với chuẩn < 5 cm sau 6 s;
     - có cập nhật tư thế và có range bị bỏ qua do giới hạn tốc độ.
   - Lần chạy thứ hai dừng IMU 0,5 s ở giây 15: node phải hồi phục trong vòng 0,5 s và có `imu_gap_resets = 1`.

---

## 8. Nghiệm thu

1. 12 test E2.2 xanh; toàn bộ test E2.0/E2.1/GUI xanh; test của gói `uwb_px4_logger` xanh.
2. Parity C++ ↔ Python đạt tiêu chí mục 4.3 trên cả 7 kịch bản. Báo cáo số lớn nhất của từng chỉ số.
3. Test_1 qua bản tham chiếu Python với cấu hình ở mục 1.3 đạt các ngưỡng của mục 5.1 trong E2.1 (tham chiếu: 10,9 / 16,3 / 23,0 cm; Z p95 ≤ 12,2 cm; 0,953 / 0,95).
4. Người dùng chạy được build, test và smoke test trên UP7000; Codex chuẩn bị lệnh và kết quả mong đợi.
5. Không có thay đổi nào ngoài phạm vi mục 2.

---

## 9. Sau E2.2: chuyến shadow-mode (E2.3, ghi lại để làm sau)

- Bay như bình thường, PX4 vẫn tự bay (`EKF2_EV_CTRL = 0`). Node ESKF chạy với cấu hình mới và ghi bag.
- Kiểm tần số HIGHRES_IMU thực tế; flow và khoảng cách có trong tlog ở 50 Hz.
- Công cụ E2.3:
  - replay bag (input thật) qua bản Python;
  - so với `/uwb_eskf/odom` đã ghi;
  - đo lại nhiễu trên **đúng luồng 200 Hz → trung bình 50 Hz** để chốt `na`/`ng`;
  - xem chất lượng flow trên sàn hiện tại.
- Chỉ khi shadow-mode ổn qua vài chuyến mới bàn tới việc cho PX4 dùng vị trí UWB.

---

## 10. Quy tắc

1. Không sửa các file hợp đồng (E2.0, E2.1, E2.2). Thấy test sai thì dừng lại và báo lý do.
2. Không đổi tham số PX4 hay firmware; không SSH vào drone. Phần chạy trên UP7000 do người dùng làm theo hướng dẫn.
3. Không commit hay sửa thay đổi chưa commit của việc khác. Chỉ commit file của E2.2, gồm 3 file hợp đồng đã chép, giữ nguyên nội dung.
4. Lõi Python chỉ dùng numpy và thư viện chuẩn. Lõi C++ chỉ dùng Eigen và thư viện chuẩn. ROS chỉ xuất hiện trong node.
5. Chạy test:

   ```powershell
   cd "Software\New_GUI"; $env:QT_QPA_PLATFORM = "offscreen"; python -m unittest discover -s tests -p "test_*.py"
   cd "..\..\UP4000\ros2_ws\src\uwb_px4_logger"; $env:PYTHONPATH = "."; python -m pytest -q test
   ```

6. Commit gợi ý:
   - `feat(e2.2-a): replay reference for averaged imu, px4 attitude init and imu-gap recovery`
   - `feat(e2.2-b): ros-free estimator core with attitude update and parity harness`
   - `feat(e2.2-c): px4 attitude topic and 200 Hz imu stream`
   - `feat(e2.2-d): eskf configuration for vibration-aware onboard estimator`
   - `feat(e2.2-e): thin ros node and smoke test`

---

## 11. Claude sẽ review

1. Diff các file hợp đồng phải rỗng; toàn bộ test xanh.
2. Đối chiếu `estimator.hpp` với mục 3 từng nhánh: thứ tự xử lý, khung lấy trung bình, đặt lại khi mất IMU, chọn tư thế lúc khởi tạo, giới hạn tốc độ.
3. Tự chạy bộ kiểm parity; kiểm cả 7 kịch bản và số lớn nhất.
4. Đọc node: các kiểm tra trực tiếp không bị bỏ; offset yaw chỉ tính một lần mỗi phiên; status đủ trường.
5. Đọc smoke test và lệnh chạy trên UP7000 trước khi người dùng chạy.

---

## 12. Lời nhắn mẫu để giao cho Codex

> Đọc `Plan/E2_2_YEU_CAU_ESKF_ONBOARD_CHONG_RUNG_BANG_PHAN_MEM.md`. Trước hết kiểm điều kiện ở đầu file (E2.1 đã nghiệm thu, phần onboard đã được commit), rồi chép 3 file hợp đồng trong `Plan/E2_2_hop_dong/` vào đúng đường dẫn. Làm lần lượt E2.2-A → E: mở rộng `eskf_replay.py` theo mục 3; viết lõi C++ `estimator.hpp` cùng bộ kiểm parity (mục 4); sửa bridge `uwb_px4_logger` (mục 5); thêm cấu hình và ô trên GUI (mục 6); biến node thành lớp vỏ mỏng và viết `smoke_test_eskf.py` (mục 7). Không sửa test hợp đồng, không đổi tham số PX4, không SSH vào drone, không commit thay đổi của việc khác. Viết `E22_CHANGE_DESCRIPTION.md`, gồm kết quả parity và lệnh để người dùng build/test/smoke test trên UP7000.
