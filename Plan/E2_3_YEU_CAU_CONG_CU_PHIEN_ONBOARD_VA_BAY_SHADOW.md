# E2.3 — Công cụ phân tích phiên onboard và chuẩn bị bay shadow-mode

**Ngày:** 11/11/2026 · **Code:** Codex · **Review:** Claude
**Hợp đồng:** 6 test đang đỏ trong `Software/New_GUI/tests/test_e23_onboard_replay.py`. **Không sửa file này** và các hợp đồng E2.0/E2.1/E2.2.
**Khung code:** `Software/New_GUI/onboard_replay.py`, file mới; docstring là đặc tả.

**Dữ liệu thật mới**, người review đã chép từ UP7000 sang (chỉ đọc trên máy):
- `Logs/10112026/Test_1/onboard/eskf/20261110_101854_5500/`: phiên recorder onboard **đúng lúc bay Test_1**, gồm bag MCAP 298 s, `session.json` và `config/`;
- `Logs/10112026/Test_1/onboard/20261110_100743/px4.tlog`: tlog MAVLink của px4_logger, cùng `px4_config.json`, `px4_status.json`, `timesync.csv`.

---

## 0. Tóm tắt

E2.2 đã đạt (mục 1). Bước tiếp theo là **bay shadow-mode**: ESKF onboard chạy và ghi log, nhưng PX4 không dùng. Muốn bay vậy cần ba thứ:
- công cụ đọc và đánh giá một phiên onboard;
- cách đưa bias hiệu chuẩn có hỗ trợ lên onboard;
- quy trình triển khai an toàn.

| Gói | Việc | Kết quả mong đợi |
|---|---|---|
| **E2.3-A** | `onboard_replay.py`: đổi một phiên ghi onboard thành đúng đầu vào mà node E2.2 đưa vào lõi, rồi replay. Thêm bias hiệu chuẩn có hỗ trợ (tùy chọn) vào cấu hình GUI → onboard | Replay offline tái hiện được việc node chạy thật; bật được bias đã học ở E2.0 |
| **E2.3-B** | `tools/onboard_session_io.py`: đọc bag MCAP (nhiều file), cấu hình phiên, tlog (tách theo từng lần PX4 khởi động), ánh xạ đồng hồ UP7000 ↔ PX4 | Một lớp IO dùng chung, tất định |
| **E2.3-C** | `tools/onboard_session_report.py`: kiểm kê luồng, dòng thời gian trạng thái, replay "đúng như đã ghi" và "giả định đã cấu hình", nhiễu IMU, so với tham chiếu RF/GNSS, flow/khoảng cách từ tlog | Đạt bảng nghiệm thu mục 5 trên Test_1 |
| **E2.3-D** | `smoke_test_eskf.py --record-session`: tạo một phiên giả lập giống production để kiểm công cụ | So được node chạy thật với replay mà không cần drone |
| **E2.3-E** | Script triển khai `UP7000/scripts/deploy_onboard_eskf.sh`, mặc định dry-run, có sao lưu và rollback; sửa hướng dẫn build/test | Người dùng tự triển khai an toàn |
| **E2.3-F** | GUI: nhập `aided_calibration.json`; cảnh báo khi bắt đầu log mà lever = 0 hoặc chưa xác nhận cấu hình | Tránh lặp lại lỗi cấu hình của Test_1 |

Ngoài phạm vi:
- fuse optical flow;
- gửi vị trí UWB cho PX4;
- đổi tham số PX4;
- chạy bất cứ thứ gì trên UP7000 hay drone. Việc đó do người dùng làm.

---

## 1. Kết quả nghiệm thu E2.2 và các phát hiện dẫn tới E2.3

1. **E2.2 đạt.** Người review đã tự kiểm:
   - hợp đồng nguyên vẹn; New_GUI 226/226;
   - replay Python khớp bản tham chiếu riêng (≤ 1,8e-15 m, cờ và bộ đếm trùng);
   - parity C++ ↔ Python trên WSL do người review tự xuất lại: 7 nhóm khớp, Δp ≤ 2,1e-14 m;
   - **trên UP7000 thật** (workspace riêng `~/e22_review`, domain 91):
     - build OK; gtest 4/4; pytest logger 33 đạt, 1 bỏ qua;
     - smoke thường: RUNNING sau 0,21 s, RMS 1,25 cm;
     - smoke mất IMU: phục hồi sau 0,11 s.
2. **Hướng dẫn build/test trong `E22_CHANGE_DESCRIPTION.md` cần sửa** (E2.3-E):
   - `colcon test` **phải có cùng `--base-paths`** như lúc build. Nếu thiếu, colcon chạy 0 test mà vẫn báo xong;
   - pytest của logger cần `PYTHONPATH=$HOME/drone_tools/python` để có pymavlink, và cần có thư mục `UP7000/scripts/` (`reset_tag_usb.py`).
3. **Rủi ro CRLF:**
   - PC đặt `core.autocrlf=true`. Chép working tree hoặc dùng `git archive` thường sẽ ra CRLF trên UP7000; khi đó gtest golden lỗi vì `"external\r" != "external"`.
   - `UP7000/.gitattributes` (`eol=lf`) **chưa được track**.
   - Cách sửa:
     - triển khai bằng `git -c core.autocrlf=false archive`;
     - script kiểm không còn ký tự CR;
     - bộ đọc CSV trong `test/estimator_parity.cpp` bỏ `\r` ở cuối dòng.
   - Việc commit `.gitattributes` để người dùng quyết định (mục 7).
4. **Phiên onboard Test_1:**
   - cấu hình ESKF lúc bay **chưa được xác nhận**, `lever = [0, 0, 0]`, `yaw0 = 0`;
   - node đứng `WAIT_CONFIGURATION` suốt chuyến, 0 odom.
   - Luồng dữ liệu:
     - IMU 14 890 mẫu, 50 Hz, dt trung vị 19,55 ms, không có khe > 0,2 s;
     - range 12 522–12 936 mẫu mỗi anchor (khoảng 43 Hz, ít hơn nhiều so với CSV của GUI);
     - `/px4/odometry` 50 Hz;
     - bag bắt đầu ở PX4 boot 45,8 s, **trước khi ARM** (57,4 s).
5. **Replay luồng onboard thật** bằng bản tham chiếu:
   - giả định đã xác nhận cấu hình, lever (−0,09; 0; 0,31), yaw0 lấy từ phép căn GNSS ở mẫu tư thế đầu, tham số mặc định GUI E2.2, tư thế lấy từ `/px4/odometry`;
   - khởi tạo ở 45,80 s, không khởi tạo lại, nhận 100% range và tư thế:

| Biến thể | XY RMSE (cm) 75–195 / 198–218 / 235–300 | XY p95 | Z trung vị | Z p95 | \|dz\|<2σ / RMS chuẩn hóa | Yaw lệch PX4 p95 |
|---|---|---|---|---|---|---|
| không có bias hỗ trợ | 11,0 / 16,1 / 23,5 | 19,2 / 26,0 / 29,6 | **+14,9 / +16,5 / +14,2** | 23,9 / 25,9 / 26,0 | – | – |
| **có bias hỗ trợ (E2.0 K=0)** | **11,1 / 15,2 / 22,4** | 20,5 / 25,9 / 28,2 | +0,5 / +2,3 / +1,0 | 11,7 / 12,5 / 12,1 | 0,966 / 0,93 | 1,7° |

   → Onboard **cần** bias hỗ trợ, nếu không Z lệch khoảng +15 cm.
6. **Nhiễu IMU trên luồng onboard** (FLU, 51 Hz):
   - đứng yên trước ARM (46–57 s): std gia tốc 0,20–0,26 m/s², gyro khoảng 0,7°/s; `na` 0,036, `ng` 0,0017, tức khoảng 20 lần datasheet;
   - bay 75–195 s: `na` 0,948, `ng` 0,0317, khớp pick50 của E2.1.
7. **tlog:**
   - có `OPTICAL_FLOW_RAD` và `DISTANCE_SENSOR` khoảng 10 Hz (flow tích phân 20 ms trong mỗi 100 ms);
   - **tlog chứa 2 lần PX4 khởi động:**
     - lần 1: boot 50,9–808 s, 10:07:44–10:18:03 UTC, flow khoảng 8 Hz;
     - lần 2: boot 1,8–423 s, 10:18:10–10:25:11 UTC, flow 10 Hz. **Chuyến bay nằm ở lần 2** (ULog 57,4–310 s).
   - Chất lượng flow trong 75–300 s của lần 2: trung vị 70 (p10 64, p90 77). Nếu trộn hai lần khởi động thì ra con số sai (74).
   - Phải chọn đúng lần khởi động theo thời gian thật (`_timestamp`) trùng với chuyến bay.

---

## 2. Phạm vi file

**Mới:**
- `onboard_replay.py` (đã có khung);
- `tools/onboard_session_io.py`, `tools/onboard_session_report.py`;
- `UP7000/scripts/deploy_onboard_eskf.sh` (và `rollback`);
- `E23_CHANGE_DESCRIPTION.md`;
- test riêng của Codex.

**Sửa:**
- `eskf_config.py` (`aided_bias_m`);
- `main_gui.py` (nhập bias, cảnh báo);
- `tools/flight_log_io.py` (cho phép `analysis/e23_*`);
- `UP7000/estimator/smoke_test_eskf.py` (`--record-session`);
- `UP7000/estimator/uwb_eskf/test/estimator_parity.cpp` (bỏ `\r`);
- phần lệnh trong `E22_CHANGE_DESCRIPTION.md` và README.

**Không sửa:**
- lõi `eskf_replay.py`, `estimator.hpp`, `filter.hpp`, node C++: E2.3 không đổi toán;
- dữ liệu trong `Logs/`, kể cả thư mục `onboard/` vừa chép;
- các hợp đồng;
- tham số PX4.

---

## 3. E2.3-A — `onboard_replay.py` và bias hỗ trợ trong cấu hình

### 3.1 Replay đúng như node (docstring là đặc tả)

- `node_config(ros_params, rig_text)`:
  - ánh xạ giống hệt `uwb_eskf_node.cpp` của E2.2, kể cả giá trị mặc định khi file `eskf.yaml` cũ thiếu khóa mới. Ví dụ phiên Test_1 không có `use_px4_attitude`, nên node E2.2 sẽ mặc định 1 (external);
  - lever rỗng → `ValueError`, vì nhánh đọc URDF không được replay.
- `parse_rig`: cùng luật với `load_rig` / `load_bias` trong `filter.hpp`.
- `node_events`:
  - gốc thời gian là int64 ns, trừ trước rồi mới đổi sang giây;
  - IMU nhân `imu_R`;
  - range: chưa xác nhận thì bỏ hết; bỏ anchor lạ, giá trị ≤ 0, > 100 m, tem không tăng; trừ bias nếu `ignore_rig_bias = false`;
  - tư thế: offset yaw lấy từ `yaw0` ở mẫu hợp lệ đầu tiên;
  - đếm mọi thứ bị bỏ.
- Node thật còn kiểm độ tươi theo đồng hồ thật và các timeout; replay **không** mô hình hóa việc này. Mọi báo cáo phải ghi rõ.

### 3.2 `aided_bias_m` trong `eskf_config.py`

- Khóa top-level tùy chọn: `aided_bias_m: [[anchor_id, bias_m], ...]`.
  - `anchor_id` là số nguyên, có trong danh sách anchor, không trùng; \|bias\| ≤ 0,5 m;
  - sắp theo id;
  - **chỉ xuất hiện trong cấu hình đã chuẩn hóa khi danh sách không rỗng**, để hash của cấu hình cũ giữ nguyên.
- `render_files`:
  - với anchor đang bật, ghi thêm `anN_bias, b` vào `rig_onboard.txt`, kèm một dòng chú thích;
  - `ignore_rig_bias = not aided`.
- Quy ước dấu: đo = thật + b, node trừ b. Đây là đúng quy ước `bias_m` của `aided_calibration.json` (E2.0).
- Node E2.2 đã hỗ trợ `anN_bias` và `ignore_rig_bias`, nên **không phải sửa C++**.

---

## 4. E2.3-B — `tools/onboard_session_io.py`

1. **Phiên recorder** (`session.json`, `config/eskf.yaml` dạng JSON, `rig_onboard.txt`, `gui_config.json`, `bag/`):
   - đọc mọi `bag_*.mcap` theo thứ tự số; chịu được phiên chưa đóng (không có `metadata.yaml`, file cuối bị cắt dở);
   - dùng `mcap` + `mcap_ros2`, import lười.
2. **Topic cần đọc:**
   - `/px4/imu_frd`, `/px4/imu_device_time_us`;
   - `/px4/attitude_frd` (E2.2); nếu không có thì dùng `/px4/odometry`, quaternion w,x,y,z, đánh dấu nguồn là "odometry, receive-stamped";
   - `/uwb/anchor_N/range`, `/uwb_eskf/odom`, `/uwb_eskf/status` (JSON), `/px4/status`, `/uwb_relay/status`.
   - Header stamp là int64 ns; giữ thêm `log_time`.
3. **Đồng hồ:** `offset_ns = trung vị(stamp_imu − device_us·1000)`. Báo độ trải và số lần nhảy (reset). Thời gian PX4 boot = (stamp − offset)/1e9, dùng để ghép với ULog và CSV. Test_1: độ trải 0.
4. **tlog** (pymavlink, import lười):
   - `HIGHRES_IMU`, `ATTITUDE_QUATERNION`, `OPTICAL_FLOW_RAD`, `DISTANCE_SENSOR`, `VIBRATION`, cùng `_timestamp`;
   - **tách theo lần khởi động** khi `time_usec` / `time_boot_ms` tụt;
   - chọn lần khởi động trùng nhiều nhất với khoảng thời gian của flight (hoặc tham số `--boot`);
   - trường thiếu theo dialect (ví dụ `signal_quality`) thì trả `None`, không làm sập.

---

## 5. E2.3-C — `tools/onboard_session_report.py` và nghiệm thu trên Test_1

```powershell
cd "Software\New_GUI"
python tools/onboard_session_report.py Logs/10112026/Test_1/onboard/eskf/20261110_101854_5500 --tlog Logs/10112026/Test_1/onboard/20261110_100743/px4.tlog --flight Logs/10112026/Test_1 --train 75:195 --test 198:218,235:300 --what-if-confirm --lever -0.09,0,0.31 --yaw0-from-flight --aided-bias-json Logs/10112026/Test_1/analysis/e20_aided/aided_calibration.json --out Logs/10112026/Test_1/analysis/e23_onboard_session
```

**Nội dung báo cáo** (`report.json`, `report.md`, `streams.csv`, `trajectories.npz`, hình):

1. **Kiểm kê:** `stream_stats` cho từng topic; dòng thời gian trạng thái (thời gian ở mỗi state, lý do reset); tham số trong cấu hình phiên.
2. **Replay "đúng như đã ghi":** cấu hình của phiên giữ nguyên. Nếu có `/uwb_eskf/odom` thì so bằng `compare_tracks`.
3. **Replay "giả định đã cấu hình"** (`--what-if-confirm`):
   - ép đã xác nhận, dùng lever đã cho;
   - yaw0 lấy từ `--yaw0-deg` hoặc `--yaw0-from-flight`. Cờ này dùng yaw rig đã căn GNSS chỉ trên đoạn học, tại mẫu tư thế đầu tiên;
   - hai biến thể: không bias, và có bias từ `--aided-bias-json`;
   - các tham số khác lấy mặc định GUI E2.2 (`eskf_config.PARAMETERS`).
4. **Nhiễu** (`eskf_tuning`) trên luồng IMU đã ghi, đổi sang FLU:
   - đoạn đứng yên trước ARM (tự dò bằng trạng thái armed trong ULog hoặc snapshot, hoặc dùng `--static-window`);
   - các đoạn học;
   - đề xuất `na`/`ng` **cho đúng luồng đó**, có ghi rõ tốc độ luồng.
5. **Tham chiếu** (`--flight`): `z_ref` và GNSS đã căn, dùng lại `flight_log_io`. Chấm như E2.1: các epoch CSV khoảng 10 Hz, RF tươi ≤ 0,5 s.
6. **tlog:**
   - tốc độ và chất lượng flow; khoảng cách ở 10 Hz so với `z_ref` (độ lệch, tương quan);
   - vận tốc ngang suy từ flow (`integrated_x/y` trừ gyro tích phân, nhân khoảng cách, chia thời gian tích phân, đổi sang khung rig) so với vận tốc ESKF;
   - **chỉ báo cáo khả thi, chưa fuse**.
7. **Đầu ra:** chỉ ghi vào `analysis/e23_*`. Mở rộng allowlist của `output_directory`.

**Nghiệm thu trên Test_1.** Số trong ngoặc là kết quả của người review.

| Mục | Tiêu chí |
|---|---|
| Kiểm kê | IMU 14 890 mẫu, dt trung vị 19,55 ± 0,2 ms, 0 khe > 0,2 s; 8 anchor, mỗi anchor 12 522–12 936 mẫu; odometry 14 887; độ trải offset đồng hồ < 1 ms; 100% status là `WAIT_CONFIGURATION` |
| Đúng như đã ghi | 0 đầu ra; mọi range nằm trong `ranges_unconfirmed`; khớp node thật (0 odom) |
| Giả định + bias hỗ trợ | Khởi tạo ≤ 47 s boot (45,80); 0 khởi tạo lại; nhận range và nhận tư thế ≥ 0,99. XY RMSE ≤ **12,5 / 18 / 25 cm** (11,1 / 15,2 / 22,4); XY p95 ≤ **24 / 32 / 33** (20,5 / 25,9 / 28,2); \|Z trung vị\| ≤ **3 cm** (+0,5 / +2,3 / +1,0); Z p95 ≤ **15 cm** (11,7 / 12,5 / 12,1). Gộp 75–300 s: \|dz\|<2σ ≥ **0,90** (0,966), RMS chuẩn hóa trong **[0,6; 1,4]** (0,93). Yaw lệch PX4 p95 ≤ **3°** (1,7°) |
| Giả định, không bias | Báo Z trung vị ở mỗi cửa sổ (tham chiếu +14,9 / +16,5 / +14,2 cm), ghi rõ thành phát hiện |
| Nhiễu | Trước ARM: `na` 0,036 ± 20%, `ng` 0,0017 ± 20%. Bay 75–195: `na` 0,948 ± 15%, `ng` 0,0317 ± 15% (luồng 51 Hz) |
| tlog | Thấy 2 lần khởi động; chọn lần 2 (10:18:10–10:25:11 UTC, trùng ULog 57,4–310 s); flow và khoảng cách khoảng 10 Hz; chất lượng flow trung vị 70 ± 3 trong 75–300 s |

Không chỉnh tham số theo đoạn kiểm. Không đạt thì báo kèm số liệu, không nới ngưỡng.

---

## 6. E2.3-D, E, F

1. **`smoke_test_eskf.py --record-session DIR`:**
   - trong domain cách ly, ghi bag MCAP các `TOPICS` của production và viết `session.json` + `config/` giống `run_onboard.py`;
   - Codex chạy trong WSL, rồi chạy `onboard_session_report.py` trên phiên này (không `--flight`): so node thật với replay. Mục tiêu: trung vị ≤ 1 cm, p95 ≤ 3 cm; nếu lệch hơn thì giải thích nguồn (thứ tự đến, kiểm độ tươi), không chỉnh để khớp.
2. **`UP7000/scripts/deploy_onboard_eskf.sh`:**
   - mặc định `--dry-run`, chỉ in ra các việc sẽ làm;
   - lấy nguồn bằng `git -c core.autocrlf=false archive`, kiểm không còn CR;
   - build overlay riêng, chạy `colcon test` đúng `--base-paths` và `PYTHONPATH`;
   - với `--apply`:
     - **sao lưu** binary, gói logger, helper `up7000_eskf_config.py` và `run_onboard.py` đang chạy vào thư mục có tem thời gian;
     - cài bản mới vào đúng chỗ service đang nạp (đường dẫn có trong `E22_CHANGE_DESCRIPTION.md`);
   - chỉ khởi động lại service khi có `--restart`; có `rollback.sh` dùng bản sao lưu;
   - không đụng HDD UUID guard, cấu hình anchor/lever/yaw hay phiên cũ;
   - Codex kiểm dry-run trong WSL bằng một cây thư mục giả. Người dùng mới là người chạy `--apply`.
3. **GUI:**
   - nút "Nhập hiệu chuẩn hỗ trợ" đọc `aided_calibration.json` → `aided_bias_m`; hiển thị bias từng anchor và nút xóa;
   - khi bắt đầu log, nếu lever = (0, 0, 0) hoặc chưa xác nhận cấu hình ESKF thì **cảnh báo rõ** (không chặn), ghi vào manifest;
   - các test GUI hiện có phải giữ xanh; thêm test offscreen cho nút nhập và cảnh báo.
4. **Sửa hướng dẫn trong `E22_CHANGE_DESCRIPTION.md`:** lệnh `colcon test` và `PYTHONPATH` như mục 1.2; thêm `UP7000/scripts`; cách chép không CRLF. Bỏ `\r` trong `estimator_parity.cpp`.

---

## 7. Việc của người dùng (cần bạn quyết định, Codex không làm)

1. **Triển khai lên UP7000:** chạy `deploy_onboard_eskf.sh` (dry-run trước, rồi `--apply --restart`). Kiểm `/px4/status`: `rates_hz.HIGHRES_IMU` ≥ 180, có mẫu `/px4/attitude_frd`.
2. **Có bật bias hỗ trợ cho bay hay không:** nên bật. Số liệu ở mục 1.5: không bật thì Z lệch khoảng +15 cm. Bias học từ một chuyến, cùng rig, và chưa có thành phần theo hướng mũi.
3. **Commit `UP7000/.gitattributes`** (`eol=lf`), hiện đang chưa track.
4. **Trước mỗi chuyến shadow:**
   - nhập lever (−0,09; 0; 0,31) và yaw0 (hướng mũi so với trục +X của rig **lúc bấm bắt đầu log**);
   - tick xác nhận cấu hình;
   - giữ `EKF2_EV_CTRL = 0`.
5. **Kịch bản bay gợi ý:**
   - đứng yên 10 s trước ARM;
   - hover 60 s giữ yaw;
   - đổi yaw theo bước 0 / 90 / 180 / 270°, mỗi hướng 20 s;
   - vài lần đi ngang chậm;
   - hạ cánh.
6. **Sau khi bay:** chép thư mục phiên onboard, tlog và ULog về `Logs/<ngày>/<Test>/`, rồi gọi Claude nghiệm thu bằng `onboard_session_report.py`.

---

## 8. Quy tắc

1. Không sửa `tests/test_e23_onboard_replay.py` và các hợp đồng cũ. Thấy test sai thì dừng lại và báo lý do.
2. Không SSH vào UP7000, không đụng drone hay PX4. Script triển khai chỉ được kiểm bằng dry-run và cây thư mục giả.
3. Không commit hay sửa thay đổi chưa commit của việc khác. Chỉ commit file của E2.3, gồm file hợp đồng và khung, giữ nguyên nội dung.
4. `onboard_replay.py` chỉ dùng numpy và thư viện chuẩn. Công cụ import lười `mcap`, `mcap_ros2`, `pymavlink`, `pyulog`, `matplotlib`. Không thêm vào `requirements.txt`.
5. Không ghi đè dữ liệu gốc hay `analysis/e20_*`, `analysis/e21_*`. Đầu ra mới chỉ ghi vào `analysis/e23_*`.
6. Test: hiện có 226 test xanh và 6 test E2.3 đỏ. Kỳ vọng **232/232**, cộng test riêng của Codex.

---

## 9. Claude sẽ review

1. Diff các file hợp đồng phải rỗng; toàn bộ test xanh.
2. So `onboard_replay` với bản tham chiếu riêng của người review trên phiên Test_1 (cả hai biến thể) và trên dữ liệu giả lập.
3. Tự chạy lệnh ở mục 5 và kiểm bảng nghiệm thu.
4. Đọc kỹ script triển khai: dry-run thật sự không ghi gì; sao lưu và rollback đầy đủ; không khởi động lại service khi không có `--restart`.
5. Kiểm trên GUI: nhập bias, hash cấu hình, cảnh báo lever = 0.

---

## 10. Lời nhắn mẫu để giao cho Codex

> Đọc `Plan/E2_3_YEU_CAU_CONG_CU_PHIEN_ONBOARD_VA_BAY_SHADOW.md`. Trong `Software/New_GUI`: làm `onboard_replay.py` theo docstring và thêm `aided_bias_m` vào `eskf_config.py` (mục 3). Hợp đồng là 6 test `tests/test_e23_onboard_replay.py` đang đỏ; không sửa nó và các hợp đồng cũ. Sau đó viết `tools/onboard_session_io.py` và `tools/onboard_session_report.py` (mục 4–5), chạy lệnh ở mục 5 trên `Logs/10112026/Test_1/onboard/...` và đối chiếu bảng nghiệm thu; làm E2.3-D/E/F ở mục 6 (smoke `--record-session`, script triển khai mặc định dry-run kèm rollback, GUI nhập bias và cảnh báo lever = 0, sửa lệnh `colcon test` / CRLF). Không SSH vào UP7000, không đổi tham số PX4, không commit thay đổi của việc khác. Viết `E23_CHANGE_DESCRIPTION.md`. Kỳ vọng 232/232 test xanh.
