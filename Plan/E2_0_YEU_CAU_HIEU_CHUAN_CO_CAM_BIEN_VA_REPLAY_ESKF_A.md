# E2.0 — Hiệu chuẩn UWB có cảm biến hỗ trợ + replay ESKF-A

**Ngày:** 10/11/2026 · **Code:** Codex · **Review:** Claude
**Hợp đồng:** 14 test đang đỏ:
- `Software/New_GUI/tests/test_e20_aided_calibration.py` (6 test);
- `Software/New_GUI/tests/test_e20_eskf_replay.py` (8 test).

Dữ liệu giả lập dùng chung: `tests/e20_fixtures.py`. **Không sửa ba file này.**

**Khung code:** `Software/New_GUI/aided_calibration.py`, `Software/New_GUI/eskf_replay.py`. Docstring trong hai file là đặc tả chi tiết.

**Dữ liệu thật:** `Logs/10112026/Test_1/`, gồm phân tích trước của Codex trong `analysis/`.

---

## 0. Tóm tắt

Hai việc **offline**, chạy song song, chưa đụng vào UP7000, PX4 hay firmware:

| Gói | Việc | Kỳ vọng |
|---|---|---|
| **E2.0-A** | Hiệu chuẩn UWB có cảm biến hỗ trợ: cố định độ cao TAG bằng rangefinder (đã bù nghiêng và lever arm), lấy hướng mũi từ PX4, rồi giải sai số riêng của từng anchor. Có thể thêm thành phần theo hướng mũi khi dữ liệu có đủ hướng | Xóa độ lệch Z hệ thống, kể cả trên đoạn dữ liệu không dùng để học |
| **E2.0-B** | Bản Python bám đúng `filter.hpp` của ESKF onboard, để replay IMU + range đã ghi | Có công cụ tất định để nghiên cứu và chỉnh bộ lọc onboard; định lượng được nó giúp gì và **không** giúp gì |

---

## 1. Bằng chứng từ Test_1 (người review đã tự kiểm)

1. **Trước khi xoay yaw** (giây 75–225), mọi bộ giải đều cho Z lệch **+12 đến +18 cm** so với tham chiếu rangefinder. Đây là sai số hệ thống ổn định.
   - Cả 7 điểm hiệu chuẩn đều ở Z = 0,51 m (`FLAT_HEIGHTS`).
   - Anchor tầng trên đổi dấu sai số: +5 cm khi ở dưới đất, −5 cm khi bay.
2. **Sau khi xoay yaw khoảng +98°** (giây ~220), các anchor mâu thuẫn nhau: A3 dài ra khoảng 15 cm, A4 ngắn lại. Các bộ giải cho kết quả khác hẳn nhau (IRLS của GUI +26 đến +37 cm, bình phương tối thiểu robust +4 đến +16 cm).
3. **Không phải do nhiệt** (TAG ổn định 33–35 °C), không phải do drone ra mép hộp. Độ cao local của PX4 và barometer đều không thấy drone bay lên.
4. **Thử nhanh hiệu chuẩn có rangefinder** (người review):
   - Học trên giây 75–195, chỉ bias hằng (K = 0), dùng bình phương tối thiểu Huber từng epoch, seed XY từ nghiệm UWB.
   - Kết quả trung vị dz:
     - đoạn kiểm cùng yaw (198–218 s): **+15,6 → +3,7 cm**;
     - đoạn kiểm sau khi xoay (235–300 s): **+9,4 → +0,8 cm**.
   - Độ tản từng burst (p95 khoảng 20 cm) vẫn còn.
5. **Với dữ liệu giả lập**, ESKF 15 trạng thái xử lý tốt xung NLOS (p99 1,0 cm so với 83 cm của bình phương tối thiểu từng epoch) và mất UWB 1 s. Nhưng nó **không** xóa được bias cố định của một anchor (p95 18,3 cm, gần bằng 18,8 cm của bình phương tối thiểu). Test `test_persistent_anchor_bias_is_not_removed` ghi lại giới hạn này để không ai kỳ vọng sai.

---

## 2. Phạm vi file

| File | Trạng thái | Việc |
|---|---|---|
| `Software/New_GUI/aided_calibration.py` | Khung | E2.0-A, theo docstring |
| `Software/New_GUI/eskf_replay.py` | Khung | E2.0-B, theo docstring |
| `Software/New_GUI/tools/flight_log_io.py` | **Mới** | Đọc log thật dùng chung (mục 5) |
| `Software/New_GUI/tools/aided_calibration_report.py` | **Mới** | Công cụ dòng lệnh cho E2.0-A trên log thật (mục 3.2) |
| `Software/New_GUI/tools/eskf_a_replay.py` | **Mới** | Công cụ dòng lệnh cho E2.0-B trên log thật, kèm bảng so sánh (mục 4.2) |
| `Software/New_GUI/README.md` | Cập nhật | Mục "E2.0 offline tools" |

Không sửa `UP4000/estimator` (C++ onboard), cấu hình PX4, firmware hay GUI ở đợt này.

---

## 3. E2.0-A — Hiệu chuẩn có cảm biến hỗ trợ

### 3.1 Thuật toán

Mô hình, quy ước khung và từng bước nằm trong docstring của `aided_calibration.py`. Tóm tắt:

- Phép đo: $r_{ij}=\lVert(x_j,y_j,z^{ref}_j)-\mathbf a_i\rVert+e_i(\theta_{ij})+n$, với $e_i(\theta)=b_i+\sum_{k\le K}(c_{ik}\cos k\theta+s_{ik}\sin k\theta)$.
- $\theta$ là phương vị của anchor nhìn từ TAG, trong **khung thân**: `atan2(dy, dx) − yaw_rig`.
- `yaw_rig` = hướng trục +X thân trong khung rig ENU, tính ngược chiều kim đồng hồ. Từ PX4: `yaw_rig = rig_yaw_offset − yaw_ned`.
- Giải xen kẽ, có trọng số Huber: lần lượt giải XY từng epoch (Z cố định bằng z_ref), rồi hồi quy hệ số cho từng anchor.
- Chỉ fit thành phần theo hướng (K ≥ 1) khi yaw phủ ≥ 4 cung 45°. Ngược lại báo `HEADING_COVERAGE` và lùi về K = 0. Lý do: Test_1 chỉ có 2 hướng, nên K = 1 sẽ chỉ khớp vào sai số phụ thuộc vị trí (ví dụ tọa độ anchor sai), không phải hiệu ứng hướng thật.

### 3.2 Công cụ dòng lệnh trên log thật

```powershell
python tools/aided_calibration_report.py Logs/10112026/Test_1 --train 75:195 --test 198:218,235:300 --harmonics 0 --out Logs/10112026/Test_1/analysis/e20_aided
```

- **Đầu vào** (qua `flight_log_io`):
  - range đã hiệu chuẩn, chỉ lấy mẫu có đủ RADIO_OK + CAL_OK + VALID;
  - z_ref = độ cao TAG từ rangefinder đã bù nghiêng và lever arm, neo theo đoạn đứng yên trước khi bay. Cách tính giống `analysis/analyze_test1.py`, nhưng gom vào hàm dùng chung;
  - `yaw_rig` từ attitude PX4 và `rig_yaw_offset` (mục 5);
  - seed XY từ nghiệm UWB đã ghi.
- **Chỉ dùng các epoch có rangefinder còn mới** (≤ 0,5 s kể từ mẫu gần nhất), vì Test_1 ghi rangefinder chỉ khoảng 1–3 Hz. Báo số epoch bị loại.
- **Đầu ra:**
  - `aided_calibration.json` (`AidedCalibration.to_dict()` kèm cấu hình chạy);
  - `report.json` (holdout từng đoạn kiểm);
  - `dz_before_after.png` (dz theo thời gian, đánh dấu đoạn học và đoạn kiểm).
- **Nghiệm thu trên Test_1** (người review sẽ chạy lại):

  | Chỉ số | Kỳ vọng (số của người review) | Dung sai |
  |---|---|---|
  | Bias thêm học được A1…A8 (mm) | +18, +36, +43, +83, −21, −36, −80, −24 | ±15 mm mỗi anchor |
  | Trung vị dz, đoạn 198–218 s, sau hiệu chỉnh | +3,7 cm (trước +15,6) | \|sau\| ≤ 6 cm |
  | Trung vị dz, đoạn 235–300 s, sau hiệu chỉnh | +0,8 cm (trước +9,4) | \|sau\| ≤ 5 cm |
  | Yêu cầu `--harmonics 1` | `HEADING_COVERAGE`, `harmonics_used = 0` | bắt buộc |

  Nếu không đạt, ghi rõ khác biệt về cách chọn mẫu hoặc tham chiếu. Không chỉnh thuật toán để khớp số.
- **Cảnh báo bắt buộc trong báo cáo:** z_ref giả định sàn phẳng; đoạn kiểm phải không trùng đoạn học; đánh giá Z bằng chính rangefinder là đánh giá **cùng nguồn**, chưa phải độ chính xác tuyệt đối.

---

## 4. E2.0-B — Replay ESKF-A (bản Python của `filter.hpp`)

### 4.1 Thuật toán

Theo docstring của `eskf_replay.py`: chép **đúng** `ESKF::predict`, `ESKF::update`, `range_update`, `so3_exp`, `attitude_from_accel`, `gauss_newton` và logic khởi tạo / khởi tạo lại của node.

Khác biệt duy nhất có chủ đích: lúc khởi tạo, vị trí thân = `tag0 − R0·lever`. Node C++ hiện dùng thẳng `tag0` (lệch khoảng 0,33 m ngay khi khởi tạo). **Ghi điểm này vào mô tả thay đổi** để sửa C++ ở đợt sau.

### 4.2 Công cụ dòng lệnh trên log thật

```powershell
python tools/eskf_a_replay.py Logs/10112026/Test_1 --lever -0.09,0,0.31 --train 75:195 --test 198:218,235:300 --out Logs/10112026/Test_1/analysis/e20_eskf
```

- **IMU:** ULog `sensor_combined` (khoảng 164 Hz). Đổi FRD → FLU **một lần**: `diag(1, −1, −1)`. Dùng `timestamp` + `accelerometer_timestamp_relative`, và giữ dt thật vì mẫu không đều.
- **Range:** từ CSV, theo cùng quy tắc cờ như mục 3.2. Thời gian TAG được ánh xạ sang thời gian boot PX4 bằng mô hình đồng hồ của `analyze_test1.py`.
- **Yaw ban đầu:** `yaw_rig` lúc khởi tạo (mục 5).
- **Bảng so sánh** (chỉ báo cáo, không có ngưỡng), tính riêng trên đoạn học và đoạn kiểm:

  | Hàng | Z so với rangefinder: trung vị / p95 | XY so với GPS đã căn: RMSE / p95 |
  |---|---|---|
  | GUI đã lọc (log) | | |
  | IRLS thô (log) | | |
  | Bình phương tối thiểu Huber từng epoch | | |
  | ESKF-A, tham số mặc định | | |
  | ESKF-A + hiệu chỉnh E2.0-A (K = 0, **chỉ học trên đoạn học**) | | |

  Thêm: tỷ lệ chấp nhận range, NIS trung bình theo anchor, số lần khởi tạo lại, thời gian chạy.
- **Không dò tham số trên đoạn kiểm.** Nếu thử `sig_r` khác, chỉ chọn trên đoạn học và ghi rõ.

---

## 5. `tools/flight_log_io.py` (dùng chung)

Tách các hàm đọc log trong `analysis/analyze_test1.py` thành hàm dùng lại được, có docstring:
- đọc CSV (cột range, cờ, nghiệm GUI);
- ánh xạ đồng hồ TAG → PX4 boot;
- đọc ULog (`sensor_combined`, `vehicle_attitude`, `distance_sensor`, `vehicle_land_detected`);
- đọc cache MCAP đã có (`px4_bridge.json`, `uwb_receive_times.npz`);
- hàm tính `z_ref`;
- hàm ước lượng `rig_yaw_offset` bằng cách căn quỹ đạo UWB với quỹ đạo local NED của PX4: phép quay 2D + tịnh tiến như `align()` của Codex. Trả về cả độ lệch và phần dư căn chỉnh.

`pyulog`, `mcap`, `matplotlib`, `scipy` chỉ import bên trong công cụ (import lười). Thiếu thư viện thì báo lỗi rõ ràng và thoát mã 2. **Không thêm vào `requirements.txt`**, vì GUI không cần chúng.

---

## 6. Việc phần cứng cho người dùng (song song, không phải việc của Codex)

1. **Ghi rangefinder và optical flow ở tốc độ đầy đủ.** Test_1 chỉ có 1–3 Hz.
   - Cách 1: cấu hình topic cho logger PX4 (file `etc/logging/logger_topics.txt` trên thẻ SD; kiểm lại cách cấu hình theo tài liệu PX4).
   - Cách 2: tăng tần số stream MAVLink `DISTANCE_SENSOR` / `OPTICAL_FLOW_RAD` về UP7000 và ghi vào bag onboard.
2. **Thử xoay yaw dưới đất:** đặt drone trên giá để TAG ở khoảng 1,3 m, gần tâm hộp. Xoay 8 hướng (0°, 45°, …, 315°), mỗi hướng 30 s, đánh nhãn waypoint `R0_start` … `R7_end`. Đây là dữ liệu để học thành phần theo hướng (K = 1).
3. **Test_2:** bay ở độ cao cố định trong hộp anchor, đổi yaw theo bước 0/90/180/270°, mỗi hướng giữ 20 s. Đây là dữ liệu kiểm độc lập.
4. **Hiệu chuẩn lại ở 2 độ cao** (ví dụ 0,5 m và 1,4 m), các điểm nằm trong khoảng 25–75% hộp, không đặt trên tường anchor. Đo lại độ cao anchor tầng trên của rig 4,8 m.

---

## 7. Quy tắc

1. Không sửa `tests/test_e20_*.py` và `tests/e20_fixtures.py`. Thấy test sai thì dừng lại và báo lý do.
2. Thư mục làm việc đang có **thay đổi chưa commit của việc khác** (MCAP/onboard, `last_rig_config.json`…). **Không commit, không sửa** các file đó. Chỉ commit file của E2.0.
3. `aided_calibration.py` và `eskf_replay.py` chỉ dùng numpy và thư viện chuẩn (không Qt, không pyulog).
4. Không ghi đè dữ liệu gốc trong `Logs/`. Đầu ra công cụ ghi vào `analysis/e20_*`.
5. Chạy test:

   ```powershell
   cd "Software\New_GUI"
   $env:QT_QPA_PLATFORM = "offscreen"
   python -m unittest discover -s tests -p "test_*.py"
   ```

   Hiện tại: 165 test xanh và 14 test E2.0 đỏ. Kỳ vọng sau khi xong: **179/179**. Riêng test E2.0 mất khoảng 50 s.
6. Commit gợi ý:
   - `feat(e2.0-a): sensor-aided UWB calibration core`
   - `feat(e2.0-b): python mirror of onboard ESKF for replay`
   - `feat(e2.0): flight log io and replay/report tools`
   - `docs(e2.0): …`

---

## 8. Claude sẽ review

1. `git diff` các file hợp đồng phải rỗng; 179/179 test xanh.
2. So bộ giải với lời giải tham chiếu riêng của người review: cùng dữ liệu giả lập thì kết quả phải khớp hoặc rất gần.
3. Đối chiếu từng dòng `eskf_replay.py` với `filter.hpp` (F, Q, Huber bất đối xứng, gate, Joseph, injection, khởi tạo).
4. Chạy lại hai công cụ trên Test_1 và kiểm bảng nghiệm thu ở mục 3.2.
5. Kiểm đầu ra có tách đoạn học/kiểm và có ghi rõ giới hạn tham chiếu cùng nguồn.

---

## 9. Lời nhắn mẫu để giao cho Codex

> Đọc `Plan/E2_0_YEU_CAU_HIEU_CHUAN_CO_CAM_BIEN_VA_REPLAY_ESKF_A.md`. Làm E2.0-A (`aided_calibration.py`) và E2.0-B (`eskf_replay.py`) trong `Software/New_GUI` theo docstring; hợp đồng là 14 test `tests/test_e20_*.py` đang đỏ, không sửa chúng và `tests/e20_fixtures.py`. Sau đó viết `tools/flight_log_io.py`, `tools/aided_calibration_report.py`, `tools/eskf_a_replay.py` và chạy trên `Logs/10112026/Test_1` với các lệnh ở mục 3.2 và 4.2; ghi kết quả vào `analysis/e20_*` và mô tả thay đổi. Không commit hay sửa các thay đổi chưa commit của việc khác trong thư mục làm việc. Kỳ vọng 179/179 test xanh.
