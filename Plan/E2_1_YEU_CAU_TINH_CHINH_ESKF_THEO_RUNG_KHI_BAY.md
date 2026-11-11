# E2.1 — Tinh chỉnh ESKF-A theo rung khi bay (offline)

**Ngày:** 10/11/2026 · **Code:** Codex · **Review:** Claude
**Hợp đồng:** 14 test đang đỏ:
- `Software/New_GUI/tests/test_e21_imu_noise.py` (6 test);
- `Software/New_GUI/tests/test_e21_eskf_tuning.py` (8 test).

Dữ liệu giả lập dùng chung: `tests/e21_fixtures.py` (dựa trên `e20_fixtures.py`, không sửa file đó). **Không sửa ba file E2.1 này và các file hợp đồng E2.0.**

**Khung code:** `Software/New_GUI/eskf_tuning.py`, file mới, docstring là đặc tả. `eskf_replay.py` (của E2.0) được mở rộng theo mục 4.1.

**Dữ liệu thật:** `Logs/10112026/Test_1/`, gồm kết quả E2.0 trong `analysis/e20_*`.

---

## 0. Tóm tắt

Lần replay E2.0 cho thấy ESKF-A **phân kỳ trên dữ liệu bay thật** và báo σ quá nhỏ: RMSE XY 87,8 cm, p95 262 cm, std báo cáo 0,7 cm. Người review đã tìm ra nguyên nhân và thử cách sửa trên Test_1 (mục 1). Bước này gồm ba việc **offline**, chưa đụng tới UP7000, C++, PX4 hay firmware:

| Gói | Việc | Kỳ vọng |
|---|---|---|
| **E2.1-A** | Đo nhiễu IMU khi bay, **trên đúng luồng IMU mà bộ lọc sẽ nhận** (tốc độ, cách giảm mẫu). Từ đó đề xuất `na`/`ng`. Báo cáo kèm chỉ số rung và tham số lọc của PX4 | Có một con số đo được thay cho giá trị datasheet BMI088 (nhỏ hơn thực tế khoảng 300–500 lần) |
| **E2.1-B** | Mở rộng replay: giới hạn tốc độ cập nhật mỗi anchor, giữ tư thế khi khởi tạo lại, cập nhật tư thế từ PX4 (tùy chọn), cùng các chỉ số nhất quán (σ so với sai số thật, tự tương quan innovation) | Bộ lọc không phân kỳ, σ trung thực, heading không trôi |
| **E2.1-C** | Công cụ báo cáo trên Test_1 với luồng IMU mô phỏng đúng onboard (50 Hz), tham số chỉ học từ đoạn 75–195 s | Đạt bảng nghiệm thu ở mục 5 |

Phần sửa node C++ và cấu hình GUI → onboard được ghi ở mục 6 (E2.2), **không làm trong E2.1**.

---

## 1. Bằng chứng từ Test_1 (người review đã tự kiểm)

Tham chiếu dùng để chấm: Z so với rangefinder (cùng nguồn đã dùng để học hiệu chuẩn có hỗ trợ), XY so với GNSS đã căn trên đoạn học. Cả hai đều **không** phải ground truth.

1. **Rung rất lớn, không bị clipping.**
   - Độ lệch chuẩn từng mẫu IMU trong các cửa sổ 10 s khi bay (body FLU):
     - gia tốc: x 1,4–4,7; y 4,7–10; z 2,2–5,3 m/s²;
     - con quay: 10–17 / 7–11 / 6–19 °/s.
   - Phần lớn năng lượng nằm trên 25 Hz. Log 221 Hz có các đỉnh khoảng 20 / 41 / 101 Hz. Rất có thể đây là tần số motor/cánh quạt đã bị **chồng phổ (alias)**, chưa phải tần số thật.
   - PX4 `vehicle_imu_status`:
     - `accel_vibration_metric`: trung vị 12,1 / 14,0, p95 18,8 / 22,4 (IMU 0 / IMU 1);
     - `gyro_vibration_metric`: trung vị khoảng 0,55–0,61;
     - clipping = 0.
   - Tham số PX4: `IMU_GYRO_NF0_FRQ = 0`, `IMU_GYRO_DNF_EN = 0`, `IMU_GYRO_CUTOFF = IMU_ACCEL_CUTOFF = 30`, `IMU_INTEG_RATE = 200`.
   - ULog bắt đầu ở 57,4 s khi **đã ARM**, nên không có đoạn đứng yên trước khi ARM để đo nhiễu nền.
2. **Mô hình nhiễu sai mấy bậc độ lớn.**
   - Bộ lọc dùng `na` = 1,7e-3 m/s²/√Hz, `ng` = 2,4e-4 rad/s/√Hz.
   - Mật độ đo được trên luồng 50 Hz kiểu onboard là khoảng **0,90** và **0,028**.
   - Hệ quả: P quá nhỏ → gate từ chối range đúng → bộ lọc chạy hở bằng IMU rung → phân kỳ.
3. **Kết quả replay của người review** (range đã hiệu chuẩn có hỗ trợ K=0; giới hạn mỗi anchor 0,02 s trừ khi ghi khác; ba cửa sổ 75–195 / 198–218 / 235–300 s):

| Luồng IMU | Q | Tư thế PX4 | Khởi tạo lại | XY RMSE (cm) | Z p95 (cm) | \|dz\|<2σ | Yaw lệch PX4, p95 |
|---|---|---|---:|---|---|---:|---|
| 221 Hz, không giới hạn | mặc định | – | 0 | 50,2 / 25,9 / 36,8 | 33,2 / 13,3 / 20,3 | – | – |
| **pick50** (giống HIGHRES_IMU 50 Hz onboard) | mặc định | – | **18** | 221 / 177 / 256 | 250 / 130 / 188 | 0,32 | – |
| pick50 | đề xuất (0,90 / 0,028) | – | 0 | 11,4 / 16,4 / 23,2 | 13,1 / 12,8 / 12,0 | 0,98 | **trôi +46° / +94° / +132°** (trung vị) |
| pick50 | đề xuất | có | 0 | 11,0 / 16,3 / 23,1 | 13,0 / 12,9 / 11,9 | 0,985 | ≤ 1,7° |
| **mean50** (trung bình 20 ms) | đề xuất (0,29 / 0,011) | – | 0 | 10,8 / 16,4 / 22,9 | 12,0 / 12,7 / 11,5 | 0,95 | 14 / 1,4 / 5,6° |
| **mean50** | đề xuất | **có** | 0 | **10,8 / 16,3 / 22,9** | **12,0 / 12,7 / 11,6** | **0,95** | **≤ 0,7°** |
| *So sánh:* LS từng epoch (E2.0, range thô) | | | | 10,7 / 16,3 / 26,5 | 25,2 / 24,9 / 31,9 | | |

4. **Độ nhạy theo Q** (pick50):
   - Q/4: sai số tăng (Z p95 17–23 cm, \|dz\|<2σ chỉ 0,69).
   - Q/12: phân kỳ ở 235–300 s.
   - Q×3: độ chính xác như cũ, σ rộng hơn (0,99).
   - Kết luận: **đặt Q thấp hơn thực tế thì nguy hiểm, đặt cao hơn thì an toàn.**
5. **Sai số UWB tương quan theo thời gian.**
   - Tự tương quan của innovation còn 0,5–0,85 ở độ trễ 1 s, trên mọi anchor.
   - Cập nhật đủ tốc độ (khoảng 250 Hz mỗi anchor) làm σ quá nhỏ: \|dz\|<2σ chỉ 0,80.
   - Giới hạn 0,02 s mỗi anchor: 0,95–0,98, không mất độ chính xác, số lần cập nhật giảm 4–5 lần.
   - Với 0,01 s được 0,93; với 0,05 s được 0,99 (σ rộng hơn).
6. **Đã thử nhưng chưa nên đưa vào lúc này:**
   - Trạng thái bias Gauss–Markov cho từng anchor (23 trạng thái): không chính xác hơn, Z trung vị ở 235–300 s còn lệch thêm khoảng +9 cm.
   - Chỉ tăng `sig_r`: độ chính xác không đổi, chỉ σ thay đổi.
7. **Heading.**
   - Với luồng pick50, yaw của ESKF trôi tới hàng trăm độ. Vị trí vẫn ổn chỉ vì cánh tay đòn ngang của TAG chỉ 9 cm.
   - Lấy trung bình 20 ms gần như hết trôi.
   - Cập nhật tư thế từ PX4 (`ATTITUDE_QUATERNION` vốn đã được stream 50 Hz) đưa yaw về dưới 1°.
8. Node C++ hiện đã trừ lever arm khi khởi tạo (`p0 − R0·lever`). Ghi chú cũ của người review là sai. Tuy vậy, khi khởi tạo lại sau chuỗi từ chối, node vẫn dùng **R0 lúc khởi động** (đã cũ).

---

## 2. Phạm vi file

**Mới:**
- `eskf_tuning.py` (đã có khung);
- `tools/imu_noise_report.py`;
- `tools/eskf_tuning_report.py`;
- `E21_CHANGE_DESCRIPTION.md`;
- thêm mục E2.1 trong `README.md`, **chỉ phần E2.1**. `README.md` đang có thay đổi chưa commit của việc khác, nên chỉ stage đúng hunk của mình.

**Sửa:**
- `eskf_replay.py` (mục 4.1);
- `tools/flight_log_io.py` (mục 4.3).

**Không sửa:**
- `tools/eskf_a_replay.py`, `tools/aided_calibration_report.py`, `aided_calibration.py`: các lệnh E2.0 phải cho ra kết quả như cũ;
- toàn bộ `UP4000/`;
- `eskf_config.py`, `onboard_eskf_settings.json`, `last_rig_config.json`;
- mọi thay đổi chưa commit khác trong thư mục làm việc.

Codex có thể thêm test riêng cho công cụ, ví dụ `tests/test_offline_e21_tools.py`.

---

## 3. E2.1-A — Đo nhiễu IMU khi bay

### 3.1 Định nghĩa (đã ghi trong docstring `eskf_tuning.py`)

- **Độ lệch chuẩn từng mẫu** = std của (tín hiệu − trung bình trượt có tâm, cửa sổ `detrend_s` = 0,2 s), tính riêng từng đoạn liên tục rồi nhân √(n/(n−1)). Không nối qua khe hở dữ liệu hay qua mép khoảng thời gian.
- **Mật độ** = std từng mẫu của trục xấu nhất / √`stream_hz`.
  - Lý do: ESKF cộng Q_v = na²·dt mỗi bước. Nhiễu trắng σ trên mỗi mẫu ở tần số f tương đương na = σ/√f.
  - Vì vậy **phải đo trên đúng luồng mà bộ lọc nhận**. Cùng một IMU, lấy 1 mẫu trên 4 (pick50) thì `na` tăng khoảng 2 lần; lấy trung bình (mean50) thì std giảm khoảng 3 lần.
- `decimate_stream(..., "pick")` mô phỏng HIGHRES_IMU 50 Hz hiện tại, tức lấy mẫu đầu tiên của mỗi khung. `"mean"` mô phỏng một bridge lấy trung bình.
- `recommend_params`:
  - `na = max(datasheet, margin·acc_density)`, tương tự cho `ng`;
  - đặt `min_anchor_interval_s = 0.02` và `reinit_keep_attitude = True`;
  - các trường khác giữ nguyên.

### 3.2 Công cụ `tools/imu_noise_report.py`

```powershell
cd "Software\New_GUI"
python tools/imu_noise_report.py Logs/10112026/Test_1 --windows 75:195 --out Logs/10112026/Test_1/analysis/e21_imu_noise_native
python tools/imu_noise_report.py Logs/10112026/Test_1 --windows 75:195 --stream-hz 50 --decimate pick --out Logs/10112026/Test_1/analysis/e21_imu_noise_pick50
python tools/imu_noise_report.py Logs/10112026/Test_1 --windows 75:195 --stream-hz 50 --decimate mean --out Logs/10112026/Test_1/analysis/e21_imu_noise_mean50
```

Đầu ra trong thư mục `--out` (bắt buộc nằm dưới `analysis/e21_*`):
- **`imu_noise.json`**, gồm:
  - `ImuNoise.to_dict()` cho các cửa sổ đã chọn;
  - bảng theo từng cửa sổ 10 s trên toàn bộ log (std từng trục, độ cao rangefinder nếu có);
  - khi có `--stream-hz`: `ImuNoise` của luồng gốc để so sánh, và cảnh báo **`STREAM_ALIASING`** nếu `max(acc_trend_std)` của luồng đã giảm mẫu lớn hơn 2 lần luồng gốc. Test_1: pick50 khoảng 2,4 so với 0,39 m/s² ở trục y nên có cảnh báo; mean50 thì không;
  - mục `px4`:
    - `vehicle_imu_status`: trung vị/p95 của `accel_vibration_metric` và `gyro_vibration_metric` theo từng instance trong các cửa sổ, tổng clipping;
    - tham số `IMU_GYRO_NF0_FRQ`, `IMU_GYRO_NF0_BW`, `IMU_GYRO_CUTOFF`, `IMU_ACCEL_CUTOFF`, `IMU_GYRO_DNF_EN`, `IMU_INTEG_RATE`, `IMU_GYRO_RATEMAX`, `SDLOG_PROFILE`, `SDLOG_MODE`;
    - thời điểm log bắt đầu so với lúc ARM.

    Chỉ ghi số, **không tự đặt ngưỡng đánh giá** cho chỉ số của PX4.
- **`recommended_params.json`**: `EskfParams` dạng dict, kèm nguồn gốc (flight, windows, stream_hz, decimate, margin, tool version).
- **`psd.png`**: phổ từng trục của luồng gốc và luồng đã giảm mẫu, trục log.
- **`windows.csv`**.

Nghiệm thu, giá trị tham chiếu ±15% (khoảng cho phép vì cách khử xu hướng và chia đoạn có thể khác chút):

| Luồng | `na` (m/s²/√Hz) | `ng` (rad/s/√Hz) | Cảnh báo bắt buộc |
|---|---:|---:|---|
| native (~221 Hz) | 0,462 | 0,0151 | `VIBRATION_HIGH` |
| pick50 | 0,896 | 0,0279 | `VIBRATION_HIGH`, `STREAM_ALIASING` |
| mean50 | 0,293 | 0,0107 | `VIBRATION_HIGH`, không có `STREAM_ALIASING` |

Mục `px4` phải báo trung vị `accel_vibration_metric` khoảng 12,1 (IMU 0) và 14,0 (IMU 1), clipping = 0, và log bắt đầu khi đã ARM.

---

## 4. E2.1-B — Mở rộng replay

### 4.1 `eskf_replay.py` (giữ đúng hành vi E2.0 khi dùng giá trị mặc định)

1. `REPLAY_VERSION = "eskf-a-replay-v2"`.
2. Thêm vào `EskfParams` (giá trị mặc định giữ nguyên E2.0 / `filter.hpp`):

   ```python
   min_anchor_interval_s: float = 0.0          # 0 = dùng mọi range
   reinit_keep_attitude: bool = False
   att_sig_deg: tuple[float, float, float] = (2.0, 2.0, 5.0)
   att_gate: float = 25.0
   att_min_interval_s: float = 0.02
   ```

3. Thêm `@dataclass(frozen=True) class AttitudeSample: t: float; R: Any`. `R` là ma trận 3×3, body FLU → rig, lấy từ flight controller.
4. Thêm `log_so3(R)`, nghịch đảo của `so3_exp`, đúng tới π.
5. Thêm vào cuối `ReplayResult`, đều có giá trị mặc định:
   - `skipped_ranges: int = 0`;
   - `att_update_t`, `att_accepted`, `att_nis`: mảng, mặc định rỗng.
6. Chữ ký mới: `run_eskf_replay(anchors, imu, ranges, lever_flu, yaw0_rad, params=EskfParams(), attitude=())`.
7. **Giới hạn tốc độ mỗi anchor:**
   - Sau khi sắp xếp sự kiện, với từng anchor, bỏ range nếu `t − t_range_giữ_lại_gần_nhất < min_anchor_interval_s − 1e-9`.
   - Áp cho mọi range hợp lệ, kể cả trước khi khởi tạo.
   - Range bị bỏ được đếm vào `skipped_ranges`, không xuất hiện trong `update_*`.
   - Giá trị `0` thì kết quả giống E2.0 từng bit.
   - `min_anchor_interval_s` âm hoặc không hữu hạn → `ValueError`.
8. **Giữ tư thế khi khởi tạo lại:**
   - Khi bỏ bộ lọc sau chuỗi từ chối và `reinit_keep_attitude=True`, giữ `(R, bg)` của bộ lọc vừa bỏ.
   - Tại mỗi IMU cho tới lần khởi tạo kế tiếp: `R ← R·Exp((gyro − bg)·dt)`.
   - Khởi tạo bằng `p = tag0 − R·lever`, với `R` và `bg` vừa giữ.
   - Khi `False` thì dùng R0/bg0 lúc khởi động, như E2.0.
9. **Cập nhật tư thế (tùy chọn):**
   - Sự kiện loại 2; khi trùng thời điểm thì xếp sau IMU và range.
   - Bỏ qua khi bộ lọc chưa khởi tạo.
   - Tối đa một lần cập nhật mỗi `att_min_interval_s`, tính từ lần **đưa vào** gần nhất.
   - Phần dư `r = log_so3(R̂ᵀ·R_đo)`: vector quay trong body, khớp với phép tiêm `R ← R·Exp(δθ)`.
   - `H = [0 0 I 0 0]`, `Rn = diag(radians(att_sig_deg)²)`, `NIS = rᵀS⁻¹r`.
   - `NIS > att_gate` → từ chối, không đổi trạng thái và P.
   - Ngược lại: cập nhật Joseph, tiêm đủ 15 trạng thái giống update range, rồi ghi `(t, accepted, nis)`.
   - Mẫu không hợp lệ (không hữu hạn, không trực chuẩn với dung sai 1e-6, det ≤ 0) thì bỏ âm thầm.
   - Không tính vào chuỗi từ chối range.
10. 8 test E2.0 phải vẫn xanh, không sửa test.

### 4.2 `eskf_tuning.py`: chẩn đoán

Xem docstring: `normalized_errors` (độ nhất quán σ so với sai số) và `innovation_autocorrelation`.

### 4.3 `tools/flight_log_io.py`

- Thêm hàm trả về tư thế PX4 trong khung rig: `R_rig_flu = Rz(rig_yaw_offset − π/2) · T_ned→enu · R_ned_frd · diag(1, −1, −1)`.
  - Kiểm: yaw của `R_rig_flu` phải khớp `yaw_rig` hiện có trong 0,2°. Người review đo được 0,10°.
- Đọc lười `vehicle_imu_status` và các tham số ở mục 3.2.
- `output_directory`: cho phép `analysis/e20_*` **và** `analysis/e21_*`, vẫn từ chối các đường dẫn khác.

### 4.4 Công cụ `tools/eskf_tuning_report.py`

```powershell
python tools/eskf_tuning_report.py Logs/10112026/Test_1 --lever -0.09,0,0.31 --train 75:195 --test 198:218,235:300 --imu-stream-hz 50 --imu-decimate mean --params-json Logs/10112026/Test_1/analysis/e21_imu_noise_mean50/recommended_params.json --out Logs/10112026/Test_1/analysis/e21_eskf_mean50
python tools/eskf_tuning_report.py Logs/10112026/Test_1 --lever -0.09,0,0.31 --train 75:195 --test 198:218,235:300 --imu-stream-hz 50 --imu-decimate pick --params-json Logs/10112026/Test_1/analysis/e21_imu_noise_pick50/recommended_params.json --out Logs/10112026/Test_1/analysis/e21_eskf_pick50
```

- Dùng range đã hiệu chuẩn có hỗ trợ K=0, học trên `--train`, giống E2.0. Tái dùng hàm của `eskf_a_replay.py` bằng import, không sửa file đó.
- `--params-json` có khóa lạ hoặc thiếu → báo lỗi, thoát mã 2.
- Chạy ba biến thể trên cùng luồng IMU:
  - `default`: `EskfParams()`;
  - `tuned`: tham số từ JSON;
  - `tuned_att`: tham số từ JSON + tư thế PX4 50 Hz.
- Kèm các đường so sánh của E2.0: `gui_filtered`, `logged_raw_irls`, `epoch_huber`.
- `report.json`, `comparison.md/.csv`, `trajectories.npz`, cho mỗi biến thể và mỗi cửa sổ:
  - XY RMSE/p95, Z trung vị/p95 (tính như E2.0);
  - `normalized_errors` của TAG z so với `z_ref` (RF tươi ≤ 0,5 s), cho từng cửa sổ và gộp 75–300 s. σ lấy từ `p_std[:, 2]` (std vị trí body; bỏ qua phần do tư thế, vì cánh tay đòn chỉ khoảng 0,3 m);
  - tự tương quan innovation theo anchor ở 0,1 / 0,5 / 1,0 s;
  - yaw lệch PX4 (trung vị, p95) và độ nghiêng lệch PX4 (p95);
  - số lần khởi tạo lại, tỉ lệ nhận range và tư thế, `skipped_ranges`, thời gian chạy.
- Hình:
  - `z_sigma.png`: z của ESKF với dải ±2σ, chồng `z_ref`;
  - `yaw_vs_px4.png`.
- Ghi rõ trong báo cáo:
  - tham số chỉ học từ 75–195 s;
  - `min_anchor_interval_s` và σ tư thế là giá trị thiết kế cố định, không chỉnh trên đoạn kiểm;
  - tham chiếu không phải ground truth.

---

## 5. Nghiệm thu trên Test_1

Mục 5.1–5.3 áp cho cả hai lần chạy ở mục 4.4 (mean50 và pick50). Số trong ngoặc là kết quả tham chiếu của người review, mean50 trước, pick50 sau.

### 5.1 Biến thể `tuned_att`

- Không khởi tạo lại lần nào. Tỉ lệ nhận range ≥ 0,99 và nhận tư thế ≥ 0,99 (1,0 / 1,0).
- XY RMSE ≤ **12,5 / 18,5 / 25,5 cm** (10,8 / 16,3 / 22,9 · 11,0 / 16,3 / 23,1).
- XY p95 ≤ **24 / 34 / 34 cm** (20,2 / 29,9 / 29,0 · 20,0 / 28,7 / 29,9).
- \|Z trung vị\| ≤ **3 cm**, Z p95 ≤ **15 cm** ở mọi cửa sổ (12,0 / 12,7 / 11,6 · 13,0 / 12,9 / 11,9).
- Gộp 75–300 s: \|dz\|<2σ ≥ **0,90** và `rms_normalized` trong **[0,6; 1,4]** (0,954 / 0,95 · 0,985 / 0,77).
- Yaw lệch PX4 p95 ≤ **3°**, nghiêng lệch PX4 p95 ≤ **2°** (≤ 0,7° / ≤ 0,6° · ≤ 1,7° / ≤ 1,8°).

### 5.2 Biến thể `tuned`

- Cùng tiêu chí XY, Z và độ nhất quán như 5.1.
- Báo cáo **phải nêu** độ trôi yaw. Kỳ vọng với pick50: trung vị trong 235–300 s > 30° (+132°). Đây là phát hiện để đưa sang E2.2, không phải tiêu chí đạt/không đạt của code.

### 5.3 Biến thể `default` (kiểm tra hợp lý)

Phải tái hiện được lỗi:
- pick50: ≥ 5 lần khởi tạo lại, hoặc XY RMSE đoạn học > 50 cm (18 lần; 221 cm);
- mean50: ≥ 1 lần khởi tạo lại, hoặc XY RMSE 235–300 s > 50 cm (1 lần; 154 cm).

### 5.4 Thời gian và cách làm

- Mỗi replay mất < 60 s trên PC (tham chiếu khoảng 12–15 s).
- Không chỉnh tham số theo đoạn kiểm. Nếu không đạt một tiêu chí, **báo lại kèm số liệu, không nới ngưỡng**.

---

## 6. Ngoài phạm vi: E2.2 onboard (ghi lại để làm sau, cần người dùng commit phần onboard đang dở trước)

1. Node C++:
   - khai báo ROS param cho `na`, `ng`, `nba`, `nbg`, `min_anchor_interval_s`, `reinit_keep_attitude`, cùng σ, gate và khoảng thời gian của cập nhật tư thế. Hiện chỉ `sig_r` và `q_cv` là param, còn `na`/`ng` bị viết cứng;
   - JSON trạng thái in ra các giá trị này;
   - `eskf_config.py` thêm khoảng hợp lệ.
2. **Luồng IMU:**
   - pick50 từ HIGHRES_IMU gây chồng phổ và trôi yaw.
   - Hướng sửa: xin HIGHRES_IMU ≥ 200 Hz rồi lấy trung bình về 50 Hz trong bridge, hoặc predict ở tốc độ đầy đủ.
   - Kiểm lại theo tài liệu PX4 xem HIGHRES_IMU lấy từ `vehicle_imu` (tích phân thô) hay đã qua lọc. Nếu là tích phân thô thì notch filter của PX4 **không** làm sạch luồng này.
3. Cập nhật tư thế từ `ATTITUDE_QUATERNION` (đã có 50 Hz trong `DEFAULT_RATES`), thêm param `rig_yaw_offset_deg`.
4. **Khởi tạo tư thế khi đứng yên:**
   - Node đòi \|‖a‖ − 9,81\| ≤ 0,8 và ‖ω‖ ≤ 0,15 suốt 1 s.
   - Khi motor đang quay (Test_1 lúc ARM dưới đất: std 1,4–4,6 m/s²) thì không bao giờ đạt.
   - Sau `IMU_GAP` giữa chuyến bay, `reset_attitude` đòi đứng yên lại nên không bao giờ hồi phục.
   - Hướng sửa: khởi tạo / khởi tạo lại tư thế từ PX4.
5. Khởi tạo lại bằng R0 cũ: sửa bằng logic giữ tư thế ở mục 4.1.
6. Mở rộng bộ kiểm parity C++/Python (`logs/e20_validation/`) cho các update mới.

---

## 7. Việc phần cứng cho người dùng (song song, không phải việc của Codex)

1. **Giảm rung cơ khí:**
   - cân bằng cánh và motor, siết ốc, thay cánh cong/mẻ;
   - kiểm đệm giảm chấn của Pixhawk. Trục y (ngang) xấu nhất, nên xem cách gắn.
   - So trước/sau bằng `imu_noise_report.py` trên cùng kiểu bay hover. Ngưỡng `VIBRATION_HIGH` = 1 m/s² chỉ là ngưỡng kinh nghiệm.
2. **Notch / dynamic notch cho vòng điều khiển** (`IMU_GYRO_DNF_EN`, ESC RPM hoặc FFT; tra tài liệu PX4 trước khi bật). Việc này giúp bộ điều khiển. Với ESKF, xem mục 6.2.
3. **Một log hover có IMU tốc độ cao** (profile ghi IMU tốc độ cao trong `SDLOG_PROFILE`; tra tài liệu PX4 để chọn bit) để thấy tần số rung thật. Log 221 Hz hiện tại bị chồng phổ.
4. **Ghi log từ lúc bật nguồn** (`SDLOG_MODE`) và để drone đứng yên khoảng 10 s trước khi ARM, để có nhiễu nền của cảm biến.
5. Các việc cũ vẫn còn: thử xoay yaw dưới đất, Test_2 đổi yaw theo bước, hiệu chuẩn ở 2 độ cao, đo lại anchor tầng trên.

---

## 8. Quy tắc

1. Không sửa:
   - `tests/test_e21_*.py`, `tests/e21_fixtures.py`;
   - `tests/test_e20_*.py`, `tests/e20_fixtures.py`, `tests/test_offline_flight_log_io.py`.

   Thấy test sai thì dừng lại và báo lý do.
2. Thư mục làm việc đang có **thay đổi chưa commit của việc khác** (MCAP/onboard, `eskf_config.py`, `onboard_eskf_settings.json`, `last_rig_config.json`, `README.md`…). **Không commit, không sửa** các file đó, trừ hunk E2.1 trong `README.md`. Chỉ commit file của E2.1, gồm cả các file hợp đồng ở trên, không sửa gì.
3. `eskf_tuning.py` và `eskf_replay.py` chỉ dùng numpy và thư viện chuẩn. Công cụ import lười `pyulog`, `mcap`, `matplotlib`. **Không thêm vào `requirements.txt`.**
4. Không ghi đè dữ liệu gốc hay `analysis/e20_*`. Đầu ra mới chỉ ghi vào `analysis/e21_*`.
5. Chạy test:

   ```powershell
   cd "Software\New_GUI"
   $env:QT_QPA_PLATFORM = "offscreen"
   python -m unittest discover -s tests -p "test_*.py"
   ```

   Hiện tại (tính cả test của các thay đổi chưa commit trong thư mục làm việc): 191 test xanh và 14 test E2.1 đỏ. Kỳ vọng: **205/205** (cộng thêm test riêng của Codex nếu có). Riêng test E2.1 mất khoảng 45 s.
6. Commit gợi ý:
   - `feat(e2.1-a): in-flight IMU noise estimation and parameter recommendation`
   - `feat(e2.1-b): replay rate limit, attitude carry-over and attitude aiding`
   - `feat(e2.1): imu noise and eskf tuning report tools`
   - `docs(e2.1): …`

---

## 9. Claude sẽ review

1. `git diff` các file hợp đồng phải rỗng; 205/205 test xanh.
2. So với bản tham chiếu riêng của người review: cùng dữ liệu giả lập thì `eskf_replay` phải khớp tới sai số số học. `estimate_imu_noise` khớp trong vài phần trăm.
3. Đối chiếu từng dòng update tư thế: dấu của phần dư, khung body, Joseph, phép tiêm. Đối chiếu giới hạn tốc độ: tất định, không phụ thuộc thứ tự đầu vào.
4. Chạy lại cả năm lệnh ở mục 3.2 và 4.4 trên Test_1, kiểm bảng nghiệm thu mục 3.2 và mục 5.
5. Kiểm lệnh E2.0 cho ra kết quả như cũ, và đầu ra mới chỉ nằm trong `analysis/e21_*`.

---

## 10. Lời nhắn mẫu để giao cho Codex

> Đọc `Plan/E2_1_YEU_CAU_TINH_CHINH_ESKF_THEO_RUNG_KHI_BAY.md`. Trong `Software/New_GUI`: làm `eskf_tuning.py` theo docstring và mở rộng `eskf_replay.py` theo mục 4.1. Hợp đồng là 14 test `tests/test_e21_*.py` đang đỏ; không sửa chúng, `tests/e21_fixtures.py` và các hợp đồng E2.0. Sau đó mở rộng `tools/flight_log_io.py` (mục 4.3), viết `tools/imu_noise_report.py` và `tools/eskf_tuning_report.py`, chạy năm lệnh ở mục 3.2 và 4.4 trên `Logs/10112026/Test_1`, ghi kết quả vào `analysis/e21_*`, đối chiếu bảng nghiệm thu mục 3.2 và mục 5, rồi viết `E21_CHANGE_DESCRIPTION.md`. Không đụng `UP4000/`, `eskf_config.py`, `onboard_eskf_settings.json`, `last_rig_config.json`; không commit hay sửa các thay đổi chưa commit của việc khác. Kỳ vọng 205/205 test xanh.
