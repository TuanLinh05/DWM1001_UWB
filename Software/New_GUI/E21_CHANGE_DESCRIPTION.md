# E2.1 — Đo rung IMU và tinh chỉnh replay ESKF-A offline

Ngày: 10/11/2026. Yêu cầu: `Plan/E2_1_YEU_CAU_TINH_CHINH_ESKF_THEO_RUNG_KHI_BAY.md`.

## Kết quả và điểm cần Claude review

Đã triển khai E2.1-A/B/C và chạy đủ năm lệnh trên Test_1. Các biến thể `tuned`
và `tuned_att` của cả mean50 và pick50 đạt các ngưỡng vị trí/độ nhất quán được
yêu cầu. `tuned_att` không khởi tạo lại, nhận 99,9988% range và 100% tư thế.
Pick50 đạt toàn bộ 41 phép kiểm nghiệm thu tự động trong báo cáo.

**Mean50 còn một phép kiểm không đạt: mục 5.3 về tái hiện lỗi của `default`.**
Theo mục 4.4, biến thể này dùng đúng `EskfParams()`, tức khoảng range tối thiểu
bằng 0. Kết quả: 0 lần khởi tạo lại; XY RMSE đoạn 235–300 s là 34,13 cm,
chưa vượt 50 cm. Vì vậy `e21_eskf_mean50/report.json` ghi `all_passed=false`;
không đổi tham số mặc định, không nới ngưỡng, không giấu phép kiểm này.

Phần bằng chứng tham chiếu ở mục 1 lại nói áp khoảng 0,02 s mỗi anchor trừ khi
ghi khác. Kiểm tra riêng với đúng khoảng cố định đó, vẫn dùng Q mặc định và
`reinit_keep_attitude=false`, cho **1 lần khởi tạo lại, XY RMSE đoạn cuối
157,91 cm**. Kết quả gần tham chiếu 1 lần / 154 cm. Đây là bằng chứng cho khác
biệt về cấu hình baseline trong hai phần của yêu cầu. Nó không thay thế
biến thể `default` bắt buộc và không được dùng để chọn tham số.

Dù chưa phân kỳ theo ngưỡng mục 5.3, default mean50 vẫn báo σ quá nhỏ:
chỉ 27,83% sai số Z nằm trong ±2σ, RMS chuẩn hóa 7,93; yaw p95 ở ba cửa sổ
khoảng 169–175°. Vì vậy không thể coi baseline này là bộ lọc nhất quán.

## Phạm vi thay đổi

- `eskf_tuning.py`: đo nhiễu, giảm mẫu pick/mean, đề xuất tham số, kiểm tra σ
  và tự tương quan innovation. Chỉ numpy/thư viện chuẩn, không IO.
- `eskf_replay.py`: phiên bản v2; giới hạn range mỗi anchor, giữ R/bg và tích
  phân tư thế khi chờ khởi tạo lại, cập nhật tư thế tùy chọn. Giá trị mặc định
  giữ nguyên các mảng kết quả số của E2.0.
- `tools/flight_log_io.py`: thêm phép đổi tư thế PX4 sang rig, đọc lười chỉ số
  rung/tham số/ARM, cho phép đầu ra `analysis/e21_*` bên cạnh `e20_*`.
- `tools/imu_noise_report.py`, `tools/eskf_tuning_report.py`: hai CLI mới.
- `tests/test_offline_e21_tools.py`: 13 test bổ sung về biên IO, file tham số,
  thứ tự sự kiện, SO(3), covariance và lỗi thiếu dependency.
- README: chỉ thêm mục E2.1. Các thay đổi khác có sẵn trong README được giữ lại
  và không đưa vào commit E2.1.

Các test/fixture hợp đồng E2.0/E2.1 và `test_offline_flight_log_io.py` giữ nguyên.
Không sửa `aided_calibration.py`, hai CLI E2.0, `requirements.txt`, GUI,
`eskf_config.py`, cấu hình rig/onboard hay nguồn C++/PX4/firmware. Không triển
khai E2.2 và không kết nối COM/SSH/UP7000.

## Các quy ước số cần kiểm

Độ lệch chuẩn là std của phần dư sau trung bình trượt có tâm, cửa sổ lẻ gần
0,2 s, nhân √(n/(n−1)). Chỉ dùng miền convolution hợp lệ, bỏ mép mỗi đoạn;
tách đoạn tại mép khoảng được chọn hoặc khe hở >5 median dt. Không padding,
không nối qua khoảng trống. Mật độ = std trục lớn nhất / √(1/median dt).
Tín hiệu chậm bị loại vào trend; trend của pick50 giúp nhận ra alias tần số thấp.

Giảm mẫu chia bin `floor((t-t0)*hz)`, sau khi lọc giá trị không hữu hạn và sắp
xếp tất định. Pick giữ nguyên mẫu đầu mỗi bin, mean lấy trung bình t/acc/gyro.
Không giả sử luồng native hay pick50 có dt đều. Các đỉnh PSD chỉ là tần số
biểu kiến trong log đã lấy mẫu, không chứng minh được tần số motor thật.

Giới hạn range áp trên mọi range hợp lệ đã sắp xếp, kể cả trước khởi tạo;
so với lần giữ lại gần nhất cùng anchor, dung sai 1 ns. Range bị bỏ không vào
`update_*`, có đếm `skipped_ranges`. Giá trị 0 giữ mọi range như E2.0.

Khi bỏ bộ lọc và bật giữ tư thế, copy R/bg từ bộ lọc vừa bỏ, tiếp tục
`R = R @ Exp((gyro-bg)*dt)` ở mỗi IMU trong lúc chờ. Lần khởi tạo sau dùng
`body0 = tag0 - R @ lever`. Khi tắt tùy chọn vẫn dùng R0/bg0 lúc khởi động.

Update tư thế dùng `log_so3(R_est.T @ R_px4)` trong body và H có I tại cột
6:9; Rn là bình phương σ góc theo radian. Gate NIS trước khi thay đổi bất kỳ
trạng thái/P nào. Dùng Joseph và tiêm đủ p/v/R/ba/bg, phép tiêm R bên phải.
Không thêm reset Jacobian ngoài convention hiện có của filter E2.0.
Khoảng tư thế tính từ lần đưa vào gần nhất, cả khi bị từ chối. Tư thế không
ảnh hưởng chuỗi từ chối range. Sự kiện đồng thời: IMU, range, rồi tư thế.
Ma trận NaN, sai trực chuẩn hoặc reflection bị bỏ. `log_so3` ổn định tại 0 và π;
ở đúng π, dấu trục là mơ hồ về mặt toán học và được chọn tất định.

Đổi khung:

```text
R_rig_flu = Rz(rig_yaw_offset - pi/2) @ T_ned_to_enu @ R_ned_frd @ diag(1,-1,-1)
```

Đối chiếu yaw với `yaw_rig` trên toàn CSV: sai khác tối đa 5,09e-14 độ.
Tư thế update được SLERP từ quaternion MAVLink/ULog sang grid 50 Hz.
Sai số yaw báo cáo là yaw của `R_px4.T @ R_est`, trung vị có dấu/p95 trị tuyệt
đối; độ nghiêng là `acos(relative_R[2,2])`. Hình yaw unwrap chỉ để hiển thị,
không dùng các phép dịch nhánh 2π của hình để chấm điểm.

## Nhiễu đo trên đoạn học 75–195 s

| Luồng | Median Hz | na (m/s²/√Hz) | ng (rad/s/√Hz) | So với tham chiếu |
| --- | ---: | ---: | ---: | --- |
| Native | 221,631 | 0,461871 | 0,015065 | đạt ±15% |
| Pick50 | 51,156 | 0,896575 | 0,027901 | đạt ±15% |
| Mean50 | 49,878 | 0,293222 | 0,010743 | đạt ±15% |

Native có 19.664 phần dư; mỗi luồng 50 Hz có 5.990. Cả ba có
`VIBRATION_HIGH` và `IRREGULAR_RATE`. Pick50 thêm `STREAM_ALIASING`;
max std trend gia tốc = 2,399 m/s² so với native 0,390. Mean50 = 0,479,
không phát cảnh báo alias.

PX4 trên các khoảng học, không áp thêm ngưỡng phân loại:

| Instance | Accel vibration median / p95 | Gyro vibration median / p95 | Clipping tăng thêm |
| --- | --- | --- | ---: |
| 0 | 12,300 / 18,985 | 0,541 / 0,656 | 0 |
| 1 | 14,152 / 22,854 | 0,606 / 0,724 | 0 |

Các giá trị clipping counter lớn nhất cũng bằng 0. Tham số đọc từ ULog:
NF0_FRQ=0, NF0_BW=20, GYRO_CUTOFF=30, ACCEL_CUTOFF=30,
DNF_EN=0, INTEG_RATE=200, GYRO_RATEMAX=800, SDLOG_PROFILE=1, SDLOG_MODE=0.
Log bắt đầu 57,439777 s, bản ghi armed đầu tiên = true; armed_time ghi nhận
57,431960 s, trước bắt đầu log khoảng 7,8 ms. Không có baseline đứng yên
trước ARM để suy ra nhiễu nền cảm biến.

## Kết quả vị trí của tuned và tuned_att

Các cửa sổ lần lượt là học 75–195 s, kiểm 198–218 s, kiểm 235–300 s.
Đơn vị cm; Z p95 là p95 |dz|. Chấm cùng grid gần đều 10 Hz bằng epoch CSV
thật như E2.0, với RF tươi ≤0,5 s. Số epoch: 1200 / 200 / 617.

| Luồng / biến thể | XY RMSE | XY p95 | Z median | Z p95 | Kết quả vị trí |
| --- | --- | --- | --- | --- | --- |
| Mean50 tuned | 10,76 / 16,40 / 22,93 | 19,94 / 29,26 / 29,12 | −0,40 / +0,87 / −0,09 | 12,00 / 12,69 / 11,55 | đạt |
| Mean50 tuned_att | 10,81 / 16,24 / 22,88 | 20,17 / 29,92 / 29,05 | −0,40 / +0,84 / −0,08 | 12,04 / 12,71 / 11,57 | đạt |
| Pick50 tuned | 11,36 / 16,41 / 23,14 | 21,58 / 28,16 / 29,96 | −0,29 / +0,98 / −0,02 | 13,09 / 12,83 / 12,02 | đạt |
| Pick50 tuned_att | 10,95 / 16,35 / 23,15 | 20,01 / 28,72 / 30,09 | −0,22 / +1,23 / +0,16 | 13,00 / 12,95 / 11,92 | đạt |

Hiệu chuẩn K=0 chỉ học trên 75–195 s, rồi cố định trong cả replay, kể cả
khởi tạo trước đoạn học. Các bias bổ sung giống E2.0. Không học từ đoạn kiểm.
Không chỉnh σ range, bias random walk hay ngưỡng gate trên holdout.
Khoảng range 0,02 s, σ tư thế (2°,2°,5°), gate 25 và khoảng tư thế 0,02 s
là lựa chọn thiết kế cố định trong yêu cầu.

## Độ nhất quán, heading và thời gian

Độ nhất quán gộp 75–300 s dùng 56.562 epoch CSV có RF tươi và đủ range;
không nối qua khe hở estimate >0,1 s. Dùng σ vị trí body theo yêu cầu,
bỏ phần covariance tư thế của lever arm khoảng 0,3 m.

| Luồng / biến thể | Tỷ lệ abs(dz)<2σ | RMS chuẩn hóa | Median σz (cm) | Khởi tạo lại | Range được nhận | Tư thế được nhận | Replay (s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Mean50 default | 0,2783 | 7,9299 | 1,331 | 0 | 99,9415% | — | 45,87 |
| Mean50 tuned | 0,9541 | 0,9473 | 6,189 | 0 | 99,9988% | — | 10,40 |
| Mean50 tuned_att | 0,9536 | 0,9485 | 6,185 | 0 | 99,9988% | 100% | 12,09 |
| Pick50 default | 0,1880 | 58,2003 | 1,877 | 15 | 81,0275% | — | 30,57 |
| Pick50 tuned | 0,9835 | 0,7728 | 8,334 | 0 | 99,9988% | — | 6,09 |
| Pick50 tuned_att | 0,9843 | 0,7699 | 8,317 | 0 | 99,9988% | 100% | 8,48 |

Mỗi replay đo trong các lần chạy chính thức đều <60 s. Không tính thời gian
đọc log/học K=0/tạo hình vào thời gian lõi replay. `tuned`/`tuned_att` bỏ
420.768 range vì giới hạn tốc độ; số đưa vào filter khoảng 86.510/86.514
(mean/pick), thay vì khoảng 505.900 ở default. Tự tương quan innovation tại
1 s của tuned_att vẫn khoảng 0,44–0,84 (mean) / 0,40–0,82 (pick), nên không
thể diễn giải các range liên tiếp là phép đo nhiễu độc lập hoàn toàn.

| Luồng / biến thể | Yaw median theo cửa sổ (độ) | Yaw p95 abs (độ) | Nghiêng p95 (độ) |
| --- | --- | --- | --- |
| Mean50 tuned | +2,28 / −0,78 / +1,09 | 13,85 / 1,35 / 5,63 | 1,87 / 1,54 / 1,64 |
| Mean50 tuned_att | −0,14 / −0,01 / −0,03 | 0,61 / 0,56 / 0,44 | 0,51 / 0,45 / 0,44 |
| Pick50 tuned | +45,51 / +93,62 / +132,31 | 87,77 / 98,83 / 151,50 | 10,54 / 8,21 / 12,83 |
| Pick50 tuned_att | −0,01 / −0,06 / +0,00 | 1,17 / 1,36 / 1,21 | 1,48 / 1,57 / 1,47 |

Pick50 tuned vẫn có heading không sử dụng được dù vị trí đạt. Đây là phát
hiện cần đưa sang E2.2; chưa triển khai bất kỳ thay đổi onboard nào.

## Kiểm thử và tái lập

Năm lệnh đầy đủ nằm trong mục E2.1 của README. Mỗi CLI trả mã 0 khi tạo báo
cáo thành công; mã 2 khi lỗi đầu vào/dependency. Mã 0 không chứng nhận mọi
tiêu chí vị trí đã đạt: xem `report.json` → `acceptance`, đặc biệt mean50.
Mỗi báo cáo có toàn bộ ngưỡng/giá trị/kết quả, tham số và provenance để review.

- Trước triển khai: 14 test E2.1 đều lỗi, lưu `logs/e21_validation/contracts_before.txt`.
- Sau triển khai lõi: 14/14 xanh trong 60,743 s; test/fixture không sửa.
- Test bổ sung: schema/provenance JSON nghiêm ngặt, từ chối đường dẫn E2.0,
  lỗi thiếu pyulog không ghi output, từ chối params trước khi đọc flight,
  Joseph/injection các trạng thái có cross-covariance, gate không đổi state/P,
  khoảng cập nhật tính từ lần đưa vào bị từ chối, bỏ tư thế invalid, thứ tự
  đầu vào không đổi kết quả, SO(3) tại π, đổi khung và detrend qua khe hở.
- E2.0 defaults: so bản module trước sửa với bản v2 trên cùng flight giả lập;
  **11 mảng output giống từng bit**, init_time/reinits giống nhau. Metadata
  phiên bản đổi thành v2 và thêm các trường mới đúng yêu cầu.

- **Full suite cuối: 218/218 xanh trong 165,325 s**, gồm 205 test hiện có
  và 13 test bổ sung; Qt offscreen, UDP loopback cục bộ, không kết nối drone.
  Lệnh: `python -m unittest discover -s tests -p "test_*.py"`.
- Chạy lại hai CLI E2.0, chỉ chuyển output vào `analysis/e21_e20_regression_a/b`
  để bảo vệ các kết quả E2.0: 4 mảng của `evaluation.npz`, 14 mảng của
  `trajectories.npz` đều giống từng bit (NaN cùng vị trí); JSON hiệu chuẩn
  giống nhau; bảng so sánh CSV/Markdown giống từng byte. Chỉ metadata
  ReplayResult/param mới và thời gian chạy khác theo phiên bản/máy.
- So SHA-256 trước/sau: **468 file được bảo vệ không thay đổi**, bao gồm các
  thay đổi chưa commit có sẵn, test hợp đồng, recording gốc và đầu ra E2.0.
  Bốn file có sẵn được phép sửa được loại trừ riêng: README, eskf_tuning,
  eskf_replay và flight_log_io. Không stage các file của công việc khác.
- Kiểm tra lại NPZ E2.1: 56 array key khác nhau cho mỗi file; độ dài các
  mảng sự kiện khớp nhau, số lần nhận tư thế mỗi cửa sổ khớp JSON. Tính lại
  normalized_errors từ các mảng đã lưu cho cả ba biến thể: khớp báo cáo.
  Test roundtrip ngăn tên `tuned + att_update_t` đè `tuned_att + update_t`.
  Namespace cuối là `variant__field`, ví dụ `tuned__att_update_t` và
  `tuned_att__update_t`; các track scoring giữ tên biến thể không hậu tố.

Bằng chứng chi tiết (không track log vào Git):

```text
logs/e21_validation/full_suite.txt
logs/e21_validation/e20_bitwise_parity.json
logs/e21_validation/e20_cli_regression.json
logs/e21_validation/noise_acceptance.json
logs/e21_validation/npz_roundtrip_audit.json
logs/e21_validation/trajectory_invariants.json
logs/e21_validation/preservation_audit.json
logs/e21_validation/mean50_run.txt
logs/e21_validation/pick50_run.txt
```

## Đầu ra và giới hạn diễn giải

Đầu ra mới đều trong `Logs/10112026/Test_1/analysis/e21_*`:

- `e21_imu_noise_native`, `e21_imu_noise_pick50`, `e21_imu_noise_mean50`:
  JSON nhiễu/tham số, bảng 10 s toàn log, CSV và PSD.
- `e21_eskf_mean50`, `e21_eskf_pick50`: report JSON, bảng MD/CSV, NPZ đầy
  đủ trạng thái/σ/update/tham chiếu, `z_sigma.png`, `yaw_vs_px4.png`.
- `e21_default_rate_diagnostic`: kiểm tra riêng baseline mean50 với khoảng
  range cố định 0,02 s. Script tái lập tại
  `logs/e21_validation/review_interval_check.py`.
- `e21_e20_regression_a`, `e21_e20_regression_b`: đầu ra chạy lại CLI E2.0,
  dùng để so với các thư mục E2.0 gốc, không ghi đè bản cũ.

NPZ/báo cáo/hình nằm trong Logs chưa track, không đưa log chuyến bay vào Git.
Schema JSON tham số bắt buộc đủ mọi trường EskfParams và provenance khớp
flight/khoảng học/stream/mode; lỗi unknown/missing/NaN/sai kiểu đều bị từ chối.
Core không import pyulog/mcap/scipy/matplotlib; công cụ import lười và báo lệnh
pip cụ thể khi thiếu. Không thêm dependency vào requirements.

RF là cùng nguồn đã dùng để học hiệu chuẩn, không phải ground truth độc lập;
GNSS được căn cứng trên đoạn học, không phải tọa độ khảo sát. PX4 attitude
và IMU có tương quan. Replay vẫn giả định cửa sổ IMU ban đầu đứng yên như
E2.0, dù log thực tế đã ARM. Timestamp trong một burst CSV được dùng cho mọi
anchor, chưa có timestamp từng slot. Không mô phỏng các guard thời gian thực
của node ROS. Những giới hạn này có trong báo cáo tự sinh và không được dùng
để kết luận độ chính xác bay tuyệt đối hay đã sẵn sàng triển khai onboard.
