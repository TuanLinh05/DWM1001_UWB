# E2.3 — Phiên onboard, bias hỗ trợ và triển khai shadow

Thực hiện theo `Plan/E2_3_YEU_CAU_CONG_CU_PHIEN_ONBOARD_VA_BAY_SHADOW.md`.
Người dùng yêu cầu thêm SSH và đồng bộ code sang UP7000 trong chat này;
chỉ dẫn đó ưu tiên hơn điều cấm SSH trong tài liệu. Đã đồng bộ nguồn và
build/test overlay riêng, chưa chạy `--apply` hoặc restart production.

## Các thay đổi

- `onboard_replay.py`: ánh xạ cấu hình như node E2.2, xử lý tem ns bằng phép
  trừ số nguyên, đổi IMU sang FLU một lần, lọc/đếm range và attitude bị bỏ,
  trừ bias đúng dấu, replay lõi cũ và so track có kiểm khe thời gian.
- `tools/onboard_session_io.py`: đọc nhiều file MCAP theo thứ tự số, giữ
  header/log time riêng, phục hồi phần đọc được của file cuối chưa đóng;
  decoder MCAP/ROS và pymavlink được import lười. Ưu tiên attitude có tem
  device; fallback odometry được ghi rõ là receive-stamped. Ghép IMU/device
  bằng log time một-một, giữ offset int64, báo độ trải và các epoch reset.
  Tlog tách reset theo từng loại message, chọn boot trùng thời gian thật.
  Trường dialect thiếu như `signal_quality` được giữ `None`.
- `tools/onboard_session_report.py`: kiểm kê CSV, status timeline, cấu hình,
  replay as-recorded/what-if, chỉ số train/holdout, nhiễu đúng luồng đã ghi,
  flow/rangefinder, JSON/Markdown/NPZ và hình. Chỉ ghi `analysis/e23_*`.
  Không chấm flight nếu thiếu mapping device clock hoặc clock có nhiều epoch.
  Aided artifact có provenance phải khớp lever và cửa sổ học đã chỉ định.
- `eskf_config.py`: `aided_bias_m` tùy chọn, id duy nhất trong rig,
  |bias| ≤0,5 m; chỉ có trong cấu hình chuẩn hóa khi không rỗng. Cấu hình cũ
  giữ nguyên hash. Chỉ ghi bias cho anchor bật; node trừ đúng một lần.
- GUI: **Nhập hiệu chuẩn hỗ trợ…**, hiển thị mm từng anchor, **Xóa bias hỗ trợ**.
  Chỉ nhận artifact E2.0 thành công, K=0; từ chối thành phần theo hướng mũi
  thay vì âm thầm bỏ chúng. Không tự bật bias hay sửa cấu hình bay hiện có.
  Khi bắt đầu log WiFi/onboard, lever zero/chưa xác nhận được cảnh báo bằng
  nhãn, tooltip và `Log: WARN`, không mở hộp thoại chặn. Cảnh báo lưu vào
  `session_metadata.json` và `onboard_recording.json`.
- `smoke_test_eskf.py --record-session`: native rosbag2 MCAP với TOPICS của
  production, metadata/config snapshot và device-time giả lập. Chờ recorder
  subscribe trước khi phát; không ghi đè phiên cũ. Lỗi khởi chạy recorder
  vẫn dừng child estimator và đánh dấu phiên chưa đóng.
- `deploy_onboard_eskf.sh`, `onboard_deploy.py`, `rollback.sh`: mặc định chỉ
  in kế hoạch, không ghi file. `--build-only` dùng overlay mới. Nguồn lấy bằng
  `git -c core.autocrlf=false archive`; archive chuyển từ PC phải khớp SHA-256.
  Chặn CR ở source/CSV/JSON và chặn đường dẫn/link không hợp lệ trước giải nén.
  Build/test dùng cùng `--base-paths`, có `UP7000/scripts/reset_tag_usb.py`
  và `PYTHONPATH=$HOME/drone_tools/python`. Kiểm kết quả cả hai package,
  không chấp nhận colcon chỉ chạy 0 test.
- `--apply` sao lưu đủ binary, logger prefix, schema helper và runner vào
  `~/e23_backups/backup_<UTC>_<workspace>/`, kèm checksum. Runner chỉ được
  thêm `/px4/attitude_frd` trong TOPICS, giữ nguyên byte phần guard/path máy.
  Logger giữ cấu hình cổng, chỉ cập nhật request `HIGHRES_IMU:200` và
  `ATTITUDE_QUATERNION:50`. Đây là request MAVLink stream, không đổi tham số PX4.
  Cài file bằng atomic replace; không ghi đè inode executable đang chạy.
  Rollback kiểm đủ checksum trước khi sửa; khôi phục nội dung gói logger
  materialized và ba file còn lại. Symlink leaf được phục hồi nếu target cũ
  còn khớp backup, nếu không dùng chính file backup để bảo đảm đúng mã cũ.
  Cài đặt lỗi sẽ thử rollback; chỉ restart khi có cờ `--restart`.
- Sửa hướng dẫn colcon/PYTHONPATH/archive trong E22/README; parser CSV C++
  bỏ CR cuối cell để fixture CRLF không làm `attitude_init` sai.

Không đổi `eskf_replay.py`, `estimator.hpp`, `filter.hpp`, node C++, các hợp
đồng, requirements hoặc dữ liệu gốc. `UP7000/.gitattributes` vẫn chưa track,
đúng phần quyết định được tài liệu dành cho người dùng.

## Nghiệm thu Test_1

Lệnh đã chạy từ root repo (cũng dùng được từ `Software/New_GUI`):

```powershell
python Software/New_GUI/tools/onboard_session_report.py Logs/10112026/Test_1/onboard/eskf/20261110_101854_5500 --tlog Logs/10112026/Test_1/onboard/20261110_100743/px4.tlog --flight Logs/10112026/Test_1 --train 75:195 --test 198:218,235:300 --what-if-confirm --lever -0.09,0,0.31 --yaw0-from-flight --aided-bias-json Logs/10112026/Test_1/analysis/e20_aided/aided_calibration.json --out Logs/10112026/Test_1/analysis/e23_onboard_session
```

Toàn bộ ngưỡng trong kế hoạch đạt, không điều chỉnh theo đoạn kiểm.

| Chỉ số có bias hỗ trợ | 75–195 s | 198–218 s | 235–300 s |
|---|---:|---:|---:|
| XY RMSE (cm) | 11,117 | 15,197 | 22,441 |
| XY p95 (cm) | 20,476 | 25,864 | 28,263 |
| Z trung vị (cm) | +0,459 | +2,271 | +1,049 |
| Z p95 tuyệt đối (cm) | 11,712 | 12,461 | 12,122 |
| Z trung vị không bias (cm) | +14,888 | +16,543 | +14,205 |

What-if dùng lever (−0,09;0;0,31), yaw rig ở attitude hợp lệ đầu tiên từ phép
căn chỉ trên train, các tham số GUI E2.2 mặc định. Khởi tạo boot 45,799 s,
0 reinit; range/attitude acceptance đều 100%. Gộp 75–300 s: |dz|<2σ =
0,96618; RMS chuẩn hóa = 0,93381; yaw p95 = 1,661°. Không có aided bias,
TAG Z vẫn lệch khoảng +15 cm: đây là phát hiện trên bộ dữ liệu này.

Kiểm kê: 14.890 IMU, dt median 19,548 ms, 0 khe >0,2 s; 8 anchor có
12.522–12.936 mẫu, `/px4/odometry` 14.887 mẫu, clock offset spread = 0 ns.
100% status `WAIT_CONFIGURATION`; as-recorded bỏ đủ 102.403 range vì chưa
xác nhận và cho 0 output, đúng bag thật không có `/uwb_eskf/odom`.

Nhiễu đo trên stream đã ghi 51,156 Hz, không giả vờ đó là luồng 200 Hz:

| Đoạn | na | ng |
|---|---:|---:|
| Trước ARM, 46–57 s, tự dò từ ULog armed_time | 0,036145 | 0,001738 |
| Train, 75–195 s | 0,947523 | 0,031738 |

Tlog thấy 2 boot, chọn boot 2; flow 10,011 Hz, distance 10 Hz, quality
p10/median/p90 = 64/70/77. Tích phân flow khoảng 20 ms cho mỗi chu kỳ
100 ms. Velocity diagnostic trừ gyro tích phân, dùng dấu MAVLink, nhân
distance/dt, chuyển body FLU → rig. PX4 đã áp dụng `SENS_FLOW_ROT=4`, không
áp dụng lại mount rotation. Với aided replay, RMSE velocity rig X/Y khoảng
0,421/0,345 m/s, tương quan 0,332/0,220: chưa đủ để kết luận sẵn sàng fuse.
Giả thiết mặt đất phẳng, bỏ sensor lever/vertical velocity/gyro bias được
ghi rõ; không fit dấu hoặc delay để cải thiện kết quả.

Kết quả đầy đủ:
`Logs/10112026/Test_1/analysis/e23_onboard_session/{report.json,report.md,streams.csv,trajectories.npz,trajectories.png,flow_velocity.png}`.

## Tự kiểm trên PC

- Full New_GUI: **246/246** (232 yêu cầu +14 test riêng).
- E2.3 sau rà soát cuối: **20/20**; bao gồm offscreen import/clear/hash,
  import lỗi không thay editor, cảnh báo không chặn log và có manifest,
  multi-segment MCAP/file cuối bị cắt, clock reset, tlog boot selection,
  archive CR/traversal, dry-run không ghi và cài đặt/rollback bốn phần.
- Logger WSL: **34 passed**, 20 subtests passed.
- CTest: **1/1**, chứa **4/4 gtest**. Bản sao golden thật sự đổi sang CRLF
  cũng khớp cả C++ states/updates với chuẩn Python.
- WSL smoke có ghi bag: RMS body 1,068 cm, không reinit. Replay so với
  node: median **2,233 mm**, p95 **4,311 mm**, max 8,090 mm.
- Smoke cuối sau sửa cleanup: RMS body **1,068 cm**, RUNNING sau 0,069 s;
  phiên native đóng thành công. Lỗi spawn recorder giả lập: đúng 1 child,
  child đã dừng, `closed=false`, không để node chạy sót.
- Dry-run trên cây giả WSL: nội dung, danh sách file và mtime không đổi.
  Thử install/rollback trên cây giả kiểm cả giữ cổng logger, giữ UUID runner,
  cập nhật đúng hai rate request và khôi phục đủ nội dung; backup hỏng bị từ
  chối trước khi thay file.
- So SHA-256: **470 file được bảo vệ nguyên vẹn**, gồm dữ liệu gốc, các
  hợp đồng, lõi toán và thay đổi việc khác. README chỉ stage hunk E2.3.

Một lần full suite trong sandbox bị test UDP timeout, sau đó gặp native Qt
access violation khi callback test UDP còn sống. Full suite có quyền loopback
sau đó đạt 245/245, và lượt cuối đạt 246/246. Không sửa/skip test để né lỗi;
không tuyên bố đã sửa triệt để vấn đề native Qt sẵn có.

Bằng chứng ở `Software/New_GUI/logs/e23_validation/`: `full_suite_final.txt`,
`e23_tests_final.txt`, `logger_pytest.txt`, `ctest.txt`, `crlf_parity.json`,
`deploy_dry_run_wsl.json`, `preservation_audit.json`, `gui_aided_warning.png`,
`failure_cleanup/failure_path_result.json`, các thư mục smoke và report.

## Đồng bộ UP7000

Nguồn xuất từ Git với LF, kiểm SHA-256 sau truyền, lưu riêng dưới
`~/e23_sources/<commit>/`; `~/e23_sources/latest` trỏ bản nguồn cuối của lượt
này. Archive và manifest ở `source.tar`, `source.manifest.json`; code ở
`source/`. Không xóa `~/UP4000` vì service production thực tế còn dùng nó.

Overlay đã build/test từ commit mã chức năng `0f94726` tại
`~/e23_validation_0f94726`: **39 tests, 0 errors, 0 failures, 1 skipped**
(4 gtest, logger 33 passed/1 skipped, các kiểm tra ament). Các sửa cuối ở
script cleanup/backup, report và test không đổi source estimator C++/logger
đã build; manifest đồng bộ xác nhận native inputs trùng nhau.

Smoke trên UP7000 chỉ dùng dữ liệu giả, domain **92**, localhost:
RUNNING sau **0,007 s**, RMS body **1,064 cm**, không reinit, không mẫu trễ
hơn 50 ms. Bag giả đưa về PC so replay: median **2,247 mm**, p95 **4,311 mm**,
max **8,090 mm**, coverage 99,934%. Node/recorder con được dừng sau bài này.
Tên thư mục smoke tạm từng có CR do stdin PowerShell → bash; đã chuẩn hóa
đúng thư mục cô lập trước khi chép bag. Source archive không gặp CR này.

Đối chiếu trước/sau: binary, helper schema, runner, logger config và active
config production giữ nguyên SHA-256; ba service giữ nguyên PID. Không đổi
guard HDD thực tế (UUID `32e1168f-3eaf-4449-83c4-d45fcd2bd2bf`), không đổi
anchor/lever/yaw, không gửi lệnh PX4, không restart hoặc cắt phiên đang ghi.
Vì chưa activate, chưa xác nhận rate HIGHRES_IMU ≥180 trên production mới.

### Khi người dùng muốn cài và kích hoạt

Trên UP7000, dùng nguồn đã đồng bộ; dry-run không thay đổi gì:

```bash
E23_DIR="$HOME/e23_sources/latest"
E23_SHA="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["sha256"])' "$E23_DIR/source.manifest.json")"
bash "$E23_DIR/source/UP7000/scripts/deploy_onboard_eskf.sh" --source-archive "$E23_DIR/source.tar" --archive-sha256 "$E23_SHA"
```

`--apply` sẽ build/test lại trong overlay mới trước khi sao lưu/cài. Nếu muốn
restart các service và bắt đầu phiên mới, dùng rõ `--restart`:

```bash
sudo -v
bash "$E23_DIR/source/UP7000/scripts/deploy_onboard_eskf.sh" --apply --restart --source-archive "$E23_DIR/source.tar" --archive-sha256 "$E23_SHA"
```

Script in thư mục backup cụ thể. Khôi phục bằng đúng thư mục đó:

```bash
bash "$E23_DIR/source/UP7000/scripts/rollback.sh" "$HOME/e23_backups/backup_<UTC>_<workspace>"
# Thêm --restart nếu muốn service nạp lại bản khôi phục.
```

Script không tự xác nhận cấu hình hay bật aided bias. Những lựa chọn đó
được nhập trong GUI theo rig/lever/yaw thực tế. Không bật EV_CTRL.

## Giới hạn diễn giải

Replay không mô hình hóa độ tươi theo đồng hồ thật, arrival order, timeout
hoặc lịch publish của live node. RF dùng học bias nên không độc lập; GNSS
chỉ được căn theo train và không phải tọa độ khảo sát. Đánh giá TAG Z dùng
sigma vị trí body như E2.1, chưa cộng covariance lever/attitude; IMU và PX4
attitude có tương quan. Kết quả này xác nhận công cụ và tính nhất quán trên
Test_1, không phải chứng nhận độ chính xác bay tuyệt đối. Chưa fuse flow hay
đưa vị trí UWB vào PX4.
