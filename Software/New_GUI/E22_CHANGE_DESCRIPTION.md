# E2.2 — ESKF chống rung bằng phần mềm trên UP7000

Thực hiện 10–11/11/2026, theo `Plan/E2_2_YEU_CAU_ESKF_ONBOARD_CHONG_RUNG_BANG_PHAN_MEM.md`.

## Kết quả

Đã triển khai E2.2-A → E, gồm bản tham chiếu Python, lõi C++ độc lập ROS,
bridge IMU/tư thế, cấu hình GUI, lớp vỏ ROS và chương trình smoke test.

- Full suite GUI/replay cuối: **226/226 đạt, 155,281 s**; 12 test hợp đồng
  E2.2 giữ nguyên. Các test/fixture hợp đồng E2.0/E2.1 không sửa.
- Logger pytest trên Windows: 32 đạt, 2 skip vì flock/pseudo-terminal cần Linux.
  Chạy lại trên WSL Linux: **34/34 đạt, 20 subtest đạt**, không còn skip.
- Node ROS build/install thành công trong WSL Humble; **4 gtest đạt** qua CTest.
- **7 nhóm parity**, gồm 10 biến thể tổng hợp, 100 phép đổi khung và Test_1:
  mọi cờ nhận/từ chối và các bộ đếm driver trong hợp đồng đều khớp.
- Test_1 qua cấu hình onboard: qua toàn bộ ngưỡng E2.1 mục 5.1;
  khởi tạo tại **57,736204 s**, không khởi tạo lại, nhận 86.788/86.789 range
  và 12.657/12.657 cập nhật tư thế.
- Hai smoke ROS localhost 30 s đều đạt: RMS body position khoảng **1,07 cm**
  và **1,05 cm**; mất IMU 0,5 s cho đúng một reset và phục hồi sau **0,069 s**.

**Chưa build/chạy trên UP7000 vật lý và chưa đo stream USB thật.** Mọi chạy
ROS ở PC dùng WSL, localhost và domain 91 riêng. Không SSH, không truy cập
TAG/Pixhawk, không đổi firmware/tham số PX4, không gửi vị trí UWB cho PX4.
Lệnh dành cho người dùng chạy trên UP7000 nằm bên dưới.

## Đổi thư mục và baseline theo yêu cầu bổ sung của người dùng

`UP4000/` vẫn chứa logger/ESKF đang dùng cho UP7000; không phải mã đã bỏ.
Bằng chứng tại tài liệu eMMC/estimator và các service: chạy trên tài khoản
`thoa`, eMMC và kho dữ liệu `/mnt/droneup/onboard_20261107/thoa` của UP7000.

Theo chỉ dẫn của người dùng, đã **đổi toàn bộ thư mục trong workspace thành
`UP7000/`**, giữ nguyên các archive/báo cáo cũ bên trong. Đã commit baseline
62 file nguồn/cấu hình onboard và GUI liên quan, khoảng 599 KB, trước khi sửa
E2.2: `6ebf03b`. Không đưa log chuyến bay, archive cài đặt, migration/test_results,
phần video hay các thay đổi ngoài danh sách vào baseline.

Các đường dẫn `UP4000/...` trong kế hoạch E2.2 được hiểu thành `UP7000/...`.
Ba test E2.2 được chép nguyên byte từ `Plan/E2_2_hop_dong/`, chỉ đổi thư mục
đích của test bridge. Các import đang dùng, hướng dẫn và target của sync_vendor
đã cập nhật. Tên giao thức/node `up4000_status` được giữ để tương thích với GUI.
Những đường dẫn trong bằng chứng lịch sử và docstring test bất biến giữ nguyên.

Đổi thư mục tại PC **không đổi đường dẫn đang chạy trên UP7000**. Bản cài cũ
trên thiết bị có thể vẫn dùng `~/UP4000`; không được xóa/đổi nó khi service
còn phụ thuộc. Các lệnh dưới build trong workspace riêng để kiểm tra nguồn mới.

## Các thay đổi cần review

### Python và C++

`eskf_replay.EstimatorCore` xử lý callback theo thứ tự được đưa vào;
`run_eskf_replay` sắp sự kiện như E2.1 rồi chuyển vào cùng lõi. Phiên bản
`eskf-a-replay-v2` giữ nguyên. Tất cả 11 mảng output với mặc định cũ giống
từng bit so với module trước E2.2.

IMU gốc được kiểm khe hở trước khi lấy trung bình. Khi khe hở vượt ngưỡng,
bỏ filter/tư thế carry, static sums, previous IMU, bin đang mở, buffer GN và
chuỗi từ chối. PX4 R mới nhất còn lưu nhưng không tự làm tư thế sẵn sàng:
cần mẫu tư thế mới sau khi đã nhận ít nhất một raw IMU mới.

Bin = `floor((t-t0)/imu_average_s + 1e-9)`. Chỉ khi mẫu thuộc bin tiếp theo
tới, bin cũ mới phát trung bình t/acc/gyro. Thời gian xử lý vẫn là lúc mẫu
đóng bin tới; thời gian predict/output là mean t. Bin cuối không flush.
Không biến phép lấy trung bình online thành giảm mẫu offline rồi sắp lại
quanh range/tư thế, vì làm vậy thay đổi thứ tự nhân quả.

Chế độ static áp luật đứng yên sau lấy trung bình. Chế độ external lấy
PX4 R mới nhất để khởi tạo/khởi tạo lại; bg lấy từ carry nếu có, ngược lại 0.
Body init vẫn là `tag0 - R*lever`. Giữ R/bg qua range-gate reset hoạt động
như E2.1, độc lập với reset toàn bộ tư thế khi mất IMU.

`Estimator::Params` kế thừa Params filter cũ và thêm các trường driver.
**Không thay giá trị mặc định Params trong filter.hpp.** Thêm log SO(3)
và attitude update dùng body residual, H tại cột 6:9, NIS gate, Joseph và
tiêm đủ 15 trạng thái bằng phép nhân R bên phải. Core chỉ Eigen/stdlib;
Python chỉ numpy/stdlib. Lõi không dùng ROS hoặc đọc đồng hồ thật.

Rà soát biên bổ sung: nếu seed GN trùng anchor, C++ trả NaN như Python
trước khi đưa ma trận suy biến vào Eigen QR; nếu TAG estimate trùng anchor,
range bị từ chối và innovation ghi NaN như Python. Có gtest riêng, không đổi
phép tính trên dữ liệu hợp lệ; các nhóm parity và fixture cũ vẫn khớp.

Một điều chỉnh tương thích cần thiết ngoài danh sách chính: loader tham số
ở `tools/eskf_tuning_report.py` chấp nhận recommendation E2.1 cũ chưa có 6
trường E2.2. Các trường cũ vẫn bắt buộc, khóa lạ/sai kiểu vẫn từ chối.
`imu_gap_reset_s=+inf` mặc định được mã hóa strict JSON thành null và đọc lại
thành +inf; đây là ngoại lệ riêng cho ngưỡng vô hiệu hóa gap. Các số khác vẫn
phải hữu hạn. Đã kiểm file E2.1 thật và roundtrip JSON mới, không sửa test.

### Bridge và GUI

- HIGHRES_IMU xin 200 Hz; ODOMETRY và các stream cũ giữ nguyên; xin thêm
  OPTICAL_FLOW_RAD/DISTANCE_SENSOR 50 Hz để ghi tlog, không fuse.
- `ImuSampleClock.to_stamp` dùng offset hiện có, không thay clock state.
  Quaternion ODOMETRY chuẩn hóa, dùng device timestamp và frame
  `px4_local_ned`, xuất `/px4/attitude_frd`. `/px4/odometry` và IMU giữ quy
  ước cũ; status có số mẫu/loại bỏ của nhánh tư thế.
- Recorder chỉ thêm `/px4/attitude_frd` vào TOPICS.
- Schema thêm đúng default/bounds được yêu cầu, số nguyên cho
  `min_init_anchors` và `use_px4_attitude`, render đủ tham số và attitude_topic.
- GUI thêm 4 ô noise/interval/average và checkbox PX4 attitude. Payload chỉ
  đọc những ô có thật; tham số nâng cao còn lại lấy default. Initial yaw có
  tooltip chỉ hướng mũi so với rig +X tại lúc node bắt đầu.

Default GUI/onboard là na=0,45; ng=0,016; min range interval=0,02 s;
IMU average=0,02 s; PX4 attitude=1; reinit_keep_attitude=true. Mặc định Python
vẫn là cấu hình cũ để giữ hành vi E2.0/E2.1. Không sửa file cấu hình rig hay
`onboard_eskf_settings.json` hiện có; các khóa mới được điền trong bộ nhớ.

### Lớp vỏ ROS

Node ánh xạ ROS params vào Core; giữ kiểm tra cấu hình xác nhận, tuổi mẫu,
timestamp tăng, giới hạn range và timeout theo đồng hồ thật. Range-gate/GN,
static init, averaging và attitude update nằm trong Core. Nhánh CV UWB-only
cũ được giữ riêng. Subscribe sẵn các anchor cấu hình để không phải chờ
timer discovery 1 s. Status phát 10 Hz để quan sát khởi tạo/phục hồi nhanh.

Yaw offset tính một lần từ quaternion PX4 hợp lệ đầu tiên và yaw0 của GUI,
giữ nguyên qua reset. IMU_TIMEOUT gọi reset_attitude một lần, xóa raw-IMU
history để callback tiếp theo không đếm reset lần thứ hai. Mẫu tư thế mới
khôi phục external mode mà không đòi drone đứng yên.

Header ROS ở mốc Unix có double ULP khoảng hàng trăm ns, đủ làm lệch biên
20 ms dù epsilon lõi là 1 ns. Node vì vậy trừ **int64 nanosecond** tại gốc
thời gian phiên trước khi đổi sang giây cho Core; publication cộng lại gốc.
Freshness vẫn kiểm trên clock ROS thực. Với smoke 6.000 IMU/24.000 range,
giới hạn giữ đúng 12.000 range, không thành hiệu ứng lấy mẫu thưa do mất
độ phân giải thời gian. Đây là chỉnh sửa ở adapter, không đổi toán Core.

CMake có `UWB_ESKF_WITH_PX4_MSGS` mặc định ON để giữ nhánh uXRCE-DDS `imu=px4`.
WSL PC thiếu px4_msgs và colcon nên đã build bằng CMake/ament với tùy chọn
OFF; nhánh MAVLink thực dùng cho E2.2 vẫn được build và smoke đầy đủ. Máy
UP7000 có workspace px4_msgs có thể giữ ON như lệnh dưới.

## Parity và Test_1

| Nhóm | Biến thể |
| --- | --- |
| 1. Online average | online / preaveraged |
| 2. Static start rung | blocked / legacy loose / external |
| 3. IMU gap | static / external |
| 4. External reinit | dùng R PX4 mới nhất |
| 5. Frame conversion | 100 quaternion, chuẩn hóa, yaw và yaw offset |
| 6. E2.1 carry | giữ tư thế / dùng lại boot attitude |
| 7. Test_1 | native IMU → online mean50 + PX4 attitude |

Sai khác lớn nhất trên mọi nhóm:

| Đại lượng | Đo được | Giới hạn |
| --- | ---: | ---: |
| max abs Δp | 2,0872e-14 m | 1e-6 m |
| Góc ΔR | 2,8577e-14 rad | 1e-6 rad |
| max abs ΔP / max abs P | 7,8733e-15 | 1e-6 |
| max ΔNIS | 3,5527e-13 | chỉ báo thêm |
| Cờ mọi range/tư thế, bộ đếm hợp đồng và thời gian output | trùng | phải trùng |

Các trường summary được so: reinits, skipped_ranges, imu_gap_resets,
attitude_inits, initializations, offered_ranges, accepted_ranges,
accepted_attitudes, rejected_attitudes, output_rows và init_time.

Harness filter E2.0 cũ cũng chạy lại bằng bản sao ở validation E2.2:
360 sự kiện giữ flag giống nhau; max sai khác state/P/NIS 1,4211e-13.
Không sửa harness/log E2.0 gốc.

Test_1 dùng đúng cấu hình tham chiếu: na=0,293; ng=0,0107; averaging/range
interval 0,02 s; external init/update; static_check=true; gap threshold 0,2 s.
K=0 và căn khung chỉ học 75–195 s; không chọn tham số bằng holdout.

| Cửa sổ (s) | XY RMSE (cm) | XY p95 (cm) | Z median (cm) | Z p95 abs (cm) | Yaw p95 (độ) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 75–195 học | 10,945 | 20,158 | −0,316 | 12,050 | 0,604 |
| 198–218 kiểm | 16,305 | 29,925 | +0,941 | 12,218 | 0,694 |
| 235–300 kiểm | 22,980 | 29,239 | −0,010 | 11,147 | 0,444 |

Gộp 75–300 s: 56.562 epoch RF tươi; abs(dz)<2σ = **0,952919**;
RMS chuẩn hóa = **0,949558**; median σz body = 6,182 cm. Nghiêng p95
theo ba cửa sổ = 0,528 / 0,452 / 0,451°. Không reinit, một attitude init,
420.768 range bị bỏ vì giới hạn tốc độ.

RF vẫn là nguồn dùng học hiệu chuẩn, không phải ground truth độc lập.
GNSS được căn trên đoạn học, không phải khảo sát tuyệt đối; σ TAG dùng std
vị trí body, bỏ phần lever-arm attitude covariance như yêu cầu. PX4 attitude
và IMU có tương quan. Không diễn giải các số này là độ chính xác bay tuyệt đối.

## Smoke test tại PC

| Chạy 30 s | RUNNING sau (s) | RMS sau 6 s | Reinit range | IMU gap reset | Phục hồi sau IMU trở lại |
| --- | ---: | ---: | ---: | ---: | ---: |
| Bình thường | 0,0998 | 1,065 cm | 0 | 0 | — |
| Mất IMU [15;15,5) s | 0,1022 | 1,045 cm | 0 | 1 | 0,0686 s |

Normal nhận 6.000 IMU/24.000 range; gap nhận 5.900 IMU/24.000 range.
Cả hai bỏ 12.000 range do rate limit; attitude update nhận 1.499 / 1.483,
không từ chối. Gap có đúng 2 attitude init và 2 position init. Không mẫu nào
trễ lịch mô phỏng hơn 50 ms. Script so body origin với truth, không nhầm TAG.

## Lệnh để người dùng chạy trên UP7000

Xuất bản đã commit với LF từ PC, gồm cả `UP7000/scripts/reset_tag_usb.py`:

```powershell
git -c core.autocrlf=false archive --format=tar --output=e22_source.tar HEAD UP7000/estimator UP7000/ros2_ws/src/uwb_px4_logger UP7000/scripts Software/New_GUI/eskf_config.py
```

Giải nén archive vào `~/Github Clone UWB Test` trên UP7000; kiểm nguồn không có CR.
Không chép working tree Windows trực tiếp vì `core.autocrlf=true` có thể tạo CRLF.
Không cần đổi/xóa
`~/UP4000` đang được service cũ sử dụng. Chạy tại terminal **trên UP7000**:

```bash
E22_REPO="$HOME/Github Clone UWB Test"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_px4_v2/install/setup.bash"
mkdir -p "$HOME/e22_validation_ws"
cd "$HOME/e22_validation_ws"
colcon build --base-paths "$E22_REPO/UP7000/estimator/uwb_eskf" "$E22_REPO/UP7000/ros2_ws/src/uwb_px4_logger" --packages-select uwb_eskf uwb_px4_logger --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export PYTHONPATH="$HOME/drone_tools/python${PYTHONPATH:+:$PYTHONPATH}"
colcon test --base-paths "$E22_REPO/UP7000/estimator/uwb_eskf" "$E22_REPO/UP7000/ros2_ws/src/uwb_px4_logger" --packages-select uwb_eskf uwb_px4_logger --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
```

Giữ cùng `--base-paths` khi build/test để tránh colcon chạy 0 test. `PYTHONPATH`
trên cung cấp pymavlink đã có trên máy. Test logger còn dùng script reset USB
trong `UP7000/scripts`, nên không được bỏ thư mục đó khỏi archive. E2.3 bổ sung
`UP7000/scripts/deploy_onboard_eskf.sh`: mặc định dry-run, build overlay riêng,
`--apply` sao lưu đủ bốn phần đang dùng và chỉ restart khi có `--restart`.

Nếu máy không có px4_msgs, đường MAVLink không cần nó: thêm
`-DUWB_ESKF_WITH_PX4_MSGS=OFF` ở cuối lệnh build. Nhánh `imu=px4` trực tiếp
sẽ báo lỗi rõ nếu được chọn trong bản build này. Không tắt dependency bằng
cách sửa package hoặc giả lập header.

Tiếp tục trong terminal đã source workspace mới, chạy hai mô phỏng cách ly:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=91
export FASTRTPS_DEFAULT_PROFILES_FILE="$E22_REPO/UP7000/estimator/fastdds-udp.xml"
python3 "$E22_REPO/UP7000/estimator/smoke_test_eskf.py" --out "$HOME/flight_logs/e22_smoke_normal"
python3 "$E22_REPO/UP7000/estimator/smoke_test_eskf.py" --imu-gap --out "$HOME/flight_logs/e22_smoke_gap"
```

Mỗi lệnh tự mở đúng node từ overlay workspace, phát 30 s dữ liệu giả và dừng
chính child đó. Không mở cổng serial. Mã thoát 0 và `report.json: passed=true`
là đạt; 1 là tiêu chí không đạt, 2 là lỗi môi trường/khởi chạy. Kết quả mong
đợi: RUNNING<1 s, RMS<5 cm sau 6 s, reinit=0, attitude_updated/rate_limit_active
đều true; bài gap phải có reset=1 và recovery<0,5 s. `node.txt` và
`status_history.json` giữ bằng chứng. Sau smoke, `unset ROS_DOMAIN_ID
ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE` trong terminal này nếu cần
quay về domain vận hành cũ.

Các lệnh trên chỉ build/test/smoke ở overlay riêng, chưa thay service production.
Khi cập nhật phần đang chạy, cần đồng bộ cả ba phần, tránh chỉ thay binary:

1. Binary `uwb_eskf` và package/logger config mới phải là bản service đang nạp.
   Recorder hiện trỏ binary tại `~/ros2_px4_v2/install/uwb_eskf/lib/uwb_eskf/uwb_eskf_node`;
   chỉ build overlay không đổi con trỏ này.
2. Schema `Software/New_GUI/eskf_config.py` phải được cài đồng bộ cho helper
   dưới tên `/usr/local/libexec/up7000_eskf_config.py` (recording_action import
   tên đó), cùng GUI mới trên laptop. Schema cũ sẽ từ chối các khóa na/ng mới.
3. Bản `run_onboard.py` đang chạy tại `~/drone_tools/eskf/run_onboard.py` phải
   được cập nhật để bag có `/px4/attitude_frd`. Không thay các guard HDD/UUID,
   cấu hình anchor/lever/yaw hoặc snapshot phiên cũ.

Đây là thao tác cập nhật service do người dùng thực hiện sau khi build/smoke
trên máy đạt, ngoài các thao tác PC đã làm trong lần này. Không chạy lại toàn
bộ installer cũ chỉ để cập nhật E2.2. Không đổi PX4 hay bật EV_CTRL.
Khi chạy thật, kiểm `/px4/status` bằng `ros2 topic echo --once /px4/status`:
`rates_hz.HIGHRES_IMU` phải ≥180, có attitude samples tăng; request 200 Hz
không tự chứng minh USB/PX4 đã phát đủ tốc độ. Kiểm bag có topic tư thế mới.

## Tái lập parity tại PC/Linux

Từ `Software/New_GUI` trên PC có các dependency offline của E2.0:

```powershell
python tools/export_core_events.py --out logs/e22_validation/parity --flight Logs/10112026/Test_1
```

Từ root repo trên Linux/WSL có numpy, g++ và Eigen:

```bash
g++ -std=c++17 -O2 -IUP7000/estimator/uwb_eskf/include -I/usr/include/eigen3 UP7000/estimator/uwb_eskf/test/estimator_parity.cpp -o Software/New_GUI/logs/e22_validation/estimator_parity
python3 Software/New_GUI/tools/check_core_parity.py --suite Software/New_GUI/logs/e22_validation/parity --exe Software/New_GUI/logs/e22_validation/estimator_parity
```

`parity_report.json: all_passed=true` xác nhận từng threshold. Dữ liệu Test_1
xuất ra `Software/New_GUI/logs/e22_validation`, không ghi vào Logs gốc.
Fixture nhỏ `test/data/golden` được commit để gtest chạy độc lập Test_1.

## Bảo toàn và ghi chú kiểm thử

So SHA-256 trước/sau có mapping thư mục UP4000→UP7000: **485 file được bảo vệ
không thay đổi**. Bao gồm dữ liệu Logs, archive, test bất biến, cấu hình rig,
settings có sẵn và các chỉnh sửa ngoài phạm vi. Chỉ baseline được người dùng
chấp thuận và các file E2.2/đường dẫn rename được đưa vào commit. Hunk có sẵn
khác trong `test_udp_relay_input.py` được giữ ngoài commit; README GUI cũng
giữ thay đổi cũ ngoài commit. Ba hợp đồng E2.2 có hash byte trùng bản trong Plan.

Một lần full suite gặp native Qt access violation khi pump event UDP/GUI
(log `full_suite_native_crash.txt`). Bài UDP chạy riêng 2/2 đạt; full suite verbose sau đó
226/226 đạt. Không sửa/skip test để né lỗi, và không khẳng định đã sửa dứt
điểm lỗi native Qt này bằng thay đổi E2.2.

Bằng chứng local tại `Software/New_GUI/logs/e22_validation/`: full_suite.txt,
logger_pytest_linux.txt, parity/parity_report.json, parity/07_test1/test1_report.json,
smoke_normal/report.json, smoke_gap/report.json, default_bitwise_parity.json,
cpp_parity_report.json, preservation_audit.json và gui_config_panel.png.
Kiểm CTest tại `ros_build/Testing/Temporary/LastTest.log`.
