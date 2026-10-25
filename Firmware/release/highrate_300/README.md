# UART quality / calibration GUI — 25/10/2026

TAG config **`0xb5f8839d`**.
[Đặc tả, băng thông và hướng dẫn GUI](../../docs/TELEMETRY_QUALITY_20261025.md).

Gói GUI/tester để tải từ GitHub nằm tại
[`Software/UWB_UART_GUI/releases`](../../../Software/UWB_UART_GUI/releases).
Các thư mục previous_*, build_staging_* và transcript build chỉ lưu ở máy
phát triển; branch chứa bộ HEX hiện tại cùng báo cáo kiểm tra cần thiết.

Giữ lịch ranging của baseline0b033006 đã đo khoảng275–276 DS/s/anchor.
Lượt này bổ sung UART/GUI, tạm gác tối ưu tốc độ. Chưa đo Hz của ảnh mới.

- Base450/slot275/margin450 UUS, gap180 us, info extra64 UUS giữ nguyên.
- Giữ double RX, pipeline tính range và định kỳ STATS/DIAG của vòng trước.
- Profile300 từ chối slot<275 hoặc margin<450 để tránh áp lại lịch lỗi.
- Period3100 us là cận dưới. Chưa xác nhận300 DS/s trên board với bản này.
- Chỉ cần nạp TAG; anchor300/PHY64 hiện có vẫn tương thích.
- Giữ khung compact0x18; thêm0x19 raw quality10 Hz khi DIAG bật.
- Temperature/VBAT TAG cập nhật khoảng1 Hz; có sample identity và tuổi mẫu.
- GUI mới `DWM1001_UWB_Ground_Control_quality.exe`, tab RX chất lượng,
  log rx_quality.csv. Gói tester `UWB_tester_quality_20261025.zip` gồm GUI
  ImGui đã sửa, bridge UART→ROS và patch áp vào checkout gốc.
- `manifest.csv`: SHA256/config ID của chín ảnh trong bộ hiện tại.
- `package_verification_quality_20261025.json`: đối chiếu9 HEX/build.
- `host_tests_quality_20261025.txt`:26 nhóm test và GUI test.
- `validation_quality_20261025.json`: hash và biên nhận kiểm tra cuối.
- `bandwidth_quality_20261025.json`: tính tải UART65,32%→71,21%.
- `previous_20261025_before_quality/`: bộ baseline0b033006 trước sửa UART.
- `audit_log_20261019_143252.json`: số đo của phiên lỗi, không phải bản mới.
- `previous_20261019_before_final_recovery/`: lưu bản394ce68f đã lỗi trên board.
- `previous_20261019_before_pipeline/`: baseline84e6c9b9 đã đo khoảng271 cycle/s.

Đóng GUI/relay, nạp riêng TAG:

```powershell
& 'D:\Drone Project\Github Clone UWB Test\Firmware\scripts\flash_highrate.ps1' -Node TAG -Rate 300
```

Mở GUI mới, kiểm configb5f8839d và DIAG slot275/margin450, ghi60–120 s.
Cần đánh giá FINAL late/timeout, DS/s mỗi anchor và đủ tám/s theo TAG time,
gap, CRC UART và queue drop. Sau khi phiên ngắn ổn mới thử10 phút.
Chưa tự flash thiết bị trong lượt sửa này. Các báo cáo ngày19/10 mô tả
ảnh vòng trước; dùng báo cáoquality_20261025 để đối chiếu bộ hiện tại.
