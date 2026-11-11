# Gói thẻ 64 GB đã chuẩn bị cho UP7000

Đã cài gói tại **`/opt/up7000-card64-kit`** trên eMMC và tạo bản ZIP
`UP7000_CARD64_KIT_20261108.zip`, kèm SHA-256. Thẻ thật chưa được cắm;
chưa định dạng, kiểm tra hay kích hoạt thẻ.

Người dùng chọn thẻ **ext4**, dùng trên UP7000 và tải log về Windows qua
mạng. Phần mềm giữ trên eMMC; dữ liệu sẽ ghi trực tiếp lên thẻ.

## Đã kiểm tra

- 12 kiểm thử Linux đạt: chọn đúng loại/dung lượng ổ, từ chối ổ hệ điều hành,
  guard không ghi thay thế vào eMMC, checksum, bảo toàn file, giải nén an
  toàn, đổi fstab và khôi phục cấu hình.
- Snapshot cấu hình ESKF được đóng gói và giải nén/tạo cây thư mục thử
  trong RAM thành công; SHA-256 cấu hình khớp
  `979d3e77d33e4bf25e52960f3672cea433c8372ed1e0f1e73abd700eb648cb6b`.
- Công cụ của gói đã thử Log → Save trên backend HDD hiện tại. Bag đóng
  sạch, ROS 2 giải mã đủ **48 message**, rồi tiếp tục ghi nền. Không có
  Pixhawk/TAG trong phép thử này; ESKF ở `WAIT_CONFIGURATION`.
- Checksum gói cài trên UP7000 và bản ZIP local khớp. Fstab, guard đang
  chạy và hai script của hệ thống đang dùng không bị sửa khi chuẩn bị.
- Cả ba service hiện vẫn active và tiếp tục ghi lên HDD.

## Khi cắm thẻ

Dùng đầu đọc/cáp USB 2.0. Kiểm tra đúng thẻ 64 GB và dữ liệu đang có;
chuẩn bị ext4 sau khi đã xử lý dữ liệu cần giữ. Gói không có lệnh format.
Chạy kiểm tra ghi–đọc vùng trống, rồi kích hoạt và thử Log/Save + MCAP thật.
Cuối cùng kiểm tra lại khi khởi động và so GPS khi HDD đã tháo.

Gói sao lưu cấu hình trước khi chuyển, giữ log cũ trên HDD và sử dụng UUID
để mount thẻ các lần bật máy sau. Chỉ tháo HDD khi hệ thống đã xác nhận
ghi thành công lên thẻ. Tham khảo `UP4000/card64_kit/README_VI.md`.

Kết quả máy đọc: `card64_preparation_report.json`,
`card64_preparation_checks.json`, `card64_native_control_validation.json`.
