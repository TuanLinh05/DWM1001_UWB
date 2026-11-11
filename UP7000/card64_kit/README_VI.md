# Gói thẻ dữ liệu 64 GB cho UP7000

Phần mềm Ubuntu/ROS/logger/ESKF tiếp tục chạy trên eMMC. Thẻ **ext4** chứa
log UWB/PX4, MCAP, ROS log, cache Python, dữ liệu tạm và trạng thái phiên.
Ghi trực tiếp lên filesystem thẻ, không dùng image Linux lớn bên trong thẻ.

## Ngày lắp thẻ

1. Dùng đầu đọc hoặc cáp **USB 2.0**; đầu đọc vẫn có thể phát nhiễu nên cần
   so GPS khi không cắm và khi đang ghi. Thẻ 64 GB chưa được đo tốc độ thực.
2. SSH `thoa@100.123.216.14`. Gói được đặt ở `/opt/up7000-card64-kit`.
3. Xem đúng thiết bị bằng lệnh bên dưới. Gói không có lệnh định dạng ổ.
   Nếu thẻ đang exFAT/FAT32, cần sao lưu dữ liệu và xác nhận thiết bị cụ thể
   trước khi chuẩn bị ext4. Không chọn eMMC, HDD hoặc SSD đã phát hiện lỗi.

```bash
python3 -B /opt/up7000-card64-kit/card64.py inspect
python3 -B /opt/up7000-card64-kit/card64.py check-kit
```

Sau khi đã có thẻ ext4 trống, lấy UUID ext4 từ kết quả `inspect`, thay
`<UUID_THE_64GB>` trong các lệnh:

```bash
sudo python3 -B /opt/up7000-card64-kit/card64.py qualify --uuid <UUID_THE_64GB>
sudo python3 -B /opt/up7000-card64-kit/card64.py activate --uuid <UUID_THE_64GB>
python3 -B /opt/up7000-card64-kit/verify_runtime.py --rotate-test
```

`qualify` mặc định ghi/đọc checksum trên vùng trống có thể sử dụng, chừa
2 GiB và phần filesystem dành riêng. Phép thử toàn dung lượng có thể mất
nhiều giờ trên thẻ chậm; chỉ các file kiểm thử mới được xóa khi checksum đạt.
Nếu ưu tiên đưa logger vào hoạt động ngay, có thể bỏ qua phép thử toàn dung
lượng bằng lệnh:

```bash
sudo python3 -B /opt/up7000-card64-kit/card64.py activate --uuid <UUID_THE_64GB> --waive-full-card-test
```

Lựa chọn này ghi rõ trong báo cáo rằng dung lượng đầy đủ chưa được xác minh.
Kiểm tra runtime sau đó xác nhận GUI/ROS/MCAP hoạt động, nhưng không thay thế
kiểm tra dung lượng, checksum toàn thẻ hoặc đo ảnh hưởng lên GPS.

Khi chạy qua SSH lâu, dùng phiên terminal duy trì hoặc để người hỗ trợ
khởi chạy job nền rồi theo dõi `/run/up7000-card64/status.json`. Không
đóng phiên kiểm tra giữa chừng hoặc tháo thẻ khi đang ghi.

## Khi kích hoạt

- Chỉ nhận volume ext4 khoảng 64 GB, gắn qua USB 2.0 hoặc SD native; từ
  chối ổ hệ điều hành, HDD nguồn và controller SSD lỗi đã biết.
- Kiểm tra báo cáo qualification gắn đúng UUID/thiết bị, không quá 7 ngày.
- Đóng phiên ghi nền hiện tại; từ chối chuyển khi GUI đang sở hữu phiên log.
- Giữ bố cục `/mnt/droneup/onboard_20261107/...` để GUI và đường dẫn log
  tiếp tục hoạt động. Cấu hình ESKF được chụp lại từ HDD nếu HDD còn kết nối;
  nếu không, dùng snapshot đi kèm gói. GUI vẫn gửi cấu hình mới lúc bấm Log.
- Tạo phiên log mới trên thẻ; log cũ, source/archive cũ và các bản backup
  vẫn nằm nguyên trên HDD. Không chép các archive/source không thuộc luồng
  logger/ESKF hoặc image thử SSD vào thẻ.
- Sao lưu fstab, guard và hai script chọn UUID ở
  `/etc/up7000-card-backups/<timestamp>/`; thử quay về cấu hình trước nếu
  chuyển không đạt. Nếu HDD đã tháo, cấu hình cũ có thể chưa chạy được.
- Sau khi mount đúng thẻ và cả ba service được xác nhận khỏe, mới tháo HDD.
  Các lần bật máy sau tự mount bằng UUID và chạy các service. Thiếu thẻ sẽ
  từ chối ghi, không ghi log thay thế vào eMMC.

`verify_runtime.py --rotate-test` dành cho kiểm tra bảo trì trên mặt đất:
thử giao thức Log → Save, giải mã MCAP bằng ROS 2 rồi tiếp tục phiên ghi nền.
Nó không gửi lệnh điều khiển bay. Sau đó thử thêm với Pixhawk + TAG thật và
đo C/N0/fix/GPS khi đang ghi trước khi dùng trên drone.

## Lấy dữ liệu

GUI kết nối như hiện tại, không đổi địa chỉ Tailscale. Tải một phiên đã
đóng về laptop bằng `scp -r thoa@100.123.216.14:<duong_dan_phien> <thu_muc_PC>`.
Giữ `bag/`, `config/`, `session.json`. Để tháo thẻ, tắt UP7000 sạch rồi mới rút.

Gói không chứa mật khẩu SSH; các lệnh sudo dùng quyền của tài khoản quản trị.
