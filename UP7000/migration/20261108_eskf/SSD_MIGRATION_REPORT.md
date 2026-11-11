# Chuyển ứng dụng và dữ liệu UP7000 từ HDD sang SSD

**Cập nhật sau báo cáo này:** người dùng chọn phần mềm chạy trên eMMC, ổ ngoài
chỉ chịu phần ghi. Đã đưa ROS/logger/ESKF về eMMC và kiểm thử đạt; dữ liệu vẫn
tạm ghi HDD do SSD chưa đạt kiểm tra. Xem [báo cáo runtime eMMC](EMMC_RUNTIME_REPORT.md).

**Chưa chuyển sang SSD: bản chép đọc lại sai dữ liệu. Hệ thống đã được khôi
phục chạy trên HDD USB 2.0. Cả 3 file SSD cũ giữ nguyên SHA-256 sau khi kiểm
tra lại toàn bộ; SSD đã được unmount và đặt block device ở chế độ chỉ đọc.**

## Phạm vi và cách giữ dữ liệu

Phương án đã chuẩn bị là chuyển toàn bộ nội dung HDD đang dùng cho drone, gồm ROS 2 Humble, workspace
PX4/ESKF, cấu hình, flight log, cache và log dịch vụ sang SSD. Ubuntu gốc vẫn
khởi động từ eMMC; các liên kết ứng dụng tiếp tục dùng `/mnt/droneup`.

SSD có sẵn dữ liệu Windows trên exFAT. Không định dạng, chia lại phân vùng hay
xóa các file này. Vì exFAT không lưu trực tiếp quyền và symlink Linux, hệ thống
Linux nằm trong một file ext4 **32 GiB**, ở thư mục mới:

```text
/mnt/dronessd/UP7000_SYSTEM_20261108_05deda07/droneup.ext4
```

File này được mount tại `/mnt/droneup`; dữ liệu Windows có sẵn nằm bên ngoài.
Dung lượng Linux ban đầu là 32 GiB, không phải toàn bộ dung lượng SSD báo cáo.

## Nhận diện và USB

| Thành phần | Nhận diện |
|---|---|
| HDD nguồn | ext4 UUID `896b17fa-8fe0-4111-853b-57b93ec87c60`, ORICO/JMS578, serial `DD564198838A2` |
| SSD đích | `/dev/disk/by-id/usb-SSD_SSD-0:0`, USB `048d:1234`, exFAT serial `1961929496` |
| Linux trên SSD | UUID `5b4b8b8c-74a6-4298-abde-cc8fc4387b84` |

SSD thương lượng USB 2.0 **480 Mb/s**. HDD cũng đang ở USB 2.0; việc còn cắm
và cấp nguồn HDD có thể vẫn ảnh hưởng GPS. Chưa thể đo lại GPS khi không có
Pixhawk kết nối. Các phép kiểm tra sao chép không xác nhận toàn bộ dung lượng
4 TB mà thiết bị báo cáo.

## Bảo toàn file SSD cũ

Ba file có sẵn, tổng **26.759.449.818 byte**, đã được sao lưu sang một thư mục
mới trên HDD và đọc lại để đối chiếu SHA-256 trước khi ghi hệ thống lên SSD:

```text
<gốc HDD>/_SSD_existing_backup_20261108_112314_6322386e/
  files/
  manifest.json
  original_ssd_first64MiB.bin
  original_ssd_metadata_snapshot.json
  system_stage/
```

HDD nguồn và bản sao này được giữ lại. Sau khi phát hiện bản hệ thống mới bị
lỗi, đã đọc lại toàn bộ 26.759.449.818 byte file SSD có sẵn: cả 3 SHA-256 đều
khớp manifest trước khi ghi. Không xóa hay ghi đè các file có sẵn.

## Cấu hình SSD đã chuẩn bị, chưa áp dụng

1. Xác minh đúng USB SSD, tốc độ USB 2.0 và serial exFAT trước khi mount.
2. Mount exFAT bằng loại filesystem chỉ định rõ, tại `/mnt/dronessd`.
3. Kiểm tra đúng UUID file ext4 và filesystem trước khi mount loop.
4. Mount Linux tại `/mnt/droneup`, rồi chạy logger, ESKF/MCAP và kênh điều
   khiển GUI. Các dịch vụ từ chối ghi khi ổ đúng không có mặt.

Activator chỉ được chạy khi mọi kiểm tra đạt. Lần này nó đã dừng ở trạng thái
`copy_failed_no_activation`: chưa tạo `/etc/up7000-storage.json`, chưa thay
fstab hay helper hệ điều hành để dùng SSD. Cấu hình khởi động HDD vẫn được giữ.

## Kết quả kiểm thử

1. HDD → image Linux trên HDD: sao chép xong, `e2fsck -f -n` đạt trước khi
   chuyển image và đạt thêm lần nữa sau khi phát hiện lỗi SSD. Đọc lại toàn
   bộ image HDD sau lỗi cho SHA-256 khớp chính xác hash tính trong lúc chép:
   `5837b5e26be972b2b024c93af999dec1ccce766f0198af69d52fb865eea566c6`.
2. Image đã ghi đủ 34.359.738.368 byte vào thư mục mới trên SSD, flush/fsync
   rồi kiểm tra đọc lại. `e2fsck -f -n` báo **journal superblock corrupt**;
   `debugfs` cũng báo **block bitmap checksum does not match**.
3. Các mẫu 1 MiB ở offset 0, 1 MiB, 16 MiB, 128 MiB, 1 GiB và 8 GiB khớp
   bản HDD. Các mẫu ở 16, 24 và 31 GiB không khớp.
4. Đọc 4 KiB theo ba cách — qua file exFAT, đọc block vật lý, và SCSI READ16
   — cho cùng dữ liệu ở các mẫu kiểm tra. Các mẫu tại 12, 16, 24 và 31 GiB
   đều sai so với nguồn. Vì thế lỗi không chỉ nằm ở việc mount image ext4.
5. Chưa xác định chắc chắn bộ phận gây lỗi: thiết bị lưu trữ, controller/box,
   đường USB, nguồn hoặc driver/luồng ghi. Chưa kiểm chứng được dung lượng thật của SSD.
   Không dùng kết quả này để khẳng định SSD giả hoặc một dung lượng thật cụ thể.
6. Đã chuyển SSD sang **chỉ đọc**, mount lại đúng HDD UUID cũ tại
   `/mnt/droneup` và khởi động lại `uwb-px4-logger`, `uwb-eskf-recorder`,
   `uwb-recording-control`: cả ba active. Kênh UDP điều khiển GUI xác nhận
   một phiên nền đang ghi, không lỗi; ESKF ở `WAIT_CONFIGURATION`, không có
   mẫu IMU/UWB do chưa kết nối thiết bị.
7. Đã đọc lại toàn bộ 26.759.449.818 byte file SSD cũ: **3/3 file khớp
   SHA-256 trước khi ghi**. Bản sao lưu đã kiểm tra trên HDD vẫn được giữ.
8. Đã unmount SSD và kiểm tra `blockdev --getro` trả về `1`. Ba service trên
   HDD tiếp tục active; có thể rút riêng SSD. Không rút HDD khi đang dùng hệ
   thống. Không cần khởi động lại để hoàn tác vì cấu hình SSD chưa áp dụng.

Bằng chứng nằm trong `ssd_evidence/`: `failure_diagnosis.json`,
`raw_read_comparison.json`, `hdd_restored.json`, `original_ssd_manifest.json`,
`post_failure_integrity.json`, `source_image_sha256.txt`,
`hdd_runtime_validation.json`, `finalized.json`.

Các checkpoint và script chẩn đoán cũng được lưu trên HDD tại
`/mnt/droneup/_SSD_existing_backup_20261108_112314_6322386e/failed_migration_evidence/`.

Chưa kích hoạt ROS/MCAP trên SSD, chưa kiểm thử khởi động từ SSD. Người dùng
cho biết hiện chưa có cáp/box hoặc ổ khác để thay thử.

## Sử dụng hiện tại

Tiếp tục giữ HDD kết nối. Các service vẫn tự chạy bằng cấu hình HDD đã có;
GUI kết nối như trước. Log ở `~/flight_logs/`, MCAP ở `~/flight_logs/eskf/`.
Việc thay HDD bằng SSD chưa hoàn tất và chưa kiểm chứng được cải thiện GPS.

SSD đã được unmount nên có thể rút riêng ổ này. Bản image mới bị hỏng được
giữ trong thư mục `UP7000_SYSTEM_20261108_05deda07` trên SSD và chưa được dùng
chạy hệ thống. Bản image nguồn đạt kiểm tra vẫn còn trên HDD. Cần kiểm tra
SSD/box/cáp/nguồn và kiểm thử ghi–đọc checksum đạt trước lần chuyển tiếp theo.
