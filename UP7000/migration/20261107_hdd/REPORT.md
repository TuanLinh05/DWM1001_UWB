# UP7000 — chuyển dữ liệu eMMC sang HDD, 07/11/2026

Đã hoàn tất chuyển và dọn các bản eMMC tạm. Service `uwb-px4-logger` đang active, enabled; dữ liệu làm việc, ROS runtime, log và cache APT nằm trên HDD. eMMC giải phóng thực tế **1.46 GB (1.36 GiB)**. Các gói/thư viện hệ thống dưới `/usr`, cơ sở dữ liệu dpkg, hệ điều hành và journal hệ thống vẫn giữ trên eMMC; không phải di chuyển toàn bộ OS.

## HDD và đường dẫn

- Phân vùng: `/dev/sda2`, ext4, nhãn `droneup`.
- UUID: `896b17fa-8fe0-4111-853b-57b93ec87c60`.
- Mount: `/mnt/droneup`, đã cấu hình trong fstab, dùng `noatime`, `nofail`, device-timeout 10 giây.
- Dữ liệu mới: `/mnt/droneup/onboard_20261107`.
- Thư mục cũ của HDD (vision, Docker, LiDAR, datasets, local_lib, pip_cache...) được giữ nguyên.

Các đường dẫn cũ được giữ bằng symlink, nên không phải build lại hay đổi các đường dẫn đã compile vào binary.

| Đường dẫn quen dùng | Vị trí dữ liệu thật trên HDD |
|---|---|
| `/home/thoa/UP4000` | `/mnt/droneup/onboard_20261107/thoa/UP4000` |
| `/home/thoa/ros2_px4_v2` | `/mnt/droneup/onboard_20261107/thoa/ros2_px4_v2` |
| `/home/thoa/Micro-XRCE-DDS-Agent` | `/mnt/droneup/onboard_20261107/thoa/Micro-XRCE-DDS-Agent` |
| `/home/thoa/src/8821cu-20210916` | `/mnt/droneup/onboard_20261107/thoa/src/8821cu-20210916` |
| `/home/thoa/drone_tools` | `/mnt/droneup/onboard_20261107/thoa/drone_tools` |
| `/home/thoa/lib/onnxruntime-linux-x64-1.20.1` | `/mnt/droneup/onboard_20261107/thoa/lib/onnxruntime-linux-x64-1.20.1` |
| `/home/thoa/up4000_migration_20261107` | `/mnt/droneup/onboard_20261107/thoa/up4000_migration_20261107` |
| `/home/thoa/flight_logs` | `/mnt/droneup/onboard_20261107/thoa/flight_logs` |
| `/home/thoa/.ros` | `/mnt/droneup/onboard_20261107/thoa/.ros` |
| `/home/thoa/onboard_env.sh` | `/mnt/droneup/onboard_20261107/thoa/onboard_env.sh` |
| `/home/thoa/drone_env.sh` | `/mnt/droneup/onboard_20261107/thoa/drone_env.sh` |
| `/home/thoa/UP7000_MIGRATION.md` | `/mnt/droneup/onboard_20261107/thoa/UP7000_MIGRATION.md` |
| `/home/thoa/.local/bin/MicroXRCEAgent` | `/mnt/droneup/onboard_20261107/thoa/.local/bin/MicroXRCEAgent` |
| `/opt/ros/humble` | `/mnt/droneup/onboard_20261107/system/ros/humble` |
| `/var/cache/apt/archives` | `/mnt/droneup/onboard_20261107/system/apt/archives` |
| `/var/lib/apt/lists` | `/mnt/droneup/onboard_20261107/system/apt/lists` |
| `/var/cache/apt/pkgcache.bin` | `/mnt/droneup/onboard_20261107/system/apt/pkgcache.bin` |
| `/var/cache/apt/srcpkgcache.bin` | `/mnt/droneup/onboard_20261107/system/apt/srcpkgcache.bin` |

Tổng dữ liệu theo dung lượng block được đối chiếu: 1,457,889,280 byte. Đã kiểm tra cả nội dung checksum và metadata bằng rsync trước khi đổi đường dẫn. Bản eMMC chỉ được xóa sau kiểm thử và sau khi người dùng xác nhận rõ việc xóa 18 bản có hậu tố `.emmc-before-hdd-20261107`.

## Ghi dữ liệu mới và phụ thuộc HDD

- Log chuyến bay thật: `/mnt/droneup/onboard_20261107/thoa/flight_logs/<session>`. `~/flight_logs` vẫn dùng được. Đã đặt `log_root` trực tiếp vào HDD trong `/etc/default/uwb-px4-logger`.
- ROS log: `/mnt/droneup/onboard_20261107/thoa/.ros/log`.
- Stdout/stderr ứng dụng: `/mnt/droneup/onboard_20261107/thoa/service_logs/uwb-px4-logger.log`. `journalctl -u uwb-px4-logger` vẫn có sự kiện systemd; log chi tiết của các node xem bằng `tail -f` đường dẫn trên.
- Service dùng `/run/up7000-onboard` trên RAM cho file tạm. CLI khi source `~/drone_env.sh` dùng thư mục tạm và Python cache trên HDD.
- Prefix Python, model, Agent, source, build và cache GUI đều chuyển theo các thư mục nêu trên. Môi trường onboard đặt `PYTHONNOUSERSITE=1` để không bị trộn với các thư viện cũ trong `~/.local/lib` vừa có lại khi gắn HDD.
- Cache tải APT, package lists và hai cache nhị phân pkgcache/srcpkgcache được chuyển. Hai cache nhị phân có cấu hình đường dẫn trực tiếp để file tạm khi APT tái tạo cache cũng ở HDD.
- Service có `RequiresMountsFor`, `BindsTo=mnt-droneup.mount`, kiểm tra mountpoint và script xác minh đúng UUID tại `/usr/local/libexec/up7000-onboard-storage-check`. Thiếu/sai HDD sẽ không cho logger chạy và ghi ngầm trở lại eMMC. APT update/install cũng kiểm tra HDD trước khi thao tác với cache/runtime đã chuyển.
- HDD cần được gắn khi chạy bộ onboard hoặc bảo trì APT. Boot/mount guard được kiểm tra cấu hình; chưa rút ổ hay reboot thật để thử vì HDD còn chứa dữ liệu/dịch vụ cũ.
- Các timer APT đã được khôi phục trạng thái active sau khi kết thúc di chuyển.

## Kiểm chứng sau chuyển

| Kiểm tra | Kết quả |
|---|---|
| Toàn bộ đường dẫn | 18 path trỏ đúng filesystem HDD; không còn bản eMMC tạm sau dọn |
| Logger Linux | 25 test đạt, 1 test bỏ qua vendor-sync cho source standalone |
| Model ONNX | Nạp được model K=27 từ runtime HDD trong domain cô lập, dry-run; thoát sạch |
| ROS tích hợp | TAG giả qua PTY + PX4 giả loopback + ba node ROS thật; không phải test phần cứng |
| UWB mô phỏng | UP7000 parse 17,358 Burst, PC nhận 17,358; 0 lỗi CRC/length/decode |
| ULog mô phỏng | 191 chunk, 47,393 byte; không dropped chunk/dropout |
| File kết quả mô phỏng | UWB/tlog/ULog/timesync đều có device ID của HDD; `all_outputs_on_hdd=true` |
| Trạng thái thật về New_GUI | 160 datagram, 0 lỗi parser, học ground `192.168.1.205:15555` |
| Dung lượng New_GUI nhận | Session ở HDD, ổ tổng 313.89 GB, còn 281.77 GB theo đơn vị thập phân của giao diện |
| Chống fallback | Mô phỏng thiếu mount và sai UUID đều bị từ chối; không unmount HDD đang dùng để thử |
| Descriptor file đang mở | Đã xác nhận log dịch vụ, ROS log, raw UWB, tlog, timesync thuộc device HDD; không còn process giữ backup eMMC |
| APT | `apt-get check` và tra cứu package hoạt động với metadata/cache trên HDD |

Không suy ra tình trạng UART/Pixhawk/WiFi thật từ bài test giả lập. Những giới hạn kiểm thử phần cứng của lần chuyển UP4000 → UP7000 vẫn giữ nguyên.

## Dùng sau này

```bash
systemctl status uwb-px4-logger
df -h / /mnt/droneup
tail -f /mnt/droneup/onboard_20261107/thoa/service_logs/uwb-px4-logger.log
source ~/drone_env.sh
```

Đường dẫn `~/UP4000`, `~/ros2_px4_v2`, `~/onboard_logger`, `~/flight_logs` vẫn dùng như trước, nhưng dữ liệu thật ở HDD. Các drop-in service riêng cho HDD và DDS vẫn được giữ khi chạy script update logger.

## Dung lượng sau dọn

```text
Filesystem                         Size  Used Avail Use% Mounted on
/dev/mapper/ubuntu--vg-ubuntu--lv   26G  9.8G   15G  40% /
/dev/sda2                          293G   15G  263G   6% /mnt/droneup
```

Bằng chứng đi kèm: `migration-state.json`, `storage-validation.json`, `summary.json`, `ros_integration.json`, `status_check.json`. Báo cáo này cũng nằm trên UP7000 tại `~/UP7000_HDD_STORAGE.md`, trỏ tới bản thật trên HDD. Các cấu hình nhỏ và helper cần để mount/kiểm tra HDD vẫn nằm trên eMMC; dữ liệu ứng dụng và log phát sinh được chuyển sang HDD.
