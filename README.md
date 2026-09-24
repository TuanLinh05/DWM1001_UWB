# DWM1001 UWB Ranging System

Hệ thống đo khoảng cách UWB dùng DWM1001C (DW1000 + nRF52832), gồm một TAG,
tám Anchor cố định, firmware Sniffer và GUI Python. Source firmware đang chạy
nằm trong `Firmware/`; mã dùng chung nằm trong `Firmware/common/`.

> [!WARNING]
> Cấu hình repository giữ `UWB_DS_CALIBRATED_MASK = 0`. Range raw dùng được
> cho commissioning/calibration nhưng không hợp lệ cho điều khiển drone cho tới
> khi từng đường TAG–Anchor được hiệu chuẩn và checklist phần cứng hoàn tất.

## Kiến trúc

```text
                           DS-TWR v2, Ch5
 DWM1001C TAG  <-------------------------------->  Anchor_1 ... Anchor_8
 address 0x0000                                  address 0x0001 ... 0x0008
       |
       | UART binary + CRC16 (115200 mặc định)
       v
 USB-UART / ESP32-C3  -------------------------->  GUI / logger / host commands
```

- PHY: channel 5, PRF16, preamble 256, PAC16, 6.8 Mbps.
- Khung UWB v2 có version + transaction ID; tám Anchor phải được flash trước,
  TAG flash sau.
- Telemetry có INFO/RANGE/STATS, diagnostics, DEVICE_INFO và command/ACK.
- Calibration fail-closed: dữ liệu chưa calibration vẫn được xuất để đo nhưng
  không mang cờ hợp lệ.
- Mục tiêu chu kỳ là 20 ms/50 Hz cho đủ tám Anchor. Đây vẫn là mục tiêu phải đo
  trên phần cứng; ước tính DS-TWR tuần tự hiện là 25–33 ms.

## Cấu trúc repository

| Đường dẫn | Vai trò |
|---|---|
| `Firmware/common/` | Driver DW1000, ranging, command, settings, telemetry dùng chung |
| `Firmware/Tag/` | TAG trên carrier tùy chỉnh |
| `Firmware/Tag_DevKit/` | TAG trên DWM1001-DEV |
| `Firmware/Anchor_1/` … `Anchor_8/` | Tám ảnh Anchor, địa chỉ 1…8 |
| `Firmware/Sniffer_DevKit/` | Sniffer thụ động trên DWM1001-DEV |
| `Firmware/tests/` | Host test C/Python và simulator thanh ghi DW1000 |
| `Firmware/ESP32C3_Gateway/` | Gateway ESP-IDF tùy chọn |
| `Software/UWB_UART_GUI/` | Decoder, GUI, recorder và test Python |
| `Plan/` | Kế hoạch, ADR và trạng thái triển khai |

## Build và test

Yêu cầu nRF Connect SDK v3.4.0. Từ thư mục gốc:

```powershell
# Host tests (có thể bỏ parser gateway khi chưa làm ESP32-C3)
.\Firmware\tests\run_host_tests.ps1 -SkipGateway

# Build đủ 11 project Zephyr
.\Firmware\scripts\build_all.ps1

# Hoặc chỉ một vài project
.\Firmware\scripts\build_all.ps1 -Projects Tag,Anchor_1,Sniffer_DevKit
```

Mỗi node tạo `Firmware/<project>/build/zephyr/zephyr.hex`. Build nhúng:

- Git hash;
- cờ dirty, bao gồm cả file untracked;
- hash cấu hình riêng của node (`CMakeLists.txt`, `prj.conf`, overlay và
  `uwb_app_config.h`).

Không dùng artifact có `dirty = 1` cho deployment chính thức.

## Flash

Flash theo thứ tự bắt buộc:

1. `Anchor_1` … `Anchor_8`;
2. xác nhận địa chỉ từng Anchor bằng Sniffer/diagnostics;
3. TAG hoặc Tag_DevKit;
4. gateway nếu sử dụng.

Ví dụ:

```powershell
Set-Location .\Firmware\Anchor_1
.\scripts\flash.ps1

Set-Location ..\Tag
.\scripts\flash.ps1
```

Không tráo binary Anchor. Ghi serial/PARTID/LOTID, project, hash source và hash
cấu hình vào `Firmware/DEPLOYMENT_MANIFEST.md` ngay khi flash.

## Settings và calibration

TAG lưu cấu hình bằng một snapshot NVS schema v2 có CRC và generation. Snapshot
gồm radio profile, antenna delay, TX power, calibration A1–A8, active mask và
telemetry features.

Calibration đã lưu chỉ được bật khi đồng thời khớp:

- PARTID của DW1000 trên TAG;
- calibration profile ID tạo từ PHY, TX power, antenna delay, reference tuning
  và các tùy chọn ranging ảnh hưởng tới bias.

Đổi `SET_ANT_DELAY` hoặc `SET_TX_POWER` sang giá trị mới sẽ tự động xóa mọi cờ
calibrated. Hãy calibration lại rồi mới `SAVE_SETTINGS`. Settings schema cũ
`uwb/radio`, `uwb/cal`, `uwb/tag` bị bỏ qua fail-closed; dùng `FACTORY_RESET`
để xóa chúng.

## UART và RANGE_MEAS

Baud mặc định của TAG là 115200, phù hợp telemetry snapshot/diagnostics mặc
định. `RANGE_MEAS` từng phép đo yêu cầu tối thiểu 460800 và sẽ bị firmware từ
chối ở 115200. Chỉ đổi baud khi đã cập nhật đồng bộ TAG, adapter/gateway và host.

## Tài liệu bắt buộc trước deployment

- [`Firmware/README.md`](Firmware/README.md): build, protocol và cấu hình.
- [`Firmware/CHANGELOG.md`](Firmware/CHANGELOG.md): thay đổi firmware v2.
- [`Firmware/HARDWARE_AB_CHECKLIST.md`](Firmware/HARDWARE_AB_CHECKLIST.md): A/B
  RF, timing, calibration và soak.
- [`Firmware/ANCHOR_DEPLOYMENT.md`](Firmware/ANCHOR_DEPLOYMENT.md): gán module,
  địa chỉ, thứ tự flash và trạng thái từng Anchor.
- [`Firmware/DEPLOYMENT_MANIFEST.md`](Firmware/DEPLOYMENT_MANIFEST.md): truy vết
  binary ↔ module ↔ vai trò.

Build/test thành công không thay thế việc đo RF thật, kiểm tra nguồn/antenna,
NLOS, timing tám Anchor và soak trong điều kiện bay.

## Giấy phép tham chiếu

Thiết kế có tham khảo kiến trúc Bitcraze LPS. Nếu đưa mã hoặc bảng dữ liệu từ
`bitcraze/lps-node-firmware` hay `libdw1000` vào repository, phải giữ attribution
và tuân thủ giấy phép nguồn tương ứng. Repository này cần có `LICENSE` riêng
trước khi phát hành công khai.
