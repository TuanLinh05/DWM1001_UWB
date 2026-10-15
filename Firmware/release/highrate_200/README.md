# Firmware DS-TWR mục tiêu ≥200 Hz — bản ngày 2026-10-02

Bộ dành cho **TAG DWM1001-DEV, A1–A4 DWM1001C/nRF52832, A5–A8 STM32F103C8 + DW1000**. Chu kỳ mặc định 4.500 µs, tương đương **222,22 Hz danh định**. Mục tiêu nghiệm thu là **ít nhất 200 kết quả DS mới/s trên từng anchor và 200 bộ đủ tám anchor/s**. Tần số này còn phải đo trên phần cứng sau khi nạp; build và mô phỏng không chứng minh đạt Hz thực tế.

## 1. Các thay đổi

- **Anchor tự mở RX bằng WAIT4RESP + ACK_RESP_T**, được lập lịch trước khi phát RESP. Việc nghe FINAL không còn phụ thuộc MCU kịp xử lý cờ TX hay lập lịch RX sau khi phát xong. Bù thời gian đuôi RESP và phần preamble FINAL, gồm cả RESP có TLV.
- **Không xóa FINAL đã nhận khi xử lý TXFRS**. Khi TX và RX cùng có trong một lần đọc status, anchor xử lý cả hai; giữ timestamp 40 bit và Rb đúng chu kỳ. Đây là thay đổi nhằm giải quyết nút thắt A8 của profile nhanh cũ.
- **STM32 A5–A8 dùng SPI1 trực tiếp, 16 MHz khi chạy, 2 MHz khi khởi tạo**; nRF dùng SPIM2 EasyDMA trực tiếp, 8 MHz. Kiểm tra cấu hình chặn sai bus, xung APB2 hoặc thiếu pinctrl. Mỗi giao dịch có timeout và đưa lỗi vào cơ chế recovery hiện có.
- **nRF bật FPU, toàn bộ profile dùng tối ưu tốc độ `-O2`**. Giảm chi phí logarit của phép tính FPP/RX power; phép xấp xỉ được kiểm tra 20.000 trường hợp, sai khác dưới 0,0002 dB. Phép tính ToF DS giữ số nguyên 64 bit/double, không bật fast-math.
- **TAG kiểm tra header/transaction của RESP cuối trước khi đóng RX để phát FINAL**. Frame lạ hoặc trùng không làm mất cơ hội nhận A8. Vòng phục vụ radio dùng khoảng nghỉ 1 µs.
- Giữ giao thức burst v3: một POLL, tám RESP, một FINAL; Rb của N được trả ở RESP của N+1. Mỗi bản ghi là một trao đổi mới, trễ một chu kỳ. Thiếu FINAL/Rb xuất NONE + TIMEOUT; **không dùng SS fallback** để bù số đo.
- Giữ watchdog, recovery, calibration, bộ lọc, diagnostics, commands và UART logging. Hash nhận dạng build bao gồm nguồn firmware để phân biệt các bản sửa trên cùng commit.

Cách auto RX học từ `Reference/dw1000/bdstwr/core/bd_anchor.c` và [Decawave APS022](https://forum.qorvo.com/uploads/short-url/eDORWUXYWgvoamenSVow2Pe2CIT.pdf). Packet/PHY giữ tương thích hệ thống hiện tại: Ch5, PRF16, preamble128, PAC8, 6,8 Mbps; TAG=0, anchor=1…8.

## 2. Timing mặc định

| Tham số | Giá trị |
|---|---:|
| POLL → RESP đầu | 550 UUS |
| Khoảng slot RESP | 350 UUS |
| RESP cuối → FINAL | 450 UUS |
| Gap sau FINAL | 250 µs |
| Period tối thiểu | 4.500 µs |
| Tần số danh định | 222,22 chu kỳ/s |
| FINAL, đủ tám anchor | 3.450 UUS sau POLL RMARKER |

1 UUS ≈ 1,026 µs. Chu kỳ TLV mỗi giây mở rộng slot thêm 64 UUS. MCU, telemetry và lỗi radio có thể làm chu kỳ dài hơn period được đặt. Nếu chu kỳ thực tế đạt 222 Hz thì cần khoảng 90% chu kỳ đủ tám DS để giữ 200 bộ/s; phải dùng tỷ lệ thành công đo được để kết luận.

UART **460800 baud, 8N1**. Burst tám anchor khoảng 102 byte: tại 222,22 Hz khoảng 22,7 kB/s, so với trần UART 46,08 kB/s trước các frame nền. Không đổi sang 1 Mbaud trên J-Link VCOM trong bản này.

Mục tiêu 400 Hz tương ứng 2,5 ms/chu kỳ. Lịch hiện tại đã dành khoảng 3,54 ms từ POLL đến FINAL RMARKER, nên chỉ đặt `period=2500` sẽ không tạo được 400 Hz. Cần tiếp tục giảm thời gian radio và xử lý dựa trên phép đo; bản này ưu tiên đạt ngưỡng 200 Hz đủ tám anchor.

## 3. Image và nhận dạng

| File | Thiết bị |
|---|---|
| `TAG_DWM1001_DEV.hex` | TAG nRF52832, địa chỉ 0 |
| `A1_DWM1001C_nRF52832.hex` … `A4_DWM1001C_nRF52832.hex` | A1–A4 nRF52832 |
| `A5_STM32F103C8.hex` … `A8_STM32F103C8.hex` | A5–A8 STM32F103C8 |

`manifest.csv` ghi SHA-256, config ID và SPI speed từng image. Đối chiếu config ID TAG trong DEVICE_INFO và từng anchor trong ANCHOR_INFO. `package_verification_20261002.json` là kết quả kiểm tra HEX/vector/bounds/config/build; `build_20261002.txt` và `host_tests_20261002.txt` lưu kết quả build/test. Chi tiết xác nhận ở `VERIFICATION_20261002.md`.

**Nạp lại đủ TAG và A1–A8** để các anchor có auto RX. Profile mặc định đã là 222 Hz; lệnh ROS relay chỉ bật telemetry và không đổi timing. Bản trước được lưu ở `previous_20261002_before_auto_rx`; các thư mục `previous_*` dùng truy nguyên, không chọn nhầm image ở đó.

## 4. Nạp firmware

Đóng GUI/relay đang giữ COM14 và phiên debug. Từ PowerShell:

```powershell
Set-Location 'D:\Drone Project\Github Clone UWB Test'
```

### A1–A4: ST-LINK + OpenOCD

Nạp từng board, chuyển dây đúng board trước mỗi lệnh. Ví dụ A1:

```powershell
& '.\Firmware\scripts\flash_highrate.ps1' -Node A1
```

Lặp lại với `-Node A2`, `A3`, `A4`. Script kiểm tra SHA-256 và program/verify image. Dùng `-DryRun` để xem lựa chọn file mà không kết nối probe.

### A5–A8: ST-LINK Utility

1. Kết nối một board STM32, cấp nguồn, BOOT0=0.
2. **File → Open file**: chọn HEX A5/A6/A7/A8 tương ứng trong thư mục này.
3. **Target → Program & Verify**, chờ thành công rồi reset board.
4. Chuyển sang board kế tiếp. HEX chứa địa chỉ flash STM32 `0x08000000`.

### TAG: J-Link tích hợp DWM1001-DEV

Sau khi các anchor đã nạp xong, cắm USB TAG và chạy:

```powershell
& '.\Firmware\scripts\flash_highrate.ps1' -Node TAG
```

Script giữ `DEMCR.TRCENA` khi J-Link thoát bằng `SetDbgPowerDownOnClose=0`, vì firmware dùng DWT cho deadline. Nếu nhiều J-Link đang cắm, thêm `-JLinkSerial <serial>`.

Ngắt/cấp lại nguồn toàn hệ thống sau nạp, mở GUI ở 460800 baud và xác nhận config ID mới. Compile/link và thông báo flash thành công chưa xác nhận ranging. Chưa nạp phần cứng tự động trong lượt sửa này.

## 5. Thu log để nghiệm thu

Chạy hệ thống đủ tám anchor, TAG cách anchor đủ xa để tránh khoảng cách gần 0. Ghi ít nhất **10 phút đứng yên**, sau đó ghi thêm khi di chuyển. Có thể dùng GUI Record hoặc quy trình ROS + UART relay trong `Software/UWB_UART_GUI/ros2/README.md`. Giữ nguyên preset mặc định mới khi ghi lần đầu.

Đánh giá theo timestamp TAG `t_us`, sequence và mode trong RANGE_BURST, không lấy tốc độ redraw GUI làm tần số đo:

- Tần số sinh chu kỳ: `Δcycle_seq / Δt_TAG`.
- Tần số DS mới từng A1–A8: số kết quả DS + RADIO_OK trong khoảng thời gian đó / `Δt_TAG`.
- Tần số bộ đủ tám DS: số chu kỳ có cả tám anchor DS + RADIO_OK / `Δt_TAG`. **Tất cả các tốc độ DS hữu ích này cần ≥200 Hz**.
- Không trùng `boot_id/cycle_seq`; SS/SS_FALLBACK=0. Thống kê mất sequence, FINAL late, cycle overrun, SPI/config/recovery, queue/UART drops và CRC.
- Snapshot vẫn khoảng 5 Hz và chỉ giữ trạng thái mới nhất. Topic ROS `diagnostic_range` dùng burst; topic `/range` còn yêu cầu calibration + VALID.
- Log WSL trước đã có lệch/nhảy clock host. Dùng clock TAG để đánh giá firmware; so sánh rosbag với raw UART để đánh giá đường PC.

Calibration mặc định chưa có: DS raw có thể tốt nhưng CAL_OK/VALID chưa được bật. Không coi số đo chưa calibration là khoảng cách đã nghiệm thu.

Nếu cần so sánh 100 Hz, đóng chương trình đang giữ COM và chạy:

```powershell
python '.\Software\UWB_UART_GUI\uwb_command.py' --port COM14 set-burst --profile hz100
```

Quay lại profile mới:

```powershell
python '.\Software\UWB_UART_GUI\uwb_command.py' --port COM14 set-burst --profile hz220
```

`hz200` là alias của `hz220` trong CLI mới. Preset 100 Hz dùng margin 1200 UUS giống phép thử đã ghi log. Timing chỉ ở RAM; reset TAG trả về mặc định 222 Hz.

## 6. Build lại và kiểm tra

```powershell
& '.\Firmware\tests\run_host_tests.ps1'
& '.\Firmware\scripts\build_highrate.ps1'
python '.\Firmware\tools\verify_highrate_package.py' --json '.\Firmware\release\highrate_200\package_verification_20261002.json'
```

Build riêng vào `build_highrate`, chỉ cập nhật bộ phát hành sau khi đủ chín build thành công. Build một subset được giữ trong thư mục staging để tránh trộn image cũ/mới. Các image trong thư mục build/package thông thường vẫn có thể dùng để khôi phục theo nhận dạng của bản đó.

## 7. Phạm vi xác nhận

Kiểm thử host xác nhận logic và hợp đồng telemetry. Cross build xác nhận mã biên dịch cho MCU, cấu hình FPU/SPI/pinctrl và giới hạn flash/RAM. **Mức ≥200 Hz thực tế, SPI 16 MHz trên dây board STM32 và tỷ lệ đủ tám DS chưa được đo với bản này.** Dùng log mới để quyết định đạt hay còn nút thắt; không báo PASS dựa trên period 4500.
