# Firmware DS-TWR mục tiêu 200 Hz — thử nghiệm

Bộ này dành cho TAG DWM1001-DEV, A1–A4 DWM1001C/nRF52832 và A5–A8 STM32F103 + DW1000. **200 Hz là mục tiêu phát triển, chưa được xác nhận trên phần cứng.** Không dùng số cấu hình làm bằng chứng đạt tần số.

## Sửa lỗi khởi động SPIM ngày 2026-09-30

Người dùng đã nạp TAG bản đầu và báo đèn đỏ chớp, không ranging. Log J-Link xác nhận nội dung flash khớp HEX. Khi đọc lại SDK và cấu hình build, tìm thấy lỗi xác định trong bản đầu: overlay tắt `spi2`, `CONFIG_NRFX_SPIM` cũng tắt, khiến `pinctrl_nrf.c` không biên dịch các nhánh `NRF_FUN_SPIM_*`. `pinctrl_apply_state()` trả `-ENOTSUP`; `uwb_platform_init()` thoát lỗi và TAG vào `platform_fault_forever()` trước khi khởi tạo UART. Điều này phù hợp với triệu chứng chớp đỏ, độc lập với lịch ranging/anchor.

Đã thêm `profiles/highrate_nrf.conf` với `CONFIG_NRFX_SPIM=y` để giữ hỗ trợ pinctrl, trong khi bus vẫn disabled và ứng dụng trực tiếp sở hữu SPIM2. `BUILD_ASSERT` chặn build cấu hình thiếu hỗ trợ này. TAG và A1–A4 được build lại; A5–A8 STM32 không dùng backend này. Các HEX nRF cũ được lưu trong `previous_20260930_spim_init_fault` để truy nguyên, **không nạp các file trong thư mục đó**.

Bản sửa cần được người dùng nạp và ghi log lại; compile/link thành công chưa xác nhận chạy trên phần cứng.

### Trạng thái sau nạp và bộ đếm DWT

Sau bản sửa pinctrl, người dùng ban đầu báo TAG không đỏ nhưng chưa thấy data; sau rút/cắm lại USB thì TAG và A1 kết nối. Agent không nạp firmware trong lượt chẩn đoán này: chỉ chạy J-Link đọc thanh ghi/RAM. Lần đọc ghi nhận `s_initialized=1`, lỗi SPI bằng 0, `tag_cycle_count=0x2832` và `response_ok_count=0x59A5`. Đây là snapshot bộ đếm, chưa phải phép đo tốc độ hay xác nhận đầy đủ tám anchor. Chưa đọc lại được flash để đối chiếu ELF vì probe không còn kết nối ở lần đọc tiếp theo.

[SEGGER xác nhận](https://kb.segger.com/J-Link_Cortex-M_application_uses_cycle_counter) J-Link mặc định xóa `DEMCR.TRCENA` khi đóng phiên, làm dừng bộ đếm DWT mà firmware dùng cho deadline. Đây là cơ chế phù hợp với việc khởi động lại giúp hệ thống chạy; chưa có phép đo trước/sau đóng phiên để chứng minh đó là nguyên nhân duy nhất của lần lỗi vừa báo. Script nạp TAG đã thêm `exec SetDbgPowerDownOnClose = 0` để giữ DWT sau khi J-Link thoát. Thay đổi này chỉ thuộc script, không thay HEX hay checksum. Sau nạp các anchor, ngắt/cấp lại nguồn toàn bộ rồi ghi log ở 460800 baud để xác nhận khả năng tự khởi động và ranging.

## Những phần đã kết hợp

- Giữ burst v3: một POLL, tám RESP, một FINAL; Rb của chu kỳ N nằm trong RESP của N+1. Mỗi bộ khoảng cách có độ trễ một chu kỳ.
- Áp dụng cách truy cập SPI trực tiếp từ hướng thiết kế của `Reference/dw1000/bdstwr`: nRF52832 dùng SPIM2 EasyDMA, polling END, 2 MHz khi khởi tạo và 8 MHz khi hoạt động. Zephyr không đăng ký driver trên bus này; GPIO, UART, watchdog và settings vẫn dùng Zephyr.
- Gộp RX_FINFO/RX_TIME/RX_FQUAL; chụp đủ frame, timestamp và chất lượng trước khi bật lại RX. Giữ cả carrier integrator, FPP/RX power và TLV nhận dạng anchor.
- Công thức asymmetric DS-TWR dùng tích số nguyên 64 bit trước phép chia. Reject interval bằng 0 hoặc ≥ 2³¹ DTU để tránh tính từ trao đổi cũ và tràn số.
- Thiếu FINAL/Rb: xuất NONE + TIMEOUT; không tạo khoảng cách SS fallback. Chính sách này áp dụng cả khi chuyển sang sequential trên image thử nghiệm.
- Giữ calibration theo thiết bị/PHY, xuất raw diagnostic khi CAL_MISSING, bộ lọc MEDIAN_GATE, commands, NVS, recovery, watchdog, build/config identity và GUI logging.
- Đọc nhiệt độ/điện áp thành hai bước, không sleep 1 ms trong vòng ranging. Telemetry nền đợi đủ thời gian trước chu kỳ kế tiếp; kiểm tra cấu hình định kỳ và commands vẫn được phục vụ giữa chu kỳ để giữ recovery và khả năng đổi timing khi profile bị quá tải.
- GUI đọc thêm diagnostic hiệu năng; bộ lọc host chấp nhận khoảng thời gian 4–5 ms thay vì ép mẫu 200 Hz thành 10 ms. Tab Live hiện vẫn dùng snapshot 5 Hz; log RANGE_BURST/RANGE_MEAS giữ tốc độ phép đo thực tế.

Đây là tích hợp ý tưởng vào firmware hiện tại. Packet, địa chỉ, PRF và driver của source tham khảo không được chép nguyên khối: firmware hiện tại giữ Ch5/PRF16/preamble128/6.8 Mbps và địa chỉ TAG=0, anchor=1…8. Source tham khảo sử dụng PRF64 và địa chỉ khác nên không thể chạy trực tiếp cùng các image này.

## Profile mặc định của bộ thử nghiệm

| Tham số | Giá trị |
|---|---:|
| base | 600 UUS |
| slot | 350 UUS |
| FINAL margin | 400 UUS |
| gap | 300 µs |
| min POLL period | 4800 µs |
| Tần số danh định | 208,33 chu kỳ/s |
| Số anchor | 8 |
| FINAL tại | 3450 UUS sau POLL RMARKER |

1 UUS ≈ 1,026 µs. Chu kỳ có TLV mỗi giây mở rộng slot thêm 64 UUS, làm FINAL muộn thêm 448 UUS khi có đủ tám anchor. CPU, IRQ, tính khoảng cách, UART và kiểm tra cấu hình vẫn có thể làm chu kỳ dài hơn 4800 µs. Chọn danh định 208 Hz để có dự phòng cho mục tiêu **ít nhất 200 bộ DS/s**, tương đương ít nhất 1600 khoảng cách DS/s.

UART giữ 460800 baud. RANGE_BURST khoảng 102 byte × 208,33 Hz ≈ 21,25 kB/s, chưa tính telemetry nền; giới hạn UART 8N1 khoảng 46,08 kB/s. J-Link VCOM của board đã gặp CRC lỗi ở 1 Mbaud trong các lần đo trước nên không tăng baud ở bộ này.

STM32 giữ SPI Zephyr 8 MHz. Trên cấu hình thực tế STM32 là A5–A8, có thời gian xử lý POLL dài hơn A1. Backend trực tiếp hiện chỉ triển khai cho nRF52832, là nút thắt thấy trong log TAG; hiệu năng STM32 vẫn phải đo trên board.

## Build và image

Từ thư mục repository, PowerShell:

```powershell
& '.\Firmware\scripts\build_highrate.ps1'
```

Build riêng trong `build_highrate` của từng node, đóng gói tại thư mục này. Script không nạp firmware. `manifest.csv` ghi SHA-256 từng HEX. Image stable ở `anchor_8_dist`, `stm32_anchor/dist` và các thư mục `build` hiện có vẫn giữ nguyên.

| Image | Board / địa chỉ |
|---|---|
| TAG_DWM1001_DEV.hex | TAG DWM1001-DEV, 0 |
| A1_DWM1001C_nRF52832.hex … A4_DWM1001C_nRF52832.hex | DWM1001C, 1…4 |
| A5_STM32F103C8.hex … A8_STM32F103C8.hex | STM32F103, 5…8 |

Nạp image tương ứng đúng board. Các anchor phải có responder v3 và bản sửa timeout FINAL theo lịch POLL. Bộ thử nghiệm đi cùng đủ tám image để loại trừ tình trạng firmware khác phiên bản. TAG chưa được nạp tự động.

## Nạp trực tiếp các file HEX

HEX là firmware đã build; không cần chuyển thành BIN hoặc build lại. **Script `flash.ps1` cũ trong từng project chọn thư mục `build`, không chọn bộ HEX ở đây.**

Đóng GUI và phiên debug trước khi nạp. Mở PowerShell:

```powershell
Set-Location 'D:\Drone Project\Github Clone UWB Test'
```

### TAG DWM1001-DEV: J-Link tích hợp trên board

Cắm USB của TAG vào máy tính. Chạy:

```powershell
& '.\Firmware\scripts\flash_highrate.ps1' -Node TAG
```

Script chọn `TAG_DWM1001_DEV.hex`, kiểm tra SHA-256 theo manifest và dùng J-Link Commander để nạp/verify, reset rồi chạy chương trình. Nếu có nhiều J-Link, thêm `-JLinkSerial <serial>` để chọn probe của TAG. Chỉ kết nối probe cần nạp khi chưa chỉ định serial.

### A1–A4 DWM1001C/nRF52832: ST-LINK + OpenOCD

Kết nối ST-LINK qua SWD như cách nạp anchor hiện tại, cấp nguồn board. Nạp **từng board riêng**, chọn đúng địa chỉ vật lý của anchor. Ví dụ với board A1:

```powershell
& '.\Firmware\scripts\flash_highrate.ps1' -Node A1
```

Sau khi hoàn tất, chuyển dây sang A2 và chạy với `-Node A2`; lặp lại cho A3, A4. Không chạy cả bốn lệnh liên tiếp trên cùng board. Script dùng cấu hình ST-LINK/nRF52 hiện có, nạp HEX tương ứng rồi verify/reset. ST-LINK Utility dành cho STM32 không dùng để nạp các anchor nRF52832 này.

Thêm `-DryRun` để xem file, checksum và công cụ được chọn **mà không kết nối/nạp phần cứng**. Có thể chỉ định đường dẫn cài đặt bằng `-JLinkRoot`, `-OpenOcdPath`, `-OpenOcdScripts` nếu công cụ không được tìm thấy. Script không yêu cầu SDK/west, không gọi mass erase hoặc unlock.

### A5–A8 STM32F103C8: ST-LINK Utility

1. Kết nối ST-LINK với một board STM32, cấp nguồn, giữ BOOT0 = 0.
2. Trong ST-LINK Utility chọn **File → Open file**, mở HEX tương ứng tại thư mục này: `A5_STM32F103C8.hex`, `A6_STM32F103C8.hex`, `A7_STM32F103C8.hex` hoặc `A8_STM32F103C8.hex`.
3. Chọn **Target → Program & Verify**, bắt đầu nạp, chờ báo thành công, reset board.
4. Lặp lại với board tiếp theo và đúng file địa chỉ của board đó. HEX đã chứa địa chỉ flash; không nhập địa chỉ dành cho nRF52832.

Log `20260930_181129_COM14` đã nhận diện A5–A8 đúng bộ thử nghiệm, còn TAG và A1–A4 thuộc bộ thường. Với trạng thái của log đó, cần nạp lại TAG và A1–A4 trước.

Sau nạp bản sửa SPIM, TAG thử nghiệm phải có config ID **`0x9D889DBE`**, UART **460800 baud**. ID `0x0FE6534E` thuộc bản đầu bị lỗi khởi động. Ghi log mới để kiểm tra tần số và tỷ lệ đủ tám khoảng cách DS; nạp thành công chưa xác nhận đạt 200 Hz.

Nếu cần khôi phục TAG về image 50 Hz đã dùng trong log `20260930_181129_COM14`, chạy từ root repository:

```powershell
& '.\Firmware\Tag_DevKit\scripts\flash.ps1'
```

Script này dùng `Tag_DevKit/build/zephyr/zephyr.hex`, config ID `0x771A7D24`. Build thường này được giữ nguyên khi build bản thử nghiệm. Chỉ TAG được khôi phục bằng lệnh trên.

Tài liệu lệnh nạp: [SEGGER J-Link Commander](https://kb.segger.com/J-Link_Commander), [OpenOCD Flash Programming](https://openocd.org/doc/html/Flash-Programming.html).

## Điều chỉnh khi chạy bench

Đóng GUI trước khi CLI mở cùng COM. Ví dụ COM14, chạy từ `Software/UWB_UART_GUI`:

```powershell
python .\uwb_command.py --port COM14 set-burst --profile stable50
python .\uwb_command.py --port COM14 set-burst --profile hz100
python .\uwb_command.py --port COM14 set-burst --profile hz200
```

Các lệnh đổi timing trong RAM. Reset TAG thử nghiệm trả về profile 4800 µs; `stable50` chỉ đổi timing, vẫn giữ chính sách DS-only và backend của image thử nghiệm. Không áp dụng profile hz200 vào TAG stable rồi kết luận phần cứng đạt 200 Hz. NVS hiện có vẫn có thể giữ active mask/telemetry cũ: kiểm tra mask FF và bật measurement + diag khi ghi log.

## Bằng chứng cần lấy trên phần cứng

1. Bắt đầu 50 Hz, sau đó 100 Hz và profile hz200. Ghi ít nhất 10 phút mỗi profile; kiểm tra cả khi di chuyển tag.
2. Trong `burst.csv`, dùng `t_us`/`period_us` của thiết bị: tần số bộ tám anchor DS hoàn chỉnh và tần số DS riêng từng anchor đều ≥ 200 Hz. Không dùng tốc độ vẽ GUI hoặc số RANGE records tổng để kết luận.
3. Mẫu không đầy đủ phải giữ NONE/invalid. SS_FALLBACK phải bằng 0. Phân biệt RADIO_OK DS với CAL_OK/VALID; calibration thiếu có thể cho raw DS nhưng chưa cho khoảng cách production.
4. Trong `diag_system.csv`, kiểm tra `spi_backend=1`, `strict_ds=True`, `burst_rx_service_max_us`, `burst_publish_max_us`, `cycle_overruns`, `burst_final_late`, SPI/config/recovery errors và queue/UART drops.
5. `burst_rx_service_max_us` đo đọc snapshot + CI + rearm, chưa gồm đọc SYS_STATUS và độ trễ vào main loop. `burst_publish_max_us` đo tính/publish tám anchor sau FINAL, chưa gồm đóng gói UART nền. Vì vậy không cộng hai số rồi suy ra tần số hệ thống.
6. Khi có đủ tám anchor, slot 350 UUS ≈ 359 µs; RX phải kịp bật trước preamble RESP tiếp theo, nên ngân sách đọc thực tế nhỏ hơn khoảng slot. Nếu RX max cao hoặc FINAL late tăng, nới slot/margin và xác định nút thắt bằng oscilloscope/logic analyzer.

Profile không bảo đảm mọi mẫu đều thành công. Nếu chạy danh định 208,33 Hz mà mất trên khoảng 4% DS, tốc độ DS hữu ích có thể dưới 200 Hz. Nếu STM32 xử lý FINAL/POLL hoặc TAG tính toán không kịp, cần tối ưu tiếp dựa trên log và tín hiệu SPI/IRQ thực tế. Chưa có dữ liệu để kết luận phải thay DW1000/MCU; cũng chưa có bằng chứng để bảo đảm phần cứng hiện tại đạt mục tiêu.

## Phạm vi xác nhận

Ngày 2026-09-30 đã compile/link thành công đủ **TAG + 8 anchor** bằng NCS 3.4.0 / Zephyr 4.4.0. Sau sửa pinctrl, TAG high-rate dùng 61.288 byte flash và 15.524 byte RAM; các anchor STM32 dùng 27.696 byte flash và 7.820 byte RAM, nằm trong 64 KiB flash / 20 KiB RAM. TAG dùng backend SPI thông thường cũng compile/link được trong `Tag_DevKit/build_compat`. Các file Python đã qua kiểm tra cú pháp bằng `py_compile`.

Zephyr báo `No SOURCES given to ... drivers__spi` ở các build nRF high-rate do bus SPI Zephyr được tắt để SPIM chỉ có một chủ sở hữu; cảnh báo này có thể tồn tại cả khi pinctrl đã được sửa. NCS cũng báo giới hạn hỗ trợ board STM32 ngoài Nordic. Chưa chạy test suite. Người dùng đã nạp bản đầu và phát hiện lỗi khởi động nêu trên; agent chưa nạp phần cứng, bản sửa chưa được đo trên phần cứng.

Build thành công chỉ xác nhận compile/link và kích thước image phù hợp bộ nhớ. Mốc 200 Hz, chất lượng khoảng cách, CRC đường UART và hành vi recovery cần xác nhận trên phần cứng. Bộ mới hiện được cung cấp để bench; chưa có xác nhận 200 Hz thực tế.
