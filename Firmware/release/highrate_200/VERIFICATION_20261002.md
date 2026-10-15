# Kết quả kiểm tra firmware ngày 2026-10-02

## Trạng thái

- Đã sửa mã chung cho TAG, A1–A4 nRF52832 và A5–A8 STM32F103.
- Đã cross build đủ **9/9 image** với NCS 3.4.0 / Zephyr 4.4.0.
- Đã kiểm tra **9/9 HEX**: checksum Intel HEX, SHA-256 manifest, giới hạn flash, vector SP/reset Thumb, reset handler có trong image, image trùng output build, config ID và lựa chọn SPI/FPU/pinctrl/strict DS.
- Đã chạy `flash_highrate.ps1 -DryRun` cho TAG và A1–A4: chọn đúng HEX, SHA khớp manifest; không mở probe hoặc nạp/reset thiết bị.
- **Chưa xác nhận ≥200 Hz trên phần cứng mới.** Period 4500 µs chỉ là lịch tối thiểu, không phải kết quả đo.

## Cross build và nhận dạng

| Node | Config ID | Flash linker (byte) | RAM linker (byte) | SPI khi chạy |
|---|---|---:|---:|---:|
| TAG | `0x705DE673` | 83.020 / 524.288 | 15.740 / 65.536 | 8 MHz |
| A1 | `0xBF35A8F9` | 47.588 / 524.288 | 8.212 / 65.536 | 8 MHz |
| A2 | `0x4F8DE354` | 47.588 / 524.288 | 8.212 / 65.536 | 8 MHz |
| A3 | `0xC98AE5BD` | 47.572 / 524.288 | 8.212 / 65.536 | 8 MHz |
| A4 | `0x955DD155` | 47.588 / 524.288 | 8.212 / 65.536 | 8 MHz |
| A5 | `0x8A325F47` | 32.908 / 65.536 | 7.724 / 20.480 | 16 MHz |
| A6 | `0x19EA2BBA` | 32.908 / 65.536 | 7.724 / 20.480 | 16 MHz |
| A7 | `0xAF2F135D` | 32.908 / 65.536 | 7.724 / 20.480 | 16 MHz |
| A8 | `0xB979D375` | 32.908 / 65.536 | 7.724 / 20.480 | 16 MHz |

Linker size gồm padding/alignment; số byte thực có trong HEX có thể nhỏ hơn. `.config` và `build.ninja` xác nhận `CONFIG_SPEED_OPTIMIZATIONS=y`, `-O2`, `CONFIG_PINCTRL=y`, strict DS=1. nRF dùng `CONFIG_NRFX_SPIM=y`, FPU + FPU sharing, hard-float ABI; disassembly `DW1000_GetFirstPathPower` có `vmul.f32`, `vadd.f32`, `vdiv.f32`.

SPI Zephyr trên bus DW1000 đã disabled để backend trực tiếp sở hữu bus. Cảnh báo CMake `No SOURCES given ... drivers__spi` phù hợp cấu hình này; linker cuối thành công. Bản build đầu của STM32 thiếu pinctrl khi SPI bus disabled; đã sửa bằng profile chung và thêm BUILD_ASSERT, rồi build lại đủ chín node. Image trong package là từ lần build cuối.

## Kiểm thử host

Lệnh: `Firmware/tests/run_host_tests.ps1`, gcc MinGW `-Wall -Wextra -Werror -Wshadow`, cả profile thông thường và high-rate. **23 nhóm kiểm thử PASS**, gồm 82 kiểm thử Python GUI và 4 kiểm thử HEX validator. Kết quả cuối được ghi tại `host_tests_20261002.txt`.

Các kiểm tra liên quan trực tiếp bản sửa:

- Driver ACK_RESP_T: giữ byte ACK_TIM, đặt/xóa W4R delay đúng; đọc prefix RX từ frame thật, từ chối frame quá ngắn/dài và SPI failure.
- Power diagnostics: 20.000 trường hợp so với `log10f`, sai khác dưới 0,0002 dB; chạy cả nhánh thường và high-rate.
- Anchor v1/v2/v3: transaction, TLV, timeout, delayed TX late, RX error recovery, đổi từ burst sang unicast trả W4R delay về 0.
- A8 cấu hình STM32 thật: rank7/full mask, margin450 UUS, WAIT4RESP, coalesced TXFRS+FINAL, timestamp40 bit qua wrap, Rb ở RESP kế tiếp, TLV dài63 byte cả FCS, mask thưa, FINAL sai txn/timeout, recovery làm mất hiệu lực Rb cũ.
- TAG burst cả thường và strict DS: tám anchor lệch crystal -22…+0,4 ppm, Rb trễ một chu kỳ; thiếu FINAL/FINAL late không sinh DS giả trong bản strict; probe anchor offline và timing/TLV.
- Frame RESP lạ hoặc trùng sau bảy RESP không làm đóng RX trước A8.
- 1.025 chu kỳ mô phỏng liên tiếp ở mỗi biến thể: transaction8 bit wrap nhiều lần, đúng một bản ghi mới/chu kỳ, đủ tám DS. Sai số mô hình dài hạn ≤5 mm, tương đương khoảng một tick timestamp DW1000; bài kiểm tra ngắn giữ ngưỡng2 mm.
- HEX validator: từ chối checksum hỏng, address vượt flash và vector SP/reset không hợp lệ; kiểm tra cả nRF và STM32.
- Command/settings, telemetry golden C↔Python, bố cục project, GUI decoder/recorder/filter, gateway parser và motion tests hiện có.

Motion suite MEDIAN_GATE và CV_KALMAN đều đạt 14/14 kịch bản mô phỏng. LEGACY chạy ở chế độ `--report` để mô tả các giới hạn đã có, không phải tiêu chí nghiệm thu profile mới. Timing được in ở motion test là mô hình sequential; không dùng nó để suy ra tần số burst trên MCU.

## Tiêu chí kiểm tra phần cứng tiếp theo

Nạp đủ chín image mới, khởi động lại toàn hệ thống, thu ít nhất 10 phút ở mặc định 222 Hz, rồi ghi thêm khi TAG di chuyển. Trong cùng khoảng thời gian clock TAG:

1. A1–A8: DS + RADIO_OK mới ≥200 kết quả/s mỗi anchor.
2. Bộ đầy đủ tám DS mới ≥200 bộ/s.
3. SS/SS_FALLBACK=0; không trùng boot/cycle sequence.
4. Thống kê timeout, FINAL late, overrun, CRC, mất sequence và queue/UART overflow; đối chiếu DEVICE_INFO/ANCHOR_INFO với bảng ID trên.

Snapshot 5 Hz và topic ROS sử dụng clock host không thay thế các tiêu chí này. Kết quả RAW DS chưa có calibration cần được tách khỏi kết quả CAL_OK/VALID.
