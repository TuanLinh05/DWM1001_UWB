# Source review — 2026-09-12

Phạm vi: firmware đang sử dụng trong `Tag`, `Anchor_1`, `ESP32C3_Gateway`
và phần mềm `Software/UWB_UART_GUI`. Không sửa template `Firmware Code Base`
hoặc firmware STM32 cũ. Đây là bản sửa lỗi và tăng kiểm tra đầu vào; không phải
xác nhận hệ thống đã được kiểm nghiệm bay hoặc mọi cấu hình phần cứng đều hoạt động.

## Các lỗi đã sửa

| Phần | Lỗi / hạn chế tìm thấy | Thay đổi |
|---|---|---|
| Tag | Timeout phát POLL có thể giữ `valid` của lần đo trước | Xóa tính hợp lệ, ghi TIMEOUT và tăng counter |
| Tag | Kiểm tra TX timeout trước IRQ đã đến | Xử lý IRQ TX trước; xóa IRQ không liên quan để deadline vẫn hoạt động |
| Tag / Anchor | Frame lạ liên tục có thể kéo dài trạng thái chờ | Kiểm tra deadline gốc sau khi xử lý frame lạ |
| UWB RX | Chỉ kiểm tra một phần header, dữ liệu dài bị cắt thành frame ngắn | Kiểm tra FC, PAN, độ dài chính xác, nguồn TAG ở Anchor; từ chối truncation |
| DW1000 | Kết quả SPI bị bỏ qua, buffer đọc lỗi có thể chứa dữ liệu không xác định | Đếm `dw1000_spi_error_count`, đưa buffer đọc lỗi về 0; dừng vòng ranging trước khi gửi telemetry nếu phát sinh lỗi SPI |
| Khởi động | Có hàm verify nhưng chưa gọi | Cả hai node kiểm tra lại cấu hình DW1000 sau khi chuyển tốc độ SPI |
| Timing | `k_cycle_get_32()` dùng RTC 32768 Hz, lượng tử hóa khoảng 30,5 µs | Dùng architecture timing API của Zephyr trên DWT; không gọi SoC timing API chiếm TIMER2 |
| Range filter | Candidate reacquire còn tồn tại qua input lỗi | Xóa candidate khi input không hợp lệ |
| ESP parser | CRC/length lỗi có thể nuốt gói hợp lệ tiếp theo | Quét lại suffix của buffer, không dùng đệ quy |
| ESP diagnostics | Counter nội bộ có thể bị optimizer loại bỏ | Giữ counter USB dạng volatile để xem debugger |
| GUI | Giá trị cũ vẫn trông như dữ liệu hiện tại khi stream dừng | Đánh dấu STALE sau 1 giây không có RANGE / khi ngắt kết nối |
| GUI | Reconnect, reboot và thay tập Anchor giữ dữ liệu phiên cũ | Reset trạng thái phiên, xử lý sequence quay lại, loại hàng không còn trong snapshot |
| GUI decoder | Chấp nhận schema/count/validity bất nhất | Từ chối INFO schema không hỗ trợ, count lệch, ID trùng hoặc valid kèm lỗi |

## Kiểm tra

- Build nRF52832 bằng NCS v3.4.0 / Zephyr 4.4.0, board `decawave_dwm1001_dev/nrf52832`.
- Host test C dùng driver và state machine Tag thật với SPI/time mô phỏng: lỗi SPI,
  truncation, stale-valid sau timeout TX, ưu tiên IRQ, deadline dưới traffic lạ.
- Host test gateway: CRC, frame phân mảnh, frame 4 Anchor, phục hồi gói hợp lệ
  bị nằm bên trong candidate có length/CRC hỏng.
- 9 test Python, gồm decoder và trạng thái widget khi stale/reboot.
- Kiểm tra cú pháp với `-Wall -Wextra -Werror` cho 7 tổ hợp filter/adaptive/motion
  hợp lệ. Đây không phải kiểm nghiệm chất lượng lọc với dữ liệu đo thật.

Chạy lại test firmware từ thư mục `Firmware`: `./tests/run_host_tests.ps1`.
Test gateway: `./ESP32C3_Gateway/tests/run_host_test.ps1`.
Test GUI từ thư mục GUI: `py -3.12 -m unittest discover -s tests -v`.

## Giới hạn và bước kiểm chứng trên board

UART Tag vẫn là 115200 8N1, binary CRC. Gói 4 Anchor là 81 byte; ở 50 chu kỳ/s,
riêng RANGE dùng 4050 byte/s so với trần 11520 byte/s của UART 8N1. Đây là tính
băng thông lý thuyết, không phải kết quả đo jitter trên board. UART ISR và SPI
cùng chạy; chưa có BLE trong cấu hình hiện tại (`CONFIG_BT` không bật), vì vậy
không thể kết luận hiệu năng khi bổ sung BLE chỉ từ các bài test này.

Nếu xuất hiện lỗi SPI lúc chạy, firmware vào fatal blink và ngừng xuất dữ liệu;
GUI sẽ chuyển STALE. Cần kiểm tra bus/nguồn, xem `dw1000_spi_error_count` rồi reset
board. Bản này chưa tự phục hồi hoàn toàn DW1000 và filter sau lỗi bus.

Giữ nguyên DS-TWR, PHY, antenna delay và calibration mask hiện tại. `CAL_MISSING`
vẫn có raw distance để hiệu chỉnh; không tự biến số chưa calibration thành valid.
Các ngưỡng `TUNE_REQUIRED` và filter SHADOW/ACTIVE cần dữ liệu nhiều khoảng cách,
chuyển động và NLOS để đánh giá. Giao thức hiện phục vụ một TAG, chưa bảo đảm
gán transaction bằng sequence cho hệ thống nhiều TAG.

Chưa build/flash ESP32-C3 do máy chưa có ESP-IDF. Test parser C không xác nhận
USB driver/Kconfig hay đường UART→ESP→USB trên phần cứng. Chưa nạp firmware nRF
mới hoặc đo thực tế trong lần rà soát này.

Sau khi nạp đúng firmware cho từng board, theo dõi `distance_a1_mm`,
`response_ok_count`, `anchor_response_timeout_count[0]`,
`anchor_poll_tx_timeout_count[0]`, `dw1000_spi_error_count`,
`tag_cycle_duration_max_us` và `uart_tx_overflow_count` trên Tag. Tắt/bật Anchor,
ngắt/kết nối GUI và cho chạy đủ lâu để kiểm tra wrap bộ đếm DWT (~67 giây).

Bản sao tạm và output của công cụ review không thuộc source project và không
được commit vào repository.
