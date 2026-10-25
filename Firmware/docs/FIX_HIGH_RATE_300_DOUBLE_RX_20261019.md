# Sửa mất RESP và đường nối ngang — vòng 2, ngày 19/10/2026

Bản ghi vòng 2 (TAG `0x84e6c9b9`, log thực tế khoảng271 cycle/s).
Gói phát hành hiện tại và cách nạp ở [bản pipeline vòng 3](FIX_HIGH_RATE_300_PIPELINE_20261019.md).

## Căn cứ

Log `Software/UWB_UART_GUI/Output/data_logs/20261019_132235_COM14` dùng TAG
`0x01e8d392`, slot275/margin450 UUS, A3 không cấp nguồn. FINAL late=0 nhưng
DS/s theo clock TAG của A1/A2/A4/A5/A6/A7/A8 lần lượt là
315,46 / 213,93 / 46,17 / 118,36 / 183,17 / 101,27 / 187,12.
Các mẫu còn CAL_MISSING; đây là DS radio, không phải range đã hiệu chuẩn.
UART giao được 99,95% bản ghi, queue drop=0, RX service high-water=177 us.

A8 không có mẫu từ elapsed 0,409056 đến 0,540058 s, hai đầu cùng 1260 mm.
Trong đó A1 vẫn có 41 DS. GUI cũ nối qua gap ≤500 ms nên tạo đường ngang.
Điều này không chứng minh A8 mất nguồn/treo. Xem
[audit](../release/highrate_300/log_audit_20261019_132235.json).

## Firmware

Chỉ profile burst TAG highrate_300 bật hai bộ đệm RX. RXAUTR vẫn tắt;
radio được bật nhận lại bằng lệnh thủ công theo trình tự driver Decawave:

1. Đọc carrier integrator trước khi rearm vì DRX_CAR_INT không được double buffer.
2. Xóa cờ good-RX hiện tại rồi bật RX vào buffer còn lại ngay.
3. Giữ host pointer để đọc payload/FINFO/timestamp/quality của RESP hiện tại,
   kể cả khi RESP sau đã tới. Xác thực ID/mask/version/txn/TLV trước khi dùng.
4. Đọc xong mới toggle HRBPT đúng một lần, kể cả frame bị loại. Không xóa cờ
   RX sau rearm vì cờ RESP kế tiếp có thể đã set. SPI lỗi không được xuất range.
5. Lỗi RX: dừng radio, reset RX, đồng bộ pointer, rồi nhận lại. Đồng bộ pointer
   đầu cycle chỉ khi RX đã dừng; không đổi host pointer khi đang đọc metadata.
6. Khe cuối vẫn arm FINAL trước diagnostics, độc lập số RESP nhận được.
   Chuyển về sequential tắt double buffer; radio recovery khởi tạo lại mode.

Giữ PHY64/v3, base450/slot275/margin450 UUS, gap180 us, period floor3100 us,
info extra64 UUS để đo tác dụng nhận sớm trên cùng lịch của log. Không tăng
SPI, đổi toán DS, bật calibration mask hoặc tắt backoff để che lỗi.

Tham chiếu: [Decawave API Guide, dwt_setdblrxbuffmode](https://forum.qorvo.com/uploads/default/original/1X/8b220e1e26fea4ebd83f0b0e5ef42eb9a251310d.pdf)
và [driver chính thức, dwt_isr/dwt_rxenable/dwt_syncrxbufptrs](https://github.com/Decawave/dwm1001-examples/blob/master/deca_driver/deca_device.c).

## GUI

Tab RANGE_MEAS ngắt đường nếu khoảng thiếu mẫu vượt ba khoảng lấy mẫu thông
thường, ngưỡng tối thiểu20 ms/tối đa500 ms. Cadence dùng quartile thấp của
dữ liệu gốc trước decimation để dropout không làm tăng ngưỡng. Gap131 ms
trong log hiện thành khoảng trống.

Đường ngắt được truyền qua decimation: xem 60 s không nối lại qua dropout
hoặc tạo gap giả do chỉ vẽ mỗi vài mẫu. Trục X theo clock TAG chung, kể cả
burst không có DS, nên dữ liệu anchor im lặng trôi trái và để trống bên phải.
CSV/raw và thống kê vẫn dùng toàn bộ dữ liệu gốc.

EXE riêng: `Software/UWB_UART_GUI/Output/DWM1001_UWB_Ground_Control_rx_gap_fix.exe`.
Chạy nguồn: `python Software/UWB_UART_GUI/uwb_uart_gui.py`.

## Kiểm thử

- 25 nhóm host test qua, gồm driver, TAG/anchor, v2/v3, motion, golden C/Python,
  parser gateway, package checks và GUI. GUI: 86 test, 2 skipped theo môi trường;
  test tracker chạy lại sau bổ sung giới hạn gap500 ms.
- Simulator mô hình hóa hai buffer/pointer host–IC. RESP thứ hai tới trước
  khi đọc payload/metadata RESP thứ nhất: payload, timestamp, quality phải
  vẫn thuộc đúng anchor và event thứ hai phải còn để xử lý.
- TAG chạy nhiều cycle chồng RESP, A3 tắt, cả backoff/probe; bảy anchor còn
  lại có DS, sai lệch mô phỏng ≤5 mm và FINAL late=0.
- Frame sai/trùng/stale, RX overrun/recovery, missing FINAL, info TLV, txn/clock
  wrap và chuyển sequential được kiểm tra. GUI kiểm Canvas và replay log thật:
  A1 liên tục, A2/A4–A8 vẫn có khoảng trống sau decimation.
- Build 9/9 HEX qua kiểm tra hash/config/vector/giới hạn bộ nhớ; EXE chạy demo
  ẩn và tự thoát với mã0. Biên nhận:
  [validation_double_rx_20261019.json](../release/highrate_300/validation_double_rx_20261019.json).

Mô phỏng không xác nhận WCET MCU hay đủ 300 DS/s khi cả tám board bật.
Phải đo lại trên board, cả sau hiệu chuẩn vì filter/telemetry cũng tốn CPU.
Không dùng tần số cycle làm DS throughput.

## Nạp và đo lại

TAG mới **config `0x84e6c9b9`**. HEX ở `release/highrate_300`; bộ vòng trước
được giữ tại `previous_20261019_before_double_rx/`, TAG `0x01e8d392`.
Các anchor đang nạp bộ300/PHY64 có thể giữ nguyên để thử TAG:

```powershell
& 'D:\Drone Project\Github Clone UWB Test\Firmware\scripts\flash_highrate.ps1' -Node TAG -Rate 300
```

Đóng GUI/relay giữ COM trước khi nạp. Dùng GUI mới, kiểm DEVICE_INFO đúng config
TAG mới, giữ A3 tắt và ghi ≥60 s. So sánh DS/s, max gap, timeout/backoff từng
anchor, FINAL late, RX errors, queue drop và UART CRC. Sau đó thử đủ tám anchor
và phiên dài hơn. Chưa tự nạp bất kỳ board nào trong lượt sửa này.
