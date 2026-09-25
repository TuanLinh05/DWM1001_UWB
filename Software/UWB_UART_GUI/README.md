# DWM1001 UWB Ground Control

GUI Windows nhận telemetry binary của firmware `Tag`, kiểm tra CRC và hiển thị:

- raw/filtered distance, valid, status, age và FPP của A1..A8;
- từng phép đo `RANGE_MEAS`: tần số đo thật, bản ghi mất, nhiễu, FP/RX, chỉ báo
  NLOS, clock offset và thời gian slot của mỗi anchor;
- INFO của firmware và calibration mask;
- poll/OK/timeout/RX error/overrun/UART overflow;
- tốc độ UART, CRC error, byte bị bỏ qua và sequence bị mất;
- đồ thị Raw, Host Filter và Firmware Filter đồng thời theo từng Anchor;
- Host Filter dùng mô hình khoảng cách-vận tốc từ `CV_KALMAN_V2` trong
  `STM32_UWB`: median-3 + alpha-beta, giới hạn tốc độ vật lý nhưng không giới
  hạn khoảng cách; FPP yếu chỉ giảm nhẹ gain và không còn snap khi reacquire
  gây đầu ra dạng bậc, và vẫn hoạt động cho dữ liệu diagnostic `CAL_MISSING`;
- ghi một phiên đo mà không chặn luồng UART/GUI;
- lưu CSV khoảng cách và thống kê, INFO dạng JSONL, sự kiện dạng text và toàn
  bộ frame hợp lệ dạng binary để phân tích hoặc replay sau này.

## Các tab nâng cấp

### Live

Giữ bảng A1..A8, recorder và sự kiện UART trong một trang riêng để dễ đọc.
Giao thức hiện tại vẫn có thể chỉ gửi A1..A4; các dòng A5..A8 là phần chuẩn bị
cho phiên bản firmware 8 anchor.

### Biểu đồ lớn

Đồ thị Raw / Host Filter / Firmware Filter được tách khỏi tab Live và chiếm toàn
bộ vùng nội dung của tab. Có thể chọn A1..A8 và xóa riêng lịch sử của anchor
đang xem. Trục Y tự scale theo tối đa 300 mẫu gần nhất.

### RANGE_MEAS

Snapshot 50 Hz ở tab Live chỉ giữ giá trị cuối của mỗi anchor. `RANGE_MEAS`
(TYPE 0x10) là một bản ghi cho **mỗi** phép đo thành công, nên tab này cho thấy
chuyện gì xảy ra bên trong chu kỳ:

| Cột | Ý nghĩa |
|---|---|
| Hz | Số phép đo thành công mỗi giây của anchor (2 s gần nhất) |
| Mode / Status | DS, SS hoặc SS_FALLBACK; cờ trạng thái của lần đo |
| Raw / Corrected / FW Filter | Raw trước offset; Corrected chỉ có khi anchor đã calibration; FW Filter chỉ có khi bộ lọc TAG nhận mẫu |
| Nhiễu (mm) | Độ lệch chuẩn của hiệu hai raw liên tiếp / √2 trong 2 s; vẫn đúng khi TAG di chuyển chậm |
| FP / RX (dBm) | Công suất first path và tổng của RESP tại TAG |
| NLOS Δ (dB) | RX − FP trung bình 2 s: < 6 dB thường LOS, > 10 dB thường NLOS (APS006) |
| Anchor Δ (dB) | RX − FP của FINAL đo tại anchor (DS-TWR REPORT) |
| CI (ppm) | Clock offset anchor so với TAG từ carrier integrator |
| Noise / Slot / Age | `std_noise` của DW1000, thời gian slot (µs), tuổi bản ghi cuối |

Màu dòng: xanh = tốt, vàng = `CAL_MISSING`, cam = NLOS Δ ≥ 6 dB, fallback hoặc
reject, đỏ = NLOS Δ ≥ 10 dB, xám = hơn 1 s không có bản ghi. Dòng tổng hợp báo
tổng meas/s, số bản ghi mất (khe hở `meas_seq`), `meas_queue_drops` và UART TX
overflow của TAG (từ DIAG). Biểu đồ bên dưới vẽ từng phép đo của một anchor theo
đồng hồ TAG (`meas_time_us`) trong 5–60 s: khoảng cách Raw/Corrected/FW Filter
ở trên và NLOS Δ với ngưỡng 6/10 dB ở dưới.

Khi kết nối, nếu `DEVICE_INFO` báo UART ≥ 460800 baud mà RANGE_MEAS đang tắt,
GUI tự bật cho phiên (bỏ chọn **Tự bật khi kết nối** để tắt hành vi này). Nút
**Bật/Tắt RANGE_MEAS** gửi `SET_TELEMETRY` nhưng luôn giữ snapshot cho tab Live;
thay đổi chỉ có hiệu lực tới khi TAG reboot, trừ khi lưu bằng
`uwb_command.py ... set-telemetry --snapshot --meas --diag --save`. Ở 115200 baud
firmware từ chối RANGE_MEAS nên nút bị khóa.

### Bản đồ 2D / 3D

- nhập tọa độ tâm anten X/Y/Z theo hệ ENU, đơn vị mét;
- preset 4 anchor phẳng và 8 anchor dạng hộp 5 x 4 x 3 m;
- lưu cấu hình mặc định vào `anchor_layout.json`, nhập/xuất JSON để đi cùng log;
- solver robust 2D cần ít nhất 3 range `valid`, solver 3D cần ít nhất 4;
- vẽ range, trail, RMS, residual lớn nhất, anchor được dùng/loại và chỉ báo hình
  học;
- phép đo `CAL_MISSING` chỉ dùng cho plot/calibration, tuyệt đối không đi vào
  nghiệm position.

### Phân tích

Tab này dùng **phiên lấy mẫu**, không còn cập nhật đồ thị realtime. Chọn anchor,
nguồn Raw/Host Filter/Firmware Filter, nhập số mẫu (mặc định 3.000) và nhấn
**Bắt đầu lấy mẫu**. GUI chỉ cập nhật thanh tiến độ; khi đủ số mẫu hợp lệ, phiên
đo tự đóng băng rồi mới tính và vẽ dashboard khoa học 2 x 2:

- chuỗi thời gian của cả ba nguồn, đường mean/reference và FPP trên trục phải;
- histogram mật độ với đường phân bố chuẩn khớp;
- overlapping Allan deviation để đọc độ ổn định theo thời gian tích phân;
- PSD một phía theo phương pháp Welch để quan sát phổ nhiễu và tác dụng bộ lọc.

Có thể nhập khoảng cách chuẩn theo mét. Khi có chuẩn, GUI báo bias, RMSE và
P95 sai số tuyệt đối; khi để trống, histogram được ghi đúng là độ lệch so với
trung bình. Dòng tổng hợp còn hiển thị số mẫu, tần số mẫu, thời lượng, sigma,
drift, Allan minimum, số gap và số frame DS fallback. Dòng chất lượng của anchor
được capture bên dưới hiển thị:

- availability với điều kiện `valid` và `age <= 200 ms`;
- fresh/total, mean, noise sigma, P05-P95;
- filter delta P95, FPP median, age P95;
- số mẫu invalid/stale/missing.

Có thể dừng sớm và phân tích khi đã có ít nhất 20 mẫu. Nút **Lưu biểu đồ…** tạo
ảnh PNG độ phân giải cao (hoặc PDF/SVG) theo đúng bố cục 2 x 2; nút **Lưu dữ
liệu CSV…** lưu từng mẫu raw/firmware/host/FPP cùng toàn bộ thống kê của phiên.
Mặc định DS fallback được tách khỏi thống kê chính; dữ liệu raw/host
`CAL_MISSING` vẫn có thể bật để phục vụ calibration. Nếu TAG reset hoặc mất kết
nối giữa chừng, capture bị hủy để tránh trộn dữ liệu từ hai phiên.

Phân tích nặng chỉ chạy một lần sau khi capture kết thúc; tab không tính lại theo
nhịp telemetry 50 Hz. Nhờ vậy có thể thu tới 50.000 mẫu mà không làm đồ thị nhấp
nháy hoặc liên tục thay đổi ý nghĩa thống kê trong khi đang đo.

Allan deviation và PSD chỉ có ý nghĩa đánh giá nhiễu cảm biến khi TAG và anchor
đứng yên. Cả Raw/Firmware/Host dùng timestamp của frame TAG để giữ cùng một
timebase; `age_ms` chỉ là chỉ số độ trễ/chất lượng vì trên raw diagnostic nó vẫn
tính từ lần đo production thành công cuối. GUI xử lý wrap uint32, tách gap và
nội suy chỉ trong đoạn liên tục dài nhất. Telemetry 50 Hz có Nyquist 25 Hz, vì
vậy GUI không thể đo trực tiếp thành phần nhiễu 50 Hz.

Vòng nhận UART được lên lịch lại trong `finally`, nên lỗi vẽ hoặc phân tích một
frame được ghi vào log nhưng không thể làm GUI ngừng nhận dữ liệu.

Số cột histogram tự tăng theo lượng mẫu (xấp xỉ `2*sqrt(N)`, giới hạn 8–80
cột). Trục X dùng mép ngoài của bin nên cột đầu và cuối luôn nằm trọn trong
khung đồ thị.

### Calibration

1. Chọn anchor, nhập khoảng cách chuẩn giữa hai tâm anten và số mẫu.
2. Giữ TAG/anchor đứng yên rồi nhấn **Bắt đầu capture**.
3. GUI nhận cả raw diagnostic `CAL_MISSING`, hiển thị mean, noise và acceptance.
4. Capture tự dừng đủ mẫu; kết quả gồm offset, P05-P95, drift và FPP median.
5. Lặp lại ở nhiều khoảng cách (gợi ý 0.5/1/2/3/5/8 m), sau đó xuất CSV/JSON.

Offset trong báo cáo được định nghĩa là:

```text
offset_mm = reference_mm - mean_raw_mm
```

Không trộn capture SS-TWR và DS-TWR trong cùng bộ calibration. Nên dành ít nhất
một khoảng cách chưa dùng để làm held-out validation.

## Giới hạn 3D hiện tại

GUI đã hỗ trợ layout và solver 8 anchor, nhưng firmware/protocol đang sử dụng
trong project có thể mới truyền A1..A4. Định vị 3D thực tế chỉ khả dụng sau khi
firmware gửi tối thiểu bốn range valid có hình học không đồng phẳng trong cùng
chu kỳ đo. Không dùng nghiệm có RMS/residual lớn hoặc geometry condition xấu cho
điều khiển drone.

## Kết nối phần cứng

| Nguồn | Baud trong GUI |
|---|---|
| `Tag_DevKit` qua J-Link VCOM của DWM1001-DEV | **1000000** (mặc định) |
| `Tag` PCB qua gateway ESP32-C3 (USB Serial/JTAG) | tùy ý, gateway bỏ qua baud |
| `Tag` PCB nối USB-UART 3.3 V trực tiếp | **115200** |

Nếu sau 3 s không có frame hợp lệ, GUI ghi gợi ý vào ô sự kiện: nhận được byte
mà không có frame thường là sai baud.

Firmware `Tag` (PCB) phát UART0 **115200, 8N1**, packet binary:

| Tag DWM1001C | Đích |
|---|---|
| UART_TX, nRF P0.05 | RX của USB-UART 3.3 V hoặc ESP32-C3 GPIO20 |
| UART_RX, nRF P0.11 | TX của USB-UART hoặc ESP32-C3 GPIO21 (bắt buộc để nhận heartbeat/lệnh GUI) |
| GND | GND chung |

Không đưa mức TTL 5 V vào DWM1001C.

Trên PCB `RangingSystemClassic`, UART đã nối sang ESP32-C3. Khi dùng firmware
gateway có USB binary bridge, chỉ cần cắm cổng USB của ESP32-C3 và chọn COM đó.
Nếu gateway chưa được flash, có thể thử GUI trực tiếp bằng USB-UART 3.3 V nối
vào UART_TX của Tag.

Giữ `TELEM_ASCII=0` trong `Tag/include/uwb_app_config.h`; GUI đọc protocol binary
và tự đồng bộ lại sau boot log hoặc byte nhiễu.

## Chạy

```powershell
Set-Location '.\Software\UWB_UART_GUI'
.\run_gui.ps1
```

Nếu máy chưa có PySerial:

```powershell
py -3.12 -m pip install -r requirements.txt
```

Nhấn **Làm mới**, chọn đúng COM không phải `Standard Serial over Bluetooth`, chọn
baud theo bảng ở trên (mặc định 1000000 cho Tag_DevKit) và nhấn **Kết nối**.
Dòng vàng "đã mở COM; đang chờ TAG" chỉ xác nhận cổng serial. Dòng xanh "TAG
online" xuất hiện sau khi nhận được frame hợp lệ. GUI gửi `PING` mỗi giây; nếu
`DEVICE_INFO` cho biết `RANGE_SNAPSHOT` đang tắt, GUI bật lại bit này cho phiên
chạy hiện tại để đồ thị không bị trống, và bật `RANGE_MEAS` khi UART đủ nhanh.

## Thu và lưu dữ liệu

Thanh **Thu dữ liệu** ở đầu cửa sổ mặc định được thu gọn để dành chiều cao cho
các bảng và đồ thị. Nút ghi, trạng thái và số mẫu luôn hiển thị; nhấn **Hiện chi
tiết** khi cần đổi thư mục hoặc xem thống kê UART/firmware.

Sau khi UART đã kết nối, chọn thư mục rồi nhấn **Bắt đầu ghi**. Khi hoàn tất,
nhấn **Dừng và lưu**. Mỗi lần ghi tạo một thư mục riêng theo thời gian và COM:

| File | Nội dung |
|---|---|
| `range.csv` | Một dòng cho mỗi Anchor: host/tag time, sequence, valid, status, raw/FW filtered/host filtered mm và FPP |
| `meas.csv` | Một dòng cho mỗi bản ghi RANGE_MEAS: `meas_seq`, `meas_time_us`, mode, flags, status, raw/corrected/filtered mm, FP/RX, NLOS Δ, công suất phía anchor, `std_noise`, `fp_index`, clock offset, slot. Công suất không đo được để trống |
| `stats.csv` | Poll, OK, timeout, RX error, overrun, UART overflow và tần số |
| `uart.csv` | Tốc độ byte/s và bộ đếm parser phía PC: frame, CRC, byte bỏ, length/version/decode error |
| `info.jsonl` | Cấu hình firmware/calibration nhận được trong phiên |
| `events.log` | Kết nối, lỗi protocol, STALE và sự kiện trên GUI |
| `raw_telemetry.bin` | Ghép liên tiếp các frame binary đã qua kiểm tra CRC |
| `anchor_layout.json` | Snapshot tọa độ ENU đang được bản đồ/solver sử dụng |
| `session.json` | Metadata, thời lượng và tổng số bản ghi/drop |

Writer chạy ở thread riêng và flush mỗi giây. Nếu hàng đợi ghi đầy hoặc ổ đĩa
lỗi, `queue_drops`/`error` được lưu trong `session.json` và GUI hiển thị lỗi.
`session.json` cũng ghi phiên bản/tham số Host Filter để tái lập phép thử.

Chạy giao diện mô phỏng không cần board:

```powershell
.\run_demo.ps1
```

## Đọc kết quả chưa calibration

Firmware hiện đặt `UWB_DS_CALIBRATED_MASK=0`. Vì vậy hàng A1 có thể hiển thị:

- `Valid = NO`;
- `Status = CAL_MISSING`;
- `Raw (m)` vẫn thay đổi và dùng được để thu thập dữ liệu calibration;
- `FW Filter (m)` để trống nhằm ngăn thuật toán bay dùng nhầm khoảng cách chưa
  hiệu chỉnh;
- `Host Filter (m)` và đường xanh lá vẫn xuất hiện, nhưng chỉ là dữ liệu
  diagnostic chưa calibration, không phải khoảng cách hợp lệ cho điều khiển bay.

## Test decoder

```powershell
py -3.12 -m unittest discover -s tests -v
```

## Đóng gói EXE portable

Máy build cần Python 3.12 x64 và PyInstaller 6.x:

```powershell
py -3.12 -m pip install --user "pyinstaller>=6.10,<7"
Set-Location 'D:\Drone Project\UWB DW1001\Software\UWB_UART_GUI'
.\build_exe.ps1
```

Kết quả nằm trong `Output`. File `DWM1001_UWB_Ground_Control.exe` là bản
one-file/windowed, đã chứa Python, Tk/Tcl, PySerial và các module của GUI. Máy
Windows x64 nhận demo không cần cài Python. Dùng `CHAY_DEMO.cmd` để chạy nguồn
telemetry mô phỏng không cần board.

Ở bản đóng gói, `data_logs` và `anchor_layout.json` được lưu cạnh file EXE thay
vì thư mục giải nén tạm của PyInstaller.
