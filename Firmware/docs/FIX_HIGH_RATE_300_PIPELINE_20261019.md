# Hoàn thiện lịch xử lý TAG 300 Hz — 19/10/2026

> Bản lịch250/350, TAG `0x394ce68f`, đã thất bại trên board trong log143252.
> Dùng [bản phục hồi FINAL](FIX_HIGH_RATE_300_FINAL_RECOVERY_20261019.md).
> Nội dung dưới đây lưu lại thay đổi và giả thuyết của vòng trước.

## Baseline trên board

Log `20261019_134918_COM14`, TAG `0x84e6c9b9`, đã nhận cả tám anchor với
267–270 DS/s theo timestamp TAG, thành công 98,63–99,78% mỗi anchor/cycle.
Không còn gap >20 ms trong phiên này. Cycle trung vị3684 us, chưa đạt300 Hz.
STATS/INFO/DIAG đều không xuất hiện dù features0x07. Phân tích đầy đủ ở
[evaluation_log_20261019_134918.md](../release/highrate_300/evaluation_log_20261019_134918.md).

## Thay đổi

- Giữ double RX và trình tự chụp CI/payload/timestamp/quality đã cải thiện
  việc nhận RESP. Toán DS vẫn dùng tích int64 chính xác và phép chia double;
  không bật calibration mask hoặc trộn SS fallback vào số phép đo DS.
- Tính DS của cycle trước ngay khi đã nhận đủ dữ liệu Rb từ cycle hiện tại,
  trong lúc chờ FINAL hiện tại. Chia việc theo anchor, kiểm tra IRQ FINAL
  trước từng phần, tối đa tám phần mỗi lần gọi. Chỉ làm việc khi RX đã đóng;
  không lấy CPU của các khe RESP. Một record chỉ được đưa vào queue sau khi
  đã tính đủ các anchor, đúng thứ tự ID và meas_seq.
- Ghi nhận thời điểm xử lý TXFRS trước những phần tính còn lại. Thời gian
  phục hồi của anchor được chồng lên phần tính, không bắt đầu chờ lại sau
  khi tính xong. FINAL timeout/HPDWARN vẫn làm cycle hiện tại mất Rb; kết quả
  DS hợp lệ của cycle trước vẫn được giữ đúng quan hệ transaction.
- Đổi CI→0,01 ppm cho diagnostics sang phân số nguyên -1791/31250, làm tròn
  nửa đơn vị ra xa0. Bỏ double/lround ở đường chuyển diagnostics; không thay
  clock correction hoặc công thức range. Kiểm mọi giá trị CI signed21-bit.
- Profile300 dùng base450/slot250/margin350 UUS, gap180 us, period floor3100 us,
  info extra64 UUS. POLL→FINAL đủ tám anchor là2550 UUS, ngắn hơn2825 UUS
  của baseline khoảng282 us. Lịch này đi cùng double RX và publish pipeline;
  bộ gốc cùng250/350 nhưng single RX đã lỗi, không dùng ảnh gốc để thay thế.
- Mỗi burst cycle được cấp tối đa một việc bảo trì lúc radio idle. Giới hạn
  mỗi lượt600 us: config check500, stats/diag400, DEVICE_INFO300, ADC finish200.
  Khi thiếu slack, deadline POLL kế tiếp được lùi có giới hạn để health/DIAG
  không bị chờ vô hạn. Không cấp lượt trong TX/RX đang mở. Host control vẫn
  được phục vụ ở idle để nhận PAUSE/SET_BURST khi profile quá tải.
- STATS/INFO khoảng1 Hz; DIAG_SYSTEM khoảng1 Hz; profile300 có một lượt đủ
  tám DIAG_ANCHOR mỗi vòng1 s. Gửi theo các cycle riêng, không dồn cả vòng
  vào một khoảng rảnh. Counter thời gian dùng phép trừ wrap-safe.

## Log và GUI

DIAG_SYSTEM giữ nguyên prefix; nối thêm bốn uint32 sau extension cũ:

| Trường | Ý nghĩa |
|---|---|
| burst_publish_step_max_us | CPU lớn nhất của một phần publish |
| burst_post_final_max_us | Phần xử lý còn lại sau khi đã nhận biết FINAL TXFRS |
| burst_telemetry_max_us | Thời gian lớn nhất của phần telemetry/host trong idle |
| burst_maintenance_grants | Số việc bảo trì được cấp lượt |

`burst_publish_max_us` là tổng CPU các phần publish của một cycle; không cộng
thời gian chờ giữa các phần. Các giá trị max tích lũy từ boot, không phải
trung bình hoặc cam kết WCET. GUI cũ vẫn đọc prefix, GUI mới ghi cả các cột
hiệu năng vào diag_system.csv. Raw capture giữ đủ dữ liệu để đọc lại.

GUI mới: `Software/UWB_UART_GUI/Output/DWM1001_UWB_Ground_Control_300hz.exe`.
Nó có cả sửa ngắt đường tại gap của vòng trước; phiên log phải có filterv4.

## Kiểm thử và phát hành

- 26 nhóm host test, gồm driver, v2/v3, burst, motion, periodic telemetry,
  golden C/Python, gateway và GUI. GUI87 test, 2 skipped theo môi trường.
- Kiểm IRQ FINAL tới giữa các phần tính, không xuất trùng record, không
  khởi động lại recovery gap sau CPU work, FINAL không có TXFRS, A3 tắt,
  backoff/probe, frame sai/trùng/stale, chồng RESP, info TLV và wrap counter.
- Test periodic thực sự gọi encoder, kiểm CRC và đủ tám DIAG_ANCHOR ngay
  cả khi mọi cycle có idle budget0; kiểm một lượt/cycle, ưu tiên config,
  tắt DIAG giữa vòng và wrap uptime. Driver kiểm hơn2 triệu giá trị CI.
- Kiểm chỗ trống UART trước việc định kỳ hai frame; nếu gần đầy thì giữ
  nguyên việc đang chờ, không chiếm lượt bảo trì hoặc làm rơi frame, rồi
  tiếp tục khi UART đã thoát bớt dữ liệu. Trường hợp này đã được tái hiện
  bằng test thất bại trước sửa và qua sau sửa.
- HEX/manifest và biên nhận kiểm tra nằm ở `release/highrate_300`.
  Bộ baseline `0x84e6c9b9` giữ tại `previous_20261019_before_pipeline/`.
  Tám ảnh anchor lấy từ lượt build đủ chín node; TAG được build lại sau
  bước sửa kiểm tra UART cuối. Không đổi PHY, backend SPI hoặc giao thức anchor.

Không dùng kết quả host test để kết luận đã đạt300 DS/s trên board. Slot
ngắn hơn và phần xử lý sau hiệu chuẩn cần được đo lại trên nRF/STM32 thật.
Không tự nạp board trong lượt này.

## Nạp và nghiệm thu

TAG config mới: `0x394ce68f` (đối chiếu manifest cuối).
Anchor300/PHY64 hiện tại vẫn tương thích; thử nạp riêng TAG trước:

```powershell
& 'D:\Drone Project\Github Clone UWB Test\Firmware\scripts\flash_highrate.ps1' -Node TAG -Rate 300
```

Đóng GUI/relay trước khi nạp. Mở đúng EXE300hz mới, bật đủ tám anchor như log
baseline. Ghi60–120 s, sau đó10 phút. Cần có diag_system.csv, diag_anchor.csv,
stats.csv và INFO có dữ liệu. Đối chiếu config mới, slot250/margin350 và các
cột hiệu năng; kiểm DS/s từng anchor ≥300 theo cả host và TAG time, đồng thời
báo bộ đủ tám DS/s, gap, FINAL late, RX error, CRC và queue drop.

Nếu lịch nhanh làm tăng mất RESP, phép so sánh cùng PHY/pipeline có thể đổi
timing RAM về lịch rộng hơn, không cần nạp anchor:

```powershell
python 'D:\Drone Project\Github Clone UWB Test\Software\UWB_UART_GUI\uwb_command.py' --port COM14 --baud 460800 set-burst --base 450 --slot 275 --margin 450 --gap 180 --period 3600
```

Reset TAG trả về profile300 mới. Không chạy CLI khi GUI đang giữ COM.
