# UART đầy đủ cho calibration GUI — 25/10/2026

TAG mới: **`0xb5f8839d`**, SHA256
`bd1b1571d9474f1b2e1835dde0ccf6d1b4405c8ceaf17dbe00ba5e7ab8f337bf`.
Giữ lịch radio base450/slot275/margin450 UUS và PHY của bản phục hồi trước.
Chỉ cần nạp TAG; tám anchor hiện tại vẫn tương thích. Chưa tự nạp board.

## Đối chiếu nhận xét tester

Đã đọc `Plan/FIRMWARE.md` và repository
https://github.com/khoalea/IMGUI_UWB_CALIB tại commit
`1b259c08fe42791e802c3b77c7c62ec18025209f`.
Đây là nhận xét phía host; một số suy luận trong đó không khớp encoder C:

- Header burst offset11 là **u64 thời gian POLL bắt đầu**, không phải u32
  lúc cycle kết thúc + u32 dự phòng. `period_us` là khoảng POLL→POLL đo thực.
- Record8 byte: offset5 là FP code, offset6 đã chứa **RX−FP / 0,1 dB**,
  offset7 là **i8 CI / 0,25 ppm**. Ghép byte6–7 thành i16 CI sẽ trộn NLOS
  vào clock offset. FP=−code/2 dBm; RX=FP+delta/10 dB.
- `flags=0x20` là RADIO_OK, mode DS nằm ở hai bit thấp và bằng0. CAL_OK,
  VALID và RADIO_OK là ba cờ khác nhau.
- CAL_MISSING là chưa có **residual software DS calibration**. Delay phần
  cứng16436/16436 đã được cấu hình và tham gia timestamp; không được coi
  toàn bộ antenna delay phần cứng là chưa áp dụng rồi trừ thêm lần nữa.
- Temperature/VBAT đã có trong DIAG_SYSTEM; trước đây đo chậm và GUI tester
  chưa đọc. Repo ImGui chỉ có consumer JSON, không có `vcom_node.py` đã tạo
  JSON đó. Vì vậy sửa firmware đơn thuần không làm GUI cũ hiện thêm cột.

## Dữ liệu được bổ sung

Khung range nhanh0x18 giữ nguyên kích thước, nhịp và layout. Mỗi cycle vẫn
có FP, RX−FP, CI compact; host đã có thể tính RX trên mọi cycle mà không
tăng byte UART. Thêm **RX_QUALITY0x19 tối đa10 Hz** khi DIAG bật, chứa:

- RXPACC, FP_AMPL1/2/3, CIR_PWR, STD_NOISE, FP_INDEX nguyên bản của RESP.
- Carrier integrator signed21-bit đã sign-extend sang i32; CI ppm chính xác
  hơn nằm trong bản chất lượng. Công thức cùng driver hiện tại:
  `ci_ppm = -carrier_integrator * 1791 / 3125000`.
- FP/RX tại TAG với đơn vị0,01 dBm; FP/RX của FINAL tại anchor khi có Rb
  và anchor diagnostics hợp lệ. Độ phân giải0,01 dBm không phải cam kết
  độ chính xác vật lý0,01 dB.
- Nhiệt độ, điện áp và SAR raw của **TAG**, cùng tuổi phép đo môi trường.
  Không có cảm biến nhiệt độ riêng của từng anchor trong giao thức hiện tại.
- Boot ID, cycle_seq và timestamp u64 của chính cycle được lấy mẫu.

Không thêm SPI vào khe RESP. Các thanh ghi đã được chụp đúng RX buffer;
chỉ sao chép vào record cùng cycle trước khi queue. Không tính thêm log10.
Nhiệt độ đổi từ mỗi10 config checks sang khoảng1 Hz, bắt đầu sớm sau boot,
vẫn theo hai bước ADC không chờ blocking và chỉ chạy khi radio idle.

Mẫu chất lượng là snapshot10 Hz, không phải thông số mới cho từng range
275 Hz. JSON giữ raw trong `anchors[].quality` kèm cycle/time/age; bỏ cache
khi khác boot, ở tương lai hoặc quá500 ms. Không tạo thêm phép đo range.
Các trường chưa có dữ liệu dùng null/—. Không tự bật calibrated mask để
làm VALID đẹp hơn. Các power với RXPACC hoặc amplitude/CIR bằng0 dùng
sentinel unknown trong khung chất lượng.

## Layout RX_QUALITY schema1

Khung ngoài giữ `AA55`, version1, CRC16/CCITT và payload≤256 như trước.

| Offset payload | Trường | Kiểu |
|---:|---|---|
| 0 | schema=1 | u8 |
| 1 | boot_id | u16 |
| 3 | cycle_seq | u32 |
| 7 | POLL t_us | u64 |
| 15 | PRF MHz=16 | u8 |
| 16 | count | u8 |
| 17 | env_flags: bit0 temp valid, bit1 supply valid | u8 |
| 18 | temperature_centi, INT16_MIN=unknown | i16 |
| 20 | supply_mV, 0=unknown | u16 |
| 22 | env_age_ms lúc pack, UINT32_MAX=chưa đo | u32 |
| 26 | khoảng lấy mẫu tối thiểu, default100 ms | u16 |
| 28 | raw temperature SAR | u8 |
| 29 | raw supply SAR | u8 |
| 30 | count record, mỗi record28 byte | — |

| Offset record | Trường | Kiểu |
|---:|---|---|
| 0 | anchor_id | u8 |
| 1 | bit0 RESP registers valid, bit1 FINAL power valid | u8 |
| 2 | RXPACC | u16 |
| 4,6,8 | F1,F2,F3 | 3×u16 |
| 10 | CIR_PWR | u16 |
| 12 | STD_NOISE | u16 |
| 14 | FP_INDEX, fixed point10.6 | u16 |
| 16 | carrier_integrator | i32 |
| 20,22 | TAG FP/RX cdbm | 2×i16 |
| 24,26 | anchor FINAL FP/RX cdbm | 2×i16 |

Payload đủ tám anchor254 byte, khung270 byte. Zero raw không tự có nghĩa
missing: phải xét flags. Raw SAR chỉ có nghĩa khi env_age_ms khácUINT32_MAX.
Power INT16_MIN là unknown. Header môi trường và RESP có thời điểm riêng;
env_age_ms được ghi riêng vì ADC không đồng bộ từng cycle.

## Tốc độ và tài nguyên

460800 baud8N1 cho46.080 byte/s. Từ log145731, luồng cũ khoảng30.101 B/s:

| Mục | Chi phí |
|---|---:|
| RX_QUALITY270 B×10 Hz | tối đa2.700 B/s |
| Ba counter DIAG mới1 Hz | thêm12 B/s |
| Tổng UART ước tính | 30.101→32.813 B/s |
| Tải dung lượng UART | 65,32%→71,21% |
| Serialize riêng một khung chất lượng | 5,859 ms |
| RAM TAG | 15.892→16.876 B, thêm984 B |
| FLASH TAG | 83.916→85.020 B, thêm1.104 B |

Khoảng9% byte nhiều hơn luồng cũ, tương đương5,89 điểm phần trăm dung lượng
UART. **Không phải ranging giảm5,89% hay9%**: ring TX chạy bất đồng bộ trong
lúc radio tiếp tục đo. Một burst có thể tới host muộn hơn khi nằm sau khung
quality; timestamp TAG không thay đổi. CPU thêm copy, đóng gói/CRC, IRQ UART
và ADC chậm. Chưa đo trên board nên chưa có phần trăm giảm Hz đáng tin cậy.

Gửi chỉ10 byte raw bổ sung/anchor ở mọi cycle sẽ tạo khung182 byte;
277 Hz đã cần50.414 B/s, vượt46.080 trước khi cộng chẩn đoán. Do đó bản
này giữ power/NLOS nhanh và lấy raw10 Hz. Khi gần đầy UART, quality được
hoãn trước; một max frame được chừa lại cho luồng chính. Có thể tắt DIAG
để tắt quality hoặc build `UWB_RX_QUALITY_PERIOD_MS=0`; không đổi baud.

DIAG_SYSTEM nối thêm `quality_frames`, `quality_deferred`,
`quality_pack_max_us` sau byte196; prefix cũ vẫn được giữ. Dùng các counter
này và tốc độ DS theo timestamp TAG để đo ảnh hưởng sau khi nạp.
Số liệu tính toán ở `release/highrate_300/bandwidth_quality_20261025.json`.

## GUI và adapter

Windows EXE mới: `Software/UWB_UART_GUI/Output/DWM1001_UWB_Ground_Control_quality.exe`.
Tab **RX chất lượng** có đủ raw/power/CI/temperature/supply, đánh dấu stale.
Log mới có `rx_quality.csv` theo từng anchor/sample; burst.csv vẫn là range.

Gói tester: `Software/UWB_UART_GUI/Output/UWB_tester_quality_20261025.zip`:

- `IMGUI_UWB_CALIB/`: GUI tester đã sửa cột RX, RX−FP, CI, môi trường và
  bảng raw mở rộng; hỗ trợ timestamp64-bit, reset boot và dữ liệu thiếu.
- `uart_bridge/`: parser chuẩn và `ros2/vcom_node.py` xuất đúng
  `/uwb/burst_metadata`. Có thể đọc USB serial trực tiếp hoặc UDP relay.
- Patch cho checkout tester gốc ở `patches/imgui_quality_20261025.patch`.
- `firmware/`: TAG HEX và nhận dạng; `docs/`: đặc tả này và hướng dẫn.

Nút **Start UART telemetry** của GUI mới chạy adapter kèm gói (range và
quality). Nút bringup đầy đủ cũ vẫn dành cho drone stack của tester. Vị trí
`uwb/pos` cần solver/geometry của host; firmware không xuất vị trí giả.

## Kiểm tra đã thực hiện

- Bộ26 nhóm host test qua, gồm state machine/ranging/calibration/motion,
  gateway, encoder C→decoder Python và chẩn đoán định kỳ.
- Raw quality đúng buffer/cycle; frame tối đa; throttle100 ms; UART đầy
  thì hoãn; tắt DIAG; boot mới; không thêm SPI hoặc tạo range giả.
- Python kiểm format, fragmentation, null, stale/future/boot, CSV và widget.
- ROS Humble nhận C golden và xuất JSON; kiểm dedup và raw có sample age.
- Build ImGui headless, self-test JSON legacy/new, timestamp64-bit và bộ
  phân tích calibration; có phép thử đọc chính JSON do ROS bridge tạo.
- Build chín node và verifier9/9 HEX/build/manifest; dry-run đúng TAG.
- GUI EXE build và smoke-test. Chưa dùng board để đo lại Hz/CPU.

## Nạp và kiểm tra thật

Đóng chương trình giữ COM, nạp riêng TAG:

```powershell
& 'D:\Drone Project\Github Clone UWB Test\Firmware\scripts\flash_highrate.ps1' -Node TAG -Rate 300
```

Mở GUI mới, kiểm configb5f8839d, DIAG bật và khung0x19 khoảng10 Hz.
Ghi60–120 giây với cùng cấu hình/vị trí như baseline. Cần thấy raw đổi
theo mẫu, env khoảng1 Hz, không trộn raw cũ vào cycle mới; đối chiếu DS/s,
FINAL late/timeout, CRC, queue drop và quality_deferred/pack_max_us.
