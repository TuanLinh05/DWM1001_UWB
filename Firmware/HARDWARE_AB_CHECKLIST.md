# Checklist A/B phần cứng và nghiệm thu — firmware v2

Tài liệu này là phần còn thiếu của bản nâng cấp: mọi thứ trong `CHANGELOG.md`
mới chỉ được chứng minh trên PC. Không mục nào dưới đây được bỏ qua trước khi
đưa range vào vòng điều khiển bay.

> **Cập nhật phần cứng:** A1–A4 là DWM1001C đang đo với TAG DevKit; A5–A8
> là STM32F103. HEX Zephyr v2/ID A5–A8 đã build và kiểm file, còn cần flash
> và đo trên phần cứng. Chỉ mở mask `0xFF` sau khi cả tám board được hiệu chuẩn
> và nghiệm thu. Xem `stm32_anchor/README.md`.

**Quy tắc:** mỗi lần chỉ đổi **một** cờ, luôn ghi log baseline trước và sau,
và giữ `UWB_DS_CALIBRATED_MASK = 0` cho tới khi Bước 7 hoàn tất.

---

## Bước 0 — Chuẩn bị

- [ ] Thước laser (sai số ≤ 2 mm) và vị trí đo cố định: 1 m, 3 m, 5 m, 10 m,
      tầm nhìn thẳng, ăng-ten cùng độ cao, không vật cản trong bán kính 1 m.
- [ ] Dán nhãn vật lý A1–A8; đối chiếu `Anchor_1..4` và `STM32_Anchor_5..8`.
- [ ] Ghi baseline **firmware cũ** trước khi flash: 5 phút log ở mỗi cự ly, lưu
      qua GUI (session recorder) — đây là mốc so sánh duy nhất còn lại sau khi flash.
- [ ] `git rev-parse --short HEAD` trùng với hash mà `DEVICE_INFO` sẽ báo.

## Bước 1 — Nghiệm thu chức năng (không đổi RF)

Flash theo thứ tự ở `CHANGELOG.md` §1 (anchor trước, TAG sau).

- [ ] Chỉ cấp nguồn Anchor: LED trạng thái tắt (không có TAG); nếu nhấp nhanh
      150 ms thì đó là fault, không phải link.
- [ ] Chỉ cấp nguồn TAG DevKit, chưa mở GUI: D9 xanh, D8 đỏ, D11 xanh dương đều
      tắt sau thời gian khởi động.
- [ ] Mở GUI: trạng thái chuyển từ vàng "đang chờ TAG" sang xanh "TAG online";
      D11 xanh dương sáng. Tắt GUI: D11 tắt trong tối đa 2,5 s.
- [ ] Bật một Anchor đúng ID: LED link của TAG và Anchor cùng sáng trong tối đa
      1 s sau exchange cuối; tắt Anchor thì cả hai không được tiếp tục chớp.
- [ ] `DEVICE_INFO` (0x16) của TAG và `ANCHOR_INFO` (0x12) báo đúng git hash +
      config hash, cờ dirty = 0; PARTID/LOTID của TAG khớp nhãn dán.
- [ ] `DIAG_SYSTEM` (0x15): `recovery_count = 0`, `fault_reboot_count = 0`,
      `uart_tx_high_water` < 80 % kích thước ring sau 10 phút.
- [ ] Rút nguồn 1 anchor: TAG vẫn chạy đủ chu kỳ, anchor đó vào chế độ probe
      xoay vòng (F1), các anchor còn lại **không** giảm tần suất cập nhật.
- [ ] Cắm lại anchor: được nhận lại trong < 2 s.
- [ ] Lệnh host: `ping`, `info`, `pause`, `resume`, `get-cal` trả `CMD_ACK` OK.
- [ ] `lock on` → lệnh đổi RF bị từ chối với `ERR_LOCKED`; `lock off` khôi phục.
- [ ] `SET_ANT_DELAY` khi chưa `pause` → `ERR_NOT_PAUSED`.

```powershell
Set-Location ..\Software\UWB_UART_GUI
py -3.12 uwb_command.py --port COM7 ping
py -3.12 uwb_command.py --port COM7 info
py -3.12 uwb_command.py --port COM7 get-cal
```

## Bước 2 — Ngân sách thời gian 50 Hz (8 anchor)

Ước tính trên giấy là 3,1–4,1 ms/anchor ⇒ 25–33 ms cho 8 anchor, **vượt** ngân
sách 20 ms của chu kỳ 50 Hz. Phải đo thật trước khi chốt tần số.

```powershell
Set-Location ..\..\Firmware\tools
py -3.12 uwb_sniffer.py --port COM9 --seconds 30 --quiet --save-raw sniff.bin
```

- [ ] `POLL -> REPORT` median từng anchor ≤ 4 ms.
- [ ] Tổng median 8 anchor < 20 000 µs (dòng tổng kết của công cụ). Nếu vượt:
      hạ tần số chu kỳ hoặc giảm số anchor mỗi chu kỳ — **ghi lại quyết định**.
- [ ] Không thấy chồng lấn khe thời gian (RX_ERROR cụm) trong 30 s.

## Bước 3 — Xác nhận FPP đã đúng thang

- [ ] `INFO.flags` có bit `0x20` (FPP_CORRECTED).
- [ ] Ở cùng cự ly và cùng vị trí, FPP mới ≈ FPP cũ **+ 12,0 dB** (±1 dB).
      Dùng `fpp_to_legacy_scale_cdbm()` để đưa log mới về thang cũ khi so sánh.
- [ ] Hành vi bộ lọc (độ trễ, số lần reacquire) không đổi so với baseline —
      đây là mục đích của `UWB_FILTER_FPP_COMPAT_DB`.

## Bước 4 — A/B `UWB_DW_REFERENCE_TUNING` (0 → 1)

Bật LDE NTM = 13, LDOTUNE và crystal trim từ OTP.

- [ ] Build A (=0) và build B (=1), mỗi build 5 phút/cự ly, cùng vị trí.
- [ ] So sánh: bias trung bình, độ lệch chuẩn, tỉ lệ phép đo thành công, FPP.
- [ ] Chấp nhận B chỉ khi std **không tăng** và tỉ lệ thành công **không giảm**.
- [ ] Nếu nhận B: calibration ở Bước 7 phải đo lại từ đầu trên build B.

## Bước 5 — A/B `UWB_TX_POWER_MODE`

LEGACY (`0x1E1E1E1E`) đang cao hơn mức tham chiếu của DW1000 UM khoảng 16 dB.

- [ ] Kiểm tra giới hạn công suất phát hợp pháp tại nơi sử dụng trước khi đổi.
- [ ] A/B LEGACY → REFERENCE (`0x48484848`): kỳ vọng FPP giảm, tầm xa giảm,
      nhiễu gần giảm. Đo lại tầm hoạt động tối đa thực tế.
- [ ] Nếu cần: thử SMART (bật smart TX power) và ghi nhận chênh lệch giữa khung
      ngắn/dài.
- [ ] **Bất kỳ thay đổi nào ở bước này đều huỷ calibration cũ** — quay lại Bước 7.

## Bước 6 — A/B `UWB_USE_CLOCK_CORRECTION` (chỉ ảnh hưởng SS)

Hệ số đã được sửa dấu (`UWB_CLOCK_OFFSET_MULT = -5,7312e-10`).

- [ ] Ép chế độ SS (tạm thời `UWB_USE_DS_TWR = 0`), đo cùng cự ly với DS.
- [ ] Bật `UWB_USE_CLOCK_CORRECTION = 1`: sai số SS phải **giảm** và tiến về kết
      quả DS. Nếu sai số tăng → dấu vẫn sai, trả cờ về 0 và báo lại.
- [ ] Trả `UWB_USE_DS_TWR` về 1 sau khi đo.

## Bước 7 — Calibration từng anchor rồi mới mở mask

Thực hiện sau khi **đã chốt** mọi cờ RF ở Bước 4–6.

Với mỗi anchor N = 1..8, ở cự ly chuẩn (khuyến nghị 5 m):

```powershell
py -3.12 uwb_command.py --port COM7 set-mask 0x01      # chỉ đo anchor đang hiệu chuẩn
py -3.12 uwb_command.py --port COM7 set-cal 1 12.5     # bias đo được, đơn vị mm
py -3.12 uwb_command.py --port COM7 save
```

- [ ] Thu ≥ 2000 mẫu/anchor, tính bias trung vị; nhập bias qua `set-cal`.
- [ ] Kiểm chứng lại ở cự ly khác (1 m và 10 m): sai số còn lại ≤ ±30 mm.
- [ ] `get-cal` đọc về đúng giá trị sau `save` **và sau khi reboot**.
- [ ] Chỉ khi cả 8 anchor đạt: đặt `UWB_DS_CALIBRATED_MASK = 0xFF` trong
      `Tag/include/uwb_app_config.h`, build lại, flash lại TAG.
      (`test_node_projects.py` sẽ báo lỗi vì kho mã yêu cầu mask = 0 — sửa test
      cùng lúc và ghi rõ trong commit rằng phần cứng đã được hiệu chuẩn.)

## Bước 8 — Tiêu chí nghiệm thu cuối

| Hạng mục | Ngưỡng |
|---|---|
| Bias sau calibration (1–10 m, LOS) | ≤ ±30 mm |
| Độ lệch chuẩn tĩnh, 5 m | ≤ 30 mm |
| Tỉ lệ phép đo thành công/anchor | ≥ 98 % trong 10 phút |
| Cycle overrun | 0 trong 10 phút |
| `recovery_count` | ≤ 1 trong 30 phút |
| `fault_reboot_count` | 0 |
| CRC lỗi trên link telemetry | 0 trong 10 phút |
| `uart_rx_overflow_count` (gateway + TAG) | 0 |

## Rollback

1. Flash lại TAG bằng firmware cũ **trước**, rồi tới các anchor.
2. Hoặc giữ firmware v2 và trả từng cờ về mặc định
   (`UWB_DW_REFERENCE_TUNING = 0`, `UWB_TX_POWER_MODE = UWB_TX_POWER_LEGACY`,
   `UWB_USE_CLOCK_CORRECTION = 0`) — đây đã là mặc định của kho mã.
3. `factory-reset` xoá cấu hình NVS và đưa TAG về giá trị biên dịch.
