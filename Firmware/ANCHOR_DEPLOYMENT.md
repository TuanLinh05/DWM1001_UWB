# Triển khai firmware A1..A8

Tài liệu này là checklist để tránh flash nhầm hoặc tạo hai Anchor cùng địa chỉ.
Mỗi thư mục `Anchor_N` là một project Zephyr độc lập; điểm khác biệt danh tính
duy nhất nằm ở `ANCHOR_ADDR`, tên project và script build/flash.

## Ánh xạ firmware và phần cứng

| Nhãn module | Project | `ANCHOR_ADDR` | Bit DS calibration |
|---|---|---:|---:|
| A1 | `Anchor_1` | `0x0001` | `0x01` |
| A2 | `Anchor_2` | `0x0002` | `0x02` |
| A3 | `Anchor_3` | `0x0003` | `0x04` |
| A4 | `Anchor_4` | `0x0004` | `0x08` |
| A5 | `Anchor_5` | `0x0005` | `0x10` |
| A6 | `Anchor_6` | `0x0006` | `0x20` |
| A7 | `Anchor_7` | `0x0007` | `0x40` |
| A8 | `Anchor_8` | `0x0008` | `0x80` |

Không đổi `TAG_ADDR=0`. Không cấp cùng một `ANCHOR_ADDR` cho hai module đang
hoạt động trong cùng PAN.

## Trước khi build

1. Mở terminal nRF Connect SDK v3.4.0.
2. Chạy `tests/run_host_tests.ps1`. Test kiểm tra A1..A8 có đúng địa chỉ, source
   responder không bị lệch nhau, TAG có đủ tám slot và chu kỳ là 20 ms/50 Hz.
3. Giữ `UWB_DS_CALIBRATED_MASK=0U` khi module chưa được calibration thật.
4. Build bằng `scripts/build_all.ps1`, hoặc build riêng trong `Anchor_N` bằng
   `scripts/build.ps1`.

## Flash an toàn

1. Dán nhãn vật lý A1..A8 lên module trước khi cắm programmer.
2. Chỉ cắm một module cần flash.
3. Vào đúng `Firmware/Anchor_N` và chạy `scripts/flash.ps1`.
4. Ghi serial/module ID và tên file firmware vào biên bản cấu hình.
5. Bring-up từng module với TAG; xác nhận chỉ đúng cột A tương ứng có dữ liệu.

## Bring-up theo tầng

1. A1 đơn lẻ: kiểm tra DS-TWR, FPP, timeout và raw range.
2. A1+A2: kiểm tra phân tách địa chỉ và inter-anchor guard.
3. A1..A4: kiểm tra bố trí 2D và range finder độ cao.
4. A1..A8: kiểm tra chu kỳ 50 Hz, `cycle_overrun_count`, timeout riêng từng
   Anchor và telemetry đủ tám record.
5. Chỉ bật localization 3D sau khi geometry và calibration của từng Anchor đạt
   tiêu chí sai số đã chọn.

## Calibration fail-closed

Firmware không tự coi range là hợp lệ chỉ vì đã nhận được DS-TWR. Với mỗi
Anchor, đo nhiều điểm chuẩn và nhiều hướng antenna, ghi `UWB_DS_OFFSET_AN_M`,
sau đó mới bật bit tương ứng trong `UWB_DS_CALIBRATED_MASK`. Mask cho tám Anchor
đã calibration đầy đủ là `0xFFU`; chỉ dùng giá trị này khi cả tám module thực sự
đã qua validation.

Sau khi thay module, antenna, enclosure, kênh/PHY hoặc antenna delay, phải xóa
bit của module liên quan về fail-closed và calibration lại.
