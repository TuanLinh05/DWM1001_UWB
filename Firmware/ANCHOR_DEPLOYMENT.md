# Triển khai firmware A1..A8

Tài liệu này là checklist để tránh flash nhầm hoặc tạo hai Anchor cùng địa chỉ.
Mỗi board vật lý có một project Zephyr riêng và **dùng chung mã nguồn ở
`Firmware/common`**. A1–A4 dùng `Anchor_N` trên nRF52832; A5–A8 dùng
`STM32_Anchor_N` trên STM32F103C8T6.

> **Cấu hình phần cứng thực tế xác nhận 2026-09-23:** A1–A4 là DWM1001C và dùng
> các project Zephyr `Anchor_1..Anchor_4`. A5–A8 là STM32F103 + DW1000,
> nay dùng project `STM32_Anchor_5..STM32_Anchor_8`. Các thư mục
> `Anchor_5..Anchor_8` cũ vẫn build cho **nRF52832** và không dùng để nạp
> cho bốn board STM32.

> A5–A8 đã build được HEX v2, nhưng chưa thử trên board thật. Flash anchor
> tương thích trước TAG; xác nhận từng board bằng TAG trước khi bật đủ tám.

## Ánh xạ firmware và phần cứng

| Nhãn module | Mã cho board vật lý | `ANCHOR_ADDR` cần có | Bit DS calibration |
|---|---|---:|---:|
| A1 | Zephyr `Firmware/Anchor_1` | `0x0001` | `0x01` |
| A2 | Zephyr `Firmware/Anchor_2` | `0x0002` | `0x02` |
| A3 | Zephyr `Firmware/Anchor_3` | `0x0003` | `0x04` |
| A4 | Zephyr `Firmware/Anchor_4` | `0x0004` | `0x08` |
| A5 | Zephyr `Firmware/STM32_Anchor_5` | `0x0005` | `0x10` |
| A6 | Zephyr `Firmware/STM32_Anchor_6` | `0x0006` | `0x20` |
| A7 | Zephyr `Firmware/STM32_Anchor_7` | `0x0007` | `0x40` |
| A8 | Zephyr `Firmware/STM32_Anchor_8` | `0x0008` | `0x80` |

Không đổi `TAG_ADDR=0`. Không cấp cùng một `ANCHOR_ADDR` cho hai module đang
hoạt động trong cùng PAN.

## Trước khi build

1. Mở terminal nRF Connect SDK v3.4.0.
2. Chạy `tests/run_host_tests.ps1` và
   `python tools/verify_stm32_images.py`. Build STM32 đã kiểm giới hạn bộ nhớ,
   nhưng 50 Hz thực tế còn cần đo với cả tám board.
3. Giữ `UWB_DS_CALIBRATED_MASK=0U` khi module chưa được calibration thật.
4. Build A1–A4 bằng `scripts/build_all.ps1 -Projects Anchor_1,Anchor_2,Anchor_3,Anchor_4`.
   Build A5–A8 bằng `scripts/build_stm32_anchors.ps1`. Chạy
   `scripts/package_8_anchors.ps1` để gom ảnh vào `anchor_8_dist/manifest.csv`.

## Flash an toàn

1. Dán nhãn vật lý A1..A8 lên module trước khi cắm programmer.
2. Chỉ cắm một module cần flash.
3. Với A1–A4, dùng J-Link/OpenOCD và HEX `Anchor_N` đúng số. Với A5–A8,
   dùng ST-LINK Utility và HEX `anchor_8_dist/A_N_STM32F103C8.hex` đúng số;
   chọn Program & Verify, BOOT0 = 0, rồi reset. Xem `stm32_anchor/README.md`.
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
Anchor, đo nhiều điểm chuẩn và nhiều hướng antenna, nhập bias bằng lệnh
`SET_DS_CAL` (không cần build lại, lưu vào NVS bằng `SAVE_SETTINGS`) hoặc ghi
`UWB_DS_OFFSET_AN_M`, sau đó mới bật bit tương ứng trong
`UWB_DS_CALIBRATED_MASK`. Mask cho tám Anchor
đã calibration đầy đủ là `0xFFU`; chỉ dùng giá trị này khi cả tám module thực sự
đã qua validation.

Sau khi thay module, antenna, enclosure, kênh/PHY hoặc antenna delay, phải xóa
bit của module liên quan về fail-closed và calibration lại.

Quy trình đo và tiêu chí nghiệm thu đầy đủ: `HARDWARE_AB_CHECKLIST.md` bước 7–8.
