# Kết quả xác minh build

Môi trường đã dùng:

- nRF Connect SDK v3.4.0
- Zephyr 4.4.0 trong NCS
- Board `decawave_dwm1001_dev/nrf52832`
- GNU Arm Embedded 14.3.0

Các cấu hình đã link thành công:

| Cấu hình | Kết quả | Flash | RAM |
|---|---|---:|---:|
| TAG binary | đạt | 41,544 B | 10,456 B |
| TAG ASCII | đạt | 41,520 B | 10,456 B |
| Anchor 1 | đạt | 27,844 B | 7,768 B |
| Anchor 2 | đạt | 27,860 B | 7,768 B |
| Anchor 3 | đạt | 27,860 B | 7,768 B |
| Anchor 4 | đạt | 27,860 B | 7,768 B |
| Hardware test | đạt | 27,644 B | 8,736 B |

Build xác nhận devicetree, Kconfig, compile và link. Chưa thể xác nhận giao tiếp
radio thực tế, antenna delay hoặc chất lượng range nếu chưa kết nối DWM1001-DEV.
Hãy chạy hardware-test trước, sau đó mới chạy test DS-TWR ở các khoảng cách đã
biết và calibration lại nếu đổi profile compensation.
