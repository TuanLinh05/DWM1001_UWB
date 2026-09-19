# Kiến trúc và ranh giới debug

## Luồng TAG

`main` gọi liên tục `Tag_Task()`. State machine thực hiện POLL → RESP → FINAL →
REPORT cho từng Anchor, có timeout, offline backoff và SS fallback. Sau một vòng
A1..A4, snapshot bất biến được chuyển cho telemetry. IRQ chỉ đặt cờ; mọi SPI và
tính toán chạy ở main context.

## Luồng Anchor

`Anchor_Task()` duy trì RX_WAIT → TX_RESPOND → WAIT_FINAL → TX_REPORT. Delayed
TX được kiểm tra HPDWARN ngay để tránh chờ timeout giả. Anchor address chỉ đến
từ Kconfig, vì vậy cùng một source tạo được bốn firmware.

## Ranh giới module

| Module | Trách nhiệm | Không nên chứa |
|---|---|---|
| `platform` | Zephyr SPI/GPIO/IRQ/time/LED | register hoặc protocol DW1000 |
| `drivers` | register, PHY, timestamp, diagnostics DW1000 | state TAG/Anchor |
| `ranging` | state machine và TWR | GPIO/SPI nRF trực tiếp |
| `filters` | lọc range không phụ thuộc MCU | radio I/O |
| `telemetry` | schema wire và UART ring buffer | thuật toán ranging |
| `diagnostics` | bring-up phần cứng | logic bay production |

Khi gặp lỗi, kiểm tra theo thứ tự: `hardware-test` → `DW1000_VerifyConfig()` →
counters timeout theo Anchor → status bit trong telemetry → filter diagnostics.
Trình tự này tách lỗi dây/radio khỏi lỗi protocol và lỗi filter.

## Tương thích với firmware STM32

- Giữ PAN `0xDECA`, TAG address `0x0000`, Anchor `0x0001..0x0004`.
- Giữ PHY Fast-256: channel 5, PRF16, preamble 256, PAC16, 6.8 Mbps.
- Giữ DS-TWR bốn message và SS fallback.
- Giữ telemetry binary schema 2 và CRC16-CCITT.
- Giữ calibration offsets, status bits, counters và range conditioners.
- Thay `HAL_GetTick` bằng `k_uptime_get_32`, DWT cycle timer bằng Zephyr cycle
  API, EXTI bằng GPIO callback, USART ISR bằng UART interrupt-driven API.
