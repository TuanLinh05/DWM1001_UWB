#ifndef GATEWAY_CONFIG_H
#define GATEWAY_CONFIG_H

#include "driver/gpio.h"
#include "driver/uart.h"

/* RangingSystemClassic PCB: DWM1001C UART_TX -> ESP_RX and vice versa. */
#define GATEWAY_DWM_UART       UART_NUM_1
#define GATEWAY_DWM_RX_GPIO    GPIO_NUM_20
#define GATEWAY_DWM_TX_GPIO    GPIO_NUM_21
#define GATEWAY_DWM_BAUD       115200

/* DWM1001C module pin 19 (nRF P0.26/RDY) is routed to ESP32-C3 GPIO10. */
#define GATEWAY_DWM_READY_GPIO GPIO_NUM_10

#define GATEWAY_UART_RX_BUFFER_SIZE 2048
#define GATEWAY_UART_READ_CHUNK      256

/* USB Serial/JTAG is a binary-only output owned by this gateway. A zero wait
 * keeps UART reception responsive; a complete frame is dropped if USB is full. */
#define GATEWAY_USB_TX_BUFFER_SIZE   4096
#define GATEWAY_USB_RX_BUFFER_SIZE    256
#define GATEWAY_USB_TX_WAIT_TICKS       0

#endif /* GATEWAY_CONFIG_H */
