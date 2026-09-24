#ifndef UART_TX_H
#define UART_TX_H

#include <stdint.h>

/**
 * Non-blocking UART link of the TAG: telemetry out (ring buffer drained by
 * the TX interrupt) and host commands in (RX interrupt into a ring buffer).
 */

#define UART_TX_BUF_SIZE 1024U
#define UART_RX_BUF_SIZE 256U

extern volatile uint32_t uart_tx_overflow_count;
extern volatile uint32_t uart_rx_overflow_count;
extern volatile uint16_t uart_tx_high_water;

int UART_TX_Init(void);

/** Queue `length` bytes atomically. @return length, or 0 if it does not fit. */
uint32_t UART_TX_Write(const uint8_t *data, uint16_t length);

/** Bytes still waiting to be sent. */
uint16_t UART_TX_Pending(void);

/** Copy up to `max_length` received bytes. @return number of bytes copied. */
uint16_t UART_RX_Read(uint8_t *data, uint16_t max_length);

#endif /* UART_TX_H */
