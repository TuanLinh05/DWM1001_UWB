#ifndef UART_TX_H
#define UART_TX_H

#include <stdint.h>

#define UART_TX_BUF_SIZE 1024U

extern volatile uint32_t uart_tx_overflow_count;

int UART_TX_Init(void);
uint32_t UART_TX_Write(const uint8_t *data, uint16_t length);
uint16_t UART_TX_Pending(void);

#endif /* UART_TX_H */
