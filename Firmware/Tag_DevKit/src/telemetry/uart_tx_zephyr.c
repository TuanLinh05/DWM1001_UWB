#include "uart_tx.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

#define UART_TX_MASK (UART_TX_BUF_SIZE - 1U)

BUILD_ASSERT((UART_TX_BUF_SIZE & UART_TX_MASK) == 0U,
             "UART_TX_BUF_SIZE must be a power of two");

volatile uint32_t uart_tx_overflow_count;

static const struct device *const s_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static uint8_t s_buffer[UART_TX_BUF_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;

static uint16_t buffer_count(void)
{
    return (uint16_t)((s_head - s_tail) & UART_TX_MASK);
}

static void uart_callback(const struct device *device, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(device) || !uart_irq_tx_ready(device)) {
        return;
    }

    while (s_tail != s_head && uart_irq_tx_ready(device)) {
        uint8_t byte = s_buffer[s_tail];
        if (uart_fifo_fill(device, &byte, 1) != 1) {
            break;
        }
        s_tail = (uint16_t)((s_tail + 1U) & UART_TX_MASK);
    }

    if (s_tail == s_head) {
        uart_irq_tx_disable(device);
    }
}

int UART_TX_Init(void)
{
    if (!device_is_ready(s_uart)) {
        return -ENODEV;
    }

    s_head = 0U;
    s_tail = 0U;
    uart_tx_overflow_count = 0U;
    return uart_irq_callback_user_data_set(s_uart, uart_callback, NULL);
}

uint32_t UART_TX_Write(const uint8_t *data, uint16_t length)
{
    unsigned int key;
    uint16_t head;

    if (data == NULL || length == 0U) {
        return 0U;
    }

    key = irq_lock();
    if (length > (uint16_t)(UART_TX_MASK - buffer_count())) {
        uart_tx_overflow_count++;
        irq_unlock(key);
        return 0U;
    }

    head = s_head;
    for (uint16_t index = 0U; index < length; index++) {
        s_buffer[head] = data[index];
        head = (uint16_t)((head + 1U) & UART_TX_MASK);
    }
    s_head = head;
    uart_irq_tx_enable(s_uart);
    irq_unlock(key);
    return length;
}

uint16_t UART_TX_Pending(void)
{
    return buffer_count();
}
