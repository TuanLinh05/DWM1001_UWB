#include "uart_tx.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

#define UART_TX_MASK (UART_TX_BUF_SIZE - 1U)
#define UART_RX_MASK (UART_RX_BUF_SIZE - 1U)

BUILD_ASSERT((UART_TX_BUF_SIZE & UART_TX_MASK) == 0U,
             "UART_TX_BUF_SIZE must be a power of two");
BUILD_ASSERT((UART_RX_BUF_SIZE & UART_RX_MASK) == 0U,
             "UART_RX_BUF_SIZE must be a power of two");

volatile uint32_t uart_tx_overflow_count;
volatile uint32_t uart_rx_overflow_count;
volatile uint16_t uart_tx_high_water;

static const struct device *const s_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static uint8_t s_buffer[UART_TX_BUF_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;

static uint8_t s_rx_buffer[UART_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

static uint16_t buffer_count(void)
{
    return (uint16_t)((s_head - s_tail) & UART_TX_MASK);
}

static void uart_service_tx(const struct device *device)
{
    /* Hand the driver the largest contiguous block it accepts: the UARTE
     * (DMA) driver takes up to its TX buffer per call, the legacy UART one
     * byte, so the same loop is efficient for both. */
    while (s_tail != s_head && uart_irq_tx_ready(device)) {
        const uint16_t head = s_head;
        const uint16_t contiguous = (head > s_tail)
            ? (uint16_t)(head - s_tail)
            : (uint16_t)(UART_TX_BUF_SIZE - s_tail);
        const int sent = uart_fifo_fill(device, &s_buffer[s_tail], contiguous);

        if (sent <= 0) {
            break;
        }
        s_tail = (uint16_t)((s_tail + (uint16_t)sent) & UART_TX_MASK);
    }

    if (s_tail == s_head) {
        uart_irq_tx_disable(device);
    }
}

static void uart_service_rx(const struct device *device)
{
    uint8_t chunk[16];
    int length;

    while ((length = uart_fifo_read(device, chunk, sizeof(chunk))) > 0) {
        for (int index = 0; index < length; index++) {
            const uint16_t next = (uint16_t)((s_rx_head + 1U) & UART_RX_MASK);
            if (next == s_rx_tail) {
                uart_rx_overflow_count++;   /* drop newest byte; parser resyncs */
                continue;
            }
            s_rx_buffer[s_rx_head] = chunk[index];
            s_rx_head = next;
        }
    }
}

static void uart_callback(const struct device *device, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(device)) {
        return;
    }
    if (uart_irq_rx_ready(device)) {
        uart_service_rx(device);
    }
    if (uart_irq_tx_ready(device)) {
        uart_service_tx(device);
    }
}

int UART_TX_Init(void)
{
    if (!device_is_ready(s_uart)) {
        return -ENODEV;
    }

    s_head = 0U;
    s_tail = 0U;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    uart_tx_overflow_count = 0U;
    uart_rx_overflow_count = 0U;
    uart_tx_high_water = 0U;

    const int error = uart_irq_callback_user_data_set(s_uart, uart_callback, NULL);
    if (error != 0) {
        return error;
    }
    uart_irq_rx_enable(s_uart);
    return 0;
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

    const uint16_t pending = buffer_count();
    if (pending > uart_tx_high_water) {
        uart_tx_high_water = pending;
    }
    uart_irq_tx_enable(s_uart);
    irq_unlock(key);
    return length;
}

uint16_t UART_TX_Pending(void)
{
    return buffer_count();
}

uint16_t UART_RX_Read(uint8_t *data, uint16_t max_length)
{
    uint16_t count = 0U;

    if (data == NULL) {
        return 0U;
    }
    /* Single consumer (main loop); the ISR only advances s_rx_head. */
    while (count < max_length && s_rx_tail != s_rx_head) {
        data[count++] = s_rx_buffer[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) & UART_RX_MASK);
    }
    return count;
}
