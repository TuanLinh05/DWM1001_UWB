/*
 * ESP32-C3 gateway: DWM1001C TAG telemetry -> USB, host commands -> TAG.
 *
 * UART RX is event driven so FIFO overruns and framing errors are counted
 * instead of silently losing frames. Every CRC-valid telemetry frame is
 * forwarded unchanged; bytes arriving on USB are forwarded to the TAG, whose
 * command parser validates them. A gateway health frame (TYPE 0x17) is
 * emitted once per second on the same binary stream.
 */

#include "gateway_config.h"
#include "telemetry_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static UwbTelemetryParser s_parser;
static QueueHandle_t s_uart_queue;
static uint8_t s_usb_frame_buffer[UWB_TELEM_FRAME_SIZE_MAX];

static struct {
    uint32_t uart_bytes;
    uint32_t usb_forwarded;
    uint32_t usb_dropped;
    uint32_t usb_rx_bytes;
    uint32_t cmd_bytes;
    uint32_t uart_fifo_overflow;
    uint32_t uart_buffer_full;
    uint32_t uart_frame_errors;
    uint32_t uart_parity_errors;
} s_stats;

static void usb_write_frame(const uint8_t *data, size_t length)
{
    size_t sent = 0U;

    /* Retry short writes so a busy host cannot cut a frame in half. */
    while (sent < length) {
        const int written = usb_serial_jtag_write_bytes(
            &data[sent], length - sent, GATEWAY_USB_TX_WAIT_TICKS);
        if (written <= 0) {
            s_stats.usb_dropped++;
            return;
        }
        sent += (size_t)written;
    }
    s_stats.usb_forwarded++;
}

static void telemetry_frame_received(const UwbTelemetryFrame *frame, void *context)
{
    (void)context;

    const size_t frame_length = UwbTelemetry_EncodeFrame(
        frame, s_usb_frame_buffer, sizeof(s_usb_frame_buffer));
    if (frame_length == 0U) {
        s_stats.usb_dropped++;
        return;
    }
    /* One call queues one complete CRC-validated telemetry frame. Never mix
     * printf/ESP_LOG output with this binary USB stream. */
    usb_write_frame(s_usb_frame_buffer, frame_length);
}

/*
 * GATEWAY_HEALTH payload (TYPE 0x17), little-endian:
 *   schema[1]=1 uptime_ms[4] uart_bytes[4] frames_ok[4] crc_errors[4]
 *   length_errors[4] version_errors[4] usb_forwarded[4] usb_dropped[4]
 *   usb_rx_bytes[4] cmd_bytes[4] uart_fifo_overflow[4] uart_buffer_full[4]
 *   uart_frame_errors[4] uart_parity_errors[4]              = 57 bytes
 */
static void send_health(void)
{
    uint8_t payload[57];
    uint16_t off = 0U;
    const uint32_t values[14] = {
        (uint32_t)(esp_timer_get_time() / 1000),
        s_stats.uart_bytes,
        s_parser.valid_frames,
        s_parser.crc_errors,
        s_parser.length_errors,
        s_parser.version_errors,
        s_stats.usb_forwarded,
        s_stats.usb_dropped,
        s_stats.usb_rx_bytes,
        s_stats.cmd_bytes,
        s_stats.uart_fifo_overflow,
        s_stats.uart_buffer_full,
        s_stats.uart_frame_errors,
        s_stats.uart_parity_errors,
    };

    payload[off++] = 1U;   /* schema */
    for (size_t i = 0U; i < sizeof(values) / sizeof(values[0]); i++) {
        payload[off++] = (uint8_t)values[i];
        payload[off++] = (uint8_t)(values[i] >> 8);
        payload[off++] = (uint8_t)(values[i] >> 16);
        payload[off++] = (uint8_t)(values[i] >> 24);
    }

    const UwbTelemetryFrame frame = {
        .version = UWB_TELEM_VERSION,
        .type = UWB_TELEM_TYPE_GATEWAY_HEALTH,
        .payload_length = off,
        .sequence = s_parser.valid_frames,
        .time_ms = (uint32_t)(esp_timer_get_time() / 1000),
        .payload = payload,
    };
    const size_t length = UwbTelemetry_EncodeFrame(&frame, s_usb_frame_buffer,
                                                   sizeof(s_usb_frame_buffer));
    if (length > 0U) {
        usb_write_frame(s_usb_frame_buffer, length);
    }
}

static esp_err_t gateway_uart_init(void)
{
    const uart_config_t config = {
        .baud_rate = GATEWAY_DWM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t result = uart_driver_install(
        GATEWAY_DWM_UART, GATEWAY_UART_RX_BUFFER_SIZE, GATEWAY_UART_TX_BUFFER_SIZE,
        GATEWAY_UART_QUEUE_LENGTH, &s_uart_queue, 0);
    if (result != ESP_OK) {
        return result;
    }
    result = uart_param_config(GATEWAY_DWM_UART, &config);
    if (result != ESP_OK) {
        return result;
    }
    return uart_set_pin(
        GATEWAY_DWM_UART, GATEWAY_DWM_TX_GPIO, GATEWAY_DWM_RX_GPIO,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static esp_err_t gateway_usb_init(void)
{
    usb_serial_jtag_driver_config_t config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    config.tx_buffer_size = GATEWAY_USB_TX_BUFFER_SIZE;
    config.rx_buffer_size = GATEWAY_USB_RX_BUFFER_SIZE;
    return usb_serial_jtag_driver_install(&config);
}

static esp_err_t gateway_ready_init(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << GATEWAY_DWM_READY_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&config);
}

/* Host -> TAG: forward raw bytes; the TAG validates the command CRC. */
static void command_task(void *argument)
{
    uint8_t buffer[GATEWAY_USB_READ_CHUNK];
    (void)argument;

    while (true) {
        const int length = usb_serial_jtag_read_bytes(
            buffer, sizeof(buffer), pdMS_TO_TICKS(100));
        if (length > 0) {
            s_stats.usb_rx_bytes += (uint32_t)length;
            const int written = uart_write_bytes(GATEWAY_DWM_UART, buffer, (size_t)length);
            if (written > 0) {
                s_stats.cmd_bytes += (uint32_t)written;
            }
        }
    }
}

void app_main(void)
{
    uint8_t data[GATEWAY_UART_READ_CHUNK];
    int64_t last_health_us = 0;

    ESP_ERROR_CHECK(gateway_ready_init());
    ESP_ERROR_CHECK(gateway_usb_init());
    ESP_ERROR_CHECK(gateway_uart_init());
    UwbTelemetryParser_Init(&s_parser, telemetry_frame_received, NULL);
    xTaskCreate(command_task, "uwb_cmd", 3072, NULL, 5, NULL);

    while (true) {
        uart_event_t event;

        if (xQueueReceive(s_uart_queue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (event.type) {
            case UART_DATA: {
                size_t remaining = event.size;
                while (remaining > 0U) {
                    const size_t chunk = remaining > sizeof(data) ? sizeof(data) : remaining;
                    const int length = uart_read_bytes(GATEWAY_DWM_UART, data, chunk, 0);
                    if (length <= 0) {
                        break;
                    }
                    s_stats.uart_bytes += (uint32_t)length;
                    UwbTelemetryParser_Feed(&s_parser, data, (size_t)length);
                    remaining -= (size_t)length;
                }
                break;
            }
            case UART_FIFO_OVF:
                s_stats.uart_fifo_overflow++;
                uart_flush_input(GATEWAY_DWM_UART);
                xQueueReset(s_uart_queue);
                break;
            case UART_BUFFER_FULL:
                s_stats.uart_buffer_full++;
                uart_flush_input(GATEWAY_DWM_UART);
                xQueueReset(s_uart_queue);
                break;
            case UART_FRAME_ERR:
                s_stats.uart_frame_errors++;
                break;
            case UART_PARITY_ERR:
                s_stats.uart_parity_errors++;
                break;
            default:
                break;
            }
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_health_us >= 1000000) {
            last_health_us = now_us;
            send_health();
        }
    }
}
