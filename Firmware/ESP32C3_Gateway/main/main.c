#include "gateway_config.h"
#include "telemetry_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static UwbTelemetryParser s_parser;
static uint8_t s_usb_frame_buffer[UWB_TELEM_FRAME_SIZE_MAX];
static volatile uint32_t s_usb_forwarded_frames;
static volatile uint32_t s_usb_dropped_frames;

static void telemetry_frame_received(const UwbTelemetryFrame *frame,
                                     void *context)
{
    (void)context;

    const size_t frame_length = UwbTelemetry_EncodeFrame(
        frame, s_usb_frame_buffer, sizeof(s_usb_frame_buffer));
    if (frame_length == 0U) {
        s_usb_dropped_frames++;
        return;
    }

    /* One call queues one complete CRC-validated telemetry frame. Never mix
     * printf/ESP_LOG output with this binary USB stream. */
    const int written = usb_serial_jtag_write_bytes(
        s_usb_frame_buffer, frame_length, GATEWAY_USB_TX_WAIT_TICKS);
    if (written == (int)frame_length) {
        s_usb_forwarded_frames++;
    } else {
        s_usb_dropped_frames++;
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
        GATEWAY_DWM_UART, GATEWAY_UART_RX_BUFFER_SIZE, 0, 0, NULL, 0);
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
    usb_serial_jtag_driver_config_t config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
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

void app_main(void)
{
    uint8_t data[GATEWAY_UART_READ_CHUNK];

    ESP_ERROR_CHECK(gateway_ready_init());
    ESP_ERROR_CHECK(gateway_usb_init());
    ESP_ERROR_CHECK(gateway_uart_init());
    UwbTelemetryParser_Init(&s_parser, telemetry_frame_received, NULL);

    while (true) {
        const int length = uart_read_bytes(
            GATEWAY_DWM_UART, data, sizeof(data), pdMS_TO_TICKS(100));
        if (length > 0) {
            UwbTelemetryParser_Feed(&s_parser, data, (size_t)length);
        }
    }
}
