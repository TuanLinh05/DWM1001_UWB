#include "uwb_platform.h"
#include "dw1000_hw.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/timing/timing.h>

#define DW1000_NODE DT_NODELABEL(ieee802154)
#define LED_NODE DT_ALIAS(status_led)

BUILD_ASSERT(DT_NODE_HAS_STATUS(DW1000_NODE, okay),
             "The DWM1001 DW1000 devicetree node must be enabled");
BUILD_ASSERT(DT_NODE_HAS_STATUS(LED_NODE, okay),
             "The custom carrier status LED must be enabled");

static const struct device *const s_spi = DEVICE_DT_GET(DT_BUS(DW1000_NODE));
static const struct gpio_dt_spec s_cs = SPI_CS_GPIOS_DT_SPEC_GET(DW1000_NODE);
static const struct gpio_dt_spec s_irq =
    GPIO_DT_SPEC_GET(DW1000_NODE, int_gpios);
static const struct gpio_dt_spec s_reset =
    GPIO_DT_SPEC_GET(DW1000_NODE, reset_gpios);
static const struct gpio_dt_spec s_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

static struct gpio_callback s_irq_callback;
static bool s_initialized;
static uint32_t s_timing_cycles_per_us;
static const struct spi_config s_spi_config_init = {
    .frequency = 2000000U,
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = DT_REG_ADDR(DW1000_NODE),
    .word_delay = 0U,
};
static const struct spi_config s_spi_config_fast = {
    .frequency = DT_PROP(DW1000_NODE, spi_max_frequency),
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = DT_REG_ADDR(DW1000_NODE),
    .word_delay = 0U,
};
static const struct spi_config *s_active_spi_config = &s_spi_config_init;

static void dw1000_irq_callback(const struct device *port,
                                struct gpio_callback *callback,
                                gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(callback);
    ARG_UNUSED(pins);
    dw1000_irq_flag = 1U;
}

int uwb_platform_init(void)
{
    int error;

    if (s_initialized) {
        return 0;
    }

    if (!device_is_ready(s_spi) || !gpio_is_ready_dt(&s_cs) ||
        !gpio_is_ready_dt(&s_irq) || !gpio_is_ready_dt(&s_reset) ||
        !gpio_is_ready_dt(&s_led)) {
        return -ENODEV;
    }

    error = gpio_pin_configure_dt(&s_cs, GPIO_OUTPUT_INACTIVE);
    if (error != 0) {
        return error;
    }

    error = gpio_pin_configure_dt(&s_irq, GPIO_INPUT | GPIO_PULL_DOWN);
    if (error != 0) {
        return error;
    }

    error = gpio_pin_configure_dt(&s_reset, GPIO_INPUT);
    if (error != 0) {
        return error;
    }

    error = gpio_pin_configure_dt(&s_led, GPIO_OUTPUT_INACTIVE);
    if (error != 0) {
        return error;
    }

    gpio_init_callback(&s_irq_callback, dw1000_irq_callback, BIT(s_irq.pin));
    error = gpio_add_callback(s_irq.port, &s_irq_callback);
    if (error != 0) {
        return error;
    }

    /* The kernel RTC is 32768 Hz. Use the Cortex-M4 timing counter for
     * sub-millisecond deadlines and profiling, with wrap-safe subtraction. */
    /* Use the architecture API explicitly: Nordic's generic SoC timing
     * implementation claims TIMER2, whereas Cortex-M timing uses DWT. */
    arch_timing_init();
    arch_timing_start();
    s_timing_cycles_per_us = arch_timing_freq_get_mhz();
    if (s_timing_cycles_per_us == 0U)
        return -EIO;
    s_initialized = true;
    return 0;
}

int uwb_platform_spi_set_frequency(uint32_t frequency_hz)
{
    /* nRF52832 SPI and the board devicetree are both rated at 8 MHz here. */
    const uint32_t maximum = DT_PROP(DW1000_NODE, spi_max_frequency);

    if (frequency_hz == 0U || frequency_hz > maximum) {
        return -EINVAL;
    }
    s_active_spi_config = frequency_hz <= 2000000U
        ? &s_spi_config_init : &s_spi_config_fast;
    return 0;
}

int uwb_platform_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t length)
{
    struct spi_buf tx_buffer = { .buf = (void *)tx, .len = length };
    struct spi_buf rx_buffer = { .buf = rx, .len = length };
    struct spi_buf_set tx_set = { .buffers = &tx_buffer, .count = tx != NULL ? 1U : 0U };
    struct spi_buf_set rx_set = { .buffers = &rx_buffer, .count = rx != NULL ? 1U : 0U };

    if (length == 0U) {
        return 0;
    }
    return spi_transceive(s_spi, s_active_spi_config,
                          tx != NULL ? &tx_set : NULL,
                          rx != NULL ? &rx_set : NULL);
}

void uwb_platform_cs_set(bool active)
{
    (void)gpio_pin_set_dt(&s_cs, active ? 1 : 0);
}

void uwb_platform_reset_radio(void)
{
    /* RSTn is open-drain: assert low, then return the nRF pin to high-Z. */
    (void)gpio_pin_configure_dt(&s_reset, GPIO_OUTPUT_ACTIVE | GPIO_OPEN_DRAIN);
    k_msleep(2);
    (void)gpio_pin_configure_dt(&s_reset, GPIO_INPUT);
    k_msleep(50);
}

void uwb_platform_enable_irq(void)
{
    dw1000_irq_flag = 0U;
    (void)gpio_pin_interrupt_configure_dt(&s_irq, GPIO_INT_EDGE_TO_ACTIVE);
}

bool uwb_platform_irq_active(void)
{
    return gpio_pin_get_dt(&s_irq) > 0;
}

void uwb_platform_delay_ms(uint32_t delay_ms)
{
    k_msleep(delay_ms);
}

void uwb_platform_delay_us(uint32_t delay_us)
{
    k_busy_wait(delay_us);
}

uint32_t uwb_platform_time_ms(void)
{
    return k_uptime_get_32();
}

uint32_t uwb_platform_cycle_now(void)
{
    return (uint32_t)arch_timing_counter_get();
}

uint32_t uwb_platform_elapsed_us(uint32_t start_cycle)
{
    return (uint32_t)(uwb_platform_cycle_now() - start_cycle) / s_timing_cycles_per_us;
}

void uwb_platform_led_toggle(void)
{
    (void)gpio_pin_toggle_dt(&s_led);
}

void uwb_platform_led_set(bool on)
{
    (void)gpio_pin_set_dt(&s_led, on ? 1 : 0);
}
