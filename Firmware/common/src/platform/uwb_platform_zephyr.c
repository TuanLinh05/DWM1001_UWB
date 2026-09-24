#include "uwb_platform.h"
#include "dw1000_hw.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/timing/timing.h>

#define DW1000_NODE DT_NODELABEL(ieee802154)
#define LED_NODE DT_ALIAS(status_led)
#define FAULT_LED_NODE DT_ALIAS(fault_led)
#define HOST_LED_NODE DT_ALIAS(host_led)
#define EXTON_NODE DT_ALIAS(uwb_exton)
#define WAKE_NODE DT_ALIAS(uwb_wake)

#define HAS_FAULT_LED DT_NODE_EXISTS(FAULT_LED_NODE)
#define HAS_HOST_LED  DT_NODE_EXISTS(HOST_LED_NODE)
#define HAS_EXTON     DT_NODE_EXISTS(EXTON_NODE)
#define HAS_WAKE      DT_NODE_EXISTS(WAKE_NODE)

BUILD_ASSERT(DT_NODE_HAS_STATUS(DW1000_NODE, okay),
             "The DWM1001 DW1000 devicetree node must be enabled");
BUILD_ASSERT(DT_NODE_HAS_STATUS(LED_NODE, okay),
             "The board overlay must provide the status-led alias");

static const struct device *const s_spi = DEVICE_DT_GET(DT_BUS(DW1000_NODE));
static const struct gpio_dt_spec s_cs = SPI_CS_GPIOS_DT_SPEC_GET(DW1000_NODE);
static const struct gpio_dt_spec s_irq =
    GPIO_DT_SPEC_GET(DW1000_NODE, int_gpios);
static const struct gpio_dt_spec s_reset =
    GPIO_DT_SPEC_GET(DW1000_NODE, reset_gpios);
static const struct gpio_dt_spec s_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
#if HAS_FAULT_LED
static const struct gpio_dt_spec s_fault_led =
    GPIO_DT_SPEC_GET(FAULT_LED_NODE, gpios);
#endif
#if HAS_HOST_LED
static const struct gpio_dt_spec s_host_led =
    GPIO_DT_SPEC_GET(HOST_LED_NODE, gpios);
#endif
#if HAS_EXTON
static const struct gpio_dt_spec s_exton =
    GPIO_DT_SPEC_GET(EXTON_NODE, gpios);
#endif
#if HAS_WAKE
static const struct gpio_dt_spec s_wake =
    GPIO_DT_SPEC_GET(WAKE_NODE, gpios);
#endif

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
#if HAS_FAULT_LED
    if (!gpio_is_ready_dt(&s_fault_led)) {
        return -ENODEV;
    }
#endif
#if HAS_HOST_LED
    if (!gpio_is_ready_dt(&s_host_led)) {
        return -ENODEV;
    }
#endif
#if HAS_EXTON
    if (!gpio_is_ready_dt(&s_exton)) {
        return -ENODEV;
    }
#endif
#if HAS_WAKE
    if (!gpio_is_ready_dt(&s_wake)) {
        return -ENODEV;
    }
#endif

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
#if HAS_FAULT_LED
    error = gpio_pin_configure_dt(&s_fault_led, GPIO_OUTPUT_INACTIVE);
    if (error != 0) {
        return error;
    }
#endif
#if HAS_HOST_LED
    error = gpio_pin_configure_dt(&s_host_led, GPIO_OUTPUT_INACTIVE);
    if (error != 0) {
        return error;
    }
#endif
#if HAS_EXTON
    /* The STM32 CubeIDE board monitors DW1000 EXTON on PA1 as an input. */
    error = gpio_pin_configure_dt(&s_exton, GPIO_INPUT | GPIO_PULL_DOWN);
    if (error != 0) {
        return error;
    }
#endif
#if HAS_WAKE
    /* Match the earlier STM32 project: WAKE is an input with pull-down. */
    error = gpio_pin_configure_dt(&s_wake, GPIO_INPUT | GPIO_PULL_DOWN);
    if (error != 0) {
        return error;
    }
#endif

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

int uwb_platform_spi_xfer(const uint8_t *header, size_t header_length,
                          const uint8_t *tx, uint8_t *rx, size_t length)
{
    /* One spi_transceive() per DW1000 register access. The DWM1001 uses the
     * interrupt-per-byte nRF SPI peripheral, so every extra call (the
     * baseline issued 2-4 per access) cost a full driver round trip.
     * During a read the header is clocked out while the received bytes are
     * discarded (NULL rx buffer), then over-run characters clock the data. */
    const struct spi_buf tx_buffers[2] = {
        { .buf = (void *)header, .len = header_length },
        { .buf = (void *)tx, .len = length },
    };
    const struct spi_buf rx_buffers[2] = {
        { .buf = NULL, .len = header_length },
        { .buf = rx, .len = length },
    };
    const struct spi_buf_set tx_set = {
        .buffers = tx_buffers,
        .count = (tx != NULL && length > 0U) ? 2U : 1U,
    };
    const struct spi_buf_set rx_set = {
        .buffers = rx_buffers,
        .count = 2U,
    };

    if (header == NULL || header_length == 0U || (tx != NULL && rx != NULL)) {
        return -EINVAL;
    }
    return spi_transceive(s_spi, s_active_spi_config, &tx_set,
                          (rx != NULL && length > 0U) ? &rx_set : NULL);
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

uint64_t uwb_platform_time_us64(void)
{
    static uint32_t last_cycle;
    static uint64_t cycle_high;
    const uint32_t now = uwb_platform_cycle_now();

    if (now < last_cycle) {
        cycle_high += (uint64_t)1U << 32;
    }
    last_cycle = now;
    return (cycle_high + now) / s_timing_cycles_per_us;
}

void uwb_platform_led_toggle(void)
{
    (void)gpio_pin_toggle_dt(&s_led);
}

void uwb_platform_led_set(bool on)
{
    (void)gpio_pin_set_dt(&s_led, on ? 1 : 0);
}

bool uwb_platform_fault_led_available(void)
{
    return HAS_FAULT_LED != 0;
}

void uwb_platform_fault_led_toggle(void)
{
#if HAS_FAULT_LED
    (void)gpio_pin_toggle_dt(&s_fault_led);
#endif
}

void uwb_platform_fault_led_set(bool on)
{
#if HAS_FAULT_LED
    (void)gpio_pin_set_dt(&s_fault_led, on ? 1 : 0);
#else
    ARG_UNUSED(on);
#endif
}

bool uwb_platform_host_led_available(void)
{
    return HAS_HOST_LED != 0;
}

void uwb_platform_host_led_set(bool on)
{
#if HAS_HOST_LED
    (void)gpio_pin_set_dt(&s_host_led, on ? 1 : 0);
#else
    ARG_UNUSED(on);
#endif
}

uint32_t uwb_platform_reset_cause(void)
{
    uint32_t cause = 0U;
    uint32_t result = 0U;

    if (hwinfo_get_reset_cause(&cause) != 0) {
        return UWB_RESET_OTHER;
    }
    (void)hwinfo_clear_reset_cause();

    if ((cause & RESET_PIN) != 0U) {
        result |= UWB_RESET_PIN;
    }
    if ((cause & RESET_SOFTWARE) != 0U) {
        result |= UWB_RESET_SOFTWARE;
    }
    if ((cause & RESET_BROWNOUT) != 0U) {
        result |= UWB_RESET_BROWNOUT;
    }
    if ((cause & RESET_POR) != 0U) {
        result |= UWB_RESET_POR;
    }
    if ((cause & RESET_WATCHDOG) != 0U) {
        result |= UWB_RESET_WATCHDOG;
    }
    if ((cause & RESET_DEBUG) != 0U) {
        result |= UWB_RESET_DEBUG;
    }
    if ((cause & RESET_CPU_LOCKUP) != 0U) {
        result |= UWB_RESET_LOCKUP;
    }
    if (cause != 0U && result == 0U) {
        result = UWB_RESET_OTHER;
    }
    return result;
}

uint32_t uwb_platform_device_id(void)
{
    uint8_t id[8] = {0};
    const ssize_t length = hwinfo_get_device_id(id, sizeof(id));

    if (length < 4) {
        return 0U;
    }
    /* Zephyr returns the identifier big-endian; keep the low word. */
    return ((uint32_t)id[length - 4] << 24) | ((uint32_t)id[length - 3] << 16)
         | ((uint32_t)id[length - 2] << 8) | (uint32_t)id[length - 1];
}

uint32_t uwb_platform_random32(void)
{
    uint32_t value = 0U;

#if defined(CONFIG_ENTROPY_GENERATOR) && DT_HAS_CHOSEN(zephyr_entropy)
    const struct device *const entropy = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));

    if (device_is_ready(entropy)
        && entropy_get_entropy(entropy, (uint8_t *)&value, sizeof(value)) == 0) {
        return value;
    }
#endif
    value = uwb_platform_cycle_now() ^ k_cycle_get_32() ^ uwb_platform_device_id();
    return value ^ (value >> 16) ^ 0x9E3779B9U;
}

void uwb_platform_reboot(void)
{
    sys_reboot(SYS_REBOOT_COLD);
}
