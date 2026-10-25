#include "uwb_platform.h"
#include "dw1000_hw.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/timing/timing.h>

#ifndef UWB_DIRECT_SPIM
#define UWB_DIRECT_SPIM 0
#endif
#ifndef UWB_DIRECT_STM32_SPI
#define UWB_DIRECT_STM32_SPI 0
#endif
#if UWB_DIRECT_SPIM || UWB_DIRECT_STM32_SPI
#include <zephyr/drivers/pinctrl.h>
#endif
#if UWB_DIRECT_SPIM
#include <hal/nrf_gpio.h>
#include <hal/nrf_spim.h>
#if !defined(CONFIG_SOC_NRF52832)
#error "Direct SPIM backend is qualified only for nRF52832"
#endif
#endif
#if UWB_DIRECT_STM32_SPI
#include <stm32f1xx.h>
#if !defined(CONFIG_SOC_SERIES_STM32F1X)
#error "Direct STM32 SPI is qualified only for the F103 SPI1 board"
#endif
#endif

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

#if UWB_DIRECT_SPIM || UWB_DIRECT_STM32_SPI
BUILD_ASSERT(IS_ENABLED(CONFIG_PINCTRL),
             "Direct SPI requires highrate_common.conf for pinctrl support");
#endif

#if UWB_DIRECT_SPIM
/* Exclusive ownership: the overlay disables the Zephyr bus device. GPIO,
 * pinctrl, UART, watchdog and the kernel still use their Zephyr drivers. */
BUILD_ASSERT(IS_ENABLED(CONFIG_NRFX_SPIM),
             "Direct SPIM requires highrate_nrf.conf for Nordic pinctrl support");
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_BUS(DW1000_NODE), disabled),
             "Direct SPIM requires the DW1000 SPI bus disabled in devicetree");
BUILD_ASSERT(DT_REG_ADDR(DT_BUS(DW1000_NODE)) == 0x40023000UL,
             "Direct SPIM requires the DWM1001 internal SPIM2 bus");
BUILD_ASSERT(DT_SAME_NODE(DT_SPI_DEV_CS_GPIOS_CTLR(DW1000_NODE), DT_NODELABEL(gpio0)),
             "Direct SPIM chip select must be on nRF52832 P0");
BUILD_ASSERT((DT_SPI_DEV_CS_GPIOS_FLAGS(DW1000_NODE) & GPIO_ACTIVE_LOW) != 0,
             "DW1000 chip select must be active low");
PINCTRL_DT_DEFINE(DT_BUS(DW1000_NODE));
static NRF_SPIM_Type *const s_spim =
    (NRF_SPIM_Type *)DT_REG_ADDR(DT_BUS(DW1000_NODE));
#elif UWB_DIRECT_STM32_SPI
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_BUS(DW1000_NODE), disabled),
             "Direct STM32 SPI requires highrate_stm32.overlay");
BUILD_ASSERT(DT_REG_ADDR(DT_BUS(DW1000_NODE)) == 0x40013000UL,
             "Direct STM32 SPI requires SPI1");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(rcc), clock_frequency) == 64000000UL,
             "SPI1 divisors are qualified for the 64 MHz board clock");
BUILD_ASSERT(DT_PROP(DT_NODELABEL(rcc), apb2_prescaler) == 1,
             "SPI1 requires APB2 = 64 MHz");
PINCTRL_DT_DEFINE(DT_BUS(DW1000_NODE));
#else
static const struct device *const s_spi = DEVICE_DT_GET(DT_BUS(DW1000_NODE));
#endif
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
#if !UWB_DIRECT_SPIM && !UWB_DIRECT_STM32_SPI
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
#endif

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

    if (
#if !UWB_DIRECT_SPIM && !UWB_DIRECT_STM32_SPI
        !device_is_ready(s_spi) ||
#endif
        !gpio_is_ready_dt(&s_cs) ||
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
#if UWB_DIRECT_SPIM
    s_spim->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
    error = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_BUS(DW1000_NODE)),
                                PINCTRL_STATE_DEFAULT);
    if (error != 0)
        return error;
    s_spim->INTENCLR = 0xFFFFFFFFUL;
    s_spim->SHORTS = 0U;
    s_spim->CONFIG = 0U; /* mode 0, MSB first */
    s_spim->ORC = 0U;
    s_spim->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M2;
    s_spim->ENABLE = SPIM_ENABLE_ENABLE_Enabled;
#elif UWB_DIRECT_STM32_SPI
    error = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_BUS(DW1000_NODE)),
                                PINCTRL_STATE_DEFAULT);
    if (error != 0)
        return error;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    __DSB();
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    SPI1->CR2 = 0U;
    /* APB2/32 = 2 MHz for reset/OTP/LDE. Mode 0, 8 bit, software NSS. */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_2;
    SPI1->CR1 |= SPI_CR1_SPE;
#endif
    s_initialized = true;
    return 0;
}

int uwb_platform_spi_set_frequency(uint32_t frequency_hz)
{
    /* The devicetree profile owns the qualified runtime ceiling. */
    const uint32_t maximum = DT_PROP(DW1000_NODE, spi_max_frequency);

    if (frequency_hz == 0U || frequency_hz > maximum) {
        return -EINVAL;
    }
#if UWB_DIRECT_SPIM
    if (frequency_hz != 2000000U && frequency_hz != 8000000U)
        return -EINVAL;
    s_spim->FREQUENCY = frequency_hz == 2000000U
        ? SPIM_FREQUENCY_FREQUENCY_M2 : SPIM_FREQUENCY_FREQUENCY_M8;
#elif UWB_DIRECT_STM32_SPI
    uint32_t divisor;
    if (frequency_hz == 2000000U)
        divisor = SPI_CR1_BR_2;
    else if (frequency_hz == 8000000U)
        divisor = SPI_CR1_BR_1;
    else if (frequency_hz == 16000000U)
        divisor = SPI_CR1_BR_0;
    else
        return -EINVAL;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR) | divisor;
    SPI1->CR1 |= SPI_CR1_SPE;
#else
    s_active_spi_config = frequency_hz <= 2000000U
        ? &s_spi_config_init : &s_spi_config_fast;
#endif
    return 0;
}

/* Longest DW1000 access: 3-byte header + a 127-byte frame buffer. */
#define SPI_XFER_MAX (3U + 127U)

#if UWB_DIRECT_STM32_SPI
static int stm32_spi_abort(int error)
{
    const uint32_t config = SPI1->CR1;
    SPI1->CR1 = 0U;
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    SPI1->CR2 = 0U;
    SPI1->CR1 = config;
    return error; /* caller releases CS; SPI-fault recovery resets radio */
}
#endif

int uwb_platform_spi_xfer(const uint8_t *header, size_t header_length,
                          const uint8_t *tx, uint8_t *rx, size_t length)
{
    /* One spi_transceive() with ONE contiguous buffer per DW1000 register
     * access. Split header/data buffers made the driver run a separate
     * chunk per buffer: a 1-byte header chunk on the EasyDMA SPIM hits
     * nRF52832 anomaly 58 (an extra clocked byte) and costs a DMA round
     * trip, and the legacy SPI paid one interrupt per byte. The staging
     * buffers are in RAM as EasyDMA requires; SPI runs only from the main
     * loop, never from an ISR, so static buffers are not shared. */
    static uint8_t s_tx[SPI_XFER_MAX];
    static uint8_t s_rx[SPI_XFER_MAX];
    const size_t total = header_length + length;

    if (header == NULL || header_length == 0U || header_length > 3U
        || (tx != NULL && rx != NULL) || total > SPI_XFER_MAX) {
        return -EINVAL;
    }

    memcpy(s_tx, header, header_length);
    if (tx != NULL) {
        memcpy(&s_tx[header_length], tx, length);
    } else {
        memset(&s_tx[header_length], 0, length);
    }

    const bool reading = rx != NULL && length > 0U;
#if UWB_DIRECT_SPIM
    /* Full duplex even for writes; equal TX/RX counts >= 2 avoid anomaly 58.
     * Buffers live in RAM. END is polled, so a register access has no kernel
     * semaphore/context-switch or DMA interrupt round trip. */
    if (total < 2U)
        return -EINVAL;
    s_spim->EVENTS_END = 0U;
    s_spim->EVENTS_STOPPED = 0U;
    s_spim->TXD.PTR = (uint32_t)(uintptr_t)s_tx;
    s_spim->TXD.MAXCNT = total;
    s_spim->RXD.PTR = (uint32_t)(uintptr_t)s_rx;
    s_spim->RXD.MAXCNT = total;
    __DMB();
    const uint32_t started = uwb_platform_cycle_now();
    const uint32_t timeout_cycles = s_timing_cycles_per_us * 1000U;
    s_spim->TASKS_START = 1U;
    while (s_spim->EVENTS_END == 0U) {
        /* Compare raw DWT cycles in the byte-transfer hot loop. Keep the
         * same wrap-safe 1 ms timeout without a timing call/divide per poll. */
        if ((uint32_t)(uwb_platform_cycle_now() - started) > timeout_cycles) {
            s_spim->TASKS_STOP = 1U;
            const uint32_t stopping = uwb_platform_cycle_now();
            while (s_spim->EVENTS_STOPPED == 0U
                   && uwb_platform_elapsed_us(stopping) < 100U) {
            }
            /* Always stop DMA before the caller reuses the staging buffer.
             * The existing SPI-fault path resets the DW1000 and reinitializes. */
            s_spim->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
            s_spim->ENABLE = SPIM_ENABLE_ENABLE_Enabled;
            return -ETIMEDOUT;
        }
    }
    __DMB();
    const int error = (s_spim->TXD.AMOUNT == total && s_spim->RXD.AMOUNT == total)
        ? 0 : -EIO;
#elif UWB_DIRECT_STM32_SPI
    /* One byte outstanding at a time. Reads drain RXNE even for writes,
     * preventing OVR. No ISR, semaphore or context switch per access. */
    const uint32_t started = uwb_platform_cycle_now();
    for (size_t i = 0U; i < total; i++) {
        while ((SPI1->SR & SPI_SR_TXE) == 0U) {
            if (uwb_platform_elapsed_us(started) > 1000U)
                return stm32_spi_abort(-ETIMEDOUT);
        }
        *(volatile uint8_t *)&SPI1->DR = s_tx[i];
        while ((SPI1->SR & SPI_SR_RXNE) == 0U) {
            if (uwb_platform_elapsed_us(started) > 1000U)
                return stm32_spi_abort(-ETIMEDOUT);
        }
        s_rx[i] = *(volatile uint8_t *)&SPI1->DR;
    }
    while ((SPI1->SR & SPI_SR_BSY) != 0U) {
        if (uwb_platform_elapsed_us(started) > 1000U)
            return stm32_spi_abort(-ETIMEDOUT);
    }
    const int error = (SPI1->SR & (SPI_SR_OVR | SPI_SR_MODF)) != 0U
        ? stm32_spi_abort(-EIO) : 0;
#else
    const struct spi_buf tx_buffer = { .buf = s_tx, .len = total };
    const struct spi_buf rx_buffer = { .buf = s_rx, .len = total };
    const struct spi_buf_set tx_set = { .buffers = &tx_buffer, .count = 1U };
    const struct spi_buf_set rx_set = { .buffers = &rx_buffer, .count = 1U };

    const int error = spi_transceive(s_spi, s_active_spi_config, &tx_set,
                                     reading ? &rx_set : NULL);
#endif
    if (error == 0 && reading) {
        memcpy(rx, &s_rx[header_length], length);
    }
    return error;
}

void uwb_platform_cs_set(bool active)
{
#if UWB_DIRECT_SPIM
    /* GPIO is configured once by Zephyr during init. In the short burst
     * RX window every SPI register needs two CS transitions; avoid a GPIO
     * vtable call and generic pin checks for each transition on P0. */
    nrf_gpio_pin_write(DT_SPI_DEV_CS_GPIOS_PIN(DW1000_NODE), active ? 0U : 1U);
#else
    (void)gpio_pin_set_dt(&s_cs, active ? 1 : 0);
#endif
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
