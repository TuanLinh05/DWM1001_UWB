#ifndef UWB_PLATFORM_H
#define UWB_PLATFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int uwb_platform_init(void);
int uwb_platform_spi_set_frequency(uint32_t frequency_hz);

/**
 * One SPI transaction: `header` bytes out, then either `tx` bytes out (write)
 * or `length` bytes into `rx` (read). Exactly one of tx/rx may be non-NULL.
 * Chip select is driven by the caller around the call.
 */
int uwb_platform_spi_xfer(const uint8_t *header, size_t header_length,
                          const uint8_t *tx, uint8_t *rx, size_t length);

void uwb_platform_cs_set(bool active);
void uwb_platform_reset_radio(void);
void uwb_platform_enable_irq(void);
bool uwb_platform_irq_active(void);
void uwb_platform_delay_ms(uint32_t delay_ms);
void uwb_platform_delay_us(uint32_t delay_us);
uint32_t uwb_platform_time_ms(void);
uint32_t uwb_platform_cycle_now(void);
uint32_t uwb_platform_elapsed_us(uint32_t start_cycle);

/**
 * Monotonic microseconds since boot, 64-bit. Extends the 32-bit cycle
 * counter in software: call it at least once per counter period (~67 s at
 * 64 MHz); the main loops call it continuously. Main context only.
 */
uint64_t uwb_platform_time_us64(void);

void uwb_platform_led_toggle(void);
void uwb_platform_led_set(bool on);

/** Optional, board-specific indicators. Boards with only status-led simply
 * report false and ignore the corresponding set/toggle calls. */
bool uwb_platform_fault_led_available(void);
void uwb_platform_fault_led_toggle(void);
void uwb_platform_fault_led_set(bool on);
bool uwb_platform_host_led_available(void);
void uwb_platform_host_led_set(bool on);

/* Reset-cause bits reported by uwb_platform_reset_cause() (Zephyr hwinfo). */
#define UWB_RESET_PIN        (1UL << 0)
#define UWB_RESET_SOFTWARE   (1UL << 1)
#define UWB_RESET_BROWNOUT   (1UL << 2)
#define UWB_RESET_POR        (1UL << 3)
#define UWB_RESET_WATCHDOG   (1UL << 4)
#define UWB_RESET_DEBUG      (1UL << 5)
#define UWB_RESET_LOCKUP     (1UL << 6)
#define UWB_RESET_OTHER      (1UL << 7)

/** Reset cause of this boot (UWB_RESET_* bits); cleared after reading. */
uint32_t uwb_platform_reset_cause(void);

/** Low 32 bits of the nRF52 factory device ID (FICR DEVICEID). */
uint32_t uwb_platform_device_id(void);

/** 32 random bits from the nRF RNG (falls back to counter mixing). */
uint32_t uwb_platform_random32(void);

/** Cold reboot of the MCU. Does not return. */
void uwb_platform_reboot(void);

#endif /* UWB_PLATFORM_H */
