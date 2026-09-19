#ifndef UWB_PLATFORM_H
#define UWB_PLATFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int uwb_platform_init(void);
int uwb_platform_spi_set_frequency(uint32_t frequency_hz);
int uwb_platform_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t length);
void uwb_platform_cs_set(bool active);
void uwb_platform_reset_radio(void);
void uwb_platform_enable_irq(void);
bool uwb_platform_irq_active(void);
void uwb_platform_delay_ms(uint32_t delay_ms);
void uwb_platform_delay_us(uint32_t delay_us);
uint32_t uwb_platform_time_ms(void);
uint32_t uwb_platform_cycle_now(void);
uint32_t uwb_platform_elapsed_us(uint32_t start_cycle);
void uwb_platform_led_toggle(void);
void uwb_platform_led_set(bool on);

#endif /* UWB_PLATFORM_H */
