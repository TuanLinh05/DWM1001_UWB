#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "../Tag/src/drivers/dw1000.c"

static int fail_spi;
static uint8_t rx_length;
int uwb_platform_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)tx;
    if (fail_spi) return -EIO;
    if (rx) { memset(rx, 0, len); rx[0] = rx_length; }
    return 0;
}
int uwb_platform_init(void) { return 0; }
int uwb_platform_spi_set_frequency(uint32_t hz) { (void)hz; return 0; }
void uwb_platform_cs_set(bool active) { (void)active; }
void uwb_platform_reset_radio(void) {}
void uwb_platform_enable_irq(void) {}
bool uwb_platform_irq_active(void) { return false; }
void uwb_platform_delay_ms(uint32_t ms) { (void)ms; }
void uwb_platform_delay_us(uint32_t us) { (void)us; }
uint32_t uwb_platform_time_ms(void) { static uint32_t t; return t++; }
uint32_t uwb_platform_cycle_now(void) { return 0; }
uint32_t uwb_platform_elapsed_us(uint32_t start) { return start; }
void uwb_platform_led_toggle(void) {}
void uwb_platform_led_set(bool on) { (void)on; }

int main(void)
{
    uint8_t data[20];
    memset(data, 0xCC, sizeof(data));
    fail_spi = 1;
    assert(DW1000_ReadStatus() == 0);
    assert(dw1000_spi_error_count == 2);
    fail_spi = 0;
    rx_length = 40;
    assert(DW1000_ReadRxData(data, sizeof(data)) == 0);
    assert(data[0] == 0xCC);
    rx_length = 16;
    assert(DW1000_ReadRxData(data, sizeof(data)) == 16);
    puts("driver failure/truncation tests passed");
    return 0;
}
