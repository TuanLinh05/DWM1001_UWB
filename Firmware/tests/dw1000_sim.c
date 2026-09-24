/* Register-level DW1000 simulator for host tests (see dw1000_sim.h). */
#include "dw1000_sim.h"
#include "dw1000_hw.h"
#include "uwb_platform.h"

#include <errno.h>
#include <string.h>

uint64_t sim_us;
uint64_t sim_status;
uint8_t  sim_reg[SIM_REG_COUNT][SIM_SUB_SPAN];
uint32_t sim_otp[0x800];
uint8_t  sim_tx_frame[128];
uint16_t sim_tx_len;
uint32_t sim_tx_count;
uint8_t  sim_ctrl_log[SIM_CTRL_LOG];
uint32_t sim_ctrl_count;
uint32_t sim_rx_enable_count;
uint32_t sim_rx_soft_resets;
int      sim_fail_spi;
int      sim_force_hpdwarn;
int      sim_force_pll_unlock;
int      sim_pll_lock_event_enabled;
uint32_t sim_pll_failures_remaining;
uint32_t sim_spi_calls;

static uint8_t s_last_softreset;

void sim_write_bytes(uint8_t reg, uint16_t sub, const uint8_t *data, uint16_t len)
{
    memcpy(&sim_reg[reg][sub], data, len);
}

uint32_t sim_read_u32(uint8_t reg, uint16_t sub)
{
    const uint8_t *p = &sim_reg[reg][sub];
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

void sim_reset(void)
{
    memset(sim_reg, 0, sizeof(sim_reg));
    memset(sim_otp, 0, sizeof(sim_otp));
    sim_us = 1000000U;
    sim_status = 1ULL << 1;          /* CPLOCK */
    sim_tx_len = 0U;
    sim_tx_count = 0U;
    sim_ctrl_count = 0U;
    sim_rx_enable_count = 0U;
    sim_rx_soft_resets = 0U;
    sim_fail_spi = 0;
    sim_force_hpdwarn = 0;
    sim_force_pll_unlock = 0;
    sim_pll_lock_event_enabled = 1;
    sim_pll_failures_remaining = 0U;
    sim_spi_calls = 0U;
    s_last_softreset = 0U;
    const uint8_t dev_id[4] = { 0x30, 0x01, 0xCA, 0xDE };
    sim_write_bytes(DW_REG_DEV_ID, 0, dev_id, 4);
    dw1000_irq_flag = 0U;
}

uint8_t sim_last_ctrl(void)
{
    return sim_ctrl_count ? sim_ctrl_log[(sim_ctrl_count - 1U) % SIM_CTRL_LOG] : 0U;
}

static void put_ts(uint8_t reg, uint64_t ts)
{
    for (int i = 0; i < 5; i++)
        sim_reg[reg][i] = (uint8_t)(ts >> (8 * i));
}

void sim_complete_tx(uint64_t ts)
{
    put_ts(DW_REG_TX_TIME, ts);
    sim_status |= DW_TXFRS_BIT;
    dw1000_irq_flag = 1U;
}

void sim_set_rx_quality(uint16_t rxpacc, uint16_t f1, uint16_t f2, uint16_t f3,
                        uint16_t cir_pwr, uint16_t std_noise)
{
    uint32_t finfo = sim_read_u32(DW_REG_RX_FINFO, 0);
    finfo = (finfo & 0x000FFFFFU) | ((uint32_t)rxpacc << 20) | (1U << 18); /* RXPSR */
    uint8_t b[4] = { (uint8_t)finfo, (uint8_t)(finfo >> 8), (uint8_t)(finfo >> 16),
                     (uint8_t)(finfo >> 24) };
    sim_write_bytes(DW_REG_RX_FINFO, 0, b, 4);
    uint8_t fq[8] = { (uint8_t)std_noise, (uint8_t)(std_noise >> 8),
                      (uint8_t)f2, (uint8_t)(f2 >> 8), (uint8_t)f3, (uint8_t)(f3 >> 8),
                      (uint8_t)cir_pwr, (uint8_t)(cir_pwr >> 8) };
    sim_write_bytes(DW_REG_RX_FQUAL, 0, fq, 8);
    uint8_t a1[2] = { (uint8_t)f1, (uint8_t)(f1 >> 8) };
    sim_write_bytes(DW_REG_RX_TIME, 7, a1, 2);
}

void sim_deliver_rx(const uint8_t *frame, uint16_t len, uint64_t rx_ts,
                    uint64_t extra_status)
{
    const uint16_t rx_len = (uint16_t)(len + 2U);
    memcpy(&sim_reg[DW_REG_RX_BUFFER][0], frame, len);
    sim_reg[DW_REG_RX_BUFFER][len] = 0xAB;       /* FCS placeholder */
    sim_reg[DW_REG_RX_BUFFER][len + 1U] = 0xCD;
    uint32_t finfo = sim_read_u32(DW_REG_RX_FINFO, 0);
    if ((finfo >> 20) == 0U)
        finfo |= (200U << 20) | (1U << 18);      /* plausible RXPACC */
    finfo = (finfo & ~0x7FU) | rx_len;
    uint8_t b[4] = { (uint8_t)finfo, (uint8_t)(finfo >> 8), (uint8_t)(finfo >> 16),
                     (uint8_t)(finfo >> 24) };
    sim_write_bytes(DW_REG_RX_FINFO, 0, b, 4);
    put_ts(DW_REG_RX_TIME, rx_ts);
    sim_status |= DW_RXDFR_BIT | DW_RXFCG_BIT | DW_LDEDONE_BIT | extra_status;
    dw1000_irq_flag = 1U;
}

/* ------------------------------------------------------------------------- */
/* uwb_platform.h implementation                                              */
/* ------------------------------------------------------------------------- */

static void handle_write(uint8_t reg, uint16_t sub, const uint8_t *data, uint16_t len)
{
    if (reg == DW_REG_SYS_STATUS)
    {
        for (uint16_t i = 0; i < len && (sub + i) < 5U; i++)
            sim_status &= ~((uint64_t)data[i] << (8U * (sub + i)));
        return;
    }
    if (reg == DW_REG_SYS_CTRL)
    {
        if (sub == 0U && len >= 1U)
        {
            sim_ctrl_log[sim_ctrl_count % SIM_CTRL_LOG] = data[0];
            sim_ctrl_count++;
            if (data[0] & DW_TXSTRT_BIT)
                sim_tx_count++;
            if ((data[0] & DW_TXDLYS_BIT) && sim_force_hpdwarn)
            {
                sim_status |= DW_HPDWARN_BIT;
                sim_force_hpdwarn = 0;
            }
        }
        if ((sub == 0U && len >= 2U && (data[1] & 0x01U))
            || (sub == 1U && (data[0] & 0x01U)))
            sim_rx_enable_count++;
        return;
    }
    if (reg == DW_REG_TX_BUFFER)
    {
        memcpy(&sim_tx_frame[sub], data, len);
        sim_tx_len = (uint16_t)(sub + len);
        return;
    }
    if (reg == DW_REG_OTP_IF && sub == DW_SUB_OTP_CTRL && len >= 1U && data[0] == 0x03U)
    {
        const uint16_t addr = (uint16_t)(sim_reg[DW_REG_OTP_IF][DW_SUB_OTP_ADDR]
                             | (sim_reg[DW_REG_OTP_IF][DW_SUB_OTP_ADDR + 1U] << 8)) & 0x7FFU;
        const uint32_t v = sim_otp[addr];
        uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
        sim_write_bytes(DW_REG_OTP_IF, DW_SUB_OTP_RDAT, b, 4);
        return;
    }
    if (reg == DW_REG_PMSC && sub == DW_SUB_PMSC_SOFTRESET && len == 1U)
    {
        if (s_last_softreset == 0xE0U && data[0] == 0xF0U)
            sim_rx_soft_resets++;
        s_last_softreset = data[0];
    }
    if (reg == DW_REG_FS_CTRL && sub == DW_SUB_FS_PLLTUNE && len >= 1U)
    {
        if (sim_force_pll_unlock || sim_pll_failures_remaining != 0U)
        {
            sim_status &= ~((uint64_t)DW_CPLOCK_BIT);
            if (sim_pll_failures_remaining != 0U)
                sim_pll_failures_remaining--;
        }
        else if (sim_pll_lock_event_enabled)
            sim_status |= DW_CPLOCK_BIT;
    }
    memcpy(&sim_reg[reg][sub], data, len);
}

int uwb_platform_spi_xfer(const uint8_t *header, size_t header_length,
                          const uint8_t *tx, uint8_t *rx, size_t length)
{
    sim_spi_calls++;
    if (sim_fail_spi)
        return -EIO;
    if (header == NULL || header_length == 0U)
        return -EINVAL;

    const uint8_t is_write = (header[0] & 0x80U) != 0U;
    const uint8_t reg = header[0] & 0x3FU;
    uint16_t sub = 0U;
    if (header[0] & 0x40U)
    {
        sub = header[1] & 0x7FU;
        if ((header[1] & 0x80U) && header_length >= 3U)
            sub = (uint16_t)(sub | ((uint16_t)header[2] << 7));
    }

    if (is_write)
    {
        if (tx != NULL && length > 0U)
            handle_write(reg, sub, tx, (uint16_t)length);
        return 0;
    }

    if (rx == NULL)
        return 0;
    if (reg == DW_REG_SYS_STATUS)
    {
        for (size_t i = 0; i < length; i++)
            rx[i] = (uint8_t)(sim_status >> (8U * (sub + i)));
        return 0;
    }
    memcpy(rx, &sim_reg[reg][sub], length);
    return 0;
}

int uwb_platform_init(void) { return 0; }
int uwb_platform_spi_set_frequency(uint32_t hz) { (void)hz; return 0; }
void uwb_platform_cs_set(bool active) { (void)active; }
void uwb_platform_reset_radio(void) {}
void uwb_platform_enable_irq(void) {}
bool uwb_platform_irq_active(void)
{
    return (sim_status & (uint64_t)sim_read_u32(DW_REG_SYS_MASK, 0)) != 0U;
}
void uwb_platform_delay_ms(uint32_t ms) { sim_us += (uint64_t)ms * 1000U; }
void uwb_platform_delay_us(uint32_t us) { sim_us += us; }
uint32_t uwb_platform_time_ms(void) { return (uint32_t)(sim_us / 1000U); }
uint32_t uwb_platform_cycle_now(void) { return (uint32_t)sim_us; }
uint32_t uwb_platform_elapsed_us(uint32_t start) { return (uint32_t)sim_us - start; }
uint64_t uwb_platform_time_us64(void) { return sim_us; }
void uwb_platform_led_toggle(void) {}
void uwb_platform_led_set(bool on) { (void)on; }
bool uwb_platform_fault_led_available(void) { return false; }
void uwb_platform_fault_led_toggle(void) {}
void uwb_platform_fault_led_set(bool on) { (void)on; }
bool uwb_platform_host_led_available(void) { return false; }
void uwb_platform_host_led_set(bool on) { (void)on; }
uint32_t uwb_platform_reset_cause(void) { return UWB_RESET_POR; }
uint32_t uwb_platform_device_id(void) { return 0x12345678U; }
uint32_t uwb_platform_random32(void) { return 0xBEEFU; }
void uwb_platform_reboot(void) {}
