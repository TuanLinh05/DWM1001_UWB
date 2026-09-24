/* DW1000 driver tests against the register-level simulator. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dw1000_hw.h"
#include "dw1000_sim.h"

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

static DW1000_RadioConfig_t s_default_cfg;

static int bring_up(void)
{
    sim_reset();
    if (DW1000_Init() != 0)
        return -1;
    if (DW1000_Configure() != 0)
        return -3;
    DW1000_SetAddress(DW_PAN_ID, 1U);
    if (DW1000_EnableFastSPI() == 0U)
        return -2;
    return 0;
}

static int test_spi_failure_and_truncation(void)
{
    uint8_t data[20];

    sim_reset();
    const uint32_t errors = dw1000_spi_error_count;
    sim_fail_spi = 1;
    CHECK(DW1000_ReadStatus() == 0U);
    /* One register access = one SPI transaction = one error. */
    CHECK(dw1000_spi_error_count == errors + 1U);
    sim_fail_spi = 0;

    memset(data, 0xCC, sizeof(data));
    sim_reg[DW_REG_RX_FINFO][0] = 40;
    CHECK(DW1000_ReadRxData(data, sizeof(data)) == 0U);
    CHECK(data[0] == 0xCC);
    sim_reg[DW_REG_RX_FINFO][0] = 16;
    CHECK(DW1000_ReadRxData(data, sizeof(data)) == 16U);
    return 0;
}

static int test_rx_classification(void)
{
    const uint64_t good = DW_RXDFR_BIT | DW_RXFCG_BIT | DW_LDEDONE_BIT;

    CHECK(DW1000_ClassifyRx(0U) == DW_RX_EVENT_NONE);
    CHECK(DW1000_ClassifyRx(DW_TXFRS_BIT) == DW_RX_EVENT_NONE);
    CHECK(DW1000_ClassifyRx(good) == DW_RX_EVENT_GOOD);
    /* F3: an error flag in the same snapshot rejects the frame. */
    CHECK(DW1000_ClassifyRx(good | DW_LDEERR_BIT) == DW_RX_EVENT_ERROR);
    CHECK(DW1000_ClassifyRx(good | DW_RXOVRR_BIT) == DW_RX_EVENT_ERROR);
    /* Frame without LDEDONE: timestamp not valid. */
    CHECK(DW1000_ClassifyRx(DW_RXDFR_BIT | DW_RXFCG_BIT) == DW_RX_EVENT_ERROR);
    CHECK(DW1000_ClassifyRx(DW_RXDFR_BIT | DW_RXFCE_BIT) == DW_RX_EVENT_ERROR);
    CHECK(DW1000_ClassifyRx(DW_RXPHE_BIT) == DW_RX_EVENT_ERROR);
    return 0;
}

static int test_default_configuration_is_baseline(void)
{
    dw1000_radio_config = s_default_cfg;
    CHECK(bring_up() == 0);
    CHECK(DW1000_VerifyConfig() == 0U);

    /* Defaults must reproduce the deployed register values exactly. */
    CHECK(sim_read_u32(DW_REG_SYS_CFG, 0) == 0x00041200UL);
    CHECK(sim_read_u32(DW_REG_TX_POWER, 0) == 0x1E1E1E1EUL);
    CHECK(sim_read_u32(DW_REG_CHAN_CTRL, 0) == 0x21040055UL);
    CHECK(sim_read_u32(DW_REG_SYS_MASK, 0) == 0x2437D080UL);
    CHECK(sim_read_u32(DW_REG_ACK_RESP_T, 0) == 0U);
    CHECK((sim_read_u32(DW_REG_TX_ANTD, 0) & 0xFFFFU) == UWB_TX_ANT_DLY);
    CHECK((sim_read_u32(DW_REG_LDE_IF, DW_SUB_LDE_RXANTD) & 0xFFFFU) == UWB_RX_ANT_DLY);
    /* Reference tuning off: LDE_CFG1 and FS_XTALT untouched. */
    CHECK(sim_reg[DW_REG_LDE_IF][DW_SUB_LDE_CFG1] == 0U);
    CHECK(sim_reg[DW_REG_FS_CTRL][DW_SUB_FS_XTALT] == 0U);
    CHECK(dw1000_otp.applied_xtal_trim == 0xFFU);

    /* Register drift (e.g. a DW1000 brown-out) is detected. */
    sim_reg[DW_REG_CHAN_CTRL][0] = 0x00;
    CHECK((DW1000_VerifyConfig() & DW_VERIFY_CHAN_CTRL) != 0U);
    return 0;
}

static int test_reference_tuning_and_otp(void)
{
    dw1000_radio_config = s_default_cfg;
    dw1000_radio_config.reference_tuning = 1U;
    sim_reset();
    sim_otp[DW_OTP_XTRIM_ADDR] = 0x0515U;    /* rev 5, trim 0x15 */
    sim_otp[DW_OTP_LDOTUNE_ADDR] = 0x88U;
    sim_otp[DW_OTP_PARTID_ADDR] = 0xA1B2C3D4U;
    sim_otp[DW_OTP_VTEMP_ADDR] = 0x7FU;
    CHECK(DW1000_Init() == 0);
    CHECK(DW1000_Configure() == 0);
    CHECK(dw1000_otp.valid == 1U);
    CHECK(dw1000_otp.xtal_trim == 0x15U && dw1000_otp.otp_rev == 0x05U);
    CHECK(dw1000_otp.part_id == 0xA1B2C3D4U);
    CHECK(sim_reg[DW_REG_FS_CTRL][DW_SUB_FS_XTALT] == (0x60U | 0x15U));
    CHECK(sim_reg[DW_REG_OTP_IF][DW_SUB_OTP_SF] == 0x02U);   /* LDO kick */
    CHECK(sim_reg[DW_REG_LDE_IF][DW_SUB_LDE_CFG1] == 0x6DU);
    CHECK(DW1000_VerifyConfig() == 0U);

    /* Blank OTP: mid-range trim, no LDO kick. */
    sim_reset();
    CHECK(DW1000_Init() == 0);
    CHECK(sim_reg[DW_REG_FS_CTRL][DW_SUB_FS_XTALT] == (0x60U | 0x10U));
    CHECK(dw1000_otp.ldo_kicked == 0U);

    /* Temperature conversion uses the OTP 23 degC point. */
    dw1000_otp.vtemp_cal = 0x80U;
    CHECK(DW1000_TemperatureCentiDeg(0x80U) == 2300);
    CHECK(DW1000_TemperatureCentiDeg(0x8AU) == 2300 + 10 * 114);
    dw1000_radio_config = s_default_cfg;
    return 0;
}

static int test_tx_power_modes(void)
{
    dw1000_radio_config = s_default_cfg;
    dw1000_radio_config.tx_power_mode = UWB_TX_POWER_REFERENCE;
    CHECK(bring_up() == 0);
    CHECK(sim_read_u32(DW_REG_TX_POWER, 0) == 0x48484848UL);
    CHECK((sim_read_u32(DW_REG_SYS_CFG, 0) & (1UL << 18)) != 0U);
    CHECK(DW1000_VerifyConfig() == 0U);

    dw1000_radio_config.tx_power_mode = UWB_TX_POWER_SMART;
    CHECK(bring_up() == 0);
    CHECK(sim_read_u32(DW_REG_TX_POWER, 0) == 0x0E082848UL);
    CHECK((sim_read_u32(DW_REG_SYS_CFG, 0) & (1UL << 18)) == 0U);   /* smart on */
    CHECK(DW1000_VerifyConfig() == 0U);
    dw1000_radio_config = s_default_cfg;
    return 0;
}

static int test_pll_lock_and_calibration_profile(void)
{
    dw1000_radio_config = s_default_cfg;

    /* CPLOCK is an event latch. A radio that is already locked need not
     * generate a second edge when the same PLL tuning value is written. */
    sim_reset();
    CHECK(DW1000_Init() == 0);
    sim_pll_lock_event_enabled = 0;
    CHECK((sim_status & DW_CPLOCK_BIT) != 0U);
    CHECK(DW1000_Configure() == 0);
    CHECK((sim_status & DW_CPLOCK_BIT) != 0U);

    sim_reset();
    CHECK(DW1000_Init() == 0);
    sim_force_pll_unlock = 1;
    sim_status &= ~((uint64_t)DW_CPLOCK_BIT);
    const uint64_t started_us = sim_us;
    CHECK(DW1000_Configure() == -1);
    CHECK(sim_us - started_us >= 105000U);
    CHECK(sim_read_u32(DW_REG_SYS_MASK, 0) == 0U); /* IRQs stay disabled */

    sim_reset();
    CHECK(DW1000_Init() == 0);
    const uint32_t baseline = DW1000_CalibrationProfileId();
    dw1000_radio_config.tx_ant_dly++;
    CHECK(DW1000_CalibrationProfileId() != baseline);
    dw1000_radio_config = s_default_cfg;
    dw1000_radio_config.tx_power_mode = UWB_TX_POWER_REFERENCE;
    CHECK(DW1000_CalibrationProfileId() != baseline);
    dw1000_radio_config = s_default_cfg;
    return 0;
}

static int test_signal_diagnostics(void)
{
    DW1000_SignalDiag_t diag;

    sim_reset();
    sim_set_rx_quality(200U, 1000U, 1000U, 1000U, 5000U, 40U);
    DW1000_ReadSignalDiag(&diag);
    /* RXPACC lives in RX_FINFO[31:20]; the baseline read [27:18] (~4x). */
    CHECK(diag.rxpacc == 200U);
    CHECK(diag.fp_ampl1 == 1000U && diag.fp_ampl2 == 1000U && diag.fp_ampl3 == 1000U);
    CHECK(diag.cir_pwr == 5000U && diag.std_noise == 40U);

    const float fpp = DW1000_GetFirstPathPower(&diag);
    const float expected = 10.0f * log10f(3.0e6f / (200.0f * 200.0f)) - 113.77f;
    CHECK(fabsf(fpp - expected) < 0.01f);
    const float rx = DW1000_GetRxPower(&diag);
    CHECK(rx > fpp);   /* total power >= first path */

    /* Carrier integrator: 21-bit sign extension and Decawave sign convention. */
    const uint8_t ci_neg1[3] = { 0xFF, 0xFF, 0x1F };
    sim_write_bytes(DW_REG_DRX_CONF, DW_SUB_DRX_CAR_INT, ci_neg1, 3);
    CHECK(DW1000_ReadCarrierIntegrator() == -1);
    CHECK(DW1000_CarrierIntegratorToPpmX100(1000) == -57);
    return 0;
}

static int test_status_and_reset_helpers(void)
{
    sim_reset();
    sim_status = DW_TXFRS_BIT | DW_RXFCG_BIT | DW_RXDFR_BIT;
    DW1000_ClearTxStatus();
    CHECK((sim_status & DW_TXFRS_BIT) == 0U);
    CHECK((sim_status & (DW_RXFCG_BIT | DW_RXDFR_BIT)) == (DW_RXFCG_BIT | DW_RXDFR_BIT));
    DW1000_ClearAllStatus();
    CHECK(sim_status == 0U);

    DW1000_RxSoftReset();
    CHECK(sim_rx_soft_resets == 1U);

    DW1000_StartTxWait4Resp();
    CHECK(sim_last_ctrl() == (DW_TXSTRT_BIT | DW_WAIT4RESP_BIT));
    sim_status = DW_HPDWARN_BIT;
    CHECK(DW1000_StartTxDelayedEx(1U) == -1);
    CHECK(sim_last_ctrl() == (DW_TXSTRT_BIT | DW_TXDLYS_BIT | DW_WAIT4RESP_BIT));
    return 0;
}

int main(void)
{
    s_default_cfg = dw1000_radio_config;

    if (test_spi_failure_and_truncation()
        || test_rx_classification()
        || test_default_configuration_is_baseline()
        || test_reference_tuning_and_otp()
        || test_tx_power_modes()
        || test_pll_lock_and_calibration_profile()
        || test_signal_diagnostics()
        || test_status_and_reset_helpers())
    {
        return 1;
    }
    puts("driver tests passed (SPI, RX, config, PLL, profile identity, OTP, TX power, diagnostics)");
    return 0;
}
