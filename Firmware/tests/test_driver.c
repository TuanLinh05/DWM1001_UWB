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
    sim_reg[DW_REG_RX_BUFFER][0] = 0x41U;
    sim_reg[DW_REG_RX_FINFO][0] = 29U;
    CHECK(DW1000_PeekRxPrefix(data, 12U) == 29U && data[0] == 0x41U);
    sim_reg[DW_REG_RX_FINFO][0] = 13U;
    CHECK(DW1000_PeekRxPrefix(data, 12U) == 0U);
    sim_reg[DW_REG_RX_FINFO][0] = 29U;
    sim_reg[DW_REG_RX_FINFO][1] = 1U; /* extended frame >127 bytes */
    CHECK(DW1000_PeekRxPrefix(data, 12U) == 0U);
    sim_reg[DW_REG_RX_FINFO][1] = 0U;
    sim_fail_spi = 1;
    CHECK(DW1000_PeekRxPrefix(data, 12U) == 0U);
    sim_fail_spi = 0;
    CHECK(DW1000_PeekRxPrefix(NULL, 12U) == 0U);
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
    sim_reg[DW_REG_ACK_RESP_T][3] = 0xA5U;
    DW1000_SetRxAfterTxDelay(0x12345U);
    CHECK(sim_read_u32(DW_REG_ACK_RESP_T,0) == 0xA5012345UL);
    DW1000_SetRxAfterTxDelay(0U);
    CHECK(sim_read_u32(DW_REG_ACK_RESP_T,0) == 0xA5000000UL);
    return 0;
}

static int test_split_rx_capture_preserves_metadata(void)
{
    sim_reset();
    const uint8_t frame[] = { 0x41U, 0x88U, 0xAAU, 0x55U };
    const uint64_t expected_ts = 0x123456789AULL;
    sim_set_rx_quality(321U, 1234U, 2345U, 3456U, 4567U, 89U);
    sim_deliver_rx(frame, sizeof(frame), expected_ts, 0U);
    uint8_t data[20], ts[5];
    uint32_t finfo;
    DW1000_SignalDiag_t diag;
    const uint32_t calls = sim_spi_calls;
    CHECK(DW1000_ReadRxPayload(data, sizeof(data), &finfo) == sizeof(frame) + 2U);
    CHECK(memcmp(data, frame, sizeof(frame)) == 0);
    CHECK(sim_spi_calls == calls + 2U);
    /* Arm delayed TX between the two stages, as the last burst slot does. */
    const uint8_t dx[5] = { 0U, 2U, 3U, 4U, 5U };
    DW1000_ForceRxOff();
    DW1000_ClearAllStatus();
    DW1000_SetDelayedTxTime(dx);
    CHECK(DW1000_StartTxDelayedEx(0U) == 0);
    const uint32_t meta_calls = sim_spi_calls;
    CHECK(DW1000_ReadRxMetadata(finfo, ts, &diag));
    CHECK(sim_spi_calls == meta_calls + 2U); /* no repeated FINFO read */
    uint64_t actual_ts = 0U;
    for (unsigned i = 0U; i < 5U; i++)
        actual_ts |= (uint64_t)ts[i] << (8U * i);
    CHECK(actual_ts == expected_ts);
    CHECK(diag.rxpacc == 321U && diag.fp_ampl1 == 1234U && diag.fp_ampl2 == 2345U);
    CHECK(diag.fp_ampl3 == 3456U && diag.cir_pwr == 4567U && diag.std_noise == 89U);
    sim_fail_spi = 1;
    CHECK(!DW1000_ReadRxMetadata(finfo, ts, &diag));
    CHECK(DW1000_ReadRxPayload(data, sizeof(data), &finfo) == 0U);
    sim_fail_spi = 0;
    sim_reg[DW_REG_RX_FINFO][1] |= 1U;  /* extended length must not wrap to 7 bits */
    CHECK(DW1000_ReadRxPayload(data, sizeof(data), &finfo) == 0U);
    CHECK(DW1000_ReadRxPayload(NULL, sizeof(data), &finfo) == 0U);
    CHECK(DW1000_ReadRxPayload(data, sizeof(data), NULL) == 0U);
    CHECK(!DW1000_ReadRxMetadata(finfo, NULL, &diag));
    return 0;
}

static int test_signal_power_accuracy_sweep(void)
{
    uint32_t random = 0xC0FFEEU;
    for (unsigned k=0U; k<20000U; k++)
    {
        random = random*1664525U + 1013904223U;
        const DW1000_SignalDiag_t d = {
            .rxpacc=(uint16_t)(1U+(random&0xFFFU)),
            .fp_ampl1=(uint16_t)(1U+(random&0x7FFFU)),
            .fp_ampl2=(uint16_t)(random>>16),
            .fp_ampl3=(uint16_t)(random>>8),
            .cir_pwr=(uint16_t)(1U+((random>>16)&0x7FFFU)),
        };
        const float f1=d.fp_ampl1, f2=d.fp_ampl2, f3=d.fp_ampl3, n=d.rxpacc;
        const float fp=10.0f*log10f((f1*f1+f2*f2+f3*f3)/(n*n))-UWB_FPP_A_CONST;
        const float rx=10.0f*log10f((d.cir_pwr*131072.0f)/(n*n))-UWB_FPP_A_CONST;
        CHECK(fabsf(DW1000_GetFirstPathPower(&d)-fp)<0.0002f);
        CHECK(fabsf(DW1000_GetRxPower(&d)-rx)<0.0002f);
    }
    return 0;
}

static int test_integer_clock_diagnostic_rounding(void)
{
    for (int32_t ci = -1048576; ci < 1048576; ci++)
    {
        const double reference = (double)ci * UWB_CLOCK_OFFSET_MULT * 1e8;
        const int32_t actual = DW1000_CarrierIntegratorToPpmX100(ci);
        CHECK(fabs((double)actual - reference) <= 0.500001);
    }
    CHECK(DW1000_CarrierIntegratorToPpmX100(15625) == -896);
    CHECK(DW1000_CarrierIntegratorToPpmX100(-15625) == 896);
    CHECK(DW1000_CarrierIntegratorToPpmX100(INT32_MIN) > 0);
    CHECK(DW1000_CarrierIntegratorToPpmX100(INT32_MAX) < 0);
    return 0;
}

static void inject_second_rx(uint8_t reg, uint16_t sub, uint8_t writing)
{
    (void)sub;
    if (!writing && reg == DW_REG_RX_BUFFER)
    {
        sim_spi_hook = NULL;
        const uint8_t second[] = {0x22U, 0x33U, 0x44U};
        sim_set_rx_quality(654U, 4444U, 5555U, 6666U, 7777U, 98U);
        sim_deliver_rx(second, sizeof(second), 0x9988776655ULL, 0U);
    }
}

static int test_double_buffer_owns_payload_timestamp_and_quality(void)
{
    CHECK(bring_up() == 0);
    DW1000_SetDoubleRxBuffer(1U);
    CHECK(DW1000_DoubleRxBufferEnabled());
    CHECK((sim_read_u32(DW_REG_SYS_CFG, 0) & (1UL << 12)) == 0U);
    CHECK((sim_read_u32(DW_REG_SYS_CFG, 0) & (1UL << 29)) == 0U); /* no RXAUTR */
    CHECK(DW1000_VerifyConfig() == 0U);
    const uint8_t first[] = {0x11U, 0xAAU};
    sim_set_rx_quality(321U, 1234U, 2345U, 3456U, 4567U, 89U);
    sim_deliver_rx(first, sizeof(first), 0x123456789AULL, 0U);
    DW1000_ClearRxGoodStatus();
    DW1000_StartRx();
    sim_spi_hook = inject_second_rx; /* arrives BEFORE reading first payload */
    uint8_t data[20], ts[5];
    uint32_t finfo;
    DW1000_SignalDiag_t diag;
    CHECK(DW1000_ReadRxPayload(data, sizeof(data), &finfo) == sizeof(first) + 2U);
    CHECK(memcmp(data, first, sizeof(first)) == 0);
    CHECK(DW1000_ReadRxMetadata(finfo, ts, &diag));
    const uint8_t first_ts[] = {0x9AU,0x78U,0x56U,0x34U,0x12U};
    CHECK(memcmp(ts, first_ts, sizeof(ts)) == 0);
    CHECK(diag.rxpacc == 321U && diag.fp_ampl1 == 1234U && diag.cir_pwr == 4567U);
    CHECK(DW1000_ClassifyRx(DW1000_ReadStatus()) == DW_RX_EVENT_GOOD);
    DW1000_ReleaseRxBuffer();
    /* Releasing the first buffer must not clear the second event. */
    CHECK(DW1000_ClassifyRx(DW1000_ReadStatus()) == DW_RX_EVENT_GOOD);
    CHECK(DW1000_ReadRxPayload(data, sizeof(data), &finfo) == 5U && data[0] == 0x22U);
    CHECK(DW1000_ReadRxMetadata(finfo, ts, &diag));
    CHECK(ts[0] == 0x55U && ts[4] == 0x99U);
    CHECK(diag.rxpacc == 654U && diag.fp_ampl1 == 4444U && diag.cir_pwr == 7777U);
    DW1000_ClearRxGoodStatus();
    DW1000_ReleaseRxBuffer();
    CHECK(sim_rx_buffer_releases == 2U);
    CHECK(((sim_status >> 30) & 1U) == ((sim_status >> 31) & 1U));
    sim_deliver_rx(first, sizeof(first), 123U, 0U);
    DW1000_ForceRxOff();
    DW1000_SyncRxBuffers(); /* abandon pending frame at a cycle/error boundary */
    CHECK(sim_rx_buffer_releases == 3U);
    CHECK(((sim_status >> 30) & 1U) == ((sim_status >> 31) & 1U));
    DW1000_SetDoubleRxBuffer(0U);
    CHECK(!DW1000_DoubleRxBufferEnabled());
    CHECK(DW1000_VerifyConfig() == 0U);
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
        || test_split_rx_capture_preserves_metadata()
        || test_double_buffer_owns_payload_timestamp_and_quality()
        || test_integer_clock_diagnostic_rounding()
        || test_status_and_reset_helpers() || test_signal_power_accuracy_sweep())
    {
        return 1;
    }
    puts("driver tests passed (SPI, RX, config, PLL, profile identity, OTP, TX power, diagnostics)");
    return 0;
}
