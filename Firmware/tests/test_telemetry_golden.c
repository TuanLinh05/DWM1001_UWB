/*
 * Emit one frame of every telemetry type with known values using the real
 * firmware encoders. test_telemetry_golden.py decodes the file with the GUI
 * decoder and checks every field, so C encoder and Python decoder can never
 * drift apart silently.
 */
#include <stdio.h>
#include <string.h>

#include "dw1000_sim.h"
#include "uart_tx.h"
#include "uwb_cmd.h"
#include "uwb_health.h"
#include "uwb_settings.h"
#include "../common/src/ranging/tag_ranging.c"
#include "telemetry.h"

UwbHealth_t uwb_health;
uint8_t uwb_settings_status;
volatile uint32_t uart_tx_overflow_count;
volatile uint32_t uart_rx_overflow_count;
volatile uint16_t uart_tx_high_water;

static uint8_t s_out[8192];
static size_t s_out_len;

uint32_t UART_TX_Write(const uint8_t *data, uint16_t length)
{
    if (s_out_len + length > sizeof(s_out))
        return 0U;
    memcpy(&s_out[s_out_len], data, length);
    s_out_len += length;
    return length;
}

uint16_t UART_TX_Pending(void) { return 0U; }

int main(int argc, char **argv)
{
    if (argc < 2)
        return 2;

    sim_reset();
    if (Tag_Init() != 0)
        return 3;
    (void)Tag_SetDsCalibration(1U, 123456, 1U);
    Tag_SetActiveAnchorMask(0x0FU);

    /* INFO */
    Telem_SendInfo();

    /* RANGE snapshot */
    s_track[0].raw_mm = 5000;
    s_track[0].filtered_mm = 4990;
    s_track[0].fpp_cdbm = -8123;
    s_track[0].valid = 1U;
    s_track[0].status = 0U;
    s_track[0].last_success_tick = uwb_platform_time_ms();
    tag_sample_seq = 77U;
    TagCycleSnapshot_t snap;
    Tag_GetSnapshot(&snap);
    Telem_SendRangeCycle(&snap);

    /* STATS */
    poll_sent_count = 1000U;
    response_ok_count = 900U;
    rx_timeout_count = 50U;
    rx_error_count = 5U;
    cycle_overrun_count = 2U;
    uart_tx_overflow_count = 1U;
    Telem_SendStats(50U, 180U);

    /* RANGE_MEAS */
    uwb_health.boot_id = 0xBEEFU;
    const TagMeasurement_t m = {
        .meas_time_us = 123456789012ULL, .meas_seq = 42U, .anchor_id = 3U, .txn = 0x5AU,
        .mode = TAG_MEAS_MODE_DS, .flags = 0x000BU, .status = 0U, .raw_mm = 5100,
        .corrected_mm = 5000, .filtered_mm = 4995, .fp_cdbm = -8200, .rx_cdbm = -7600,
        .anchor_fp_cdbm = -8300, .anchor_rx_cdbm = -7700, .std_noise = 35U,
        .fp_index = 0x2A40U, .ci_ppm_x100 = -57, .slot_us = 3100U,
    };
    Telem_SendRangeMeas(&m);

    /* DIAG_ANCHOR for A2 */
    anchor_success_count[1] = 11U;
    anchor_response_timeout_count[1] = 12U;
    anchor_poll_tx_timeout_count[1] = 13U;
    anchor_rx_error_count[1] = 14U;
    anchor_poll_skipped_count[1] = 15U;
    anchor_calibration_missing_count[1] = 16U;
    anchor_ds_ok_count[1] = 17U;
    anchor_report_timeout_count[1] = 18U;
    anchor_ds_fallback_count[1] = 19U;
    anchor_txn_mismatch_count[1] = 20U;
    anchor_probe_count[1] = 21U;
    anchor_response_timeout_streak[1] = 3U;
    anchor_ds_incomplete_streak[1] = 1U;
    anchor_slot_duration_us[1] = 3300U;
    anchor_slot_duration_max_us[1] = 4100U;
    anchor_response_wait_max_us[1] = 1500U;
    anchor_processing_max_us[1] = 420U;
    anchor_poll_tx_duration_max_us[1] = 330U;
    anchor_burst_late_count[1] = 22U;
    Telem_SendDiagAnchor(1U);

    /* ANCHOR_INFO */
    const TagAnchorInfo_t info = {
        .anchor_id = 4U, .valid = 1U, .anchor_status = 1U, .pos_valid = 1U,
        .build_dirty = 1U, .tx_power_mode = 0U, .pos_mm = { 1000, -2000, 2500 },
        .build_hash = 0x08D28188U, .tx_ant_dly = 16436U, .rx_ant_dly = 16437U,
        .boot_count = 3U, .build_config_hash = 0xAABBCCDDU,
    };
    Telem_SendAnchorInfo(&info);

    /* CMD_ACK */
    const uint8_t ack_data[3] = { 1U, 2U, 3U };
    Telem_SendCmdAck(77U, 0x07U, 0U, ack_data, 3U);

    /* DIAG_SYSTEM */
    uwb_health.boot_count = 2U;
    uwb_health.reset_cause = UWB_RESET_WATCHDOG;
    uwb_health.radio_recoveries = 4U;
    uwb_health.config_checks = 99U;
    uwb_health.last_fault_cause = UWB_FAULT_SPI;
    uwb_cmd_stats.executed = 5U;
    uwb_cmd_stats.rejected = 6U;
    uwb_cmd_stats.crc_errors = 7U;
    uwb_cmd_stats.locked = 1U;
    uwb_settings_status = 0x03U;
    tag_rx_error_stats.lde = 8U;
    tag_rx_error_stats.soft_resets = 9U;
    ds_ok_count = 1234U;
    dw1000_otp.valid = 1U;
    dw1000_otp.vtemp_cal = 0x80U;
    dw1000_otp.vbat_cal = 0xA0U;
    Telem_SetEnvironment(0x8AU, 0xA0U);     /* 34.4 degC, 3.300 V */
    const TagBurstTiming_t timing = {
        .base_uus = 700U, .slot_uus = 300U, .final_margin_uus = 400U,
        .gap_us = 250U, .period_us = 5000U,
    };
    if (Tag_SetBurstTiming(&timing) != 0)
        return 5;
    tag_burst_final_late_count = 23U;
    tag_burst_record_drops = 24U;
    Telem_SendDiagSystem();

    /* DEVICE_INFO */
    dw1000_otp.part_id = 0xA1B2C3D4U;
    dw1000_otp.xtal_trim = 0x15U;
    uwb_health.device_id = 0x12345678U;
    Telem_SendDeviceInfo();

    /* RANGE_BURST: calibrated DS, late anchor on SS fallback, timeout. */
    TagBurstRecord_t burst;
    memset(&burst, 0, sizeof(burst));
    burst.cycle_seq = 5000U;
    burst.meas_seq_first = 900U;
    burst.t_us = 987654321012ULL;
    burst.period_us = 4480U;
    burst.count = 3U;
    burst.anchor[0] = (TagBurstAnchor_t){
        .anchor_id = 1U, .status = TAG_ST_OK, .range_mm = 4321U,
        .flags = TAG_BURST_REC_RADIO_OK | TAG_BURST_REC_CAL_OK | TAG_BURST_REC_VALID,
        .fp_cdbm = -8212, .rx_cdbm = -7650, .ci_ppm_x100 = -1837,
    };
    burst.anchor[1] = (TagBurstAnchor_t){
        .anchor_id = 5U, .status = TAG_ST_DS_FALLBACK, .range_mm = 12345U,
        .flags = 2U | TAG_BURST_REC_RADIO_OK | TAG_BURST_REC_ANCHOR_LATE,
        .fp_cdbm = INT16_MIN, .rx_cdbm = -7000, .ci_ppm_x100 = INT16_MIN,
    };
    burst.anchor[2] = (TagBurstAnchor_t){
        .anchor_id = 8U, .status = TAG_ST_TIMEOUT, .range_mm = 0xFFFFU,
        .flags = TAG_BURST_REC_MODE_NONE,
        .fp_cdbm = INT16_MIN, .rx_cdbm = INT16_MIN, .ci_ppm_x100 = INT16_MIN,
    };
    Telem_SendRangeBurst(&burst);
    burst.period_us = 70000U;                /* saturates at 65535 */
    burst.count = 0U;
    burst.cycle_seq = 5001U;
    Telem_SendRangeBurst(&burst);

    FILE *f = fopen(argv[1], "wb");
    if (f == NULL)
        return 4;
    fwrite(s_out, 1, s_out_len, f);
    fclose(f);
    printf("telemetry golden file written (%zu bytes)\n", s_out_len);
    return 0;
}
