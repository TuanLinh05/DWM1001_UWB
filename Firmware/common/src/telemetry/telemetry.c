/**
 ******************************************************************************
 * @file    telemetry.c
 * @brief   TAG telemetry — binary packets / ASCII CSV (see telemetry.h)
 ******************************************************************************
 */
#include "telemetry.h"
#include "uart_tx.h"
#include "uwb_calibration.h"
#include "uwb_cmd.h"
#include "uwb_health.h"
#include "uwb_platform.h"
#include "uwb_settings.h"
#include <limits.h>
#include <stdio.h>

#if defined(__ZEPHYR__)
#include <zephyr/devicetree.h>
#define TELEM_UART_BAUD  DT_PROP(DT_CHOSEN(zephyr_console), current_speed)
#else
#define TELEM_UART_BAUD  115200U
#endif

#ifndef UWB_BUILD_GIT_HASH
#define UWB_BUILD_GIT_HASH  0
#endif
#ifndef UWB_BUILD_GIT_DIRTY
#define UWB_BUILD_GIT_DIRTY 1
#endif
#ifndef UWB_BUILD_CONFIG_HASH
#define UWB_BUILD_CONFIG_HASH 0
#endif

#define TELEM_RANGE_RECORD_LEN    16U
#define TELEM_RANGE_PAYLOAD_MAX   (1U + TAG_NUM_ANCHORS * TELEM_RANGE_RECORD_LEN)
#define TELEM_RANGE_PKT           (TELEM_HDR_LEN + TELEM_RANGE_PAYLOAD_MAX + TELEM_CRC_LEN)
#define TELEM_UART_BITS_PER_BYTE  10U     /* 8N1. */
#define TELEM_CYCLE_HZ            (1000U / TAG_CYCLE_MS)

#define TELEM_SCHEMA_V2           1U     /* schema byte of every 0x1x payload */
#define TELEM_ROLE_TAG            1U

_Static_assert(TELEM_MAX_FRAME <= UART_TX_BUF_SIZE,
               "Telemetry packet exceeds UART TX ring capacity");
_Static_assert((1000U % TAG_CYCLE_MS) == 0U,
               "TAG cycle period must produce an integer telemetry rate");
_Static_assert(
    (TELEM_RANGE_PKT * TELEM_CYCLE_HZ * TELEM_UART_BITS_PER_BYTE)
        <= ((TELEM_UART_BAUD * 8U) / 10U),
    "Range snapshots exceed 80 percent of the UART budget");

static uint8_t  s_features = (uint8_t)UWB_TELEM_DEFAULT_FEATURES;
static uint32_t s_frame_seq;
static uint8_t  s_temp_raw;
static uint8_t  s_vbat_raw;
static uint8_t  s_env_valid;

uint32_t Telem_UartBaud(void)
{
    return (uint32_t)TELEM_UART_BAUD;
}

int Telem_SetFeatures(uint8_t features)
{
    if ((features & TELEM_FEATURE_RANGE_MEAS) != 0U
        && Telem_UartBaud() < TELEM_RANGE_MEAS_MIN_BAUD)
    {
        return -1;
    }
    s_features = (uint8_t)(features & (TELEM_FEATURE_RANGE_SNAPSHOT
                                       | TELEM_FEATURE_RANGE_MEAS
                                       | TELEM_FEATURE_DIAG));
    return 0;
}

uint8_t Telem_GetFeatures(void)
{
    /* A RANGE_MEAS default that the UART cannot carry is dropped. */
    if ((s_features & TELEM_FEATURE_RANGE_MEAS) != 0U
        && Telem_UartBaud() < TELEM_RANGE_MEAS_MIN_BAUD)
    {
        s_features = (uint8_t)(s_features & ~TELEM_FEATURE_RANGE_MEAS);
    }
    return s_features;
}

void Telem_SetEnvironment(uint8_t temp_raw, uint8_t vbat_raw)
{
    s_temp_raw = temp_raw;
    s_vbat_raw = vbat_raw;
    s_env_valid = 1U;
}

static void send_frame(uint8_t *buf, uint8_t type, uint32_t seq, uint16_t payload_len)
{
    const uint16_t total = telem_frame_pack(buf, type, seq,
                                            uwb_platform_time_ms(), payload_len);
    (void)UART_TX_Write(buf, total);
}

static uint8_t info_flags(void)
{
    uint8_t flags = TELEM_INFO_FLAG_FPP_CORRECTED;
#if UWB_USE_HW_ANTENNA_DELAY
    flags |= TELEM_INFO_FLAG_HW_ANTENNA_DELAY;
#endif
#if UWB_USE_LEGACY_OFFSET
    flags |= TELEM_INFO_FLAG_LEGACY_OFFSET;
#endif
#if UWB_USE_DS_TWR
    flags |= TELEM_INFO_FLAG_DS_BUILD;
#endif
    flags |= (uint8_t)(((uint8_t)UWB_LEGACY_ADAPTIVE_MODE <<
                        TELEM_INFO_FLAG_LEGACY_ADAPTIVE_SHIFT) &
                       TELEM_INFO_FLAG_LEGACY_ADAPTIVE_MASK);
    return flags;
}

/** Offset currently subtracted from anchor i, in micrometres. */
static int32_t active_offset_um(uint8_t index)
{
#if UWB_USE_DS_TWR
    int32_t bias_um = 0;
    (void)Tag_GetDsCalibration((uint16_t)(index + 1U), &bias_um, NULL);
    return bias_um;
#elif UWB_USE_LEGACY_OFFSET
    return (int32_t)(calibration_offset_m[index] * 1000000.0);
#else
    return (int32_t)(residual_offset_m[index] * 1000000.0);
#endif
}

/* ========================================================================== */
/*                     INFO (0x00)                                             */
/* ========================================================================== */

/*
 * schema[1]=2 flags[1] ranging_mode[1] num_rec[1] ds_calibrated_mask[1]
 * filter_mode[1] phy_profile_id[1] spi_clock_mhz[1] c9_2_motion_mode[1]
 * c9_2_global_motion_state[1] + N × (anchor_id[2] active_offset_um[4])
 */
void Telem_SendInfo(void)
{
    const uint8_t flags = info_flags();
#if TELEM_ASCII
    char line[512];
    int n = snprintf(line, sizeof(line), "I,2,%u,%u,%u,%u,%u,%u,%u",
        (unsigned)flags,
        (unsigned)uwb_ds_mode_enabled_build,
        (unsigned)TAG_NUM_ANCHORS,
        (unsigned)Tag_DsCalibratedMask(),
        (unsigned)UWB_RANGE_FILTER_MODE,
        (unsigned)uwb_c9_2_motion_mode_build,
        (unsigned)c9_2_global_motion_state);
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS && n > 0 && n < (int)sizeof(line); i++)
    {
        n += snprintf(line + n, sizeof(line) - n, ",%u,%ld",
            (unsigned)(i + 1), (long)active_offset_um(i));
    }
    if (n > 0 && n < (int)sizeof(line) - 2)
    {
        line[n++] = '\r';
        line[n++] = '\n';
        UART_TX_Write((const uint8_t *)line, (uint16_t)n);
    }
#else
    uint8_t buf[TELEM_MAX_FRAME];
    uint16_t off = TELEM_HDR_LEN;
    buf[off++] = TELEM_INFO_SCHEMA;
    buf[off++] = flags;
    buf[off++] = uwb_ds_mode_enabled_build;       /* configured ranging mode */
    buf[off++] = TAG_NUM_ANCHORS;
    buf[off++] = Tag_DsCalibratedMask();          /* runtime, not build-time */
    buf[off++] = (uint8_t)UWB_RANGE_FILTER_MODE;
    buf[off++] = (uint8_t)DW_PHY_PROFILE_ID;
    buf[off++] = dw1000_spi_mhz;
    buf[off++] = uwb_c9_2_motion_mode_build;
    buf[off++] = c9_2_global_motion_state;
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        off += telem_put_u16(&buf[off], (uint16_t)(i + 1));
        off += telem_put_u32(&buf[off], (uint32_t)active_offset_um(i));
    }
    send_frame(buf, TELEM_TYPE_INFO, tag_sample_seq, (uint16_t)(off - TELEM_HDR_LEN));
#endif
}

/* ========================================================================== */
/*                     RANGE snapshot (0x01)                                   */
/* ========================================================================== */

/* num_rec[1] + N × (anchor_id[2] valid[1] status[1] age_ms[2] raw_mm[4]
 *                   filtered_mm[4] fpp_cdbm[2]) */
void Telem_SendRangeCycle(const TagCycleSnapshot_t *snap)
{
#if TELEM_ASCII
    char line[512];
    int n = snprintf(line, sizeof(line),
        "R2,%lu,%lu",
        (unsigned long)snap->seq, (unsigned long)snap->time_ms);
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS && n > 0 && n < (int)sizeof(line); i++)
    {
        const TagAnchorSample_t *a = &snap->anchor[i];
        n += snprintf(line + n, sizeof(line) - n,
            ",%u,%u,%u,%u,%ld,%ld,%d",
            (unsigned)a->anchor_id, (unsigned)a->valid, (unsigned)a->status,
            (unsigned)a->age_ms,
            (long)a->raw_mm, (long)a->filtered_mm, (int)a->fpp_cdbm);
    }
    if (n > 0 && n < (int)sizeof(line) - 2)
    {
        line[n++] = '\r';
        line[n++] = '\n';
        UART_TX_Write((const uint8_t *)line, (uint16_t)n);
    }
#else
    uint8_t buf[TELEM_MAX_FRAME];
    uint16_t off = TELEM_HDR_LEN;
    buf[off++] = TAG_NUM_ANCHORS;                 /* num_rec */
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        const TagAnchorSample_t *a = &snap->anchor[i];
        off += telem_put_u16(&buf[off], a->anchor_id);
        buf[off++] = a->valid;
        buf[off++] = a->status;
        off += telem_put_u16(&buf[off], a->age_ms);
        off += telem_put_u32(&buf[off], (uint32_t)a->raw_mm);
        off += telem_put_u32(&buf[off], (uint32_t)a->filtered_mm);
        off += telem_put_u16(&buf[off], (uint16_t)a->fpp_cdbm);
    }
    const uint16_t payload_len = (uint16_t)(off - TELEM_HDR_LEN);
    const uint16_t total = telem_frame_pack(buf, TELEM_TYPE_RANGE,
                                            snap->seq, snap->time_ms, payload_len);
    UART_TX_Write(buf, total);
#endif
}

/* ========================================================================== */
/*                     STATS (0x02)                                            */
/* ========================================================================== */

/* poll[4] ok[4] rx_to[4] rx_err[4] overrun[4] uart_ovf[4] cyc_hz[2] ops_hz[2] */
void Telem_SendStats(uint16_t cyc_hz, uint16_t ops_hz)
{
#if TELEM_ASCII
    char line[160];
    int n = snprintf(line, sizeof(line),
        "S,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u\r\n",
        (unsigned long)poll_sent_count,
        (unsigned long)response_ok_count,
        (unsigned long)rx_timeout_count,
        (unsigned long)rx_error_count,
        (unsigned long)cycle_overrun_count,
        (unsigned long)uart_tx_overflow_count,
        (unsigned)cyc_hz, (unsigned)ops_hz);
    if (n > 0 && n < (int)sizeof(line))
        UART_TX_Write((const uint8_t *)line, (uint16_t)n);
#else
    uint8_t buf[TELEM_MAX_FRAME];
    uint16_t off = TELEM_HDR_LEN;

    off += telem_put_u32(&buf[off], poll_sent_count);
    off += telem_put_u32(&buf[off], response_ok_count);
    off += telem_put_u32(&buf[off], rx_timeout_count);
    off += telem_put_u32(&buf[off], rx_error_count);
    off += telem_put_u32(&buf[off], cycle_overrun_count);
    off += telem_put_u32(&buf[off], uart_tx_overflow_count);
    off += telem_put_u16(&buf[off], cyc_hz);
    off += telem_put_u16(&buf[off], ops_hz);

    send_frame(buf, TELEM_TYPE_STATS, tag_sample_seq, (uint16_t)(off - TELEM_HDR_LEN));
#endif
}

/* ========================================================================== */
/*                     RANGE_MEAS (0x10)                                       */
/* ========================================================================== */

/*
 * schema[1]=1 boot_id[2] meas_seq[4] meas_time_us[8] anchor_id[2] txn[1]
 * mode[1] flags[2] status[1] raw_mm[4] corrected_mm[4] filtered_mm[4]
 * fp_cdbm[2] rx_cdbm[2] anchor_fp_cdbm[2] anchor_rx_cdbm[2] std_noise[2]
 * fp_index[2] ci_ppm_x100[2] slot_us[2]                        = 50 bytes
 * raw_mm is always BEFORE any software offset; corrected_mm is meaningful
 * only with flags bit1 (CAL_OK). INT16_MIN marks an unknown power.
 */
void Telem_SendRangeMeas(const TagMeasurement_t *m)
{
#if !TELEM_ASCII
    uint8_t buf[TELEM_MAX_FRAME];
    uint16_t off = TELEM_HDR_LEN;

    buf[off++] = TELEM_SCHEMA_V2;
    off += telem_put_u16(&buf[off], uwb_health.boot_id);
    off += telem_put_u32(&buf[off], m->meas_seq);
    off += telem_put_u64(&buf[off], m->meas_time_us);
    off += telem_put_u16(&buf[off], m->anchor_id);
    buf[off++] = m->txn;
    buf[off++] = m->mode;
    off += telem_put_u16(&buf[off], m->flags);
    buf[off++] = m->status;
    off += telem_put_u32(&buf[off], (uint32_t)m->raw_mm);
    off += telem_put_u32(&buf[off], (uint32_t)m->corrected_mm);
    off += telem_put_u32(&buf[off], (uint32_t)m->filtered_mm);
    off += telem_put_u16(&buf[off], (uint16_t)m->fp_cdbm);
    off += telem_put_u16(&buf[off], (uint16_t)m->rx_cdbm);
    off += telem_put_u16(&buf[off], (uint16_t)m->anchor_fp_cdbm);
    off += telem_put_u16(&buf[off], (uint16_t)m->anchor_rx_cdbm);
    off += telem_put_u16(&buf[off], m->std_noise);
    off += telem_put_u16(&buf[off], m->fp_index);
    off += telem_put_u16(&buf[off], (uint16_t)m->ci_ppm_x100);
    off += telem_put_u16(&buf[off], m->slot_us);

    send_frame(buf, TELEM_TYPE_RANGE_MEAS, m->meas_seq, (uint16_t)(off - TELEM_HDR_LEN));
#else
    (void)m;
#endif
}

/* ========================================================================== */
/*                     DIAG_ANCHOR (0x11)                                      */
/* ========================================================================== */

/*
 * schema[1]=1 anchor_id[2] active[1] flags[1] (bit0 backed off, bit1 DS
 * calibrated) success[4] resp_timeouts[4] poll_tx_timeouts[4] rx_errors[4]
 * poll_skipped[4] cal_missing[4] ds_ok[4] report_timeouts[4] ds_fallbacks[4]
 * txn_mismatch[4] probes[4] resp_streak[2] ds_streak[2] slot_us[4]
 * slot_max_us[4] resp_wait_max_us[4] processing_max_us[4] poll_tx_max_us[4]
 *                                                               = 73 bytes
 */
void Telem_SendDiagAnchor(uint8_t i)
{
#if !TELEM_ASCII
    if (i >= TAG_NUM_ANCHORS)
        return;

    uint8_t buf[TELEM_MAX_FRAME];
    uint16_t off = TELEM_HDR_LEN;
    uint8_t calibrated = 0U;
    (void)Tag_GetDsCalibration((uint16_t)(i + 1U), NULL, &calibrated);
    const uint8_t backed_off =
        (anchor_response_timeout_streak[i] >= TAG_OFFLINE_AFTER_TIMEOUTS
         || anchor_ds_incomplete_streak[i] >= TAG_OFFLINE_AFTER_TIMEOUTS) ? 1U : 0U;

    buf[off++] = TELEM_SCHEMA_V2;
    off += telem_put_u16(&buf[off], (uint16_t)(i + 1U));
    buf[off++] = (uint8_t)((Tag_GetActiveAnchorMask() >> i) & 1U);
    buf[off++] = (uint8_t)(backed_off | (calibrated ? 0x02U : 0x00U));
    off += telem_put_u32(&buf[off], anchor_success_count[i]);
    off += telem_put_u32(&buf[off], anchor_response_timeout_count[i]);
    off += telem_put_u32(&buf[off], anchor_poll_tx_timeout_count[i]);
    off += telem_put_u32(&buf[off], anchor_rx_error_count[i]);
    off += telem_put_u32(&buf[off], anchor_poll_skipped_count[i]);
    off += telem_put_u32(&buf[off], anchor_calibration_missing_count[i]);
    off += telem_put_u32(&buf[off], anchor_ds_ok_count[i]);
    off += telem_put_u32(&buf[off], anchor_report_timeout_count[i]);
    off += telem_put_u32(&buf[off], anchor_ds_fallback_count[i]);
    off += telem_put_u32(&buf[off], anchor_txn_mismatch_count[i]);
    off += telem_put_u32(&buf[off], anchor_probe_count[i]);
    off += telem_put_u16(&buf[off], anchor_response_timeout_streak[i]);
    off += telem_put_u16(&buf[off], anchor_ds_incomplete_streak[i]);
    off += telem_put_u32(&buf[off], anchor_slot_duration_us[i]);
    off += telem_put_u32(&buf[off], anchor_slot_duration_max_us[i]);
    off += telem_put_u32(&buf[off], anchor_response_wait_max_us[i]);
    off += telem_put_u32(&buf[off], anchor_processing_max_us[i]);
    off += telem_put_u32(&buf[off], anchor_poll_tx_duration_max_us[i]);

    send_frame(buf, TELEM_TYPE_DIAG_ANCHOR, s_frame_seq++, (uint16_t)(off - TELEM_HDR_LEN));
#else
    (void)i;
#endif
}

/* ========================================================================== */
/*                     ANCHOR_INFO (0x12)                                      */
/* ========================================================================== */

/*
 * schema[1]=1 anchor_id[2] anchor_status[1] pos_valid[1] x_mm[4] y_mm[4]
 * z_mm[4] build_hash[4] build_dirty[1] tx_power_mode[1] tx_ant_dly[2]
 * rx_ant_dly[2] boot_count[2] build_config_hash[4]              = 33 bytes
 */
void Telem_SendAnchorInfo(const TagAnchorInfo_t *info)
{
#if !TELEM_ASCII
    uint8_t buf[TELEM_MAX_FRAME];
    uint16_t off = TELEM_HDR_LEN;

    buf[off++] = TELEM_SCHEMA_V2;
    off += telem_put_u16(&buf[off], info->anchor_id);
    buf[off++] = info->anchor_status;
    buf[off++] = info->pos_valid;
    off += telem_put_u32(&buf[off], (uint32_t)info->pos_mm[0]);
    off += telem_put_u32(&buf[off], (uint32_t)info->pos_mm[1]);
    off += telem_put_u32(&buf[off], (uint32_t)info->pos_mm[2]);
    off += telem_put_u32(&buf[off], info->build_hash);
    buf[off++] = info->build_dirty;
    buf[off++] = info->tx_power_mode;
    off += telem_put_u16(&buf[off], info->tx_ant_dly);
    off += telem_put_u16(&buf[off], info->rx_ant_dly);
    off += telem_put_u16(&buf[off], info->boot_count);
    off += telem_put_u32(&buf[off], info->build_config_hash);

    send_frame(buf, TELEM_TYPE_ANCHOR_INFO, s_frame_seq++, (uint16_t)(off - TELEM_HDR_LEN));
#else
    (void)info;
#endif
}

/* ========================================================================== */
/*                     CMD_ACK (0x13)                                          */
/* ========================================================================== */

/* cmd_id[1] result[1] data[...]; frame SEQ = command SEQ */
void Telem_SendCmdAck(uint32_t cmd_seq, uint8_t cmd_id, uint8_t result,
                      const uint8_t *data, uint16_t length)
{
    uint8_t buf[TELEM_MAX_FRAME];
    uint16_t off = TELEM_HDR_LEN;

    if (length > (uint16_t)(TELEM_MAX_PAYLOAD - 2U))
        length = (uint16_t)(TELEM_MAX_PAYLOAD - 2U);
    buf[off++] = cmd_id;
    buf[off++] = result;
    for (uint16_t k = 0U; k < length && data != NULL; k++)
        buf[off++] = data[k];

    send_frame(buf, TELEM_TYPE_CMD_ACK, cmd_seq, (uint16_t)(off - TELEM_HDR_LEN));
}

/* ========================================================================== */
/*                     DIAG_SYSTEM (0x15)                                      */
/* ========================================================================== */

/*
 * schema[1]=1 boot_id[2] boot_count[2] reset_cause[4] uptime_ms[4]
 * cycle_count[4] cycle_us[4] cycle_max_us[4] overruns[4] spi_errors[4]
 * recoveries[4] recovery_failures[4] config_checks[4] config_mismatches[4]
 * last_config_mismatch[4] fault_hold[1] last_fault_cause[1] paused[1]
 * active_mask[1] uart_tx_overflow[4] uart_rx_overflow[4] uart_high_water[2]
 * meas_queue_drops[4] cmd_ok[4] cmd_rejected[4] cmd_crc_errors[4]
 * locked[1] settings_status[1] rx_err: phy[4] fcs[4] sync[4] frame_to[4]
 * lde[4] overrun[4] preamble_to[4] sfd_to[4] filtered[4] incomplete[4]
 * soft_resets[4] ds_ok[4] ds_fallback[4] ds_report_timeout[4]
 * ds_final_tx_timeout[4] temp_centi[2] vbat_mv[2]              = 149 bytes
 */
void Telem_SendDiagSystem(void)
{
#if !TELEM_ASCII
    uint8_t buf[TELEM_MAX_FRAME];
    uint16_t off = TELEM_HDR_LEN;
    const int16_t temp = s_env_valid ? DW1000_TemperatureCentiDeg(s_temp_raw) : INT16_MIN;
    const uint16_t vbat = s_env_valid ? DW1000_VoltageMilliVolt(s_vbat_raw) : 0U;

    buf[off++] = TELEM_SCHEMA_V2;
    off += telem_put_u16(&buf[off], uwb_health.boot_id);
    off += telem_put_u16(&buf[off], uwb_health.boot_count);
    off += telem_put_u32(&buf[off], uwb_health.reset_cause);
    off += telem_put_u32(&buf[off], uwb_platform_time_ms());
    off += telem_put_u32(&buf[off], tag_cycle_count);
    off += telem_put_u32(&buf[off], tag_cycle_duration_us);
    off += telem_put_u32(&buf[off], tag_cycle_duration_max_us);
    off += telem_put_u32(&buf[off], cycle_overrun_count);
    off += telem_put_u32(&buf[off], dw1000_spi_error_count);
    off += telem_put_u32(&buf[off], uwb_health.radio_recoveries);
    off += telem_put_u32(&buf[off], uwb_health.radio_recovery_failures);
    off += telem_put_u32(&buf[off], uwb_health.config_checks);
    off += telem_put_u32(&buf[off], uwb_health.config_mismatches);
    off += telem_put_u32(&buf[off], uwb_health.last_config_mismatch);
    buf[off++] = uwb_health.fault_hold;
    buf[off++] = uwb_health.last_fault_cause;
    buf[off++] = Tag_IsPaused();
    buf[off++] = Tag_GetActiveAnchorMask();
    off += telem_put_u32(&buf[off], uart_tx_overflow_count);
    off += telem_put_u32(&buf[off], uart_rx_overflow_count);
    off += telem_put_u16(&buf[off], uart_tx_high_water);
    off += telem_put_u32(&buf[off], tag_meas_queue_drops);
    off += telem_put_u32(&buf[off], uwb_cmd_stats.executed);
    off += telem_put_u32(&buf[off], uwb_cmd_stats.rejected);
    off += telem_put_u32(&buf[off], uwb_cmd_stats.crc_errors);
    buf[off++] = uwb_cmd_stats.locked;
    buf[off++] = uwb_settings_status;
    off += telem_put_u32(&buf[off], tag_rx_error_stats.phy_header);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.fcs);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.sync_loss);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.frame_timeout);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.lde);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.overrun);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.preamble_to);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.sfd_timeout);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.filtered);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.incomplete);
    off += telem_put_u32(&buf[off], tag_rx_error_stats.soft_resets);
    off += telem_put_u32(&buf[off], ds_ok_count);
    off += telem_put_u32(&buf[off], ds_fallback_count);
    off += telem_put_u32(&buf[off], ds_report_timeout_count);
    off += telem_put_u32(&buf[off], ds_final_tx_timeout_count);
    off += telem_put_u16(&buf[off], (uint16_t)temp);
    off += telem_put_u16(&buf[off], vbat);

    send_frame(buf, TELEM_TYPE_DIAG_SYSTEM, s_frame_seq++, (uint16_t)(off - TELEM_HDR_LEN));
#endif
}

/* ========================================================================== */
/*                     DEVICE_INFO (0x16)                                      */
/* ========================================================================== */

/*
 * schema[1]=1 role[1] node_addr[2] git_hash[4] git_dirty[1] frame_version[1]
 * telem_features[1] wait4resp[1] device_id[4] boot_id[2] boot_count[2]
 * reset_cause[4] tx_power_mode[1] reference_tuning[1] tx_ant_dly[2]
 * rx_ant_dly[2] tx_power_reg[4] spi_mhz[1] otp_valid[1] otp_part_id[4]
 * otp_lot_id[4] otp_ldotune[4] otp_xtal_trim[1] otp_rev[1] otp_vbat_cal[1]
 * otp_vtemp_cal[1] applied_xtal_trim[1] ldo_kicked[1] settings_status[1]
 * uart_baud[4] active_mask[1] ds_calibrated_mask[1]
 * build_config_hash[4] calibration_profile_id[4]               = 69 bytes
 */
void Telem_SendDeviceInfo(void)
{
#if !TELEM_ASCII
    uint8_t buf[TELEM_MAX_FRAME];
    uint16_t off = TELEM_HDR_LEN;

    buf[off++] = TELEM_SCHEMA_V2;
    buf[off++] = TELEM_ROLE_TAG;
    off += telem_put_u16(&buf[off], TAG_ADDR);
    off += telem_put_u32(&buf[off], (uint32_t)UWB_BUILD_GIT_HASH);
    buf[off++] = (uint8_t)UWB_BUILD_GIT_DIRTY;
    buf[off++] = (uint8_t)UWB_TAG_FRAME_VERSION;
    buf[off++] = Telem_GetFeatures();
    buf[off++] = (uint8_t)UWB_USE_WAIT4RESP;
    off += telem_put_u32(&buf[off], uwb_health.device_id);
    off += telem_put_u16(&buf[off], uwb_health.boot_id);
    off += telem_put_u16(&buf[off], uwb_health.boot_count);
    off += telem_put_u32(&buf[off], uwb_health.reset_cause);
    buf[off++] = dw1000_radio_config.tx_power_mode;
    buf[off++] = dw1000_radio_config.reference_tuning;
    off += telem_put_u16(&buf[off], dw1000_radio_config.tx_ant_dly);
    off += telem_put_u16(&buf[off], dw1000_radio_config.rx_ant_dly);
    off += telem_put_u32(&buf[off], DW1000_TxPowerRegisterValue());
    buf[off++] = dw1000_spi_mhz;
    buf[off++] = dw1000_otp.valid;
    off += telem_put_u32(&buf[off], dw1000_otp.part_id);
    off += telem_put_u32(&buf[off], dw1000_otp.lot_id);
    off += telem_put_u32(&buf[off], dw1000_otp.ldotune);
    buf[off++] = dw1000_otp.xtal_trim;
    buf[off++] = dw1000_otp.otp_rev;
    buf[off++] = dw1000_otp.vbat_cal;
    buf[off++] = dw1000_otp.vtemp_cal;
    buf[off++] = dw1000_otp.applied_xtal_trim;
    buf[off++] = dw1000_otp.ldo_kicked;
    buf[off++] = uwb_settings_status;
    off += telem_put_u32(&buf[off], Telem_UartBaud());
    buf[off++] = Tag_GetActiveAnchorMask();
    buf[off++] = Tag_DsCalibratedMask();
    off += telem_put_u32(&buf[off], (uint32_t)UWB_BUILD_CONFIG_HASH);
    off += telem_put_u32(&buf[off], DW1000_CalibrationProfileId());

    send_frame(buf, TELEM_TYPE_DEVICE_INFO, s_frame_seq++, (uint16_t)(off - TELEM_HDR_LEN));
#endif
}
