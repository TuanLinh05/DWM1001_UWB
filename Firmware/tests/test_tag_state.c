/* Run the real TAG state machine against the DW1000 register simulator. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dw1000_sim.h"
#include "../common/src/ranging/tag_ranging.c"

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

/* Exchange geometry, DW1000 ticks (15.65 ps). */
#define TOF_TICKS  1066ULL          /* ~5.0 m */
#define DA_TICKS   78643200ULL      /* 1200 UUS reply delay at the anchor */
#define DB_TICKS   25559105ULL      /* ~400 us TAG turnaround */

static int32_t expected_mm(uint64_t tof_ticks)
{
    return (int32_t)((double)tof_ticks * UWB_DWT_TIME_UNIT_S * UWB_SPEED_OF_LIGHT * 1000.0);
}

static int parse_tx(UwbFrameHeader_t *hdr)
{
    return uwb_frame_parse_header(sim_tx_frame, (uint16_t)(sim_tx_len + UWB_FRAME_FCS_LEN),
                                  DW_PAN_ID, hdr);
}

static uint8_t tx_txn(void)
{
    return sim_tx_frame[11];   /* v2 POLL/FINAL: [10] ver [11] txn */
}

static void deliver_resp(uint16_t anchor, uint8_t txn, uint32_t da, uint64_t rx_ts,
                         uint64_t extra, const uint8_t *tlv, uint8_t tlv_len)
{
    uint8_t f[UWB_FRAME_MAX_RX_LEN];
    const UwbResp_t r = {
        .version = UWB_FRAME_V2, .txn = txn, .reply_ticks = da,
        .anchor_status = tlv_len ? UWB_ANCHOR_ST_POS_VALID : 0U,
        .tlv_len = tlv_len, .tlv = tlv,
    };
    const uint16_t len = uwb_frame_build_resp(f, 0U, DW_PAN_ID, TAG_ADDR, anchor, &r);
    sim_deliver_rx(f, len, rx_ts, extra);
}

static void deliver_report(uint16_t anchor, uint8_t txn, uint32_t rb, uint64_t rx_ts)
{
    uint8_t f[UWB_FRAME_MAX_RX_LEN];
    const UwbReport_t r = {
        .version = UWB_FRAME_V2, .txn = txn, .round_ticks = rb,
        .final_fp_cdbm = -8000, .final_rx_cdbm = -7500,
    };
    const uint16_t len = uwb_frame_build_report(f, 0U, DW_PAN_ID, TAG_ADDR, anchor, &r);
    sim_deliver_rx(f, len, rx_ts, 0U);
}

static void start_cycle(void)
{
    sim_us += (uint64_t)TAG_CYCLE_MS * 1000U + 1000U;
    Tag_Task();
}

static int fresh_tag(uint8_t mask)
{
    sim_reset();
    if (Tag_Init() != 0)
        return -1;
    memset(s_track, 0, sizeof(s_track));
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        anchor_response_timeout_streak[i] = 0U;
        anchor_ds_incomplete_streak[i] = 0U;
        s_anchor_next_probe_cycle[i] = 0U;
        anchor_probe_count[i] = 0U;
        anchor_txn_mismatch_count[i] = 0U;
        anchor_report_timeout_count[i] = 0U;
        anchor_ds_ok_count[i] = 0U;
        anchor_poll_tx_timeout_count[i] = 0U;
        (void)Tag_SetDsCalibration((uint16_t)(i + 1U), 0, 0U);
    }
    memset(&tag_rx_error_stats, 0, sizeof(tag_rx_error_stats));
    Tag_SetActiveAnchorMask(mask);
    Tag_RequestPause(0U);
    Tag_EnableMeasurementQueue(0U);
    tag_cycle_count = 1U;   /* not a multiple of the info period */
    return 0;
}

/* One complete DS exchange with anchor `id`, from IDLE back to IDLE. */
static int ds_exchange(uint16_t id, uint64_t tof)
{
    UwbFrameHeader_t hdr;

    start_cycle();
    CHECK(s_state == TAG_STATE_TX_POLL);
    CHECK(parse_tx(&hdr) && hdr.func == FRAME_POLL_FUNC && hdr.dst == id);
    CHECK(sim_tx_frame[10] == UWB_FRAME_V2);
    CHECK(sim_last_ctrl() == (DW_TXSTRT_BIT | DW_WAIT4RESP_BIT));
    const uint8_t txn = tx_txn();

    const uint64_t t1 = 1000000000ULL;
    sim_complete_tx(t1);
    const uint32_t rx_enables = sim_rx_enable_count;
    Tag_Task();
    CHECK(s_state == TAG_STATE_WAIT_RESP);
    CHECK(sim_rx_enable_count == rx_enables);        /* WAIT4RESP: no RXENAB */

    const uint64_t t4 = t1 + 2U * tof + DA_TICKS;
    deliver_resp(id, txn, (uint32_t)DA_TICKS, t4, 0U, NULL, 0U);
    Tag_Task();
    CHECK(s_state == TAG_STATE_TX_FINAL);
    CHECK(parse_tx(&hdr) && hdr.func == FRAME_FINAL_FUNC && hdr.dst == id);
    CHECK(tx_txn() == txn);                           /* F4: txn echoed */

    sim_complete_tx(t4 + DB_TICKS);
    Tag_Task();
    CHECK(s_state == TAG_STATE_WAIT_REPORT);

    deliver_report(id, txn, (uint32_t)(2U * tof + DB_TICKS), t4 + DB_TICKS + 5000U);
    Tag_Task();
    CHECK(s_state == TAG_STATE_IDLE || s_state == TAG_STATE_INTER_ANCHOR_GUARD);
    return 0;
}

static int test_ds_exchange_and_calibration(void)
{
    CHECK(fresh_tag(0x01U) == 0);
    CHECK(ds_exchange(1U, TOF_TICKS) == 0);

    /* Uncalibrated: diagnostic range, production invalid. */
    CHECK(s_track[0].valid == 0U);
    CHECK(s_track[0].status == TAG_ST_CAL_MISSING_DS);
    CHECK(abs(s_track[0].raw_mm - expected_mm(TOF_TICKS)) <= 1);
    CHECK(anchor_ds_ok_count[0] == 1U);

    /* Runtime calibration: corrected = measured - bias. */
    CHECK(Tag_SetDsCalibration(1U, 100000, 1U) == 0);      /* 100 mm */
    CHECK(Tag_DsCalibratedMask() == 0x01U);
    Tag_EnableMeasurementQueue(1U);
    CHECK(ds_exchange(1U, TOF_TICKS) == 0);
    CHECK(s_track[0].valid == 1U);
    CHECK(abs(s_track[0].raw_mm - (expected_mm(TOF_TICKS) - 100)) <= 1);

    TagMeasurement_t m;
    CHECK(Tag_PopMeasurement(&m) == 1U);
    CHECK(m.anchor_id == 1U && m.mode == TAG_MEAS_MODE_DS);
    CHECK(abs(m.raw_mm - expected_mm(TOF_TICKS)) <= 1);           /* before offset */
    CHECK(abs(m.corrected_mm - (expected_mm(TOF_TICKS) - 100)) <= 1);
    CHECK((m.flags & (TAG_MEAS_FLAG_RADIO_OK | TAG_MEAS_FLAG_CAL_OK))
          == (TAG_MEAS_FLAG_RADIO_OK | TAG_MEAS_FLAG_CAL_OK));
    CHECK((m.flags & TAG_MEAS_FLAG_ANCHOR_DIAG) != 0U);
    CHECK(m.anchor_fp_cdbm == -8000 && m.anchor_rx_cdbm == -7500);
    CHECK(Tag_PopMeasurement(&m) == 0U);

    CHECK(Tag_SetDsCalibration(9U, 0, 1U) != 0);            /* unknown anchor */
    CHECK(Tag_SetDsCalibration(1U, 20000000, 1U) != 0);     /* > 10 m bias */
    return 0;
}

static int test_clock_drift_tolerance(void)
{
    /* Anchor crystal +20 ppm: DS-TWR must stay within 1 mm. */
    CHECK(fresh_tag(0x01U) == 0);
    UwbFrameHeader_t hdr;
    start_cycle();
    const uint8_t txn = tx_txn();
    const uint64_t t1 = 5000000000ULL;
    sim_complete_tx(t1);
    Tag_Task();
    const double k = 1.0 + 20e-6;
    const uint64_t t4 = t1 + 2U * TOF_TICKS + DA_TICKS;          /* TAG clock */
    deliver_resp(1U, txn, (uint32_t)((double)DA_TICKS * k), t4, 0U, NULL, 0U);
    Tag_Task();
    CHECK(parse_tx(&hdr) && hdr.func == FRAME_FINAL_FUNC);
    sim_complete_tx(t4 + DB_TICKS);
    Tag_Task();
    deliver_report(1U, txn, (uint32_t)((double)(2U * TOF_TICKS + DB_TICKS) * k), t4 + DB_TICKS);
    Tag_Task();
    CHECK(abs(s_track[0].raw_mm - expected_mm(TOF_TICKS)) <= 1);
    return 0;
}

static int test_stale_txn_and_rx_error(void)
{
    CHECK(fresh_tag(0x01U) == 0);
    start_cycle();
    const uint8_t txn = tx_txn();
    sim_complete_tx(1000U);
    Tag_Task();

    /* F4: a RESP of an older transaction is ignored, deadline kept. */
    const uint32_t rx_enables = sim_rx_enable_count;
    deliver_resp(1U, (uint8_t)(txn - 1U), (uint32_t)DA_TICKS, 2000U, 0U, NULL, 0U);
    Tag_Task();
    CHECK(s_state == TAG_STATE_WAIT_RESP);
    CHECK(anchor_txn_mismatch_count[0] == 1U);
    CHECK(sim_rx_enable_count == rx_enables + 1U);

    /* F3: RX_GOOD together with LDEERR is rejected and the RX reset. */
    deliver_resp(1U, txn, (uint32_t)DA_TICKS, 3000U, DW_LDEERR_BIT, NULL, 0U);
    Tag_Task();
    CHECK(s_state == TAG_STATE_WAIT_RESP);
    CHECK(tag_rx_error_stats.lde == 1U && sim_rx_soft_resets == 1U);

    /* Deadline: timeout, flagged with the RX error seen in this slot. */
    sim_us += TAG_RESP_TIMEOUT_US + 10U;
    Tag_Task();
    CHECK(s_state == TAG_STATE_IDLE);
    CHECK(s_track[0].valid == 0U);
    CHECK(s_track[0].status == (TAG_ST_TIMEOUT | TAG_ST_RXERR));
    return 0;
}

static int test_report_timeout_falls_back(void)
{
    CHECK(fresh_tag(0x01U) == 0);
    start_cycle();
    const uint8_t txn = tx_txn();
    sim_complete_tx(1000000U);
    Tag_Task();
    deliver_resp(1U, txn, (uint32_t)DA_TICKS, 1000000U + 2U * TOF_TICKS + DA_TICKS,
                 0U, NULL, 0U);
    Tag_Task();
    sim_complete_tx(1000000U + 2U * TOF_TICKS + DA_TICKS + DB_TICKS);
    Tag_Task();
    CHECK(s_state == TAG_STATE_WAIT_REPORT);
    sim_us += TAG_REPORT_TIMEOUT_US + 10U;
    Tag_Task();
    CHECK(s_state == TAG_STATE_IDLE);
    CHECK(anchor_report_timeout_count[0] == 1U);
    CHECK(anchor_ds_incomplete_streak[0] == 1U);
    /* SS residual profile is uncalibrated → diagnostic only. */
    CHECK(s_track[0].status == (TAG_ST_DS_FALLBACK | TAG_ST_CALIBRATION_MISSING));
    return 0;
}

/* Drive one full cycle in which every POLL gets no answer, except the
 * anchor listed in `answer_id` (0 = none) which completes a DS exchange. */
static int run_silent_cycle(uint16_t answer_id)
{
    start_cycle();
    for (int guard = 0; guard < 100 && s_state != TAG_STATE_IDLE; guard++)
    {
        UwbFrameHeader_t hdr;
        switch (s_state)
        {
            case TAG_STATE_TX_POLL:
                sim_complete_tx(sim_us * 64000ULL);
                Tag_Task();
                if (parse_tx(&hdr) && hdr.dst == answer_id && s_state == TAG_STATE_WAIT_RESP)
                {
                    const uint8_t txn = tx_txn();
                    const uint64_t t4 = sim_us * 64000ULL + 2U * TOF_TICKS + DA_TICKS;
                    deliver_resp(answer_id, txn, (uint32_t)DA_TICKS, t4, 0U, NULL, 0U);
                    Tag_Task();
                    sim_complete_tx(t4 + DB_TICKS);
                    Tag_Task();
                    deliver_report(answer_id, txn, (uint32_t)(2U * TOF_TICKS + DB_TICKS),
                                   t4 + DB_TICKS);
                    Tag_Task();
                }
                break;
            case TAG_STATE_WAIT_RESP:
                sim_us += TAG_RESP_TIMEOUT_US + 10U;
                Tag_Task();
                break;
            case TAG_STATE_INTER_ANCHOR_GUARD:
                sim_us += TAG_INTER_ANCHOR_GUARD_US + 10U;
                Tag_Task();
                break;
            default:
                Tag_Task();
                break;
        }
    }
    CHECK(s_state == TAG_STATE_IDLE);
    return 0;
}

static int test_offline_probe_fairness(void)
{
    CHECK(fresh_tag(0xFFU) == 0);
    for (int cycle = 0; cycle < 500; cycle++)
        CHECK(run_silent_cycle(0U) == 0);

    /* F1: every offline anchor gets probed, the last ones included. */
    uint32_t min_p = 0xFFFFFFFFU, max_p = 0U;
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        if (anchor_probe_count[i] < min_p) min_p = anchor_probe_count[i];
        if (anchor_probe_count[i] > max_p) max_p = anchor_probe_count[i];
    }
    CHECK(min_p >= 50U);
    CHECK(max_p - min_p <= 1U);

    /* A8 comes back: it must be found by a probe and polled every cycle. */
    const uint32_t ok_before = anchor_ds_ok_count[7];
    for (uint32_t cycle = 0U; cycle < 3U * TAG_NUM_ANCHORS; cycle++)
        CHECK(run_silent_cycle(8U) == 0);
    CHECK(anchor_ds_ok_count[7] > ok_before);
    CHECK(anchor_response_timeout_streak[7] == 0U);
    CHECK(anchor_ds_incomplete_streak[7] == 0U);
    const uint32_t ok_mid = anchor_ds_ok_count[7];
    for (int cycle = 0; cycle < 5; cycle++)
        CHECK(run_silent_cycle(8U) == 0);
    CHECK(anchor_ds_ok_count[7] == ok_mid + 5U);   /* healthy: every cycle */
    return 0;
}

static int test_active_mask_and_pause(void)
{
    UwbFrameHeader_t hdr;

    CHECK(fresh_tag(0x05U) == 0);                    /* A1 and A3 */
    start_cycle();
    CHECK(parse_tx(&hdr) && hdr.dst == 1U);
    sim_complete_tx(1000U);
    Tag_Task();
    sim_us += TAG_RESP_TIMEOUT_US + 10U;
    Tag_Task();
    CHECK(s_state == TAG_STATE_INTER_ANCHOR_GUARD);
    sim_us += TAG_INTER_ANCHOR_GUARD_US + 10U;
    Tag_Task();
    CHECK(parse_tx(&hdr) && hdr.dst == 3U);          /* A2 skipped: disabled */

    /* Pause takes effect at the end of the cycle, then no radio activity. */
    Tag_RequestPause(1U);
    CHECK(Tag_IsPaused() == 0U);
    sim_complete_tx(2000U);
    Tag_Task();
    sim_us += TAG_RESP_TIMEOUT_US + 10U;
    Tag_Task();
    CHECK(s_state == TAG_STATE_IDLE);
    Tag_Task();
    CHECK(Tag_IsPaused() == 1U);
    const uint32_t tx_before = sim_tx_count;
    for (int k = 0; k < 10; k++)
        start_cycle();
    CHECK(sim_tx_count == tx_before);
    Tag_RequestPause(0U);
    start_cycle();
    CHECK(sim_tx_count == tx_before + 1U);
    return 0;
}

static int test_poll_tx_timeout_and_stray(void)
{
    CHECK(fresh_tag(0x01U) == 0);
    s_track[0].valid = 1U;
    start_cycle();
    CHECK(s_state == TAG_STATE_TX_POLL);
    sim_us += TAG_TX_TIMEOUT_US + 10U;
    Tag_Task();
    CHECK(s_track[0].valid == 0U && s_track[0].status == TAG_ST_TIMEOUT);
    CHECK(anchor_poll_tx_timeout_count[0] == 1U);

    /* Correctly framed traffic from another anchor never extends the RESP
     * deadline. */
    start_cycle();
    sim_complete_tx(1000U);
    Tag_Task();
    const McuCycleStamp_t deadline_start = s_state_cycle;
    deliver_resp(2U, tx_txn(), (uint32_t)DA_TICKS, 5000U, 0U, NULL, 0U);
    Tag_Task();
    CHECK(s_state == TAG_STATE_WAIT_RESP && s_state_cycle == deadline_start);
    sim_us += TAG_RESP_TIMEOUT_US + 10U;
    Tag_Task();
    CHECK(s_state == TAG_STATE_IDLE && s_track[0].status == TAG_ST_TIMEOUT);
    return 0;
}

static int test_anchor_info_tlv(void)
{
    CHECK(fresh_tag(0x01U) == 0);
    tag_cycle_count = TAG_INFO_REQUEST_PERIOD_CYCLES - 1U;   /* next cycle asks */
    start_cycle();
    CHECK((sim_tx_frame[12] & UWB_POLL_FLAG_REQ_INFO) != 0U);
    const uint8_t txn = tx_txn();
    sim_complete_tx(1000U);
    Tag_Task();

    uint8_t tlv[32];
    uint8_t n = 0U;
    tlv[n++] = UWB_TLV_ANCHOR_POSITION;
    tlv[n++] = UWB_TLV_ANCHOR_POSITION_LEN;
    uwb_put_u32(&tlv[n], 1000U); uwb_put_u32(&tlv[n + 4], (uint32_t)-2000);
    uwb_put_u32(&tlv[n + 8], 2500U);
    n = (uint8_t)(n + UWB_TLV_ANCHOR_POSITION_LEN);
    deliver_resp(1U, txn, (uint32_t)DA_TICKS, 1000U + DA_TICKS, 0U, tlv, n);
    Tag_Task();
    CHECK(s_state == TAG_STATE_TX_FINAL);

    TagAnchorInfo_t info;
    CHECK(Tag_TakeAnchorInfo(&info) == 1U);
    CHECK(info.anchor_id == 1U && info.pos_valid == 1U);
    CHECK(info.pos_mm[0] == 1000 && info.pos_mm[1] == -2000 && info.pos_mm[2] == 2500);
    CHECK(Tag_TakeAnchorInfo(&info) == 0U);
    return 0;
}

static int test_legacy_outlier_does_not_publish_or_update(void)
{
    CHECK(fresh_tag(0x01U) == 0);
    CHECK(Tag_SetDsCalibration(1U, 0, 1U) == 0);
    Tag_EnableMeasurementQueue(1U);

    /* Fill median-3 at a stable 5 m, then present two coherent 15 m
     * exchanges. The first is hidden by the median; the second trips the
     * deployed motion gate. */
    TagMeasurement_t m;
    for (int i = 0; i < 3; i++)
    {
        CHECK(ds_exchange(1U, TOF_TICKS) == 0);
        CHECK(Tag_PopMeasurement(&m) == 1U);
    }
    CHECK(ds_exchange(1U, 3U * TOF_TICKS) == 0);
    CHECK(Tag_PopMeasurement(&m) == 1U);
    CHECK(s_track[0].valid == 1U);

    const double x_before = s_kf[0].x;
    const double p_before = s_kf[0].P;
    const int32_t filtered_before = s_track[0].filtered_mm;
    const uint32_t success_before = s_track[0].last_success_tick;
    CHECK(ds_exchange(1U, 3U * TOF_TICKS) == 0);
    CHECK(Tag_PopMeasurement(&m) == 1U);

    CHECK(s_kf[0].x == x_before && s_kf[0].P == p_before);
    CHECK(s_track[0].valid == 0U);
    CHECK(s_track[0].status == TAG_ST_RANGE_REJECT);
    CHECK(s_track[0].filtered_mm == filtered_before);
    CHECK(s_track[0].last_success_tick == success_before);
    CHECK((m.flags & (TAG_MEAS_FLAG_RADIO_OK | TAG_MEAS_FLAG_CAL_OK))
          == (TAG_MEAS_FLAG_RADIO_OK | TAG_MEAS_FLAG_CAL_OK));
    CHECK((m.flags & TAG_MEAS_FLAG_FILTER_OK) == 0U);
    CHECK(m.status == TAG_ST_RANGE_REJECT);

    /* The rejected median candidate must not poison the next good sample. */
    CHECK(ds_exchange(1U, TOF_TICKS) == 0);
    CHECK(s_track[0].valid == 1U);
    return 0;
}

static int test_recovery_discards_pending_measurements(void)
{
    CHECK(fresh_tag(0x01U) == 0);
    CHECK(Tag_SetDsCalibration(1U, 0, 1U) == 0);
    Tag_EnableMeasurementQueue(1U);
    CHECK(ds_exchange(1U, TOF_TICKS) == 0);
    CHECK(s_meas_count == 1U && s_kf_initialized[0] == 1U);
    CHECK(s_track[0].valid == 1U && tag_cycle_ready != 0U);

    CHECK(Tag_RecoverRadio() == 0);
    TagMeasurement_t m;
    CHECK(Tag_PopMeasurement(&m) == 0U);
    CHECK(s_kf_initialized[0] == 0U && s_mf[0].count == 0U);
    CHECK(s_track[0].valid == 0U && s_track[0].status == TAG_ST_RXERR);
    CHECK(s_track[0].raw_mm == 0 && tag_cycle_ready == 0U);
    return 0;
}

static int test_calibration_change_invalidates_published_range(void)
{
    CHECK(fresh_tag(0x01U) == 0);
    CHECK(Tag_SetDsCalibration(1U, 0, 1U) == 0);
    Tag_EnableMeasurementQueue(1U);
    CHECK(ds_exchange(1U, TOF_TICKS) == 0);
    CHECK(s_track[0].valid == 1U);
    CHECK(s_meas_count == 1U && tag_cycle_ready != 0U);

    /* Re-applying the same setting must leave the pending data alone. */
    CHECK(Tag_SetDsCalibration(1U, 0, 1U) == 0);
    CHECK(s_meas_count == 1U && tag_cycle_ready != 0U);

    CHECK(Tag_SetDsCalibration(1U, 100000, 1U) == 0);
    CHECK(s_track[0].valid == 0U && s_track[0].raw_mm == 0);
    TagMeasurement_t m;
    CHECK(Tag_PopMeasurement(&m) == 0U && tag_cycle_ready == 0U);
    CHECK(ds_exchange(1U, TOF_TICKS) == 0);
    CHECK(s_track[0].valid == 1U);
    CHECK(s_meas_count == 1U && tag_cycle_ready != 0U);

    Tag_InvalidateDsCalibration();
    CHECK(s_track[0].valid == 0U);
    CHECK(s_track[0].status == TAG_ST_CALIBRATION_MISSING);
    CHECK(s_track[0].raw_mm == 0 && s_kf_initialized[0] == 0U);
    CHECK(Tag_PopMeasurement(&m) == 0U && tag_cycle_ready == 0U);

    CHECK(Tag_SetDsCalibration(1U, 0, 1U) == 0);
    CHECK(ds_exchange(1U, TOF_TICKS) == 0);
    CHECK(s_track[0].valid == 1U);
    CHECK(s_meas_count == 1U && tag_cycle_ready != 0U);
    CHECK(Tag_SetDsCalibration(1U, 0, 0U) == 0);
    CHECK(s_track[0].valid == 0U);
    CHECK(s_track[0].status == TAG_ST_CALIBRATION_MISSING);
    CHECK(Tag_PopMeasurement(&m) == 0U && tag_cycle_ready == 0U);

    /* RF profile invalidation also flushes pending SS fallback records when
     * no DS calibration bit is currently set. */
    m.anchor_id = 1U;
    m.mode = TAG_MEAS_MODE_SS_FALLBACK;
    meas_push(&m);
    tag_cycle_ready = 1U;
    Tag_InvalidateDsCalibration();
    CHECK(Tag_PopMeasurement(&m) == 0U && tag_cycle_ready == 0U);
    return 0;
}

static int test_nonpositive_corrected_range_is_rejected(void)
{
    CHECK(fresh_tag(0x01U) == 0);
    CHECK(Tag_SetDsCalibration(1U, 6000000, 1U) == 0); /* 6 m offset on a 5 m link */
    CHECK(ds_exchange(1U, TOF_TICKS) == 0);
    CHECK(s_track[0].valid == 0U && s_track[0].status == TAG_ST_COMPUTE);
    CHECK(s_meas.raw_mm > 4000);  /* pre-offset ToF remains available for diagnosis */
    CHECK(apply_offset_and_clamp(6.0, UWB_MODE_DS) == RANGE_COMPUTE_ERROR);
    CHECK(apply_offset_and_clamp(6.0005, UWB_MODE_DS) == RANGE_COMPUTE_ERROR);

    CHECK(fresh_tag(0x01U) == 0);
    s_ss_calibrated_mask = UWB_SS_CAL_A1_BIT;
    residual_offset_m[0] = 6.0;

    /* Lose REPORT to exercise the SS fallback calibration path. */
    start_cycle();
    const uint8_t txn = tx_txn();
    const uint64_t t1 = 1000000ULL;
    sim_complete_tx(t1);
    Tag_Task();
    deliver_resp(1U, txn, (uint32_t)DA_TICKS,
                 t1 + 2U * TOF_TICKS + DA_TICKS, 0U, NULL, 0U);
    Tag_Task();
    sim_complete_tx(t1 + 2U * TOF_TICKS + DA_TICKS + DB_TICKS);
    Tag_Task();
    CHECK(s_state == TAG_STATE_WAIT_REPORT);
    sim_us += TAG_REPORT_TIMEOUT_US + 10U;
    Tag_Task();
    CHECK(s_state == TAG_STATE_IDLE);
    CHECK(s_track[0].valid == 0U && s_track[0].status == TAG_ST_COMPUTE);
    CHECK(s_meas.raw_mm > 4000);
    CHECK(apply_offset_and_clamp(6.0, UWB_MODE_SS_FALLBACK) == RANGE_COMPUTE_ERROR);
    CHECK(apply_offset_and_clamp(6.0005, UWB_MODE_SS_FALLBACK) == RANGE_COMPUTE_ERROR);

    residual_offset_m[0] = 0.0;
    s_ss_calibrated_mask = UWB_SS_ACTIVE_CALIBRATED_MASK;
    return 0;
}

int main(void)
{
    /* The production topology and scheduler must stay aligned. */
    _Static_assert(TAG_NUM_ANCHORS == 8U, "TAG must poll all eight anchors");
    _Static_assert(TAG_CYCLE_MS == 20U, "Eight-anchor cycle must stay at 50 Hz");
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        if (anchor_id_at(i) != (uint16_t)(i + 1U))
            return 1;
    }

    if (test_ds_exchange_and_calibration()
        || test_clock_drift_tolerance()
        || test_stale_txn_and_rx_error()
        || test_report_timeout_falls_back()
        || test_offline_probe_fairness()
        || test_active_mask_and_pause()
        || test_poll_tx_timeout_and_stray()
        || test_anchor_info_tlv()
        || test_legacy_outlier_does_not_publish_or_update()
        || test_recovery_discards_pending_measurements()
        || test_calibration_change_invalidates_published_range()
        || test_nonpositive_corrected_range_is_rejected())
    {
        return 1;
    }
    puts("TAG state machine tests passed (DS v2, txn, F1, F3, fallback, mask, pause, TLV, filter recovery)");
    return 0;
}
