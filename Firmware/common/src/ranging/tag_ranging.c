/**
 ******************************************************************************
 * @file    tag_ranging.c
 * @brief   TAG ranging — non-blocking DS-TWR state machine, up to 8 anchors
 *
 * Exchange per anchor (UWB_USE_DS_TWR = 1, default):
 *   TAG ──POLL──> Anchor ──RESP(Da)──> TAG ──FINAL──> Anchor ──REPORT(Rb)──> TAG
 *   tof = (Ra·Rb − Da·Db) / (Ra + Rb + Da + Db)
 * Protocol v2 frames echo a transaction ID through the whole exchange (F4).
 *
 * State machine (20 ms cycle):
 *   IDLE --cycle due--> TX_POLL --TXFRS--> WAIT_RESP --RESP--> TX_FINAL
 *   --TXFRS--> WAIT_REPORT --REPORT--> INTER_ANCHOR_GUARD --> TX_POLL (next)
 *   Any timeout ends the slot; a lost REPORT falls back to the SS terms.
 *
 * Key design:
 *   - No blocking waits in the state machine; all SPI runs in Tag_Task().
 *   - The GPIO IRQ callback only sets dw1000_irq_flag.
 *   - A frame is accepted only when data, FCS and LDE all succeeded with no
 *     error flag in the same status (F3). Errors trigger an RX soft reset
 *     and the receiver keeps listening until the original deadline.
 *   - POLL/FINAL use WAIT4RESP so the receiver is on before the reply (N6).
 *   - Offline anchors are probed round-robin, one probe per cycle (F1);
 *     anchors that answer RESP but never complete DS are backed off too (F8).
 ******************************************************************************
 */

#include "tag_ranging.h"
#include "legacy_adaptive_tracking.h"
#include "motion_adaptive_range.h"
#include "range_filter.h"
#include "uwb_frame.h"
#include "uwb_platform.h"
#include <limits.h>
#include <math.h>
#include <string.h>

/* ========================================================================== */
/*                     CALIBRATION                                             */
/* ========================================================================== */

/* SS-TWR legacy offsets of the former STM32 hardware. They are only used
 * with UWB_USE_LEGACY_OFFSET=1 and guarded by UWB_SS_LEGACY_CALIBRATED_MASK. */
volatile double calibration_offset_m[TAG_NUM_ANCHORS] = {
    156.7284371582,
    156.4618,
    155.9958,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
};
volatile double auto_calibrate_actual_m = 0.0;
volatile uint16_t auto_calibrate_anchor_id = 0U;

/* Residual offset (mét) khi dùng HW antenna delay — nhỏ (cỡ cm). */
volatile double residual_offset_m[TAG_NUM_ANCHORS] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

/* ========================================================================== */
/*                     PRIVATE DEFINES                                         */
/* ========================================================================== */

#define MAX_RX_FRAME_LEN    UWB_FRAME_MAX_RX_LEN

#define RANGE_COMPUTE_ERROR       (-1)
#define RANGE_CALIBRATION_MISSING (-2)

/* Largest DS bias a command may install (±10 m): anything bigger is a typo. */
#define TAG_DS_BIAS_LIMIT_UM      10000000L

/* One anchor is asked for its info TLVs every this many cycles (1 s). */
#define TAG_INFO_REQUEST_PERIOD_CYCLES 50U

typedef struct {
    uint16_t id;
    uint32_t ss_calibration_bit;
    uint32_t ds_calibration_bit;
} TagAnchorConfig_t;

/** ID is the data key. Array index is only the sequential radio slot. */
static const TagAnchorConfig_t s_anchor_cfg[TAG_NUM_ANCHORS] = {
    { 0x0001U, UWB_SS_CAL_A1_BIT, UWB_DS_CAL_A1_BIT },
    { 0x0002U, UWB_SS_CAL_A2_BIT, UWB_DS_CAL_A2_BIT },
    { 0x0003U, UWB_SS_CAL_A3_BIT, UWB_DS_CAL_A3_BIT },
    { 0x0004U, UWB_SS_CAL_A4_BIT, UWB_DS_CAL_A4_BIT },
    { 0x0005U, UWB_SS_CAL_A5_BIT, UWB_DS_CAL_A5_BIT },
    { 0x0006U, UWB_SS_CAL_A6_BIT, UWB_DS_CAL_A6_BIT },
    { 0x0007U, UWB_SS_CAL_A7_BIT, UWB_DS_CAL_A7_BIT },
    { 0x0008U, UWB_SS_CAL_A8_BIT, UWB_DS_CAL_A8_BIT },
};

static uint32_t s_ss_calibrated_mask = UWB_SS_ACTIVE_CALIBRATED_MASK;

static inline uint16_t anchor_id_at(uint8_t anchor_index)
{
    return s_anchor_cfg[anchor_index].id;
}

static int anchor_index_of(uint16_t anchor_id)
{
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        if (s_anchor_cfg[i].id == anchor_id)
            return (int)i;
    }
    return -1;
}

/* DS calibration: runtime table seeded from the build configuration. */
typedef struct {
    int32_t bias_um;      /* subtracted from the DS range */
    uint8_t calibrated;   /* 0: publish diagnostics only (valid = 0) */
} TagDsCalibration_t;

#define TAG_DS_BIAS_UM(offset_m) ((int32_t)((offset_m) * 1000000.0))
#define TAG_DS_CAL_ENTRY(n) \
    { TAG_DS_BIAS_UM(UWB_DS_OFFSET_A##n##_M), \
      (uint8_t)((UWB_DS_CALIBRATED_MASK & UWB_DS_CAL_A##n##_BIT) != 0U) }

static const TagDsCalibration_t k_ds_cal_default[TAG_NUM_ANCHORS] = {
    TAG_DS_CAL_ENTRY(1), TAG_DS_CAL_ENTRY(2), TAG_DS_CAL_ENTRY(3),
    TAG_DS_CAL_ENTRY(4), TAG_DS_CAL_ENTRY(5), TAG_DS_CAL_ENTRY(6),
    TAG_DS_CAL_ENTRY(7), TAG_DS_CAL_ENTRY(8),
};

static TagDsCalibration_t s_ds_cal[TAG_NUM_ANCHORS] = {
    TAG_DS_CAL_ENTRY(1), TAG_DS_CAL_ENTRY(2), TAG_DS_CAL_ENTRY(3),
    TAG_DS_CAL_ENTRY(4), TAG_DS_CAL_ENTRY(5), TAG_DS_CAL_ENTRY(6),
    TAG_DS_CAL_ENTRY(7), TAG_DS_CAL_ENTRY(8),
};

#if UWB_USE_DS_TWR
/**
 * @brief  Resolve DS calibration for a radio slot.
 * @retval 0 on success, RANGE_CALIBRATION_MISSING when the anchor has not
 *         been calibrated, RANGE_COMPUTE_ERROR for bad input.
 */
static int32_t ds_calibration_offset_for_index(uint8_t anchor_index,
                                               double *offset_m)
{
    if (offset_m == NULL || anchor_index >= TAG_NUM_ANCHORS)
        return RANGE_COMPUTE_ERROR;
    if (!s_ds_cal[anchor_index].calibrated)
        return RANGE_CALIBRATION_MISSING;

    *offset_m = (double)s_ds_cal[anchor_index].bias_um / 1000000.0;
    return 0;
}
#endif

/* ========================================================================== */
/*                     STATE MACHINE                                           */
/* ========================================================================== */

typedef enum {
    TAG_STATE_IDLE      = 0,  /* Waiting for next 20ms cycle */
    TAG_STATE_TX_POLL   = 1,  /* POLL sent, waiting for TXFRS IRQ */
    TAG_STATE_WAIT_RESP = 2,  /* RX active, waiting for RESP from anchor */
    TAG_STATE_INTER_ANCHOR_GUARD = 3, /* Let all anchors return to RX before next POLL */
#if UWB_USE_DS_TWR
    TAG_STATE_TX_FINAL    = 4, /* FINAL sent, waiting for TXFRS to read T5 */
    TAG_STATE_WAIT_REPORT = 5, /* RX active, waiting for REPORT from anchor */
#endif
} TagState_t;

/* ========================================================================== */
/*                     PRIVATE DATA                                            */
/* ========================================================================== */

static TagState_t          s_state          = TAG_STATE_IDLE;
static uint32_t            s_cycle_tick     = 0;  /* Tick of last cycle start */
static McuCycleStamp_t     s_state_cycle    = 0;  /* FIX-02: raw cycle stamp of state entry */
static McuCycleStamp_t     s_cycle_end_cycle = 0; /* End of previous full cycle; overload recovery */
static uint8_t             s_current_anchor = 0;  /* Sequential slot in s_anchor_cfg */
static uint8_t             s_seq_num        = 0;  /* MAC sequence number */
static uint8_t             s_txn            = 0;  /* v2 transaction ID of the open slot */
static uint8_t             s_slot_rx_error  = 0;  /* an RX error happened in this slot */
static uint32_t            s_anchor_next_probe_cycle[TAG_NUM_ANCHORS] = {0};
static uint8_t             s_probe_rr       = 0;  /* next offline anchor to probe (F1) */
static int8_t              s_cycle_probe_idx = -1;
static int8_t              s_info_request_idx = -1;
static uint8_t             s_info_rr        = 0;
static uint8_t             s_active_anchor_mask = (uint8_t)(UWB_TAG_ACTIVE_ANCHOR_MASK & 0xFFU);
static volatile uint8_t    s_pause_requested = 0;
static uint8_t             s_paused         = 0;

/* Poll TX timestamp for current POLL (used in SS-TWR calc) */
static uint8_t             s_poll_tx_ts[5]  = {0};

static DW1000_RangingResult s_result        = {0};

#if UWB_USE_DS_TWR
/* DS-TWR per-exchange state — valid only during TX_FINAL / WAIT_REPORT */
static uint8_t             s_resp_rx_ts[5]   = {0};  /* T4: RESP RX timestamp */
static uint32_t            s_da_from_resp    = 0;    /* Da from RESP payload */
static uint8_t             s_final_tx_ts[5]  = {0};  /* T5: FINAL TX timestamp (read after TXFRS) */
static DW1000_SignalDiag_t s_resp_diag       = {0};  /* RESP diag (read before FINAL TX) */
static float               s_resp_fpp        = 0.0f; /* FPP from RESP (saved before FINAL TX) */
static int32_t             s_resp_ci         = 0;    /* FIX-04: carrier integrator of RESP */
#endif

/* Measurement assembled during the current slot (telemetry v2). */
static TagMeasurement_t    s_meas;
static uint8_t             s_meas_queue_enabled = 0;
static TagMeasurement_t    s_meas_queue[TAG_MEAS_QUEUE_LEN];
static uint8_t             s_meas_head = 0;
static uint8_t             s_meas_tail = 0;
static uint8_t             s_meas_count = 0;
static uint32_t            s_meas_seq = 0;
volatile uint32_t          tag_meas_queue_drops = 0;

/* Anchor information TLVs received in RESP (commissioning data). */
static TagAnchorInfo_t     s_anchor_info[TAG_NUM_ANCHORS];
static uint8_t             s_anchor_info_pending = 0;

/* Published per-anchor distances (volatile for Live Expressions / debugger) */
volatile int32_t distance_a1_mm = 0;
volatile int32_t distance_a2_mm = 0;
volatile int32_t distance_a3_mm = 0;
volatile int32_t distance_a4_mm = 0;
volatile int32_t distance_a5_mm = 0;
volatile int32_t distance_a6_mm = 0;
volatile int32_t distance_a7_mm = 0;
volatile int32_t distance_a8_mm = 0;
volatile int32_t distance_raw_mm[TAG_NUM_ANCHORS] = {0};

volatile int32_t distance_a1_filtered_mm = 0;
volatile int32_t distance_a2_filtered_mm = 0;
volatile int32_t distance_a3_filtered_mm = 0;
volatile int32_t distance_a4_filtered_mm = 0;
volatile int32_t distance_a5_filtered_mm = 0;
volatile int32_t distance_a6_filtered_mm = 0;
volatile int32_t distance_a7_filtered_mm = 0;
volatile int32_t distance_a8_filtered_mm = 0;
volatile int32_t distance_filtered_mm[TAG_NUM_ANCHORS] = {0};

volatile uint32_t tag_cycle_count = 0;

/* ========================================================================== */
/*                     TELEMETRY STATE & COUNTERS                              */
/* ========================================================================== */

volatile uint8_t  tag_cycle_ready     = 0;
volatile uint32_t tag_sample_seq      = 0;

volatile uint32_t poll_sent_count     = 0;
volatile uint32_t response_ok_count   = 0;
volatile uint32_t rx_timeout_count    = 0;
volatile uint32_t rx_error_count      = 0;
volatile uint32_t cycle_overrun_count = 0;
volatile uint32_t anchor_success_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_timeout_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_rx_error_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_poll_tx_timeout_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_response_timeout_count[TAG_NUM_ANCHORS] = {0};
volatile uint16_t anchor_response_timeout_streak[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_poll_skipped_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_calibration_missing_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_txn_mismatch_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_ds_ok_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_report_timeout_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_ds_fallback_count[TAG_NUM_ANCHORS] = {0};
volatile uint16_t anchor_ds_incomplete_streak[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_probe_count[TAG_NUM_ANCHORS] = {0};
TagRxErrorStats_t tag_rx_error_stats;

/* Adaptive Legacy diagnostics intentionally remain outside the frozen range
 * record. They are available in Live Expressions for SHADOW/ACTIVE tuning. */
volatile uint8_t legacy_adaptive_state[TAG_NUM_ANCHORS] = {0};
volatile uint8_t legacy_adaptive_candidate_count[TAG_NUM_ANCHORS] = {0};
volatile int8_t legacy_adaptive_candidate_direction[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_track_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_track_exit_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_true_reject_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_stale_reacquire_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_track_duration_ms[TAG_NUM_ANCHORS] = {0};
volatile int32_t legacy_adaptive_last_innovation_mm[TAG_NUM_ANCHORS] = {0};
volatile uint32_t legacy_adaptive_max_abs_innovation_mm[TAG_NUM_ANCHORS] = {0};
volatile double legacy_adaptive_last_q[TAG_NUM_ANCHORS] = {0.0};
volatile double legacy_adaptive_last_r[TAG_NUM_ANCHORS] = {0.0};
volatile double legacy_adaptive_last_gain[TAG_NUM_ANCHORS] = {0.0};
const uint8_t uwb_legacy_adaptive_mode_build = (uint8_t)UWB_LEGACY_ADAPTIVE_MODE;

/* C9.2 diagnostics remain out of the frozen range record. The runtime global
 * regime is a quality hint derived from independently filtered anchor states;
 * it never controls radio scheduling or a single anchor's acceptance gate. */
volatile uint8_t c9_2_motion_state[TAG_NUM_ANCHORS] = {0};
volatile uint8_t c9_2_global_motion_state = 0;
volatile uint32_t c9_2_static_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_slow_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_fast_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_settling_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_degraded_enter_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_stale_reacquire_count[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_true_reject_count[TAG_NUM_ANCHORS] = {0};
volatile int32_t c9_2_last_slope_mm_s[TAG_NUM_ANCHORS] = {0};
volatile uint32_t c9_2_last_motion_score_milli[TAG_NUM_ANCHORS] = {0};
volatile double c9_2_last_gain[TAG_NUM_ANCHORS] = {0.0};
const uint8_t uwb_c9_2_motion_mode_build = (uint8_t)UWB_C9_2_MOTION_MODE;

/* DS-TWR counters (Phase 4) — always defined as 0 when flag is OFF */
volatile uint32_t ds_ok_count             = 0;
volatile uint32_t ds_report_ok_count      = 0;
volatile uint32_t ds_report_timeout_count = 0;
volatile uint32_t ds_fallback_count       = 0;
volatile uint32_t ds_final_tx_timeout_count = 0;
volatile uint32_t ds_report_rx_error_count  = 0;
/** C0.1: DS results rejected because the anchor is not calibrated. */
volatile uint32_t ds_cal_missing_count    = 0;

const uint8_t uwb_ds_mode_enabled_build = (uint8_t)UWB_USE_DS_TWR;
const uint8_t uwb_ds_calibrated_mask_build = (uint8_t)UWB_DS_CALIBRATED_MASK;

/* Cycle instrumentation (§5.2 Phase 4) */
volatile uint32_t tag_cycle_duration_us     = 0;
volatile uint32_t tag_cycle_duration_max_us = 0;
volatile uint32_t anchor_slot_duration_us[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_slot_duration_max_us[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_poll_tx_duration_max_us[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_response_wait_max_us[TAG_NUM_ANCHORS] = {0};
volatile uint32_t anchor_processing_max_us[TAG_NUM_ANCHORS] = {0};
static McuCycleStamp_t s_full_cycle_start   = 0;
static McuCycleStamp_t s_slot_start         = 0;

/* Theo dõi kết quả gần nhất của từng anchor (valid/age/quality). */
typedef struct {
    int32_t  raw_mm;
    int32_t  filtered_mm;
    int16_t  fpp_cdbm;
    uint8_t  valid;
    uint8_t  status;
    uint32_t last_success_tick;
    uint16_t consecutive_failures;
} AnchorTrack_t;

static AnchorTrack_t s_track[TAG_NUM_ANCHORS];

static int16_t dbm_to_cdbm(float dbm)
{
    const float centi = dbm * 100.0f;

    if (!(centi > (float)INT16_MIN))   /* also catches NaN */
        return INT16_MIN;
    if (centi > (float)INT16_MAX)
        return INT16_MAX;
    return (int16_t)centi;
}

/**
 * @brief  Ghi nhận kết quả một anchor trong lần thử của chu kỳ hiện tại.
 * @param  idx      chỉ số radio slot (0..TAG_NUM_ANCHORS-1)
 * @param  ok       1 = đo thành công; 0 = thất bại
 * @param  status   TAG_ST_* bit flags
 * @param  raw_mm   khoảng cách (đã calibration) — chỉ dùng khi ok
 * @param  filt_mm  khoảng cách đã lọc — chỉ dùng khi ok
 * @param  fpp_dbm  first-path power dBm — chỉ dùng khi ok
 */
static void mark_anchor_result(uint8_t idx, uint8_t ok, uint8_t status,
                               int32_t raw_mm, int32_t filt_mm, float fpp_dbm)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    AnchorTrack_t *t = &s_track[idx];
    t->status = status;
    if (idx == s_current_anchor)
        s_meas.status = status;
    if (ok)
    {
        t->raw_mm            = raw_mm;
        t->filtered_mm       = filt_mm;
        t->fpp_cdbm          = dbm_to_cdbm(fpp_dbm);
        t->valid             = 1;
        t->last_success_tick = uwb_platform_time_ms();
        t->consecutive_failures = 0;

        if (idx == s_current_anchor)
        {
            s_meas.corrected_mm = raw_mm;
            s_meas.filtered_mm = filt_mm;
            s_meas.flags |= (uint16_t)(TAG_MEAS_FLAG_RADIO_OK | TAG_MEAS_FLAG_CAL_OK
                                       | TAG_MEAS_FLAG_FILTER_OK);
        }
    }
    else
    {
        /* Generic radio/compute failures carry no current measurement.
         * Clear raw/FPP so a previous pre-offset calibration diagnostic cannot
         * survive under an unrelated TIMEOUT/RXERR/COMPUTE status. Keep only
         * filtered_mm + last_success_tick as the explicit last-good hold. */
        t->raw_mm  = 0;
        t->fpp_cdbm = 0;
        t->valid   = 0;
        if (t->consecutive_failures < 0xFFFF)
            t->consecutive_failures++;
    }
}

static void mirror_raw_distance(uint8_t idx, int32_t raw_mm)
{
    distance_raw_mm[idx] = raw_mm;
    switch (anchor_id_at(idx))
    {
        case 0x0001U: distance_a1_mm = raw_mm; break;
        case 0x0002U: distance_a2_mm = raw_mm; break;
        case 0x0003U: distance_a3_mm = raw_mm; break;
        case 0x0004U: distance_a4_mm = raw_mm; break;
        case 0x0005U: distance_a5_mm = raw_mm; break;
        case 0x0006U: distance_a6_mm = raw_mm; break;
        case 0x0007U: distance_a7_mm = raw_mm; break;
        case 0x0008U: distance_a8_mm = raw_mm; break;
        default: break;
    }
}

/**
 * @brief  Lưu measurement diagnostic hiện tại nhưng vẫn đánh dấu production invalid.
 *         Dùng khi profile calibration hiện tại còn thiếu: raw/FPP phải còn để
 *         hiệu chỉnh, nhưng solver không được coi đây là một lần đo thành công.
 */
static void mark_anchor_rejected_measurement(uint8_t idx, uint8_t status,
                                             int32_t raw_mm, float fpp_dbm)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    AnchorTrack_t *t = &s_track[idx];
    t->status   = status;
    t->raw_mm   = raw_mm;
    t->fpp_cdbm = dbm_to_cdbm(fpp_dbm);
    t->valid    = 0U;

    if (idx == s_current_anchor)
    {
        s_meas.status = status;
        s_meas.flags |= TAG_MEAS_FLAG_RADIO_OK;
    }

    /* The UWB exchange and ToF computation succeeded; only the calibration
     * profile is missing. Count it as a ranging operation so Link/Stats do not
     * misleadingly report 0 ops/s, but keep production valid=0 so the solver
     * can never consume this pre-offset value. */
    s_result.ranging_count++;
    response_ok_count++;
    anchor_calibration_missing_count[idx]++;

    /* Expose the pre-offset measurement in debugger mirrors. Telemetry still
     * carries it through the explicit diagnostic channel/status 0x20. */
    mirror_raw_distance(idx, raw_mm);

    if (t->consecutive_failures < 0xFFFFU)
        t->consecutive_failures++;
}

/**
 * @brief Preserve a radio-successful measurement rejected by C9 conditioner.
 *
 * Unlike calibration rejection this must not increment the calibration-missing
 * counter. last_success_tick and the held filtered value are deliberately not
 * updated, so age/valid continue to describe the canonical measurement.
 */
static __attribute__((unused)) void mark_anchor_conditioner_reject(
    uint8_t idx, uint8_t status, int32_t raw_mm, float fpp_dbm)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    AnchorTrack_t *t = &s_track[idx];
    t->status = status;
    t->raw_mm = raw_mm;
    t->fpp_cdbm = dbm_to_cdbm(fpp_dbm);
    t->valid = 0U;
    if (t->consecutive_failures < 0xFFFFU)
        t->consecutive_failures++;

    if (idx == s_current_anchor)
    {
        s_meas.status = status;
        s_meas.corrected_mm = raw_mm;
        s_meas.flags |= (uint16_t)(TAG_MEAS_FLAG_RADIO_OK | TAG_MEAS_FLAG_CAL_OK);
    }

    s_result.ranging_count++;
    s_result.distance_mm = raw_mm;
    response_ok_count++;

    mirror_raw_distance(idx, raw_mm);
}

/* ========================================================================== */
/*                     KALMAN FILTER                                           */
/* ========================================================================== */

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN
typedef struct {
    double Q; /* Process noise */
    double R; /* Measurement noise */
    double x; /* Estimated value */
    double P; /* Estimation error */
    uint8_t  outlier_count;  /* Consecutive outliers */
    uint32_t last_meas_tick; /* Uptime của lần đo được CHẤP NHẬN gần nhất */
} KalmanFilter_t;

static KalmanFilter_t s_kf[TAG_NUM_ANCHORS];
static uint8_t s_kf_initialized[TAG_NUM_ANCHORS] = {0};

#if UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_OFF
static const LegacyAdaptiveTrackingConfig_t s_legacy_adaptive_config = {
    .motion_enter_mm = UWB_LEGACY_ADAPTIVE_MOTION_ENTER_MM,
    .settle_residual_mm = UWB_LEGACY_ADAPTIVE_SETTLE_RESIDUAL_MM,
    .candidate_cluster_mm = UWB_LEGACY_ADAPTIVE_CANDIDATE_CLUSTER_MM,
    .max_radial_speed_mm_s = UWB_DRONE_VMAX_MM_S,
    .gate_margin_up_mm = UWB_GATE_MARGIN_UP_MM,
    .gate_margin_down_mm = UWB_GATE_MARGIN_DOWN_MM,
    .reacquire_min_fpp_dbm = UWB_LEGACY_ADAPTIVE_REACQUIRE_MIN_FPP_DBM,
    .motion_confirm_samples = UWB_LEGACY_ADAPTIVE_MOTION_CONFIRM_SAMPLES,
    .stale_reacquire_samples = UWB_LEGACY_ADAPTIVE_STALE_REACQUIRE_SAMPLES,
    .settle_samples = UWB_LEGACY_ADAPTIVE_SETTLE_SAMPLES,
    .stale_reset_ms = UWB_LEGACY_ADAPTIVE_STALE_RESET_MS,
    .candidate_max_gap_ms = UWB_LEGACY_ADAPTIVE_CANDIDATE_MAX_GAP_MS,
    .nominal_sample_ms = UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS,
    .min_tracking_dt_ms = UWB_LEGACY_ADAPTIVE_TRACK_MIN_DT_MS,
    .max_tracking_dt_ms = UWB_LEGACY_ADAPTIVE_TRACK_MAX_DT_MS,
};

static LegacyAdaptiveTrackingState_t s_legacy_adaptive_state[TAG_NUM_ANCHORS];

#if UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_SHADOW
/* SHADOW owns independent range state. It can never alter the published
 * Legacy state, even if the candidate has a bug or an extreme observation. */
static KalmanFilter_t s_legacy_adaptive_shadow_kf[TAG_NUM_ANCHORS];
static uint8_t s_legacy_adaptive_shadow_initialized[TAG_NUM_ANCHORS] = {0};
#endif
#endif

/* ========================================================================== */
/*                     MEDIAN FILTER                                           */
/* ========================================================================== */

typedef struct {
    int32_t buf[3];
    uint8_t idx;
    uint8_t count;
} MedianFilter_t;

static MedianFilter_t s_mf[TAG_NUM_ANCHORS];

#if UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_SHADOW
static MedianFilter_t s_legacy_adaptive_shadow_mf[TAG_NUM_ANCHORS];
#endif

#if UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_OFF
/* C9.2 owns an entirely separate candidate state in SHADOW. ACTIVE reuses
 * this state as the only published candidate; it never aliases s_kf/s_mf. */
static MotionAdaptiveRangeState_t s_c9_2_motion[TAG_NUM_ANCHORS];
static KalmanFilter_t s_c9_2_kf[TAG_NUM_ANCHORS];
static uint8_t s_c9_2_kf_initialized[TAG_NUM_ANCHORS] = {0};
static MedianFilter_t s_c9_2_mf[TAG_NUM_ANCHORS];

static const MotionAdaptiveRangeConfig_t s_c9_2_motion_config = {
    .slow_enter_z = UWB_C9_2_SLOW_ENTER_Z,
    .slow_exit_z = UWB_C9_2_SLOW_EXIT_Z,
    .fast_enter_z = UWB_C9_2_FAST_ENTER_Z,
    .fast_exit_z = UWB_C9_2_FAST_EXIT_Z,
    .cusum_drift_z = UWB_C9_2_CUSUM_DRIFT_Z,
    .slow_speed_mm_s = UWB_C9_2_SLOW_SPEED_MM_S,
    .fast_speed_mm_s = UWB_C9_2_FAST_SPEED_MM_S,
    .slow_gain = UWB_C9_2_SLOW_GAIN,
    .fast_gain = UWB_C9_2_FAST_GAIN,
    .settling_gain = UWB_C9_2_SETTLING_GAIN,
    .minimum_sigma_mm = UWB_C9_2_MINIMUM_SIGMA_MM,
    .reacquire_min_fpp_dbm = UWB_C9_2_REACQUIRE_MIN_FPP_DBM,
    .reacquire_cluster_mm = UWB_C9_2_REACQUIRE_CLUSTER_MM,
    .max_radial_speed_mm_s = UWB_DRONE_VMAX_MM_S,
    .gate_margin_up_mm = UWB_GATE_MARGIN_UP_MM,
    .gate_margin_down_mm = UWB_GATE_MARGIN_DOWN_MM,
    .slow_confirm_samples = UWB_C9_2_SLOW_CONFIRM_SAMPLES,
    .fast_confirm_samples = UWB_C9_2_FAST_CONFIRM_SAMPLES,
    .settle_dwell_samples = UWB_C9_2_SETTLE_DWELL_SAMPLES,
    .static_dwell_samples = UWB_C9_2_STATIC_DWELL_SAMPLES,
    .reacquire_samples = UWB_C9_2_REACQUIRE_SAMPLES,
    .degrade_after_rejects = UWB_C9_2_DEGRADE_AFTER_REJECTS,
    .stale_reset_ms = UWB_C9_2_STALE_RESET_MS,
    .candidate_max_gap_ms = UWB_C9_2_CANDIDATE_MAX_GAP_MS,
    .min_dt_ms = UWB_C9_2_MIN_DT_MS,
    .max_dt_ms = UWB_C9_2_MAX_DT_MS,
};
#endif

static int32_t apply_median_filter(MedianFilter_t* mf, int32_t new_val)
{
    /* Add to circular buffer */
    mf->buf[mf->idx] = new_val;
    mf->idx = (mf->idx + 1) % 3;
    if (mf->count < 3) mf->count++;

    /* If not enough samples, just return the raw value */
    if (mf->count < 3) return new_val;

    /* Sort the 3 values to find the median */
    int32_t a = mf->buf[0];
    int32_t b = mf->buf[1];
    int32_t c = mf->buf[2];

    if (a > b) { int32_t tmp = a; a = b; b = tmp; }
    if (b > c) { int32_t tmp = b; b = c; c = tmp; }
    if (a > b) { int32_t tmp = a; a = b; b = tmp; }

    return b; /* Median is in the middle */
}

/* ========================================================================== */
/*                     OUTLIER GATE                                            */
/* ========================================================================== */

/**
 * @brief  Outlier gate động học theo vận tốc × thời gian thực đã trôi (Task 1, Phase 4).
 *         Ngưỡng tự nới khi drone di chuyển nhanh hoặc khi bị timeout vài chu kỳ.
 *         Vẫn giữ bất đối xứng: multipath chỉ tăng khoảng cách, hướng dương chặt hơn.
 * @retval 1 if measurement is an outlier (rejected)
 * @retval 0 if measurement is normal (or snap backstop triggered)
 */
static uint8_t apply_outlier_gate_at(KalmanFilter_t* kf, double meas,
                                     uint32_t now, uint8_t allow_legacy_snap)
{
    uint32_t dt_ms = now - kf->last_meas_tick;
    /* Clamp: tối thiểu 1 chu kỳ, tối đa 500ms — quá 500ms ngưỡng rộng
     * gần như tắt gate, backstop snap sẽ xử lý phần còn lại. */
    if (dt_ms < TAG_CYCLE_MS) dt_ms = TAG_CYCLE_MS;
    if (dt_ms > 500) dt_ms = 500;
    double dt_s = (double)dt_ms / 1000.0;

    double up_thr   =  UWB_DRONE_VMAX_MM_S * dt_s + UWB_GATE_MARGIN_UP_MM;
    double down_thr = -(UWB_DRONE_VMAX_MM_S * dt_s + UWB_GATE_MARGIN_DOWN_MM);

    double jump = meas - kf->x;
    if (jump > up_thr || jump < down_thr) {
        if (kf->outlier_count < 0xFFU)
            kf->outlier_count++;
        /* KHÔNG cập nhật last_meas_tick khi reject → dt tự lớn dần → ngưỡng tự
         * nới → tự phục hồi nhanh, không cần chờ đủ snap count trong đa số case. */
        if (allow_legacy_snap == 0U || kf->outlier_count < UWB_GATE_SNAP_AFTER) {
            return 1;
        }
        kf->x            = meas;   /* snap backstop */
        kf->outlier_count  = 0;
        kf->last_meas_tick = now;
        return 0;
    }
    kf->outlier_count  = 0;
    kf->last_meas_tick = now;
    return 0;
}

/* Retain the deployed Legacy call path unchanged. Adaptive builds use the
 * explicit-timestamp form above so host tests and offline-anchor timing use
 * the same wrap-safe elapsed-time semantics. */
#if UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_ACTIVE
static uint8_t apply_outlier_gate(KalmanFilter_t* kf, double meas)
{
    return apply_outlier_gate_at(kf, meas, uwb_platform_time_ms(), 1U);
}
#endif

/* ========================================================================== */
/*                     KALMAN UPDATE                                           */
/* ========================================================================== */

static double kalman_measurement_noise(float fpp)
{
    /* FPP is a noise hint, not a validity threshold: calibrated DS captures
     * can legitimately be around -97 dBm. */
    if (fpp <= -95.0f) {
        return 10000.0;
    } else if (fpp > -75.0f) {
        return 50.0;
    } else if (fpp > -82.0f) {
        return 200.0;
    }
    return 1000.0;  /* Medium/noisy signal -> filter heavily. */
}

static double kalman_update_with_gain(KalmanFilter_t* kf, double meas,
                                      float fpp, double *gain_out)
{
    kf->R = kalman_measurement_noise(fpp);

    /* Prediction (static model) */
    kf->P = kf->P + kf->Q;

    /* Update */
    double K = kf->P / (kf->P + kf->R);
    kf->x = kf->x + K * (meas - kf->x);
    kf->P = (1.0 - K) * kf->P;
    if (gain_out != NULL)
        *gain_out = K;

    return kf->x;
}

#if UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_OFF
typedef struct {
    uint8_t publish_valid;
    uint8_t reacquired;
    int32_t filtered_mm;
    double innovation_mm;
    double gain;
} C9_2MotionProcessResult_t;

static double c9_2_static_posterior_covariance(double measurement_noise)
{
    const double q = UWB_LEGACY_BASE_PROCESS_NOISE;
    return (sqrt(q * q + 4.0 * q * measurement_noise) - q) * 0.5;
}

static void c9_2_kalman_initialize(KalmanFilter_t *kf, double measurement_mm,
                                   float fpp, uint32_t now_ms)
{
    const double measurement_noise = kalman_measurement_noise(fpp);

    kf->x = measurement_mm;
    kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
    kf->R = measurement_noise;
    kf->P = c9_2_static_posterior_covariance(measurement_noise);
    kf->outlier_count = 0U;
    kf->last_meas_tick = now_ms;
}

/* Set P/Q so the first update in a new dynamic regime has exactly target K.
 * This makes the response deterministic across FPP/R, rather than relying on
 * a process-noise value tuned only for one signal strength. */
static void c9_2_seed_dynamic_gain(KalmanFilter_t *kf,
                                   double measurement_noise,
                                   double target_gain,
                                   uint32_t dt_ms)
{
    double dt_scale;

    if (target_gain <= 0.0 || target_gain >= 0.95)
        return;
    dt_scale = (double)dt_ms / (double)UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS;
    kf->R = measurement_noise;
    kf->Q = measurement_noise * target_gain * target_gain
          / (1.0 - target_gain) * dt_scale;
    /* kalman_update_with_gain adds Q before K.  P=R*K gives K=target_gain
     * for the first dynamic update at nominal dt. */
    kf->P = measurement_noise * target_gain;
}

static int32_t c9_2_to_i32(double value)
{
    if (value > (double)INT32_MAX)
        return INT32_MAX;
    if (value < (double)INT32_MIN)
        return INT32_MIN;
    return (int32_t)value;
}

static uint32_t c9_2_score_to_milli(float score)
{
    double scaled = (double)score * 1000.0;
    if (scaled <= 0.0)
        return 0U;
    if (scaled >= (double)UINT32_MAX)
        return UINT32_MAX;
    return (uint32_t)scaled;
}

static void c9_2_publish_diagnostics(uint8_t anchor_index,
                                     const MotionAdaptiveRangeState_t *motion,
                                     double gain)
{
    if (anchor_index >= TAG_NUM_ANCHORS)
        return;

    c9_2_motion_state[anchor_index] = motion->mode;
    c9_2_static_enter_count[anchor_index] = motion->static_enter_count;
    c9_2_slow_enter_count[anchor_index] = motion->slow_enter_count;
    c9_2_fast_enter_count[anchor_index] = motion->fast_enter_count;
    c9_2_settling_enter_count[anchor_index] = motion->settling_enter_count;
    c9_2_degraded_enter_count[anchor_index] = motion->degraded_enter_count;
    c9_2_stale_reacquire_count[anchor_index] = motion->stale_reacquire_count;
    c9_2_true_reject_count[anchor_index] = motion->true_reject_count;
    c9_2_last_slope_mm_s[anchor_index] = c9_2_to_i32(motion->last_slope_mm_s);
    c9_2_last_motion_score_milli[anchor_index] =
        c9_2_score_to_milli(motion->last_motion_score);
    c9_2_last_gain[anchor_index] = gain;
}

/* Aggregate only for telemetry/UI.  It cannot alter the per-anchor state
 * machines: a single degraded anchor must not slow healthy anchors. */
static void c9_2_refresh_global_motion_state(void)
{
    uint8_t index;
    uint8_t static_count = 0U;
    uint8_t slow_count = 0U;
    uint8_t fast_count = 0U;
    uint8_t settling_count = 0U;
    uint8_t degraded_count = 0U;
    uint8_t reacquire_count = 0U;

    for (index = 0U; index < TAG_NUM_ANCHORS; index++)
    {
        uint8_t mode = c9_2_motion_state[index];
        if (mode == (uint8_t)MOTION_RANGE_FAST)
            fast_count++;
        else if (mode == (uint8_t)MOTION_RANGE_SLOW)
            slow_count++;
        else if (mode == (uint8_t)MOTION_RANGE_SETTLING)
            settling_count++;
        else if (mode == (uint8_t)MOTION_RANGE_STATIC)
            static_count++;
        else if (mode == (uint8_t)MOTION_RANGE_DEGRADED)
            degraded_count++;
        else
            reacquire_count++;
    }

    if (degraded_count != 0U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_DEGRADED;
    else if (fast_count >= 2U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_FAST;
    else if ((slow_count + fast_count) >= 2U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_SLOW;
    else if (settling_count >= 2U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_SETTLING;
    else if (static_count >= 3U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_STATIC;
    else if (reacquire_count != 0U)
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_REACQUIRE;
    else
        c9_2_global_motion_state = (uint8_t)MOTION_RANGE_STATIC;
}

static C9_2MotionProcessResult_t c9_2_motion_process(
    uint8_t anchor_index,
    int32_t raw_mm,
    float fpp,
    uint32_t now_ms)
{
    C9_2MotionProcessResult_t result = {0U, 0U, 0, 0.0, 0.0};
    MotionAdaptiveRangeState_t *motion = &s_c9_2_motion[anchor_index];
    KalmanFilter_t *kf = &s_c9_2_kf[anchor_index];
    int32_t median_mm;
    double measurement_noise;
    double sigma_mm;
    uint32_t dt_ms = UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS;

    if (MotionAdaptiveRange_BeginSample(motion, now_ms,
                                        &s_c9_2_motion_config) != 0U)
    {
        /* A discontinuity invalidates both median history and old covariance. */
        memset(&s_c9_2_mf[anchor_index], 0, sizeof(s_c9_2_mf[anchor_index]));
        s_c9_2_kf_initialized[anchor_index] = 0U;
    }

    median_mm = apply_median_filter(&s_c9_2_mf[anchor_index], raw_mm);

    if (motion->last_median_tick != 0U)
    {
        dt_ms = MotionAdaptiveRange_ClampU32(
            MotionAdaptiveRange_ElapsedMs(now_ms, motion->last_median_tick),
            s_c9_2_motion_config.min_dt_ms,
            s_c9_2_motion_config.max_dt_ms);
    }

    if (motion->mode == (uint8_t)MOTION_RANGE_REACQUIRE
        || motion->mode == (uint8_t)MOTION_RANGE_DEGRADED)
    {
        float reacquired_mm = 0.0f;
        if (MotionAdaptiveRange_ObserveReacquire(
                motion, (float)median_mm, fpp, now_ms,
                &s_c9_2_motion_config, &reacquired_mm) == 0U)
        {
            c9_2_publish_diagnostics(anchor_index, motion, 0.0);
            c9_2_refresh_global_motion_state();
            return result;
        }

        c9_2_kalman_initialize(kf, (double)reacquired_mm, fpp, now_ms);
        s_c9_2_kf_initialized[anchor_index] = 1U;
        MotionAdaptiveRange_NoteAcceptedMedian(
            motion, reacquired_mm, now_ms, &s_c9_2_motion_config);
        result.publish_valid = 1U;
        result.reacquired = 1U;
        result.filtered_mm = c9_2_to_i32(reacquired_mm);
        result.gain = 1.0;
        c9_2_publish_diagnostics(anchor_index, motion, result.gain);
        c9_2_refresh_global_motion_state();
        return result;
    }

    if (s_c9_2_kf_initialized[anchor_index] == 0U)
    {
        /* This normally only occurs when mode was changed in a debugger.
         * Restart cleanly rather than mixing a filter from a former profile. */
        MotionAdaptiveRange_Enter(motion, (uint8_t)MOTION_RANGE_REACQUIRE);
        c9_2_publish_diagnostics(anchor_index, motion, 0.0);
        c9_2_refresh_global_motion_state();
        return result;
    }

    result.innovation_mm = (double)median_mm - kf->x;
    if (MotionAdaptiveRange_AcceptKinematic(
            motion, (float)median_mm, now_ms, &s_c9_2_motion_config) == 0U)
    {
        MotionAdaptiveRange_NoteReject(motion, &s_c9_2_motion_config);
        if (motion->mode == (uint8_t)MOTION_RANGE_DEGRADED)
        {
            /* Do not let the outlier that caused degradation contaminate the
             * new physical-location candidate. The next three compatible
             * medians are therefore a bounded 60 ms reacquire at 50 Hz. */
            memset(&s_c9_2_mf[anchor_index], 0, sizeof(s_c9_2_mf[anchor_index]));
            s_c9_2_kf_initialized[anchor_index] = 0U;
            motion->last_median_mm = 0.0f;
            motion->last_median_tick = 0U;
            motion->slope_count = 0U;
            motion->slope_head = 0U;
            motion->last_slope_mm_s = 0.0f;
            motion->last_motion_score = 0.0f;
        }
        c9_2_publish_diagnostics(anchor_index, motion, 0.0);
        c9_2_refresh_global_motion_state();
        return result;
    }

    measurement_noise = kalman_measurement_noise(fpp);
    sigma_mm = sqrt(measurement_noise);
    (void)MotionAdaptiveRange_ObserveAccepted(
        motion, (float)median_mm, (float)result.innovation_mm, (float)sigma_mm,
        now_ms, &s_c9_2_motion_config);

    if (MotionAdaptiveRange_IsDynamic(motion->mode) != 0U)
    {
        const double target_gain = (double)MotionAdaptiveRange_TargetGain(
            motion, &s_c9_2_motion_config);
        c9_2_seed_dynamic_gain(kf, measurement_noise, target_gain, dt_ms);
    }
    else
    {
        kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
        kf->R = measurement_noise;
        kf->P = c9_2_static_posterior_covariance(measurement_noise);
    }

    result.filtered_mm = c9_2_to_i32(kalman_update_with_gain(
        kf, (double)median_mm, fpp, &result.gain));
    result.publish_valid = 1U;
    c9_2_publish_diagnostics(anchor_index, motion, result.gain);
    c9_2_refresh_global_motion_state();
    return result;
}
#endif

#if UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_OFF
typedef struct {
    uint8_t publish_valid;
    int32_t filtered_mm;
    double innovation_mm;
    double gain;
} LegacyAdaptiveProcessResult_t;

static double legacy_stable_posterior_covariance(double measurement_noise)
{
    const double q = UWB_LEGACY_BASE_PROCESS_NOISE;
    return (sqrt(q * q + 4.0 * q * measurement_noise) - q) * 0.5;
}

static void legacy_kalman_initialize(KalmanFilter_t *kf, int32_t measurement_mm,
                                     uint32_t now_ms)
{
    kf->x = (double)measurement_mm;
    kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
    kf->P = 1.0;
    kf->R = 0.0;
    kf->outlier_count = 0U;
    kf->last_meas_tick = now_ms;
}

static double legacy_tracking_process_noise(double measurement_noise,
                                            uint32_t dt_ms)
{
    const double tracking_gain = UWB_LEGACY_ADAPTIVE_TRACK_GAIN;
    const double nominal_q = measurement_noise * tracking_gain * tracking_gain
                           / (1.0 - tracking_gain);
    const double dt_scale = (double)dt_ms
                          / (double)UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS;
    return nominal_q * dt_scale;
}

static void legacy_seed_tracking_gain(KalmanFilter_t *kf,
                                      double measurement_noise)
{
    const double tracking_gain = UWB_LEGACY_ADAPTIVE_TRACK_GAIN;
    kf->R = measurement_noise;
    kf->Q = legacy_tracking_process_noise(
        measurement_noise, UWB_LEGACY_ADAPTIVE_NOMINAL_SAMPLE_MS);
    /* kalman_update adds Q before forming K. This P/Q pair gives the first
     * tracking update the requested K_track, even at weak FPP (R=10000). */
    kf->P = measurement_noise * tracking_gain;
}

static int32_t legacy_adaptive_innovation_to_i32(double innovation_mm)
{
    if (innovation_mm > (double)INT32_MAX)
        return INT32_MAX;
    if (innovation_mm < (double)INT32_MIN)
        return INT32_MIN;
    return (int32_t)innovation_mm;
}

static uint32_t legacy_adaptive_abs_innovation_to_u32(double innovation_mm)
{
    const double absolute_mm = innovation_mm < 0.0 ? -innovation_mm : innovation_mm;
    if (absolute_mm > (double)UINT32_MAX)
        return UINT32_MAX;
    return (uint32_t)absolute_mm;
}

static void legacy_adaptive_publish_diagnostics(
    uint8_t anchor_index,
    const LegacyAdaptiveTrackingState_t *adaptive,
    const KalmanFilter_t *kf,
    double innovation_mm,
    double gain,
    uint32_t now_ms)
{
    uint32_t absolute_innovation;

    if (anchor_index >= TAG_NUM_ANCHORS)
        return;

    absolute_innovation = legacy_adaptive_abs_innovation_to_u32(innovation_mm);
    legacy_adaptive_state[anchor_index] = adaptive->state;
    legacy_adaptive_candidate_count[anchor_index] = adaptive->candidate_count;
    legacy_adaptive_candidate_direction[anchor_index] = adaptive->candidate_direction;
    legacy_adaptive_track_enter_count[anchor_index] = adaptive->track_enter_count;
    legacy_adaptive_track_exit_count[anchor_index] = adaptive->track_exit_count;
    legacy_adaptive_true_reject_count[anchor_index] = adaptive->true_reject_count;
    legacy_adaptive_stale_reacquire_count[anchor_index] = adaptive->stale_reacquire_count;
    legacy_adaptive_last_innovation_mm[anchor_index] =
        legacy_adaptive_innovation_to_i32(innovation_mm);
    if (absolute_innovation > legacy_adaptive_max_abs_innovation_mm[anchor_index])
        legacy_adaptive_max_abs_innovation_mm[anchor_index] = absolute_innovation;
    legacy_adaptive_last_q[anchor_index] = kf->Q;
    legacy_adaptive_last_r[anchor_index] = kf->R;
    legacy_adaptive_last_gain[anchor_index] = gain;
    legacy_adaptive_track_duration_ms[anchor_index] =
        adaptive->state == (uint8_t)LEGACY_ADAPTIVE_STATE_TRACKING
            ? LegacyAdaptive_ElapsedMs(now_ms, adaptive->tracking_start_tick)
            : 0U;
}

/**
 * @brief Controlled candidate used by SHADOW and ACTIVE Adaptive Legacy.
 *
 * ACTIVE returns publish_valid=0 during a stale reacquire or an unconfirmed
 * candidate that also breaches the existing dynamic gate, preserving the last
 * canonical range rather than feeding a likely spike into the solver. A
 * candidate still inside the proven Legacy gate remains on the exact baseline
 * path until it has enough evidence to enter TRACK. SHADOW follows the same
 * candidate logic on independent filter state, but its return value is
 * intentionally ignored by production.
 */
static LegacyAdaptiveProcessResult_t legacy_adaptive_process(
    KalmanFilter_t *kf,
    uint8_t *kf_initialized,
    MedianFilter_t *median,
    LegacyAdaptiveTrackingState_t *adaptive,
    int32_t raw_mm,
    float fpp,
    uint32_t now_ms)
{
    LegacyAdaptiveProcessResult_t result = {0U, 0, 0.0, 0.0};
    int32_t median_mm;
    double measurement_noise;
    uint8_t legacy_gate_rejected;
    uint8_t entered_tracking;

    if (LegacyAdaptiveTracking_BeginSample(adaptive, now_ms,
                                           &s_legacy_adaptive_config) != 0U)
    {
        /* Never mix a pre-gap median window with a new physical location. */
        memset(median, 0, sizeof(*median));
    }

    median_mm = apply_median_filter(median, raw_mm);

    if (adaptive->state == (uint8_t)LEGACY_ADAPTIVE_STATE_STALE_REACQUIRE)
    {
        double reacquired_mm = 0.0;
        if (LegacyAdaptiveTracking_ObserveStaleReacquire(
                adaptive, (double)median_mm, fpp, now_ms,
                &s_legacy_adaptive_config, &reacquired_mm) == 0U)
        {
            result.innovation_mm = (double)median_mm - kf->x;
            legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                                result.innovation_mm, 0.0, now_ms);
            return result;
        }

        measurement_noise = kalman_measurement_noise(fpp);
        kf->x = reacquired_mm;
        kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
        kf->R = measurement_noise;
        kf->P = legacy_stable_posterior_covariance(measurement_noise);
        kf->outlier_count = 0U;
        kf->last_meas_tick = now_ms;
        *kf_initialized = 1U;
        result.publish_valid = 1U;
        result.filtered_mm = (int32_t)reacquired_mm;
        result.innovation_mm = 0.0;
        result.gain = 1.0; /* Explicit controlled stale reacquire, not a jump hidden as K. */
        legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                            result.innovation_mm, result.gain, now_ms);
        return result;
    }

    if (*kf_initialized == 0U)
    {
        legacy_kalman_initialize(kf, median_mm, now_ms);
        *kf_initialized = 1U;
    }

    result.innovation_mm = (double)median_mm - kf->x;

    if (adaptive->state == (uint8_t)LEGACY_ADAPTIVE_STATE_TRACKING)
    {
        uint32_t tracking_dt_ms = LegacyAdaptiveTracking_TrackingDtMs(
            adaptive, now_ms, &s_legacy_adaptive_config);
        if (LegacyAdaptiveTracking_AcceptTrackingMeasurement(
                adaptive, (double)median_mm, now_ms,
                &s_legacy_adaptive_config) == 0U)
        {
            legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                                result.innovation_mm, 0.0, now_ms);
            return result;
        }

        measurement_noise = kalman_measurement_noise(fpp);
        kf->Q = legacy_tracking_process_noise(measurement_noise, tracking_dt_ms);
        result.filtered_mm = (int32_t)kalman_update_with_gain(
            kf, (double)median_mm, fpp, &result.gain);
        LegacyAdaptiveTracking_NoteTrackingUpdate(adaptive, now_ms);
        if (LegacyAdaptiveTracking_ObserveSettled(
                adaptive, (double)median_mm, kf->x,
                &s_legacy_adaptive_config) != 0U)
        {
            /* Restore the Legacy steady-state covariance immediately after
             * movement settles, so a prior fast-tracking P cannot leak noise
             * into the same smooth static output the user has approved. */
            kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
            kf->P = legacy_stable_posterior_covariance(kf->R);
        }
        result.publish_valid = 1U;
        legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                            result.innovation_mm, result.gain, now_ms);
        return result;
    }

    /* Outside TRACK, retain the existing dt-aware gate but disable its legacy
     * 15-sample snap. ACTIVE accepts a large shift only after the explicit
     * four-sample candidate; SHADOW records the same decision independently. */
    legacy_gate_rejected = apply_outlier_gate_at(kf, (double)median_mm, now_ms, 0U);
    entered_tracking = LegacyAdaptiveTracking_ObserveMotionCandidate(
        adaptive, (double)median_mm, kf->x, now_ms, &s_legacy_adaptive_config);

    if (entered_tracking != 0U)
    {
        measurement_noise = kalman_measurement_noise(fpp);
        legacy_seed_tracking_gain(kf, measurement_noise);
        kf->outlier_count = 0U;
        kf->last_meas_tick = now_ms;
        (void)LegacyAdaptiveTracking_AcceptTrackingMeasurement(
            adaptive, (double)median_mm, now_ms, &s_legacy_adaptive_config);
        result.filtered_mm = (int32_t)kalman_update_with_gain(
            kf, (double)median_mm, fpp, &result.gain);
        LegacyAdaptiveTracking_NoteTrackingUpdate(adaptive, now_ms);
        result.publish_valid = 1U;
        legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                            result.innovation_mm, result.gain, now_ms);
        return result;
    }

    if (legacy_gate_rejected != 0U)
    {
        adaptive->true_reject_count++;
        legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                            result.innovation_mm, 0.0, now_ms);
        return result;
    }

    /* STABLE/CANDIDATE normal samples retain the original Legacy Q/R model. */
    kf->Q = UWB_LEGACY_BASE_PROCESS_NOISE;
    result.filtered_mm = (int32_t)kalman_update_with_gain(
        kf, (double)median_mm, fpp, &result.gain);
    result.publish_valid = 1U;
    legacy_adaptive_publish_diagnostics(s_current_anchor, adaptive, kf,
                                        result.innovation_mm, result.gain, now_ms);
    return result;
}
#endif
#else
/* Candidate state is global so counters and decisions remain visible in Live
 * Expressions without changing the frozen 16-byte range record. */
RangeFilterState_t range_filter_state[TAG_NUM_ANCHORS];
volatile uint8_t range_filter_last_decision[TAG_NUM_ANCHORS] = {0};
volatile int32_t range_filter_last_innovation_mm[TAG_NUM_ANCHORS] = {0};
volatile uint32_t range_filter_max_abs_innovation_mm[TAG_NUM_ANCHORS] = {0};
#endif


/* ========================================================================== */
/*                     PRIVATE HELPERS                                         */
/* ========================================================================== */

static uint64_t ts_to_u64(const uint8_t *ts)
{
    uint64_t v = 0;
    v  = (uint64_t)ts[0];
    v |= (uint64_t)ts[1] <<  8;
    v |= (uint64_t)ts[2] << 16;
    v |= (uint64_t)ts[3] << 24;
    v |= (uint64_t)ts[4] << 32;
    return v;
}

static inline uint8_t anchor_active(uint8_t idx)
{
    return (uint8_t)((s_active_anchor_mask >> idx) & 1U);
}

/** Record which RX failure occurred; the status can hold several flags. */
static void count_rx_errors(uint64_t status)
{
    if (status & DW_RXPHE_BIT)   tag_rx_error_stats.phy_header++;
    if (status & DW_RXFCE_BIT)   tag_rx_error_stats.fcs++;
    if (status & DW_RXRFSL_BIT)  tag_rx_error_stats.sync_loss++;
    if (status & DW_RXRFTO_BIT)  tag_rx_error_stats.frame_timeout++;
    if (status & DW_LDEERR_BIT)  tag_rx_error_stats.lde++;
    if (status & DW_RXOVRR_BIT)  tag_rx_error_stats.overrun++;
    if (status & DW_RXPTO_BIT)   tag_rx_error_stats.preamble_to++;
    if (status & DW_RXSFDTO_BIT) tag_rx_error_stats.sfd_timeout++;
    if (status & DW_AFFREJ_BIT)  tag_rx_error_stats.filtered++;
    if ((status & DW_ALL_RX_ERR) == 0U)
        tag_rx_error_stats.incomplete++;
}

/** Stray or foreign frame: re-arm the receiver, keep the slot deadline. */
static void restart_rx_keep_deadline(void)
{
    DW1000_ClearAllStatus();
    DW1000_StartRx();
}

/**
 * RX error: soft-reset the receiver so the LDE restarts cleanly, then keep
 * listening until the original deadline — the real reply may still come.
 */
static void recover_rx_after_error(uint64_t status)
{
    count_rx_errors(status);
    DW1000_ForceRxOff();
    DW1000_RxSoftReset();
    tag_rx_error_stats.soft_resets++;
    restart_rx_keep_deadline();
    s_slot_rx_error = 1U;
}

/** Start a TX that expects a reply, with WAIT4RESP when enabled. */
static void start_tx_expecting_reply(void)
{
#if UWB_USE_WAIT4RESP
    DW1000_StartTxWait4Resp();
#else
    DW1000_StartTx();
#endif
}

/** After TXFRS of a frame sent by start_tx_expecting_reply(). */
static void arm_rx_after_tx(void)
{
#if UWB_USE_WAIT4RESP
    /* The receiver is already on: clear TX events only, so an RX event that
     * may already be pending is not lost. */
    DW1000_ClearTxStatus();
#else
    DW1000_ClearAllStatus();
    DW1000_StartRx();
#endif
}

/**
 * @brief  Build and fire a POLL frame to the specified anchor.
 */
static void send_poll(uint16_t target_anchor)
{
    uint8_t frame[UWB_FRAME_HEADER_LEN + 3U];
    UwbPoll_t poll = {
        .version = UWB_TAG_FRAME_VERSION,
        .txn = s_txn,
        .flags = ((int8_t)s_current_anchor == s_info_request_idx)
            ? UWB_POLL_FLAG_REQ_INFO : 0U,
    };
    const uint16_t len = uwb_frame_build_poll(frame, s_seq_num++, DW_PAN_ID,
                                              target_anchor, TAG_ADDR, &poll);

    DW1000_ClearAllStatus();
    DW1000_WriteTxData(frame, len);
    DW1000_SetTxFrameCtrl((uint16_t)(len + UWB_FRAME_FCS_LEN));
    start_tx_expecting_reply();

    poll_sent_count++;
}

/** Reset the per-slot bookkeeping before the POLL of a new slot. */
static void begin_slot(void)
{
    s_txn++;
    s_slot_rx_error = 0U;
    memset(&s_meas, 0, sizeof(s_meas));
    s_meas.anchor_id = anchor_id_at(s_current_anchor);
    s_meas.txn = s_txn;
    s_meas.fp_cdbm = INT16_MIN;
    s_meas.rx_cdbm = INT16_MIN;
    s_meas.anchor_fp_cdbm = INT16_MIN;
    s_meas.anchor_rx_cdbm = INT16_MIN;
    s_slot_start = MCU_CycleNow();
}

/** Copy the RESP receive diagnostics into the slot measurement. */
static void capture_resp_diagnostics(const DW1000_SignalDiag_t *diag, float fpp,
                                     int32_t carrier_integrator)
{
    const int32_t ppm_x100 = DW1000_CarrierIntegratorToPpmX100(carrier_integrator);

    s_meas.fp_cdbm = dbm_to_cdbm(fpp);
    s_meas.rx_cdbm = dbm_to_cdbm(DW1000_GetRxPower(diag));
    s_meas.std_noise = diag->std_noise;
    s_meas.fp_index = diag->fp_index;
    s_meas.ci_ppm_x100 = (int16_t)(ppm_x100 > INT16_MAX ? INT16_MAX
                                   : (ppm_x100 < INT16_MIN ? INT16_MIN : ppm_x100));
}

/** Store anchor information TLVs that arrived in a RESP. */
static void capture_anchor_info(uint8_t idx, const UwbResp_t *resp)
{
    TagAnchorInfo_t *info = &s_anchor_info[idx];
    const uint8_t *pos = uwb_frame_find_tlv(resp->tlv, resp->tlv_len,
                                            UWB_TLV_ANCHOR_POSITION,
                                            UWB_TLV_ANCHOR_POSITION_LEN);
    const uint8_t *build = uwb_frame_find_tlv(resp->tlv, resp->tlv_len,
                                               UWB_TLV_ANCHOR_BUILD,
                                               UWB_TLV_ANCHOR_BUILD_LEN);
    const uint8_t *config = uwb_frame_find_tlv(resp->tlv, resp->tlv_len,
                                               UWB_TLV_ANCHOR_CONFIG,
                                               UWB_TLV_ANCHOR_CONFIG_LEN);

    if (pos == NULL && build == NULL && config == NULL)
        return;

    info->anchor_id = anchor_id_at(idx);
    info->anchor_status = resp->anchor_status;
    if (pos != NULL)
    {
        info->pos_valid = (resp->anchor_status & UWB_ANCHOR_ST_POS_VALID) != 0U;
        info->pos_mm[0] = (int32_t)uwb_get_u32(&pos[0]);
        info->pos_mm[1] = (int32_t)uwb_get_u32(&pos[4]);
        info->pos_mm[2] = (int32_t)uwb_get_u32(&pos[8]);
    }
    if (build != NULL)
    {
        info->build_hash = uwb_get_u32(&build[0]);
        info->build_config_hash = 0U;    /* old anchor without CONFIG TLV */
        info->build_dirty = build[4];
        info->tx_ant_dly = uwb_get_u16(&build[5]);
        info->rx_ant_dly = uwb_get_u16(&build[7]);
        info->tx_power_mode = build[9];
        info->boot_count = uwb_get_u16(&build[10]);
    }
    if (config != NULL)
        info->build_config_hash = uwb_get_u32(config);
    info->updated_ms = uwb_platform_time_ms();
    info->valid = 1U;
    s_anchor_info_pending |= (uint8_t)(1U << idx);
}

/* Forward declaration — compute_distance_ss_terms_mm gọi apply_offset_and_clamp
 * nhưng hàm đó được định nghĩa sau. Đặt ngoài #if để luôn visible (FIX-04 §8.4).
 * C0: nhận thêm tham số mode để chọn đúng calibration profile (P0 fix). */
static int32_t apply_offset_and_clamp(double dist_m, UwbRangingMode_t mode);
static uint8_t meters_to_mm_checked(double dist_m, int32_t *out_mm);

/**
 * @brief  SS-TWR core calculation shared by SS path and DS fallback, with the
 *         carrier integrator correction (FIX-04 §8.2).
 * @retval Distance in mm (>=0), RANGE_COMPUTE_ERROR, or
 *         RANGE_CALIBRATION_MISSING.
 */
static int32_t compute_distance_ss_terms_mm(const uint8_t *resp_rx_ts_raw,
                                             uint32_t t_reply_ticks,
                                             int32_t carrier_integrator,
                                             UwbRangingMode_t mode,
                                             int32_t *uncalibrated_raw_mm)
{
    if (uncalibrated_raw_mm == NULL)
        return RANGE_COMPUTE_ERROR;

    *uncalibrated_raw_mm = 0;

    uint64_t poll_tx = ts_to_u64(s_poll_tx_ts);
    uint64_t resp_rx = ts_to_u64(resp_rx_ts_raw);
    uint64_t t_round = (resp_rx - poll_tx) & 0xFFFFFFFFFFULL;
    uint64_t t_reply = (uint64_t)t_reply_ticks;

    if (t_reply == 0U || t_reply >= t_round)
        return RANGE_COMPUTE_ERROR;

    /* Chỉ một EMA cho mỗi anchor, dùng chung cho mọi đường SS. */
    static double ci_ema[TAG_NUM_ANCHORS] = {0.0};
    if (ci_ema[s_current_anchor] == 0.0)
    {
        ci_ema[s_current_anchor] = (double)carrier_integrator;
    }
    else
    {
        ci_ema[s_current_anchor] = ci_ema[s_current_anchor] * 0.95
                                 + (double)carrier_integrator * 0.05;
    }

#if UWB_USE_CLOCK_CORRECTION
    double clock_offset_ratio = ci_ema[s_current_anchor] * UWB_CLOCK_OFFSET_MULT;
#else
    double clock_offset_ratio = 0.0;
#endif

    double t_reply_corrected = (double)t_reply * (1.0 - clock_offset_ratio);
    double diff = (double)t_round - t_reply_corrected;
    if (diff < 0.0)
        return RANGE_COMPUTE_ERROR;

    double tof    = diff / 2.0;
    double dist_m = tof * UWB_DWT_TIME_UNIT_S * UWB_SPEED_OF_LIGHT;

    if (!meters_to_mm_checked(dist_m, uncalibrated_raw_mm))
        return RANGE_COMPUTE_ERROR;

    s_meas.raw_mm = *uncalibrated_raw_mm;
    s_meas.mode = (mode == UWB_MODE_SS_FALLBACK)
        ? TAG_MEAS_MODE_SS_FALLBACK : TAG_MEAS_MODE_SS;
    return apply_offset_and_clamp(dist_m, mode);
}

static uint8_t meters_to_mm_checked(double dist_m, int32_t *out_mm)
{
    if (out_mm == NULL || !isfinite(dist_m) || dist_m < 0.0
        || dist_m > ((double)INT32_MAX / 1000.0))
    {
        return 0U;
    }

    *out_mm = (int32_t)(dist_m * 1000.0);
    return 1U;
}

/**
 * @brief  Apply calibration theo mode bằng switch fail-closed.
 *         SS_FALLBACK luôn dùng profile SS. DS dùng bảng calibration runtime.
 * @retval Distance in mm (>0), RANGE_COMPUTE_ERROR, hoặc
 *         RANGE_CALIBRATION_MISSING.
 */
static int32_t apply_offset_and_clamp(double dist_m, UwbRangingMode_t mode)
{
    int32_t dist_mm = 0;

    if (s_current_anchor >= TAG_NUM_ANCHORS || !isfinite(dist_m) || dist_m < 0.0)
        return RANGE_COMPUTE_ERROR;

    switch (mode)
    {
        case UWB_MODE_DS:
#if UWB_USE_DS_TWR
        {
            double ds_offset_m = 0.0;
            int32_t calibration_status = ds_calibration_offset_for_index(
                s_current_anchor, &ds_offset_m);
            if (calibration_status != 0)
                return calibration_status;

            dist_m -= ds_offset_m;
            /* A bad offset must not turn a positive ToF into a valid zero
             * range, including values that truncate to 0 mm. */
            if (!meters_to_mm_checked(dist_m, &dist_mm) || dist_mm == 0)
                return RANGE_COMPUTE_ERROR;
            return dist_mm;
        }
#else
            /* Fail closed: DS mode không được rơi xuống profile SS khi DS compile OFF. */
            return RANGE_COMPUTE_ERROR;
#endif

        case UWB_MODE_SS:
        case UWB_MODE_SS_FALLBACK:
        {
            const TagAnchorConfig_t *cfg = &s_anchor_cfg[s_current_anchor];
#if UWB_USE_LEGACY_OFFSET
            volatile double *offset = &calibration_offset_m[s_current_anchor];
#else
            volatile double *offset = &residual_offset_m[s_current_anchor];
#endif

            /* Auto-calibration is keyed by physical anchor ID. */
            if (auto_calibrate_actual_m > 0.001
                && auto_calibrate_anchor_id == cfg->id)
            {
                *offset = dist_m - auto_calibrate_actual_m;
                s_ss_calibrated_mask |= cfg->ss_calibration_bit;
                auto_calibrate_actual_m = 0.0;
                auto_calibrate_anchor_id = 0U;
            }

            if ((s_ss_calibrated_mask & cfg->ss_calibration_bit) == 0U)
                return RANGE_CALIBRATION_MISSING;

            dist_m -= *offset;
            if (!meters_to_mm_checked(dist_m, &dist_mm) || dist_mm == 0)
                return RANGE_COMPUTE_ERROR;
            return dist_mm;
        }

        default:
            return RANGE_COMPUTE_ERROR;
    }
}

/**
 * @brief  Common post-distance pipeline: median → outlier gate → Kalman → publish.
 *         Di chuyển code từ WAIT_RESP valid branch, không đổi logic.
 *         Called by SS path (direct) and DS paths (DS ok + DS fallback).
 * @param  dist_mm   Raw distance (mm) from compute_distance_mm or compute_distance_ds_mm
 * @param  diag      Signal diagnostics pointer (for FPP computation)
 * @param  fpp       First-path power in dBm (pre-computed from diag)
 * @param  st_flags  Extra TAG_ST_* flags to OR into mark_anchor_result (e.g. TAG_ST_DS_FALLBACK)
 */
static void publish_distance(int32_t dist_mm, const DW1000_SignalDiag_t *diag,
                             float fpp, uint8_t st_flags)
{
    (void)diag;  /* diag kept for future use / debug; FPP already extracted */
    float fpp_report = fpp;   /* corrected FPP, reported unchanged */
    /* The filters keep the FPP scale they were tuned on (see
     * UWB_FILTER_FPP_COMPAT_DB); only telemetry sees the corrected value. */
    fpp += UWB_FILTER_FPP_COMPAT_DB;
    const float fpp_filter = fpp;
    (void)fpp_filter;
    int32_t filtered_dist_mm;
    uint8_t publish_status = (uint8_t)(TAG_ST_OK | st_flags);

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN
#if UWB_C9_2_MOTION_MODE == UWB_C9_2_MOTION_ACTIVE
    {
        C9_2MotionProcessResult_t motion_result = c9_2_motion_process(
            s_current_anchor, dist_mm, fpp_filter, uwb_platform_time_ms());
        if (motion_result.publish_valid == 0U)
        {
            /* A controlled candidate/reacquire/reject is not a radio loss.
             * Keep the canonical range's age unchanged and expose raw/FPP
             * evidence for diagnostics and calibration. */
            mark_anchor_conditioner_reject(
                s_current_anchor,
                (uint8_t)(st_flags | TAG_ST_RANGE_REJECT),
                dist_mm,
                fpp_report);
            return;
        }
        filtered_dist_mm = motion_result.filtered_mm;
        if (motion_result.reacquired != 0U)
            publish_status = (uint8_t)(publish_status | TAG_ST_FILTER_REACQUIRE);
    }
#else
#if UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_OFF
    /* Apply Median Filter */
    MedianFilter_t median_before = s_mf[s_current_anchor];
    int32_t med_dist_mm = apply_median_filter(&s_mf[s_current_anchor], dist_mm);

    /* Initialize Kalman Filter if needed */
    if (!s_kf_initialized[s_current_anchor]) {
        s_kf[s_current_anchor].x              = (double)med_dist_mm;
        s_kf[s_current_anchor].Q              = 0.05;
        s_kf[s_current_anchor].P              = 1.0;
        s_kf[s_current_anchor].outlier_count  = 0;
        s_kf[s_current_anchor].last_meas_tick = uwb_platform_time_ms();
        s_kf_initialized[s_current_anchor]    = 1;
    }

    /* Apply Outlier Gate */
    uint8_t is_outlier = apply_outlier_gate(&s_kf[s_current_anchor], (double)med_dist_mm);
    if (is_outlier) {
        /* A high Kalman R only attenuates a spike; it still changes the state
         * and the old path published that estimate as valid. Roll back the
         * unaccepted median candidate and keep the last good range instead. */
        s_mf[s_current_anchor] = median_before;
#if UWB_C9_2_MOTION_MODE == UWB_C9_2_MOTION_SHADOW
        (void)c9_2_motion_process(
            s_current_anchor, dist_mm, fpp_filter, uwb_platform_time_ms());
#endif
        mark_anchor_conditioner_reject(
            s_current_anchor,
            (uint8_t)(st_flags | TAG_ST_RANGE_REJECT),
            dist_mm,
            fpp_report);
        return;
    }

    /* Apply Adaptive Kalman Filter */
    double filtered_mm = kalman_update_with_gain(
        &s_kf[s_current_anchor], (double)med_dist_mm, fpp, NULL);
    filtered_dist_mm = (int32_t)filtered_mm;
#elif UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_SHADOW
    /* Production is deliberately the exact deployed Legacy path in SHADOW. */
    MedianFilter_t median_before = s_mf[s_current_anchor];
    int32_t med_dist_mm = apply_median_filter(&s_mf[s_current_anchor], dist_mm);
    if (!s_kf_initialized[s_current_anchor]) {
        s_kf[s_current_anchor].x              = (double)med_dist_mm;
        s_kf[s_current_anchor].Q              = 0.05;
        s_kf[s_current_anchor].P              = 1.0;
        s_kf[s_current_anchor].outlier_count  = 0;
        s_kf[s_current_anchor].last_meas_tick = uwb_platform_time_ms();
        s_kf_initialized[s_current_anchor]    = 1;
    }
    const uint8_t is_outlier = apply_outlier_gate(
        &s_kf[s_current_anchor], (double)med_dist_mm);
    if (is_outlier)
        s_mf[s_current_anchor] = median_before;
    else
        filtered_dist_mm = (int32_t)kalman_update_with_gain(
            &s_kf[s_current_anchor], (double)med_dist_mm, fpp, NULL);

    /* Run candidate state independently after the published path. It has its
     * own median/Kalman memory and cannot mutate s_kf or s_mf. */
    (void)legacy_adaptive_process(
        &s_legacy_adaptive_shadow_kf[s_current_anchor],
        &s_legacy_adaptive_shadow_initialized[s_current_anchor],
        &s_legacy_adaptive_shadow_mf[s_current_anchor],
        &s_legacy_adaptive_state[s_current_anchor],
        dist_mm, fpp_filter, uwb_platform_time_ms());
    if (is_outlier)
    {
#if UWB_C9_2_MOTION_MODE == UWB_C9_2_MOTION_SHADOW
        (void)c9_2_motion_process(
            s_current_anchor, dist_mm, fpp_filter, uwb_platform_time_ms());
#endif
        mark_anchor_conditioner_reject(
            s_current_anchor,
            (uint8_t)(st_flags | TAG_ST_RANGE_REJECT),
            dist_mm,
            fpp_report);
        return;
    }
#else /* UWB_LEGACY_ADAPTIVE_ACTIVE */
    {
        LegacyAdaptiveProcessResult_t adaptive_result = legacy_adaptive_process(
            &s_kf[s_current_anchor],
            &s_kf_initialized[s_current_anchor],
            &s_mf[s_current_anchor],
            &s_legacy_adaptive_state[s_current_anchor],
            dist_mm, fpp_filter, uwb_platform_time_ms());
        if (adaptive_result.publish_valid == 0U)
        {
            /* Preserve actual raw/FPP evidence while holding the last good
             * range. This covers an unconfirmed gate-breaching candidate or a
             * controlled stale reacquire; it is not a radio timeout. */
            mark_anchor_conditioner_reject(
                s_current_anchor,
                (uint8_t)(st_flags | TAG_ST_RANGE_REJECT),
                dist_mm,
                fpp_report);
            return;
        }
        filtered_dist_mm = adaptive_result.filtered_mm;
    }
#endif
#if UWB_C9_2_MOTION_MODE == UWB_C9_2_MOTION_SHADOW
    /* C9.2 has wholly independent median/Kalman/state memory in SHADOW.
     * This call is intentionally after the canonical path and its result is
     * discarded, so a shadow defect cannot affect flight output. */
    (void)c9_2_motion_process(
        s_current_anchor, dist_mm, fpp_filter, uwb_platform_time_ms());
#endif
#endif /* UWB_C9_2_MOTION_MODE == ACTIVE */
#else
    RangeFilterInput_t filter_input = {
        .corrected_raw_mm = dist_mm,
        .fpp_dbm = fpp,
        .now_ms = uwb_platform_time_ms(),
        .radio_status = st_flags
    };
    RangeFilterOutput_t filter_output = RangeFilter_Update(
        &range_filter_state[s_current_anchor],
        &filter_input,
        &g_range_filter_config);
    uint32_t abs_innovation = (uint32_t)(
        filter_output.innovation_mm < 0.0f
            ? -filter_output.innovation_mm
            : filter_output.innovation_mm);

    range_filter_last_decision[s_current_anchor] = (uint8_t)filter_output.decision;
    range_filter_last_innovation_mm[s_current_anchor] =
        (int32_t)filter_output.innovation_mm;
    if (abs_innovation > range_filter_max_abs_innovation_mm[s_current_anchor])
        range_filter_max_abs_innovation_mm[s_current_anchor] = abs_innovation;

    filtered_dist_mm = filter_output.filtered_mm;
    if (filter_output.publish_valid == 0U)
    {
        /* Preserve corrected raw/FPP as diagnostic evidence while keeping the
         * canonical measurement invalid and last-success age unchanged. */
        mark_anchor_conditioner_reject(
            s_current_anchor,
            (uint8_t)(st_flags | TAG_ST_RANGE_REJECT),
            dist_mm,
            fpp_report);
        return;
    }
    if (filter_output.decision == RANGE_FILTER_REACQUIRED)
        publish_status = (uint8_t)(publish_status | TAG_ST_FILTER_REACQUIRE);
#endif

    distance_raw_mm[s_current_anchor] = dist_mm;
    distance_filtered_mm[s_current_anchor] = filtered_dist_mm;

    /* Legacy debugger mirrors; the arrays above are the source of truth. */
    switch (anchor_id_at(s_current_anchor))
    {
        case 0x0001U:
            distance_a1_mm = dist_mm;
            distance_a1_filtered_mm = filtered_dist_mm;
            break;
        case 0x0002U:
            distance_a2_mm = dist_mm;
            distance_a2_filtered_mm = filtered_dist_mm;
            break;
        case 0x0003U:
            distance_a3_mm = dist_mm;
            distance_a3_filtered_mm = filtered_dist_mm;
            break;
        case 0x0004U:
            distance_a4_mm = dist_mm;
            distance_a4_filtered_mm = filtered_dist_mm;
            break;
        case 0x0005U:
            distance_a5_mm = dist_mm;
            distance_a5_filtered_mm = filtered_dist_mm;
            break;
        case 0x0006U:
            distance_a6_mm = dist_mm;
            distance_a6_filtered_mm = filtered_dist_mm;
            break;
        case 0x0007U:
            distance_a7_mm = dist_mm;
            distance_a7_filtered_mm = filtered_dist_mm;
            break;
        case 0x0008U:
            distance_a8_mm = dist_mm;
            distance_a8_filtered_mm = filtered_dist_mm;
            break;
        default:
            break;
    }

    s_result.ranging_count++;
    s_result.distance_mm = dist_mm;
    response_ok_count++;
    anchor_success_count[s_current_anchor]++;
    mark_anchor_result(s_current_anchor, 1, publish_status,
                       dist_mm, filtered_dist_mm, fpp_report);
}


/* Forward declaration — the DS helpers call this before its definition. */
static void advance_to_next_anchor(void);

static void note_anchor_ds_incomplete(uint8_t idx);
static void note_anchor_ds_complete(uint8_t idx);

#if UWB_USE_DS_TWR
/**
 * @brief  DS-TWR distance calculation using the asymmetric 4-term formula
 *         tof = (Ra*Rb - Da*Db) / (Ra+Rb+Da+Db).
 * @param  rb_ticks             Rb value from REPORT frame
 * @param  uncalibrated_raw_mm  DS range before the software offset
 * @retval Distance in mm (>=0), RANGE_COMPUTE_ERROR or RANGE_CALIBRATION_MISSING
 */
static int32_t compute_distance_ds_mm(uint32_t rb_ticks,
                                      int32_t *uncalibrated_raw_mm)
{
    if (uncalibrated_raw_mm == NULL)
        return RANGE_COMPUTE_ERROR;

    *uncalibrated_raw_mm = 0;

    uint64_t t1 = ts_to_u64(s_poll_tx_ts);
    uint64_t t4 = ts_to_u64(s_resp_rx_ts);
    uint64_t t5 = ts_to_u64(s_final_tx_ts);

    double Ra = (double)((t4 - t1) & 0xFFFFFFFFFFULL);
    double Db = (double)((t5 - t4) & 0xFFFFFFFFFFULL);
    double Da = (double)s_da_from_resp;
    double Rb = (double)rb_ticks;

    double denom = Ra + Rb + Da + Db;
    if (denom <= 0.0)
        return RANGE_COMPUTE_ERROR;
    double tof = (Ra * Rb - Da * Db) / denom;
    if (tof <= 0.0)
        return RANGE_COMPUTE_ERROR;

    double dist_m = tof * UWB_DWT_TIME_UNIT_S * UWB_SPEED_OF_LIGHT;

    if (!meters_to_mm_checked(dist_m, uncalibrated_raw_mm))
        return RANGE_COMPUTE_ERROR;

    s_meas.raw_mm = *uncalibrated_raw_mm;
    s_meas.mode = TAG_MEAS_MODE_DS;
    /* C0.1: raw đã được capture; production output vẫn bị guard theo từng anchor. */
    return apply_offset_and_clamp(dist_m, UWB_MODE_DS);
}

/**
 * @brief  Build and send the FINAL frame (immediate TX, WAIT4RESP for REPORT).
 */
static void build_and_send_final(uint16_t target_anchor)
{
    uint8_t frame[UWB_FRAME_HEADER_LEN + 2U];
    UwbFinal_t fin = { .version = UWB_TAG_FRAME_VERSION, .txn = s_txn };
    const uint16_t len = uwb_frame_build_final(frame, s_seq_num++, DW_PAN_ID,
                                               target_anchor, TAG_ADDR, &fin);

    DW1000_ClearAllStatus();
    DW1000_WriteTxData(frame, len);
    DW1000_SetTxFrameCtrl((uint16_t)(len + UWB_FRAME_FCS_LEN));
    start_tx_expecting_reply();
}

/**
 * @brief  SS fallback using shared helper — ensures same formula as SS thuần
 *         including carrier integrator correction (FIX-04 §8.6).
 *         Called when FINAL TX stuck or REPORT lost/timeout.
 */
static void finish_with_ss_fallback(void)
{
    ds_fallback_count++;
    anchor_ds_fallback_count[s_current_anchor]++;
    note_anchor_ds_incomplete(s_current_anchor);
    int32_t ss_uncalibrated_raw_mm = 0;

    /* C0: SS_FALLBACK phải resolve về profile SS, không dùng DS. */
    int32_t dist_mm = compute_distance_ss_terms_mm(
        s_resp_rx_ts,
        s_da_from_resp,
        s_resp_ci,
        UWB_MODE_SS_FALLBACK,
        &ss_uncalibrated_raw_mm);

    if (dist_mm == RANGE_CALIBRATION_MISSING)
    {
        mark_anchor_rejected_measurement(
            s_current_anchor,
            (uint8_t)(TAG_ST_DS_FALLBACK | TAG_ST_CALIBRATION_MISSING),
            ss_uncalibrated_raw_mm,
            s_resp_fpp);
        advance_to_next_anchor();
        return;
    }
    else if (dist_mm < 0)
    {
        mark_anchor_result(s_current_anchor, 0, TAG_ST_COMPUTE, 0, 0, 0.0f);
        advance_to_next_anchor();
        return;
    }

    publish_distance(dist_mm, &s_resp_diag, s_resp_fpp, TAG_ST_DS_FALLBACK);
    advance_to_next_anchor();
}
#endif /* UWB_USE_DS_TWR */

/* ========================================================================== */
/*                     SCHEDULING                                              */
/* ========================================================================== */

static uint8_t cycle_deadline_reached(uint32_t now_cycle, uint32_t target_cycle)
{
    return ((int32_t)(now_cycle - target_cycle) >= 0) ? 1U : 0U;
}

static void schedule_next_probe(uint8_t idx)
{
    s_anchor_next_probe_cycle[idx] = tag_cycle_count + TAG_OFFLINE_PROBE_INTERVAL_CYCLES;
}

/** An anchor that keeps missing RESP, or keeps failing DS, is backed off. */
static uint8_t anchor_is_unhealthy(uint8_t idx)
{
    return (anchor_response_timeout_streak[idx] >= TAG_OFFLINE_AFTER_TIMEOUTS
            || anchor_ds_incomplete_streak[idx] >= TAG_OFFLINE_AFTER_TIMEOUTS)
        ? 1U : 0U;
}

static void note_anchor_response_timeout(uint8_t idx)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    if (anchor_response_timeout_streak[idx] < 0xFFFFU)
        anchor_response_timeout_streak[idx]++;

    if (anchor_response_timeout_streak[idx] >= TAG_OFFLINE_AFTER_TIMEOUTS)
        schedule_next_probe(idx);
}

static void note_anchor_response_received(uint8_t idx)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    anchor_response_timeout_streak[idx] = 0U;
}

static void note_anchor_ds_incomplete(uint8_t idx)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    if (anchor_ds_incomplete_streak[idx] < 0xFFFFU)
        anchor_ds_incomplete_streak[idx]++;

    if (anchor_ds_incomplete_streak[idx] >= TAG_OFFLINE_AFTER_TIMEOUTS)
        schedule_next_probe(idx);
}

static void note_anchor_ds_complete(uint8_t idx)
{
    if (idx >= TAG_NUM_ANCHORS)
        return;

    anchor_ds_incomplete_streak[idx] = 0U;
    anchor_ds_ok_count[idx]++;
}

/**
 * F1: choose at most one unhealthy anchor to probe in this cycle, round-robin
 * from the anchor after the last one probed, among those whose backoff
 * expired. Every offline anchor therefore gets a probe within
 * TAG_NUM_ANCHORS cycles of becoming due, whatever its position.
 */
static void choose_cycle_probe(void)
{
    s_cycle_probe_idx = -1;

    for (uint8_t k = 0U; k < TAG_NUM_ANCHORS; k++)
    {
        const uint8_t idx = (uint8_t)((s_probe_rr + k) % TAG_NUM_ANCHORS);

        if (!anchor_active(idx) || !anchor_is_unhealthy(idx))
            continue;
        if (!cycle_deadline_reached(tag_cycle_count, s_anchor_next_probe_cycle[idx]))
            continue;

        s_cycle_probe_idx = (int8_t)idx;
        s_probe_rr = (uint8_t)((idx + 1U) % TAG_NUM_ANCHORS);
        /* Reserve the next deadline now: a malformed reply or a TAG-side TX
         * fault must not let this anchor take the probe again next cycle. */
        schedule_next_probe(idx);
        return;
    }
}

/** Ask one active anchor per second for its info TLVs, round-robin. */
static void choose_cycle_info_request(void)
{
    s_info_request_idx = -1;
    if ((tag_cycle_count % TAG_INFO_REQUEST_PERIOD_CYCLES) != 0U)
        return;

    for (uint8_t k = 0U; k < TAG_NUM_ANCHORS; k++)
    {
        const uint8_t idx = (uint8_t)((s_info_rr + k) % TAG_NUM_ANCHORS);
        if (anchor_active(idx) && !anchor_is_unhealthy(idx))
        {
            s_info_request_idx = (int8_t)idx;
            s_info_rr = (uint8_t)((idx + 1U) % TAG_NUM_ANCHORS);
            return;
        }
    }
}

/**
 * Healthy active anchors are always polled. An unhealthy one only when it
 * is this cycle's probe.
 */
static uint8_t anchor_should_poll(uint8_t idx)
{
    if (!anchor_active(idx))
        return 0U;
    if (!anchor_is_unhealthy(idx))
        return 1U;
    return ((int8_t)idx == s_cycle_probe_idx) ? 1U : 0U;
}

static uint8_t select_poll_anchor(uint8_t first_idx)
{
    for (uint8_t idx = first_idx; idx < TAG_NUM_ANCHORS; idx++)
    {
        if (anchor_should_poll(idx))
        {
            s_current_anchor = idx;
            if ((int8_t)idx == s_cycle_probe_idx)
                anchor_probe_count[idx]++;
            return 1U;
        }

        /* Keep telemetry truthful: this cycle has no new measurement for the
         * skipped anchor. Backed-off skips have their own counter and are not
         * counted as UWB timeouts; disabled anchors are not counted at all. */
        if (anchor_active(idx))
            anchor_poll_skipped_count[idx]++;
        mark_anchor_result(idx, 0U, TAG_ST_TIMEOUT, 0, 0, 0.0f);
    }

    return 0U;
}

static void meas_push(const TagMeasurement_t *m)
{
    if (!s_meas_queue_enabled)
        return;
    if (s_meas_count >= TAG_MEAS_QUEUE_LEN)
    {
        tag_meas_queue_drops++;
        return;
    }
    s_meas_queue[s_meas_head] = *m;
    s_meas_head = (uint8_t)((s_meas_head + 1U) % TAG_MEAS_QUEUE_LEN);
    s_meas_count++;
}

static void discard_pending_measurements(void)
{
    /* Queued records and a pending cycle snapshot describe the previous
     * calibration/radio state and must not be emitted after it changes. */
    s_meas_head = 0U;
    s_meas_tail = 0U;
    s_meas_count = 0U;
    tag_cycle_ready = 0U;
}

static void finish_full_cycle(void)
{
    uint32_t dur_us = MCU_ElapsedUs(s_full_cycle_start);
    tag_cycle_duration_us = dur_us;
    if (dur_us > tag_cycle_duration_max_us)
        tag_cycle_duration_max_us = dur_us;

    if (dur_us > ((uint32_t)TAG_CYCLE_MS * 1000U))
        cycle_overrun_count++;

    tag_sample_seq++;
    tag_cycle_ready = 1;
    s_cycle_end_cycle = MCU_CycleNow();
    s_info_request_idx = -1;
    s_state = TAG_STATE_IDLE;
}

/**
 * @brief  Close the current slot and move to the next due anchor, or finish
 *         the full telemetry cycle.
 */
static void advance_to_next_anchor(void)
{
    /* Record the whole slot, including TX, response wait and DS exchange when
     * enabled. This pinpoints the slot that consumes the cycle budget. */
    uint32_t slot_us = MCU_ElapsedUs(s_slot_start);
    anchor_slot_duration_us[s_current_anchor] = slot_us;
    if (slot_us > anchor_slot_duration_max_us[s_current_anchor])
        anchor_slot_duration_max_us[s_current_anchor] = slot_us;

    if ((s_meas.flags & TAG_MEAS_FLAG_RADIO_OK) != 0U)
    {
        const uint64_t now_us = uwb_platform_time_us64();
        s_meas.slot_us = (uint16_t)(slot_us > 0xFFFFU ? 0xFFFFU : slot_us);
        s_meas.meas_time_us = now_us - (uint64_t)(slot_us / 2U);
        s_meas.meas_seq = s_meas_seq++;
        meas_push(&s_meas);
    }

    if (select_poll_anchor((uint8_t)(s_current_anchor + 1U)))
    {
        DW1000_ForceRxOff();
        DW1000_ClearAllStatus();
        dw1000_irq_flag = 0;
        s_state       = TAG_STATE_INTER_ANCHOR_GUARD;
        s_state_cycle = MCU_CycleNow();
    }
    else
    {
        DW1000_ForceRxOff();
        DW1000_ClearAllStatus();
        dw1000_irq_flag = 0;
        finish_full_cycle();
    }
}

/* ========================================================================== */
/*                     RESP / REPORT HANDLING                                  */
/* ========================================================================== */

typedef enum {
    FRAME_KEEP_WAITING = 0,   /* stray/stale frame: RX re-armed, deadline kept */
    FRAME_ADVANCE      = 1,   /* slot finished (result already recorded) */
    FRAME_CONTINUE_DS  = 2,   /* FINAL sent, state changed to TX_FINAL */
} FrameOutcome_t;

static FrameOutcome_t handle_resp_frame(void)
{
    uint8_t  rx_buf[MAX_RX_FRAME_LEN];
    const uint16_t rx_len = DW1000_ReadRxData(rx_buf, MAX_RX_FRAME_LEN);
    UwbFrameHeader_t hdr;
    UwbResp_t resp;

    /* Foreign traffic, another anchor's RESP or a late REPORT: ignore it and
     * keep the original deadline (FIX-03). */
    if (!uwb_frame_parse_header(rx_buf, rx_len, DW_PAN_ID, &hdr)
        || hdr.func != FRAME_RESP_FUNC
        || hdr.src != anchor_id_at(s_current_anchor))
    {
        restart_rx_keep_deadline();
        return FRAME_KEEP_WAITING;
    }

    /* Right anchor but unusable frame. */
    if (hdr.dst != TAG_ADDR || !uwb_frame_parse_resp(rx_buf, rx_len, &resp)
        || resp.version != UWB_TAG_FRAME_VERSION)
    {
        DW1000_ClearAllStatus();
        s_result.timeout_count++;
        rx_error_count++;
        anchor_rx_error_count[s_current_anchor]++;
        mark_anchor_result(s_current_anchor, 0, TAG_ST_BADFRAME, 0, 0, 0.0f);
        return FRAME_ADVANCE;
    }

#if UWB_TAG_FRAME_VERSION >= 2
    /* A RESP of an older transaction (e.g. retransmitted late) is stale. */
    if (resp.txn != s_txn)
    {
        anchor_txn_mismatch_count[s_current_anchor]++;
        restart_rx_keep_deadline();
        return FRAME_KEEP_WAITING;
    }
#endif

    note_anchor_response_received(s_current_anchor);
    if (resp.tlv_len != 0U)
        capture_anchor_info(s_current_anchor, &resp);

    uint8_t resp_rx_ts[5];
    DW1000_ReadRxTimestamp(resp_rx_ts);

#if UWB_USE_DS_TWR
    /* DS path: save T4 + Da + diag BEFORE sending FINAL (the receive
     * diagnostics are overwritten by the REPORT reception). */
    memcpy(s_resp_rx_ts, resp_rx_ts, 5);
    s_da_from_resp = resp.reply_ticks;
    DW1000_ReadSignalDiag(&s_resp_diag);
    s_resp_fpp = DW1000_GetFirstPathPower(&s_resp_diag);
    s_resp_ci  = DW1000_ReadCarrierIntegrator();
    capture_resp_diagnostics(&s_resp_diag, s_resp_fpp, s_resp_ci);

    build_and_send_final(anchor_id_at(s_current_anchor));
    s_state       = TAG_STATE_TX_FINAL;
    s_state_cycle = MCU_CycleNow();
    return FRAME_CONTINUE_DS;
#else
    /* SS path: compute distance now and publish. */
    DW1000_SignalDiag_t diag;
    DW1000_ReadSignalDiag(&diag);
    const float fpp = DW1000_GetFirstPathPower(&diag);
    const int32_t ci = DW1000_ReadCarrierIntegrator();
    capture_resp_diagnostics(&diag, fpp, ci);
    DW1000_ClearAllStatus();

    int32_t ss_uncalibrated_raw_mm = 0;
    const int32_t dist_mm = compute_distance_ss_terms_mm(
        resp_rx_ts, resp.reply_ticks, ci, UWB_MODE_SS, &ss_uncalibrated_raw_mm);
    if (dist_mm >= 0)
    {
        publish_distance(dist_mm, &diag, fpp, TAG_ST_OK);
    }
    else if (dist_mm == RANGE_CALIBRATION_MISSING)
    {
        mark_anchor_rejected_measurement(s_current_anchor,
                                         TAG_ST_CALIBRATION_MISSING,
                                         ss_uncalibrated_raw_mm, fpp);
    }
    else
    {
        mark_anchor_result(s_current_anchor, 0, TAG_ST_COMPUTE, 0, 0, 0.0f);
    }
    return FRAME_ADVANCE;
#endif
}

#if UWB_USE_DS_TWR
static FrameOutcome_t handle_report_frame(void)
{
    uint8_t  rx_buf[MAX_RX_FRAME_LEN];
    const uint16_t rx_len = DW1000_ReadRxData(rx_buf, MAX_RX_FRAME_LEN);
    UwbFrameHeader_t hdr;
    UwbReport_t rep;

    if (!uwb_frame_parse_header(rx_buf, rx_len, DW_PAN_ID, &hdr)
        || hdr.func != FRAME_REPORT_FUNC
        || hdr.dst != TAG_ADDR
        || hdr.src != anchor_id_at(s_current_anchor)
        || !uwb_frame_parse_report(rx_buf, rx_len, &rep)
        || rep.version != UWB_TAG_FRAME_VERSION)
    {
        /* Stray frame — keep listening, DO NOT reset deadline (FIX-03). */
        restart_rx_keep_deadline();
        return FRAME_KEEP_WAITING;
    }

#if UWB_TAG_FRAME_VERSION >= 2
    if (rep.txn != s_txn)
    {
        anchor_txn_mismatch_count[s_current_anchor]++;
        restart_rx_keep_deadline();
        return FRAME_KEEP_WAITING;
    }
    s_meas.anchor_fp_cdbm = rep.final_fp_cdbm;
    s_meas.anchor_rx_cdbm = rep.final_rx_cdbm;
    if (rep.final_fp_cdbm != INT16_MIN)
        s_meas.flags |= TAG_MEAS_FLAG_ANCHOR_DIAG;
#endif

    /* Valid REPORT — extract Rb, compute DS distance */
    ds_report_ok_count++;
    DW1000_ClearAllStatus();
    note_anchor_ds_complete(s_current_anchor);

    int32_t ds_uncalibrated_raw_mm = 0;
    const int32_t dist_mm = compute_distance_ds_mm(rep.round_ticks,
                                                   &ds_uncalibrated_raw_mm);
    if (dist_mm >= 0)
    {
        ds_ok_count++;
        publish_distance(dist_mm, &s_resp_diag, s_resp_fpp, TAG_ST_OK);
    }
    else if (dist_mm == RANGE_CALIBRATION_MISSING)
    {
        /* Giữ raw/FPP để calibration nhưng production valid=0. */
        ds_cal_missing_count++;
        mark_anchor_rejected_measurement(s_current_anchor, TAG_ST_CAL_MISSING_DS,
                                         ds_uncalibrated_raw_mm, s_resp_fpp);
    }
    else /* RANGE_COMPUTE_ERROR */
    {
        mark_anchor_result(s_current_anchor, 0, TAG_ST_COMPUTE, 0, 0, 0.0f);
    }
    return FRAME_ADVANCE;
}
#endif

/* ========================================================================== */
/*                     PUBLIC API                                              */
/* ========================================================================== */

static void reset_anchor_filter(uint8_t idx)
{
#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN
    s_kf_initialized[idx] = 0U;
    memset(&s_mf[idx], 0, sizeof(s_mf[idx]));
#if UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_OFF
    LegacyAdaptiveTracking_Init(&s_legacy_adaptive_state[idx]);
#if UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_SHADOW
    s_legacy_adaptive_shadow_initialized[idx] = 0U;
    memset(&s_legacy_adaptive_shadow_mf[idx], 0, sizeof(s_legacy_adaptive_shadow_mf[idx]));
#endif
#endif
#if UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_OFF
    s_c9_2_kf_initialized[idx] = 0U;
    memset(&s_c9_2_mf[idx], 0, sizeof(s_c9_2_mf[idx]));
    MotionAdaptiveRange_Init(&s_c9_2_motion[idx]);
#endif
#else
    RangeFilter_Init(&range_filter_state[idx]);
#endif
}

static int tag_radio_init(void)
{
    if (DW1000_Init() != 0)
        return -1;

    if (DW1000_Configure() != 0)        /* Also writes SYS_MASK */
        return -2;
    DW1000_SetAddress(DW_PAN_ID, TAG_ADDR);
    if (DW1000_EnableFastSPI() == 0U)
        return -3;
    if (DW1000_VerifyConfig() != 0U)
        return -4;
    DW1000_ClearAllStatus();           /* Ensure IRQ pin LOW before EnableIRQ */
    return 0;
}

int Tag_Init(void)
{
    const int rc = tag_radio_init();
    if (rc != 0)
        return rc;

    memset(&s_result, 0, sizeof(s_result));
    memset(s_track, 0, sizeof(s_track));
    memset(s_anchor_info, 0, sizeof(s_anchor_info));
    s_anchor_info_pending = 0U;
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        legacy_adaptive_state[i] = TAG_LEGACY_ADAPTIVE_STABLE;
        legacy_adaptive_candidate_count[i] = 0U;
        legacy_adaptive_candidate_direction[i] = 0;
        legacy_adaptive_track_enter_count[i] = 0U;
        legacy_adaptive_track_exit_count[i] = 0U;
        legacy_adaptive_true_reject_count[i] = 0U;
        legacy_adaptive_stale_reacquire_count[i] = 0U;
        legacy_adaptive_track_duration_ms[i] = 0U;
        legacy_adaptive_last_innovation_mm[i] = 0;
        legacy_adaptive_max_abs_innovation_mm[i] = 0U;
        legacy_adaptive_last_q[i] = 0.0;
        legacy_adaptive_last_r[i] = 0.0;
        legacy_adaptive_last_gain[i] = 0.0;
        c9_2_motion_state[i] = (uint8_t)MOTION_RANGE_REACQUIRE;
        c9_2_static_enter_count[i] = 0U;
        c9_2_slow_enter_count[i] = 0U;
        c9_2_fast_enter_count[i] = 0U;
        c9_2_settling_enter_count[i] = 0U;
        c9_2_degraded_enter_count[i] = 0U;
        c9_2_stale_reacquire_count[i] = 0U;
        c9_2_true_reject_count[i] = 0U;
        c9_2_last_slope_mm_s[i] = 0;
        c9_2_last_motion_score_milli[i] = 0U;
        c9_2_last_gain[i] = 0.0;
    }
    c9_2_global_motion_state = (uint8_t)MOTION_RANGE_REACQUIRE;
#if UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
        RangeFilter_Init(&range_filter_state[i]);
#endif
#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN && \
    UWB_LEGACY_ADAPTIVE_MODE != UWB_LEGACY_ADAPTIVE_OFF
    memset(s_legacy_adaptive_state, 0, sizeof(s_legacy_adaptive_state));
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
        LegacyAdaptiveTracking_Init(&s_legacy_adaptive_state[i]);
#if UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_SHADOW
    memset(s_legacy_adaptive_shadow_kf, 0, sizeof(s_legacy_adaptive_shadow_kf));
    memset(s_legacy_adaptive_shadow_initialized, 0,
           sizeof(s_legacy_adaptive_shadow_initialized));
    memset(s_legacy_adaptive_shadow_mf, 0, sizeof(s_legacy_adaptive_shadow_mf));
#endif
#endif
#if UWB_C9_2_MOTION_MODE != UWB_C9_2_MOTION_OFF
    memset(s_c9_2_kf, 0, sizeof(s_c9_2_kf));
    memset(s_c9_2_kf_initialized, 0, sizeof(s_c9_2_kf_initialized));
    memset(s_c9_2_mf, 0, sizeof(s_c9_2_mf));
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
        MotionAdaptiveRange_Init(&s_c9_2_motion[i]);
#endif

    s_state           = TAG_STATE_IDLE;
    s_cycle_tick      = uwb_platform_time_ms();
    s_cycle_end_cycle = MCU_CycleNow();
    return 0;
}

int Tag_RecoverRadio(void)
{
    const int rc = tag_radio_init();

    /* A failed SPI transfer may have occurred after a sample entered the
     * telemetry queue. Drop it and reset temporal state before ranging again. */
    discard_pending_measurements();
    memset(&s_meas, 0, sizeof(s_meas));
    s_anchor_info_pending = 0U;
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        reset_anchor_filter(i);
        s_track[i].valid = 0U;
        s_track[i].status = TAG_ST_RXERR;
        s_track[i].raw_mm = 0;
        s_track[i].fpp_cdbm = 0;
    }
    dw1000_irq_flag   = 0U;
    s_state           = TAG_STATE_IDLE;
    s_cycle_tick      = uwb_platform_time_ms();
    s_cycle_end_cycle = MCU_CycleNow();
    return rc;
}

uint8_t Tag_IsIdle(void)
{
    return (s_state == TAG_STATE_IDLE) ? 1U : 0U;
}

void Tag_RequestPause(uint8_t pause)
{
    s_pause_requested = pause ? 1U : 0U;
    if (!pause)
        s_paused = 0U;
}

uint8_t Tag_IsPaused(void)
{
    return (s_paused && s_state == TAG_STATE_IDLE) ? 1U : 0U;
}

int Tag_SetDsCalibration(uint16_t anchor_id, int32_t bias_um, uint8_t calibrated)
{
    const int idx = anchor_index_of(anchor_id);

    if (idx < 0)
        return -1;
    if (bias_um > TAG_DS_BIAS_LIMIT_UM || bias_um < -TAG_DS_BIAS_LIMIT_UM)
        return -2;

    if (s_ds_cal[idx].bias_um != bias_um
        || s_ds_cal[idx].calibrated != (calibrated ? 1U : 0U))
    {
        discard_pending_measurements();
        s_ds_cal[idx].bias_um = bias_um;
        s_ds_cal[idx].calibrated = calibrated ? 1U : 0U;
        /* The filter state lives in corrected units: restart it. */
        reset_anchor_filter((uint8_t)idx);
        /* The last published range was computed using the old calibration. */
        s_track[idx].valid = 0U;
        s_track[idx].raw_mm = 0;
        s_track[idx].fpp_cdbm = 0;
        if (!s_ds_cal[idx].calibrated)
            s_track[idx].status = TAG_ST_CALIBRATION_MISSING;
    }
    return 0;
}

int Tag_GetDsCalibration(uint16_t anchor_id, int32_t *bias_um, uint8_t *calibrated)
{
    const int idx = anchor_index_of(anchor_id);

    if (idx < 0)
        return -1;
    if (bias_um != NULL)
        *bias_um = s_ds_cal[idx].bias_um;
    if (calibrated != NULL)
        *calibrated = s_ds_cal[idx].calibrated;
    return 0;
}

void Tag_InvalidateDsCalibration(void)
{
    /* This also runs after an RF profile change, which invalidates every
     * queued range even if no DS calibration bit was set. */
    discard_pending_measurements();
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        if (s_ds_cal[i].calibrated != 0U)
        {
            s_ds_cal[i].calibrated = 0U;
            reset_anchor_filter(i);
            s_track[i].valid = 0U;
            s_track[i].status = TAG_ST_CALIBRATION_MISSING;
            s_track[i].raw_mm = 0;
            s_track[i].fpp_cdbm = 0;
        }
    }
}

void Tag_RestoreDefaultConfiguration(void)
{
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        (void)Tag_SetDsCalibration(anchor_id_at(i), k_ds_cal_default[i].bias_um,
                                   k_ds_cal_default[i].calibrated);
    }
    Tag_SetActiveAnchorMask((uint8_t)(UWB_TAG_ACTIVE_ANCHOR_MASK & 0xFFU));
}

uint8_t Tag_DsCalibratedMask(void)
{
    uint8_t mask = 0U;

    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        if (s_ds_cal[i].calibrated)
            mask |= (uint8_t)s_anchor_cfg[i].ds_calibration_bit;
    }
    return mask;
}

void Tag_SetActiveAnchorMask(uint8_t mask)
{
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        const uint8_t bit = (uint8_t)(1U << i);
        if ((s_active_anchor_mask & bit) != 0U && (mask & bit) == 0U)
        {
            s_track[i].valid = 0U;
            s_track[i].status = TAG_ST_TIMEOUT;
        }
    }
    s_active_anchor_mask = mask;
}

uint8_t Tag_GetActiveAnchorMask(void)
{
    return s_active_anchor_mask;
}

void Tag_EnableMeasurementQueue(uint8_t enable)
{
    s_meas_queue_enabled = enable ? 1U : 0U;
    s_meas_head = 0U;
    s_meas_tail = 0U;
    s_meas_count = 0U;
}

uint8_t Tag_PopMeasurement(TagMeasurement_t *out)
{
    if (out == NULL || s_meas_count == 0U)
        return 0U;

    *out = s_meas_queue[s_meas_tail];
    s_meas_tail = (uint8_t)((s_meas_tail + 1U) % TAG_MEAS_QUEUE_LEN);
    s_meas_count--;
    return 1U;
}

uint8_t Tag_TakeAnchorInfo(TagAnchorInfo_t *out)
{
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        const uint8_t bit = (uint8_t)(1U << i);
        if ((s_anchor_info_pending & bit) != 0U)
        {
            s_anchor_info_pending &= (uint8_t)~bit;
            if (out != NULL)
                *out = s_anchor_info[i];
            return 1U;
        }
    }
    return 0U;
}

void Tag_Task(void)
{
    uint32_t now = uwb_platform_time_ms();

    switch (s_state)
    {
        /* ================================================================== */
        case TAG_STATE_IDLE:
        /* ================================================================== */
        {
            if (s_pause_requested)
            {
                s_paused = 1U;
                break;
            }
            s_paused = 0U;

            /* Keep the nominal 20ms schedule while healthy. After an overrun,
             * also guarantee a real quiet interval before A1 is polled again. */
            if ((now - s_cycle_tick) < TAG_CYCLE_MS)
                break;
            if (MCU_ElapsedUs(s_cycle_end_cycle) < TAG_INTER_CYCLE_RECOVERY_US)
                break;

            /* Start a new cycle */
            tag_cycle_count++;
            s_cycle_tick     = now;
            s_full_cycle_start = MCU_CycleNow();
            choose_cycle_probe();
            choose_cycle_info_request();

            /* A cycle can contain only healthy anchors or one periodic offline
             * probe. If no anchor is due, still publish a truthful age/status
             * snapshot without touching the radio. */
            if (!select_poll_anchor(0U))
            {
                finish_full_cycle();
                break;
            }

            begin_slot();
            s_slot_start = s_full_cycle_start;
            send_poll(anchor_id_at(s_current_anchor));
            s_state      = TAG_STATE_TX_POLL;
            s_state_cycle = MCU_CycleNow();
            break;
        }

        /* ================================================================== */
        case TAG_STATE_TX_POLL:
        /* ================================================================== */
        {
            /* Guard: TX must complete within TAG_TX_TIMEOUT_US (FIX-02: wrap-safe) */
            uint32_t tx_elapsed_us = MCU_ElapsedUs(s_state_cycle);
            if (tx_elapsed_us > TAG_TX_TIMEOUT_US && !dw1000_irq_flag)
            {
                /* TX stuck — skip this anchor */
                if (tx_elapsed_us > anchor_poll_tx_duration_max_us[s_current_anchor])
                    anchor_poll_tx_duration_max_us[s_current_anchor] = tx_elapsed_us;
                s_result.timeout_count++;
                anchor_timeout_count[s_current_anchor]++;
                anchor_poll_tx_timeout_count[s_current_anchor]++;
                rx_timeout_count++;
                mark_anchor_result(s_current_anchor, 0U, TAG_ST_TIMEOUT, 0, 0, 0.0f);
                advance_to_next_anchor();
                break;
            }

            /* Wait for TXFRS IRQ */
            if (!dw1000_irq_flag)
                break;

            dw1000_irq_flag = 0;
            uint64_t status = DW1000_ReadStatus();

            if (status & DW_TXFRS_BIT)
            {
                /* POLL sent — capture TX timestamp; RX is (or is now) enabled. */
                tx_elapsed_us = MCU_ElapsedUs(s_state_cycle);
                if (tx_elapsed_us > anchor_poll_tx_duration_max_us[s_current_anchor])
                    anchor_poll_tx_duration_max_us[s_current_anchor] = tx_elapsed_us;
                DW1000_ReadTxTimestamp(s_poll_tx_ts);
                arm_rx_after_tx();
                s_state      = TAG_STATE_WAIT_RESP;
                s_state_cycle = MCU_CycleNow();
            }
            else
            {
                /* Release an unrelated level-held IRQ so the TX deadline can fire. */
                DW1000_ClearAllStatus();
            }
            break;
        }

        /* ================================================================== */
        case TAG_STATE_WAIT_RESP:
        /* ================================================================== */
        {
            uint8_t advance = 0;
            uint8_t processed_irq = 0U;
            McuCycleStamp_t processing_start = 0;

            /* FIX-03: handle a pending IRQ before looking at the timeout. */
            if (dw1000_irq_flag)
            {
                uint32_t wait_us = MCU_ElapsedUs(s_state_cycle);
                if (wait_us > anchor_response_wait_max_us[s_current_anchor])
                    anchor_response_wait_max_us[s_current_anchor] = wait_us;
                processing_start = MCU_CycleNow();
                processed_irq = 1U;

                dw1000_irq_flag = 0;
                const uint64_t status = DW1000_ReadStatus();
                const DW1000_RxEvent_t event = DW1000_ClassifyRx(status);

                if (event == DW_RX_EVENT_ERROR)
                {
                    /* F3/N3: never use a frame that shares its status with an
                     * error flag. Reset RX and keep listening until deadline. */
                    rx_error_count++;
                    anchor_rx_error_count[s_current_anchor]++;
                    recover_rx_after_error(status);
                }
                else if (event == DW_RX_EVENT_NONE)
                {
                    /* Level-held TX event or spurious edge: release the line. */
                    DW1000_ClearTxStatus();
                }
                else
                {
                    const FrameOutcome_t outcome = handle_resp_frame();
                    if (outcome == FRAME_ADVANCE)
                        advance = 1;
                }
            }

            /* Timeout — only while still waiting for this RESP. */
            if (!advance && s_state == TAG_STATE_WAIT_RESP)
            {
                /* Unrelated frames must not postpone the original deadline. */
                uint32_t wait_us = MCU_ElapsedUs(s_state_cycle);
                if (wait_us > TAG_RESP_TIMEOUT_US)
                {
                    if (wait_us > anchor_response_wait_max_us[s_current_anchor])
                        anchor_response_wait_max_us[s_current_anchor] = wait_us;

                    s_result.timeout_count++;
                    rx_timeout_count++;
                    anchor_timeout_count[s_current_anchor]++;
                    anchor_response_timeout_count[s_current_anchor]++;
                    note_anchor_response_timeout(s_current_anchor);
                    mark_anchor_result(s_current_anchor, 0,
                                       (uint8_t)(TAG_ST_TIMEOUT
                                                 | (s_slot_rx_error ? TAG_ST_RXERR : 0U)),
                                       0, 0, 0.0f);
                    advance = 1;
                }
            }

            if (processed_irq)
            {
                uint32_t processing_us = MCU_ElapsedUs(processing_start);
                if (processing_us > anchor_processing_max_us[s_current_anchor])
                    anchor_processing_max_us[s_current_anchor] = processing_us;
            }

            if (advance)
                advance_to_next_anchor();
            break;
        }

        /* ================================================================== */
        case TAG_STATE_INTER_ANCHOR_GUARD:
        /* ================================================================== */
        {
            if (MCU_ElapsedUs(s_state_cycle) < TAG_INTER_ANCHOR_GUARD_US)
                break;

            /* send_poll() clears any residual DW1000 status before TX. */
            dw1000_irq_flag = 0;
            begin_slot();
            send_poll(anchor_id_at(s_current_anchor));
            s_state       = TAG_STATE_TX_POLL;
            s_state_cycle = MCU_CycleNow();
            break;
        }

#if UWB_USE_DS_TWR
        /* ================================================================== */
        case TAG_STATE_TX_FINAL:
        /* ================================================================== */
        {
            /* Guard: FINAL TX must complete within TAG_FINAL_TX_TIMEOUT_US */
            if (MCU_ElapsedUs(s_state_cycle) > TAG_FINAL_TX_TIMEOUT_US
                && !dw1000_irq_flag)
            {
                /* TX stuck — fallback to SS using data already captured */
                DW1000_ForceRxOff();
                DW1000_ClearAllStatus();
                ds_final_tx_timeout_count++;
                anchor_timeout_count[s_current_anchor]++;
                finish_with_ss_fallback();
                break;
            }

            if (!dw1000_irq_flag)
                break;

            dw1000_irq_flag = 0;
            uint64_t status_final = DW1000_ReadStatus();

            if (status_final & DW_TXFRS_BIT)
            {
                /* FINAL sent — capture T5; RX for REPORT is (or is now) on. */
                DW1000_ReadTxTimestamp(s_final_tx_ts);   /* T5 */
                arm_rx_after_tx();
                s_state       = TAG_STATE_WAIT_REPORT;
                s_state_cycle = MCU_CycleNow();
            }
            else
            {
                DW1000_ClearAllStatus();
            }
            break;
        }

        /* ================================================================== */
        case TAG_STATE_WAIT_REPORT:
        /* ================================================================== */
        {
            if (dw1000_irq_flag)
            {
                dw1000_irq_flag = 0;
                const uint64_t status_rpt = DW1000_ReadStatus();
                const DW1000_RxEvent_t event = DW1000_ClassifyRx(status_rpt);

                if (event == DW_RX_EVENT_ERROR)
                {
                    ds_report_rx_error_count++;
                    anchor_rx_error_count[s_current_anchor]++;
                    recover_rx_after_error(status_rpt);
                }
                else if (event == DW_RX_EVENT_NONE)
                {
                    DW1000_ClearTxStatus();
                }
                else if (handle_report_frame() == FRAME_ADVANCE)
                {
                    advance_to_next_anchor();
                    break;
                }
            }

            /* Timeout: REPORT didn't arrive in time → SS fallback. */
            if (s_state == TAG_STATE_WAIT_REPORT
                && MCU_ElapsedUs(s_state_cycle) > TAG_REPORT_TIMEOUT_US)
            {
                DW1000_ForceRxOff();
                DW1000_ClearAllStatus();
                ds_report_timeout_count++;
                anchor_timeout_count[s_current_anchor]++;
                anchor_report_timeout_count[s_current_anchor]++;
                finish_with_ss_fallback();
            }
            break;
        }
#endif /* UWB_USE_DS_TWR */

        /* ================================================================== */
        default:
        /* ================================================================== */
            s_state = TAG_STATE_IDLE;
            break;
    }
}

void Tag_GetSnapshot(TagCycleSnapshot_t *out)
{
    if (out == NULL)
        return;

    uint32_t now = uwb_platform_time_ms();
    out->seq     = tag_sample_seq;
    out->time_ms = now;

    for (uint8_t i = 0; i < TAG_NUM_ANCHORS; i++)
    {
        const AnchorTrack_t *t = &s_track[i];
        TagAnchorSample_t   *a = &out->anchor[i];

        a->anchor_id   = anchor_id_at(i);
        a->valid       = t->valid;
        a->status      = t->status;

        uint32_t age   = now - t->last_success_tick;
        a->age_ms      = (age > 0xFFFF) ? 0xFFFF : (uint16_t)age;

        a->raw_mm      = t->raw_mm;
        a->filtered_mm = t->filtered_mm;
        a->fpp_cdbm    = t->fpp_cdbm;
    }
}

const DW1000_RangingResult* Tag_GetResult(void)
{
    return &s_result;
}
