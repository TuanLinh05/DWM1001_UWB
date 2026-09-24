/**
 ******************************************************************************
 * @file    tag_ranging.h
 * @brief   TAG DS-TWR ranging — non-blocking state machine header
 *
 * Architecture: non-blocking, event-driven, 20 ms cycle.
 *   - Tag_Init()  : DW1000 init + full state reset.
 *   - Tag_Task()  : call repeatedly from the main loop. Runs the
 *                   POLL -> RESP -> FINAL -> REPORT exchange for every active
 *                   anchor in turn and returns immediately if nothing to do.
 *
 * Timing is measured at runtime (tag_cycle_duration_us, slot counters) and
 * published in the DIAG telemetry; 8 sequential DS exchanges do not fit in
 * 20 ms (see Plan/KE_HOACH_NANG_CAP_TOAN_DIEN_FIRMWARE_DWM1001.md §5.3).
 ******************************************************************************
 */

#ifndef TAG_RANGING_H
#define TAG_RANGING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dw1000_hw.h"
#include "range_filter.h"
#include "uwb_frame.h"

/** Single source of truth for the number of anchors polled by TAG. */
#define TAG_NUM_ANCHORS    8U

/* ========================================================================== */
/*                     TIMING CONSTANTS                                        */
/* ========================================================================== */

/** Full ranging cycle period (ms). 20ms = 50Hz for eight DS-TWR anchors. */
#define TAG_CYCLE_MS            20U

/** Fast-50: reply delay ~=1.23ms (1200 UUS) + preamble256 airtime + margin. */
#define TAG_RESP_TIMEOUT_US     2600U

/** TX stuck timeout: POLL TX should never take more than 5ms. */
#define TAG_TX_TIMEOUT_US       5000U

/**
 * Quiet time between two anchor slots.
 *
 * Every anchor can hear the previous anchor's RESP even when the frame is not
 * addressed to it. Give those receivers enough time to consume the stray frame
 * and restart RX before the TAG transmits the next POLL. This state is
 * non-blocking and adds only 7 x 150us to an eight-anchor cycle.
 */
#define TAG_INTER_ANCHOR_GUARD_US 150U

/**
 * Minimum quiet time after a complete A1..A8 cycle.
 *
 * If a cycle overruns TAG_CYCLE_MS, the scheduler deadline is already in the
 * past. Without this recovery interval the next A1 POLL is sent immediately,
 * which can keep the radios in a timeout/overrun feedback loop.
 */
#define TAG_INTER_CYCLE_RECOVERY_US 500U

/**
 * Offline-anchor probing policy.
 *
 * A missing anchor otherwise consumes the full response timeout in every
 * 20 ms cycle. After a short confirmation streak, poll it periodically while
 * keeping responsive anchors at the nominal cycle rate. At most one backoff
 * probe is admitted per cycle, preventing two offline anchors from combining
 * into a deadline miss.
 */
#define TAG_OFFLINE_AFTER_TIMEOUTS          3U
#define TAG_OFFLINE_PROBE_INTERVAL_CYCLES   5U

/** DS-TWR: FINAL TX stuck timeout (µs). */
#define TAG_FINAL_TX_TIMEOUT_US  5000U
/** DS-TWR: REPORT is immediate and uses preamble256; retain generous margin. */
#define TAG_REPORT_TIMEOUT_US    3000U

/* ========================================================================== */
/*                     PROTOCOL OPTIONS                                        */
/* ========================================================================== */

/**
 * Frame version the TAG transmits. v2 carries a transaction ID through the
 * whole exchange (F4). Anchors built from this source accept v1 and v2, so
 * flash every anchor first, then the TAG. Set to 1 only to talk to anchors
 * that still run the baseline firmware.
 */
#ifndef UWB_TAG_FRAME_VERSION
#define UWB_TAG_FRAME_VERSION   UWB_FRAME_V2
#endif
#if (UWB_TAG_FRAME_VERSION != UWB_FRAME_V1) && (UWB_TAG_FRAME_VERSION != UWB_FRAME_V2)
#error "UWB_TAG_FRAME_VERSION must be 1 or 2"
#endif

/**
 * 1: POLL and FINAL are sent with WAIT4RESP so the DW1000 enables its
 * receiver right after the frame. Removes the FINAL -> REPORT race (N6).
 */
#ifndef UWB_USE_WAIT4RESP
#define UWB_USE_WAIT4RESP       1
#endif

/** Anchors polled after boot (bit n = slot n = anchor ID n+1). */
#ifndef UWB_TAG_ACTIVE_ANCHOR_MASK
#define UWB_TAG_ACTIVE_ANCHOR_MASK 0xFFU
#endif

/** Measurement records buffered for per-measurement telemetry. */
#define TAG_MEAS_QUEUE_LEN      16U

/* ========================================================================== */
/*                     CALIBRATION VARIABLES                                   */
/* ========================================================================== */

/**
 * Software calibration offset (meters).
 * distance_final = distance_raw - calibration_offset_m
 * Set auto_calibrate_actual_m to the true distance to auto-recalibrate.
 */
extern volatile double calibration_offset_m[TAG_NUM_ANCHORS];
extern volatile double auto_calibrate_actual_m;
extern volatile uint16_t auto_calibrate_anchor_id;
extern volatile double residual_offset_m[TAG_NUM_ANCHORS];

/* ========================================================================== */
/*                     PUBLIC VARIABLES                                        */
/* ========================================================================== */

/* Published per-anchor distances (raw) */
extern volatile int32_t distance_a1_mm;
extern volatile int32_t distance_a2_mm;
extern volatile int32_t distance_a3_mm;
extern volatile int32_t distance_a4_mm;
extern volatile int32_t distance_a5_mm;
extern volatile int32_t distance_a6_mm;
extern volatile int32_t distance_a7_mm;
extern volatile int32_t distance_a8_mm;
extern volatile int32_t distance_raw_mm[TAG_NUM_ANCHORS];

/* Published per-anchor distances (Kalman filtered) */
extern volatile int32_t distance_a1_filtered_mm;
extern volatile int32_t distance_a2_filtered_mm;
extern volatile int32_t distance_a3_filtered_mm;
extern volatile int32_t distance_a4_filtered_mm;
extern volatile int32_t distance_a5_filtered_mm;
extern volatile int32_t distance_a6_filtered_mm;
extern volatile int32_t distance_a7_filtered_mm;
extern volatile int32_t distance_a8_filtered_mm;
extern volatile int32_t distance_filtered_mm[TAG_NUM_ANCHORS];

/* Total 20ms cycles executed (nominally 50 per second). */
extern volatile uint32_t tag_cycle_count;

/* ========================================================================== */
/*                     RUNTIME CALIBRATION / SCHEDULING API                    */
/* ========================================================================== */

/**
 * DS calibration is a runtime table initialised from UWB_DS_OFFSET_Ax_M /
 * UWB_DS_CALIBRATED_MASK and replaceable by command/settings, so a
 * calibration no longer needs a rebuild.
 *
 * Sign convention (F5): bias_um is subtracted from the measurement,
 *   corrected = measured - bias.
 * The GUI's "offset = reference - mean_raw" is the NEGATIVE of bias.
 */
int Tag_SetDsCalibration(uint16_t anchor_id, int32_t bias_um, uint8_t calibrated);
int Tag_GetDsCalibration(uint16_t anchor_id, int32_t *bias_um, uint8_t *calibrated);
uint8_t Tag_DsCalibratedMask(void);

/**
 * Fail closed after any RF/profile change: retain the measured biases for
 * diagnostics, but mark every entry uncalibrated and reset filters whose
 * corrected-unit state is no longer valid.
 */
void Tag_InvalidateDsCalibration(void);

/** Restore the compile-time calibration table and anchor mask. */
void Tag_RestoreDefaultConfiguration(void);

/** Select the anchors that are polled (bit n = anchor ID n+1). */
void Tag_SetActiveAnchorMask(uint8_t mask);
uint8_t Tag_GetActiveAnchorMask(void);

/**
 * Pause/resume ranging. A pause takes effect at the end of the current
 * cycle; Tag_IsPaused() reports when the radio is idle and it is safe to
 * reconfigure it or write flash.
 */
void Tag_RequestPause(uint8_t pause);
uint8_t Tag_IsPaused(void);

/**
 * Re-initialise the DW1000 after a fault and restart the state machine.
 * Every anchor is marked invalid until it is measured again.
 * @retval 0 on success, negative on DW1000 init failure.
 */
int Tag_RecoverRadio(void);

/** 1 while the state machine is between cycles (radio idle). */
uint8_t Tag_IsIdle(void);

/* ========================================================================== */
/*                     PHASE 1: TELEMETRY & COUNTERS                           */
/* ========================================================================== */

/** Per-anchor status bit flags (trong TagAnchorSample_t.status). */
#define TAG_ST_OK          0x00  /* Đo thành công trong lần thử gần nhất */
#define TAG_ST_TIMEOUT     0x01  /* Không nhận RESP kịp thời */
#define TAG_ST_RXERR       0x02  /* Lỗi RX (FCS fail...) */
#define TAG_ST_BADFRAME    0x04  /* Frame sai địa chỉ/loại/độ dài */
#define TAG_ST_COMPUTE     0x08  /* compute_distance_mm trả -1 */
#define TAG_ST_DS_FALLBACK 0x10  /* Chu kỳ này dùng SS fallback (mất REPORT) */
/** Status allocation đã freeze tại C0.1 để không xung đột C9 range filter.
 *  0x20 mang nghĩa chung: profile calibration của mode hiện tại còn thiếu;
 *  measurement chỉ là diagnostic và production valid=0. */
#define TAG_ST_CALIBRATION_MISSING 0x20U
/** Tên tương thích cho code/GUI C0 DS; không tạo thêm wire bit mới. */
#define TAG_ST_CAL_MISSING_DS      TAG_ST_CALIBRATION_MISSING
#define TAG_ST_RANGE_REJECT      0x40U /* C9: bounds/motion reject; cause ở diag counter */
#define TAG_ST_FILTER_REACQUIRE  0x80U /* C9: range conditioner reacquired */

/**
 * @brief Kết quả một anchor trong một chu kỳ đo.
 *        valid = có phép đo mới thành công trong lần thử gần nhất.
 *        age_ms = tuổi kể từ lần đo thành công cuối (0 nếu tươi).
 */
typedef struct {
    uint16_t anchor_id;
    uint8_t  valid;
    uint8_t  status;        /* TAG_ST_* bit flags của lần thử gần nhất */
    uint16_t age_ms;        /* now - last_success_tick (clamp 65535) */
    int32_t  raw_mm;        /* thường: sau offset; CALIBRATION_MISSING: trước offset để calib */
    int32_t  filtered_mm;   /* sau median+Kalman; giữ success cũ khi valid=0 */
    int16_t  fpp_cdbm;      /* first-path power, centi-dBm (dBm*100) */
} TagAnchorSample_t;

/** Snapshot toàn bộ một chu kỳ để đóng gói telemetry. */
typedef struct {
    uint32_t          seq;       /* tag_sample_seq */
    uint32_t          time_ms;   /* Zephyr uptime lúc chụp */
    TagAnchorSample_t anchor[TAG_NUM_ANCHORS];
} TagCycleSnapshot_t;

/* Cờ báo một chu kỳ (A1→...→A8) vừa hoàn tất — set trong Tag_Task, xoá ở main. */
extern volatile uint8_t  tag_cycle_ready;

/* Sequence tăng đúng 1 lần mỗi chu kỳ hoàn tất (thay cho việc so sánh giá trị). */
extern volatile uint32_t tag_sample_seq;

/* Counters cho observability (Phase 1). */
extern volatile uint32_t poll_sent_count;
extern volatile uint32_t response_ok_count;
extern volatile uint32_t rx_timeout_count;
extern volatile uint32_t rx_error_count;
extern volatile uint32_t cycle_overrun_count;

/** Per-anchor diagnostics, indexed by the TAG configuration table. */
extern volatile uint32_t anchor_success_count[TAG_NUM_ANCHORS];
extern volatile uint32_t anchor_timeout_count[TAG_NUM_ANCHORS];
extern volatile uint32_t anchor_rx_error_count[TAG_NUM_ANCHORS];
/** Split timeout causes so Live Expressions can distinguish TX from missing RESP. */
extern volatile uint32_t anchor_poll_tx_timeout_count[TAG_NUM_ANCHORS];
extern volatile uint32_t anchor_response_timeout_count[TAG_NUM_ANCHORS];
extern volatile uint16_t anchor_response_timeout_streak[TAG_NUM_ANCHORS];
extern volatile uint32_t anchor_poll_skipped_count[TAG_NUM_ANCHORS];
/** Valid radio/ToF measurements rejected only because calibration is missing. */
extern volatile uint32_t anchor_calibration_missing_count[TAG_NUM_ANCHORS];

#if UWB_RANGE_FILTER_MODE != UWB_RANGE_FILTER_LEGACY_KALMAN
/** C9 candidate diagnostics; intentionally kept outside the range wire record. */
extern RangeFilterState_t range_filter_state[TAG_NUM_ANCHORS];
extern volatile uint8_t range_filter_last_decision[TAG_NUM_ANCHORS];
extern volatile int32_t range_filter_last_innovation_mm[TAG_NUM_ANCHORS];
extern volatile uint32_t range_filter_max_abs_innovation_mm[TAG_NUM_ANCHORS];
#endif

/* Adaptive Legacy diagnostics are intentionally Live-Expressions-only. The
 * fixed range record is not extended, so the USB/Wi-Fi binary contract stays
 * identical in OFF, SHADOW and ACTIVE builds. */
#define TAG_LEGACY_ADAPTIVE_STABLE          0U
#define TAG_LEGACY_ADAPTIVE_CANDIDATE       1U
#define TAG_LEGACY_ADAPTIVE_TRACKING        2U
#define TAG_LEGACY_ADAPTIVE_STALE_REACQUIRE 3U

extern volatile uint8_t legacy_adaptive_state[TAG_NUM_ANCHORS];
extern volatile uint8_t legacy_adaptive_candidate_count[TAG_NUM_ANCHORS];
extern volatile int8_t legacy_adaptive_candidate_direction[TAG_NUM_ANCHORS];
extern volatile uint32_t legacy_adaptive_track_enter_count[TAG_NUM_ANCHORS];
extern volatile uint32_t legacy_adaptive_track_exit_count[TAG_NUM_ANCHORS];
extern volatile uint32_t legacy_adaptive_true_reject_count[TAG_NUM_ANCHORS];
extern volatile uint32_t legacy_adaptive_stale_reacquire_count[TAG_NUM_ANCHORS];
extern volatile uint32_t legacy_adaptive_track_duration_ms[TAG_NUM_ANCHORS];
extern volatile int32_t legacy_adaptive_last_innovation_mm[TAG_NUM_ANCHORS];
extern volatile uint32_t legacy_adaptive_max_abs_innovation_mm[TAG_NUM_ANCHORS];
extern volatile double legacy_adaptive_last_q[TAG_NUM_ANCHORS];
extern volatile double legacy_adaptive_last_r[TAG_NUM_ANCHORS];
extern volatile double legacy_adaptive_last_gain[TAG_NUM_ANCHORS];
extern const uint8_t uwb_legacy_adaptive_mode_build;

/* C9.2 motion-regime diagnostics. States: 0=reacquire, 1=static, 2=slow,
 * 3=fast, 4=settling, 5=degraded. The global state is informational only;
 * canonical filtering and validity remain per physical Anchor ID. */
extern volatile uint8_t c9_2_motion_state[TAG_NUM_ANCHORS];
extern volatile uint8_t c9_2_global_motion_state;
extern volatile uint32_t c9_2_static_enter_count[TAG_NUM_ANCHORS];
extern volatile uint32_t c9_2_slow_enter_count[TAG_NUM_ANCHORS];
extern volatile uint32_t c9_2_fast_enter_count[TAG_NUM_ANCHORS];
extern volatile uint32_t c9_2_settling_enter_count[TAG_NUM_ANCHORS];
extern volatile uint32_t c9_2_degraded_enter_count[TAG_NUM_ANCHORS];
extern volatile uint32_t c9_2_stale_reacquire_count[TAG_NUM_ANCHORS];
extern volatile uint32_t c9_2_true_reject_count[TAG_NUM_ANCHORS];
extern volatile int32_t c9_2_last_slope_mm_s[TAG_NUM_ANCHORS];
extern volatile uint32_t c9_2_last_motion_score_milli[TAG_NUM_ANCHORS];
extern volatile double c9_2_last_gain[TAG_NUM_ANCHORS];
extern const uint8_t uwb_c9_2_motion_mode_build;

/** v2: frames whose transaction ID did not match the open exchange. */
extern volatile uint32_t anchor_txn_mismatch_count[TAG_NUM_ANCHORS];
/** DS completion health, separate from RESP health (F8). */
extern volatile uint32_t anchor_ds_ok_count[TAG_NUM_ANCHORS];
extern volatile uint32_t anchor_report_timeout_count[TAG_NUM_ANCHORS];
extern volatile uint32_t anchor_ds_fallback_count[TAG_NUM_ANCHORS];
extern volatile uint16_t anchor_ds_incomplete_streak[TAG_NUM_ANCHORS];
/** Offline probes actually sent, per anchor (F1 fairness evidence). */
extern volatile uint32_t anchor_probe_count[TAG_NUM_ANCHORS];

/** RX failures by cause, all anchors (F3). */
typedef struct {
    uint32_t phy_header;    /* RXPHE */
    uint32_t fcs;           /* RXFCE */
    uint32_t sync_loss;     /* RXRFSL */
    uint32_t frame_timeout; /* RXRFTO */
    uint32_t lde;           /* LDEERR */
    uint32_t overrun;       /* RXOVRR */
    uint32_t preamble_to;   /* RXPTO */
    uint32_t sfd_timeout;   /* RXSFDTO */
    uint32_t filtered;      /* AFFREJ */
    uint32_t incomplete;    /* RXDFR without FCS/LDE success */
    uint32_t soft_resets;   /* RX soft resets issued */
} TagRxErrorStats_t;
extern TagRxErrorStats_t tag_rx_error_stats;

/* DS-TWR counters (Phase 4) — Live Expressions. 0 khi UWB_USE_DS_TWR=0. */
extern volatile uint32_t ds_ok_count;
extern volatile uint32_t ds_report_ok_count;
extern volatile uint32_t ds_report_timeout_count;
extern volatile uint32_t ds_fallback_count;
extern volatile uint32_t ds_final_tx_timeout_count;
extern volatile uint32_t ds_report_rx_error_count;
/** C0.1: Đếm DS result bị invalidate vì anchor hiện tại chưa có bit calibration. */
extern volatile uint32_t ds_cal_missing_count;

/** Build fingerprint đọc qua Live Expressions để tránh test nhầm DS OFF/ON. */
extern const uint8_t uwb_ds_mode_enabled_build;
extern const uint8_t uwb_ds_calibrated_mask_build;

/* Instrumentation chu kỳ (§5.2 Phase 4) — đọc qua Live Expressions. */
extern volatile uint32_t tag_cycle_duration_us;
extern volatile uint32_t tag_cycle_duration_max_us;
/** Duration of the latest/slowest complete slot for each configured Anchor. */
extern volatile uint32_t anchor_slot_duration_us[TAG_NUM_ANCHORS];
extern volatile uint32_t anchor_slot_duration_max_us[TAG_NUM_ANCHORS];
/** Phase timings prove whether a slow slot is radio wait or CPU post-processing. */
extern volatile uint32_t anchor_poll_tx_duration_max_us[TAG_NUM_ANCHORS];
extern volatile uint32_t anchor_response_wait_max_us[TAG_NUM_ANCHORS];
extern volatile uint32_t anchor_processing_max_us[TAG_NUM_ANCHORS];

/**
 * @brief  Chụp snapshot chu kỳ hiện tại (gọi từ main context, không từ ISR).
 * @param  out  con trỏ struct để ghi kết quả
 */
void Tag_GetSnapshot(TagCycleSnapshot_t *out);

/* ========================================================================== */
/*                     PER-MEASUREMENT RECORDS (telemetry v2)                  */
/* ========================================================================== */

#define TAG_MEAS_MODE_DS           0U
#define TAG_MEAS_MODE_SS           1U
#define TAG_MEAS_MODE_SS_FALLBACK  2U

#define TAG_MEAS_FLAG_RADIO_OK     0x0001U  /* exchange + ToF computed */
#define TAG_MEAS_FLAG_CAL_OK       0x0002U  /* calibration applied, valid range */
#define TAG_MEAS_FLAG_FILTER_OK    0x0004U  /* tag filter accepted the sample */
#define TAG_MEAS_FLAG_ANCHOR_DIAG  0x0008U  /* anchor-side FINAL RX power present */

/**
 * @brief One ranging measurement, emitted as soon as it completes.
 *        raw_mm is always the range BEFORE any software offset (fixes the
 *        dual meaning of the snapshot raw_mm, F7).
 */
typedef struct {
    uint64_t meas_time_us;   /* TAG clock, midpoint of the exchange */
    uint32_t meas_seq;
    uint16_t anchor_id;
    uint8_t  txn;
    uint8_t  mode;           /* TAG_MEAS_MODE_* */
    uint16_t flags;          /* TAG_MEAS_FLAG_* */
    uint8_t  status;         /* TAG_ST_* of this attempt */
    int32_t  raw_mm;
    int32_t  corrected_mm;   /* 0 unless TAG_MEAS_FLAG_CAL_OK */
    int32_t  filtered_mm;
    int16_t  fp_cdbm;        /* RESP first-path power at the TAG */
    int16_t  rx_cdbm;        /* RESP receive power at the TAG */
    int16_t  anchor_fp_cdbm; /* FINAL first-path power at the anchor */
    int16_t  anchor_rx_cdbm; /* FINAL receive power at the anchor */
    uint16_t std_noise;
    uint16_t fp_index;       /* 10.6 fixed point */
    int16_t  ci_ppm_x100;    /* clock offset anchor vs TAG, 0.01 ppm */
    uint16_t slot_us;
} TagMeasurement_t;

/**
 * @brief Anchor self-description received in RESP TLVs (commissioning).
 *        The TAG asks one anchor per second (POLL flag REQ_INFO).
 */
typedef struct {
    uint16_t anchor_id;
    uint8_t  valid;
    uint8_t  anchor_status;   /* UWB_ANCHOR_ST_* */
    uint8_t  pos_valid;
    uint8_t  build_dirty;
    uint8_t  tx_power_mode;
    uint8_t  reserved;
    int32_t  pos_mm[3];
    uint32_t build_hash;
    uint32_t build_config_hash;
    uint16_t tx_ant_dly;
    uint16_t rx_ant_dly;
    uint16_t boot_count;
    uint32_t updated_ms;
} TagAnchorInfo_t;

/** Take the next anchor info that arrived since the last call. @retval 1 if any. */
uint8_t Tag_TakeAnchorInfo(TagAnchorInfo_t *out);

/** Enable/disable filling the measurement queue (off: zero overhead). */
void Tag_EnableMeasurementQueue(uint8_t enable);
/** Pop the oldest measurement. @retval 1 if one was returned. */
uint8_t Tag_PopMeasurement(TagMeasurement_t *out);
/** Measurements dropped because the queue was full. */
extern volatile uint32_t tag_meas_queue_drops;

/* ========================================================================== */
/*                     API                                                     */
/* ========================================================================== */

/**
 * @brief  Initialize the TAG: DW1000 init + configure + set address.
 *         Must call DW1000_EnableIRQ() after this.
 * @retval 0 = success, -1 = DW1000 init failure
 */
int Tag_Init(void);

/**
 * @brief  Non-blocking state machine tick. Call from main while(1).
 *
 *         State machine:
 *           IDLE          --[every 20ms]--> TX_POLL (Anchor 1)
 *           TX_POLL       --[IRQ: TXFRS] --> WAIT_RESP
 *           WAIT_RESP     --[IRQ: RXFCG] --> TX_POLL (next anchor)
 *                          --[timeout]   --> TX_POLL (skip anchor)
 *           (after A8)    --> IDLE
 *
 *         Results written to distance_raw_mm[]/distance_filtered_mm[].
 *         distance_a1_mm...distance_a8_mm remain debugger mirrors.
 */
void Tag_Task(void);

/**
 * @brief  Get pointer to the internal ranging result struct.
 * @retval Pointer to DW1000_RangingResult (static, always valid)
 */
const DW1000_RangingResult* Tag_GetResult(void);

#ifdef __cplusplus
}
#endif

#endif /* TAG_RANGING_H */
