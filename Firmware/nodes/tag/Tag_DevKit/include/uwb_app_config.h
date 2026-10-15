#ifndef UWB_APP_CONFIG_H
#define UWB_APP_CONFIG_H

#include <stdint.h>

/* Node identity. The TAG addresses anchors dynamically. */
#define TAG_ADDR ((uint16_t)0U)
#define ANCHOR_ADDR ((uint16_t)1U)

/*
 * DWM1001-DEV evaluation-board profile.
 *
 * Calibration from the custom carrier must not be reused on the devkit.
 * Start from the nominal DW1000 antenna delay and publish measurements as
 * calibration diagnostics until every physical Tag/Anchor pair has been
 * measured again with this board.
 */
#define UWB_USE_DS_TWR 1
#define UWB_USE_HW_ANTENNA_DELAY 1
#define UWB_USE_LEGACY_OFFSET 0
#define UWB_USE_CLOCK_CORRECTION 0
#define UWB_DS_CALIBRATED_MASK 0U
#define UWB_DS_OFFSET_A1_M 0.0
#define UWB_DS_OFFSET_A2_M 0.0
#define UWB_DS_OFFSET_A3_M 0.0
#define UWB_DS_OFFSET_A4_M 0.0
#define UWB_DS_OFFSET_A5_M 0.0
#define UWB_DS_OFFSET_A6_M 0.0
#define UWB_DS_OFFSET_A7_M 0.0
#define UWB_DS_OFFSET_A8_M 0.0

/* Range conditioner: MEDIAN_GATE (median-3, physical/dynamic gate and
 * NLOS-aware reacquisition). The Legacy median + static Kalman lags
 * 0.75-2.9 s once the drone moves (test_tag_motion.c,
 * Plan/BAO_CAO_KIEM_TRA_FIRMWARE_V2_2026-09-24.md). The host (UP 7000) runs
 * the real estimator on corrected_mm; this filter only sets validity.
 * #ifndef so that an A/B build can pass -DUWB_RANGE_FILTER_MODE=0U (Legacy)
 * or 2U (CV_KALMAN_V2). */
#ifndef UWB_RANGE_FILTER_MODE
#define UWB_RANGE_FILTER_MODE 1U   /* UWB_RANGE_FILTER_MEDIAN_GATE */
#endif

/* Adaptive Legacy is OFF (it requires UWB_RANGE_FILTER_MODE=0U). SHADOW
 * doubled the soft-float filter work in every slot. Set 1U (SHADOW) or 2U
 * (ACTIVE) only for a controlled A/B of the Legacy path. */
#define UWB_LEGACY_ADAPTIVE_MODE 0U

/* One-to-many burst DS-TWR v3 (tag_ranging.h): one broadcast POLL, eight
 * slotted RESPs, one broadcast FINAL, ~4.5 ms per cycle for all eight
 * anchors instead of ~34 ms sequentially. Every anchor must run the v3
 * responder. SET_BURST switches schemes and tunes the slots at run time;
 * -DUWB_TAG_BURST_DEFAULT=0U builds the sequential A/B. */
#ifndef UWB_TAG_BURST_DEFAULT
#define UWB_TAG_BURST_DEFAULT 1U
#endif

#if defined(UWB_HIGH_RATE_300) && UWB_HIGH_RATE_300
#ifndef UWB_HIGH_RATE
#define UWB_HIGH_RATE 1
#endif
#ifndef UWB_PHY_PREAMBLE
#define UWB_PHY_PREAMBLE 64U
#endif
/* 300 Hz foundation profile: nominal 322.6 cycles/s (3100 us period), leaving
 * ~7.5% DS/drop headroom above 300 sets/s across all eight anchors.
 * Active air + gap time is 2550 UUS (~2615 us) + 73 us POLL preamble + 25 us
 * FINAL tail + 180 us gap = ~2893 us (3025 us on the 1 Hz info-TLV cycle). */
#define UWB_BURST_BASE_UUS           450U
#define UWB_BURST_SLOT_UUS           250U
#define UWB_BURST_FINAL_MARGIN_UUS   350U
#define UWB_BURST_GAP_US             180U
#define UWB_BURST_PERIOD_US          3100U
#define UWB_BURST_INFO_EXTRA_UUS     32U
#elif defined(UWB_HIGH_RATE) && UWB_HIGH_RATE
/* Rev 2: nominal 222.2 cycles/s, leaving DS/drop headroom above 200 sets/s.
 * WAIT4RESP anchor RX, fast SPI and CPU profiles must be flashed together.
 * These are scheduling targets; successful DS throughput needs bench data. */
#define UWB_BURST_BASE_UUS           550U
#define UWB_BURST_SLOT_UUS           350U
#define UWB_BURST_FINAL_MARGIN_UUS   450U
#define UWB_BURST_GAP_US             250U
#define UWB_BURST_PERIOD_US          4500U
#else
/* Bench-verified timing (2026-09-30): the previous 800/350/450 UUS settings
 * missed nearly every RESP on real boards.  All eight anchors respond with
 * this profile; A1-A4 also need the scheduled FINAL timeout in the current
 * anchor image for reliable eight-anchor DS.  The 20 ms floor gives 50 Hz. */
#define UWB_BURST_BASE_UUS          2500U
#define UWB_BURST_SLOT_UUS          1000U
#define UWB_BURST_FINAL_MARGIN_UUS  1200U
#define UWB_BURST_GAP_US            1500U
#define UWB_BURST_PERIOD_US         20000U
#endif
/* Eight installed anchors: A1-A4 DWM1001C, A5-A8 STM32F103 + DW1000. */
#define UWB_TAG_ACTIVE_ANCHOR_MASK  0xFFU

/* Set to 1 for human-readable CSV, or 0 for the binary packet protocol. */
#define TELEM_ASCII 0

/* Telemetry at boot: snapshot + per-measurement records (RANGE_BURST 0x18
 * per cycle in the burst scheme, RANGE_MEAS 0x10 when sequential) +
 * diagnostics. RANGE_MEAS needs >= 460800 baud; app.overlay runs UART0 at
 * 460800 (UARTE). Settings saved earlier with SAVE_SETTINGS keep their own
 * features and override this default until the next SAVE_SETTINGS or
 * FACTORY_RESET. Expands to TELEM_FEATURE_* from telemetry.h at its use. */
#define UWB_TELEM_DEFAULT_FEATURES \
    (TELEM_FEATURE_RANGE_SNAPSHOT | TELEM_FEATURE_RANGE_MEAS | TELEM_FEATURE_DIAG)

#define UWB_WATCHDOG_ENABLED 1
#define UWB_WATCHDOG_TIMEOUT_MS 1000U

#endif /* UWB_APP_CONFIG_H */
