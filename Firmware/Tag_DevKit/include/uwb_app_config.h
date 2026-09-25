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

/* Set to 1 for human-readable CSV, or 0 for the binary packet protocol. */
#define TELEM_ASCII 0

/* Telemetry at boot: snapshot + one RANGE_MEAS (0x10) per measurement +
 * diagnostics. RANGE_MEAS needs >= 460800 baud; app.overlay runs UART0 at
 * 1 Mbaud (UARTE). Settings saved earlier with SAVE_SETTINGS keep their own
 * features and override this default until the next SAVE_SETTINGS or
 * FACTORY_RESET. Expands to TELEM_FEATURE_* from telemetry.h at its use. */
#define UWB_TELEM_DEFAULT_FEATURES \
    (TELEM_FEATURE_RANGE_SNAPSHOT | TELEM_FEATURE_RANGE_MEAS | TELEM_FEATURE_DIAG)

#define UWB_WATCHDOG_ENABLED 1
#define UWB_WATCHDOG_TIMEOUT_MS 1000U

#endif /* UWB_APP_CONFIG_H */
