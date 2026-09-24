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

/* Adaptive Legacy is OFF: the published range uses the validated Legacy
 * median + Kalman path, and the host (UP 7000) runs the real estimator.
 * SHADOW doubled the soft-float filter work in every slot. Set 1U (SHADOW)
 * or 2U (ACTIVE) only for a controlled A/B. */
#define UWB_LEGACY_ADAPTIVE_MODE 0U

/* Set to 1 for human-readable CSV, or 0 for the binary packet protocol. */
#define TELEM_ASCII 0

#define UWB_WATCHDOG_ENABLED 1
#define UWB_WATCHDOG_TIMEOUT_MS 1000U

#endif /* UWB_APP_CONFIG_H */
