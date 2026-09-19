#ifndef UWB_APP_CONFIG_H
#define UWB_APP_CONFIG_H

#include <stdint.h>

/* Fixed identity for this standalone project. */
#define TAG_ADDR ((uint16_t)0U)
#define ANCHOR_ADDR ((uint16_t)2U)

/* Custom carrier profile; keep this identical to the Tag radio profile. */
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

#define TELEM_ASCII 0

#define UWB_WATCHDOG_ENABLED 1
#define UWB_WATCHDOG_TIMEOUT_MS 1000U

#endif /* UWB_APP_CONFIG_H */
