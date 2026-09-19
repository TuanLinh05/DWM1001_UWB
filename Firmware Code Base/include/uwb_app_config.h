#ifndef UWB_APP_CONFIG_H
#define UWB_APP_CONFIG_H

#include <stdint.h>
#include <zephyr/autoconf.h>

#ifdef CONFIG_UWB_COMPENSATION_ANTENNA_DELAY
#define UWB_USE_HW_ANTENNA_DELAY 1
#define UWB_USE_LEGACY_OFFSET 0
#else
#define UWB_USE_HW_ANTENNA_DELAY 0
#define UWB_USE_LEGACY_OFFSET 1
#endif

#ifdef CONFIG_UWB_CLOCK_CORRECTION
#define UWB_USE_CLOCK_CORRECTION 1
#else
#define UWB_USE_CLOCK_CORRECTION 0
#endif

#ifdef CONFIG_UWB_DS_TWR
#define UWB_USE_DS_TWR 1
#else
#define UWB_USE_DS_TWR 0
#endif

#ifdef CONFIG_UWB_TELEMETRY_ASCII
#define TELEM_ASCII 1
#else
#define TELEM_ASCII 0
#endif

#define TAG_ADDR ((uint16_t)CONFIG_UWB_TAG_ADDRESS)

#ifdef CONFIG_UWB_ROLE_ANCHOR
#define ANCHOR_ADDR ((uint16_t)CONFIG_UWB_ANCHOR_ADDRESS)
#else
/* The TAG addresses anchors dynamically; this value is never used as a target. */
#define ANCHOR_ADDR ((uint16_t)1U)
#endif

#endif /* UWB_APP_CONFIG_H */
