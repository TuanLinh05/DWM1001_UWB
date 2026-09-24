#ifndef UWB_APP_CONFIG_H
#define UWB_APP_CONFIG_H

#include <stdint.h>

/*
 * Passive sniffer on a DWM1001-DEV. It never transmits; the addresses only
 * satisfy the shared headers. Keep the radio profile identical to the TAG
 * and anchors (same PHY, same antenna-delay profile).
 */
#define TAG_ADDR ((uint16_t)0U)
#define ANCHOR_ADDR ((uint16_t)0xFFFEU)

#define UWB_USE_DS_TWR 1
#define UWB_USE_HW_ANTENNA_DELAY 1
#define UWB_USE_LEGACY_OFFSET 0
#define UWB_USE_CLOCK_CORRECTION 0

#define UWB_WATCHDOG_ENABLED 1
#define UWB_WATCHDOG_TIMEOUT_MS 1000U

#endif /* UWB_APP_CONFIG_H */
