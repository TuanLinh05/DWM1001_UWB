/**
 ******************************************************************************
 * @file    uwb_settings.h
 * @brief   Persistent TAG configuration (Zephyr settings on the board's
 *          storage_partition, 0x7A000, 24 KB).
 *
 * Schema v2 stores one CRC-protected snapshot at uwb/config. It contains the
 * radio profile, DS calibration table, active mask, telemetry features,
 * generation, TAG PARTID and calibration profile identity. Legacy schema-1
 * keys are ignored fail-closed and removed by FACTORY_RESET.
 *
 * Nothing is written unless the host sends SAVE_SETTINGS while ranging is
 * paused: a flash erase stalls the CPU and must never overlap a ranging slot.
 * Without stored settings the node behaves exactly like the compile-time
 * configuration.
 ******************************************************************************
 */

#ifndef UWB_SETTINGS_H
#define UWB_SETTINGS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_SETTINGS_LOADED_RADIO 0x01U
#define UWB_SETTINGS_LOADED_CAL   0x02U
#define UWB_SETTINGS_LOADED_TAG   0x04U
#define UWB_SETTINGS_CAL_REJECTED 0x08U
#define UWB_SETTINGS_INVALID_BLOB 0x10U
#define UWB_SETTINGS_LEGACY_IGNORED 0x20U
#define UWB_SETTINGS_LOAD_FAILED  0x40U
#define UWB_SETTINGS_INIT_FAILED  0x80U

/** Bitmask of the blobs applied at boot (UWB_SETTINGS_LOADED_*). */
extern uint8_t uwb_settings_status;

/** Initialise the backend and apply stored values to the runtime config.
 *  Call before the radio is configured. @retval 0, or negative on failure
 *  (the compile-time defaults then stay in effect). */
int uwb_settings_load(void);

/**
 * Apply the calibration part of a loaded snapshot after DW1000_Init() has
 * populated OTP identity. Mismatched PARTID/RF profiles are rejected and all
 * calibration flags are cleared. Safe to call repeatedly; it applies once.
 */
int uwb_settings_apply_calibration(void);

/** Store the current runtime configuration. Ranging must be paused. */
int uwb_settings_save(void);

/** Delete stored settings and restore the compile-time defaults in RAM. */
int uwb_settings_factory_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* UWB_SETTINGS_H */
