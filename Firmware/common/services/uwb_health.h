/**
 ******************************************************************************
 * @file    uwb_health.h
 * @brief   Node health: reset cause, boot identity, progress-based watchdog
 *          and bounded radio recovery (FIRMWARE_REVIEW F2, plan N9).
 *
 * Watchdog policy: the hardware watchdog is fed only while the application
 * reports progress (a completed TAG cycle, a handled anchor radio event, or
 * an intentional pause). A state machine that is alive but no longer ranging
 * therefore resets the node instead of hiding behind an unconditional feed.
 *
 * Radio recovery: a DW1000 fault (SPI error, register drift, init failure)
 * triggers up to UWB_HEALTH_RECOVERY_ATTEMPTS re-initialisations with
 * back-off. If they all fail the MCU reboots, at most
 * UWB_HEALTH_MAX_FAULT_REBOOTS times in a row (counter kept in no-init RAM);
 * after that the node holds in a fault state and retries every
 * UWB_HEALTH_HOLD_RETRY_MS, so it is never dead and never reboot-looping.
 ******************************************************************************
 */

#ifndef UWB_HEALTH_H
#define UWB_HEALTH_H

#include <stdint.h>
#include "uwb_app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UWB_WATCHDOG_ENABLED
#define UWB_WATCHDOG_ENABLED 1
#endif
#ifndef UWB_WATCHDOG_TIMEOUT_MS
#define UWB_WATCHDOG_TIMEOUT_MS 1000U
#endif

/** No progress for this long stops the watchdog feed. */
#define UWB_HEALTH_PROGRESS_TIMEOUT_MS  500U
#define UWB_HEALTH_RECOVERY_ATTEMPTS    3U
#define UWB_HEALTH_MAX_FAULT_REBOOTS    3U
#define UWB_HEALTH_HOLD_RETRY_MS        10000U
/** Healthy uptime after which the consecutive fault-reboot counter clears. */
#define UWB_HEALTH_STABLE_CLEAR_MS      60000U
/** Period of the DW1000 register readback (brown-out detection). */
#define UWB_HEALTH_CONFIG_CHECK_MS      1000U

/* Fault causes (last_fault_cause). */
#define UWB_FAULT_NONE     0U
#define UWB_FAULT_INIT     1U   /* DW1000 init/verify failed */
#define UWB_FAULT_SPI      2U   /* SPI transfer error */
#define UWB_FAULT_CONFIG   3U   /* periodic register readback differed */

typedef struct {
    uint32_t reset_cause;             /* UWB_RESET_* of this boot */
    uint32_t device_id;               /* nRF52 FICR DEVICEID low word */
    uint16_t boot_count;              /* resets since power-on (no-init RAM) */
    uint16_t boot_id;                 /* random per boot: host detects reboots */
    uint16_t fault_reboots;           /* consecutive reboots requested by recovery */
    uint8_t  watchdog_enabled;
    uint8_t  fault_hold;              /* recovery gave up, retrying slowly */
    uint8_t  last_fault_cause;        /* UWB_FAULT_* */
    uint8_t  reserved[3];
    uint32_t radio_recoveries;        /* successful DW1000 re-initialisations */
    uint32_t radio_recovery_failures;
    uint32_t config_checks;
    uint32_t config_mismatches;
    uint32_t last_config_mismatch;    /* DW_VERIFY_* bits of the last check */
} UwbHealth_t;

extern UwbHealth_t uwb_health;

typedef int (*uwb_radio_reinit_fn)(void);

/** Read reset cause, update boot counters, start the watchdog. */
int uwb_health_init(void);

/** The application made progress (keeps the watchdog fed). */
void uwb_health_note_progress(void);

/** Call every main-loop iteration: feeds the watchdog while progress is recent. */
void uwb_health_service(void);

/** Unconditional feed, only for deliberate long waits (recovery back-off,
 *  flash writes). */
void uwb_health_feed(void);

/**
 * Try to bring the radio back with `reinit`. Reboots (bounded) or enters the
 * fault hold on persistent failure.
 * @retval 0 when the radio works again, negative while in fault hold.
 */
int uwb_health_recover_radio(uwb_radio_reinit_fn reinit, uint8_t cause);

/** In fault hold: 1 when the next slow retry is due. */
uint8_t uwb_health_hold_retry_due(void);

/** 1 when the periodic DW1000 register check is due. */
uint8_t uwb_health_config_check_due(void);

/** Record the result of a DW1000_VerifyConfig() check. */
void uwb_health_note_config_check(uint32_t mismatch);

#ifdef __cplusplus
}
#endif

#endif /* UWB_HEALTH_H */
