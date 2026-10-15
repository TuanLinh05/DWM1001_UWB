/**
 ******************************************************************************
 * @file    uwb_health.c
 * @brief   Reset cause, boot identity, progress watchdog, radio recovery.
 ******************************************************************************
 */

#include "uwb_health.h"
#include "uwb_platform.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

#define WATCHDOG_NODE DT_ALIAS(watchdog0)
#define RETAINED_MAGIC 0x55574248UL   /* "UWBH" */

UwbHealth_t uwb_health;

/* Survives warm resets (software, watchdog, pin); garbage after power-on,
 * which the magic check detects. */
static __noinit struct {
    uint32_t magic;
    uint16_t boot_count;
    uint16_t fault_reboots;
    uint32_t magic_inv;
} s_retained;

static const struct device *const s_watchdog = DEVICE_DT_GET(WATCHDOG_NODE);
static int s_watchdog_channel = -1;
static uint32_t s_last_progress_ms;
static uint32_t s_last_config_check_ms;
static uint32_t s_hold_since_ms;

static int watchdog_start(void)
{
#if UWB_WATCHDOG_ENABLED
    const struct wdt_timeout_cfg timeout = {
        .window = {
            .min = 0U,
            .max = UWB_WATCHDOG_TIMEOUT_MS,
        },
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };

    if (!device_is_ready(s_watchdog)) {
        return -ENODEV;
    }

    s_watchdog_channel = wdt_install_timeout(s_watchdog, &timeout);
    if (s_watchdog_channel < 0) {
        return s_watchdog_channel;
    }

    int error = wdt_setup(s_watchdog,
                          WDT_OPT_PAUSE_HALTED_BY_DBG | WDT_OPT_PAUSE_IN_SLEEP);
    if (error == -ENOTSUP) {
        error = wdt_setup(s_watchdog, 0U);
    }
    return error;
#else
    return 0;
#endif
}

void uwb_health_feed(void)
{
#if UWB_WATCHDOG_ENABLED
    if (s_watchdog_channel >= 0) {
        (void)wdt_feed(s_watchdog, s_watchdog_channel);
    }
#endif
}

int uwb_health_init(void)
{
    const uint32_t cause = uwb_platform_reset_cause();
    const uint8_t power_on = (cause & (UWB_RESET_POR | UWB_RESET_BROWNOUT)) != 0U;

    if (power_on || s_retained.magic != RETAINED_MAGIC
        || s_retained.magic_inv != ~RETAINED_MAGIC) {
        s_retained.boot_count = 0U;
        s_retained.fault_reboots = 0U;
    }
    s_retained.magic = RETAINED_MAGIC;
    s_retained.magic_inv = ~RETAINED_MAGIC;
    if (s_retained.boot_count < 0xFFFFU) {
        s_retained.boot_count++;
    }

    uwb_health.reset_cause = cause;
    uwb_health.device_id = uwb_platform_device_id();
    uwb_health.boot_count = s_retained.boot_count;
    uwb_health.fault_reboots = s_retained.fault_reboots;
    uwb_health.boot_id = (uint16_t)(uwb_platform_random32() & 0xFFFFU);
    uwb_health.last_fault_cause = UWB_FAULT_NONE;

    s_last_progress_ms = uwb_platform_time_ms();
    s_last_config_check_ms = s_last_progress_ms;

    const int error = watchdog_start();
    uwb_health.watchdog_enabled = (error == 0 && s_watchdog_channel >= 0) ? 1U : 0U;
    return error;
}

void uwb_health_note_progress(void)
{
    s_last_progress_ms = uwb_platform_time_ms();
}

void uwb_health_service(void)
{
    const uint32_t now = uwb_platform_time_ms();

    if ((uint32_t)(now - s_last_progress_ms) < UWB_HEALTH_PROGRESS_TIMEOUT_MS) {
        uwb_health_feed();
    }

    /* A node that has been healthy for a while earns back its reboot budget. */
    if (s_retained.fault_reboots != 0U && !uwb_health.fault_hold
        && now > UWB_HEALTH_STABLE_CLEAR_MS) {
        s_retained.fault_reboots = 0U;
        uwb_health.fault_reboots = 0U;
    }
}

/* Sleep while keeping the watchdog alive. */
static void health_wait_ms(uint32_t delay_ms)
{
    while (delay_ms > 0U) {
        const uint32_t step = delay_ms > 100U ? 100U : delay_ms;
        uwb_health_feed();
        k_msleep((int32_t)step);
        delay_ms -= step;
    }
    uwb_health_feed();
}

int uwb_health_recover_radio(uwb_radio_reinit_fn reinit, uint8_t cause)
{
    static const uint16_t backoff_ms[UWB_HEALTH_RECOVERY_ATTEMPTS] = { 20U, 100U, 500U };

    uwb_health.last_fault_cause = cause;

    for (uint8_t attempt = 0U; attempt < UWB_HEALTH_RECOVERY_ATTEMPTS; attempt++) {
        uwb_health_feed();
        if (reinit != NULL && reinit() == 0) {
            uwb_health.radio_recoveries++;
            uwb_health.fault_hold = 0U;
            uwb_health_note_progress();
            return 0;
        }
        uwb_health.radio_recovery_failures++;
        health_wait_ms(backoff_ms[attempt]);
    }

    if (s_retained.fault_reboots < UWB_HEALTH_MAX_FAULT_REBOOTS) {
        s_retained.fault_reboots++;
        uwb_platform_reboot();
    }

    /* Reboots did not help either: hold, keep the watchdog fed, retry slowly. */
    uwb_health.fault_hold = 1U;
    uwb_health.fault_reboots = s_retained.fault_reboots;
    s_hold_since_ms = uwb_platform_time_ms();
    uwb_health_note_progress();
    return -EIO;
}

uint8_t uwb_health_hold_retry_due(void)
{
    const uint32_t now = uwb_platform_time_ms();

    /* The hold loop is alive by design: keep the watchdog fed. */
    uwb_health_note_progress();
    if (!uwb_health.fault_hold) {
        return 0U;
    }
    if ((uint32_t)(now - s_hold_since_ms) < UWB_HEALTH_HOLD_RETRY_MS) {
        return 0U;
    }
    s_hold_since_ms = now;
    return 1U;
}

uint8_t uwb_health_config_check_due(void)
{
    const uint32_t now = uwb_platform_time_ms();

    if ((uint32_t)(now - s_last_config_check_ms) < UWB_HEALTH_CONFIG_CHECK_MS) {
        return 0U;
    }
    s_last_config_check_ms = now;
    return 1U;
}

void uwb_health_note_config_check(uint32_t mismatch)
{
    uwb_health.config_checks++;
    uwb_health.last_config_mismatch = mismatch;
    if (mismatch != 0U) {
        uwb_health.config_mismatches++;
    }
}
