/**
 ******************************************************************************
 * @file    main_anchor.c
 * @brief   Anchor main loop: responder state machine, health, recovery.
 ******************************************************************************
 */

#include "anchor_ranging.h"
#include "dw1000_hw.h"
#include "uwb_health.h"
#include "uwb_platform.h"

#include <zephyr/kernel.h>

/*
 * Status LED. Each state has its own rhythm, so a board without a UART still
 * tells how far the radio gets (the STM32 anchors have only PC13):
 *   3 quick flashes at boot   firmware started; repeating = reboot loop
 *   steady on                 a DS exchange completed within the last second
 *   1 Hz blink                POLLs for this anchor arrive, but no exchange
 *                             completes (late RESP, lost FINAL or REPORT)
 *   short blip every 2 s      radio listening, no POLL for this anchor
 *   fast 150 ms blink         radio fault, retrying (single-LED boards)
 */
#define ANCHOR_FAULT_BLINK_MS      150U
#define ANCHOR_LINK_HOLD_MS        1000U
#define ANCHOR_BOOT_FLASHES        3U
#define ANCHOR_BOOT_FLASH_MS       120U
#define ANCHOR_POLL_BLINK_MS       500U
#define ANCHOR_HEARTBEAT_PERIOD_MS 2000U
#define ANCHOR_HEARTBEAT_ON_MS     50U

/* Last level written to the status LED; -1 forces the next write. */
static int8_t s_led_state = -1;

static void status_led_show(bool on)
{
    const int8_t state = on ? 1 : 0;

    if (s_led_state != state) {
        uwb_platform_led_set(on);
        s_led_state = state;
    }
}

/* Proves the image runs and the LED is wired, before anything can fail. */
static void boot_flash(void)
{
    for (uint8_t i = 0U; i < ANCHOR_BOOT_FLASHES; i++) {
        status_led_show(true);
        k_msleep(ANCHOR_BOOT_FLASH_MS);
        status_led_show(false);
        k_msleep(ANCHOR_BOOT_FLASH_MS);
    }
}

/* LED level while the radio runs, from the last exchange and POLL times. */
static bool running_led_on(uint32_t now, uint32_t last_link_ms, uint32_t last_poll_ms)
{
    if ((uint32_t)(now - last_link_ms) < ANCHOR_LINK_HOLD_MS) {
        return true;
    }
    if ((uint32_t)(now - last_poll_ms) < ANCHOR_LINK_HOLD_MS) {
        return ((now / ANCHOR_POLL_BLINK_MS) & 1U) == 0U;
    }
    return (now % ANCHOR_HEARTBEAT_PERIOD_MS) < ANCHOR_HEARTBEAT_ON_MS;
}

/* Platform failure before the watchdog runs: blink forever. */
static void platform_fault_forever(void)
{
    while (true) {
        if (uwb_platform_fault_led_available()) {
            uwb_platform_fault_led_toggle();
        } else {
            uwb_platform_led_toggle();
        }
        k_msleep(ANCHOR_FAULT_BLINK_MS);
    }
}

static uint32_t completed_exchange_count(void)
{
#if UWB_USE_DS_TWR
    return anchor_stats.reports_sent;
#else
    return anchor_stats.resp_sent;
#endif
}

static uint8_t recover(uint8_t cause)
{
    const uint8_t ok = (uwb_health_recover_radio(Anchor_RecoverRadio, cause) == 0) ? 1U : 0U;
    if (ok) {
        status_led_show(false);
        uwb_platform_fault_led_set(false);
        Anchor_NoteRecovery();
    }
    return ok;
}

int main(void)
{
    if (uwb_platform_init() != 0) {
        platform_fault_forever();
    }
    uwb_platform_fault_led_set(false);
    boot_flash();
    MCU_TimerInit();
    (void)uwb_health_init();

    uint8_t radio_ok = 1U;
    if (Anchor_Init() != 0) {
        radio_ok = recover(UWB_FAULT_INIT);
    }
    DW1000_EnableIRQ();
    if (radio_ok) {
        Anchor_StartListening();
    }

    uint32_t spi_errors_seen = dw1000_spi_error_count;
    uint32_t events_seen = Anchor_EventCount();
    uint32_t last_blink_ms = uwb_platform_time_ms();
    uint32_t link_count_seen = completed_exchange_count();
    uint32_t polls_seen = anchor_stats.polls;
    /* Both already expired: nothing has happened since boot. */
    uint32_t last_link_ms = last_blink_ms - ANCHOR_LINK_HOLD_MS;
    uint32_t last_poll_ms = last_link_ms;

    while (true) {
        const uint32_t now = uwb_platform_time_ms();
        /* IRQ is level-held by the DW1000; polling closes the edge race. */
        if (DW1000_IrqLineActive() != 0U) {
            dw1000_irq_flag = 1U;
        }

        if (radio_ok) {
            uwb_platform_fault_led_set(false);
            Anchor_Task();

            if (dw1000_spi_error_count != spi_errors_seen) {
                radio_ok = recover(UWB_FAULT_SPI);
                spi_errors_seen = dw1000_spi_error_count;
            } else if (Anchor_InQuietWindow()) {
                /* Right after an exchange or an idle RX restart: a register
                 * readback cannot collide with this anchor's next POLL. */
                Anchor_ConsumeQuietWindow();
                if (uwb_health_config_check_due()) {
                    const uint32_t mismatch = DW1000_VerifyConfig();
                    uwb_health_note_config_check(mismatch);
                    Anchor_NoteConfigCheck(mismatch);
                    if (mismatch != 0U) {
                        radio_ok = recover(UWB_FAULT_CONFIG);
                    }
                    spi_errors_seen = dw1000_spi_error_count;
                }
            }

            /* Progress: every handled radio event, including the idle RX
             * restart that runs at least every 200 ms. */
            if (Anchor_EventCount() != events_seen) {
                events_seen = Anchor_EventCount();
                uwb_health_note_progress();
            }
        } else {
            if (uwb_platform_fault_led_available()) {
                status_led_show(false);
                uwb_platform_fault_led_set(true);
            } else if ((uint32_t)(now - last_blink_ms) >= ANCHOR_FAULT_BLINK_MS) {
                status_led_show(s_led_state != 1);
                last_blink_ms = now;
            }
            if (uwb_health_hold_retry_due()) {
                radio_ok = recover(UWB_FAULT_INIT);
                spi_errors_seen = dw1000_spi_error_count;
            }
        }

        if (radio_ok) {
            if (completed_exchange_count() != link_count_seen) {
                link_count_seen = completed_exchange_count();
                last_link_ms = now;
            }
            if (anchor_stats.polls != polls_seen) {
                polls_seen = anchor_stats.polls;
                last_poll_ms = now;
            }
            status_led_show(running_led_on(now, last_link_ms, last_poll_ms));
        }

        uwb_health_service();
        k_busy_wait(10U);
    }

    return 0;
}
