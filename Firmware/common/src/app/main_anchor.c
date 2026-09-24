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

#define ANCHOR_FAULT_BLINK_MS 150U
#define ANCHOR_LINK_HOLD_MS   1000U

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
        uwb_platform_led_set(false);
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
    uwb_platform_led_set(false);
    uwb_platform_fault_led_set(false);
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
    uint32_t last_link_ms = last_blink_ms;

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
                uwb_platform_led_set(false);
                uwb_platform_fault_led_set(true);
            } else if ((uint32_t)(now - last_blink_ms) >= ANCHOR_FAULT_BLINK_MS) {
                uwb_platform_led_toggle();
                last_blink_ms = now;
            }
            if (uwb_health_hold_retry_due()) {
                radio_ok = recover(UWB_FAULT_INIT);
                spi_errors_seen = dw1000_spi_error_count;
            }
        }

        if (radio_ok && completed_exchange_count() != link_count_seen) {
            link_count_seen = completed_exchange_count();
            last_link_ms = now;
            uwb_platform_led_set(true);
        } else if (radio_ok && (uint32_t)(now - last_link_ms) >= ANCHOR_LINK_HOLD_MS) {
            uwb_platform_led_set(false);
        }

        uwb_health_service();
        k_busy_wait(10U);
    }

    return 0;
}
