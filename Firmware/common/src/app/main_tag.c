/**
 ******************************************************************************
 * @file    main_tag.c
 * @brief   TAG main loop: ranging, telemetry, host commands, health.
 *
 * Loop order: poll the level-held DW1000 IRQ line, run the ranging state
 * machine, react to radio faults, publish telemetry, execute host commands,
 * then feed the watchdog only if the ranging made progress.
 ******************************************************************************
 */

#include "dw1000_hw.h"
#include "tag_ranging.h"
#include "telemetry.h"
#include "uart_tx.h"
#include "uwb_cmd.h"
#include "uwb_health.h"
#include "uwb_platform.h"
#include "uwb_settings.h"

#include <zephyr/kernel.h>

#define TAG_STATS_PERIOD_MS        1000U
#define TAG_DEVICE_INFO_PERIOD_MS  10000U
#define TAG_ENV_PERIOD_CHECKS      10U    /* temperature every 10 config checks */
#define TAG_DIAG_ANCHORS_PER_SEC   2U
#define TAG_MEAS_PER_LOOP          4U
#define TAG_FAULT_BLINK_MS         150U
#define TAG_LINK_HOLD_MS           1000U
#define TAG_HOST_HOLD_MS           2500U

/* Platform failure before anything else works: blink forever. The watchdog
 * is not running yet, so this is the only state that does not self-reset. */
static void platform_fault_forever(void)
{
    while (true) {
        if (uwb_platform_fault_led_available()) {
            uwb_platform_fault_led_toggle();
        } else {
            uwb_platform_led_toggle();
        }
        k_msleep(TAG_FAULT_BLINK_MS);
    }
}

static uint8_t recover(uint8_t cause)
{
    if (uwb_health_recover_radio(Tag_RecoverRadio, cause) != 0)
        return 0U;
    uwb_platform_led_set(false);
    uwb_platform_fault_led_set(false);
    (void)uwb_settings_apply_calibration();
    return 1U;
}

int main(void)
{
    if (uwb_platform_init() != 0) {
        platform_fault_forever();
    }
    uwb_platform_led_set(false);
    uwb_platform_fault_led_set(false);
    uwb_platform_host_led_set(false);
    MCU_TimerInit();
    (void)uwb_health_init();

    if (UART_TX_Init() != 0) {
        platform_fault_forever();
    }
    uwb_cmd_init();

    /* Stored settings override the compile-time defaults before the radio is
     * configured; without stored settings nothing changes. */
    (void)uwb_settings_load();
    Tag_EnableMeasurementQueue((Telem_GetFeatures() & TELEM_FEATURE_RANGE_MEAS) != 0U);

    uint8_t radio_ok = (Tag_Init() == 0) ? 1U : recover(UWB_FAULT_INIT);
    if (radio_ok)
        (void)uwb_settings_apply_calibration();
    DW1000_EnableIRQ();

    Telem_SendDeviceInfo();
    Telem_SendInfo();

    uint32_t spi_errors_seen = dw1000_spi_error_count;
    uint32_t last_cycle_seen = tag_cycle_count;
    uint32_t last_stats_ms = uwb_platform_time_ms();
    uint32_t last_device_info_ms = last_stats_ms;
    uint32_t last_blink_ms = last_stats_ms;
    uint32_t last_cycle_count = tag_cycle_count;
    uint32_t last_ok_count = response_ok_count;
    uint32_t link_ok_seen = response_ok_count;
    uint32_t last_link_ms = last_stats_ms;
    uint32_t host_frames_seen = uwb_cmd_stats.frames_ok;
    uint32_t last_host_ms = last_stats_ms;
    uint32_t config_checks = 0U;
    uint8_t diag_anchor = 0U;

    while (true) {
        const uint32_t now = uwb_platform_time_ms();
        (void)uwb_platform_time_us64();   /* keeps the 64-bit clock extended */

        /* IRQ is level-held by the DW1000; polling closes the edge race. */
        if (DW1000_IrqLineActive() != 0U) {
            dw1000_irq_flag = 1U;
        }

        if (radio_ok) {
            uwb_platform_fault_led_set(false);
            Tag_Task();

            if (dw1000_spi_error_count != spi_errors_seen) {
                /* Never publish data read through a failed SPI transfer:
                 * Tag_RecoverRadio() invalidates every anchor. */
                radio_ok = recover(UWB_FAULT_SPI);
                spi_errors_seen = dw1000_spi_error_count;
            } else if (Tag_IsIdle() && uwb_health_config_check_due()) {
                /* Brown-out/reset of the DW1000 restores register defaults. */
                const uint32_t mismatch = DW1000_VerifyConfig();
                uwb_health_note_config_check(mismatch);
                if (mismatch != 0U) {
                    radio_ok = recover(UWB_FAULT_CONFIG);
                } else if ((++config_checks % TAG_ENV_PERIOD_CHECKS) == 0U) {
                    uint8_t temp_raw = 0U;
                    uint8_t vbat_raw = 0U;
                    DW1000_ReadTempVbatRaw(&temp_raw, &vbat_raw);
                    Telem_SetEnvironment(temp_raw, vbat_raw);
                }
                spi_errors_seen = dw1000_spi_error_count;
            }
        } else {
            if (uwb_platform_fault_led_available()) {
                uwb_platform_led_set(false);
                uwb_platform_fault_led_set(true);
            } else if ((uint32_t)(now - last_blink_ms) >= TAG_FAULT_BLINK_MS) {
                uwb_platform_led_toggle();
                last_blink_ms = now;
            }
            if (uwb_health_hold_retry_due()) {
                radio_ok = recover(UWB_FAULT_INIT);
                spi_errors_seen = dw1000_spi_error_count;
            }
        }

        /* The link LED is a state, not a free-running activity blink: it is
         * on only while successful UWB exchanges are arriving. */
        if (radio_ok && response_ok_count != link_ok_seen) {
            link_ok_seen = response_ok_count;
            last_link_ms = now;
            uwb_platform_led_set(true);
        } else if (radio_ok && (uint32_t)(now - last_link_ms) >= TAG_LINK_HOLD_MS) {
            uwb_platform_led_set(false);
        }

        /* Progress = a completed cycle, or an intentional pause. */
        if (tag_cycle_count != last_cycle_seen || Tag_IsPaused()) {
            last_cycle_seen = tag_cycle_count;
            uwb_health_note_progress();
        }

        const uint8_t features = Telem_GetFeatures();

        if (tag_cycle_ready != 0U) {
            tag_cycle_ready = 0U;
            if ((features & TELEM_FEATURE_RANGE_SNAPSHOT) != 0U) {
                TagCycleSnapshot_t snapshot;
                Tag_GetSnapshot(&snapshot);
                Telem_SendRangeCycle(&snapshot);
            }
        }

        TagMeasurement_t measurement;
        for (uint8_t k = 0U; k < TAG_MEAS_PER_LOOP && Tag_PopMeasurement(&measurement); k++) {
            Telem_SendRangeMeas(&measurement);
        }

        TagAnchorInfo_t anchor_info;
        if (Tag_TakeAnchorInfo(&anchor_info)) {
            Telem_SendAnchorInfo(&anchor_info);
        }

        if ((uint32_t)(now - last_stats_ms) >= TAG_STATS_PERIOD_MS) {
            const uint32_t elapsed = now - last_stats_ms;
            const uint32_t cycles = tag_cycle_count - last_cycle_count;
            const uint32_t successes = response_ok_count - last_ok_count;

            Telem_SendStats((uint16_t)((cycles * 1000U) / elapsed),
                            (uint16_t)((successes * 1000U) / elapsed));
            Telem_SendInfo();
            if ((features & TELEM_FEATURE_DIAG) != 0U) {
                Telem_SendDiagSystem();
                for (uint8_t k = 0U; k < TAG_DIAG_ANCHORS_PER_SEC; k++) {
                    Telem_SendDiagAnchor(diag_anchor);
                    diag_anchor = (uint8_t)((diag_anchor + 1U) % TAG_NUM_ANCHORS);
                }
            }
            last_stats_ms = now;
            last_cycle_count = tag_cycle_count;
            last_ok_count = response_ok_count;
        }

        if ((uint32_t)(now - last_device_info_ms) >= TAG_DEVICE_INFO_PERIOD_MS) {
            Telem_SendDeviceInfo();
            last_device_info_ms = now;
        }

        uwb_cmd_poll();

        /* On the DevKit the blue LED proves a bidirectional GUI link. The GUI
         * sends a PING heartbeat; opening a COM port alone does not light it. */
        if (uwb_cmd_stats.frames_ok != host_frames_seen) {
            host_frames_seen = uwb_cmd_stats.frames_ok;
            last_host_ms = now;
            uwb_platform_host_led_set(true);
        } else if ((uint32_t)(now - last_host_ms) >= TAG_HOST_HOLD_MS) {
            uwb_platform_host_led_set(false);
        }
        uwb_health_service();
        k_busy_wait(10U);
    }

    return 0;
}
