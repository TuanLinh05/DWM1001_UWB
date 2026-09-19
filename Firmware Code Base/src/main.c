#include "anchor_ranging.h"
#include "dw1000_hw.h"
#include "hardware_test.h"
#include "tag_ranging.h"
#include "telemetry.h"
#include "uart_tx.h"
#include "uwb_platform.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

#define WATCHDOG_NODE DT_ALIAS(watchdog0)

static const struct device *const s_watchdog = DEVICE_DT_GET(WATCHDOG_NODE);
static int s_watchdog_channel = -1;

static int watchdog_start(void)
{
#ifdef CONFIG_UWB_WATCHDOG
    const struct wdt_timeout_cfg timeout = {
        .window = {
            .min = 0U,
            .max = CONFIG_UWB_WATCHDOG_TIMEOUT_MS,
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

static void watchdog_feed(void)
{
#ifdef CONFIG_UWB_WATCHDOG
    if (s_watchdog_channel >= 0) {
        (void)wdt_feed(s_watchdog, s_watchdog_channel);
    }
#endif
}

static void fatal_blink(void)
{
    while (true) {
        uwb_platform_led_toggle();
        watchdog_feed();
        k_msleep(150);
    }
}

#if defined(CONFIG_UWB_ROLE_TAG)
static int run_tag(void)
{
    TagCycleSnapshot_t snapshot;
    uint32_t last_stats_ms;
    uint32_t last_cycle_count;
    uint32_t last_ok_count;

    if (UART_TX_Init() != 0 || Tag_Init() != 0) {
        return -EIO;
    }

    DW1000_EnableIRQ();
    Telem_SendInfo();
    last_stats_ms = uwb_platform_time_ms();
    last_cycle_count = tag_cycle_count;
    last_ok_count = response_ok_count;

    while (true) {
        /* IRQ is level-held by DW1000; polling closes the edge-race window. */
        if (DW1000_IrqLineActive() != 0U) {
            dw1000_irq_flag = 1U;
        }
        Tag_Task();

        if (tag_cycle_ready != 0U) {
            Tag_GetSnapshot(&snapshot);
            tag_cycle_ready = 0U;
            Telem_SendRangeCycle(&snapshot);
        }

        uint32_t now = uwb_platform_time_ms();
        if ((now - last_stats_ms) >= 1000U) {
            uint32_t elapsed = now - last_stats_ms;
            uint32_t cycles = tag_cycle_count - last_cycle_count;
            uint32_t successes = response_ok_count - last_ok_count;
            uint16_t cycle_hz = (uint16_t)((cycles * 1000U) / elapsed);
            uint16_t operation_hz = (uint16_t)((successes * 1000U) / elapsed);

            Telem_SendStats(cycle_hz, operation_hz);
            Telem_SendInfo();
            last_stats_ms = now;
            last_cycle_count = tag_cycle_count;
            last_ok_count = response_ok_count;
        }

        watchdog_feed();
        k_busy_wait(10U);
    }
}
#elif defined(CONFIG_UWB_ROLE_ANCHOR)
static int run_anchor(void)
{
    if (Anchor_Init() != 0) {
        return -EIO;
    }

    DW1000_EnableIRQ();
    Anchor_StartListening();

    while (true) {
        if (DW1000_IrqLineActive() != 0U) {
            dw1000_irq_flag = 1U;
        }
        Anchor_Task();
        watchdog_feed();
        k_busy_wait(10U);
    }
}
#else
static int run_hardware_test(void)
{
    int result = HardwareTest_Run();
    uint32_t blink_ms = result == 0 ? 200U : 1000U;

    while (true) {
        uwb_platform_led_toggle();
        watchdog_feed();
        k_msleep(blink_ms);
    }

    return 0;
}
#endif

int main(void)
{
    if (uwb_platform_init() != 0) {
        fatal_blink();
    }

    MCU_TimerInit();
    if (watchdog_start() != 0) {
        fatal_blink();
    }

#if defined(CONFIG_UWB_ROLE_TAG)
    if (run_tag() != 0) {
        fatal_blink();
    }
#elif defined(CONFIG_UWB_ROLE_ANCHOR)
    if (run_anchor() != 0) {
        fatal_blink();
    }
#else
    (void)run_hardware_test();
#endif

    return 0;
}
