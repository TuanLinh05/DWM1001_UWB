#include "anchor_ranging.h"
#include "dw1000_hw.h"
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

static void watchdog_feed(void)
{
#if UWB_WATCHDOG_ENABLED
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

static int run_anchor(void)
{
    if (Anchor_Init() != 0) {
        return -EIO;
    }

    DW1000_EnableIRQ();
    Anchor_StartListening();
    const uint32_t spi_errors_at_start = dw1000_spi_error_count;

    while (true) {
        /* IRQ is level-held by DW1000; polling closes the edge-race window. */
        if (DW1000_IrqLineActive() != 0U) {
            dw1000_irq_flag = 1U;
        }
        Anchor_Task();
        if (dw1000_spi_error_count != spi_errors_at_start) {
            return -EIO;
        }
        watchdog_feed();
        k_busy_wait(10U);
    }
}

int main(void)
{
    if (uwb_platform_init() != 0) {
        fatal_blink();
    }

    MCU_TimerInit();
    if (watchdog_start() != 0) {
        fatal_blink();
    }

    if (run_anchor() != 0) {
        fatal_blink();
    }

    return 0;
}
