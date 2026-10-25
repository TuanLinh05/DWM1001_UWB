/* Real periodic encoders + idle reservation under a persistently late POLL.
 * Radio exchanges themselves are covered by test_tag_burst. */
#include <stdio.h>
#include <string.h>
#include "dw1000_sim.h"
#include "uart_tx.h"
#include "uwb_health.h"
#include "uwb_settings.h"
#include "../common/ranging/tag_ranging.c"
#include "telemetry.h"

#define CHECK(c) do { if (!(c)) { fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#c); return 1; } } while(0)
UwbHealth_t uwb_health;
uint8_t uwb_settings_status;
volatile uint32_t uart_tx_overflow_count, uart_rx_overflow_count;
volatile uint16_t uart_tx_high_water;
static unsigned s_types[256], s_anchors[8], s_bad_frame;
static uint16_t s_pending;

uint32_t UART_TX_Write(const uint8_t *b, uint16_t length)
{
    const uint16_t payload_len = (uint16_t)(b[4] | ((uint16_t)b[5] << 8));
    if (length != TELEM_HDR_LEN + payload_len + TELEM_CRC_LEN
        || telem_crc16_ccitt(b + 2, (uint16_t)(length - 4))
           != (uint16_t)(b[length-2] | ((uint16_t)b[length-1] << 8)))
        s_bad_frame++;
    s_types[b[3]]++;
    if (b[3] == TELEM_TYPE_RX_QUALITY)
    {
        if (payload_len != 254U || length != 270U
            || uwb_get_u16(b + TELEM_HDR_LEN + 30U + 20U) != 0x8000U
            || uwb_get_u16(b + TELEM_HDR_LEN + 30U + 22U) != 0x8000U)
            s_bad_frame++;
    }
    if (b[3] == TELEM_TYPE_DIAG_ANCHOR && b[TELEM_HDR_LEN+1] >= 1U
        && b[TELEM_HDR_LEN+1] <= 8U)
        s_anchors[b[TELEM_HDR_LEN+1]-1U]++;
    if (b[3] == TELEM_TYPE_DIAG_SYSTEM)
    {
        if (payload_len != 208U || uwb_get_u32(b + TELEM_HDR_LEN + 180) != 77U
            || uwb_get_u32(b + TELEM_HDR_LEN + 184) != 123U
            || uwb_get_u32(b + TELEM_HDR_LEN + 188) != 88U
            || uwb_get_u32(b + TELEM_HDR_LEN + 192) != tag_burst_maintenance_grants)
            s_bad_frame++;
    }
    return length;
}
uint16_t UART_TX_Pending(void) { return s_pending; }

static void overloaded_idle_cycle(void)
{
    s_state = TAG_STATE_IDLE;
    tag_cycle_count++;
    s_burst_maintenance_claimed = 0U; /* new radio cycle, as burst_start_cycle */
    s_burst_maintenance_until_us = 0U;
    s_burst_last_poll_us = sim_us - 10000U;
    s_burst_final_done_us = sim_us - 1000U;
}

int main(void)
{
    sim_reset();
    CHECK(Tag_Init() == 0);
    CHECK(Telem_SetFeatures(TELEM_FEATURE_DIAG | TELEM_FEATURE_RANGE_SNAPSHOT) == 0);
    tag_burst_publish_step_max_us = 77U;
    tag_burst_post_final_max_us = 123U;
    tag_burst_telemetry_max_us = 88U;
    TelemPeriodic_t schedule;
    Telem_InitPeriodic(&schedule, uwb_platform_time_ms());
    for (unsigned cycle = 0; cycle < 900; cycle++)
    {
        sim_us += 3100U;
        overloaded_idle_cycle();
        CHECK(Tag_IdleBudgetUs() == 0U);
        Telem_ServicePeriodic(&schedule, uwb_platform_time_ms());
        const uint32_t grants = tag_burst_maintenance_grants;
        Telem_ServicePeriodic(&schedule, uwb_platform_time_ms());
        CHECK(tag_burst_maintenance_grants == grants); /* no second heavy job */
    }
    CHECK(!s_bad_frame);
    CHECK(s_types[TELEM_TYPE_STATS] == 2U && s_types[TELEM_TYPE_INFO] == 2U);
    CHECK(s_types[TELEM_TYPE_DIAG_SYSTEM] == 2U);
    for (unsigned i = 0; i < 8; i++)
        CHECK(s_anchors[i] == 2U); /* all eight within each round despite 0 slack */

    overloaded_idle_cycle();
    Telem_InitPeriodic(&schedule, uwb_platform_time_ms() - 1000U);
    CHECK(Tag_ClaimMaintenanceSlot(500U)); /* config check takes this cycle */
    const unsigned before = s_types[TELEM_TYPE_STATS];
    Telem_ServicePeriodic(&schedule, uwb_platform_time_ms());
    CHECK(s_types[TELEM_TYPE_STATS] == before);
    sim_us += 3100U;
    overloaded_idle_cycle();
    Telem_ServicePeriodic(&schedule, uwb_platform_time_ms());
    CHECK(s_types[TELEM_TYPE_STATS] == before + 1U);
    CHECK(Telem_SetFeatures(TELEM_FEATURE_RANGE_SNAPSHOT) == 0);
    sim_us += 3100U;
    overloaded_idle_cycle();
    Telem_ServicePeriodic(&schedule, uwb_platform_time_ms());
    CHECK(schedule.phase == 0U && s_types[TELEM_TYPE_DIAG_SYSTEM] == 2U);

    overloaded_idle_cycle();
    Telem_InitPeriodic(&schedule, UINT32_MAX - 499U);
    Telem_ServicePeriodic(&schedule, 500U); /* millisecond uptime wrap */
    CHECK(s_types[TELEM_TYPE_STATS] == before + 2U);
    overloaded_idle_cycle();
    Telem_InitPeriodic(&schedule, uwb_platform_time_ms() - 1000U);
    s_pending = UART_TX_BUF_SIZE - 100U;
    const unsigned blocked = s_types[TELEM_TYPE_STATS];
    const uint32_t grants = tag_burst_maintenance_grants;
    Telem_ServicePeriodic(&schedule, uwb_platform_time_ms());
    CHECK(s_types[TELEM_TYPE_STATS] == blocked);
    CHECK(tag_burst_maintenance_grants == grants);
    s_pending = 0U;
    Telem_ServicePeriodic(&schedule, uwb_platform_time_ms());
    CHECK(s_types[TELEM_TYPE_STATS] == blocked + 1U); /* resume without lost work */
    CHECK(Telem_SetFeatures(TELEM_FEATURE_DIAG) == 0);
    TagBurstRecord_t rec = { .count = 8U, .t_us = 10000000U, .cycle_seq = 100U };
    for (unsigned i = 0; i < 8; i++)
        rec.anchor[i].anchor_id = (uint8_t)(i+1U);
    const uint32_t spi_calls = sim_spi_calls;
    Telem_SendRxQuality(&rec);
    CHECK(telem_quality_frames == 1U && s_types[TELEM_TYPE_RX_QUALITY] == 1U);
    Telem_SendRxQuality(&rec); /* same sample is not emitted twice */
    CHECK(telem_quality_frames == 1U);
    rec.t_us += 99999U;
    Telem_SendRxQuality(&rec);
    CHECK(telem_quality_frames == 1U);
    rec.t_us++;
    s_pending = UART_TX_BUF_SIZE - 1U;
    Telem_SendRxQuality(&rec);
    CHECK(telem_quality_deferred == 1U && telem_quality_frames == 1U);
    s_pending = 0U;
    Telem_SendRxQuality(&rec); /* retry deferred diagnostics without losing ranges */
    CHECK(telem_quality_frames == 2U);
    CHECK(Telem_SetFeatures(0U) == 0);
    rec.t_us += 100000U;
    Telem_SendRxQuality(&rec);
    CHECK(telem_quality_frames == 2U);
    CHECK(Telem_SetFeatures(TELEM_FEATURE_DIAG) == 0);
    uwb_health.boot_id++;
    rec.t_us = 10U; /* restart emits immediately and cannot reuse old interval */
    Telem_SendRxQuality(&rec);
    CHECK(telem_quality_frames == 3U && sim_spi_calls == spi_calls);
    CHECK(!s_bad_frame);
    puts("periodic telemetry passed (zero-slack load, all 8 DIAG, CRC/perf fields, bounded reservations, features, uptime wrap)");
    return 0;
}
