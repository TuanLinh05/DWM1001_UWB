/* Run the real anchor responder against the DW1000 register simulator. */
#include <stdio.h>
#include <string.h>

#include "dw1000_sim.h"
#include "uwb_health.h"
#include "../common/src/ranging/anchor_ranging.c"

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

UwbHealth_t uwb_health;

static void deliver_poll(uint8_t version, uint8_t txn, uint8_t flags, uint16_t dst,
                         uint64_t rx_ts, uint64_t extra)
{
    uint8_t f[UWB_FRAME_MAX_RX_LEN];
    const UwbPoll_t p = { .version = version, .txn = txn, .flags = flags };
    const uint16_t len = uwb_frame_build_poll(f, txn, DW_PAN_ID, dst, TAG_ADDR, &p);
    sim_deliver_rx(f, len, rx_ts, extra);
}

static void deliver_final(uint8_t version, uint8_t txn, uint64_t rx_ts)
{
    uint8_t f[UWB_FRAME_MAX_RX_LEN];
    const UwbFinal_t fin = { .version = version, .txn = txn };
    const uint16_t len = uwb_frame_build_final(f, 0U, DW_PAN_ID, ANCHOR_ADDR, TAG_ADDR, &fin);
    sim_deliver_rx(f, len, rx_ts, 0U);
}

static uint64_t expected_resp_rmarker(uint64_t poll_rx)
{
    const uint64_t resp_tx = (poll_rx + REPLY_DELAY_TICKS) & 0xFFFFFFFE00ULL;
    return (resp_tx + DW1000_ActiveTxAntennaDelay()) & UWB_TS40_MASK;
}

static int fresh_anchor(void)
{
    sim_reset();
    if (Anchor_Init() != 0)
        return -1;
    DW1000_EnableIRQ();
    Anchor_StartListening();
    return 0;
}

static int test_v1_exchange(void)
{
    UwbFrameHeader_t hdr;
    UwbResp_t resp;
    UwbReport_t rep;

    CHECK(fresh_anchor() == 0);
    const uint64_t poll_rx = 2000000000ULL;
    deliver_poll(UWB_FRAME_V1, 0x10U, 0U, ANCHOR_ADDR, poll_rx, 0U);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_TX_RESPOND);
    CHECK(sim_last_ctrl() == (DW_TXSTRT_BIT | DW_TXDLYS_BIT | DW_WAIT4RESP_BIT));

    const uint16_t rx_len = (uint16_t)(sim_tx_len + UWB_FRAME_FCS_LEN);
    CHECK(uwb_frame_parse_header(sim_tx_frame, rx_len, DW_PAN_ID, &hdr));
    CHECK(hdr.func == FRAME_RESP_FUNC && hdr.dst == TAG_ADDR && hdr.src == ANCHOR_ADDR);
    CHECK(uwb_frame_parse_resp(sim_tx_frame, rx_len, &resp) && resp.version == UWB_FRAME_V1);
    const uint64_t rmarker = expected_resp_rmarker(poll_rx);
    CHECK(resp.reply_ticks == (uint32_t)(rmarker - poll_rx));

    /* RESP sent at exactly the predicted RMARKER. */
    const uint32_t rx_enables = sim_rx_enable_count;
    sim_complete_tx(rmarker);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_WAIT_FINAL);
    CHECK(anchor_tx_prediction_error_dtu == 0);
    CHECK(sim_rx_enable_count == rx_enables);   /* WAIT4RESP */

    const uint64_t final_rx = rmarker + 30000000ULL;
    deliver_final(UWB_FRAME_V1, 0U, final_rx);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_TX_REPORT);
    CHECK(uwb_frame_parse_report(sim_tx_frame, (uint16_t)(sim_tx_len + UWB_FRAME_FCS_LEN), &rep));
    CHECK(rep.version == UWB_FRAME_V1 && rep.round_ticks == 30000000U);

    sim_complete_tx(final_rx + 1000U);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_RX_WAIT);
    CHECK(Anchor_InQuietWindow() == 1U);
    CHECK(anchor_stats.reports_sent == 1U);
    return 0;
}

static int test_v2_exchange_txn_and_info(void)
{
    UwbResp_t resp;
    UwbReport_t rep;

    CHECK(fresh_anchor() == 0);
    const uint64_t poll_rx = 3000000000ULL;
    sim_set_rx_quality(210U, 900U, 800U, 700U, 4000U, 30U);
    deliver_poll(UWB_FRAME_V2, 0x42U, UWB_POLL_FLAG_REQ_INFO, ANCHOR_ADDR, poll_rx, 0U);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_TX_RESPOND);
    const uint16_t rx_len = (uint16_t)(sim_tx_len + UWB_FRAME_FCS_LEN);
    CHECK(uwb_frame_parse_resp(sim_tx_frame, rx_len, &resp));
    CHECK(resp.version == UWB_FRAME_V2 && resp.txn == 0x42U);
    CHECK(resp.tlv_len == 3U * 2U + UWB_TLV_ANCHOR_POSITION_LEN
                          + UWB_TLV_ANCHOR_BUILD_LEN + UWB_TLV_ANCHOR_CONFIG_LEN);
    CHECK(uwb_frame_find_tlv(resp.tlv, resp.tlv_len, UWB_TLV_ANCHOR_BUILD,
                             UWB_TLV_ANCHOR_BUILD_LEN) != NULL);
    CHECK(uwb_frame_find_tlv(resp.tlv, resp.tlv_len, UWB_TLV_ANCHOR_CONFIG,
                             UWB_TLV_ANCHOR_CONFIG_LEN) != NULL);
    CHECK(anchor_stats.info_replies == 1U && anchor_stats.polls_v2 == 1U);

    const uint64_t rmarker = expected_resp_rmarker(poll_rx);
    sim_complete_tx(rmarker);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_WAIT_FINAL);

    /* F4: a FINAL of another transaction is not answered. */
    const uint32_t tx_before = sim_tx_count;
    deliver_final(UWB_FRAME_V2, 0x41U, rmarker + 1000U);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_WAIT_FINAL);
    CHECK(sim_tx_count == tx_before && anchor_stats.txn_mismatch == 1U);

    deliver_final(UWB_FRAME_V2, 0x42U, rmarker + 25000000ULL);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_TX_REPORT);
    CHECK(uwb_frame_parse_report(sim_tx_frame, (uint16_t)(sim_tx_len + UWB_FRAME_FCS_LEN), &rep));
    CHECK(rep.version == UWB_FRAME_V2 && rep.txn == 0x42U && rep.round_ticks == 25000000U);
    CHECK(rep.final_fp_cdbm != INT16_MIN && rep.final_rx_cdbm != INT16_MIN);
    return 0;
}

static int test_errors_and_foreign_traffic(void)
{
    CHECK(fresh_anchor() == 0);

    /* RX error: soft reset, stay listening. */
    deliver_poll(UWB_FRAME_V2, 1U, 0U, ANCHOR_ADDR, 1000U, DW_RXFCE_BIT);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_RX_WAIT);
    CHECK(anchor_stats.rx_errors == 1U && sim_rx_soft_resets == 1U);

    /* POLL for another anchor: no reply. */
    const uint32_t tx_before = sim_tx_count;
    deliver_poll(UWB_FRAME_V2, 2U, 0U, (uint16_t)(ANCHOR_ADDR + 1U), 2000U, 0U);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_RX_WAIT && sim_tx_count == tx_before);

    /* Late delayed TX (HPDWARN): immediate recovery, no stuck TX state. */
    sim_force_hpdwarn = 1;
    deliver_poll(UWB_FRAME_V2, 3U, 0U, ANCHOR_ADDR, 3000U, 0U);
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_RX_WAIT);
    CHECK(anchor_stats.delayed_tx_late == 1U);

    /* FINAL never comes: back to RX_WAIT after the timeout. */
    deliver_poll(UWB_FRAME_V2, 4U, 0U, ANCHOR_ADDR, 4000000U, 0U);
    Anchor_Task();
    sim_complete_tx(expected_resp_rmarker(4000000U));
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_WAIT_FINAL);
    Anchor_ConsumeQuietWindow();
    sim_us += (ANCHOR_FINAL_TIMEOUT_MS + 1U) * 1000U;
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_RX_WAIT && anchor_stats.final_timeouts == 1U);
    CHECK(Anchor_InQuietWindow() == 1U);
    return 0;
}

int main(void)
{
    if (test_v1_exchange() || test_v2_exchange_txn_and_info()
        || test_errors_and_foreign_traffic())
        return 1;
    puts("anchor responder tests passed (v1/v2, txn, TLV, WAIT4RESP, errors, timeouts)");
    return 0;
}
