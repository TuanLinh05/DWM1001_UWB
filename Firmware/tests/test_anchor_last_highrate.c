/* Real A8 responder with the 222 Hz full-mask schedule. Register simulation
 * checks state/interval correctness; it does not benchmark MCU execution. */
#include <stdio.h>
#include <string.h>
#include "dw1000_sim.h"
#include "uwb_health.h"
#include "../common/src/ranging/anchor_ranging.c"

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

UwbHealth_t uwb_health;

static void poll_rx(uint8_t txn, uint8_t mask, uint8_t info, uint64_t t2)
{
    const int rank = uwb_frame_burst_slot(mask, ANCHOR_ADDR);
    const uint16_t slot = info ? 414U : 350U;
    const UwbPoll_t poll = {
        .version = UWB_FRAME_V3, .txn = txn, .anchor_mask = mask, .info_id = info,
        .base_uus = 550U, .slot_uus = slot,
        .final_uus = (uint16_t)(550U + (rank >= 0 ? rank : 7) * slot + 450U),
    };
    uint8_t frame[UWB_FRAME_MAX_RX_LEN];
    const uint16_t len = uwb_frame_build_poll(frame, 0U, DW_PAN_ID,
                                             UWB_FRAME_BROADCAST, TAG_ADDR, &poll);
    sim_deliver_rx(frame, len, t2, 0U);
    Anchor_Task();
}

static uint64_t tx_rmarker(uint64_t t2, uint16_t offset)
{
    const uint64_t dx = (t2 + (uint64_t)offset * UWB_UUS_TO_DWT) & 0xFFFFFFFE00ULL;
    return (dx + DW1000_ActiveTxAntennaDelay()) & UWB_TS40_MASK;
}

static void final_rx(uint8_t txn, uint64_t t6)
{
    const UwbFinal_t fin = { .version = UWB_FRAME_V3, .txn = txn };
    uint8_t frame[UWB_FRAME_MAX_RX_LEN];
    const uint16_t len = uwb_frame_build_final(frame, 0U, DW_PAN_ID,
                                              UWB_FRAME_BROADCAST, TAG_ADDR, &fin);
    sim_deliver_rx(frame, len, t6, 0U);
}

static int fresh(void)
{
    sim_reset();
    CHECK(ANCHOR_ADDR == 8U && DW_SPI_FAST_MHZ == 16U);
    CHECK(Anchor_Init() == 0);
    DW1000_EnableIRQ();
    Anchor_StartListening();
    return 0;
}

static int test_last_slot_coalesced_and_next_rb(void)
{
    CHECK(fresh() == 0);
    const uint64_t t2 = UWB_TS40_MASK - 1000000ULL;
    poll_rx(0xFEU, 0xFFU, 0U, t2);
    CHECK(s_state == ANCHOR_STATE_TX_BURST_RESP);
    CHECK(sim_last_ctrl() == (DW_TXSTRT_BIT | DW_TXDLYS_BIT | DW_WAIT4RESP_BIT));
    CHECK((sim_read_u32(DW_REG_ACK_RESP_T, 0U) & 0xFFFFFU)
          == 450U - ANCHOR_BURST_FINAL_LEAD_UUS - ANCHOR_BURST_RESP_TAIL_MAX_UUS);
    const uint64_t t3 = tx_rmarker(t2, 3000U);
    const uint64_t t6 = (t2 + 3450ULL * UWB_UUS_TO_DWT) & UWB_TS40_MASK;
    sim_complete_tx(t3);
    final_rx(0xFEU, t6); /* Hardware latched FINAL before software sees TXFRS. */
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_RX_WAIT && s_burst_prev_valid);
    CHECK(anchor_stats.burst_finals == 1U);
    CHECK(anchor_tx_prediction_error_dtu == 0);
    poll_rx(0xFFU, 0xFFU, 8U, (t2 + 450000000ULL) & UWB_TS40_MASK);
    UwbResp_t resp;
    CHECK(uwb_frame_parse_resp(sim_tx_frame, sim_tx_len + UWB_FRAME_FCS_LEN, &resp));
    CHECK(resp.prev_txn == 0xFEU && (resp.anchor_status & UWB_ANCHOR_ST_PREV_RB));
    CHECK(resp.prev_rb_ticks == (uint32_t)((t6 - t3) & UWB_TS40_MASK));
    CHECK(resp.tlv_len == 34U);
    CHECK(sim_tx_len + UWB_FRAME_FCS_LEN <= 64U);
    return 0;
}

static int test_timeout_rank_and_recovery(void)
{
    CHECK(fresh() == 0);
    uint64_t t2 = 2000000000ULL;
    poll_rx(1U, 0xFFU, 0U, t2);
    sim_complete_tx(tx_rmarker(t2, 3000U));
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_WAIT_BURST_FINAL);
    final_rx(0U, t2 + 3450ULL * UWB_UUS_TO_DWT); /* wrong transaction */
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_WAIT_BURST_FINAL && !s_burst_prev_valid);
    sim_us += (s_burst_final_timeout_ms + 1U) * 1000U;
    Anchor_Task();
    CHECK(s_state == ANCHOR_STATE_RX_WAIT && anchor_stats.burst_final_missed == 1U);
    t2 += 450000000ULL;
    poll_rx(2U, 0x80U, 0U, t2); /* A8 alone moves to slot zero. */
    CHECK(s_state == ANCHOR_STATE_TX_BURST_RESP);
    UwbResp_t resp;
    CHECK(uwb_frame_parse_resp(sim_tx_frame, sim_tx_len + UWB_FRAME_FCS_LEN, &resp));
    CHECK(!(resp.anchor_status & UWB_ANCHOR_ST_PREV_RB));
    sim_complete_tx(tx_rmarker(t2, 550U));
    final_rx(2U, t2 + 1000ULL * UWB_UUS_TO_DWT);
    Anchor_Task();
    CHECK(s_burst_prev_valid);
    CHECK(Anchor_RecoverRadio() == 0 && !s_burst_prev_valid);
    t2 += 450000000ULL;
    sim_force_hpdwarn = 1;
    poll_rx(3U, 0x80U, 0U, t2);
    CHECK(s_state == ANCHOR_STATE_RX_WAIT && anchor_stats.delayed_tx_late == 1U);
    t2 += 450000000ULL;
    poll_rx(4U, 0x80U, 0U, t2);
    CHECK(uwb_frame_parse_resp(sim_tx_frame, sim_tx_len + UWB_FRAME_FCS_LEN, &resp));
    CHECK(resp.anchor_status & UWB_ANCHOR_ST_LATE);
    CHECK(!(resp.anchor_status & UWB_ANCHOR_ST_PREV_RB));
    return 0;
}

int main(void)
{
    if (test_last_slot_coalesced_and_next_rb() || test_timeout_rank_and_recovery())
        return 1;
    puts("A8 high-rate tests passed (450 UUS margin, hardware auto RX, coalesced FINAL, "
         "40-bit wrap, info TLV, previous Rb, timeout, sparse mask, recovery)");
    return 0;
}
