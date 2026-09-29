/*
 * One-to-many (burst) DS-TWR v3: the real TAG state machine against the
 * DW1000 register simulator and eight modelled anchors.
 *
 * Every anchor has its own crystal offset (up to -22 ppm, like the STM32
 * boards measured on 2026-09-26) and clock origin, answers in the slot given
 * by its rank in the POLL mask, and returns the Rb of an exchange in its RESP
 * of the next cycle, exactly like anchor_ranging.c. The TAG must publish each
 * cycle one cycle later, with every range within 2 mm of the truth.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dw1000_sim.h"
#include "../common/src/ranging/tag_ranging.c"

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

#define MASK40         0xFFFFFFFFFFULL
#define ANT_DLY        16436.0
#define TICKS_PER_M    (1.0 / (UWB_DWT_TIME_UNIT_S * UWB_SPEED_OF_LIGHT))

typedef struct {
    double   distance_m;
    double   ppm;           /* anchor clock rate vs the TAG */
    double   origin;        /* anchor clock at TAG time 0 */
    int      answers;       /* hears the POLL and replies */
    int      hears_final;
    int      report_late;   /* LATE bit in its RESP */
    /* anchor_ranging.c state */
    int      prev_valid;
    uint8_t  prev_txn;
    uint32_t prev_rb;
    uint64_t t3;            /* RESP RMARKER, anchor clock */
    uint8_t  open_txn;
    int      open;
} SimAnchor;

typedef struct {
    uint8_t  mask;
    uint16_t base;
    uint16_t slot;
    uint16_t final;
    uint8_t  info_id;
    uint8_t  txn;
    uint64_t dx_final;
} CycleSeen;

static SimAnchor g_anchor[TAG_NUM_ANCHORS];
static double g_t1 = 1.0e10;            /* TAG clock of the next POLL */

static double tof_of(const SimAnchor *a)
{
    return a->distance_m * TICKS_PER_M;
}

static uint64_t to_anchor(const SimAnchor *a, double tag_ticks)
{
    return (uint64_t)llround(a->origin + tag_ticks * (1.0 + a->ppm * 1e-6)) & MASK40;
}

static double from_anchor(const SimAnchor *a, uint64_t anchor_ticks)
{
    return ((double)anchor_ticks - a->origin) / (1.0 + a->ppm * 1e-6);
}

static int32_t true_mm(const SimAnchor *a)
{
    return (int32_t)lround(a->distance_m * 1000.0);
}

static void set_carrier_integrator(double ppm)
{
    /* ratio = CI * UWB_CLOCK_OFFSET_MULT must equal the anchor rate offset. */
    int32_t ci = (int32_t)lround(ppm * 1e-6 / UWB_CLOCK_OFFSET_MULT);
    const uint32_t raw = (uint32_t)ci & 0x1FFFFFU;
    const uint8_t b[3] = { (uint8_t)raw, (uint8_t)(raw >> 8), (uint8_t)(raw >> 16) };
    sim_write_bytes(DW_REG_DRX_CONF, DW_SUB_DRX_CAR_INT, b, 3);
}

static void default_anchors(void)
{
    static const double dist[TAG_NUM_ANCHORS] = { 0.62, 0.80, 0.52, 1.13, 0.91, 1.47, 1.06, 1.03 };
    static const double ppm[TAG_NUM_ANCHORS]  = { -1.5, 0.3, -3.0, 0.4, -15.8, -20.5, -20.1, -21.7 };
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        memset(&g_anchor[i], 0, sizeof(g_anchor[i]));
        g_anchor[i].distance_m = dist[i];
        g_anchor[i].ppm = ppm[i];
        g_anchor[i].origin = 3.0e9 * (double)(i + 1U);
        g_anchor[i].answers = 1;
        g_anchor[i].hears_final = 1;
    }
}

static int fresh_tag(void)
{
    sim_reset();
    if (Tag_Init() != 0)
        return -1;
    memset(s_track, 0, sizeof(s_track));
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        anchor_response_timeout_streak[i] = 0U;
        s_anchor_next_probe_cycle[i] = 0U;
        anchor_probe_count[i] = 0U;
        anchor_burst_late_count[i] = 0U;
        anchor_report_timeout_count[i] = 0U;
        (void)Tag_SetDsCalibration((uint16_t)(i + 1U), 0, 0U);
    }
    tag_burst_final_late_count = 0U;
    const TagBurstTiming_t defaults = {
        UWB_BURST_BASE_UUS, UWB_BURST_SLOT_UUS, UWB_BURST_FINAL_MARGIN_UUS,
        UWB_BURST_GAP_US, UWB_BURST_PERIOD_US,
    };
    if (Tag_SetBurstTiming(&defaults) != 0)
        return -1;
    s_burst_timing = defaults;
    s_burst_timing_pending_valid = 0U;
    s_burst_last_info_ms = uwb_platform_time_ms();   /* no info cycle yet */
    Tag_SetActiveAnchorMask(0xFFU);
    Tag_RequestPause(0U);
    Tag_EnableMeasurementQueue(1U);
    default_anchors();
    g_t1 = 1.0e10;
    return 0;
}

static void deliver_resp(SimAnchor *a, uint16_t id, uint8_t rank, const CycleSeen *cs,
                         uint64_t poll_done_us, const uint8_t *tlv, uint8_t tlv_len)
{
    const double tof = tof_of(a);
    const uint64_t t2 = to_anchor(a, g_t1 + tof);
    const uint64_t reply = ((uint64_t)cs->base + (uint64_t)rank * cs->slot) * UWB_UUS_TO_DWT;
    const uint64_t resp_tx = (t2 + reply) & 0xFFFFFFFE00ULL;
    const uint64_t t3 = (resp_tx + (uint64_t)ANT_DLY) & MASK40;
    const uint64_t t4 = (uint64_t)llround(from_anchor(a, t3) + tof);

    uint8_t status = 0U;
    if (a->prev_valid)
        status |= UWB_ANCHOR_ST_PREV_RB;
    if (a->report_late)
        status |= UWB_ANCHOR_ST_LATE;
    const UwbResp_t r = {
        .version = UWB_FRAME_V3, .txn = cs->txn,
        .reply_ticks = (uint32_t)((t3 - t2) & MASK40),
        .anchor_status = status, .tlv_len = tlv_len, .tlv = tlv,
        .prev_txn = a->prev_txn, .prev_rb_ticks = a->prev_rb,
        .prev_final_fp_cdbm = -8200, .prev_final_rx_cdbm = -7700,
    };
    uint8_t f[UWB_FRAME_MAX_RX_LEN];
    const uint16_t len = uwb_frame_build_resp(f, 0U, DW_PAN_ID, TAG_ADDR, id, &r);

    a->prev_valid = 0;          /* sent once, like the firmware */
    a->t3 = t3;
    a->open_txn = cs->txn;
    a->open = 1;

    sim_us = poll_done_us + uus_to_us((uint32_t)cs->base + (uint32_t)rank * cs->slot) + 60U;
    set_carrier_integrator(a->ppm);
    sim_set_rx_quality(900U, 6000U, 5000U, 4000U, 3000U, 40U);
    sim_deliver_rx(f, len, t4, 0U);
    Tag_Task();
}

static int parse_final_tx(CycleSeen *cs);

/* One complete cycle: POLL, slotted RESPs, delayed FINAL. */
static int run_cycle(CycleSeen *cs)
{
    UwbFrameHeader_t hdr;
    UwbPoll_t poll;

    memset(cs, 0, sizeof(*cs));
    sim_us += 5000U;                                   /* past gap/period */
    Tag_Task();
    CHECK(s_state == TAG_STATE_BURST_TX_POLL);
    CHECK(Tag_TelemetryWindow() == 0U);
    CHECK(sim_last_ctrl() == (DW_TXSTRT_BIT | DW_WAIT4RESP_BIT));
    const uint16_t rx_len = (uint16_t)(sim_tx_len + UWB_FRAME_FCS_LEN);
    CHECK(uwb_frame_parse_header(sim_tx_frame, rx_len, DW_PAN_ID, &hdr));
    CHECK(hdr.func == FRAME_POLL_FUNC && hdr.dst == UWB_FRAME_BROADCAST && hdr.src == TAG_ADDR);
    CHECK(uwb_frame_parse_poll(sim_tx_frame, rx_len, &poll) && poll.version == UWB_FRAME_V3);
    cs->mask = poll.anchor_mask;
    cs->base = poll.base_uus;
    cs->slot = poll.slot_uus;
    cs->final = poll.final_uus;
    cs->info_id = poll.info_id;
    cs->txn = poll.txn;

    sim_complete_tx((uint64_t)llround(g_t1));
    Tag_Task();
    CHECK(s_state == TAG_STATE_BURST_RX);
    const uint64_t poll_done_us = sim_us;

    uint8_t info_tlv[UWB_RESP_V3_TLV_MAX];
    uint8_t info_len = 0U;
    for (uint8_t idx = 0U; idx < TAG_NUM_ANCHORS; idx++)
    {
        SimAnchor *a = &g_anchor[idx];
        const int rank = uwb_frame_burst_slot(cs->mask, (uint16_t)(idx + 1U));
        a->open = 0;
        if (rank < 0)
        {
            a->prev_valid = 0;                         /* left out: drops Rb */
            continue;
        }
        if (!a->answers)
            continue;
        info_len = 0U;
        if (cs->info_id == idx + 1U)
        {
            /* Position TLV only: enough to see the TAG capture it. */
            info_tlv[0] = UWB_TLV_ANCHOR_POSITION;
            info_tlv[1] = UWB_TLV_ANCHOR_POSITION_LEN;
            uwb_put_u32(&info_tlv[2], 1000U * (idx + 1U));
            uwb_put_u32(&info_tlv[6], 2000U);
            uwb_put_u32(&info_tlv[10], 3000U);
            info_len = 14U;
        }
        const uint32_t rx_enables = sim_rx_enable_count;
        deliver_resp(a, (uint16_t)(idx + 1U), (uint8_t)rank, cs, poll_done_us,
                     info_tlv, info_len);
        /* IDLE: the last RESP came in and the FINAL could not keep its time. */
        CHECK(s_state == TAG_STATE_BURST_RX || s_state == TAG_STATE_BURST_TX_FINAL
              || s_state == TAG_STATE_IDLE);
        if (s_state == TAG_STATE_BURST_RX)
        {
            CHECK(sim_rx_enable_count == rx_enables + 1U);  /* re-armed for the next */
        }
        else
        {
            /* Last RESP: FINAL scheduled in the same pass, no re-arm, and the
             * RESP still read out after it. */
            CHECK(sim_rx_enable_count == rx_enables);
            CHECK(s_bc[s_bc_cur].resp[idx].present);
            if (s_state == TAG_STATE_BURST_TX_FINAL)
                CHECK(sim_last_ctrl() == (DW_TXSTRT_BIT | DW_TXDLYS_BIT));
        }
    }

    if (s_state == TAG_STATE_BURST_RX)
    {
        /* Someone stayed silent: the TAG waits for the last slot. */
        sim_us = poll_done_us + s_burst_rx_limit_us - 5U;
        Tag_Task();
        CHECK(s_state == TAG_STATE_BURST_RX);
        sim_us += 10U;
        Tag_Task();
    }
    if (s_state == TAG_STATE_IDLE)
    {
        g_t1 += 4.5e-3 / UWB_DWT_TIME_UNIT_S;          /* FINAL was too late */
        return 0;
    }
    CHECK(s_state == TAG_STATE_BURST_TX_FINAL);
    CHECK(sim_last_ctrl() == (DW_TXSTRT_BIT | DW_TXDLYS_BIT));
    CHECK(parse_final_tx(cs) == 0);

    /* FINAL leaves at DX_TIME (+ TX antenna delay). */
    const uint64_t t5 = (cs->dx_final + (uint64_t)ANT_DLY) & MASK40;
    sim_complete_tx(t5);
    Tag_Task();
    CHECK(s_state == TAG_STATE_IDLE);
    CHECK(Tag_TelemetryWindow() == 1U);

    for (uint8_t idx = 0U; idx < TAG_NUM_ANCHORS; idx++)
    {
        SimAnchor *a = &g_anchor[idx];
        if (!a->open || !a->hears_final)
            continue;
        const uint64_t t6 = to_anchor(a, (double)t5 + tof_of(a));
        a->prev_valid = 1;
        a->prev_txn = a->open_txn;
        a->prev_rb = (uint32_t)((t6 - a->t3) & MASK40);
    }
    g_t1 += 4.5e-3 / UWB_DWT_TIME_UNIT_S;              /* next POLL ~4.5 ms later */
    return 0;
}

static int parse_final_tx(CycleSeen *cs)
{
    UwbFrameHeader_t hdr;
    UwbFinal_t fin;
    const uint16_t rx_len = (uint16_t)(sim_tx_len + UWB_FRAME_FCS_LEN);

    CHECK(uwb_frame_parse_header(sim_tx_frame, rx_len, DW_PAN_ID, &hdr));
    CHECK(hdr.func == FRAME_FINAL_FUNC && hdr.dst == UWB_FRAME_BROADCAST);
    CHECK(uwb_frame_parse_final(sim_tx_frame, rx_len, &fin));
    CHECK(fin.version == UWB_FRAME_V3 && fin.txn == cs->txn);
    uint64_t dx = 0U;
    for (int i = 4; i >= 0; i--)
        dx = (dx << 8) | sim_reg[DW_REG_DX_TIME][i];
    const uint64_t expected = ((uint64_t)llround(g_t1)
                               + (uint64_t)cs->final * UWB_UUS_TO_DWT) & 0xFFFFFFFE00ULL;
    CHECK(dx == expected);
    cs->dx_final = dx;
    return 0;
}

static int pop_record(TagBurstRecord_t *rec)
{
    CHECK(Tag_PopBurst(rec) == 1U);
    return 0;
}

static const TagBurstAnchor_t *entry_for(const TagBurstRecord_t *rec, uint8_t id)
{
    for (uint8_t k = 0U; k < rec->count; k++)
        if (rec->anchor[k].anchor_id == id)
            return &rec->anchor[k];
    return NULL;
}

static int test_eight_anchors_one_cycle_late(void)
{
    CycleSeen c1, c2;
    TagBurstRecord_t rec;

    CHECK(fresh_tag() == 0);
    CHECK(Tag_BurstMode() == 1U && Tag_FrameVersion() == UWB_FRAME_V3);
    CHECK(run_cycle(&c1) == 0);
    CHECK(c1.mask == 0xFFU && c1.info_id == 0U);
    CHECK(c1.base == UWB_BURST_BASE_UUS && c1.slot == UWB_BURST_SLOT_UUS);
    CHECK(c1.final == UWB_BURST_BASE_UUS + 7U * UWB_BURST_SLOT_UUS + UWB_BURST_FINAL_MARGIN_UUS);
    CHECK(Tag_PopBurst(&rec) == 0U);          /* nothing until Rb arrives */

    CHECK(run_cycle(&c2) == 0);
    CHECK(c2.txn == (uint8_t)(c1.txn + 1U));
    CHECK(pop_record(&rec) == 0);
    CHECK(rec.cycle_seq == 1U && rec.count == 8U);
    CHECK(rec.period_us > 5000U && rec.period_us < 12000U);   /* simulated POLL-to-POLL */
    for (uint8_t idx = 0U; idx < TAG_NUM_ANCHORS; idx++)
    {
        const TagBurstAnchor_t *e = entry_for(&rec, (uint8_t)(idx + 1U));
        CHECK(e != NULL);
        CHECK((e->flags & TAG_BURST_REC_RADIO_OK) != 0U);
        CHECK((e->flags & TAG_BURST_REC_MODE_MASK) == TAG_MEAS_MODE_DS);
        CHECK((e->flags & TAG_BURST_REC_CAL_OK) == 0U);
        CHECK(e->status == TAG_ST_CAL_MISSING_DS);
        if (abs((int)e->range_mm - true_mm(&g_anchor[idx])) > 2)
        {
            fprintf(stderr, "A%u range %u mm, expected %d mm\n",
                    idx + 1U, e->range_mm, true_mm(&g_anchor[idx]));
            return 1;
        }
        CHECK(e->fp_cdbm != INT16_MIN && e->rx_cdbm != INT16_MIN);
        /* Clock offset read from the carrier integrator: 0.01 ppm units. */
        CHECK(abs(e->ci_ppm_x100 - (int)lround(g_anchor[idx].ppm * 100.0)) <= 2);
        CHECK(anchor_ds_ok_count[idx] >= 1U);
    }
    CHECK(rec.meas_seq_first + 8U == s_meas_seq);
    return 0;
}

static int test_calibrated_ranges_and_snapshot(void)
{
    CycleSeen cs;
    TagBurstRecord_t rec;

    CHECK(fresh_tag() == 0);
    CHECK(Tag_SetDsCalibration(6U, 50000, 1U) == 0);     /* A6: bias 50 mm */
    for (int k = 0; k < 4; k++)
        CHECK(run_cycle(&cs) == 0);
    while (Tag_PopBurst(&rec))
    {
        const TagBurstAnchor_t *e = entry_for(&rec, 6U);
        CHECK(e != NULL && (e->flags & TAG_BURST_REC_CAL_OK) != 0U);
        CHECK(abs((int)e->range_mm - (true_mm(&g_anchor[5]) - 50)) <= 2);
    }
    /* The snapshot follows the published track, decimated in time. */
    TagCycleSnapshot_t snap;
    Tag_GetSnapshot(&snap);
    CHECK(abs(snap.anchor[5].raw_mm - (true_mm(&g_anchor[5]) - 50)) <= 2);
    return 0;
}

static int test_missing_final_falls_back_to_corrected_ss(void)
{
    CycleSeen cs;
    TagBurstRecord_t rec;

    CHECK(fresh_tag() == 0);
    g_anchor[5].hears_final = 0;          /* A6, -20.5 ppm, never hears FINAL */
    CHECK(run_cycle(&cs) == 0);
    CHECK(run_cycle(&cs) == 0);
    CHECK(pop_record(&rec) == 0);
    const TagBurstAnchor_t *e = entry_for(&rec, 6U);
    CHECK(e != NULL && (e->flags & TAG_BURST_REC_MODE_MASK) == TAG_MEAS_MODE_SS_FALLBACK);
    CHECK((e->status & TAG_ST_DS_FALLBACK) != 0U);
    /* Uncorrected, a 2.3 ms reply at -20.5 ppm would read ~7 m too long. */
    CHECK(abs((int)e->range_mm - true_mm(&g_anchor[5])) < 60);
    CHECK(anchor_report_timeout_count[5] == 1U);
    CHECK(anchor_ds_fallback_count[5] >= 1U);
    const TagBurstAnchor_t *ok = entry_for(&rec, 5U);
    CHECK(ok != NULL && (ok->flags & TAG_BURST_REC_MODE_MASK) == TAG_MEAS_MODE_DS);
    return 0;
}

static int test_missing_resp_and_offline_probe(void)
{
    CycleSeen cs;
    TagBurstRecord_t rec;

    CHECK(fresh_tag() == 0);
    g_anchor[7].answers = 0;               /* A8 silent */
    g_anchor[2].report_late = 1;           /* A3 says it missed a slot */
    CHECK(run_cycle(&cs) == 0);
    CHECK(run_cycle(&cs) == 0);
    CHECK(pop_record(&rec) == 0);
    const TagBurstAnchor_t *e = entry_for(&rec, 8U);
    CHECK(e != NULL && e->status == TAG_ST_TIMEOUT && e->range_mm == 0xFFFFU);
    CHECK((entry_for(&rec, 3U)->flags & TAG_BURST_REC_ANCHOR_LATE) != 0U);
    CHECK(anchor_burst_late_count[2] >= 1U);

    /* After UWB_BURST_OFFLINE_AFTER misses A8 leaves the mask ... */
    for (uint32_t k = 2U; k < UWB_BURST_OFFLINE_AFTER; k++)
        CHECK(run_cycle(&cs) == 0);
    CHECK(Tag_AnchorBackedOff(7U) == 1U);
    CHECK(run_cycle(&cs) == 0);
    CHECK(cs.mask == 0x7FU);
    CHECK(cs.final == UWB_BURST_BASE_UUS + 6U * UWB_BURST_SLOT_UUS + UWB_BURST_FINAL_MARGIN_UUS);
    /* ... and is probed every UWB_BURST_PROBE_CYCLES; answering brings it back. */
    uint32_t probes = 0U;
    g_anchor[7].answers = 1;
    for (uint32_t k = 0U; k < UWB_BURST_PROBE_CYCLES + 1U; k++)
    {
        CHECK(run_cycle(&cs) == 0);
        if (cs.mask & 0x80U)
            probes++;
    }
    CHECK(probes >= 1U);
    CHECK(Tag_AnchorBackedOff(7U) == 0U);
    CHECK(run_cycle(&cs) == 0);
    CHECK(cs.mask == 0xFFU);
    while (Tag_PopBurst(&rec)) {}
    return 0;
}

static int test_info_tlv_widens_its_cycle(void)
{
    CycleSeen cs;

    CHECK(fresh_tag() == 0);
    s_burst_last_info_ms = uwb_platform_time_ms() - UWB_BURST_INFO_PERIOD_MS;
    CHECK(run_cycle(&cs) == 0);
    CHECK(cs.info_id == 1U);
    CHECK(cs.slot == UWB_BURST_SLOT_UUS + UWB_BURST_INFO_EXTRA_UUS);
    TagAnchorInfo_t info;
    CHECK(Tag_TakeAnchorInfo(&info) == 1U);
    CHECK(info.anchor_id == 1U && info.pos_mm[0] == 1000 && info.pos_mm[2] == 3000);
    CHECK(run_cycle(&cs) == 0);
    CHECK(cs.info_id == 0U && cs.slot == UWB_BURST_SLOT_UUS);
    return 0;
}

static int test_late_final_and_timing_command(void)
{
    CycleSeen cs;
    TagBurstRecord_t rec;

    CHECK(fresh_tag() == 0);
    CHECK(run_cycle(&cs) == 0);
    sim_force_hpdwarn = 1;                 /* FINAL cannot keep its time */
    CHECK(run_cycle(&cs) == 0);
    CHECK(tag_burst_final_late_count == 1U);
    CHECK(pop_record(&rec) == 0);          /* cycle 1: its Rb came in cycle 2 */
    CHECK((entry_for(&rec, 1U)->flags & TAG_BURST_REC_MODE_MASK) == TAG_MEAS_MODE_DS);
    CHECK(run_cycle(&cs) == 0);
    CHECK(pop_record(&rec) == 0);          /* cycle 2 had no FINAL: SS fallback */
    CHECK((entry_for(&rec, 1U)->flags & TAG_BURST_REC_MODE_MASK) == TAG_MEAS_MODE_SS_FALLBACK);

    /* New timing is validated, then used from the next cycle on. */
    const TagBurstTiming_t bad = { 100U, 350U, 450U, 500U, 0U };
    CHECK(Tag_SetBurstTiming(&bad) == -1);
    const TagBurstTiming_t good = { 600U, 300U, 400U, 300U, 5000U };
    CHECK(Tag_SetBurstTiming(&good) == 0);
    TagBurstTiming_t now;
    Tag_GetBurstTiming(&now);
    CHECK(now.base_uus == 600U && now.slot_uus == 300U && now.period_us == 5000U);
    CHECK(run_cycle(&cs) == 0);
    CHECK(cs.base == 600U && cs.slot == 300U && cs.final == 600U + 7U * 300U + 400U);
    CHECK(run_cycle(&cs) == 0);
    CHECK(pop_record(&rec) == 0);
    CHECK(abs((int)entry_for(&rec, 8U)->range_mm - true_mm(&g_anchor[7])) <= 2);
    while (Tag_PopBurst(&rec)) {}
    return 0;
}

static int test_switch_back_to_sequential(void)
{
    CycleSeen cs;

    CHECK(fresh_tag() == 0);
    CHECK(run_cycle(&cs) == 0);
    CHECK(Tag_SetBurstMode(0U) == 0);
    sim_us += (uint64_t)TAG_CYCLE_MS * 1000U + 1000U;
    Tag_Task();                               /* switches while idle */
    CHECK(Tag_BurstMode() == 0U && Tag_FrameVersion() == UWB_TAG_FRAME_VERSION);
    CHECK(s_state == TAG_STATE_TX_POLL);      /* first unicast POLL of v2 */
    CHECK(sim_tx_frame[10] == UWB_FRAME_V2);
    CHECK(Tag_SetBurstMode(1U) == 0);
    return 0;
}

int main(void)
{
    if (test_eight_anchors_one_cycle_late()
        || test_calibrated_ranges_and_snapshot()
        || test_missing_final_falls_back_to_corrected_ss()
        || test_missing_resp_and_offline_probe()
        || test_info_tlv_widens_its_cycle()
        || test_late_final_and_timing_command()
        || test_switch_back_to_sequential())
    {
        return 1;
    }
    puts("TAG burst tests passed (v3 POLL/slots/FINAL, 8 anchors to 2 mm at -22..+0.4 ppm, "
         "Rb one cycle late, SS fallback with CI, offline probe, info TLV, timing, switch)");
    return 0;
}
