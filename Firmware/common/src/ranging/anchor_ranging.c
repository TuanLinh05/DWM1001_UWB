/**
 ******************************************************************************
 * @file    anchor_ranging.c
 * @brief   Anchor TWR responder — non-blocking state machine
 *
 * SS-TWR (UWB_USE_DS_TWR = 0):
 *   TAG ──POLL──> Anchor ──RESP(Da)──> TAG
 * DS-TWR (UWB_USE_DS_TWR = 1, default):
 *   TAG ──POLL──> Anchor ──RESP(Da)──> TAG ──FINAL──> Anchor ──REPORT(Rb)──> TAG
 *
 * RESP is a delayed TX at POLL_RX + ANCHOR_REPLY_DELAY_UUS; Da is predicted
 * from the scheduled time plus the active TX antenna delay (FIX-01) and the
 * prediction is checked against the real TX timestamp afterwards.
 *
 * Every reply uses the protocol version of the POLL that opened the
 * exchange; v2 echoes the TAG's transaction ID (F4).
 *
 * The GPIO IRQ callback only sets dw1000_irq_flag; all SPI work runs in
 * Anchor_Task(), called from the main loop.
 ******************************************************************************
 */

#include "anchor_ranging.h"
#include "uwb_health.h"
#include "uwb_platform.h"
#include <limits.h>
#include <string.h>

/* ========================================================================== */
/*                     PRIVATE DEFINES                                         */
/* ========================================================================== */

#define UUS_TO_DWT_TIME     65536ULL
#define REPLY_DELAY_TICKS   ((uint64_t)ANCHOR_REPLY_DELAY_UUS * UUS_TO_DWT_TIME)

/* 40-bit timestamp mask (counter wraps every ~17.2 s). */
#define UWB_TS40_MASK       0xFFFFFFFFFFULL

#ifndef UWB_BUILD_GIT_HASH
#define UWB_BUILD_GIT_HASH  0
#endif
#ifndef UWB_BUILD_GIT_DIRTY
#define UWB_BUILD_GIT_DIRTY 1
#endif
#ifndef UWB_BUILD_CONFIG_HASH
#define UWB_BUILD_CONFIG_HASH 0
#endif

/* ========================================================================== */
/*                     STATE MACHINE                                           */
/* ========================================================================== */

typedef enum {
    ANCHOR_STATE_RX_WAIT    = 0,  /* Listening for POLL from TAG */
    ANCHOR_STATE_TX_RESPOND = 1,  /* Waiting for delayed TX DONE (RESP) */
#if UWB_USE_DS_TWR
    ANCHOR_STATE_WAIT_FINAL = 2,  /* RESP sent, waiting for FINAL from TAG */
    ANCHOR_STATE_TX_REPORT  = 3,  /* REPORT being sent, waiting for TXFRS */
#endif
} AnchorState_t;

/* ========================================================================== */
/*                     PRIVATE DATA                                            */
/* ========================================================================== */

static AnchorState_t        s_state      = ANCHOR_STATE_RX_WAIT;
static uint32_t             s_state_tick = 0;   /* uptime at state entry */
static DW1000_RangingResult s_result     = {0};
static uint8_t              s_tx_seq     = 0;

/* Exchange opened by the last POLL addressed to this anchor. */
static uint16_t             s_active_tag     = 0;
static uint8_t              s_active_version = UWB_FRAME_V1;
static uint8_t              s_active_txn     = 0;

static uint32_t             s_event_count      = 0;
static uint8_t              s_quiet_window     = 0;
static uint8_t              s_recovered        = 0;
static uint32_t             s_last_cfg_mismatch = 0;

AnchorStats_t anchor_stats;

/* Phase 2 (F-09): số lần delayed-TX bị trễ (HPDWARN). Live Expressions. */
volatile uint32_t anchor_delayed_tx_late_count = 0;

/* FIX-01 verify: predicted RMARKER TX vs the TX_TIME read back after TXFRS.
 * A few DTU of quantization error is normal. */
static uint64_t s_predicted_resp_tx_rmarker     = 0;
volatile int32_t  anchor_tx_prediction_error_dtu   = 0;
volatile uint32_t anchor_tx_prediction_error_count = 0;

/* ========================================================================== */
/*                     PRIVATE HELPERS                                         */
/* ========================================================================== */

static uint64_t ts_to_u64(const uint8_t *ts)
{
    uint64_t v = 0;
    v  = (uint64_t)ts[0];
    v |= (uint64_t)ts[1] <<  8;
    v |= (uint64_t)ts[2] << 16;
    v |= (uint64_t)ts[3] << 24;
    v |= (uint64_t)ts[4] << 32;
    return v;
}

static int16_t dbm_to_cdbm(float dbm)
{
    const float centi = dbm * 100.0f;

    if (!(centi > (float)INT16_MIN))
        return INT16_MIN;
    if (centi > (float)INT16_MAX)
        return INT16_MAX;
    return (int16_t)centi;
}

static void enter_state(AnchorState_t state)
{
    s_state      = state;
    s_state_tick = uwb_platform_time_ms();
}

/**
 * @brief  Return to RX_WAIT: force the radio off, clear status, restart RX.
 *         Used for error recovery from any state.
 */
static void reset_to_rx_wait(void)
{
    DW1000_ForceRxOff();
    DW1000_ClearAllStatus();
    DW1000_StartRx();
    enter_state(ANCHOR_STATE_RX_WAIT);
}

/** Frame not for us: the receiver is idle after RX, re-arm it in place. */
static void rearm_rx(void)
{
    DW1000_ClearAllStatus();
    DW1000_StartRx();
}

/** RX error: soft-reset the receiver (LDE re-initialisation), re-arm RX. */
static void recover_rx_after_error(void)
{
    anchor_stats.rx_errors++;
    DW1000_ForceRxOff();
    DW1000_RxSoftReset();
    anchor_stats.rx_soft_resets++;
    rearm_rx();
}

static uint8_t anchor_status_byte(void)
{
    uint8_t status = 0U;

    if (ANCHOR_POS_VALID)
        status |= UWB_ANCHOR_ST_POS_VALID;
    if (s_recovered)
        status |= UWB_ANCHOR_ST_RECOVERED;
    if (s_last_cfg_mismatch != 0U)
        status |= UWB_ANCHOR_ST_CFG_DRIFT;
    return status;
}

/** Info TLVs sent when the POLL asks for them (commissioning, ~1/s). */
static uint8_t build_info_tlvs(uint8_t *tlv)
{
    uint8_t len = 0U;

    tlv[len++] = UWB_TLV_ANCHOR_POSITION;
    tlv[len++] = UWB_TLV_ANCHOR_POSITION_LEN;
    uwb_put_u32(&tlv[len], (uint32_t)(int32_t)ANCHOR_POS_X_MM);
    uwb_put_u32(&tlv[len + 4U], (uint32_t)(int32_t)ANCHOR_POS_Y_MM);
    uwb_put_u32(&tlv[len + 8U], (uint32_t)(int32_t)ANCHOR_POS_Z_MM);
    len = (uint8_t)(len + UWB_TLV_ANCHOR_POSITION_LEN);

    tlv[len++] = UWB_TLV_ANCHOR_BUILD;
    tlv[len++] = UWB_TLV_ANCHOR_BUILD_LEN;
    uwb_put_u32(&tlv[len], (uint32_t)UWB_BUILD_GIT_HASH);
    tlv[len + 4U] = (uint8_t)UWB_BUILD_GIT_DIRTY;
    uwb_put_u16(&tlv[len + 5U], dw1000_radio_config.tx_ant_dly);
    uwb_put_u16(&tlv[len + 7U], dw1000_radio_config.rx_ant_dly);
    tlv[len + 9U] = dw1000_radio_config.tx_power_mode;
    uwb_put_u16(&tlv[len + 10U], uwb_health.boot_count);
    len = (uint8_t)(len + UWB_TLV_ANCHOR_BUILD_LEN);

    tlv[len++] = UWB_TLV_ANCHOR_CONFIG;
    tlv[len++] = UWB_TLV_ANCHOR_CONFIG_LEN;
    uwb_put_u32(&tlv[len], (uint32_t)UWB_BUILD_CONFIG_HASH);
    len = (uint8_t)(len + UWB_TLV_ANCHOR_CONFIG_LEN);

    return len;
}

/**
 * @brief  Handle a POLL addressed to this anchor: schedule the delayed RESP.
 *         Called from RX_WAIT and from WAIT_FINAL (a new POLL aborts a DS
 *         exchange whose FINAL never came).
 */
static void handle_poll(const UwbFrameHeader_t *hdr, const UwbPoll_t *poll)
{
    DW1000_ReadRxTimestamp(s_result.poll_rx_ts);

    s_active_tag = hdr->src;
    s_active_version = poll->version;
    s_active_txn = poll->txn;
    anchor_stats.polls++;
    if (poll->version >= UWB_FRAME_V2)
        anchor_stats.polls_v2++;

    /* resp_tx: time written to DX_TIME (9 LSB cleared by the hardware rule).
     * resp_tx_rmarker: when the RMARKER actually leaves the antenna, i.e.
     * resp_tx + active TX antenna delay (FIX-01).
     * t_reply (Da): POLL RX RMARKER -> RESP TX RMARKER, 40-bit masked. */
    const uint64_t poll_rx         = ts_to_u64(s_result.poll_rx_ts);
    const uint64_t resp_tx         = (poll_rx + REPLY_DELAY_TICKS) & 0xFFFFFFFE00ULL;
    const uint64_t resp_tx_rmarker = (resp_tx + DW1000_ActiveTxAntennaDelay()) & UWB_TS40_MASK;
    const uint32_t t_reply         = (uint32_t)((resp_tx_rmarker - poll_rx) & UWB_TS40_MASK);

    s_predicted_resp_tx_rmarker = resp_tx_rmarker;

    uint8_t tlv[UWB_RESP_TLV_MAX];
    uint8_t tlv_len = 0U;
    if (poll->version >= UWB_FRAME_V2 && (poll->flags & UWB_POLL_FLAG_REQ_INFO) != 0U)
    {
        tlv_len = build_info_tlvs(tlv);
        anchor_stats.info_replies++;
    }

    const UwbResp_t resp = {
        .version = poll->version,
        .txn = poll->txn,
        .reply_ticks = t_reply,
        .anchor_status = anchor_status_byte(),
        .tlv_len = tlv_len,
        .tlv = tlv,
    };
    uint8_t frame[UWB_FRAME_MAX_RX_LEN];
    const uint16_t len = uwb_frame_build_resp(frame, s_tx_seq++, DW_PAN_ID,
                                              hdr->src, ANCHOR_ADDR, &resp);

    uint8_t dx[5];
    dx[0] = (uint8_t)((resp_tx >>  0) & 0xFF);
    dx[1] = (uint8_t)((resp_tx >>  8) & 0xFF);
    dx[2] = (uint8_t)((resp_tx >> 16) & 0xFF);
    dx[3] = (uint8_t)((resp_tx >> 24) & 0xFF);
    dx[4] = (uint8_t)((resp_tx >> 32) & 0xFF);

    DW1000_ClearAllStatus();
    DW1000_SetDelayedTxTime(dx);
    DW1000_WriteTxData(frame, len);
    DW1000_SetTxFrameCtrl((uint16_t)(len + UWB_FRAME_FCS_LEN));

    const uint8_t wait4resp = (UWB_USE_DS_TWR && UWB_USE_WAIT4RESP) ? 1U : 0U;
    if (DW1000_StartTxDelayedEx(wait4resp) != 0)
    {
        /* F-09: the transmit time already passed (HPDWARN); the TX would only
         * happen after the counter wraps. Recover at once. */
        anchor_delayed_tx_late_count++;
        anchor_stats.delayed_tx_late++;
        reset_to_rx_wait();
        return;
    }

    enter_state(ANCHOR_STATE_TX_RESPOND);
}

#if UWB_USE_DS_TWR
/** Valid FINAL of the open exchange: compute Rb and send the REPORT. */
static void send_report(void)
{
    uint8_t final_rx_ts[5];
    DW1000_ReadRxTimestamp(final_rx_ts);   /* T6 */

    const uint64_t t6 = ts_to_u64(final_rx_ts);
    const uint64_t t3 = ts_to_u64(s_result.resp_tx_ts);  /* real T3 (TX_TIME) */

    UwbReport_t rep = {
        .version = s_active_version,
        .txn = s_active_txn,
        .round_ticks = (uint32_t)((t6 - t3) & UWB_TS40_MASK),
        .final_fp_cdbm = INT16_MIN,
        .final_rx_cdbm = INT16_MIN,
    };

#if UWB_ANCHOR_REPORT_DIAG
    if (s_active_version >= UWB_FRAME_V2)
    {
        DW1000_SignalDiag_t diag;
        DW1000_ReadSignalDiag(&diag);
        rep.final_fp_cdbm = dbm_to_cdbm(DW1000_GetFirstPathPower(&diag));
        rep.final_rx_cdbm = dbm_to_cdbm(DW1000_GetRxPower(&diag));
    }
#endif

    uint8_t frame[UWB_FRAME_HEADER_LEN + 10U];
    const uint16_t len = uwb_frame_build_report(frame, s_tx_seq++, DW_PAN_ID,
                                                s_active_tag, ANCHOR_ADDR, &rep);

    /* Immediate TX — no HPDWARN possible here */
    DW1000_ClearAllStatus();
    DW1000_WriteTxData(frame, len);
    DW1000_SetTxFrameCtrl((uint16_t)(len + UWB_FRAME_FCS_LEN));
    DW1000_StartTx();
    enter_state(ANCHOR_STATE_TX_REPORT);
}
#endif

/** Read and classify one received frame. Returns 1 if it was consumed. */
static void handle_rx_frame(void)
{
    uint8_t  rx_buf[UWB_FRAME_MAX_RX_LEN];
    const uint16_t rx_len = DW1000_ReadRxData(rx_buf, UWB_FRAME_MAX_RX_LEN);
    UwbFrameHeader_t hdr;

    if (!uwb_frame_parse_header(rx_buf, rx_len, DW_PAN_ID, &hdr))
    {
        anchor_stats.stray_frames++;
        rearm_rx();
        return;
    }

    /* A POLL for this anchor always (re)starts an exchange, including while
     * a DS exchange waits for a FINAL that was lost. */
    if (hdr.func == FRAME_POLL_FUNC && hdr.src == TAG_ADDR)
    {
        UwbPoll_t poll;
        if (!uwb_frame_parse_poll(rx_buf, rx_len, &poll))
        {
            anchor_stats.stray_frames++;
            rearm_rx();
            return;
        }
        if (hdr.dst != ANCHOR_ADDR)
        {
            /* POLL for another anchor — ignore silently */
            rearm_rx();
            return;
        }
        handle_poll(&hdr, &poll);
        return;
    }

#if UWB_USE_DS_TWR
    if (s_state == ANCHOR_STATE_WAIT_FINAL
        && hdr.func == FRAME_FINAL_FUNC
        && hdr.dst == ANCHOR_ADDR
        && hdr.src == s_active_tag)
    {
        UwbFinal_t fin;
        if (!uwb_frame_parse_final(rx_buf, rx_len, &fin)
            || fin.version != s_active_version)
        {
            anchor_stats.stray_frames++;
            rearm_rx();        /* keep the FINAL deadline (FIX-03) */
            return;
        }
        if (fin.version >= UWB_FRAME_V2 && fin.txn != s_active_txn)
        {
            anchor_stats.txn_mismatch++;
            rearm_rx();
            return;
        }
        anchor_stats.finals++;
        send_report();
        return;
    }
#endif

    /* Another anchor's RESP/REPORT or foreign traffic. */
    anchor_stats.stray_frames++;
    rearm_rx();
}

/* ========================================================================== */
/*                     PUBLIC API                                              */
/* ========================================================================== */

static int anchor_radio_init(void)
{
    if (DW1000_Init() != 0)
        return -1;

    if (DW1000_Configure() != 0)            /* Also writes SYS_MASK */
        return -2;
    DW1000_SetAddress(DW_PAN_ID, ANCHOR_ADDR);
    if (DW1000_EnableFastSPI() == 0U)
        return -3;
    if (DW1000_VerifyConfig() != 0U)
        return -4;
    DW1000_ClearAllStatus();               /* Ensure IRQ pin is LOW */
    return 0;
}

int Anchor_Init(void)
{
    const int rc = anchor_radio_init();
    if (rc != 0)
        return rc;

    memset(&s_result, 0, sizeof(s_result));
    memset(&anchor_stats, 0, sizeof(anchor_stats));
    return 0;
}

void Anchor_StartListening(void)
{
    DW1000_ClearAllStatus();
    DW1000_StartRx();
    enter_state(ANCHOR_STATE_RX_WAIT);
}

int Anchor_RecoverRadio(void)
{
    const int rc = anchor_radio_init();

    dw1000_irq_flag = 0U;
    s_recovered = 1U;
    if (rc == 0)
        Anchor_StartListening();
    return rc;
}

void Anchor_NoteRecovery(void)
{
    s_recovered = 1U;
}

uint8_t Anchor_InQuietWindow(void)
{
    return s_quiet_window;
}

void Anchor_ConsumeQuietWindow(void)
{
    s_quiet_window = 0U;
}

void Anchor_NoteConfigCheck(uint32_t mismatch)
{
    s_last_cfg_mismatch = mismatch;
}

uint32_t Anchor_EventCount(void)
{
    return s_event_count;
}

void Anchor_Task(void)
{
    const uint32_t now = uwb_platform_time_ms();

    /* No IRQ: only then consider timeouts/recovery. */
    if (!dw1000_irq_flag)
    {
        const uint32_t elapsed = now - s_state_tick;

        if (s_state == ANCHOR_STATE_TX_RESPOND)
        {
            if (elapsed > ANCHOR_TX_TIMEOUT_MS)
            {
                s_result.timeout_count++;
                anchor_stats.tx_timeouts++;
                reset_to_rx_wait();
            }
        }
        else if (s_state == ANCHOR_STATE_RX_WAIT)
        {
            /* Known DW1000 RX lock-up: restart RX after a quiet period. This
             * is also a safe moment for periodic register checks. */
            if (elapsed > ANCHOR_RX_IDLE_RESTART_MS)
            {
                anchor_stats.idle_rx_restarts++;
                s_event_count++;
                s_quiet_window = 1U;
                reset_to_rx_wait();
            }
        }
#if UWB_USE_DS_TWR
        else if (s_state == ANCHOR_STATE_WAIT_FINAL)
        {
            /* An SS-only TAG never sends FINAL → timeout → back to RX_WAIT. */
            if (elapsed > ANCHOR_FINAL_TIMEOUT_MS)
            {
                anchor_stats.final_timeouts++;
                s_quiet_window = 1U;
                reset_to_rx_wait();
            }
        }
        else if (s_state == ANCHOR_STATE_TX_REPORT)
        {
            if (elapsed > ANCHOR_TX_TIMEOUT_MS)
            {
                s_result.timeout_count++;
                anchor_stats.tx_timeouts++;
                reset_to_rx_wait();
            }
        }
#endif
        return;
    }

    /* IRQ pending: handle it before any timeout. */
    dw1000_irq_flag = 0;
    s_event_count++;

    const uint64_t status = DW1000_ReadStatus();

    switch (s_state)
    {
        case ANCHOR_STATE_RX_WAIT:
#if UWB_USE_DS_TWR
        case ANCHOR_STATE_WAIT_FINAL:
#endif
        {
            const DW1000_RxEvent_t event = DW1000_ClassifyRx(status);

            if (event == DW_RX_EVENT_ERROR)
                recover_rx_after_error();   /* deadline of WAIT_FINAL is kept */
            else if (event == DW_RX_EVENT_NONE)
                DW1000_ClearTxStatus();     /* level-held TX event: release */
            else
                handle_rx_frame();
            break;
        }

        case ANCHOR_STATE_TX_RESPOND:
        {
            if (status & DW_TXFRS_BIT)
            {
                /* TX completed — read back the actual TX timestamp. */
                DW1000_ReadTxTimestamp(s_result.resp_tx_ts);
                s_result.ranging_count++;
                anchor_stats.resp_sent++;

                /* FIX-01 verify: predicted RMARKER vs TX_TIME read back. */
                const uint64_t actual = ts_to_u64(s_result.resp_tx_ts);
                int64_t err40 = (int64_t)((actual - s_predicted_resp_tx_rmarker) & UWB_TS40_MASK);
                if (err40 & (1LL << 39))        /* sign-extend 40-bit → 64-bit */
                    err40 -= (1LL << 40);
                anchor_tx_prediction_error_dtu = (int32_t)err40;
                if (err40 < -5 || err40 > 5)    /* tolerance: ±5 DTU */
                    anchor_tx_prediction_error_count++;

#if UWB_USE_DS_TWR
                /* Wait for FINAL. With WAIT4RESP the receiver is already on:
                 * clear TX events only. */
#if UWB_USE_WAIT4RESP
                DW1000_ClearTxStatus();
#else
                DW1000_ClearAllStatus();
                DW1000_StartRx();
#endif
                enter_state(ANCHOR_STATE_WAIT_FINAL);
#else
                /* SS-TWR: one exchange complete. */
                s_quiet_window = 1U;
                reset_to_rx_wait();
#endif
            }
            else
            {
                /* Unexpected IRQ in TX_RESPOND state — recover */
                reset_to_rx_wait();
            }
            break;
        }

#if UWB_USE_DS_TWR
        case ANCHOR_STATE_TX_REPORT:
        {
            if (status & DW_TXFRS_BIT)
            {
                /* REPORT sent — one DS exchange complete. */
                anchor_stats.reports_sent++;
            }
            /* The next POLL for this anchor is a full TAG cycle away: a safe
             * window for register checks. */
            s_quiet_window = 1U;
            reset_to_rx_wait();
            break;
        }
#endif

        default:
            reset_to_rx_wait();
            break;
    }

#if UWB_USE_DS_TWR
    /* A stream of unrelated frames must not keep WAIT_FINAL alive forever. */
    if (s_state == ANCHOR_STATE_WAIT_FINAL
        && (uint32_t)(uwb_platform_time_ms() - s_state_tick) > ANCHOR_FINAL_TIMEOUT_MS)
    {
        anchor_stats.final_timeouts++;
        reset_to_rx_wait();
    }
#endif
}

const DW1000_RangingResult* Anchor_GetResult(void)
{
    return &s_result;
}
