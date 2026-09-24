/**
 ******************************************************************************
 * @file    anchor_ranging.h
 * @brief   Anchor TWR responder — non-blocking state machine header
 *
 * - Anchor_Init()           : DW1000 init + configure + set address.
 * - Anchor_StartListening() : arm RX for the first time.
 * - Anchor_Task()           : call repeatedly from the main loop; returns
 *                             immediately if no IRQ is pending.
 *
 * The anchor answers protocol v1 and v2 POLLs in the version it received,
 * so a TAG still running the baseline firmware keeps working.
 ******************************************************************************
 */

#ifndef ANCHOR_RANGING_H
#define ANCHOR_RANGING_H

#include "dw1000_hw.h"
#include "uwb_frame.h"

/* ========================================================================== */
/*                     TIMING CONSTANTS                                        */
/* ========================================================================== */

/**
 * Reply delay from POLL RX to RESP TX, in UWB microseconds (1 UUS = 512/499.2
 * µs ≈ 1.026 µs). 1200 UUS ≈ 1.23 ms leaves room for the SPI work between the
 * POLL interrupt and the delayed-TX deadline; HPDWARN counts show when the
 * margin is too small.
 */
#define ANCHOR_REPLY_DELAY_UUS  1200UL

/**
 * Backstop for a delayed TX that never completes. Late TX is caught at once
 * through HPDWARN (F-09), so this only covers a hung transmitter.
 */
#define ANCHOR_TX_TIMEOUT_MS    10

/** DS-TWR: max wait for FINAL after RESP TX. FINAL normally arrives within
 *  ~1-2 ms; must stay below the TAG's slot so the anchor is back in RX_WAIT
 *  before its next POLL. */
#define ANCHOR_FINAL_TIMEOUT_MS  5

/** Restart RX after this long without any event (known DW1000 RX lock-up). */
#define ANCHOR_RX_IDLE_RESTART_MS 200U

/**
 * 1: RESP is sent with WAIT4RESP in DS mode so the receiver is on for FINAL
 * without MCU latency.
 */
#ifndef UWB_USE_WAIT4RESP
#define UWB_USE_WAIT4RESP       1
#endif

/**
 * 1: a v2 REPORT carries the first-path and receive power of the FINAL as
 * seen by the anchor, giving the host a two-way LOS/NLOS indication.
 */
#ifndef UWB_ANCHOR_REPORT_DIAG
#define UWB_ANCHOR_REPORT_DIAG  1
#endif

/* Optional surveyed anchor position (mm), reported in the ANCHOR_POSITION
 * TLV. Define ANCHOR_POS_VALID 1 in uwb_app_config.h once surveyed. */
#ifndef ANCHOR_POS_VALID
#define ANCHOR_POS_VALID        0
#endif
#ifndef ANCHOR_POS_X_MM
#define ANCHOR_POS_X_MM         0
#endif
#ifndef ANCHOR_POS_Y_MM
#define ANCHOR_POS_Y_MM         0
#endif
#ifndef ANCHOR_POS_Z_MM
#define ANCHOR_POS_Z_MM         0
#endif

/* ========================================================================== */
/*                     DIAGNOSTICS                                             */
/* ========================================================================== */

/** Counters kept by the anchor (debugger / future telemetry). */
typedef struct {
    uint32_t polls;               /* POLLs addressed to this anchor */
    uint32_t polls_v2;            /* ... of which protocol v2 */
    uint32_t resp_sent;
    uint32_t delayed_tx_late;     /* HPDWARN/TXPUTE on RESP */
    uint32_t tx_timeouts;
    uint32_t finals;
    uint32_t final_timeouts;
    uint32_t reports_sent;
    uint32_t info_replies;        /* RESP with info TLVs */
    uint32_t rx_errors;
    uint32_t rx_soft_resets;
    uint32_t stray_frames;
    uint32_t txn_mismatch;
    uint32_t idle_rx_restarts;
} AnchorStats_t;

extern AnchorStats_t anchor_stats;

/** Đếm số lần delayed-TX bị trễ (HPDWARN) — Live Expressions (F-09). */
extern volatile uint32_t anchor_delayed_tx_late_count;

/* ========================================================================== */
/*                     FUNCTION PROTOTYPES                                     */
/* ========================================================================== */

/**
 * @brief  Initialize the Anchor: DW1000 init + configure + set address.
 * @retval 0 = success, negative = DW1000 init failure
 */
int Anchor_Init(void);

/** @brief Arm RX for the first time. Call after DW1000_EnableIRQ(). */
void Anchor_StartListening(void);

/** @brief Non-blocking state machine tick; call continuously. */
void Anchor_Task(void);

/**
 * @brief  Re-initialise the DW1000 after a fault and return to RX.
 * @retval 0 on success, negative on DW1000 init failure.
 */
int Anchor_RecoverRadio(void);

/**
 * @brief  1 when a DW1000 register check will not disturb an exchange: the
 *         anchor has just finished one, or has been idle in RX for a while.
 *         Cleared by the caller through Anchor_ConsumeQuietWindow().
 */
uint8_t Anchor_InQuietWindow(void);
void Anchor_ConsumeQuietWindow(void);

/** @brief Store the last DW1000_VerifyConfig() result for the status byte. */
void Anchor_NoteConfigCheck(uint32_t mismatch);

/** @brief Record that a radio recovery happened (sticky status bit). */
void Anchor_NoteRecovery(void);

/** @brief Monotonic count of handled radio events (health progress). */
uint32_t Anchor_EventCount(void);

/** @brief Get a pointer to the ranging result structure. */
const DW1000_RangingResult* Anchor_GetResult(void);

#endif /* ANCHOR_RANGING_H */
