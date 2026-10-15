/**
 ******************************************************************************
 * @file    uwb_frame.h
 * @brief   UWB ranging frame encoding/decoding (protocol v1, v2 and v3).
 *
 * All frames use the IEEE 802.15.4 data-frame header with 16-bit addresses
 * and PAN ID compression, followed by a function code:
 *
 *   [0-1] FC 0x41 0x88 | [2] MAC seq | [3-4] PAN | [5-6] dst | [7-8] src | [9] func
 *
 * Version 1 (baseline) payloads:
 *   POLL   : -
 *   RESP   : [10-13] Da (u32)
 *   FINAL  : -
 *   REPORT : [10-13] Rb (u32)
 *
 * Version 2 adds a version byte and a transaction ID echoed through the whole
 * POLL -> RESP -> FINAL -> REPORT exchange (FIRMWARE_REVIEW F4):
 *   POLL   : [10] ver=2 [11] txn [12] flags
 *   RESP   : [10] ver [11] txn [12-15] Da (u32) [16] anchor_status
 *            [17] tlv_len [18..] TLV (type, len, value)*
 *   FINAL  : [10] ver [11] txn
 *   REPORT : [10] ver [11] txn [12-15] Rb (u32)
 *            [16-17] FINAL first-path power (cdBm, i16)
 *            [18-19] FINAL receive power (cdBm, i16)
 *
 * Version 3 is the one-to-many ("burst") DS-TWR: one broadcast POLL, one
 * RESP per anchor in its own time slot, one broadcast FINAL, no REPORT. The
 * anchor's Rb of a cycle rides in its RESP of the NEXT cycle:
 *   POLL   : dst 0xFFFF [10] ver=3 [11] txn [12] flags [13] anchor_mask
 *            [14-15] base_uus [16-17] slot_uus [18-19] final_uus [20] info_id
 *            Anchor with ID a (bit a-1 of the mask) and rank k among the set
 *            bits sends its RESP RMARKER at POLL_RX + base + k * slot; the
 *            TAG's FINAL RMARKER is at POLL + final. UUS = 65536 DW ticks.
 *   RESP   : [10] ver=3 [11] txn [12-15] Da (u32) [16] anchor_status
 *            [17] prev_txn [18-21] prev Rb (u32) [22-23] prev FINAL
 *            first-path power [24-25] prev FINAL receive power (cdBm, i16)
 *            [26] tlv_len [27..] TLV. prev_* are valid only with
 *            UWB_ANCHOR_ST_PREV_RB and belong to transaction prev_txn.
 *   FINAL  : dst 0xFFFF [10] ver=3 [11] txn
 *
 * Lengths passed to the parsers are the RX_FINFO lengths, i.e. they include
 * the 2-byte FCS that the DW1000 appends. Builders return the length
 * without FCS (the DW1000 adds it; TX_FCTRL gets length + 2).
 *
 * Pure C, no platform dependency: covered by host tests.
 ******************************************************************************
 */

#ifndef UWB_FRAME_H
#define UWB_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_FRAME_HEADER_LEN      10U
#define UWB_FRAME_FCS_LEN         2U
#define UWB_FRAME_V1              1U
#define UWB_FRAME_V2              2U
#define UWB_FRAME_V3              3U  /* one-to-many (burst) DS-TWR */
#define UWB_FRAME_BROADCAST       0xFFFFU

/** Largest frame (including FCS) that any node reads from the RX buffer. */
#define UWB_FRAME_MAX_RX_LEN      64U

/* Lengths as reported by RX_FINFO (with FCS). */
#define UWB_POLL_V1_RX_LEN        (10U + UWB_FRAME_FCS_LEN)
#define UWB_POLL_V2_RX_LEN        (13U + UWB_FRAME_FCS_LEN)
#define UWB_RESP_V1_RX_LEN        (14U + UWB_FRAME_FCS_LEN)
#define UWB_RESP_V2_MIN_RX_LEN    (18U + UWB_FRAME_FCS_LEN)
#define UWB_FINAL_V1_RX_LEN       (10U + UWB_FRAME_FCS_LEN)
#define UWB_FINAL_V2_RX_LEN       (12U + UWB_FRAME_FCS_LEN)
#define UWB_REPORT_V1_RX_LEN      (14U + UWB_FRAME_FCS_LEN)
#define UWB_REPORT_V2_RX_LEN      (20U + UWB_FRAME_FCS_LEN)
#define UWB_POLL_V3_RX_LEN        (21U + UWB_FRAME_FCS_LEN)
#define UWB_RESP_V3_MIN_RX_LEN    (27U + UWB_FRAME_FCS_LEN)
#define UWB_FINAL_V3_RX_LEN       (12U + UWB_FRAME_FCS_LEN)

/** Maximum TLV bytes carried in a v2 RESP. */
#define UWB_RESP_TLV_MAX          (UWB_FRAME_MAX_RX_LEN - UWB_RESP_V2_MIN_RX_LEN)
/** Maximum TLV bytes carried in a v3 RESP (the three info TLVs need 34). */
#define UWB_RESP_V3_TLV_MAX       (UWB_FRAME_MAX_RX_LEN - UWB_RESP_V3_MIN_RX_LEN)

/** One UWB microsecond (UUS) in DW1000 ticks. */
#define UWB_UUS_TO_DWT            65536ULL

/* POLL v2 flags */
#define UWB_POLL_FLAG_REQ_INFO    0x01U  /* ask the anchor for its info TLVs */

/* RESP v2 anchor_status bits */
#define UWB_ANCHOR_ST_POS_VALID   0x01U  /* anchor position is configured */
#define UWB_ANCHOR_ST_RECOVERED   0x02U  /* radio recovery happened since boot */
#define UWB_ANCHOR_ST_CFG_DRIFT   0x04U  /* last DW1000 config readback differed */
#define UWB_ANCHOR_ST_PREV_RB     0x08U  /* v3: prev_txn/prev Rb are valid */
#define UWB_ANCHOR_ST_LATE        0x10U  /* v3: the previous RESP missed its slot */

/* RESP v2 TLV types */
#define UWB_TLV_ANCHOR_POSITION   0x01U  /* int32 x_mm, y_mm, z_mm */
#define UWB_TLV_ANCHOR_BUILD      0x02U  /* u32 git hash, u8 dirty, u16 tx_ant,
                                             u16 rx_ant, u8 tx_power_mode, u16 boot_count */
#define UWB_TLV_ANCHOR_CONFIG     0x03U  /* u32 node-local build config hash */
#define UWB_TLV_ANCHOR_POSITION_LEN 12U
#define UWB_TLV_ANCHOR_BUILD_LEN    12U
#define UWB_TLV_ANCHOR_CONFIG_LEN    4U

typedef struct {
    uint8_t  seq;
    uint16_t pan;
    uint16_t dst;
    uint16_t src;
    uint8_t  func;
} UwbFrameHeader_t;

typedef struct {
    uint8_t  version;
    uint8_t  txn;
    uint8_t  flags;
    /* v3 only */
    uint8_t  anchor_mask;     /* bit a-1 = anchor ID a answers this cycle */
    uint16_t base_uus;        /* POLL RMARKER -> first RESP RMARKER */
    uint16_t slot_uus;        /* RESP RMARKER spacing */
    uint16_t final_uus;       /* POLL RMARKER -> FINAL RMARKER */
    uint8_t  info_id;         /* anchor that attaches its info TLVs, 0 = none */
} UwbPoll_t;

typedef struct {
    uint8_t        version;
    uint8_t        txn;
    uint32_t       reply_ticks;     /* Da: POLL RX RMARKER -> RESP TX RMARKER */
    uint8_t        anchor_status;
    uint8_t        tlv_len;
    const uint8_t *tlv;             /* parse: points into the frame buffer */
    /* v3 only: the anchor's side of the previous cycle */
    uint8_t        prev_txn;
    uint32_t       prev_rb_ticks;   /* Rb: RESP TX RMARKER -> FINAL RX RMARKER */
    int16_t        prev_final_fp_cdbm;  /* INT16_MIN when unknown */
    int16_t        prev_final_rx_cdbm;
} UwbResp_t;

typedef struct {
    uint8_t version;
    uint8_t txn;
} UwbFinal_t;

typedef struct {
    uint8_t  version;
    uint8_t  txn;
    uint32_t round_ticks;           /* Rb: RESP TX RMARKER -> FINAL RX RMARKER */
    int16_t  final_fp_cdbm;         /* v2 only, INT16_MIN when unknown */
    int16_t  final_rx_cdbm;         /* v2 only, INT16_MIN when unknown */
} UwbReport_t;

/** Write the common 10-byte header. @return 10 */
uint16_t uwb_frame_write_header(uint8_t *buf, uint8_t seq, uint16_t pan,
                                uint16_t dst, uint16_t src, uint8_t func);

/**
 * Check frame control, PAN ID and minimum length, and decode the header.
 * @return 1 if the frame is a well-formed ranging frame for `pan`, else 0.
 */
int uwb_frame_parse_header(const uint8_t *frame, uint16_t rx_len, uint16_t pan,
                           UwbFrameHeader_t *hdr);

uint16_t uwb_frame_build_poll(uint8_t *buf, uint8_t seq, uint16_t pan,
                              uint16_t dst, uint16_t src, const UwbPoll_t *poll);
uint16_t uwb_frame_build_resp(uint8_t *buf, uint8_t seq, uint16_t pan,
                              uint16_t dst, uint16_t src, const UwbResp_t *resp);
uint16_t uwb_frame_build_final(uint8_t *buf, uint8_t seq, uint16_t pan,
                               uint16_t dst, uint16_t src, const UwbFinal_t *fin);
uint16_t uwb_frame_build_report(uint8_t *buf, uint8_t seq, uint16_t pan,
                                uint16_t dst, uint16_t src, const UwbReport_t *rep);

/* Payload parsers: the header must already have been validated and the
 * function code checked. Return 1 when the length/version is consistent. */
int uwb_frame_parse_poll(const uint8_t *frame, uint16_t rx_len, UwbPoll_t *out);
int uwb_frame_parse_resp(const uint8_t *frame, uint16_t rx_len, UwbResp_t *out);
int uwb_frame_parse_final(const uint8_t *frame, uint16_t rx_len, UwbFinal_t *out);
int uwb_frame_parse_report(const uint8_t *frame, uint16_t rx_len, UwbReport_t *out);

/** Rank of anchor ID `anchor_id` among the set bits of a v3 anchor mask,
 *  i.e. its RESP slot index. @return -1 if the anchor is not in the mask. */
int uwb_frame_burst_slot(uint8_t anchor_mask, uint16_t anchor_id);

/**
 * Find a TLV of `type` in a RESP TLV area.
 * @return pointer to the value (length checked == expected_len), or NULL.
 */
const uint8_t *uwb_frame_find_tlv(const uint8_t *tlv, uint8_t tlv_len,
                                  uint8_t type, uint8_t expected_len);

/* Little-endian helpers shared by the ranging code. */
static inline void uwb_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void uwb_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline uint16_t uwb_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t uwb_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#ifdef __cplusplus
}
#endif

#endif /* UWB_FRAME_H */
