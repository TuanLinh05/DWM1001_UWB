/**
 ******************************************************************************
 * @file    uwb_frame.c
 * @brief   UWB ranging frame encoding/decoding (see uwb_frame.h).
 ******************************************************************************
 */

#include "uwb_frame.h"

#include <stddef.h>
#include <string.h>

#define FC0 0x41U   /* data frame, PAN ID compression */
#define FC1 0x88U   /* 16-bit destination and source addresses */

uint16_t uwb_frame_write_header(uint8_t *buf, uint8_t seq, uint16_t pan,
                                uint16_t dst, uint16_t src, uint8_t func)
{
    buf[0] = FC0;
    buf[1] = FC1;
    buf[2] = seq;
    uwb_put_u16(&buf[3], pan);
    uwb_put_u16(&buf[5], dst);
    uwb_put_u16(&buf[7], src);
    buf[9] = func;
    return UWB_FRAME_HEADER_LEN;
}

int uwb_frame_parse_header(const uint8_t *frame, uint16_t rx_len, uint16_t pan,
                           UwbFrameHeader_t *hdr)
{
    if (frame == NULL || hdr == NULL
        || rx_len < (uint16_t)(UWB_FRAME_HEADER_LEN + UWB_FRAME_FCS_LEN)
        || rx_len > UWB_FRAME_MAX_RX_LEN
        || frame[0] != FC0 || frame[1] != FC1
        || uwb_get_u16(&frame[3]) != pan)
    {
        return 0;
    }

    hdr->seq = frame[2];
    hdr->pan = pan;
    hdr->dst = uwb_get_u16(&frame[5]);
    hdr->src = uwb_get_u16(&frame[7]);
    hdr->func = frame[9];
    return 1;
}

uint16_t uwb_frame_build_poll(uint8_t *buf, uint8_t seq, uint16_t pan,
                              uint16_t dst, uint16_t src, const UwbPoll_t *poll)
{
    uint16_t len = uwb_frame_write_header(buf, seq, pan, dst, src, 0x21U);

    if (poll->version >= UWB_FRAME_V2)
    {
        buf[len++] = UWB_FRAME_V2;
        buf[len++] = poll->txn;
        buf[len++] = poll->flags;
    }
    return len;
}

uint16_t uwb_frame_build_resp(uint8_t *buf, uint8_t seq, uint16_t pan,
                              uint16_t dst, uint16_t src, const UwbResp_t *resp)
{
    uint16_t len = uwb_frame_write_header(buf, seq, pan, dst, src, 0x10U);

    if (resp->version < UWB_FRAME_V2)
    {
        uwb_put_u32(&buf[len], resp->reply_ticks);
        return (uint16_t)(len + 4U);
    }

    uint8_t tlv_len = resp->tlv_len;
    if (resp->tlv == NULL || tlv_len > UWB_RESP_TLV_MAX)
        tlv_len = 0U;

    buf[len++] = UWB_FRAME_V2;
    buf[len++] = resp->txn;
    uwb_put_u32(&buf[len], resp->reply_ticks);
    len = (uint16_t)(len + 4U);
    buf[len++] = resp->anchor_status;
    buf[len++] = tlv_len;
    if (tlv_len != 0U)
    {
        memcpy(&buf[len], resp->tlv, tlv_len);
        len = (uint16_t)(len + tlv_len);
    }
    return len;
}

uint16_t uwb_frame_build_final(uint8_t *buf, uint8_t seq, uint16_t pan,
                               uint16_t dst, uint16_t src, const UwbFinal_t *fin)
{
    uint16_t len = uwb_frame_write_header(buf, seq, pan, dst, src, 0x23U);

    if (fin->version >= UWB_FRAME_V2)
    {
        buf[len++] = UWB_FRAME_V2;
        buf[len++] = fin->txn;
    }
    return len;
}

uint16_t uwb_frame_build_report(uint8_t *buf, uint8_t seq, uint16_t pan,
                                uint16_t dst, uint16_t src, const UwbReport_t *rep)
{
    uint16_t len = uwb_frame_write_header(buf, seq, pan, dst, src, 0x22U);

    if (rep->version < UWB_FRAME_V2)
    {
        uwb_put_u32(&buf[len], rep->round_ticks);
        return (uint16_t)(len + 4U);
    }

    buf[len++] = UWB_FRAME_V2;
    buf[len++] = rep->txn;
    uwb_put_u32(&buf[len], rep->round_ticks);
    len = (uint16_t)(len + 4U);
    uwb_put_u16(&buf[len], (uint16_t)rep->final_fp_cdbm);
    len = (uint16_t)(len + 2U);
    uwb_put_u16(&buf[len], (uint16_t)rep->final_rx_cdbm);
    len = (uint16_t)(len + 2U);
    return len;
}

int uwb_frame_parse_poll(const uint8_t *frame, uint16_t rx_len, UwbPoll_t *out)
{
    if (rx_len == UWB_POLL_V1_RX_LEN)
    {
        out->version = UWB_FRAME_V1;
        out->txn = frame[2];        /* v1 has no transaction ID */
        out->flags = 0U;
        return 1;
    }
    if (rx_len == UWB_POLL_V2_RX_LEN && frame[10] == UWB_FRAME_V2)
    {
        out->version = UWB_FRAME_V2;
        out->txn = frame[11];
        out->flags = frame[12];
        return 1;
    }
    return 0;
}

int uwb_frame_parse_resp(const uint8_t *frame, uint16_t rx_len, UwbResp_t *out)
{
    if (rx_len == UWB_RESP_V1_RX_LEN)
    {
        out->version = UWB_FRAME_V1;
        out->txn = 0U;
        out->reply_ticks = uwb_get_u32(&frame[10]);
        out->anchor_status = 0U;
        out->tlv_len = 0U;
        out->tlv = NULL;
        return 1;
    }
    if (rx_len >= UWB_RESP_V2_MIN_RX_LEN && frame[10] == UWB_FRAME_V2)
    {
        const uint8_t tlv_len = frame[17];
        if ((uint16_t)(UWB_RESP_V2_MIN_RX_LEN + tlv_len) != rx_len)
            return 0;
        out->version = UWB_FRAME_V2;
        out->txn = frame[11];
        out->reply_ticks = uwb_get_u32(&frame[12]);
        out->anchor_status = frame[16];
        out->tlv_len = tlv_len;
        out->tlv = tlv_len != 0U ? &frame[18] : NULL;
        return 1;
    }
    return 0;
}

int uwb_frame_parse_final(const uint8_t *frame, uint16_t rx_len, UwbFinal_t *out)
{
    if (rx_len == UWB_FINAL_V1_RX_LEN)
    {
        out->version = UWB_FRAME_V1;
        out->txn = 0U;
        return 1;
    }
    if (rx_len == UWB_FINAL_V2_RX_LEN && frame[10] == UWB_FRAME_V2)
    {
        out->version = UWB_FRAME_V2;
        out->txn = frame[11];
        return 1;
    }
    return 0;
}

int uwb_frame_parse_report(const uint8_t *frame, uint16_t rx_len, UwbReport_t *out)
{
    if (rx_len == UWB_REPORT_V1_RX_LEN)
    {
        out->version = UWB_FRAME_V1;
        out->txn = 0U;
        out->round_ticks = uwb_get_u32(&frame[10]);
        out->final_fp_cdbm = INT16_MIN;
        out->final_rx_cdbm = INT16_MIN;
        return 1;
    }
    if (rx_len == UWB_REPORT_V2_RX_LEN && frame[10] == UWB_FRAME_V2)
    {
        out->version = UWB_FRAME_V2;
        out->txn = frame[11];
        out->round_ticks = uwb_get_u32(&frame[12]);
        out->final_fp_cdbm = (int16_t)uwb_get_u16(&frame[16]);
        out->final_rx_cdbm = (int16_t)uwb_get_u16(&frame[18]);
        return 1;
    }
    return 0;
}

const uint8_t *uwb_frame_find_tlv(const uint8_t *tlv, uint8_t tlv_len,
                                  uint8_t type, uint8_t expected_len)
{
    uint16_t offset = 0U;

    if (tlv == NULL)
        return NULL;

    while ((uint16_t)(offset + 2U) <= tlv_len)
    {
        const uint8_t item_type = tlv[offset];
        const uint8_t item_len = tlv[offset + 1U];
        const uint16_t next = (uint16_t)(offset + 2U + item_len);

        if (next > tlv_len)
            return NULL;            /* truncated item: stop, trust nothing */
        if (item_type == type)
            return item_len == expected_len ? &tlv[offset + 2U] : NULL;
        offset = next;
    }
    return NULL;
}
