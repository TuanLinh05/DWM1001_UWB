/* UWB frame encode/decode tests (protocol v1 and v2). */
#include <stdio.h>
#include <string.h>

#include "uwb_frame.h"

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

#define PAN 0xDECAU

/* The parsers take RX_FINFO lengths, which include the 2-byte FCS. */
#define RX_LEN(n) ((uint16_t)((n) + UWB_FRAME_FCS_LEN))

static int test_header(void)
{
    uint8_t f[UWB_FRAME_MAX_RX_LEN] = {0};
    UwbFrameHeader_t h;

    CHECK(uwb_frame_write_header(f, 7U, PAN, 0x0003U, 0x0000U, 0x21U) == 10U);
    CHECK(f[0] == 0x41U && f[1] == 0x88U && f[2] == 7U);
    CHECK(uwb_frame_parse_header(f, RX_LEN(10), PAN, &h) == 1);
    CHECK(h.dst == 3U && h.src == 0U && h.func == 0x21U && h.seq == 7U);

    CHECK(uwb_frame_parse_header(f, RX_LEN(10), 0x1234U, &h) == 0);   /* PAN */
    CHECK(uwb_frame_parse_header(f, 5U, PAN, &h) == 0);               /* short */
    CHECK(uwb_frame_parse_header(f, UWB_FRAME_MAX_RX_LEN + 1U, PAN, &h) == 0);
    f[1] = 0xCCU;                                                      /* FC */
    CHECK(uwb_frame_parse_header(f, RX_LEN(10), PAN, &h) == 0);
    return 0;
}

static int test_poll_final(void)
{
    uint8_t f[UWB_FRAME_MAX_RX_LEN];
    UwbPoll_t out;
    UwbFinal_t fin_out;

    const UwbPoll_t v1 = { .version = UWB_FRAME_V1, .txn = 9U, .flags = 0U };
    uint16_t len = uwb_frame_build_poll(f, 42U, PAN, 1U, 0U, &v1);
    CHECK(len == 10U);
    CHECK(uwb_frame_parse_poll(f, RX_LEN(len), &out) == 1);
    CHECK(out.version == UWB_FRAME_V1 && out.txn == 42U);   /* v1: MAC seq */

    const UwbPoll_t v2 = { .version = UWB_FRAME_V2, .txn = 0x5AU,
                           .flags = UWB_POLL_FLAG_REQ_INFO };
    len = uwb_frame_build_poll(f, 43U, PAN, 1U, 0U, &v2);
    CHECK(len == 13U);
    CHECK(uwb_frame_parse_poll(f, RX_LEN(len), &out) == 1);
    CHECK(out.version == UWB_FRAME_V2 && out.txn == 0x5AU
          && out.flags == UWB_POLL_FLAG_REQ_INFO);
    f[10] = 3U;                                         /* unknown version */
    CHECK(uwb_frame_parse_poll(f, RX_LEN(len), &out) == 0);
    CHECK(uwb_frame_parse_poll(f, RX_LEN(11), &out) == 0);

    const UwbFinal_t fin = { .version = UWB_FRAME_V2, .txn = 0x77U };
    len = uwb_frame_build_final(f, 1U, PAN, 2U, 0U, &fin);
    CHECK(len == 12U && f[9] == 0x23U);
    CHECK(uwb_frame_parse_final(f, RX_LEN(len), &fin_out) == 1);
    CHECK(fin_out.version == UWB_FRAME_V2 && fin_out.txn == 0x77U);
    CHECK(uwb_frame_parse_final(f, RX_LEN(10), &fin_out) == 1
          && fin_out.version == UWB_FRAME_V1);
    return 0;
}

static int test_resp_report_tlv(void)
{
    uint8_t f[UWB_FRAME_MAX_RX_LEN];
    UwbResp_t out;
    UwbReport_t rep_out;

    const UwbResp_t v1 = { .version = UWB_FRAME_V1, .reply_ticks = 0x11223344U };
    uint16_t len = uwb_frame_build_resp(f, 5U, PAN, 0U, 4U, &v1);
    CHECK(len == 14U && f[9] == 0x10U);
    CHECK(uwb_frame_parse_resp(f, RX_LEN(len), &out) == 1);
    CHECK(out.version == UWB_FRAME_V1 && out.reply_ticks == 0x11223344U
          && out.tlv == NULL);

    uint8_t tlv[40];
    uint8_t n = 0U;
    tlv[n++] = UWB_TLV_ANCHOR_POSITION;
    tlv[n++] = UWB_TLV_ANCHOR_POSITION_LEN;
    uwb_put_u32(&tlv[n], (uint32_t)-1500); uwb_put_u32(&tlv[n + 4], 2500U);
    uwb_put_u32(&tlv[n + 8], 3000U);
    n = (uint8_t)(n + UWB_TLV_ANCHOR_POSITION_LEN);
    tlv[n++] = 0x7FU; tlv[n++] = 2U; tlv[n++] = 0xAAU; tlv[n++] = 0xBBU; /* unknown */

    const UwbResp_t v2 = { .version = UWB_FRAME_V2, .txn = 0x31U, .reply_ticks = 78643200U,
                           .anchor_status = UWB_ANCHOR_ST_POS_VALID, .tlv_len = n,
                           .tlv = tlv };
    len = uwb_frame_build_resp(f, 6U, PAN, 0U, 4U, &v2);
    CHECK(len == (uint16_t)(18U + n));
    CHECK(uwb_frame_parse_resp(f, RX_LEN(len), &out) == 1);
    CHECK(out.version == UWB_FRAME_V2 && out.txn == 0x31U && out.reply_ticks == 78643200U);
    CHECK(out.anchor_status == UWB_ANCHOR_ST_POS_VALID && out.tlv_len == n);
    const uint8_t *pos = uwb_frame_find_tlv(out.tlv, out.tlv_len,
                                            UWB_TLV_ANCHOR_POSITION,
                                            UWB_TLV_ANCHOR_POSITION_LEN);
    CHECK(pos != NULL && (int32_t)uwb_get_u32(pos) == -1500);
    CHECK(uwb_frame_find_tlv(out.tlv, out.tlv_len, UWB_TLV_ANCHOR_BUILD,
                             UWB_TLV_ANCHOR_BUILD_LEN) == NULL);
    /* Wrong declared length for a known type is rejected. */
    CHECK(uwb_frame_find_tlv(out.tlv, out.tlv_len, UWB_TLV_ANCHOR_POSITION, 8U) == NULL);
    /* Truncated TLV area: never trust a partial item. */
    CHECK(uwb_frame_find_tlv(tlv, 5U, UWB_TLV_ANCHOR_POSITION,
                             UWB_TLV_ANCHOR_POSITION_LEN) == NULL);
    /* Length field inconsistent with the frame length. */
    CHECK(uwb_frame_parse_resp(f, RX_LEN(len - 1U), &out) == 0);

    const UwbReport_t rep = { .version = UWB_FRAME_V2, .txn = 0x31U,
                              .round_ticks = 25559105U, .final_fp_cdbm = -8712,
                              .final_rx_cdbm = -7950 };
    len = uwb_frame_build_report(f, 7U, PAN, 0U, 4U, &rep);
    CHECK(len == 20U && f[9] == 0x22U);
    CHECK(uwb_frame_parse_report(f, RX_LEN(len), &rep_out) == 1);
    CHECK(rep_out.txn == 0x31U && rep_out.round_ticks == 25559105U);
    CHECK(rep_out.final_fp_cdbm == -8712 && rep_out.final_rx_cdbm == -7950);

    const UwbReport_t rep1 = { .version = UWB_FRAME_V1, .round_ticks = 99U };
    len = uwb_frame_build_report(f, 8U, PAN, 0U, 4U, &rep1);
    CHECK(len == 14U);
    CHECK(uwb_frame_parse_report(f, RX_LEN(len), &rep_out) == 1);
    CHECK(rep_out.version == UWB_FRAME_V1 && rep_out.round_ticks == 99U
          && rep_out.final_fp_cdbm == INT16_MIN);
    return 0;
}

int main(void)
{
    if (test_header() || test_poll_final() || test_resp_report_tlv())
        return 1;
    puts("uwb_frame tests passed (v1/v2 build/parse, TLV bounds)");
    return 0;
}
