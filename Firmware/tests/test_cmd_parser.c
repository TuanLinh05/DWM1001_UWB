/* Command frame parser tests: CRC, resync, fragmentation, limits. */
#include <stdio.h>
#include <string.h>

#include "telemetry_frame.h"
#include "uart_tx.h"
#include "uwb_cmd.h"
#include "uwb_platform.h"

/* telemetry_frame.c links against the UART and clock; not used here. */
uint32_t UART_TX_Write(const uint8_t *data, uint16_t length) { (void)data; return length; }
uint32_t uwb_platform_time_ms(void) { return 0U; }

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

static uint32_t s_calls;
static uint32_t s_last_seq;
static uint8_t  s_last_payload[UWB_CMD_MAX_PAYLOAD];
static uint16_t s_last_len;

static void on_frame(uint32_t seq, const uint8_t *payload, uint16_t length, void *ctx)
{
    (void)ctx;
    s_calls++;
    s_last_seq = seq;
    s_last_len = length;
    memcpy(s_last_payload, payload, length);
}

static uint16_t make_cmd(uint8_t *buf, uint8_t type, uint32_t seq,
                         const uint8_t *payload, uint16_t length)
{
    memcpy(&buf[TELEM_HDR_LEN], payload, length);
    return telem_frame_pack(buf, type, seq, 1234U, length);
}

int main(void)
{
    UwbCmdParser_t parser;
    uint8_t frame[TELEM_MAX_FRAME];
    const uint8_t ping[] = { UWB_CMD_PING };
    const uint8_t cal[] = { UWB_CMD_SET_DS_CAL, 0x01, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x01 };

    uwb_cmd_parser_init(&parser, on_frame, NULL);
    memset(&uwb_cmd_stats, 0, sizeof(uwb_cmd_stats));

    /* Complete frame. */
    uint16_t len = make_cmd(frame, TELEM_TYPE_CMD, 7U, ping, sizeof(ping));
    uwb_cmd_parser_feed(&parser, frame, len);
    CHECK(s_calls == 1U && s_last_seq == 7U && s_last_len == 1U);

    /* Garbage (including a false SOF) before a fragmented frame. */
    const uint8_t junk[] = { 0x00, 0xAA, 0x13, 0x55, 0xAA };
    uwb_cmd_parser_feed(&parser, junk, sizeof(junk));
    len = make_cmd(frame, TELEM_TYPE_CMD, 8U, cal, sizeof(cal));
    for (uint16_t i = 0U; i < len; i++)
        uwb_cmd_parser_feed(&parser, &frame[i], 1U);
    CHECK(s_calls == 2U && s_last_seq == 8U && s_last_len == sizeof(cal));
    CHECK(memcmp(s_last_payload, cal, sizeof(cal)) == 0);

    /* Corrupted CRC: counted, not delivered; the next frame still parses. */
    len = make_cmd(frame, TELEM_TYPE_CMD, 9U, ping, sizeof(ping));
    frame[TELEM_HDR_LEN] ^= 0x01U;
    uwb_cmd_parser_feed(&parser, frame, len);
    CHECK(s_calls == 2U && uwb_cmd_stats.crc_errors >= 1U);
    len = make_cmd(frame, TELEM_TYPE_CMD, 10U, ping, sizeof(ping));
    uwb_cmd_parser_feed(&parser, frame, len);
    CHECK(s_calls == 3U && s_last_seq == 10U);

    /* Telemetry frames echoed back (other types) are ignored. */
    len = make_cmd(frame, TELEM_TYPE_STATS, 11U, ping, sizeof(ping));
    uwb_cmd_parser_feed(&parser, frame, len);
    CHECK(s_calls == 3U);

    /* Oversized length field: rejected, parser resynchronises. */
    uint8_t big[6] = { TELEM_SOF0, TELEM_SOF1, TELEM_VER, TELEM_TYPE_CMD, 0xFF, 0x00 };
    uwb_cmd_parser_feed(&parser, big, sizeof(big));
    CHECK(uwb_cmd_stats.length_errors >= 1U);
    len = make_cmd(frame, TELEM_TYPE_CMD, 12U, ping, sizeof(ping));
    uwb_cmd_parser_feed(&parser, frame, len);
    CHECK(s_calls == 4U && s_last_seq == 12U);

    /* Two frames back to back in one buffer. */
    uint8_t two[2 * TELEM_MAX_FRAME];
    uint16_t a = make_cmd(two, TELEM_TYPE_CMD, 13U, ping, sizeof(ping));
    uint16_t b = make_cmd(&two[a], TELEM_TYPE_CMD, 14U, cal, sizeof(cal));
    uwb_cmd_parser_feed(&parser, two, (uint16_t)(a + b));
    CHECK(s_calls == 6U && s_last_seq == 14U);

    puts("command parser tests passed (CRC, resync, fragments, limits)");
    return 0;
}
