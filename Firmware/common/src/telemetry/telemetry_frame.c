/**
 ******************************************************************************
 * @file    telemetry_frame.c
 * @brief   Telemetry framing (see telemetry_frame.h).
 ******************************************************************************
 */

#include "telemetry_frame.h"
#include "uart_tx.h"
#include "uwb_platform.h"

#include <stddef.h>
#include <string.h>

/* Byte-wise table for CRC-16/CCITT-FALSE. The bitwise loop cost ~70 us per
 * 100-byte frame on the nRF52; with one RANGE_BURST frame per cycle at
 * 200+ Hz that time is taken from the gaps between cycles. */
static uint16_t s_crc_table[256];
static uint8_t  s_crc_table_ready;

static void crc_table_init(void)
{
    for (uint16_t v = 0U; v < 256U; v++)
    {
        uint16_t crc = (uint16_t)(v << 8);
        for (uint8_t b = 0U; b < 8U; b++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        s_crc_table[v] = crc;
    }
    s_crc_table_ready = 1U;
}

uint16_t telem_crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    if (!s_crc_table_ready)
        crc_table_init();
    for (uint16_t i = 0U; i < length; i++)
        crc = (uint16_t)((crc << 8) ^ s_crc_table[(uint8_t)((crc >> 8) ^ data[i])]);
    return crc;
}

uint16_t telem_frame_pack(uint8_t *buf, uint8_t type, uint32_t seq,
                          uint32_t time_ms, uint16_t payload_len)
{
    buf[0] = TELEM_SOF0;
    buf[1] = TELEM_SOF1;
    buf[2] = TELEM_VER;
    buf[3] = type;
    (void)telem_put_u16(&buf[4], payload_len);
    (void)telem_put_u32(&buf[6], seq);
    (void)telem_put_u32(&buf[10], time_ms);

    /* CRC over VER..end of payload (SOF and CRC excluded). */
    const uint16_t crc = telem_crc16_ccitt(&buf[2],
                                           (uint16_t)(TELEM_HDR_LEN - 2U + payload_len));
    const uint16_t off = (uint16_t)(TELEM_HDR_LEN + payload_len);
    (void)telem_put_u16(&buf[off], crc);
    return (uint16_t)(off + TELEM_CRC_LEN);
}

void Telem_SendRaw(uint8_t type, uint32_t seq, const uint8_t *payload,
                   uint16_t length)
{
    uint8_t buf[TELEM_MAX_FRAME];

    if (length > TELEM_MAX_PAYLOAD || (length > 0U && payload == NULL))
        return;
    if (length > 0U)
        memcpy(&buf[TELEM_HDR_LEN], payload, length);
    const uint16_t total = telem_frame_pack(buf, type, seq,
                                            uwb_platform_time_ms(), length);
    (void)UART_TX_Write(buf, total);
}
