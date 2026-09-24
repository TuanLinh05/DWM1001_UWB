/**
 ******************************************************************************
 * @file    telemetry_frame.h
 * @brief   Telemetry framing shared by every node role that owns the UART.
 *
 *   SOF[2]=0xAA,0x55 | VER[1]=1 | TYPE[1] | LEN[2] | SEQ[4] | TIME[4] |
 *   PAYLOAD[LEN] | CRC16[2]
 *   CRC16-CCITT (0x1021, init 0xFFFF) over VER..end of payload, little-endian.
 *
 * Pure C apart from the UART write; covered by host tests.
 ******************************************************************************
 */

#ifndef TELEMETRY_FRAME_H
#define TELEMETRY_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEM_SOF0           0xAAU
#define TELEM_SOF1           0x55U
#define TELEM_VER            1
#define TELEM_HDR_LEN        14U
#define TELEM_CRC_LEN        2U
#define TELEM_MAX_PAYLOAD    256U
#define TELEM_MAX_FRAME      (TELEM_HDR_LEN + TELEM_MAX_PAYLOAD + TELEM_CRC_LEN)

#define TELEM_TYPE_INFO          0x00
#define TELEM_TYPE_RANGE         0x01
#define TELEM_TYPE_STATS         0x02
#define TELEM_TYPE_RANGE_MEAS    0x10
#define TELEM_TYPE_DIAG_ANCHOR   0x11
#define TELEM_TYPE_ANCHOR_INFO   0x12
#define TELEM_TYPE_CMD_ACK       0x13
#define TELEM_TYPE_SNIFFER_FRAME 0x14
#define TELEM_TYPE_DIAG_SYSTEM   0x15
#define TELEM_TYPE_DEVICE_INFO   0x16
#define TELEM_TYPE_GATEWAY_HEALTH 0x17   /* emitted by the ESP32-C3 gateway */
#define TELEM_TYPE_CMD           0x20

uint16_t telem_crc16_ccitt(const uint8_t *data, uint16_t length);

/**
 * Fill the header of `buf` (payload already at buf[TELEM_HDR_LEN..]) and
 * append the CRC. @return total frame length.
 */
uint16_t telem_frame_pack(uint8_t *buf, uint8_t type, uint32_t seq,
                          uint32_t time_ms, uint16_t payload_len);

/** Frame `payload` and queue it on the UART. Oversized payloads are dropped. */
void Telem_SendRaw(uint8_t type, uint32_t seq, const uint8_t *payload,
                   uint16_t length);

/* Little-endian writers returning the number of bytes written. */
static inline uint16_t telem_put_u8(uint8_t *p, uint8_t v)
{
    p[0] = v;
    return 1U;
}

static inline uint16_t telem_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    return 2U;
}

static inline uint16_t telem_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
    return 4U;
}

static inline uint16_t telem_put_u64(uint8_t *p, uint64_t v)
{
    (void)telem_put_u32(p, (uint32_t)(v & 0xFFFFFFFFU));
    (void)telem_put_u32(&p[4], (uint32_t)(v >> 32));
    return 8U;
}

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_FRAME_H */
