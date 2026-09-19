#include "telemetry_protocol.h"

#include <string.h>

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8)
         | ((uint32_t)data[2] << 16)
         | ((uint32_t)data[3] << 24);
}

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

uint16_t UwbTelemetry_Crc16Ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t index = 0U; index < length; index++) {
        crc ^= (uint16_t)data[index] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

size_t UwbTelemetry_EncodeFrame(const UwbTelemetryFrame *frame,
                                uint8_t *buffer,
                                size_t capacity)
{
    if (frame == NULL || buffer == NULL
            || frame->payload_length > UWB_TELEM_MAX_PAYLOAD
            || (frame->payload_length > 0U && frame->payload == NULL)) {
        return 0U;
    }

    const size_t frame_length = UWB_TELEM_HEADER_SIZE
                              + (size_t)frame->payload_length + 2U;
    if (capacity < frame_length) {
        return 0U;
    }

    buffer[0] = UWB_TELEM_SOF0;
    buffer[1] = UWB_TELEM_SOF1;
    buffer[2] = frame->version;
    buffer[3] = frame->type;
    write_u16(&buffer[4], frame->payload_length);
    write_u32(&buffer[6], frame->sequence);
    write_u32(&buffer[10], frame->time_ms);
    if (frame->payload_length > 0U) {
        memcpy(&buffer[UWB_TELEM_HEADER_SIZE],
               frame->payload,
               frame->payload_length);
    }

    const uint16_t crc = UwbTelemetry_Crc16Ccitt(
        &buffer[2],
        (size_t)(UWB_TELEM_HEADER_SIZE - 2U) + frame->payload_length);
    write_u16(&buffer[UWB_TELEM_HEADER_SIZE + frame->payload_length], crc);
    return frame_length;
}

static void parser_reset(UwbTelemetryParser *parser)
{
    parser->used = 0U;
    parser->expected = 0U;
}

static int parser_accept_frame(UwbTelemetryParser *parser)
{
    const uint16_t payload_length = read_u16(&parser->buffer[4]);
    const uint16_t crc_offset =
        (uint16_t)(UWB_TELEM_HEADER_SIZE + payload_length);
    const uint16_t received_crc = read_u16(&parser->buffer[crc_offset]);
    const uint16_t calculated_crc = UwbTelemetry_Crc16Ccitt(
        &parser->buffer[2],
        (size_t)(UWB_TELEM_HEADER_SIZE - 2U + payload_length));

    if (received_crc != calculated_crc) {
        parser->crc_errors++;
        return 0;
    }
    if (parser->buffer[2] != UWB_TELEM_VERSION) {
        parser->version_errors++;
        return 0;
    }

    const UwbTelemetryFrame frame = {
        .version = parser->buffer[2],
        .type = parser->buffer[3],
        .payload_length = payload_length,
        .sequence = read_u32(&parser->buffer[6]),
        .time_ms = read_u32(&parser->buffer[10]),
        .payload = &parser->buffer[UWB_TELEM_HEADER_SIZE],
    };

    parser->valid_frames++;
    if (parser->handler != NULL) {
        parser->handler(&frame, parser->handler_context);
    }
    return 1;
}

static void parser_feed_byte(UwbTelemetryParser *parser, uint8_t byte)
{
    if (parser->used >= sizeof(parser->buffer)) {
        parser->length_errors++;
        parser_reset(parser);
    }
    parser->buffer[parser->used++] = byte;
    while (parser->used > 0U) {
        size_t discard = 1U;
        parser->expected = 0U;
        if (parser->buffer[0] == UWB_TELEM_SOF0
                && (parser->used == 1U || parser->buffer[1] == UWB_TELEM_SOF1)) {
            if (parser->used < 6U)
                return;
            const uint16_t payload_length = read_u16(&parser->buffer[4]);
            if (payload_length > UWB_TELEM_MAX_PAYLOAD) {
                parser->length_errors++;
            } else {
                parser->expected = (uint16_t)(UWB_TELEM_HEADER_SIZE + payload_length + 2U);
                if (parser->used < parser->expected)
                    return;
                if (parser_accept_frame(parser))
                    discard = parser->expected;
            }
        }
        /* On failure retain the suffix: a damaged length may have swallowed
         * the start (or entirety) of the following valid frame. No recursion. */
        parser->used = (uint16_t)(parser->used - discard);
        memmove(parser->buffer, &parser->buffer[discard], parser->used);
    }
    parser->expected = 0U;
}

void UwbTelemetryParser_Init(UwbTelemetryParser *parser,
                             UwbTelemetryFrameHandler handler,
                             void *handler_context)
{
    if (parser == NULL) {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->handler = handler;
    parser->handler_context = handler_context;
}

void UwbTelemetryParser_Feed(UwbTelemetryParser *parser,
                             const uint8_t *data,
                             size_t length)
{
    if (parser == NULL || data == NULL) {
        return;
    }

    for (size_t index = 0U; index < length; index++) {
        parser_feed_byte(parser, data[index]);
    }
}
