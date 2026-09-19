#include "telemetry_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t calls;
    uint8_t type;
    uint32_t sequence;
    uint8_t payload[UWB_TELEM_MAX_PAYLOAD];
    uint16_t payload_length;
} Capture;

static size_t make_frame(uint8_t *buffer, size_t capacity, uint8_t type,
                         uint32_t sequence, const uint8_t *payload,
                         uint16_t payload_length)
{
    const UwbTelemetryFrame frame = {
        .version = UWB_TELEM_VERSION,
        .type = type,
        .payload_length = payload_length,
        .sequence = sequence,
        .time_ms = 1234U,
        .payload = payload,
    };
    return UwbTelemetry_EncodeFrame(&frame, buffer, capacity);
}

static void capture_frame(const UwbTelemetryFrame *frame, void *context)
{
    Capture *capture = context;
    capture->calls++;
    capture->type = frame->type;
    capture->sequence = frame->sequence;
    capture->payload_length = frame->payload_length;
    assert(frame->payload_length <= sizeof(capture->payload));
    memcpy(capture->payload, frame->payload, frame->payload_length);
}

static void test_encode_and_fragmented_parse(void)
{
    UwbTelemetryParser parser;
    Capture capture = {0};
    uint8_t frame[64];
    const uint8_t payload[] = {1U, 2U, 3U, 4U};
    const size_t frame_length = make_frame(
        frame, sizeof(frame), UWB_TELEM_TYPE_RANGE, 0x12345678U,
        payload, sizeof(payload));

    assert(frame_length == UWB_TELEM_HEADER_SIZE + sizeof(payload) + 2U);
    assert(frame[0] == UWB_TELEM_SOF0);
    assert(frame[1] == UWB_TELEM_SOF1);
    assert(make_frame(frame, frame_length - 1U, UWB_TELEM_TYPE_RANGE,
                      1U, payload, sizeof(payload)) == 0U);

    UwbTelemetryParser_Init(&parser, capture_frame, &capture);
    const uint8_t noise[] = {0U, 0x55U, 0xAAU, 0xAAU};
    UwbTelemetryParser_Feed(&parser, noise, sizeof(noise));
    UwbTelemetryParser_Feed(&parser, frame, 1U);
    UwbTelemetryParser_Feed(&parser, &frame[1], 5U);
    UwbTelemetryParser_Feed(&parser, &frame[6], frame_length - 6U);

    assert(capture.calls == 1U);
    assert(capture.type == UWB_TELEM_TYPE_RANGE);
    assert(capture.sequence == 0x12345678U);
    assert(capture.payload_length == sizeof(payload));
    assert(memcmp(capture.payload, payload, sizeof(payload)) == 0);

    frame[frame_length - 1U] ^= 0x01U;
    UwbTelemetryParser_Feed(&parser, frame, frame_length);
    assert(capture.calls == 1U);
    assert(parser.crc_errors == 1U);

    const uint8_t bad_length[] = {
        UWB_TELEM_SOF0, UWB_TELEM_SOF1, UWB_TELEM_VERSION,
        UWB_TELEM_TYPE_RANGE, 0x01U, 0x01U,
    };
    UwbTelemetryParser_Feed(&parser, bad_length, sizeof(bad_length));
    assert(parser.length_errors == 1U);
}

static void test_full_eight_anchor_frame(void)
{
    UwbTelemetryParser parser;
    Capture capture = {0};
    uint8_t payload[129] = {8U};
    uint8_t frame[UWB_TELEM_FRAME_SIZE_MAX];

    for (uint8_t anchor = 0U; anchor < 8U; anchor++) {
        uint8_t *record = &payload[1U + ((size_t)anchor * 16U)];
        record[0] = (uint8_t)(anchor + 1U);
        record[2] = 0U;
        record[3] = anchor == 0U ? 0x30U : 0x20U;
    }

    const size_t frame_length = make_frame(
        frame, sizeof(frame), UWB_TELEM_TYPE_RANGE, UINT32_MAX,
        payload, sizeof(payload));
    assert(frame_length == 145U);

    UwbTelemetryParser_Init(&parser, capture_frame, &capture);
    UwbTelemetryParser_Feed(&parser, frame, frame_length);
    assert(capture.calls == 1U);
    assert(capture.sequence == UINT32_MAX);
    assert(capture.payload_length == sizeof(payload));
    assert(memcmp(capture.payload, payload, sizeof(payload)) == 0);
}

int main(void)
{
    /* A corrupt outer length consumes a complete valid inner frame. Recovery
     * must rescan that suffix immediately, without waiting for another frame. */
    {
        UwbTelemetryParser parser;
        Capture capture = {0};
        uint8_t data[80] = {0xAA, 0x55, 1, 1, 24, 0};
        const uint8_t payload[] = {0};
        size_t good = make_frame(&data[10], sizeof(data) - 10,
                                 UWB_TELEM_TYPE_RANGE, 99, payload, sizeof(payload));
        assert(good == 17U);
        UwbTelemetryParser_Init(&parser, capture_frame, &capture);
        UwbTelemetryParser_Feed(&parser, data, 40);
        assert(capture.calls == 1U && capture.sequence == 99U);
        assert(parser.crc_errors == 1U);
    }
    static const uint8_t crc_vector[] = "123456789";
    assert(UwbTelemetry_Crc16Ccitt(crc_vector, 9U) == 0x29B1U);
    test_encode_and_fragmented_parse();
    test_full_eight_anchor_frame();
    puts("telemetry protocol tests passed");
    return 0;
}
