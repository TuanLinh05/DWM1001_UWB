#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_TELEM_SOF0             0xAAU
#define UWB_TELEM_SOF1             0x55U
#define UWB_TELEM_VERSION          1U
#define UWB_TELEM_HEADER_SIZE      14U
#define UWB_TELEM_MAX_PAYLOAD      256U
#define UWB_TELEM_FRAME_SIZE_MAX   \
    (UWB_TELEM_HEADER_SIZE + UWB_TELEM_MAX_PAYLOAD + 2U)

#define UWB_TELEM_TYPE_INFO        0x00U
#define UWB_TELEM_TYPE_RANGE       0x01U
#define UWB_TELEM_TYPE_STATS       0x02U
/* Telemetry v2 types are forwarded unchanged; the gateway only adds its own
 * health frame. See Firmware/common/include/telemetry_frame.h. */
#define UWB_TELEM_TYPE_GATEWAY_HEALTH 0x17U

typedef struct {
    uint8_t version;
    uint8_t type;
    uint16_t payload_length;
    uint32_t sequence;
    uint32_t time_ms;
    const uint8_t *payload;
} UwbTelemetryFrame;

typedef void (*UwbTelemetryFrameHandler)(const UwbTelemetryFrame *frame,
                                         void *context);

typedef struct {
    uint8_t buffer[UWB_TELEM_FRAME_SIZE_MAX];
    uint16_t used;
    uint16_t expected;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t version_errors;
    UwbTelemetryFrameHandler handler;
    void *handler_context;
} UwbTelemetryParser;

void UwbTelemetryParser_Init(UwbTelemetryParser *parser,
                             UwbTelemetryFrameHandler handler,
                             void *handler_context);
void UwbTelemetryParser_Feed(UwbTelemetryParser *parser,
                             const uint8_t *data,
                             size_t length);
uint16_t UwbTelemetry_Crc16Ccitt(const uint8_t *data, size_t length);
size_t UwbTelemetry_EncodeFrame(const UwbTelemetryFrame *frame,
                                uint8_t *buffer,
                                size_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_PROTOCOL_H */
