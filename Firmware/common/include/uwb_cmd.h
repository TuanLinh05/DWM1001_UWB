/**
 ******************************************************************************
 * @file    uwb_cmd.h
 * @brief   Host → TAG command channel over the telemetry UART.
 *
 * A command is a normal telemetry frame of TYPE 0x20 (same SOF/VER/LEN/CRC):
 *   SEQ     = host command sequence number (echoed in the ACK)
 *   TIME    = host time in ms (informational)
 *   PAYLOAD = cmd_id[1] + arguments (little-endian)
 * The TAG answers every well-formed command with a CMD_ACK frame (TYPE 0x13):
 *   SEQ     = the command SEQ
 *   PAYLOAD = cmd_id[1] result[1] data[...]
 * Frames with a bad CRC are counted and never acknowledged; the host retries.
 * Commands are idempotent, so a retry after a lost ACK is harmless.
 *
 * Safety rules:
 *   - LOCK (set by the flight computer while armed) rejects every command that
 *     changes ranging, calibration or radio configuration.
 *   - Radio reconfiguration and flash writes are accepted only while ranging
 *     is paused (PAUSE, then wait until DIAG_SYSTEM reports paused=1).
 *
 * | id   | name            | arguments                          | ACK data            |
 * |------|-----------------|------------------------------------|---------------------|
 * | 0x01 | PING            | -                                  | u32 uptime_ms       |
 * | 0x02 | GET_DEVICE_INFO | -                                  | - (+ DEVICE_INFO)   |
 * | 0x03 | PAUSE           | -                                  | u8 paused_now       |
 * | 0x04 | RESUME          | -                                  | -                   |
 * | 0x05 | SET_ANCHOR_MASK | u8 mask (bit n = anchor n+1)       | -                   |
 * | 0x06 | SET_DS_CAL      | u16 anchor_id, i32 bias_um, u8 cal | -                   |
 * | 0x07 | GET_DS_CAL      | -                                  | 8 × (u16 id, i32 bias_um, u8 cal) |
 * | 0x08 | SET_ANT_DELAY   | u16 tx_dtu, u16 rx_dtu   (paused)  | -                   |
 * | 0x09 | SET_TX_POWER    | u8 mode, u32 custom      (paused)  | u32 TX_POWER value  |
 * | 0x0A | SAVE_SETTINGS   | -                        (paused)  | -                   |
 * | 0x0B | FACTORY_RESET   | -                        (paused)  | -                   |
 * | 0x0C | REBOOT          | -                                  | - (reboot ~50 ms)   |
 * | 0x0D | TIME_SYNC       | u64 t1_host_us                     | u64 t1, u64 t2, u64 t3 (TAG µs) |
 * | 0x0E | SET_LOCK        | u8 lock                            | -                   |
 * | 0x0F | SET_TELEMETRY   | u8 features (TELEM_FEATURE_*)      | u8 active features  |
 *
 * bias_um follows the firmware convention: corrected = measured − bias.
 ******************************************************************************
 */

#ifndef UWB_CMD_H
#define UWB_CMD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_CMD_PING             0x01U
#define UWB_CMD_GET_DEVICE_INFO  0x02U
#define UWB_CMD_PAUSE            0x03U
#define UWB_CMD_RESUME           0x04U
#define UWB_CMD_SET_ANCHOR_MASK  0x05U
#define UWB_CMD_SET_DS_CAL       0x06U
#define UWB_CMD_GET_DS_CAL       0x07U
#define UWB_CMD_SET_ANT_DELAY    0x08U
#define UWB_CMD_SET_TX_POWER     0x09U
#define UWB_CMD_SAVE_SETTINGS    0x0AU
#define UWB_CMD_FACTORY_RESET    0x0BU
#define UWB_CMD_REBOOT           0x0CU
#define UWB_CMD_TIME_SYNC        0x0DU
#define UWB_CMD_SET_LOCK         0x0EU
#define UWB_CMD_SET_TELEMETRY    0x0FU

#define UWB_CMD_OK               0x00U
#define UWB_CMD_ERR_UNKNOWN      0x01U
#define UWB_CMD_ERR_LENGTH       0x02U
#define UWB_CMD_ERR_ARGUMENT     0x03U
#define UWB_CMD_ERR_LOCKED       0x04U
#define UWB_CMD_ERR_NOT_PAUSED   0x05U
#define UWB_CMD_ERR_STORAGE      0x06U
#define UWB_CMD_ERR_RADIO        0x07U
#define UWB_CMD_ERR_UNSUPPORTED  0x08U

/** Largest command payload accepted (the telemetry limit is 256). */
#define UWB_CMD_MAX_PAYLOAD      64U

typedef struct {
    uint32_t frames_ok;        /* well-formed command frames */
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t executed;         /* result OK */
    uint32_t rejected;         /* any other result */
    uint8_t  locked;
    uint8_t  last_cmd_id;
    uint8_t  last_result;
    uint8_t  reboot_pending;
} UwbCmdStats_t;

extern UwbCmdStats_t uwb_cmd_stats;

/**
 * Streaming command-frame parser (pure C, host-tested). Feed received bytes;
 * `handler` is called for each frame with a valid CRC and TYPE 0x20.
 */
typedef void (*UwbCmdFrameHandler)(uint32_t seq, const uint8_t *payload,
                                   uint16_t length, void *context);

typedef struct {
    uint8_t  buffer[14U + UWB_CMD_MAX_PAYLOAD + 2U];
    uint16_t used;
    UwbCmdFrameHandler handler;
    void    *context;
} UwbCmdParser_t;

void uwb_cmd_parser_init(UwbCmdParser_t *parser, UwbCmdFrameHandler handler,
                         void *context);
void uwb_cmd_parser_feed(UwbCmdParser_t *parser, const uint8_t *data,
                         uint16_t length);

/** TAG integration: init once, then call uwb_cmd_poll() every loop. */
void uwb_cmd_init(void);
void uwb_cmd_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* UWB_CMD_H */
