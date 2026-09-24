/**
 ******************************************************************************
 * @file    uwb_cmd.c
 * @brief   Host → TAG command channel (see uwb_cmd.h for the protocol).
 *
 * The parser is platform independent (host-tested); the executor at the end
 * of the file binds commands to the TAG ranging, radio and settings modules.
 ******************************************************************************
 */

#include "uwb_cmd.h"
#include "telemetry_frame.h"

#include <stddef.h>
#include <string.h>

UwbCmdStats_t uwb_cmd_stats;

/* ========================================================================== */
/*                     FRAME PARSER                                            */
/* ========================================================================== */

void uwb_cmd_parser_init(UwbCmdParser_t *parser, UwbCmdFrameHandler handler,
                         void *context)
{
    if (parser == NULL)
        return;
    memset(parser, 0, sizeof(*parser));
    parser->handler = handler;
    parser->context = context;
}

static void parser_drop(UwbCmdParser_t *parser, uint16_t count)
{
    if (count >= parser->used)
    {
        parser->used = 0U;
        return;
    }
    parser->used = (uint16_t)(parser->used - count);
    memmove(parser->buffer, &parser->buffer[count], parser->used);
}

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Try to extract frames from the buffer; stop when more bytes are needed. */
static void parser_scan(UwbCmdParser_t *parser)
{
    while (parser->used > 0U)
    {
        if (parser->buffer[0] != TELEM_SOF0)
        {
            parser_drop(parser, 1U);
            continue;
        }
        if (parser->used < 2U)
            return;
        if (parser->buffer[1] != TELEM_SOF1)
        {
            parser_drop(parser, 1U);
            continue;
        }
        if (parser->used < 6U)
            return;

        const uint16_t payload_len = read_u16(&parser->buffer[4]);
        if (payload_len > UWB_CMD_MAX_PAYLOAD)
        {
            uwb_cmd_stats.length_errors++;
            parser_drop(parser, 1U);    /* resync on the next SOF */
            continue;
        }

        const uint16_t total = (uint16_t)(TELEM_HDR_LEN + payload_len + TELEM_CRC_LEN);
        if (parser->used < total)
            return;

        const uint16_t received = read_u16(&parser->buffer[TELEM_HDR_LEN + payload_len]);
        const uint16_t computed = telem_crc16_ccitt(&parser->buffer[2],
                                                    (uint16_t)(TELEM_HDR_LEN - 2U + payload_len));
        if (received != computed || parser->buffer[2] != TELEM_VER)
        {
            uwb_cmd_stats.crc_errors++;
            parser_drop(parser, 1U);
            continue;
        }

        if (parser->buffer[3] == TELEM_TYPE_CMD)
        {
            uwb_cmd_stats.frames_ok++;
            if (parser->handler != NULL)
            {
                parser->handler(read_u32(&parser->buffer[6]),
                                &parser->buffer[TELEM_HDR_LEN], payload_len,
                                parser->context);
            }
        }
        parser_drop(parser, total);
    }
}

void uwb_cmd_parser_feed(UwbCmdParser_t *parser, const uint8_t *data,
                         uint16_t length)
{
    if (parser == NULL || data == NULL)
        return;

    for (uint16_t i = 0U; i < length; i++)
    {
        if (parser->used >= sizeof(parser->buffer))
        {
            /* Cannot happen with the length check; resync defensively. */
            uwb_cmd_stats.length_errors++;
            parser->used = 0U;
        }
        parser->buffer[parser->used++] = data[i];
        parser_scan(parser);
    }
}

/* ========================================================================== */
/*                     TAG EXECUTOR                                            */
/* ========================================================================== */

#if defined(UWB_ROLE_TAG)

#include "dw1000_hw.h"
#include "tag_ranging.h"
#include "telemetry.h"
#include "uart_tx.h"
#include "uwb_health.h"
#include "uwb_platform.h"
#include "uwb_settings.h"

#define REBOOT_DRAIN_MAX_MS 100U

static UwbCmdParser_t s_parser;
static uint32_t s_reboot_requested_ms;

static void reply(uint32_t seq, uint8_t id, uint8_t result,
                  const uint8_t *data, uint16_t length)
{
    uwb_cmd_stats.last_cmd_id = id;
    uwb_cmd_stats.last_result = result;
    if (result == UWB_CMD_OK)
        uwb_cmd_stats.executed++;
    else
        uwb_cmd_stats.rejected++;
    Telem_SendCmdAck(seq, id, result, data, length);
}

static uint8_t allowed_when_locked(uint8_t id)
{
    switch (id)
    {
        case UWB_CMD_PING:
        case UWB_CMD_GET_DEVICE_INFO:
        case UWB_CMD_GET_DS_CAL:
        case UWB_CMD_TIME_SYNC:
        case UWB_CMD_SET_LOCK:
        case UWB_CMD_SET_TELEMETRY:
            return 1U;
        default:
            return 0U;
    }
}

static uint8_t needs_pause(uint8_t id)
{
    return (id == UWB_CMD_SET_ANT_DELAY || id == UWB_CMD_SET_TX_POWER
            || id == UWB_CMD_SAVE_SETTINGS || id == UWB_CMD_FACTORY_RESET)
        ? 1U : 0U;
}

/** Re-apply the whole radio configuration (ranging paused). */
static uint8_t reconfigure_radio(void)
{
    if (DW1000_Configure() != 0)
    {
        uwb_health_note_config_check(0xFFFFFFFFUL);
        return UWB_CMD_ERR_RADIO;
    }
    DW1000_SetAddress(DW_PAN_ID, TAG_ADDR);
    DW1000_ClearAllStatus();
    const uint32_t mismatch = DW1000_VerifyConfig();
    uwb_health_note_config_check(mismatch);
    return mismatch == 0U ? UWB_CMD_OK : UWB_CMD_ERR_RADIO;
}

static uint8_t radio_config_equal(const DW1000_RadioConfig_t *a,
                                  const DW1000_RadioConfig_t *b)
{
    return (a->tx_ant_dly == b->tx_ant_dly
            && a->rx_ant_dly == b->rx_ant_dly
            && a->tx_power_mode == b->tx_power_mode
            && a->reference_tuning == b->reference_tuning
            && a->tx_power_custom == b->tx_power_custom) ? 1U : 0U;
}

/** Apply a candidate atomically from the command channel's perspective.
 *  A verified rollback keeps the old calibration valid. If rollback cannot
 *  be verified, fail closed because the physical radio state is uncertain. */
static uint8_t apply_radio_candidate(const DW1000_RadioConfig_t *candidate)
{
    const DW1000_RadioConfig_t previous = dw1000_radio_config;

    if (radio_config_equal(&previous, candidate))
        return UWB_CMD_OK;

    dw1000_radio_config = *candidate;
    if (reconfigure_radio() == UWB_CMD_OK)
    {
        Tag_InvalidateDsCalibration();
        return UWB_CMD_OK;
    }

    dw1000_radio_config = previous;
    if (reconfigure_radio() != UWB_CMD_OK)
        Tag_InvalidateDsCalibration();
    return UWB_CMD_ERR_RADIO;
}

static void execute(uint32_t seq, const uint8_t *payload, uint16_t length,
                    void *context)
{
    uint8_t data[64];
    (void)context;

    if (length < 1U)
    {
        reply(seq, 0U, UWB_CMD_ERR_LENGTH, NULL, 0U);
        return;
    }

    const uint8_t id = payload[0];
    const uint8_t *arg = &payload[1];
    const uint16_t arg_len = (uint16_t)(length - 1U);

    if (uwb_cmd_stats.locked && !allowed_when_locked(id))
    {
        reply(seq, id, UWB_CMD_ERR_LOCKED, NULL, 0U);
        return;
    }
    if (needs_pause(id) && !Tag_IsPaused())
    {
        reply(seq, id, UWB_CMD_ERR_NOT_PAUSED, NULL, 0U);
        return;
    }

    switch (id)
    {
        case UWB_CMD_PING:
        {
            (void)telem_put_u32(data, uwb_platform_time_ms());
            reply(seq, id, UWB_CMD_OK, data, 4U);
            break;
        }

        case UWB_CMD_GET_DEVICE_INFO:
            Telem_SendDeviceInfo();
            reply(seq, id, UWB_CMD_OK, NULL, 0U);
            break;

        case UWB_CMD_PAUSE:
            Tag_RequestPause(1U);
            data[0] = Tag_IsPaused();
            reply(seq, id, UWB_CMD_OK, data, 1U);
            break;

        case UWB_CMD_RESUME:
            Tag_RequestPause(0U);
            reply(seq, id, UWB_CMD_OK, NULL, 0U);
            break;

        case UWB_CMD_SET_ANCHOR_MASK:
            if (arg_len != 1U)
            {
                reply(seq, id, UWB_CMD_ERR_LENGTH, NULL, 0U);
                break;
            }
            Tag_SetActiveAnchorMask(arg[0]);
            reply(seq, id, UWB_CMD_OK, NULL, 0U);
            break;

        case UWB_CMD_SET_DS_CAL:
        {
            if (arg_len != 7U)
            {
                reply(seq, id, UWB_CMD_ERR_LENGTH, NULL, 0U);
                break;
            }
            const uint16_t anchor_id = read_u16(&arg[0]);
            const int32_t bias_um = (int32_t)read_u32(&arg[2]);
            const int rc = Tag_SetDsCalibration(anchor_id, bias_um, arg[6]);
            reply(seq, id, rc == 0 ? UWB_CMD_OK : UWB_CMD_ERR_ARGUMENT, NULL, 0U);
            break;
        }

        case UWB_CMD_GET_DS_CAL:
        {
            uint16_t off = 0U;
            for (uint16_t anchor_id = 1U; anchor_id <= TAG_NUM_ANCHORS; anchor_id++)
            {
                int32_t bias_um = 0;
                uint8_t calibrated = 0U;
                (void)Tag_GetDsCalibration(anchor_id, &bias_um, &calibrated);
                off += telem_put_u16(&data[off], anchor_id);
                off += telem_put_u32(&data[off], (uint32_t)bias_um);
                data[off++] = calibrated;
            }
            reply(seq, id, UWB_CMD_OK, data, off);
            break;
        }

        case UWB_CMD_SET_ANT_DELAY:
        {
            if (arg_len != 4U)
            {
                reply(seq, id, UWB_CMD_ERR_LENGTH, NULL, 0U);
                break;
            }
            const uint16_t tx = read_u16(&arg[0]);
            const uint16_t rx = read_u16(&arg[2]);
            if (tx == 0U || rx == 0U)
            {
                reply(seq, id, UWB_CMD_ERR_ARGUMENT, NULL, 0U);
                break;
            }
            DW1000_RadioConfig_t candidate = dw1000_radio_config;
            candidate.tx_ant_dly = tx;
            candidate.rx_ant_dly = rx;
            reply(seq, id, apply_radio_candidate(&candidate), NULL, 0U);
            break;
        }

        case UWB_CMD_SET_TX_POWER:
        {
            if (arg_len != 5U)
            {
                reply(seq, id, UWB_CMD_ERR_LENGTH, NULL, 0U);
                break;
            }
            const uint8_t mode = arg[0];
            const uint32_t custom = read_u32(&arg[1]);
            if (mode > UWB_TX_POWER_CUSTOM || (mode == UWB_TX_POWER_CUSTOM && custom == 0U))
            {
                reply(seq, id, UWB_CMD_ERR_ARGUMENT, NULL, 0U);
                break;
            }
            DW1000_RadioConfig_t candidate = dw1000_radio_config;
            candidate.tx_power_mode = mode;
            if (mode == UWB_TX_POWER_CUSTOM)
                candidate.tx_power_custom = custom;
            const uint8_t result = apply_radio_candidate(&candidate);
            (void)telem_put_u32(data, DW1000_TxPowerRegisterValue());
            reply(seq, id, result, data, 4U);
            break;
        }

        case UWB_CMD_SAVE_SETTINGS:
        {
            uwb_health_feed();
            const int rc = uwb_settings_save();
            uwb_health_feed();
            reply(seq, id, rc == 0 ? UWB_CMD_OK : UWB_CMD_ERR_STORAGE, NULL, 0U);
            break;
        }

        case UWB_CMD_FACTORY_RESET:
        {
            uwb_health_feed();
            const int rc = uwb_settings_factory_reset();
            uwb_health_feed();
            uint8_t result = rc == 0 ? UWB_CMD_OK : UWB_CMD_ERR_STORAGE;
            if (result == UWB_CMD_OK)
                result = reconfigure_radio();
            reply(seq, id, result, NULL, 0U);
            break;
        }

        case UWB_CMD_REBOOT:
            reply(seq, id, UWB_CMD_OK, NULL, 0U);
            uwb_cmd_stats.reboot_pending = 1U;
            s_reboot_requested_ms = uwb_platform_time_ms();
            break;

        case UWB_CMD_TIME_SYNC:
        {
            if (arg_len != 8U)
            {
                reply(seq, id, UWB_CMD_ERR_LENGTH, NULL, 0U);
                break;
            }
            const uint64_t t1 = (uint64_t)read_u32(&arg[0])
                              | ((uint64_t)read_u32(&arg[4]) << 32);
            const uint64_t t2 = uwb_platform_time_us64();
            /* t3 ≈ when the ACK leaves the UART: account for queued bytes. */
            const uint32_t baud = Telem_UartBaud();
            const uint64_t queued_us = baud != 0U
                ? ((uint64_t)UART_TX_Pending() * 10000000ULL) / baud : 0U;
            const uint64_t t3 = uwb_platform_time_us64() + queued_us;
            (void)telem_put_u64(&data[0], t1);
            (void)telem_put_u64(&data[8], t2);
            (void)telem_put_u64(&data[16], t3);
            reply(seq, id, UWB_CMD_OK, data, 24U);
            break;
        }

        case UWB_CMD_SET_LOCK:
            if (arg_len != 1U)
            {
                reply(seq, id, UWB_CMD_ERR_LENGTH, NULL, 0U);
                break;
            }
            uwb_cmd_stats.locked = arg[0] ? 1U : 0U;
            reply(seq, id, UWB_CMD_OK, NULL, 0U);
            break;

        case UWB_CMD_SET_TELEMETRY:
        {
            if (arg_len != 1U)
            {
                reply(seq, id, UWB_CMD_ERR_LENGTH, NULL, 0U);
                break;
            }
            if (Telem_SetFeatures(arg[0]) != 0)
            {
                data[0] = Telem_GetFeatures();
                reply(seq, id, UWB_CMD_ERR_UNSUPPORTED, data, 1U);
                break;
            }
            Tag_EnableMeasurementQueue(
                (Telem_GetFeatures() & TELEM_FEATURE_RANGE_MEAS) != 0U);
            data[0] = Telem_GetFeatures();
            reply(seq, id, UWB_CMD_OK, data, 1U);
            break;
        }

        default:
            reply(seq, id, UWB_CMD_ERR_UNKNOWN, NULL, 0U);
            break;
    }
}

void uwb_cmd_init(void)
{
    uwb_cmd_parser_init(&s_parser, execute, NULL);
}

void uwb_cmd_poll(void)
{
    uint8_t chunk[32];
    uint16_t length;

    while ((length = UART_RX_Read(chunk, sizeof(chunk))) > 0U)
        uwb_cmd_parser_feed(&s_parser, chunk, length);

    if (uwb_cmd_stats.reboot_pending)
    {
        /* Let the ACK leave the UART before rebooting. */
        const uint32_t waited = uwb_platform_time_ms() - s_reboot_requested_ms;
        if (UART_TX_Pending() == 0U || waited > REBOOT_DRAIN_MAX_MS)
            uwb_platform_reboot();
    }
}

#endif /* UWB_ROLE_TAG */
